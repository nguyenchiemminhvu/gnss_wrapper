#include "gnss_receiver.h"
#include "global_constants.h"
#include "libevent.h"
#include "serial_helper/serial_wrapper.h"
#include "serial_helper/serial_exception.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include "logging.h"

namespace gnss
{

// ─── Constructor / destructor ──────────────────────────────────────────────────

gnss_receiver::gnss_receiver(std::shared_ptr<serial::i_serial_port> port)
    : port_(std::move(port))
{
}

gnss_receiver::~gnss_receiver()
{
    stop();
    terminate();
}

// ─── Lifecycle ─────────────────────────────────────────────────────────────────

bool gnss_receiver::init(const serial::serial_config& config)
{
    LOG_INFO("[gnss_receiver] initializing on ", config.device_path.c_str());
    std::lock_guard<std::mutex> lk(state_mutex_);
    serial::serial_wrapper local_wrapper(port_);
    if (state_ != state::idle)
    {
        // Already initialized — reconfigure the port.
        try
        {
            if (!local_wrapper.reconfigure(config))
            {
                LOG_ERROR("[gnss_receiver] failed to reconfigure port with new config");
                return false;
            }
            config_ = config;
            LOG_INFO("[gnss_receiver] successfully reconfigured port with new config");
            return true;
        }
        catch (const serial::serial_error&)
        {
            LOG_ERROR("[gnss_receiver] serial error occurred while reconfiguring port");
            return false;
        }
    }

    try
    {
        if (!local_wrapper.initialize(config))
        {
            LOG_ERROR("[gnss_receiver] failed to initialize port with config");
            return false;
        }
    }
    catch (const serial::serial_error&)
    {
        LOG_ERROR("[gnss_receiver] serial error occurred while initializing port");
        return false;
    }

    config_ = config;
    state_ = state::initialized;
    return true;
}

bool gnss_receiver::start()
{
    LOG_INFO("[gnss_receiver] starting");
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ != state::initialized)
    {
        return false;
    }

    // Build a fresh decoder registry on every start() so stop()+start() works.
    ubx::parser::ubx_decoder_registry registry;

    if (ubx_db_ != nullptr)
    {
        LOG_INFO("[gnss_receiver] wiring receiver to shared UBX database");
        ubx_db_->register_with_parser(registry);
    }

    if (extra_setup_)
    {
        LOG_INFO("[gnss_receiver] applying extra parser setup");
        extra_setup_(registry);
    }

    ubx_parser_ = std::make_unique<ubx::parser::ubx_parser>(std::move(registry));

    ubx_parser_->set_error_callback([](const ubx::parser::parse_error_info& err) {
        // Errors are silently dropped; a production build could log them here.
        (void)err;
    });

    if (raw_msg_cb_)
    {
        LOG_INFO("[gnss_receiver] setting raw message callback");
        ubx_parser_->set_raw_message_callback(raw_msg_cb_);
    }

#ifdef NMEA_PARSER_ENABLED
    if (nmea_db_ != nullptr)
    {
        LOG_INFO("[gnss_receiver] wiring receiver to shared NMEA database");
        // Build a fresh NMEA registry on every start() so stop()+start() works.
        nmea::parser::nmea_sentence_registry nmea_registry;
        nmea_db_->register_with_parser(nmea_registry);
        nmea_parser_ = std::make_unique<nmea::parser::nmea_parser>(std::move(nmea_registry));
    }
#endif

    running_.store(true);
    LOG_INFO("[gnss_receiver] worker thread starting");
    worker_thread_ = std::thread(&gnss_receiver::worker_loop, this);
    state_ = state::running;
    return true;
}

void gnss_receiver::stop()
{
    LOG_INFO("[gnss_receiver] stopping");
    {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (state_ != state::running)
        {
            return;
        }
        running_.store(false);
    }

    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }

    ubx_parser_.reset();

#ifdef NMEA_PARSER_ENABLED
    nmea_parser_.reset();
#endif

    std::lock_guard<std::mutex> lk(state_mutex_);
    state_ = state::initialized;
}

void gnss_receiver::terminate()
{
    stop(); // Ensure thread is stopped first.

    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ == state::initialized)
    {
        serial::serial_wrapper local_wrapper(port_);
        local_wrapper.shutdown();
        state_ = state::idle;
    }
}

bool gnss_receiver::is_running() const
{
    return running_.load();
}

// ─── Parser wiring ─────────────────────────────────────────────────────────────

void gnss_receiver::setup_ubx(std::shared_ptr<gnss::ubx_database_wrapper> ubx_db)
{
    ubx_db_ = ubx_db;
}

void gnss_receiver::set_extra_parser_setup(
    std::function<void(ubx::parser::ubx_decoder_registry&)> cb)
{
    extra_setup_ = std::move(cb);
}

void gnss_receiver::set_raw_message_callback(
    ubx::parser::raw_message_callback_t cb)
{
    raw_msg_cb_ = std::move(cb);
}

#ifdef NMEA_PARSER_ENABLED
void gnss_receiver::setup_nmea(std::shared_ptr<gnss::nmea_database_wrapper> nmea_db)
{
    nmea_db_ = nmea_db;
}
#endif

// ─── Worker loop ───────────────────────────────────────────────────────────────
//
// Uses fd_event::fd_event_manager (backed by poll()) to wait up to
// UART_FD_WAIT_TIMEOUT_MS (1 second) per cycle for UART data.
//
// Per-cycle behaviour:
//   1. Obtain current fd from the local_wrapper; on invalid fd, reconfigure.
//   2. Register (or re-register after fd change) the fd with the event manager.
//   3. Call wait_and_process() with a 1-second timeout.
//      • Timeout  → log "data not available in 1 second", reconfigure, continue.
//      • Poll err → log, reconfigure, continue.
//      • I/O err  → log UART error, reconfigure, continue.
//      • POLLIN   → query available() bytes, read exactly that size (capped),
//                   feed the UBX parser.
//   4. On any read() failure → log, reconfigure, continue.
//
// The loop never calls break on transient errors so the location service
// can always issue a restart command to resume data updates.

void gnss_receiver::worker_loop()
{
    LOG_INFO("[gnss_receiver] worker loop starting");
    serial::serial_wrapper local_wrapper(port_);

    fd_event::fd_event_manager fdm;

    std::vector<uint8_t> buf;
    buf.reserve(UART_READ_CHUNK_BYTES);

    bool data_ready = false;
    bool io_error   = false;
    int  current_fd = -1;

    // ── Helper: register a new fd with the event manager ──────────────────────
    auto register_fd = [&](int fd) -> bool
    {
        return fdm.add_fd(
            fd,
            static_cast<short>(fd_event::event_type::READ),
            [&data_ready, &io_error](int /*fd_arg*/, short revents, void* /*ud*/)
            {
                if ((revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
                {
                    io_error = true;
                }
                else if ((revents & POLLIN) != 0)
                {
                    data_ready = true;
                }
            },
            nullptr,
            "uart_gnss");
    };

    // ── Helper: remove stale fd and reconfigure the port ──────────────────────
    auto reconfigure_port = [&]()
    {
        if (current_fd >= 0)
        {
            fdm.remove_fd(current_fd);
            current_fd = -1;
        }
        try
        {
            if (local_wrapper.reconfigure(config_))
            {
                // fprintf(stderr, "[gnss_receiver] port reconfigured successfully\n");
                LOG_INFO("[gnss_receiver] port reconfigured successfully");
            }
            else
            {
                // fprintf(stderr, "[gnss_receiver] port reconfigure returned false\n");
                LOG_WARNING("[gnss_receiver] port reconfigure returned false");
            }
        }
        catch (const serial::serial_error& e)
        {
            // fprintf(stderr, "[gnss_receiver] port reconfigure exception: %s\n", e.what());
            LOG_ERROR("[gnss_receiver] port reconfigure exception: ", e.what());
        }
    };

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (running_.load())
    {
        // ── 1. Validate fd ────────────────────────────────────────────────────
        int fd = local_wrapper.get_fd();
        if (fd < 0)
        {
            // fprintf(stderr, "[gnss_receiver] invalid fd — reconfiguring port\n");
            LOG_ERROR("[gnss_receiver] invalid fd — reconfiguring port");
            reconfigure_port();
            continue;
        }

        // ── 2. (Re-)register fd when it changes after reconfigure ─────────────
        if (fd != current_fd)
        {
            if (current_fd >= 0)
            {
                fdm.remove_fd(current_fd);
            }
            if (!register_fd(fd))
            {
                // fprintf(stderr, "[gnss_receiver] failed to register fd %d — reconfiguring\n", fd);
                LOG_ERROR("[gnss_receiver] failed to register fd ", fd, " — reconfiguring");
                reconfigure_port();
                continue;
            }
            current_fd = fd;
        }

        // ── 3. Wait up to 1 second for UART data ──────────────────────────────
        data_ready = false;
        io_error   = false;

        int ret = fdm.wait_and_process(UART_FD_WAIT_TIMEOUT_MS);

        if (ret == 0)
        {
            // 1-second timeout: no data received
            // fprintf(stderr, "[gnss_receiver] data not available in 1 second\n");
            LOG_WARNING("[gnss_receiver] data not available in 1 second");
            reconfigure_port();
            continue;
        }

        if (ret < 0)
        {
            // fprintf(stderr, "[gnss_receiver] poll failure: %s\n", strerror(errno));
            LOG_ERROR("[gnss_receiver] poll failure: ", strerror(errno));
            reconfigure_port();
            continue;
        }

        if (io_error)
        {
            // fprintf(stderr, "[gnss_receiver] UART error or hangup on fd %d\n", fd);
            LOG_ERROR("[gnss_receiver] UART error or hangup on fd ", fd);
            reconfigure_port();
            continue;
        }

        if (!data_ready)
        {
            continue; // spurious wakeup with no POLLIN
        }

        // ── 4. Query available byte count before reading ───────────────────────
        int avail = local_wrapper.available();
        if (avail <= 0)
        {
            LOG_WARNING("[gnss_receiver] no data available in kernel buffer");
            continue; // nothing in kernel buffer yet
        }

        std::size_t read_size = static_cast<std::size_t>(avail);
        if (read_size > UART_READ_CHUNK_BYTES)
        {
            read_size = UART_READ_CHUNK_BYTES;
        }

        // ── 5. Read and feed the UBX parser ───────────────────────────────────
        buf.clear();
        ssize_t n = local_wrapper.read_bytes(buf, read_size);
        if (n > 0)
        {
            if (ubx_parser_)
            {
                ubx_parser_->feed(buf);
            }
#ifdef NMEA_PARSER_ENABLED
            // nmea_parser skips over UBX binary frames and re-synchronises on '$'.
            if (nmea_parser_)
            {
                nmea_parser_->feed(buf);
            }
#endif
        }
        else if (n < 0)
        {
            // fprintf(stderr, "[gnss_receiver] read error on fd %d — reconfiguring\n", fd);
            LOG_ERROR("[gnss_receiver] read error on fd ", fd, " — reconfiguring");
            reconfigure_port();
        }
        else
        {
            // n == 0: empty read (non-error), just continue
        }
    }

    // ── Cleanup: unregister fd before thread exit ──────────────────────────────
    if (current_fd >= 0)
    {
        LOG_INFO("[gnss_receiver] unregistering fd ", current_fd);
        fdm.remove_fd(current_fd);
    }

    running_.store(false);
}

} // namespace gnss
