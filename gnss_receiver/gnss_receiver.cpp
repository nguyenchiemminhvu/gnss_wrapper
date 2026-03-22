#include "gnss_receiver.h"
#include "global_constants.h"
#include "libevent.h"
#include "io_helper/io_channel_wrapper.h"
#include "io_helper/io_exception.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "logging.h"

namespace gnss
{

// ─── Constructor / destructor ──────────────────────────────────────────────────

gnss_receiver::gnss_receiver(std::shared_ptr<io::i_io_port> port)
    : port_(std::move(port))
{
}

gnss_receiver::~gnss_receiver()
{
    stop();
    terminate();
}

// ─── Lifecycle ─────────────────────────────────────────────────────────────────

bool gnss_receiver::init(std::shared_ptr<io::i_io_config> config)
{
    LOG_INFO("[gnss_receiver] initializing");
    if (config == nullptr)
    {
        LOG_ERROR("[gnss_receiver] null config provided");
        return false;
    }

    std::lock_guard<std::mutex> lk(state_mutex_);
    io::io_channel_wrapper local_wrapper(port_);
    if (state_ != state::idle)
    {
        // Already initialized — reconfigure the port.
        try
        {
            if (!local_wrapper.reconfigure(*config))
            {
                LOG_ERROR("[gnss_receiver] failed to reconfigure port with new config");
                return false;
            }
            config_ = config;
            LOG_INFO("[gnss_receiver] successfully reconfigured port with new config");
            return true;
        }
        catch (const io::io_exception&)
        {
            LOG_ERROR("[gnss_receiver] io error occurred while reconfiguring port");
            return false;
        }
    }

    try
    {
        if (!local_wrapper.initialize(*config))
        {
            LOG_ERROR("[gnss_receiver] failed to initialize port with config");
            return false;
        }
    }
    catch (const io::io_exception&)
    {
        LOG_ERROR("[gnss_receiver] failed to initialize port with config");
        return false;
    }

    config_ = config;
    state_ = state::initialized;
    return true;
}

bool gnss_receiver::start()
{
    LOG_INFO("[gnss_reveiver] starting");
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ != state::initialized)
    {
        return false;
    }

    // Build a fresh decoder registry on every start() so stop()+start() works.
    ubx::parser::ubx_decoder_registry registry;

    if (ubx_db_ != nullptr)
    {
        LOG_INFO("[gnss_receiver] registering UBX messages with parser");
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
        LOG_INFO("[gnss_receiver] raw message callback set");
        ubx_parser_->set_raw_message_callback(raw_msg_cb_);
    }

#ifdef NMEA_PARSER_ENABLED
    if (nmea_db_ != nullptr)
    {
        LOG_INFO("[gnss_receiver] registering NMEA sentences with parser");
        // Build a fresh NMEA registry on every start() so stop()+start() works.
        nmea::parser::nmea_sentence_registry nmea_registry;
        nmea_db_->register_with_parser(nmea_registry);
        nmea_parser_ = std::make_unique<nmea::parser::nmea_parser>(std::move(nmea_registry));
    }
#endif

    running_.store(true);
    LOG_INFO("[gnss_receiver] starting worker thread");
    worker_thread_ = std::thread(&gnss_receiver::worker_loop, this);
    state_ = state::running;
    return true;
}

void gnss_receiver::stop()
{
    LOG_INFO("[gnss_receiver] stopping worker thread");
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
        io::io_channel_wrapper local_wrapper(port_);
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

void gnss_receiver::set_port(std::shared_ptr<io::i_io_port> port)
{
    // Must only be called in the idle state (after terminate()) to ensure the
    // worker thread is not reading from the old port concurrently.
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ != state::idle)
    {
        LOG_WARNING("[gnss_receiver] set_port() called in non-idle state — ignored");
        return;
    }
    port_ = std::move(port);
    LOG_INFO("[gnss_receiver] port swapped");
}

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
    io::io_channel_wrapper local_wrapper(port_);

    fd_event::fd_event_manager fdm;

    std::vector<uint8_t> buf;
    buf.reserve(gnss::UART_READ_CHUNK_BYTES);

    bool data_ready = false;
    bool io_error   = false;
    int  current_fd = -1;
    int  reconfigure_fail_count = 0;

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
    // Returns true when the port comes back with a valid fd (counter reset).
    // Returns false on failure (counter incremented); caller should check
    // reconfigure_fail_count against UART_MAX_RECONFIGURE_RETRIES.
    auto reconfigure_port = [&]() -> bool
    {
        if (current_fd >= 0)
        {
            fdm.remove_fd(current_fd);
            current_fd = -1;
        }
        bool ok = false;
        try
        {
            ok = local_wrapper.reconfigure(*config_);
            if (ok)
            {
                LOG_INFO("[gnss_receiver] port reconfigured successfully");
            }
            else
            {
                LOG_INFO("[gnss_receiver] will retry reconfigure on next loop iteration");
            }
        }
        catch (const io::io_exception& e)
        {
            LOG_ERROR("[gnss_receiver] port reconfigure exception: ", e.what());
        }

        // A successful reconfigure must also yield a valid fd; otherwise the
        // port is still unusable and counts as a failure.
        if (ok && local_wrapper.get_fd() >= 0)
        {
            reconfigure_fail_count = 0;
            return true;
        }

        ++reconfigure_fail_count;
        LOG_INFO("[gnss_receiver] reconfigure failed (attempt ", reconfigure_fail_count, "/", gnss::UART_MAX_RECONFIGURE_RETRIES, ")");

        if (reconfigure_fail_count < gnss::UART_MAX_RECONFIGURE_RETRIES)
        {
            return true; // allow retrying before giving up and exiting the worker loop
        }

        return false;
    };

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (running_.load())
    {
        // ── 1. Validate fd ────────────────────────────────────────────────────
        int fd = local_wrapper.get_fd();
        if (fd < 0)
        {
            LOG_ERROR("[gnss_receiver] invalid fd — reconfiguring port");
            if (!reconfigure_port())
            {
                break; // exit worker loop if reconfigure fails or hits retry limit
            }
            else
            {
                continue; // retry loop with new fd
            }
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
                LOG_ERROR("[gnss_receiver] failed to register fd ", fd, " — reconfiguring port");
                if (!reconfigure_port())
                {
                    break; // exit worker loop if reconfigure fails or hits retry limit
                }
                else
                {
                    continue; // retry loop with new fd
                }
            }
            current_fd = fd;
        }

        // ── 3. Wait up to 1 second for UART data ──────────────────────────────
        data_ready = false;
        io_error   = false;

        int ret = fdm.wait_and_process(gnss::UART_FD_WAIT_TIMEOUT_MS);

        if (ret == 0)
        {
            LOG_INFO("[gnss_receiver] data not available in 1 second — reconfiguring port");
            continue;
        }

        if (ret < 0)
        {
            LOG_ERROR("[gnss_receiver] poll failure: ", strerror(errno));
            if (!reconfigure_port())
            {
                break; // exit worker loop if reconfigure fails or hits retry limit
            }
            else
            {
                continue; // retry loop with new fd
            }
        }

        if (io_error)
        {
            LOG_ERROR("[gnss_receiver] UART error or hangup on fd ", fd);
            if (!reconfigure_port())
            {
                break; // exit worker loop if reconfigure fails or hits retry limit
            }
            else
            {
                continue; // retry loop with new fd
            }
        }

        if (!data_ready)
        {
            continue; // spurious wakeup with no POLLIN
        }

        // ── 4. Query available byte count before reading ───────────────────────
        int avail = local_wrapper.available();
        if (avail <= 0)
        {
            // No data available — this is the normal EOF condition for a
            // non-looping file port (replay mode) where the file fd always
            // signals POLLIN regardless of whether data remains.  Without
            // this sleep the worker would busy-spin at 100 % CPU.
            // For a real UART this path is unreachable because available()
            // returns the TIOCOUTQ/TIOCINQ count, which is >0 after POLLIN.
            std::this_thread::sleep_for(std::chrono::milliseconds(gnss::UART_NO_DATA_SPIN_LOOP_SLEEP_MS));
            continue;
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
            if (!reconfigure_port())
            {
                break; // exit worker loop if reconfigure fails or hits retry limit
            }
        }
        else
        {
            // n == 0: empty read (non-error), just continue
        }
    }

    // ── Cleanup: unregister fd before thread exit ──────────────────────────────
    if (current_fd >= 0)
    {
        LOG_INFO("[gnss_receiver] unregistering fd: ", current_fd);
        fdm.remove_fd(current_fd);
    }

    // Ensure running_ is false whether we exited normally (stop() already set
    // it) or because we hit the reconfigure retry limit (self-terminated).
    running_.store(false);
    LOG_INFO("[gnss_receiver] worker loop exited");
}

} // namespace gnss
