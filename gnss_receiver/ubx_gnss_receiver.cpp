#include "ubx_gnss_receiver.h"
#include "global_constants.h"
#include "libevent.h"
#include "serial_helper/serial_wrapper.h"
#include "serial_helper/serial_exception.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace gnss
{

// ─── Constructor / destructor ──────────────────────────────────────────────────

ubx_gnss_receiver::ubx_gnss_receiver(std::shared_ptr<serial::i_serial_port> port)
    : port_(std::move(port))
{
}

ubx_gnss_receiver::~ubx_gnss_receiver()
{
    stop();
    terminate();
}

// ─── Lifecycle ─────────────────────────────────────────────────────────────────

bool ubx_gnss_receiver::init(const serial::serial_config& config)
{
    std::lock_guard<std::mutex> lk(state_mutex_);
    serial::serial_wrapper local_wrapper(port_);
    if (state_ != state::idle)
    {
        // Already initialized — reconfigure the port.
        try
        {
            if (!local_wrapper.reconfigure(config))
            {
                return false;
            }
            config_ = config;
            return true;
        }
        catch (const serial::serial_error&)
        {
            return false;
        }
    }

    try
    {
        if (!local_wrapper.initialize(config))
        {
            return false;
        }
    }
    catch (const serial::serial_error&)
    {
        return false;
    }

    config_ = config;
    state_ = state::initialized;
    return true;
}

bool ubx_gnss_receiver::start()
{
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (state_ != state::initialized)
    {
        return false;
    }

    // Build a fresh decoder registry on every start() so stop()+start() works.
    ubx::parser::ubx_decoder_registry registry;

    if (db_ != nullptr)
    {
        db_->register_with_parser(registry);
    }

    if (extra_setup_)
    {
        extra_setup_(registry);
    }

    parser_ = std::make_unique<ubx::parser::ubx_parser>(std::move(registry));

    parser_->set_error_callback([](const ubx::parser::parse_error_info& err) {
        // Errors are silently dropped; a production build could log them here.
        (void)err;
    });

    if (raw_msg_cb_)
    {
        parser_->set_raw_message_callback(raw_msg_cb_);
    }

    running_.store(true);
    worker_thread_ = std::thread(&ubx_gnss_receiver::worker_loop, this);
    state_ = state::running;
    return true;
}

void ubx_gnss_receiver::stop()
{
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

    parser_.reset();

    std::lock_guard<std::mutex> lk(state_mutex_);
    state_ = state::initialized;
}

void ubx_gnss_receiver::terminate()
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

bool ubx_gnss_receiver::is_running() const
{
    return running_.load();
}

// ─── Parser wiring ─────────────────────────────────────────────────────────────

void ubx_gnss_receiver::setup(std::shared_ptr<ubx_database_wrapper> db)
{
    db_ = db;
}

void ubx_gnss_receiver::set_extra_parser_setup(
    std::function<void(ubx::parser::ubx_decoder_registry&)> cb)
{
    extra_setup_ = std::move(cb);
}

void ubx_gnss_receiver::set_raw_message_callback(
    ubx::parser::raw_message_callback_t cb)
{
    raw_msg_cb_ = std::move(cb);
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

void ubx_gnss_receiver::worker_loop()
{
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
                // fprintf(stderr, "[ubx_gnss_receiver] port reconfigured successfully\n");
            }
            else
            {
                // fprintf(stderr, "[ubx_gnss_receiver] port reconfigure returned false\n");
            }
        }
        catch (const serial::serial_error& e)
        {
            // fprintf(stderr, "[ubx_gnss_receiver] port reconfigure exception: %s\n", e.what());
        }
    };

    // ── Main loop ─────────────────────────────────────────────────────────────
    while (running_.load())
    {
        // ── 1. Validate fd ────────────────────────────────────────────────────
        int fd = local_wrapper.get_fd();
        if (fd < 0)
        {
            // fprintf(stderr, "[ubx_gnss_receiver] invalid fd — reconfiguring port\n");
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
                // fprintf(stderr, "[ubx_gnss_receiver] failed to register fd %d — reconfiguring\n", fd);
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
            // fprintf(stderr, "[ubx_gnss_receiver] data not available in 1 second\n");
            reconfigure_port();
            continue;
        }

        if (ret < 0)
        {
            // fprintf(stderr, "[ubx_gnss_receiver] poll failure: %s\n", strerror(errno));
            reconfigure_port();
            continue;
        }

        if (io_error)
        {
            // fprintf(stderr, "[ubx_gnss_receiver] UART error or hangup on fd %d\n", fd);
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
            if (parser_)
            {
                parser_->feed(buf);
            }
        }
        else if (n < 0)
        {
            // fprintf(stderr, "[ubx_gnss_receiver] read error on fd %d — reconfiguring\n", fd);
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
        fdm.remove_fd(current_fd);
    }

    running_.store(false);
}

} // namespace gnss
