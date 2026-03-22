#pragma once

#include "i_gnss_receiver.h"

#include "serial_helper/i_serial_port.h"
#include "serial_helper/serial_config.h"
#include "ubx_parser/include/ubx_parser.h"
#include "ubx_parser/include/ubx_decoder_registry.h"
#include "ubx_parser/include/ubx_errors.h"

#ifdef NMEA_PARSER_ENABLED
#include "nmea_parser/include/nmea_parser.h"
#include "nmea_parser/include/nmea_sentence_registry.h"
#endif

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace gnss
{

// ─── gnss_receiver ────────────────────────────────────────────────────────────
//
// Concrete UART receiver that:
//   • holds a shared_ptr<i_serial_port> (the real fd owner, shared with the
//     controller's transport adapter; accepts serial_port in production or
//     a mock_serial_port in tests),
//   • creates a local serial_wrapper on demand to perform I/O so the
//     mutex governing the fd always lives inside the port implementation,
//   • runs a dedicated worker thread that uses fd_event::fd_event_manager
//     (backed by poll()) to wait up to UART_FD_WAIT_TIMEOUT_MS (1 second)
//     per cycle, queries available() bytes before each read, and caps the
//     read at UART_READ_CHUNK_BYTES,
//   • on timeout or any I/O error: logs, calls reconfigure(), and continues
//     — the loop never breaks on transient errors so the location service
//     can always trigger a restart without re-initialising the receiver,
//   • rebuilds the parser registry on every start() so that stop()+start()
//     cycling works correctly.
//
// State machine:
//   IDLE ──init()──> INITIALIZED ──start()──> RUNNING
//                         ^                        |
//                         └────────────stop()──────┘
//                         │
//                   terminate()──> IDLE

class gnss_receiver : public gnss::i_gnss_receiver
{
public:
    /// Construct with an abstract serial port that is also shared with the
    /// controller's write path and transport adapter.
    /// Accepts any i_serial_port implementation (serial_port in production,
    /// mock_serial_port in tests).
    explicit gnss_receiver(std::shared_ptr<serial::i_serial_port> port);
    ~gnss_receiver() override;

    // Non-copyable
    gnss_receiver(const gnss_receiver&)            = delete;
    gnss_receiver& operator=(const gnss_receiver&) = delete;

    // ── i_gnss_receiver ───────────────────────────────────────────────────────
    bool init(const serial::serial_config& config) override;
    bool start()                                   override;
    void stop()                                    override;
    void terminate()                               override;
    bool is_running()                        const override;
    void setup_ubx(std::shared_ptr<gnss::ubx_database_wrapper> ubx_db) override;
    void set_extra_parser_setup(
        std::function<void(ubx::parser::ubx_decoder_registry&)> cb) override;
    void set_raw_message_callback(
        ubx::parser::raw_message_callback_t cb) override;

#ifdef NMEA_PARSER_ENABLED
    void setup_nmea(std::shared_ptr<gnss::nmea_database_wrapper> nmea_db) override;
#endif

private:
    enum class state { idle, initialized, running };

    void worker_loop();

    std::shared_ptr<serial::i_serial_port>  port_;
    serial::serial_config                   config_;

    std::shared_ptr<gnss::ubx_database_wrapper>   ubx_db_;
    std::unique_ptr<ubx::parser::ubx_parser> ubx_parser_;

    std::function<void(ubx::parser::ubx_decoder_registry&)> extra_setup_;
    ubx::parser::raw_message_callback_t                      raw_msg_cb_;

#ifdef NMEA_PARSER_ENABLED
    std::shared_ptr<gnss::nmea_database_wrapper>  nmea_db_;
    std::unique_ptr<nmea::parser::nmea_parser>    nmea_parser_;
#endif

    std::atomic<bool>   running_{false};
    std::thread         worker_thread_;
    mutable std::mutex  state_mutex_;
    state               state_{state::idle};
};

} // namespace gnss
