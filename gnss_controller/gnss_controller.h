#pragma once

#include "i_gnss_controller.h"

#include "../gnss_receiver/gnss_receiver.h"
#include "../shared_buffer/ubx_database_wrapper.h"

#ifdef NMEA_PARSER_ENABLED
#include "../shared_buffer/nmea_database_wrapper.h"
#endif

#include "serial_helper/serial_config.h"
#include "serial_helper/serial_port.h"
#include "ubx_parser/include/config/ubx_config_manager.h"
#include "ubx_parser/include/config/ubx_config_repository.h"
#include "ubx_parser/include/config/i_ubx_transport.h"
#include "ubx_parser/include/config/i_ini_config_provider.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace gnss
{

// ─── gnss_controller ──────────────────────────────────────────────────────────
//
// Top-level orchestrator.  Owns:
//   • ports_[i]       — one serial_port per configured UART.
//   • receivers_[i]   — one gnss_receiver per UART, sharing one database.
//   • ubx_database_       — single ubx_database_wrapper shared by all receivers.
//   • transport_      — i_ubx_transport bound to ports_[0] (control UART).
//   • i_ini_config_provider / ubx_config_manager
//                      — config sync / direct VALSET/VALGET via control UART.
//
// Multi-UART behaviour:
//   The first element of configs_ is the control UART.  All chip control
//   commands (hot/warm/cold start, backup, etc.) are sent through ports_[0].
//   Receivers at indices 1..N-1 are data-only; the CFG-VALGET / MON-VER
//   extra decoders are only installed on receivers_[0].
//
// Thread-safety:
//   • cmd_mutex_ serialises all control commands.
//   • ubx_database::apply_update() holds an exclusive shared_timed_mutex,
//     so concurrent writes from multiple receiver threads are serialised.
//   • Distinct UBX message routing per UART is assumed.
//
// Usage (multi-UART):
//   std::vector<serial::serial_config> cfgs = {
//       serial::serial_config::gnss_default("/dev/ttyS3"),
//       serial::serial_config::gnss_default("/dev/ttyS4")
//   };
//   gnss_controller ctrl(std::move(cfgs));
//   ctrl.init();
//   ctrl.start();

class gnss_controller : public i_gnss_controller
{
public:
    /// Primary constructor — one serial_config per UART.
    /// The first element is the control UART.
    /// @throws std::invalid_argument if configs is empty.
    explicit gnss_controller(std::vector<serial::serial_config> configs);

    /// Backward-compatible single-UART constructor.
    explicit gnss_controller(const serial::serial_config& config);

    ~gnss_controller() override;

    // Non-copyable
    gnss_controller(const gnss_controller&)            = delete;
    gnss_controller& operator=(const gnss_controller&) = delete;

    // ── i_gnss_controller ─────────────────────────────────────────────────────
    bool init()                                  override;
    bool start()                                 override;
    void stop()                                  override;
    void terminate()                             override;
    bool is_running()                      const override;

    bool hot_stop()                              override;
    bool warm_stop()                             override;
    bool cold_stop()                             override;
    bool hot_start()                             override;
    bool warm_start()                            override;
    bool cold_start()                            override;
    bool get_version()                           override;
    bool backup()                                override;
    bool clear_backup()                          override;
    bool sync_config(const std::string& ini_path) override;

    // ── Data access ───────────────────────────────────────────────────────────
    std::shared_ptr<gnss::ubx_database_wrapper> get_ubx_database() const override;

#ifdef NMEA_PARSER_ENABLED
    /// Return the shared nmea_database_wrapper for NMEA subscriber connection and data reads.
    std::shared_ptr<gnss::nmea_database_wrapper> get_nmea_database() const override;
#endif

private:
    // Helper: send an empty-payload poll request via the control UART.
    bool send_poll(uint8_t msg_class, uint8_t msg_id);

    std::vector<serial::serial_config>                  configs_;
    std::vector<std::shared_ptr<serial::serial_port>>   ports_;     ///< One port per UART; ports_[0] is the control port

    std::shared_ptr<gnss::ubx_database_wrapper>         ubx_database_;  ///< Shared by all receivers
#ifdef NMEA_PARSER_ENABLED
    std::shared_ptr<gnss::nmea_database_wrapper>        nmea_database_; ///< Shared by all receivers
#endif

    std::vector<std::shared_ptr<gnss::gnss_receiver>>   receivers_; ///< One receiver per UART

    // Config management — all bound to the control UART (ports_[0]).
    std::shared_ptr<ubx::config::i_ubx_transport>           transport_;
    std::shared_ptr<ubx::config::i_ini_config_provider>     ini_provider_;
    std::shared_ptr<ubx::config::i_ubx_config_repository>   config_repo_;
    std::unique_ptr<ubx::config::ubx_config_manager>        config_manager_;

    mutable std::mutex cmd_mutex_; ///< Serialise control commands.
};

} // namespace gnss
