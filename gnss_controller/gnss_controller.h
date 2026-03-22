#pragma once

#include "i_gnss_controller.h"

#include "../gnss_receiver/ubx_gnss_receiver.h"
#include "../shared_buffer/ubx_database_wrapper.h"

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
//   • serial_wrapper   — shared with ubx_gnss_receiver for thread-safe I/O.
//   • ubx_gnss_receiver    — UART reader + UBX parser pipeline.
//   • ubx_database_wrapper   — parser-to-shared_buffer mapping, exposed to adapter.
//   • i_ubx_transport         (concrete: serial_transport_impl)
//   • i_ini_config_provider   (concrete: ini_config_provider_impl)
//   • ubx_config_manager (+ repository)
//                      — config sync / direct VALSET/VALGET.
//
// The transport and INI provider are managed exclusively through their
// abstract interfaces to honour the Dependency-Inversion Principle.
//
// CFG-VALGET responses are dispatched through the typed cfg_valget_decoder
// registered in the receiver's parser registry.  The decoded message is
// forwarded directly to ubx_config_manager::on_valget_response() and the
// pending sync cycle completes without any raw-message scanning.
//
// Usage:
//   gnss_controller ctrl("/dev/ttyS3");
//   ctrl.init();    // opens serial, prepares parser pipeline
//   ctrl.start();   // launches UART reader thread
//   // ... use ctrl.get_database() to access parsed data ...
//   ctrl.stop();
//   ctrl.terminate();

class gnss_controller : public i_gnss_controller
{
public:
    /// @param config  Full serial configuration for the GNSS UART.
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

    bool hot_start()                             override;
    bool warm_start()                            override;
    bool cold_start()                            override;
    bool get_version()                           override;
    bool backup()                                override;
    bool clear_backup()                          override;
    bool sync_config(const std::string& ini_path) override;

    // ── Data access ───────────────────────────────────────────────────────────
    std::shared_ptr<ubx_database_wrapper> get_database() const;

private:
    // Helper: send an empty-payload poll request.
    bool send_poll(uint8_t msg_class, uint8_t msg_id);

    serial::serial_config                       config_;
    std::shared_ptr<serial::serial_port>        port_;     ///< Owns the POSIX fd; shared with receiver and transport
    std::shared_ptr<gnss::ubx_database_wrapper> database_;
    std::shared_ptr<gnss::ubx_gnss_receiver>    receiver_;

    // Config management — order matters for initialisation.
    std::shared_ptr<ubx::config::i_ubx_transport>           transport_;
    std::shared_ptr<ubx::config::i_ini_config_provider>     ini_provider_;
    std::shared_ptr<ubx::config::i_ubx_config_repository>   config_repo_;
    std::unique_ptr<ubx::config::ubx_config_manager>        config_manager_;

    mutable std::mutex cmd_mutex_; ///< Serialise control commands.
};

} // namespace gnss
