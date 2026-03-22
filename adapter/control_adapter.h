#pragma once

#include "command_tokenizer.h"
#include "i_gnss_adapter.h"
#include "i_gnss_command.h"
#include "../gnss_controller/gnss_controller.h"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnss
{

// ─── control_adapter ──────────────────────────────────────────────────────────
//
// Service-facing integration point.  Owns the gnss_controller (+ everything it
// chains: serial ports, receivers, parser, database) and exposes the
// i_gnss_adapter command-string API to the Location Manager.
//
// Multi-UART support:
//   Accepts a vector of serial_config objects.  The FIRST element is the
//   control UART (UART1): all GNSS control/config commands are sent through
//   this serial path.  Additional entries add read-only data receiver UARTs.
//   All receivers share one ubx_database_wrapper.
//
// Data publishing:
//   The ubx_database_wrapper member (also owned by gnss_controller) fires
//   ubx_base_database::data_updated for each refreshed shared_buffer.  The Location
//   Manager connects its slot here before calling execute_command("init"):
//
//     adapter.get_ubx_database()->data_updated.connect(
//         this, &LocationManager::on_data_updated);
//
// Typical startup sequence (multi-UART):
//   std::vector<serial::serial_config> cfgs = {
//       serial::serial_config::gnss_default("/dev/ttyS3"),  // control UART
//       serial::serial_config::gnss_default("/dev/ttyS4")   // data-only UART
//   };
//   control_adapter adapter(std::move(cfgs));
//   adapter.get_ubx_database()->data_updated.connect(...);
//   adapter.execute_command("init");
//   adapter.execute_command("start");
//
// Backward-compatible single-UART usage:
//   control_adapter adapter(serial::serial_config::gnss_default("/dev/ttyS3"));

class control_adapter : public i_gnss_adapter
{
public:
    /// Primary constructor — one entry per UART.
    /// The first element is the control UART; further elements are data UARTs.
    /// @throws std::invalid_argument if configs is empty.
    explicit control_adapter(std::vector<serial::serial_config> configs);

    /// Backward-compatible single-UART constructor.
    explicit control_adapter(const serial::serial_config& config);

    ~control_adapter() override = default;

    // Non-copyable
    control_adapter(const control_adapter&)            = delete;
    control_adapter& operator=(const control_adapter&) = delete;

    // ── i_gnss_adapter ────────────────────────────────────────────────────────
    bool execute_command(const std::string& cmd) override;

    // ── Data access ───────────────────────────────────────────────────────────
    /// Return the shared ubx_database_wrapper for subscriber connection and data reads.
    std::shared_ptr<gnss::ubx_database_wrapper> get_ubx_database() const override;

#ifdef NMEA_PARSER_ENABLED
    /// Return the shared nmea_database_wrapper for NMEA subscriber connection and data reads.
    std::shared_ptr<gnss::nmea_database_wrapper> get_nmea_database() const override;
#endif

private:
    /// Factory method — maps a parsed command to a concrete i_gnss_command object.
    /// Returns nullptr for unrecognised command names.
    std::unique_ptr<i_gnss_command> create_command(const gnss::parsed_command& p);

    /// Populate command_table_ with all supported command factories.
    void build_command_table();

    using CommandFactory =
        std::function<std::unique_ptr<i_gnss_command>(const gnss::parsed_command&)>;

    std::unordered_map<std::string, CommandFactory> command_table_;
    std::shared_ptr<gnss::gnss_controller>          controller_;
};

} // namespace gnss
