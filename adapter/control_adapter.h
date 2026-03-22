#pragma once

#include "command_tokenizer.h"
#include "i_gnss_adapter.h"
#include "i_gnss_command.h"
#include "../gnss_controller/gnss_controller.h"
#include "gpio/i_gpio_pin.h"

#include "io_helper/i_io_config.h"
#include "io_helper/uart/uart_config.h"

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
//   Accepts a vector of i_io_config objects.  The FIRST element is the
//   control channel (UART1): all GNSS control/config commands are sent through
//   this path.  Additional entries add read-only data receiver channels.
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
//   std::vector<std::shared_ptr<io::i_io_config>> cfgs = {
//       std::make_shared<io::uart_config>(io::uart_config::gnss_default("/dev/ttyS3")),
//       std::make_shared<io::uart_config>(io::uart_config::gnss_default("/dev/ttyS4"))
//   };
//   control_adapter adapter(std::move(cfgs));
//   adapter.get_ubx_database()->data_updated.connect(...);
//   adapter.execute_command("init");
//   adapter.execute_command("start");
//
// Backward-compatible single-UART usage:
//   control_adapter adapter(io::uart_config::gnss_default("/dev/ttyS3"));

class control_adapter : public i_gnss_adapter
{
public:
    /// Primary constructor — one entry per channel.
    /// The first element is the control channel; further elements are data channels.
    /// @throws std::invalid_argument if configs is empty.
    explicit control_adapter(std::vector<std::shared_ptr<io::i_io_config>> configs);

    /// Single-channel convenience constructor accepting a shared_ptr<i_io_config>.
    explicit control_adapter(std::shared_ptr<io::i_io_config> config);

    /// Backward-compatible UART convenience constructor.
    /// Wraps a uart_config into a shared_ptr<i_io_config> and delegates.
    explicit control_adapter(const io::uart_config& config);

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

    // ── GPIO configuration ────────────────────────────────────────────────────

    /// Configure the RESET_N GPIO pin used by the "hardware_reset" command.
    ///
    /// Call this once after construction and before issuing "hardware_reset".
    /// Passing nullptr disables the hardware-reset capability.
    ///
    /// Example (file-based BSP GPIO):
    ///   adapter.set_reset_pin(
    ///       std::make_shared<gnss::gpio_pin_impl>(
    ///           gnss::DEFAULT_RESET_N_GPIO_PATH, "RESET_N"));
    void set_reset_pin(std::shared_ptr<gnss::i_gpio_pin> pin);

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
