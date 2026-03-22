#pragma once

#include "command_tokenizer.h"
#include "i_gnss_adapter.h"
#include "i_gnss_command.h"
#include "../gnss_controller/gnss_controller.h"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

namespace gnss
{

// ─── control_adapter ──────────────────────────────────────────────────────────
//
// Service-facing integration point.  Owns the gnss_controller (+ everything it
// chains: serial port, receiver, parser, database) and exposes the
// i_gnss_adapter command-string API to the Location Manager.
//
// Data publishing:
//   The ubx_database_wrapper member (also owned by gnss_controller) fires
//   base_database::data_updated for each refreshed shared_buffer.  The Location
//   Manager connects its slot here before calling execute_command("init"):
//
//     adapter.get_database()->data_updated.connect(
//         this, &LocationManager::on_data_updated);
//
// Typical startup sequence:
//   control_adapter adapter(serial::serial_config::gnss_default("/dev/ttyS3"));
//   adapter.get_database()->data_updated.connect(...);
//   adapter.execute_command("init");
//   adapter.execute_command("start");

class control_adapter : public i_gnss_adapter
{
public:
    /// @param config  Serial port configuration for the GNSS UART.
    explicit control_adapter(const serial::serial_config& config);

    ~control_adapter() override = default;

    // Non-copyable
    control_adapter(const control_adapter&)            = delete;
    control_adapter& operator=(const control_adapter&) = delete;

    // ── i_gnss_adapter ────────────────────────────────────────────────────────
    bool execute_command(const std::string& cmd) override;

    // ── Data access ───────────────────────────────────────────────────────────
    /// Return the shared ubx_database_wrapper for subscriber connection and data reads.
    std::shared_ptr<ubx_database_wrapper> get_database() const override;

private:
    /// Factory method — maps a parsed command to a concrete i_gnss_command object.
    /// Returns nullptr for unrecognised command names.
    std::unique_ptr<i_gnss_command> create_command(const parsed_command& p);

    /// Populate command_table_ with all supported command factories.
    void build_command_table();

    using CommandFactory =
        std::function<std::unique_ptr<i_gnss_command>(const parsed_command&)>;

    std::unordered_map<std::string, CommandFactory> command_table_;
    std::shared_ptr<gnss_controller>                controller_;
};

} // namespace gnss
