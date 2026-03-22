#pragma once

#include <string>
#include <memory>
#include "../shared_buffer/ubx_database_wrapper.h"
#ifdef NMEA_PARSER_ENABLED
#include "../shared_buffer/nmea_database_wrapper.h"
#endif

namespace gnss
{

// ─── i_gnss_adapter ───────────────────────────────────────────────────────────
//
// Service-facing interface for the GNSS HAL adapter.
// The Location Manager calls execute_command() with a command string whose
// format is: "<command> [arg1] [arg2] ... [argN]"
// where the command name is the first whitespace-delimited token and up to
// MAX_COMMAND_ARGS (6) optional argument tokens may follow.
//
// Supported commands
// ──────────────────
// Receiver lifecycle:
//   "init"          — Open serial port and prepare parser pipeline.
//   "start"         — Launch UART reader thread.
//   "stop"          — Stop UART reader thread (port remains open).
//   "terminate"     — Stop thread and close serial port.
//   "check_running" — Returns true if the reader thread is active.
//
// GNSS chip commands:
//   "backup"        — Save aiding data to battery-backed RAM (UBX-UPD-SOS).
//   "clear_backup"  — Erase previously saved aiding data.
//   "hot_start"     — Hot-start: keep all aiding data.
//   "warm_start"    — Warm-start: keep almanac, discard ephemeris.
//   "cold_start"    — Cold-start: erase all aiding data.
//   "get_version"   — Poll UBX-MON-VER.
//
// Config sync:
//   "sync_config [/path/to/config.ini]"
//               — Apply configuration from the given INI file.
//                 Omitting the path uses DEFAULT_CONFIG_INI_PATH.

class i_gnss_adapter
{
public:
    virtual ~i_gnss_adapter() = default;

    /// Execute a command identified by its string token.
    /// @param cmd  One of the command strings listed above.
    /// @return true on success; false on unknown command or execution failure.
    virtual bool execute_command(const std::string& cmd) = 0;

    /// Access the shared ubx_database_wrapper for subscriber connection and data reads.
    /// @return A shared pointer to the ubx_database_wrapper instance.
    virtual std::shared_ptr<gnss::ubx_database_wrapper> get_ubx_database() const = 0;

#ifdef NMEA_PARSER_ENABLED
    /// Access the shared nmea_database_wrapper for NMEA subscriber connection and data reads.
    /// @return A shared pointer to the nmea_database_wrapper instance.
    virtual std::shared_ptr<gnss::nmea_database_wrapper> get_nmea_database() const = 0;
#endif
};

} // namespace gnss
