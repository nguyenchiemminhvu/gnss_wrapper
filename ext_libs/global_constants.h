#pragma once

#include <cstdint>
#include <cstddef>

namespace gnss
{

/// Identifies which shared_buffer type was updated.
/// Passed as the argument of base_database::data_updated signal.
enum class buffer_type : uint8_t
{
    location       = 0u, ///< location_data_buffer was refreshed
    satellites     = 1u, ///< satellites_data_buffer was refreshed
    timing         = 2u, ///< timing_data_buffer was refreshed
#ifdef NMEA_PARSER_ENABLED
    nmea_data      = 3u, ///< nmea_data_buffer was refreshed
#endif
    chip_version   = 4u, ///< GNSS chipset version info was updated
    default_config_applied = 5u, ///< config sync cycle completed (VALGET → VALSET applied)
};

/// Identifies a GNSS constellation by its u-blox GNSS identifier value.
enum class constellation : uint8_t
{
    gps     = 0u, ///< GPS
    sbas    = 1u, ///< SBAS
    galileo = 2u, ///< Galileo
    beidou  = 3u, ///< BeiDou
    qzss    = 5u, ///< QZSS
    glonass = 6u, ///< GLONASS
    navic   = 7u, ///< NavIC
};

/// Total number of supported GNSS constellations.
static constexpr uint8_t GNSS_TOTAL_CONSTELLATION = 8u;

/// Default UART device path for the u-blox GNSS chipset.
extern const char* DEFAULT_UART_DEVICE;

/// Default filesystem path for the INI configuration file.
extern const char* DEFAULT_CONFIG_INI_PATH;

/// Maximum number of argument tokens a command string may carry (tokens[1] … [MAX_COMMAND_ARGS]).
static constexpr std::size_t MAX_COMMAND_ARGS = 6u;

// ── Command name tokens ──────────────────────────────────────────────────────
//
// Each string is the first whitespace-delimited token of a command string
// passed to i_gnss_adapter::execute_command().  Arguments (if any) follow
// after a single space, e.g. "sync_config /etc/default_ubx_configs.ini".

/// Receiver lifecycle commands.
extern const char* CMD_INIT;
extern const char* CMD_START;
extern const char* CMD_STOP;
extern const char* CMD_TERMINATE;
extern const char* CMD_CHECK_RUNNING;

/// GNSS chipset commands.
extern const char* CMD_BACKUP;
extern const char* CMD_CLEAR_BACKUP;
extern const char* CMD_HOT_START;
extern const char* CMD_WARM_START;
extern const char* CMD_COLD_START;
extern const char* CMD_GET_VERSION;
extern const char* CMD_SYNC_CONFIG;

/// Maximum bytes read from UART per loop iteration in the receiver worker thread.
static constexpr std::size_t UART_READ_CHUNK_BYTES = 2048u;

/// Timeout used by the receiver worker's fd_event wait per cycle [milliseconds].
/// If no data arrives within this window the worker logs and reconfigures the port.
static constexpr int UART_FD_WAIT_TIMEOUT_MS = 1000;

/// Legacy poll timeout constant (kept for backward compatibility).
static constexpr int UART_POLL_TIMEOUT_MS = 50;

/// Timeout for waiting for a MON-VER polling in milliseconds.
static constexpr int GNSS_CHIP_VERSION_POLL_INTERVAL_MS = 3000;

/// Timeout for retrying application of default config if chip version is not yet known, in milliseconds.
static constexpr int GNSS_DEFAULT_CONFIG_APPLY_RETRY_INTERVAL_MS = 3000;

} // namespace gnss
