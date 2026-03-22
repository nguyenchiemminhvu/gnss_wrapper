#pragma once

#include <cstdint>
#include <cstddef>

namespace gnss
{

/// Identifies which shared_buffer type was updated.
/// Passed as the argument of base_database::data_updated signal.
enum class buffer_type : uint8_t
{
    // ── Core UBX data buffers ────────────────────────────────────────────────
    location                = 0u, ///< location_data_buffer was refreshed
    satellites                  , ///< satellites_data_buffer was refreshed
    timing                      , ///< timing_data_buffer was refreshed

    // ── Extended UBX data buffers ───────────────────────────────────────────────
    attitude                    , ///< attitude_data_buffer was refreshed (UBX-NAV-ATT)
    dop                         , ///< dop_data_buffer was refreshed (UBX-NAV-DOP)
    raw_measurements            , ///< raw_measurements_data_buffer was refreshed (UBX-RXM-MEASX)
    monitor                     , ///< monitor_data_buffer was refreshed (UBX-MON-IO, UBX-MON-TXBUF)
    security                    , ///< security_data_buffer was refreshed (UBX-SEC-SIG)

    // ── Ublox GNSS chip responses ───────────────────────────────────────────────
    ack_response                , ///< ACK-ACK or ACK-NAK response received from chip
    nack_response               , ///< ACK-NAK response received from chip
    chip_version                , ///< GNSS chipset version info was updated
    valget_response             , ///< CFG-VALGET response received from chip with current config values
    default_config_applied      , ///< config sync cycle completed (VALGET → VALSET applied)
    backup_operation_completed  , ///< backup operation (save/clear) acknowledged by chip
    datum_config                , ///< datum config query cycle completed (NAVSPG-USRDAT VALGET resolved)

    // ── Extended NMEA data buffers (if NMEA_PARSER_ENABLED) ─────────────────────
#ifdef NMEA_PARSER_ENABLED
    nmea_data                   , ///< nmea_data_buffer was refreshed
#endif
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
// after a single space, e.g. "poll_config /etc/default_ubx_configs.ini".

/// Receiver lifecycle commands.
extern const char* CMD_INIT;
extern const char* CMD_START;
extern const char* CMD_STOP;
extern const char* CMD_TERMINATE;
extern const char* CMD_CHECK_RUNNING;

/// GNSS chipset commands.
extern const char* CMD_BACKUP;
extern const char* CMD_CLEAR_BACKUP;
extern const char* CMD_HOT_STOP;
extern const char* CMD_WARM_STOP;
extern const char* CMD_COLD_STOP;
extern const char* CMD_HOT_START;
extern const char* CMD_WARM_START;
extern const char* CMD_COLD_START;
extern const char* CMD_HOT_RESET;
extern const char* CMD_WARM_RESET;
extern const char* CMD_COLD_RESET;
extern const char* CMD_GET_VERSION;
extern const char* CMD_POLL_CONFIG;
extern const char* CMD_SYNC_CONFIG;
extern const char* CMD_QUERY_DATUM;

/// Maximum bytes read from UART per loop iteration in the receiver worker thread.
static constexpr std::size_t UART_READ_CHUNK_BYTES = 2048u;

/// Maximum consecutive reconfigure attempts before the worker thread gives up
/// and exits, allowing an external component to trigger a fresh start().
static constexpr int32_t UART_MAX_RECONFIGURE_RETRIES = 5;

/// Timeout used by the receiver worker's fd_event wait per cycle [milliseconds].
/// If no data arrives within this window the worker logs and reconfigures the port.
static constexpr int32_t UART_FD_WAIT_TIMEOUT_MS = 1000;

/// Poll duration is counted from the time a poll command is sent until receiving ACK message.
static constexpr uint16_t UART_POLL_TIMEOUT_MS = 1000U;

/// Timeout for waiting for a MON-VER polling in milliseconds.
static constexpr uint16_t GNSS_CHIP_VERSION_POLL_INTERVAL_MS = 2000U;

/// Timeout for retrying application of default config if chip version is not yet known, in milliseconds.
static constexpr uint16_t GNSS_DEFAULT_CONFIG_APPLY_RETRY_INTERVAL_MS = 2000U;

/// Duration to wait before reinitializing in milliseconds.
static constexpr uint32_t REINITIALIZATION_DURATION = 2000U;

/// Duration thresholds for diagnostic monitoring of GNSS PVT solution in milliseconds.
static constexpr uint16_t DIAGNOSTIC_MONITORING_GNSS_RAW_DATA_DURATION = 10000U;

/// Duration threshold for diagnosing GNSS malfunction in milliseconds. This is expected to be sufficiently long to avoid false positive diagnosis due to random delays, while short enough to detect real malfunction in a reasonable time frame and trigger recovery actions.
static constexpr uint16_t DIAGNOSTIC_MONITORING_GNSS_MALFUNCTION_DURATION = 60000U;

} // namespace gnss
