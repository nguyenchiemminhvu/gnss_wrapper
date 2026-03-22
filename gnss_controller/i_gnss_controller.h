#pragma once

#include "../shared_buffer/ubx_database_wrapper.h"
#ifdef NMEA_PARSER_ENABLED
#include "../shared_buffer/nmea_database_wrapper.h"
#endif

#include <string>
#include <memory>

namespace gnss
{

// ─── i_gnss_controller ────────────────────────────────────────────────────────
//
// Abstract interface for the GNSS controller.  Provides both lifecycle
// management for the HAL and control commands directed at the GNSS chipset.

class i_gnss_controller
{
public:
    virtual ~i_gnss_controller() = default;

    // ── Receiver lifecycle ────────────────────────────────────────────────────

    /// Initialise the serial port and prepare the parser pipeline.
    virtual bool init()      = 0;

    /// Start the UART reader thread.  Requires init() to have succeeded.
    virtual bool start()     = 0;

    /// Stop the UART reader thread (serial port remains open).
    virtual void stop()      = 0;

    /// Stop the thread and close the serial port.
    virtual void terminate() = 0;

    /// @return true if the reader thread is currently running.
    virtual bool is_running() const = 0;

    // ── GNSS chip commands ────────────────────────────────────────────────────

    /// Send a UBX-CFG-RST hot-stop (keeps all aiding data).
    virtual bool hot_stop() = 0;

    /// Send a UBX-CFG-RST warm-stop (keeps almanac, discards ephemeris).
    virtual bool warm_stop() = 0;

    /// Send a UBX-CFG-RST cold-stop (erases all aiding data).
    virtual bool cold_stop() = 0;

    /// Send a UBX-CFG-RST hot-start (keeps all aiding data).
    virtual bool hot_start()  = 0;

    /// Send a UBX-CFG-RST warm-start (keeps almanac, discards ephemeris).
    virtual bool warm_start() = 0;

    /// Send a UBX-CFG-RST cold-start (erases all aiding data).
    virtual bool cold_start() = 0;

    /// Send a UBX-CFG-RST hot-reset (hot-start + reset receiver HW).
    virtual bool hot_reset() = 0;

    /// Send a UBX-CFG-RST warm-reset (warm-start + reset receiver HW).
    virtual bool warm_reset() = 0;

    /// Send a UBX-CFG-RST cold-reset (cold-start + reset receiver HW).
    virtual bool cold_reset() = 0;

    /// Poll UBX-MON-VER to read firmware/hardware version.
    virtual bool get_version() = 0;

    /// Send a UBX-UPD-SOS backup command to save GNSS state to battery RAM.
    virtual bool backup() = 0;

    /// Send a UBX-UPD-SOS clear command to erase the backup in battery RAM.
    virtual bool clear_backup() = 0;

    /// Load defaults from the given INI file and apply to the GNSS chip.
    virtual bool poll_config(const std::string& ini_path) = 0;

    /// Apply defaults from the given INI file and sync to the GNSS chip.
    virtual bool sync_config() = 0;

    /// Poll the chip's NAVSPG-USRDAT configuration keys and resolve the active
    /// coordinate datum (WGS-84, PZ-90, GRS-80, …).
    /// Completion is signalled via data_updated(buffer_type::datum_config).
    virtual bool query_datum() = 0;

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
