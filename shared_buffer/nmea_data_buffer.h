#pragma once

#ifdef NMEA_PARSER_ENABLED

#include <cstdint>
#include <cstddef>

namespace gnss
{

/// Maximum number of satellites tracked in the NMEA sky-view buffer.
/// Matches nmea::parser::NMEA_MAX_SATS_IN_VIEW defined in nmea_types.h.
static constexpr uint8_t NMEA_MAX_SAT_VIEW = 64u;

// ─── nmea_satellite_record ────────────────────────────────────────────────────
//
// Per-satellite entry from GSV sentences.

struct nmea_satellite_record
{
    uint8_t  sv_id;           ///< Satellite identifier (PRN / SVID)
    int8_t   elevation_deg;   ///< Elevation above horizon [degrees]
    int16_t  azimuth_deg;     ///< Azimuth clockwise from true north [degrees, 0..359]
    uint8_t  snr;             ///< Signal-to-noise ratio [dBHz]; 0 if not tracked
};

// ─── nmea_data ────────────────────────────────────────────────────────────────
//
// General data collection from NMEA 0183 sentences.
// Populated by nmea_database_wrapper from GGA, RMC, GSA, VTG and GSV sentences.
//
// Fields from each sentence:
//   GGA  — primary position, altitude, fix quality, number of SVs, HDOP
//   RMC  — speed, course (true), date/validity status
//   GSA  — navigation mode, PDOP, HDOP (overrides GGA HDOP), VDOP
//   VTG  — speed and course in alternative units/reference
//   GSV  — satellite sky-view table

struct nmea_data
{
    // ── From GGA (primary position fix) ───────────────────────────────────────
    double   latitude_deg;          ///< Latitude  [degrees]; positive = north
    double   longitude_deg;         ///< Longitude [degrees]; positive = east
    float    altitude_msl_m;        ///< Altitude above mean sea level [metres]
    float    geoid_separation_m;    ///< Geoid undulation (height of WGS-84 above MSL) [metres]
    float    hdop;                  ///< Horizontal dilution of precision (GGA/GSA)
    uint8_t  fix_quality;           ///< Fix quality: 0=invalid 1=GPS 2=DGPS 4=RTK-fix 5=RTK-float
    uint8_t  num_satellites;        ///< Number of SVs used in the current GGA solution

    // ── From RMC (recommended minimum specific GNSS data) ─────────────────────
    float    speed_knots;           ///< Speed over ground [knots]
    float    course_true_deg;       ///< Course over ground, true heading [degrees]
    bool     rmc_status_active;     ///< true = data valid (RMC status field = 'A')

    // ── From GSA (GNSS DOP and active satellites) ─────────────────────────────
    uint8_t  nav_mode;              ///< Navigation mode: 1=no fix, 2=2-D fix, 3=3-D fix
    float    pdop;                  ///< Position dilution of precision
    float    vdop;                  ///< Vertical dilution of precision

    // ── From VTG (course over ground and ground speed) ────────────────────────
    float    speed_kmph;            ///< Speed over ground [km/h]
    float    course_mag_deg;        ///< Course over ground, magnetic heading [degrees]

    // ── From GSV (satellites in view) ─────────────────────────────────────────
    uint8_t              num_sats_in_view;              ///< Total SVs visible (GSV talkers combined)
    nmea_satellite_record sats[NMEA_MAX_SAT_VIEW];      ///< Per-satellite sky-view records

    // ── Validity ──────────────────────────────────────────────────────────────
    bool     valid;                 ///< true once at least one HP commit has fired (GGA or RMC)
};

} // namespace gnss

#endif // NMEA_PARSER_ENABLED
