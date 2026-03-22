#pragma once

#include <cstdint>

namespace gnss
{

/// Populated from UBX-TIM-TP and UBX-NAV-TIMEGPS (committed with HP epoch).
struct timing_data
{
    // ── From UBX-TIM-TP ───────────────────────────────────────────────────
    int64_t  tow_ms;             ///< Time-pulse TOW [ms]
    int32_t  q_err_ps;           ///< Quantisation error [picoseconds]
    uint8_t  time_base;          ///< Clock source: 0=GNSS 1=UTC 2=GLONASS 3=BeiDou
    uint8_t  time_ref;           ///< GNSS reference: bits 7:4 of ref_info
    uint8_t  utc_standard;       ///< UTC standard: bits 3:0 of ref_info
    bool     utc_available;      ///< UTC time base available
    bool     q_err_valid;        ///< q_err_ps field is valid

    // ── From UBX-NAV-TIMEGPS ──────────────────────────────────────────────
    uint32_t gps_tow_ms;         ///< GPS time of week [ms]
    uint16_t gps_week;           ///< GPS week number
    int8_t   leap_seconds;       ///< UTC-GPS leap seconds

    // ── From UBX-NAV-STATUS ─────────────────────────────────────────────────────
    uint64_t ttff_ms;            ///< Time to first fix [ms]

    bool     valid;              ///< True when populated at least once
};

} // namespace gnss
