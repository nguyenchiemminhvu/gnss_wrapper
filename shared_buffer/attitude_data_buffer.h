#pragma once

#include <cstdint>

namespace gnss
{

/// Populated from UBX-NAV-ATT (high-priority epoch commit).
/// Angles already converted to degrees (1e-5 raw → deg) by the DB handler.
struct attitude_data
{
    // ── From UBX-NAV-ATT ─────────────────────────────────────────────────────
    uint32_t i_tow_ms;          ///< GPS time of week [ms]
    uint8_t  version;           ///< Message version (= 0)

    float    roll_deg;          ///< Vehicle roll  [degrees]
    float    pitch_deg;         ///< Vehicle pitch [degrees]
    float    heading_deg;       ///< Vehicle heading [degrees]

    float    acc_roll_deg;      ///< Roll  accuracy [degrees]
    float    acc_pitch_deg;     ///< Pitch accuracy [degrees]
    float    acc_heading_deg;   ///< Heading accuracy [degrees]

    bool     valid;             ///< True when populated at least once
};

} // namespace gnss
