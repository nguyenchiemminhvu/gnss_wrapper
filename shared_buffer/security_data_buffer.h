#pragma once

#include <cstdint>

namespace gnss
{

/// Jamming/spoofing detection state populated from UBX-SEC-SIG.
struct security_data
{
    // ── Jamming / interference ────────────────────────────────────────────────
    uint8_t  jam_flags;         ///< Raw jamFlags byte (X1)
    uint8_t  jam_det_enabled;   ///< bit 0: jamming detection enabled (0/1)
    /// Jamming state:
    ///   0 = Unknown
    ///   1 = No jamming indicated
    ///   2 = Warning – jamming indicated but fix OK
    ///   3 = Critical – jamming indicated and no fix
    uint8_t  jamming_state;

    // ── Spoofing ──────────────────────────────────────────────────────────────
    uint8_t  spf_flags;         ///< Raw spfFlags byte (X1)
    uint8_t  spf_det_enabled;   ///< bit 0: spoofing detection enabled (0/1)
    /// Spoofing state (reflects current navigation epoch only):
    ///   0 = Unknown
    ///   1 = No spoofing indicated
    ///   2 = Spoofing indicated
    ///   3 = Spoofing affirmed
    uint8_t  spoofing_state;

    bool     valid;             ///< True when populated at least once
};

} // namespace gnss
