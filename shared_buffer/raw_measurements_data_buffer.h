#pragma once

#include <cstdint>

namespace gnss
{

static constexpr uint8_t GNSS_MAX_MEASX_SV_COUNT = 64u;

/// Per-satellite raw measurement record populated from UBX-RXM-MEASX.
struct raw_sv_record
{
    uint8_t  gnss_id;             ///< GNSS identifier
    uint8_t  sv_id;               ///< Satellite identifier
    uint8_t  cno;                 ///< Carrier-to-noise ratio [dBHz]
    uint8_t  mpath_indic;         ///< Multipath indicator (0=no, 1=possible, 2=probable)
    int32_t  doppler_ms;          ///< Doppler velocity  [m/s × 0.04]
    int32_t  doppler_hz;          ///< Doppler frequency [Hz × 0.2]
    uint16_t whole_chips;         ///< Whole code phase [chips]
    uint16_t frac_chips;          ///< Fractional code phase [chips/1024]
    uint32_t code_phase;          ///< Code phase measurement [ms/2^21]
    uint8_t  int_code_phase;      ///< Integer part of code phase [ms]
    uint8_t  pseu_range_rms_err;  ///< Pseudorange RMS error index
};

/// Populated from UBX-RXM-MEASX (low-priority epoch commit).
struct raw_measurements_data
{
    // ── Header TOW values ────────────────────────────────────────────────────
    uint32_t gps_tow_ms;   ///< GPS   time of week [ms]
    uint32_t glo_tow_ms;   ///< GLONASS time of week [ms]
    uint32_t bds_tow_ms;   ///< BeiDou time of week [ms]
    uint32_t qzss_tow_ms;  ///< QZSS  time of week [ms]

    uint16_t gps_tow_acc;  ///< GPS   TOW accuracy [ms/16]
    uint16_t glo_tow_acc;  ///< GLONASS TOW accuracy [ms/16]
    uint16_t bds_tow_acc;  ///< BeiDou TOW accuracy [ms/16]
    uint16_t qzss_tow_acc; ///< QZSS  TOW accuracy [ms/16]

    uint8_t  flags;        ///< Flags byte

    // ── Per-SV measurements ──────────────────────────────────────────────────
    uint8_t        num_svs;                             ///< Number of valid entries in svs[]
    raw_sv_record  svs[GNSS_MAX_MEASX_SV_COUNT];        ///< Per-SV records [0..num_svs)

    bool           valid;  ///< True when populated at least once
};

} // namespace gnss
