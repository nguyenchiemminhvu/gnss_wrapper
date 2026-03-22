#pragma once

#include <cstdint>

namespace gnss
{

static constexpr uint8_t GNSS_MAX_MON_IO_PORTS    = 8u;
static constexpr uint8_t GNSS_MON_TXBUF_TARGETS   = 6u;
static constexpr uint8_t GNSS_MON_SPAN_RF_BLOCKS  = 4u;

// ─── MON-IO ────────────────────────────────────────────────────────────────

/// Per-port I/O statistics populated from UBX-MON-IO.
struct mon_io_port_record
{
    uint32_t rx_bytes;      ///< Total bytes received
    uint32_t tx_bytes;      ///< Total bytes transmitted
    uint16_t parity_errs;   ///< Parity errors
    uint16_t framing_errs;  ///< Framing errors
    uint16_t overrun_errs;  ///< Overrun errors
    uint16_t break_cond;    ///< Break conditions
    uint8_t  rx_busy;       ///< Receive busy flag
    uint8_t  tx_busy;       ///< Transmit busy flag
};

// ─── MON-TXBUF ─────────────────────────────────────────────────────────────

/// TX buffer statistics populated from UBX-MON-TXBUF.
/// Targets: [0]=UART1, [1]=UART2, [2]=USB, [3]=SPI, [4]=DDC/I2C, [5]=reserved
struct mon_txbuf_data
{
    uint16_t pending[GNSS_MON_TXBUF_TARGETS];    ///< Bytes pending per target
    uint8_t  usage[GNSS_MON_TXBUF_TARGETS];      ///< Current buffer usage [%] per target
    uint8_t  peak_usage[GNSS_MON_TXBUF_TARGETS]; ///< Peak buffer usage [%] per target
    uint8_t  t_used;                              ///< Total TX buffer usage [%]
    uint8_t  t_peak;                              ///< Maximum TX buffer usage [%]
    uint8_t  errors;                              ///< Error flags (mem/alloc)
};

// ─── MON-SPAN ──────────────────────────────────────────────────────────────

/// Spectrum monitor data populated from UBX-MON-SPAN.
/// Only the PGA value per RF block is retained (spectrum bins are not buffered).
struct mon_span_data
{
    uint8_t num_rf_blocks;                       ///< Number of valid RF block entries
    uint8_t pga[GNSS_MON_SPAN_RF_BLOCKS];        ///< Programmable gain amplifier [dB] per RF block
};

// ─── Combined monitor buffer ───────────────────────────────────────────────

/// Combined buffer for all receiver monitor data (UBX-MON-IO + UBX-MON-TXBUF).
struct monitor_data
{
    // ── MON-IO ───────────────────────────────────────────────────────────────
    uint8_t            num_ports;                         ///< Number of valid port entries
    mon_io_port_record ports[GNSS_MAX_MON_IO_PORTS];      ///< Per-port stats [0..num_ports)
    bool               io_valid;                          ///< True when MON-IO has been received

    // ── MON-TXBUF ────────────────────────────────────────────────────────────
    mon_txbuf_data     txbuf;                             ///< TX buffer statistics
    bool               txbuf_valid;                       ///< True when MON-TXBUF has been received

    // ── MON-SPAN ─────────────────────────────────────────────────────────────
    mon_span_data      span;                              ///< Spectrum/PGA statistics per RF block
    bool               span_valid;                        ///< True when MON-SPAN has been received

    bool               valid;                             ///< True when any of io_valid, txbuf_valid, or span_valid
};

} // namespace gnss
