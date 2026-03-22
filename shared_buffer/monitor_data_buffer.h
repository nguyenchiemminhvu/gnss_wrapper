#pragma once

#include <cstdint>

namespace gnss
{

static constexpr uint8_t GNSS_MAX_MON_IO_PORTS    = 8u;
static constexpr uint8_t GNSS_MON_TXBUF_TARGETS   = 6u;

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

    bool               valid;                             ///< True when either io_valid or txbuf_valid
};

} // namespace gnss
