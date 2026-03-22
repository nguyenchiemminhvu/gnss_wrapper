#pragma once

#include <string>

namespace gnss
{

// ─── i_gpio_pin ───────────────────────────────────────────────────────────────
//
// Abstraction for a single GPIO pin that the host BSP exposes to user-space
// (typically via a sysfs /sys/class/gpio/gpioN/value entry or a custom virtual
// file written by the BSP layer).
//
// Implementations must be thread-safe: the same pin object may be driven from
// both the command thread and a hardware-interrupt callback.
//
// Polarity convention:
//   write_value(true)  → drive pin HIGH (logic 1)
//   write_value(false) → drive pin LOW  (logic 0)
//
// The caller is responsible for the correct active-low / active-high mapping
// at the call site (e.g. RESET_N is active-low, so write_value(false) asserts
// the reset).

class i_gpio_pin
{
public:
    virtual ~i_gpio_pin() = default;

    // ── Control ───────────────────────────────────────────────────────────────

    /// Drive the pin to the given logic level.
    /// @param high  true → HIGH, false → LOW
    /// @return      true on success, false if the underlying I/O fails.
    virtual bool write_value(bool high) = 0;

    /// Read the current logic level of the pin.
    /// @param out_high  Populated with true (HIGH) or false (LOW) on success.
    /// @return          true on success, false if the underlying I/O fails or
    ///                  the pin is write-only.
    virtual bool read_value(bool& out_high) const = 0;

    // ── Metadata ──────────────────────────────────────────────────────────────

    /// Human-readable identifier for logging (e.g. "RESET_N", "SAFEBOOT_N").
    virtual const std::string& name() const = 0;
};

} // namespace gnss
