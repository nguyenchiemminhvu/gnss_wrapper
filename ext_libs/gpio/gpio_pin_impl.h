#pragma once

#include "i_gpio_pin.h"

#include <string>
#include <mutex>

namespace gnss
{

// ─── gpio_pin_impl ────────────────────────────────────────────────────────────
//
// Concrete GPIO pin backed by a BSP-exported filesystem path.
//
// The BSP (kernel driver or device-tree overlay) is expected to expose each
// GPIO as a plain file where:
//   • Writing "1\n" drives the pin HIGH.
//   • Writing "0\n" drives the pin LOW.
//   • Reading the file yields "0\n" or "1\n" for the current level.
//
// This matches both the Linux sysfs GPIO ABI
//   (/sys/class/gpio/gpioN/value)
// and custom virtual files published by platform BSP layers.
//
// Thread-safety: all public methods are serialised by an internal mutex so
// the same gpio_pin_impl may be shared across threads.
//
// Example – RESET_N on a ZED-F9P:
//   auto reset_pin = std::make_shared<gnss::gpio_pin_impl>(
//       gnss::DEFAULT_RESET_N_GPIO_PATH, "RESET_N");
//
//   reset_pin->write_value(false); // assert reset (active-low)
//   std::this_thread::sleep_for(std::chrono::milliseconds(150));
//   reset_pin->write_value(true);  // deassert reset

class gpio_pin_impl final : public i_gpio_pin
{
public:
    /// @param gpio_path  Absolute filesystem path to the GPIO value file.
    ///                   e.g. "/sys/class/gpio/gpio42/value"
    /// @param pin_name   Human-readable name used in log messages.
    explicit gpio_pin_impl(std::string gpio_path, std::string pin_name = "gpio");

    ~gpio_pin_impl() override = default;

    // Non-copyable (each instance owns one physical pin path).
    gpio_pin_impl(const gpio_pin_impl&)            = delete;
    gpio_pin_impl& operator=(const gpio_pin_impl&) = delete;

    // ── i_gpio_pin ────────────────────────────────────────────────────────────
    bool write_value(bool high) override;
    bool read_value(bool& out_high) const override;
    const std::string& name() const override;

private:
    const std::string   gpio_path_;
    const std::string   name_;
    mutable std::mutex  mutex_;
};

} // namespace gnss
