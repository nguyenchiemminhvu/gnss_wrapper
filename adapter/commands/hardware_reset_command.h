#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>

namespace gnss
{

// ─── hardware_reset_command ───────────────────────────────────────────────────
//
// Triggers a hardware reset of the GNSS chip by asserting the RESET_N GPIO
// (active-low) via the controller's hardware_reset() method.
//
// Per the ZED-F9P integration manual (§3.9.2), RESET_N must be held low for
// at least 100 ms to guarantee a cold-start.  The implementation of
// gnss_controller::hardware_reset() is responsible for the correct timing.
//
// The command returns true only if the GPIO pulse was successfully delivered.
// If no RESET_N GPIO pin has been configured on the controller the call
// returns false immediately (no-op, no crash).

class hardware_reset_command final : public gnss::i_gnss_command
{
public:
    explicit hardware_reset_command(std::shared_ptr<gnss::i_gnss_controller> ctrl)
        : ctrl_(std::move(ctrl))
    {
    }

    bool execute() override
    {
        return ctrl_->hardware_reset();
    }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
