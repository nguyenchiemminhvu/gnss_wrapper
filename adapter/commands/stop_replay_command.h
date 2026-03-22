#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

namespace gnss
{

// ─── stop_replay_command ──────────────────────────────────────────────────────
//
// Restores all GNSS receivers to the original live I/O ports that were active
// before start_replay() was called.  Each receiver is terminated, its port is
// swapped back to the saved UART/I2C port, then re-initialised and started.
//
// No-op (returns false) if replay is not currently active.
//
// Command string: "stop_replay"

class stop_replay_command final : public gnss::i_gnss_command
{
public:
    explicit stop_replay_command(std::shared_ptr<gnss::i_gnss_controller> ctrl)
        : ctrl_(std::move(ctrl))
    {
    }

    bool execute() override
    {
        return ctrl_->stop_replay();
    }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
