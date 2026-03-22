#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>

namespace gnss
{

// ─── stop_record_command ──────────────────────────────────────────────────────
//
// Removes the recording callback from all UART receivers and closes the record
// file.  No-op if recording is not currently active.
//
// Command string: "stop_record"

class stop_record_command final : public gnss::i_gnss_command
{
public:
    explicit stop_record_command(std::shared_ptr<gnss::i_gnss_controller> ctrl)
        : ctrl_(std::move(ctrl))
    {
    }

    bool execute() override
    {
        return ctrl_->stop_record();
    }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
