#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

namespace gnss
{

// ─── start_record_command ─────────────────────────────────────────────────────
//
// Opens a binary record file and installs a raw-UBX callback on all active
// UART receivers so that every unregistered UBX frame is written to the file.
//
// Command string: "start_record"

class start_record_command final : public gnss::i_gnss_command
{
public:
    explicit start_record_command(std::shared_ptr<gnss::i_gnss_controller> ctrl)
        : ctrl_(std::move(ctrl))
    {
    }

    bool execute() override
    {
        return ctrl_->start_record();
    }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
