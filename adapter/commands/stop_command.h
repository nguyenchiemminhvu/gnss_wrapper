#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>

namespace gnss
{

class stop_command final : public gnss::i_gnss_command
{
public:
    explicit stop_command(std::shared_ptr<gnss::i_gnss_controller> ctrl) : ctrl_(std::move(ctrl)) {}
    bool execute() override { ctrl_->stop(); return true; }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
