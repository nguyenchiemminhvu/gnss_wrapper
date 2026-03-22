#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>

namespace gnss
{

class warm_reset_command final : public gnss::i_gnss_command
{
public:
    explicit warm_reset_command(std::shared_ptr<gnss::i_gnss_controller> ctrl) : ctrl_(std::move(ctrl)) {}
    bool execute() override { return ctrl_->warm_reset(); }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
