#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>

namespace gnss
{

class cold_start_command final : public i_gnss_command
{
public:
    explicit cold_start_command(std::shared_ptr<i_gnss_controller> ctrl) : ctrl_(std::move(ctrl)) {}
    bool execute() override { return ctrl_->cold_start(); }

private:
    std::shared_ptr<i_gnss_controller> ctrl_;
};

} // namespace gnss
