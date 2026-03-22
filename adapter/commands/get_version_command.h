#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>

namespace gnss
{

class get_version_command final : public i_gnss_command
{
public:
    explicit get_version_command(std::shared_ptr<i_gnss_controller> ctrl) : ctrl_(std::move(ctrl)) {}
    bool execute() override { return ctrl_->get_version(); }

private:
    std::shared_ptr<i_gnss_controller> ctrl_;
};

} // namespace gnss
