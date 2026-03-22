#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>
#include <string>
#include <utility>

namespace gnss
{

class poll_config_command final : public gnss::i_gnss_command
{
public:
    poll_config_command(std::shared_ptr<gnss::i_gnss_controller> ctrl, std::string ini_path)
        : ctrl_(std::move(ctrl)), ini_path_(std::move(ini_path)) {}

    bool execute() override { return ctrl_->poll_config(ini_path_); }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
    std::string                        ini_path_;
};

} // namespace gnss
