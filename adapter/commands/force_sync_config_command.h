#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>
#include <string>
#include <utility>

namespace gnss
{

class force_sync_config_command final : public gnss::i_gnss_command
{
public:
    force_sync_config_command(std::shared_ptr<gnss::i_gnss_controller> ctrl, const std::string& ini_path)
        : ctrl_(std::move(ctrl)), ini_path_(ini_path) {}

    bool execute() override { return ctrl_->force_sync_config(ini_path_); }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
    std::string                        ini_path_;
};

} // namespace gnss
