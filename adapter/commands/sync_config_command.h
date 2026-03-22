#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>
#include <string>
#include <utility>

namespace gnss
{

class sync_config_command final : public gnss::i_gnss_command
{
public:
    sync_config_command(std::shared_ptr<gnss::i_gnss_controller> ctrl)
        : ctrl_(std::move(ctrl)) {}

    bool execute() override { return ctrl_->sync_config(); }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
    std::string                        ini_path_;
};

} // namespace gnss
