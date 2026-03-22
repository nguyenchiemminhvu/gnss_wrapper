#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

#include <memory>
#include <utility>

namespace gnss
{

/// Command: "query_datum"
///
/// Sends a CFG-VALGET poll for all NAVSPG-USRDAT configuration keys.
/// When the VALGET response is decoded the datum service resolves the active
/// coordinate datum and emits data_updated(buffer_type::datum_config).

class query_datum_command final : public gnss::i_gnss_command
{
public:
    explicit query_datum_command(std::shared_ptr<gnss::i_gnss_controller> ctrl)
        : ctrl_(std::move(ctrl)) {}

    bool execute() override { return ctrl_->query_datum(); }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
