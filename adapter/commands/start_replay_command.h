#pragma once

#include "../i_gnss_command.h"
#include "../../gnss_controller/i_gnss_controller.h"

namespace gnss
{

// ─── start_replay_command ─────────────────────────────────────────────────────
//
// Switches all GNSS receivers from the live I/O port (UART/I2C) to a
// pre-recorded binary replay file.  The file path is derived from
// DEFAULT_UBX_RECORD_PATH suffixed with the receiver index, matching
// the naming convention used by start_record().
//
// Each receiver is terminated, its underlying port is swapped to a
// read-only looping file_port, then re-initialised and started.
// The shared transport on the control port remains intact but will return
// false on write attempts while the UART fd is closed.
//
// Returns false if replay is already active or no replay file can be opened.
//
// Command string: "start_replay"

class start_replay_command final : public gnss::i_gnss_command
{
public:
    explicit start_replay_command(std::shared_ptr<gnss::i_gnss_controller> ctrl)
        : ctrl_(std::move(ctrl))
    {
    }

    bool execute() override
    {
        return ctrl_->start_replay();
    }

private:
    std::shared_ptr<gnss::i_gnss_controller> ctrl_;
};

} // namespace gnss
