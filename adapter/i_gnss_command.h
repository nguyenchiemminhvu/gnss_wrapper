#pragma once

namespace gnss
{

// ─── i_gnss_command ────────────────────────────────────────────────────────────
//
// Command interface (Command pattern).
// Each concrete command encapsulates a single operation on gnss_controller.
// control_adapter::create_command() acts as the factory that maps a command
// string to the appropriate concrete command object.

struct i_gnss_command
{
    virtual ~i_gnss_command() = default;

    /// Execute the command.
    /// @return true on success, false on failure or if the operation is
    ///         not applicable in the current state.
    virtual bool execute() = 0;
};

} // namespace gnss
