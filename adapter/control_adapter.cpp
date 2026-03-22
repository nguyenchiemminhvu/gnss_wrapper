#include "control_adapter.h"
#include "command_tokenizer.h"
#include "global_constants.h"

#include "commands/init_command.h"
#include "commands/start_command.h"
#include "commands/stop_command.h"
#include "commands/terminate_command.h"
#include "commands/check_running_command.h"
#include "commands/backup_command.h"
#include "commands/clear_backup_command.h"
#include "commands/hot_start_command.h"
#include "commands/warm_start_command.h"
#include "commands/cold_start_command.h"
#include "commands/get_version_command.h"
#include "commands/sync_config_command.h"

#include <utility>

namespace gnss
{

// ─── Constructor ───────────────────────────────────────────────────────────────

control_adapter::control_adapter(const serial::serial_config& config)
    : controller_(std::make_shared<gnss_controller>(config))
{
    build_command_table();
}

// ─── get_database ──────────────────────────────────────────────────────────────

std::shared_ptr<ubx_database_wrapper> control_adapter::get_database() const
{
    return controller_->get_database();
}

// ─── build_command_table ───────────────────────────────────────────────────────
//
// Registers one factory lambda per supported command.
// To add a new command: insert a single entry here; no other code needs to change.

void control_adapter::build_command_table()
{
    // ── Receiver lifecycle ────────────────────────────────────────────────────
    command_table_[CMD_INIT] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<init_command>(controller_);
    };
    command_table_[CMD_START] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<start_command>(controller_);
    };
    command_table_[CMD_STOP] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<stop_command>(controller_);
    };
    command_table_[CMD_TERMINATE] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<terminate_command>(controller_);
    };
    command_table_[CMD_CHECK_RUNNING] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<check_running_command>(controller_);
    };

    // ── GNSS chip commands ────────────────────────────────────────────────────
    command_table_[CMD_BACKUP] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<backup_command>(controller_);
    };
    command_table_[CMD_CLEAR_BACKUP] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<clear_backup_command>(controller_);
    };
    command_table_[CMD_HOT_START] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<hot_start_command>(controller_);
    };
    command_table_[CMD_WARM_START] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<warm_start_command>(controller_);
    };
    command_table_[CMD_COLD_START] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<cold_start_command>(controller_);
    };
    command_table_[CMD_GET_VERSION] = [this](const parsed_command&) -> std::unique_ptr<i_gnss_command> {
        return std::make_unique<get_version_command>(controller_);
    };

    // ── Config sync: "sync_config [/path/to/config.ini]" ─────────────────────
    command_table_[CMD_SYNC_CONFIG] = [this](const parsed_command& p) -> std::unique_ptr<i_gnss_command> {
        std::string path = p.args.empty() ? DEFAULT_CONFIG_INI_PATH : p.args[0];
        return std::make_unique<sync_config_command>(controller_, std::move(path));
    };
}

// ─── create_command (table dispatch) ──────────────────────────────────────────

std::unique_ptr<i_gnss_command> control_adapter::create_command(const parsed_command& p)
{
    const auto it = command_table_.find(p.name);
    if (it == command_table_.end())
    {
        return nullptr; // Unrecognised command.
    }
    return it->second(p);
}

// ─── execute_command ───────────────────────────────────────────────────────────

bool control_adapter::execute_command(const std::string& cmd)
{
    const parsed_command p = tokenize(cmd);
    std::unique_ptr<i_gnss_command> command = create_command(p);
    if (!command)
    {
        return false;
    }
    return command->execute();
}

} // namespace gnss
