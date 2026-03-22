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
#include "commands/hot_stop_command.h"
#include "commands/warm_stop_command.h"
#include "commands/cold_stop_command.h"
#include "commands/hot_start_command.h"
#include "commands/warm_start_command.h"
#include "commands/cold_start_command.h"
#include "commands/hot_reset_command.h"
#include "commands/warm_reset_command.h"
#include "commands/cold_reset_command.h"
#include "commands/get_version_command.h"
#include "commands/poll_config_command.h"
#include "commands/sync_config_command.h"
#include "commands/query_datum_command.h"
#include "commands/set_datum_command.h"
#include "commands/reset_datum_command.h"
#include "commands/start_record_command.h"
#include "commands/stop_record_command.h"
#include "commands/start_replay_command.h"
#include "commands/stop_replay_command.h"

#include <utility>

namespace gnss
{

// ─── Constructors ──────────────────────────────────────────────────────────

control_adapter::control_adapter(std::vector<std::shared_ptr<io::i_io_config>> configs)
    : controller_(std::make_shared<gnss::gnss_controller>(std::move(configs)))
{
    build_command_table();
}

control_adapter::control_adapter(std::shared_ptr<io::i_io_config> config)
    : control_adapter(std::vector<std::shared_ptr<io::i_io_config>>{std::move(config)})
{
}

control_adapter::control_adapter(const io::uart_config& config)
    : control_adapter(std::make_shared<io::uart_config>(config))
{
}

// ─── get_ubx_database ──────────────────────────────────────────────────────────────

std::shared_ptr<gnss::ubx_database_wrapper> control_adapter::get_ubx_database() const
{
    return controller_->get_ubx_database();
}

#ifdef NMEA_PARSER_ENABLED
std::shared_ptr<gnss::nmea_database_wrapper> control_adapter::get_nmea_database() const
{
    return controller_->get_nmea_database();
}
#endif

// ─── build_command_table ───────────────────────────────────────────────────────
//
// Registers one factory lambda per supported command.
// To add a new command: insert a single entry here; no other code needs to change.

void control_adapter::build_command_table()
{
    // ── Receiver lifecycle ────────────────────────────────────────────────────
    command_table_[gnss::CMD_INIT] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::init_command>(controller_);
    };
    command_table_[gnss::CMD_START] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::start_command>(controller_);
    };
    command_table_[gnss::CMD_STOP] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::stop_command>(controller_);
    };
    command_table_[gnss::CMD_TERMINATE] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::terminate_command>(controller_);
    };
    command_table_[gnss::CMD_CHECK_RUNNING] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::check_running_command>(controller_);
    };

    // ── GNSS chip commands ────────────────────────────────────────────────────
    command_table_[gnss::CMD_BACKUP] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::backup_command>(controller_);
    };
    command_table_[gnss::CMD_CLEAR_BACKUP] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::clear_backup_command>(controller_);
    };
    command_table_[gnss::CMD_HOT_STOP] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::hot_stop_command>(controller_);
    };
    command_table_[gnss::CMD_WARM_STOP] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::warm_stop_command>(controller_);
    };
    command_table_[gnss::CMD_COLD_STOP] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::cold_stop_command>(controller_);
    };
    command_table_[gnss::CMD_HOT_START] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::hot_start_command>(controller_);
    };
    command_table_[gnss::CMD_WARM_START] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::warm_start_command>(controller_);
    };
    command_table_[gnss::CMD_COLD_START] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::cold_start_command>(controller_);
    };
    command_table_[gnss::CMD_HOT_RESET] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::hot_reset_command>(controller_);
    };
    command_table_[gnss::CMD_WARM_RESET] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::warm_reset_command>(controller_);
    };
    command_table_[gnss::CMD_COLD_RESET] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::cold_reset_command>(controller_);
    };
    command_table_[gnss::CMD_GET_VERSION] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::get_version_command>(controller_);
    };

    // ── Config sync: "poll_config [/path/to/config.ini]" ─────────────────────
    command_table_[gnss::CMD_POLL_CONFIG] = [this](const gnss::parsed_command& p) -> std::unique_ptr<gnss::i_gnss_command> {
        std::string path = p.args.empty() ? DEFAULT_CONFIG_INI_PATH : p.args[0];
        return std::make_unique<gnss::poll_config_command>(controller_, std::move(path));
    };
    command_table_[gnss::CMD_SYNC_CONFIG] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::sync_config_command>(controller_);
    };
    command_table_[gnss::CMD_QUERY_DATUM] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::query_datum_command>(controller_);
    };
    command_table_[gnss::CMD_SET_DATUM] = [this](const gnss::parsed_command& p) -> std::unique_ptr<gnss::i_gnss_command> {
        const uint8_t id = p.args.empty() ? 0u : static_cast<uint8_t>(std::stoul(p.args[0]));
        return std::make_unique<gnss::set_datum_command>(controller_, id);
    };
    command_table_[gnss::CMD_RESET_DATUM] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::reset_datum_command>(controller_);
    };

    // ── Record / replay ─────────────────────────────────────────────────────
    // "start_record" — open record file, install raw-UBX write callback.
    command_table_[gnss::CMD_START_RECORD] = [this](const gnss::parsed_command& p) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::start_record_command>(controller_);
    };
    // "stop_record" — remove record callback, close record file.
    command_table_[gnss::CMD_STOP_RECORD] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::stop_record_command>(controller_);
    };
    // "start_replay" — switch all receivers to replay from pre-recorded file.
    command_table_[gnss::CMD_START_REPLAY] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::start_replay_command>(controller_);
    };
    // "stop_replay" — restore all receivers to live I/O input.
    command_table_[gnss::CMD_STOP_REPLAY] = [this](const gnss::parsed_command&) -> std::unique_ptr<gnss::i_gnss_command> {
        return std::make_unique<gnss::stop_replay_command>(controller_);
    };
}

// ─── create_command (table dispatch) ──────────────────────────────────────────

std::unique_ptr<gnss::i_gnss_command> control_adapter::create_command(const gnss::parsed_command& p)
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
    const gnss::parsed_command p = tokenize(cmd);
    std::unique_ptr<gnss::i_gnss_command> command = create_command(p);
    if (!command)
    {
        return false;
    }
    return command->execute();
}

} // namespace gnss
