#include "gnss_controller.h"
#include "serial_transport_impl.h"
#include "ini_config_provider_impl.h"

#include "serial_helper/serial_wrapper.h"
#include "serial_helper/serial_port.h"
#include "ubx_parser/include/ubx_message_builder.h"
#include "ubx_parser/include/ubx_cfg_rst_builder.h"
#include "ubx_parser/include/ubx_upd_sos_builder.h"
#include "ubx_parser/include/ubx_types.h"
#include "ubx_parser/include/decoders/mon_ver_decoder.h"
#include "ubx_parser/include/decoders/cfg_valget_decoder.h"

#include <string>

namespace gnss
{
// ─── Constructor / destructor ──────────────────────────────────────────────────

gnss_controller::gnss_controller(const serial::serial_config& config)
    : config_(config)
    , port_(std::make_shared<serial::serial_port>())
    , database_(std::make_shared<ubx_database_wrapper>())
    , receiver_(std::make_shared<ubx_gnss_receiver>(port_))
    , transport_(std::make_shared<serial_transport_impl>(port_))
    , ini_provider_(std::make_shared<ini_config_provider_impl>())
    , config_repo_(std::make_shared<ubx::config::ubx_config_repository>())
{
    config_manager_ = std::make_unique<ubx::config::ubx_config_manager>(
        transport_,
        ini_provider_,
        config_repo_);
}

gnss_controller::~gnss_controller()
{
    stop();
    terminate();
}

// ─── Lifecycle ─────────────────────────────────────────────────────────────────

bool gnss_controller::init()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);

    // Wire the database adapter into the receiver's parser registry.
    receiver_->setup(database_);

    // Register cfg_valget_decoder so that UBX-CFG-VALGET responses are
    // dispatched through the typed decoder path.  The callback drives steps
    // 3 and 4 of the config-sync cycle without any raw-message scanning.
    receiver_->set_extra_parser_setup(
        [this](ubx::parser::ubx_decoder_registry& registry) {
            registry.register_decoder(
                std::make_unique<ubx::parser::mon_ver_decoder>(
                    [this](const ubx::parser::ubx_mon_ver& decoded) {
                        database_->data_updated.emit(gnss::buffer_type::chip_version);
                    }));

            registry.register_decoder(
                std::make_unique<ubx::parser::cfg_valget_decoder>(
                    [this](const ubx::parser::ubx_cfg_valget& decoded) {
                        // Step 3 – store decoded VALGET entries in the repository.
                        config_manager_->on_valget_response(decoded);

                        // Step 4 – diff repository vs. defaults, send VALSET for
                        // any mismatches, then signal sync completion.
                        if (config_manager_->apply_pending_sync())
                        {
                            database_->data_updated.emit(
                                gnss::buffer_type::default_config_applied);
                        }
                    }));
        });

    return receiver_->init(config_);
}

bool gnss_controller::start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    return receiver_->start();
}

void gnss_controller::stop()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    receiver_->stop();
}

void gnss_controller::terminate()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    receiver_->terminate();
}

bool gnss_controller::is_running() const
{
    return receiver_->is_running();
}

// ─── Chip commands ─────────────────────────────────────────────────────────────

bool gnss_controller::hot_start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    serial::serial_wrapper w(port_);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_hot_start();
    return transport_->send(frame);
}

bool gnss_controller::warm_start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    serial::serial_wrapper w(port_);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_warm_start();
    return transport_->send(frame);
}

bool gnss_controller::cold_start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    serial::serial_wrapper w(port_);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_cold_start();
    return transport_->send(frame);
}

bool gnss_controller::get_version()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    return send_poll(ubx::parser::UBX_CLASS_MON, ubx::parser::UBX_ID_MON_VER);
}

bool gnss_controller::backup()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_upd_sos_builder::build_save();
    return transport_->send(frame);
}

bool gnss_controller::clear_backup()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_upd_sos_builder::build_clear();
    return transport_->send(frame);
}

bool gnss_controller::sync_config(const std::string& ini_path)
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    bool sync_ret = false;
    try
    {
        sync_ret = config_manager_->start_sync(ini_path);
    }
    catch (const std::exception& e)
    {
        // Catch exceptions from config sync and return false to indicate failure.
        // The caller can check logs for details of the exception.
    }
    return sync_ret;
}

// ─── Data access ───────────────────────────────────────────────────────────────

std::shared_ptr<ubx_database_wrapper> gnss_controller::get_database() const
{
    return database_;
}

// ─── Private helpers ───────────────────────────────────────────────────────────

bool gnss_controller::send_poll(uint8_t msg_class, uint8_t msg_id)
{
    serial::serial_wrapper w(port_);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame =
        ubx::parser::ubx_message_builder::make_frame(msg_class, msg_id);
    return transport_->send(frame);
}

} // namespace gnss
