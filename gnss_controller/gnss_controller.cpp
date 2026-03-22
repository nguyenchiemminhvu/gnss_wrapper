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

#include <cstdio>
#include <stdexcept>
#include <string>

#include "logging.h"

namespace gnss
{
// ─── Constructors / destructor ───────────────────────────────────────────────────────

gnss_controller::gnss_controller(std::vector<serial::serial_config> configs)
    : configs_(std::move(configs))
    , ubx_database_(std::make_shared<gnss::ubx_database_wrapper>())
#ifdef NMEA_PARSER_ENABLED
    , nmea_database_(std::make_shared<gnss::nmea_database_wrapper>())
#endif
    , ini_provider_(std::make_shared<gnss::ini_config_provider_impl>())
    , config_repo_(std::make_shared<ubx::config::ubx_config_repository>())
{
    if (configs_.empty())
    {
        LOG_ERROR("[gnss_controller] at least one serial_config is required");
        throw std::invalid_argument(
            "[gnss_controller] at least one serial_config is required");
    }

    // Create one port + receiver per configured UART.
    for (const auto& cfg : configs_)
    {
        (void)cfg;
        auto port = std::make_shared<serial::serial_port>();
        ports_.push_back(port);
        receivers_.push_back(std::make_shared<gnss::gnss_receiver>(port));
    }

    // Control transport is always bound to the first (control) port.
    transport_ = std::make_shared<gnss::serial_transport_impl>(ports_[0]);

    config_manager_ = std::make_unique<ubx::config::ubx_config_manager>(
        transport_,
        ini_provider_,
        config_repo_);
}

gnss_controller::gnss_controller(const serial::serial_config& config)
    : gnss_controller(std::vector<serial::serial_config>{config})
{
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

    // All receivers share the same database.  register_with_parser() is
    // called per receiver in start() via setup_ubx(), so there is no duplicate
    // handler registration here.
    for (auto& r : receivers_)
    {
        LOG_INFO("[gnss_controller] wiring receiver to shared UBX database");
        r->setup_ubx(ubx_database_);
    }

#ifdef NMEA_PARSER_ENABLED
    // Wire each receiver to the shared NMEA database.
    for (auto& r : receivers_)
    {
        LOG_INFO("[gnss_controller] wiring receiver to shared NMEA database");
        r->setup_nmea(nmea_database_);
    }
#endif

    // CFG-VALGET and MON-VER responses travel back only on the control UART
    // (receivers_[0]).  Installing these decoders on secondary receivers would
    // register stubs that never fire, but we omit them to keep each
    // receiver's responsibility minimal (SRP).
    receivers_[0]->set_extra_parser_setup(
        [this](ubx::parser::ubx_decoder_registry& registry) {
            registry.register_decoder(
                std::make_unique<ubx::parser::mon_ver_decoder>(
                    [this](const ubx::parser::ubx_mon_ver& decoded) {
                        LOG_INFO("[gnss_controller] received MON-VER response: SW=", decoded.sw_version, ", HW=", decoded.hw_version);
                        ubx_database_->data_updated.emit(gnss::buffer_type::chip_version);
                    }));

            registry.register_decoder(
                std::make_unique<ubx::parser::cfg_valget_decoder>(
                    [this](const ubx::parser::ubx_cfg_valget& decoded) {
                        LOG_INFO("[gnss_controller] received CFG-VALGET response with ", decoded.entries.size(), " entries");
                        // Step 3 – store decoded VALGET entries in the repository.
                        config_manager_->on_valget_response(decoded);

                        // Step 4 – diff repository vs. defaults, send VALSET for
                        // any mismatches, then signal sync completion.
                        if (config_manager_->apply_pending_sync())
                        {
                            LOG_INFO("[gnss_controller] emitting data_updated signal for default_config_applied");
                            ubx_database_->data_updated.emit(
                                gnss::buffer_type::default_config_applied);
                        }
                    }));
        });

    // Initialise all receivers.  A failure on any receiver is reported but
    // the remaining ones are still attempted (partial-failure policy).
    bool all_ok = true;
    for (std::size_t i = 0; i < receivers_.size(); ++i)
    {
        if (!receivers_[i]->init(configs_[i]))
        {
            LOG_ERROR("[gnss_controller] receiver ,", i, " failed to initialize on '", configs_[i].device_path, "'");
            all_ok = false;
        }
        else
        {
            LOG_INFO("[gnss_controller] receiver ,", i, " initialized on '", configs_[i].device_path, "'");
        }
    }
    return all_ok;
}

bool gnss_controller::start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);

    bool all_ok = true;
    for (std::size_t i = 0; i < receivers_.size(); ++i)
    {
        if (!receivers_[i]->start())
        {
            LOG_ERROR("[gnss_controller] receiver ,", i, " failed to start");
            all_ok = false;
        }
        else
        {
            LOG_INFO("[gnss_controller] receiver ,", i, " started");
        }
    }
    return all_ok;
}

void gnss_controller::stop()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    for (auto& r : receivers_)
    {
        LOG_INFO("[gnss_controller] stopping receiver");
        r->stop();
    }
}

void gnss_controller::terminate()
{
    // stop() acquires and releases cmd_mutex_ before we acquire it below.
    stop();

    std::lock_guard<std::mutex> lk(cmd_mutex_);
    for (auto& r : receivers_)
    {
        LOG_INFO("[gnss_controller] terminating receiver");
        r->terminate();
    }
}

bool gnss_controller::is_running() const
{
    // The control receiver (index 0) is the authoritative liveness indicator:
    // if the control path is up, the system is considered running.
    return receivers_[0]->is_running();
}

// ─── Chip commands ─────────────────────────────────────────────────────────────

bool gnss_controller::hot_stop()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    serial::serial_wrapper w(ports_[0]);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_hot_stop();
    return transport_->send(frame);
}

bool gnss_controller::warm_stop()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    serial::serial_wrapper w(ports_[0]);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_warm_stop();
    return transport_->send(frame);
}

bool gnss_controller::cold_stop()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    serial::serial_wrapper w(ports_[0]);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_cold_stop();
    return transport_->send(frame);
}

bool gnss_controller::hot_start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    serial::serial_wrapper w(ports_[0]);
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
    serial::serial_wrapper w(ports_[0]);
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
    serial::serial_wrapper w(ports_[0]);
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
    LOG_INFO("[gnss_controller] starting config sync with INI file: ", ini_path);

    std::lock_guard<std::mutex> lk(cmd_mutex_);
    bool sync_ret = false;
    try
    {
        sync_ret = config_manager_->start_sync(ini_path);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("[gnss_controller] config sync failed with exception: ", e.what());
        // Catch exceptions from config sync and return false to indicate failure.
        // The caller can check logs for details of the exception.
    }
    return sync_ret;
}

// ─── Data access ───────────────────────────────────────────────────────────────

std::shared_ptr<gnss::ubx_database_wrapper> gnss_controller::get_ubx_database() const
{
    return ubx_database_;
}

#ifdef NMEA_PARSER_ENABLED
std::shared_ptr<gnss::nmea_database_wrapper> gnss_controller::get_nmea_database() const
{
    return nmea_database_;
}
#endif

// ─── Private helpers ───────────────────────────────────────────────────────────

bool gnss_controller::send_poll(uint8_t msg_class, uint8_t msg_id)
{
    serial::serial_wrapper w(ports_[0]);
    if (!w.is_ready())
    {
        return false;
    }

    std::vector<uint8_t> frame =
        ubx::parser::ubx_message_builder::make_frame(msg_class, msg_id);
    return transport_->send(frame);
}

} // namespace gnss
