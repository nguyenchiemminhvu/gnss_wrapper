#include "gnss_controller.h"
#include "serial_transport_impl.h"
#include "ini_config_provider_impl.h"

#include "io_helper/io_port_factory.h"
#include "io_helper/file/file_config.h"
#include "ubx_parser/include/ubx_message_builder.h"
#include "ubx_parser/include/ubx_cfg_rst_builder.h"
#include "ubx_parser/include/ubx_upd_sos_builder.h"
#include "ubx_parser/include/ubx_types.h"
#include "ubx_parser/include/decoders/ack_ack_decoder.h"
#include "ubx_parser/include/decoders/ack_nak_decoder.h"
#include "ubx_parser/include/decoders/mon_ver_decoder.h"
#include "ubx_parser/include/decoders/cfg_valget_decoder.h"
#include "ubx_parser/include/decoders/upd_sos_decoder.h"
#include "ubx_parser/include/decoders/inf_decoder.h"
#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <iomanip>

#include "logging.h"

namespace gnss
{
// ─── Constructors / destructor ───────────────────────────────────────────────────────

gnss_controller::gnss_controller(std::vector<std::shared_ptr<io::i_io_config>> configs)
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
        LOG_ERROR("[gnss_controller] at least one i_io_config is required");
        throw std::invalid_argument(
            "[gnss_controller] at least one i_io_config is required");
    }

    // Create one port + receiver per configured channel.
    for (const auto& cfg : configs_)
    {
        auto port = io::io_port_factory::create(*cfg);
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

gnss_controller::gnss_controller(std::shared_ptr<io::i_io_config> config)
    : gnss_controller(std::vector<std::shared_ptr<io::i_io_config>>{std::move(config)})
{}

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

    receivers_[0]->set_extra_parser_setup(
        [this](ubx::parser::ubx_decoder_registry& registry) {
            // Register decoders for UBX messages that the control receiver is expected to receive and that require custom handling beyond populating the database.  These include:
            //   - UBX-INF-* messages, which are logged with appropriate severity.
            //   - ACK-ACK and ACK-NAK messages, which trigger transport-level ACK handling to unblock pending synchronous commands.
            //   - MON-VER messages, which trigger an ACK and update the chip version in the database.
            //   - CFG-VALGET responses, which trigger an ACK and update the config repository with the decoded values.
            //   - UBX-UPD-SOS responses, which trigger an ACK and update the database with the backup operation result.

            registry.register_decoder(
                std::make_unique<ubx::parser::inf_decoder>(
                    ubx::parser::UBX_ID_INF_ERROR,
                    [this](const ubx::parser::ubx_inf& decoded) {
                        LOG_ERROR("[gnss_controller] received INF-ERROR: ", decoded.text);
                    }));
            registry.register_decoder(
                std::make_unique<ubx::parser::inf_decoder>(
                    ubx::parser::UBX_ID_INF_WARNING,
                    [this](const ubx::parser::ubx_inf& decoded) {
                        LOG_WARNING("[gnss_controller] received INF-WARNING: ", decoded.text);
                    }));
            registry.register_decoder(
                std::make_unique<ubx::parser::inf_decoder>(
                    ubx::parser::UBX_ID_INF_DEBUG,
                    [this](const ubx::parser::ubx_inf& decoded) {
                        LOG_DEBUG("[gnss_controller] received INF-DEBUG: ", decoded.text);
                    }));
            registry.register_decoder(
                std::make_unique<ubx::parser::inf_decoder>(
                    ubx::parser::UBX_ID_INF_NOTICE,
                    [this](const ubx::parser::ubx_inf& decoded) {
                        LOG_INFO("[gnss_controller] received INF-NOTICE: ", decoded.text);
                    }));
            registry.register_decoder(
                std::make_unique<ubx::parser::inf_decoder>(
                    ubx::parser::UBX_ID_INF_TEST,
                    [this](const ubx::parser::ubx_inf& decoded) {
                        LOG_INFO("[gnss_controller] received INF-TEST: ", decoded.text);
                    }));

            registry.register_decoder(
                std::make_unique<ubx::parser::ack_ack_decoder>(
                    [this](const ubx::parser::ubx_ack_ack& decoded) {
                        LOG_INFO("[gnss_controller] received ACK-ACK for class=", std::hex, std::setfill('0'), std::setw(2), static_cast<uint32_t>(decoded.cls_id), ", id=", std::setw(2), static_cast<uint32_t>(decoded.msg_id));
                        transport_->handle_ack(decoded.cls_id, decoded.msg_id, true);
                    }));

            registry.register_decoder(
                std::make_unique<ubx::parser::ack_nak_decoder>(
                    [this](const ubx::parser::ubx_ack_nak& decoded) {
                        LOG_INFO("[gnss_controller] received ACK-NAK for class=", std::hex, std::setfill('0'), std::setw(2), static_cast<uint32_t>(decoded.cls_id), ", id=", std::setw(2), static_cast<uint32_t>(decoded.msg_id));
                    }));

            registry.register_decoder(
                std::make_unique<ubx::parser::mon_ver_decoder>(
                    [this](const ubx::parser::ubx_mon_ver& decoded) {
                        LOG_INFO("[gnss_controller] received MON-VER response: SW=", decoded.sw_version, ", HW=", decoded.hw_version);
                        transport_->handle_ack(ubx::parser::UBX_CLASS_MON, ubx::parser::UBX_ID_MON_VER, true);
                        ubx_database_->data_updated.emit(gnss::buffer_type::chip_version);
                    }));

            registry.register_decoder(
                std::make_unique<ubx::parser::cfg_valget_decoder>(
                    [this](const ubx::parser::ubx_cfg_valget& decoded) {
                        LOG_INFO("[gnss_controller] received CFG-VALGET response with ", decoded.entries.size(), " entries");
                        transport_->handle_ack(ubx::parser::UBX_CLASS_CFG, ubx::parser::UBX_ID_CFG_VALGET, true);
                        // Step 3 – store decoded VALGET entries in the repository.
                        config_manager_->on_valget_response(decoded);
                        ubx_database_->data_updated.emit(gnss::buffer_type::valget_response);
                    }));

            registry.register_decoder(
                std::make_unique<ubx::parser::upd_sos_decoder>(
                    [this](const ubx::parser::ubx_upd_sos_output& decoded) {
                        if (decoded.response == ubx::parser::UPD_SOS_RESP_ACK)
                        {
                            LOG_INFO("[gnss_controller] UPD-SOS operation acknowledged by receiver");
                            transport_->handle_ack(ubx::parser::UBX_CLASS_UPD, ubx::parser::UBX_ID_UPD_SOS, true);
                            ubx_database_->data_updated.emit(gnss::buffer_type::backup_operation_completed);
                        }
                        else
                        {
                            LOG_WARNING("[gnss_controller] UPD-SOS operation not acknowledged by receiver");
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
            LOG_ERROR("[gnss_controller] receiver ,", i, " failed to initialize");
            all_ok = false;
        }
        else
        {
            LOG_INFO("[gnss_controller] receiver ,", i, " initialized");
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
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_hot_stop();
    return transport_->send_async(frame);
}

bool gnss_controller::warm_stop()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_warm_stop();
    return transport_->send_async(frame);
}

bool gnss_controller::cold_stop()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_cold_stop();
    return transport_->send_async(frame);
}

bool gnss_controller::hot_start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_hot_start();
    return transport_->send_async(frame);
}

bool gnss_controller::warm_start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_warm_start();
    return transport_->send_async(frame);
}

bool gnss_controller::cold_start()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_cold_start();
    return transport_->send_async(frame);
}

bool gnss_controller::hot_reset()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_hot_reset();
    return transport_->send_async(frame);
}

bool gnss_controller::warm_reset()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_warm_reset();
    return transport_->send_async(frame);
}

bool gnss_controller::cold_reset()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_cfg_rst_builder::build_cold_reset();
    return transport_->send_async(frame);
}

bool gnss_controller::get_version()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_message_builder::make_frame(ubx::parser::UBX_CLASS_MON, ubx::parser::UBX_ID_MON_VER);
    return transport_->send_sync(frame);
}

bool gnss_controller::backup()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_upd_sos_builder::build_save();
    return transport_->send_sync(frame);
}

bool gnss_controller::clear_backup()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    std::vector<uint8_t> frame = ubx::parser::ubx_upd_sos_builder::build_clear();
    return transport_->send_sync(frame);
}

bool gnss_controller::poll_config(const std::string& ini_path)
{
    LOG_INFO("[gnss_controller] starting config poll with INI file: ", ini_path);

    std::lock_guard<std::mutex> lk(cmd_mutex_);
    bool poll_ret = false;
    try
    {
        poll_ret = config_manager_->start_poll(ini_path);
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("[gnss_controller] config poll failed with exception: ", e.what());
        // Catch exceptions from config poll and return false to indicate failure.
        // The caller can check logs for details of the exception.
    }
    return poll_ret;
}

bool gnss_controller::sync_config()
{
    LOG_INFO("[gnss_controller] applying default config sync");

    std::lock_guard<std::mutex> lk(cmd_mutex_);

    // Step 4 – diff repository vs. defaults, send VALSET for
    // any mismatches, then signal sync completion.
    if (config_manager_->apply_pending_sync())
    {
        LOG_INFO("[gnss_controller] emitting data_updated signal for default_config_applied");
        ubx_database_->data_updated.emit(gnss::buffer_type::default_config_applied);
        return true;
    }
    return false;
}

bool gnss_controller::query_datum()
{
    LOG_INFO("[gnss_controller] starting datum config query");

    std::lock_guard<std::mutex> lk(cmd_mutex_);
    return config_manager_->query_datum(
        [this](const ubx::parser::datum_data& data) {
            LOG_INFO("[gnss_controller] datum resolved: ", data.name,
                     " usrdat=", data.usrdat_enabled);
            ubx_database_->update_datum(data);
        });
}

bool gnss_controller::set_datum(uint8_t datum_id)
{
    LOG_INFO("[gnss_controller] setting datum id=", static_cast<int>(datum_id));

    const auto id = static_cast<ubx::parser::datum_id>(datum_id);

    // For unknown or user-defined IDs there are no well-known ellipsoid
    // parameters to write.  Revert the chip to its internal default (WGS-84)
    // so the hardware is left in a deterministic state.
    if (id == ubx::parser::datum_id::unknown ||
        id == ubx::parser::datum_id::user_defined)
    {
        LOG_INFO("[gnss_controller] datum id is unknown/user_defined — resetting to WGS-84 default");
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        return config_manager_->reset_datum();
    }

    std::lock_guard<std::mutex> lk(cmd_mutex_);
    return config_manager_->set_datum_by_id(id);
}

bool gnss_controller::reset_datum()
{
    LOG_INFO("[gnss_controller] resetting datum to WGS-84 default (navspg_usrdat=false)");

    std::lock_guard<std::mutex> lk(cmd_mutex_);
    return config_manager_->reset_datum();
}

// ─── Record / replay ───────────────────────────────────────────────────────────

bool gnss_controller::start_record()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (record_active_)
    {
        LOG_WARNING("[gnss_controller] recording already active");
        return false;
    }

    if (replay_active_)
    {
        LOG_WARNING("[gnss_controller] cannot start recording while replay is active");
        return false;
    }

    // Open one record file per UART/I2C receiver and install a raw-UBX callback
    // that serialises every frame to that file.
    bool any_started = false;
    for (std::size_t i = 0; i < receivers_.size(); ++i)
    {
        const std::string path = make_record_path(i);
        auto cfg  = std::make_shared<io::file_config>(io::file_config::record(path));
        auto port = io::io_port_factory::create(*cfg);

        if (!port->open(*cfg))
        {
            LOG_ERROR("[gnss_controller] failed to open record file for receiver "
                      + std::to_string(i) + ": " + path);
            port.reset();
            record_ports_.push_back(nullptr);  // keep index alignment with receivers_
            continue;
        }

        // Build a callback that serialises each raw UBX frame to this receiver's
        // record file.  The port is captured by value (shared ownership) so the
        // lambda safely outlives any concurrent stop_record() call.
        std::shared_ptr<io::i_io_port> captured_port = port;
        ubx::parser::raw_message_callback_t record_cb =
            [captured_port](const ubx::parser::ubx_raw_message& msg)
            {
                const auto& frame = msg.frame;
                const auto& hdr   = frame.header;

                // Reconstruct the full on-wire UBX frame:
                //   sync1  sync2  class  id  len_lo  len_hi  [payload]  ck_a  ck_b
                std::vector<uint8_t> bytes;
                bytes.reserve(ubx::parser::UBX_FRAME_OVERHEAD + frame.payload.size());
                bytes.push_back(ubx::parser::UBX_SYNC_CHAR_1);
                bytes.push_back(ubx::parser::UBX_SYNC_CHAR_2);
                bytes.push_back(hdr.msg_class);
                bytes.push_back(hdr.msg_id);
                bytes.push_back(static_cast<uint8_t>(hdr.payload_length & 0xFFu));
                bytes.push_back(static_cast<uint8_t>((hdr.payload_length >> 8u) & 0xFFu));
                bytes.insert(bytes.end(), frame.payload.begin(), frame.payload.end());
                bytes.push_back(frame.ck_a);
                bytes.push_back(frame.ck_b);

                captured_port->write_bytes(bytes);
            };

        // Wire the callback via a stop/start cycle to refresh the parser slot.
        const bool was_running = receivers_[i]->is_running();
        if (was_running)
        {
            receivers_[i]->stop();
        }
        receivers_[i]->set_raw_message_callback(record_cb);
        if (was_running)
        {
            if (!receivers_[i]->start())
            {
                LOG_ERROR("[gnss_controller] failed to restart receiver " + std::to_string(i)
                          + " after installing record callback");
            }
        }

        record_ports_.push_back(std::move(port));
        any_started = true;
        LOG_INFO("[gnss_controller] recording receiver " + std::to_string(i) + " to: " + path);
    }

    if (!any_started)
    {
        LOG_ERROR("[gnss_controller] no record files could be opened");
        record_ports_.clear();
        return false;
    }

    record_active_ = true;
    LOG_INFO("[gnss_controller] record mode active (" + std::to_string(receivers_.size()) + " receivers)");
    return true;
}

bool gnss_controller::stop_record()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (!record_active_)
    {
        LOG_WARNING("[gnss_controller] stop_record called but recording is not active");
        return false;
    }

    // Remove the record callback from each receiver and close its record file.
    const std::size_t n = std::min(receivers_.size(), record_ports_.size());
    for (std::size_t i = 0; i < n; ++i)
    {
        const bool was_running = receivers_[i]->is_running();
        if (was_running)
        {
            receivers_[i]->stop();
        }
        receivers_[i]->set_raw_message_callback(nullptr);
        if (was_running)
        {
            if (!receivers_[i]->start())
            {
                LOG_ERROR("[gnss_controller] failed to restart receiver " + std::to_string(i)
                          + " after removing record callback");
            }
        }

        if (record_ports_[i] != nullptr)
        {
            record_ports_[i]->close();
            LOG_INFO("[gnss_controller] closed record file for receiver " + std::to_string(i));
        }
    }

    record_ports_.clear();
    record_active_ = false;

    LOG_INFO("[gnss_controller] record mode stopped");
    return true;
}

// ─── Replay ────────────────────────────────────────────────────────────────────

bool gnss_controller::start_replay()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (replay_active_)
    {
        LOG_WARNING("[gnss_controller] replay already active");
        return false;
    }

    if (record_active_)
    {
        LOG_WARNING("[gnss_controller] cannot start replay while recording is active");
        return false;
    }

    replay_saved_ports_.clear();
    replay_saved_configs_.clear();
    replay_ports_.clear();

    bool any_started = false;
    for (std::size_t i = 0; i < receivers_.size(); ++i)
    {
        const std::string path = make_record_path(i);
        auto file_cfg  = std::make_shared<io::file_config>(io::file_config::replay(path, /*loop=*/false, /*throttle_us=*/10000));
        auto file_port = io::io_port_factory::create(*file_cfg);

        if (!file_port->open(*file_cfg))
        {
            LOG_ERROR("[gnss_controller] failed to open replay file for receiver "
                      + std::to_string(i) + ": " + path);
            // Save placeholders so indices stay aligned.
            replay_saved_ports_.push_back(nullptr);
            replay_saved_configs_.push_back(nullptr);
            replay_ports_.push_back(nullptr);
            continue;
        }

        // Terminate the receiver so its port reaches the idle state before
        // we swap the underlying port.
        receivers_[i]->terminate();

        // Save original port and config so stop_replay() can restore them.
        replay_saved_ports_.push_back(ports_[i]);
        replay_saved_configs_.push_back(configs_[i]);

        // Swap in the file port and reinitialise.
        receivers_[i]->set_port(file_port);
        if (!receivers_[i]->init(file_cfg))
        {
            LOG_ERROR("[gnss_controller] failed to init receiver " + std::to_string(i)
                      + " with replay file: " + path);
            // Restore the original port so the receiver is not left broken.
            receivers_[i]->set_port(ports_[i]);
            file_port->close();
            replay_saved_ports_.back() = nullptr;
            replay_saved_configs_.back() = nullptr;
            replay_ports_.push_back(nullptr);
            continue;
        }
        if (!receivers_[i]->start())
        {
            LOG_ERROR("[gnss_controller] failed to start receiver " + std::to_string(i)
                      + " in replay mode");
        }

        replay_ports_.push_back(std::move(file_port));
        any_started = true;
        LOG_INFO("[gnss_controller] replaying receiver " + std::to_string(i)
                 + " from: " + path);
    }

    if (!any_started)
    {
        LOG_ERROR("[gnss_controller] no replay files could be opened — restoring original ports");
        // Restore any receivers that were terminated.
        for (std::size_t i = 0; i < receivers_.size(); ++i)
        {
            if (replay_saved_ports_[i] != nullptr)
            {
                receivers_[i]->set_port(replay_saved_ports_[i]);
                receivers_[i]->init(replay_saved_configs_[i]);
                receivers_[i]->start();
            }
        }
        replay_saved_ports_.clear();
        replay_saved_configs_.clear();
        replay_ports_.clear();
        return false;
    }

    replay_active_ = true;
    LOG_INFO("[gnss_controller] replay mode active ("
             + std::to_string(receivers_.size()) + " receivers)");
    return true;
}

bool gnss_controller::stop_replay()
{
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    if (!replay_active_)
    {
        LOG_WARNING("[gnss_controller] stop_replay called but replay is not active");
        return false;
    }

    const std::size_t n = receivers_.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        // Terminate the receiver (idle state) before swapping the port back.
        receivers_[i]->terminate();

        if (i < replay_ports_.size() && replay_ports_[i] != nullptr)
        {
            replay_ports_[i]->close();
        }

        if (i < replay_saved_ports_.size() && replay_saved_ports_[i] != nullptr)
        {
            // Restore the original port and reinitialise.
            receivers_[i]->set_port(replay_saved_ports_[i]);
            if (!receivers_[i]->init(replay_saved_configs_[i]))
            {
                LOG_ERROR("[gnss_controller] failed to reinit receiver "
                          + std::to_string(i) + " after stop_replay");
                continue;
            }
            if (!receivers_[i]->start())
            {
                LOG_ERROR("[gnss_controller] failed to restart receiver "
                          + std::to_string(i) + " after stop_replay");
            }
            else
            {
                LOG_INFO("[gnss_controller] receiver " + std::to_string(i)
                         + " restored to live input");
            }
        }
    }

    replay_ports_.clear();
    replay_saved_ports_.clear();
    replay_saved_configs_.clear();
    replay_active_ = false;

    LOG_INFO("[gnss_controller] replay mode stopped");
    return true;
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

std::string gnss_controller::make_record_path(std::size_t index)
{
    return std::string(gnss::DEFAULT_UBX_RECORD_PATH) + std::to_string(index);
}

bool gnss_controller::send_poll(uint8_t msg_class, uint8_t msg_id)
{
    std::vector<uint8_t> frame = ubx::parser::ubx_message_builder::make_frame(msg_class, msg_id);
    return transport_->send_async(frame);
}

} // namespace gnss
