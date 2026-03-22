#include "ubx_database_wrapper.h"

// Handler headers — registered in the adapter to cover the standard epoch.
#include "ubx_parser/include/database/handlers/db_nav_pvt_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_sat_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_dop_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_timegps_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_status_handler.h"
#include "ubx_parser/include/database/handlers/db_tim_tp_handler.h"
// Extended message handlers.
#include "ubx_parser/include/database/handlers/db_nav_timeutc_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_clock_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_att_handler.h"
#include "ubx_parser/include/database/handlers/db_rxm_measx_handler.h"
#include "ubx_parser/include/database/handlers/db_mon_io_handler.h"
#include "ubx_parser/include/database/handlers/db_mon_txbuf_handler.h"
#include "ubx_parser/include/database/handlers/db_mon_span_handler.h"
#include "ubx_parser/include/database/handlers/db_sec_sig_handler.h"
#include "ubx_parser/include/database/handlers/db_esf_ins_handler.h"
#include "ubx_parser/include/database/handlers/db_esf_status_handler.h"

// Database field constants for snapshot extraction.
#include "ubx_parser/include/database/ubx_data_fields.h"
#include "ubx_parser/include/database/i_commit_policy.h"

#include "attitude_data_buffer.h"
#include "dop_data_buffer.h"
#include "raw_measurements_data_buffer.h"
#include "monitor_data_buffer.h"
#include "security_data_buffer.h"

#include <algorithm>
#include <cstring>

namespace gnss
{

// ─── Constructor ───────────────────────────────────────────────────────────────

ubx_database_wrapper::ubx_database_wrapper()
    : db_(std::make_shared<ubx::database::ubx_database>()) // default commit policy is epoch_commit_policy()
    , adapter_(std::make_unique<ubx::database::ubx_database_adapter>(db_))
{
    // Register standard navigation handlers.
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_pvt_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_sat_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_dop_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_timegps_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_status_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_tim_tp_handler>());
    // Extended message handlers.
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_timeutc_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_clock_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_nav_att_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_rxm_measx_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_mon_io_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_mon_txbuf_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_mon_span_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_sec_sig_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_esf_ins_handler>());
    adapter_->add_handler(std::make_unique<ubx::database::db_esf_status_handler>());

    // Wire commit callbacks: HP → location + timing + attitude + security,
    //                        LP → satellites + raw_measurements + monitor.
    db_->set_commit_callback(
        ubx::database::commit_kind::high_priority,
        [this](const ubx::database::ubx_location_snapshot& snap) {
            on_hp_commit(snap);
        });

    db_->set_commit_callback(
        ubx::database::commit_kind::low_priority,
        [this](const ubx::database::ubx_location_snapshot& snap) {
            on_lp_commit(snap);
        });
}

// ─── register_with_parser ──────────────────────────────────────────────────────

void ubx_database_wrapper::register_with_parser(ubx::parser::ubx_decoder_registry& registry)
{
    adapter_->register_with_parser(registry);
}

// ─── ubx_base_database getters ─────────────────────────────────────────────────

const gnss::location_data& ubx_database_wrapper::get_location() const
{
    std::lock_guard<std::mutex> lk(loc_mutex_);
    return location_;
}

const gnss::satellites_data& ubx_database_wrapper::get_satellites() const
{
    std::lock_guard<std::mutex> lk(sat_mutex_);
    return satellites_;
}

const gnss::timing_data& ubx_database_wrapper::get_timing() const
{
    std::lock_guard<std::mutex> lk(tim_mutex_);
    return timing_;
}

const gnss::attitude_data& ubx_database_wrapper::get_attitude() const
{
    std::lock_guard<std::mutex> lk(att_mutex_);
    return attitude_;
}

const gnss::dop_data& ubx_database_wrapper::get_dop() const
{
    std::lock_guard<std::mutex> lk(dop_mutex_);
    return dop_;
}

const gnss::raw_measurements_data& ubx_database_wrapper::get_raw_measurements() const
{
    std::lock_guard<std::mutex> lk(raw_meas_mutex_);
    return raw_measurements_;
}

const gnss::monitor_data& ubx_database_wrapper::get_monitor() const
{
    std::lock_guard<std::mutex> lk(mon_mutex_);
    return monitor_;
}

const gnss::security_data& ubx_database_wrapper::get_security() const
{
    std::lock_guard<std::mutex> lk(sec_mutex_);
    return security_;
}

const gnss::dead_reckoning_data_out& ubx_database_wrapper::get_dead_reckoning_out() const
{
    std::lock_guard<std::mutex> lk(dr_out_mutex_);
    return dead_reckoning_out_;
}

const gnss::dead_reckoning_data_in& ubx_database_wrapper::get_dead_reckoning_in() const
{
    std::lock_guard<std::mutex> lk(dr_in_mutex_);
    return dead_reckoning_in_;
}

// ─── ubx_base_database setters ─────────────────────────────────────────────────

void ubx_database_wrapper::update_dead_reckoning_in(const gnss::dead_reckoning_data_in& dr_input_params)
{
    {
        std::lock_guard<std::mutex> lk(dr_in_mutex_);
        dead_reckoning_in_ = dr_input_params;
    }
    data_updated.emit(gnss::buffer_type::dead_reckoning_in);
}

// ─── on_hp_commit ──────────────────────────────────────────────────────────────
//
// Called on every NAV-PVT epoch (high-priority commit).
// Extracts position, velocity, fix quality, and time fields from the snapshot,
// then refreshes location_data and timing_data.

void ubx_database_wrapper::on_hp_commit(const ubx::database::ubx_location_snapshot& snap)
{
    // ── Location buffer ──────────────────────────────────────────────────────
    {
        gnss::location_data loc{};

        // Position
        snap.get(ubx::database::DATA_UBX_NAV_PVT_LAT,  loc.latitude_deg);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_LON,  loc.longitude_deg);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_ALT,  loc.altitude_ellipsoid_mm);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_HMSL, loc.altitude_msl_mm);

        // Accuracy
        snap.get(ubx::database::DATA_UBX_NAV_PVT_H_ACC,    loc.h_accuracy_mm);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_V_ACC,    loc.v_accuracy_mm);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_S_ACC,    loc.speed_accuracy_mmps);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_HEAD_ACC, loc.heading_accuracy_deg);

        // Velocity
        snap.get(ubx::database::DATA_UBX_NAV_PVT_VEL_N,   loc.vel_n_mmps);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_VEL_E,   loc.vel_e_mmps);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_VEL_D,   loc.vel_d_mmps);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_GSPEED,  loc.ground_speed_mmps);

        // Heading (already in degrees, converted by handler)
        snap.get(ubx::database::DATA_UBX_NAV_PVT_VEHICLE_HEADING_DEGREE,   loc.vehicle_heading_deg);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_MOTION_HEADING_DEGREE,    loc.motion_heading_deg);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_MAGNETIC_DECLINATION,     loc.magnetic_declination_deg);

        // Fix info
        snap.get(ubx::database::DATA_UBX_NAV_PVT_FIXTYPE, loc.fix_type);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_NUMSV,   loc.num_sv);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_FLAGS,   loc.flags);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_FLAGS2,  loc.flags2);

        // Time
        snap.get(ubx::database::DATA_UBX_NAV_PVT_YEAR,         loc.year);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_MONTH,         loc.month);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_DAY,           loc.day);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_HOUR,          loc.hour);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_MINUTE,        loc.minute);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_SECOND,        loc.second);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_TIME_VALID,    loc.time_valid);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_MILI_SECOND,  loc.millisecond);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_MICRO_SECOND, loc.microsecond);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_ITOW,         loc.i_tow_ms);
        snap.get(ubx::database::DATA_UBX_NAV_PVT_TIMESTAMP,    loc.utc_timestamp_ms);

        loc.valid = true;

        std::lock_guard<std::mutex> lk(loc_mutex_);
        location_ = loc;
    }
    data_updated.emit(gnss::buffer_type::location);

    // ── Timing buffer ────────────────────────────────────────────────────────
    {
        gnss::timing_data tim{};
        bool has_tp = snap.is_valid(ubx::database::DATA_TIME_PULSE_TIMESTAMP);
        if (has_tp)
        {
            snap.get(ubx::database::DATA_TIME_PULSE_TIMESTAMP,          tim.tow_ms);
            snap.get(ubx::database::DATA_TIME_PULSE_QUANTIZATION_ERROR, tim.q_err_ps);
            snap.get(ubx::database::DATA_TIME_PULSE_UTC_TIME_BASE,      tim.time_base);
            snap.get(ubx::database::DATA_TIME_PULSE_TIME_REF,           tim.time_ref);
            snap.get(ubx::database::DATA_TIME_PULSE_UTC_STANDARD,       tim.utc_standard);
            snap.get(ubx::database::DATA_TIME_PULSE_UTC_AVAILABLE,      tim.utc_available);
            snap.get(ubx::database::DATA_TIME_PULSE_Q_ERROR_VALID,      tim.q_err_valid);
        }

        bool has_gps = snap.is_valid(ubx::database::DATA_UBX_NAV_GPSTIME_TOW);
        if (has_gps)
        {
            snap.get(ubx::database::DATA_UBX_NAV_GPSTIME_TOW,          tim.gps_tow_ms);
            snap.get(ubx::database::DATA_UBX_NAV_GPSTIME_WEEK,         tim.gps_week);
            snap.get(ubx::database::DATA_UBX_NAV_GPSTIME_LEAP_SECONDS, tim.leap_seconds);
        }

        bool has_ttff = snap.is_valid(ubx::database::DATA_UBX_TTFF);
        if (has_ttff)
        {
            snap.get(ubx::database::DATA_UBX_TTFF, tim.ttff_ms);
        }

        // ── UBX-NAV-TIMEUTC supplement ───────────────────────────────────────
        bool has_timeutc = snap.is_valid(ubx::database::DATA_UBX_NAV_TIMEUTC_ITOW);
        if (has_timeutc)
        {
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_ITOW,  tim.utc_i_tow_ms);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_T_ACC, tim.utc_t_acc_ns);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_NANO,  tim.utc_nano_ns);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_YEAR,  tim.utc_year);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_MONTH, tim.utc_month);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_DAY,   tim.utc_day);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_HOUR,  tim.utc_hour);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_MIN,   tim.utc_min);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_SEC,   tim.utc_sec);
            snap.get(ubx::database::DATA_UBX_NAV_TIMEUTC_VALID, tim.utc_valid_flags);
        }

        // ── UBX-NAV-CLOCK supplement ─────────────────────────────────────────
        bool has_clock = snap.is_valid(ubx::database::DATA_UBX_NAV_CLOCK_ITOW);
        if (has_clock)
        {
            snap.get(ubx::database::DATA_UBX_NAV_CLOCK_ITOW,  tim.clock_i_tow_ms);
            snap.get(ubx::database::DATA_UBX_NAV_CLOCK_CLKB,  tim.clock_bias_ns);
            snap.get(ubx::database::DATA_UBX_NAV_CLOCK_CLKD,  tim.clock_drift_nsps);
            snap.get(ubx::database::DATA_UBX_NAV_CLOCK_T_ACC, tim.clock_t_acc_ns);
            snap.get(ubx::database::DATA_UBX_NAV_CLOCK_F_ACC, tim.clock_f_acc_psps);
        }

        tim.valid = (has_tp || has_gps || has_ttff || has_timeutc || has_clock);

        std::lock_guard<std::mutex> lk(tim_mutex_);
        timing_ = tim;
    }
    if (timing_.valid)
    {
        data_updated.emit(gnss::buffer_type::timing);
    }

    // ── Attitude buffer (UBX-NAV-ATT) ────────────────────────────────────────
    if (snap.is_valid(ubx::database::DATA_UBX_NAV_ATT_ITOW))
    {
        gnss::attitude_data att{};

        snap.get(ubx::database::DATA_UBX_NAV_ATT_ITOW,        att.i_tow_ms);
        snap.get(ubx::database::DATA_UBX_NAV_ATT_VERSION,     att.version);
        snap.get(ubx::database::DATA_UBX_NAV_ATT_ROLL,        att.roll_deg);
        snap.get(ubx::database::DATA_UBX_NAV_ATT_PITCH,       att.pitch_deg);
        snap.get(ubx::database::DATA_UBX_NAV_ATT_HEADING,     att.heading_deg);
        snap.get(ubx::database::DATA_UBX_NAV_ATT_ACC_ROLL,    att.acc_roll_deg);
        snap.get(ubx::database::DATA_UBX_NAV_ATT_ACC_PITCH,   att.acc_pitch_deg);
        snap.get(ubx::database::DATA_UBX_NAV_ATT_ACC_HEADING, att.acc_heading_deg);
        att.valid = true;

        {
            std::lock_guard<std::mutex> lk(att_mutex_);
            attitude_ = att;
        }
        data_updated.emit(gnss::buffer_type::attitude);
    }

    // ── DOP buffer (UBX-NAV-DOP) ─────────────────────────────────────────────
    if (snap.is_valid(ubx::database::DATA_UBX_NAV_HORIZONAL_DILUTION_OF_PRECISION))
    {
        gnss::dop_data dop{};

        snap.get(ubx::database::DATA_UBX_NAV_GEOMETRIC_DILUTION_OF_PRECISION, dop.gdop);
        snap.get(ubx::database::DATA_UBX_NAV_POSITION_DILUTION_OF_PRECISION,  dop.pdop);
        snap.get(ubx::database::DATA_UBX_NAV_TIME_DILUTION_OF_PRECISION,      dop.tdop);
        snap.get(ubx::database::DATA_UBX_NAV_VERTICAL_DILUTION_OF_PRECISION,  dop.vdop);
        snap.get(ubx::database::DATA_UBX_NAV_HORIZONAL_DILUTION_OF_PRECISION, dop.hdop);
        dop.valid = true;

        {
            std::lock_guard<std::mutex> lk(dop_mutex_);
            dop_ = dop;
        }
        data_updated.emit(gnss::buffer_type::dop);
    }

    // ── Security buffer (UBX-SEC-SIG) ────────────────────────────────────────
    if (snap.is_valid(ubx::database::DATA_UBX_SEC_SIG_VERSION))
    {
        gnss::security_data sec{};

        snap.get(ubx::database::DATA_UBX_SEC_SIG_JAM_FLAGS,       sec.jam_flags);
        snap.get(ubx::database::DATA_UBX_SEC_SIG_JAM_DET_ENABLED, sec.jam_det_enabled);
        snap.get(ubx::database::DATA_UBX_SEC_SIG_JAMMING_STATE,   sec.jamming_state);
        snap.get(ubx::database::DATA_UBX_SEC_SIG_SPF_FLAGS,       sec.spf_flags);
        snap.get(ubx::database::DATA_UBX_SEC_SIG_SPF_DET_ENABLED, sec.spf_det_enabled);
        snap.get(ubx::database::DATA_UBX_SEC_SIG_SPOOFING_STATE,  sec.spoofing_state);
        sec.valid = true;

        {
            std::lock_guard<std::mutex> lk(sec_mutex_);
            security_ = sec;
        }
        data_updated.emit(gnss::buffer_type::security);
    }

    // ── Dead Reckoning buffer (UBX-ESF-INS + UBX-ESF-STATUS) ─────────────────
    {
        gnss::dead_reckoning_data_out dr{};

        if (snap.is_valid(ubx::database::DATA_UBX_ESF_INS_ITOW))
        {
            snap.get(ubx::database::DATA_UBX_ESF_INS_ITOW,       dr.ins_i_tow_ms);
            snap.get(ubx::database::DATA_UBX_ESF_INS_BITFIELD0,  dr.ins_bitfield0);
            snap.get(ubx::database::DATA_UBX_ESF_INS_X_ANG_RATE, dr.x_ang_rate_dps);
            snap.get(ubx::database::DATA_UBX_ESF_INS_Y_ANG_RATE, dr.y_ang_rate_dps);
            snap.get(ubx::database::DATA_UBX_ESF_INS_Z_ANG_RATE, dr.z_ang_rate_dps);
            snap.get(ubx::database::DATA_UBX_ESF_INS_X_ACCEL,    dr.x_accel_mss);
            snap.get(ubx::database::DATA_UBX_ESF_INS_Y_ACCEL,    dr.y_accel_mss);
            snap.get(ubx::database::DATA_UBX_ESF_INS_Z_ACCEL,    dr.z_accel_mss);
            dr.ins_valid = true;
        }

        if (snap.is_valid(ubx::database::DATA_UBX_ESF_STATUS_ITOW))
        {
            snap.get(ubx::database::DATA_UBX_ESF_STATUS_ITOW,         dr.status_i_tow_ms);
            snap.get(ubx::database::DATA_UBX_ESF_STATUS_FUSION_MODE,  dr.fusion_mode);
            snap.get(ubx::database::DATA_UBX_ESF_STATUS_NUM_SENS,     dr.num_sens);

            const uint8_t count = static_cast<uint8_t>(
                std::min(static_cast<unsigned>(dr.num_sens),
                         static_cast<unsigned>(gnss::GNSS_MAX_DR_SENSORS)));

            for (uint8_t i = 0u; i < count; ++i)
            {
                gnss::dr_sensor_out_record& s = dr.sensors[i];
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_ESF_STATUS_SENS_TYPE,        i), s.type);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_ESF_STATUS_SENS_USED,        i), s.used);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_ESF_STATUS_SENS_READY,       i), s.ready);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_ESF_STATUS_SENS_CALIB,       i), s.calib_status);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_ESF_STATUS_SENS_TIME_STATUS, i), s.time_status);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_ESF_STATUS_SENS_FREQ,        i), s.freq);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_ESF_STATUS_SENS_FAULTS,      i), s.faults);
            }
            dr.status_valid = true;
        }

        dr.valid = dr.ins_valid || dr.status_valid;

        if (dr.valid)
        {
            {
                std::lock_guard<std::mutex> lk(dr_out_mutex_);
                dead_reckoning_out_ = dr;
            }
            data_updated.emit(gnss::buffer_type::dead_reckoning_out);
        }
    }
}

// ─── on_lp_commit ──────────────────────────────────────────────────────────────
//
// Called on every NAV-SAT epoch (low-priority commit).
// Extracts satellite records from the snapshot.

void ubx_database_wrapper::on_lp_commit(const ubx::database::ubx_location_snapshot& snap)
{
    using namespace ubx::database;

    gnss::satellites_data sats{};
    std::memset(&sats, 0, sizeof(sats));

    uint32_t count = 0u;
    snap.get(ubx::database::DATA_SATELLITES_IN_VIEW, count);
    sats.num_svs = static_cast<uint8_t>(
        std::min(static_cast<uint32_t>(gnss::GNSS_MAX_SV_COUNT), count));

    for (uint8_t i = 0u; i < sats.num_svs; ++i)
    {
        satellite_record& sv = sats.svs[i];

        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_GNSSID,     i), sv.gnss_id);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_SVID,       i), sv.sv_id);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_CN0_RATIO,  i), sv.cno);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_ELEVATION,  i), sv.elevation_deg);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_AZIMUTH,    i), sv.azimuth_deg);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_USED,       i), sv.used);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_ALMANAC,    i), sv.has_almanac);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_EPHEMERIS,  i), sv.has_ephemeris);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_HEALTH,     i), sv.healthy);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_DIFF_CORR,  i), sv.diff_correction);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_SBAS_USED,  i), sv.sbas_correction);
        snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_SATELLITES_IN_VIEW_RTCM_USED,  i), sv.rtcm_correction);
    }

    sats.valid = true;

    {
        std::lock_guard<std::mutex> lk(sat_mutex_);
        satellites_ = sats;
    }
    data_updated.emit(gnss::buffer_type::satellites);

    // ── Raw measurements buffer (UBX-RXM-MEASX) ──────────────────────────────
    if (snap.is_valid(ubx::database::DATA_UBX_RXM_MEASX_NUM_SV))
    {
        gnss::raw_measurements_data raw{};

        snap.get(ubx::database::DATA_UBX_RXM_MEASX_GPS_TOW,      raw.gps_tow_ms);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_GLO_TOW,      raw.glo_tow_ms);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_BDS_TOW,      raw.bds_tow_ms);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_QZS_TOW,      raw.qzss_tow_ms);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_GPS_TOW_ACC,  raw.gps_tow_acc);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_GLO_TOW_ACC,  raw.glo_tow_acc);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_BDS_TOW_ACC,  raw.bds_tow_acc);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_QZS_TOW_ACC,  raw.qzss_tow_acc);
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_FLAGS,        raw.flags);

        uint32_t num_sv = 0u;
        snap.get(ubx::database::DATA_UBX_RXM_MEASX_NUM_SV, num_sv);
        raw.num_svs = static_cast<uint8_t>(
            std::min(static_cast<uint32_t>(gnss::GNSS_MAX_MEASX_SV_COUNT), num_sv));

        for (uint8_t i = 0u; i < raw.num_svs; ++i)
        {
            gnss::raw_sv_record& sv = raw.svs[i];
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_GNSS_ID,          i), sv.gnss_id);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_SV_ID,           i), sv.sv_id);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_CN0,             i), sv.cno);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_MPH,             i), sv.mpath_indic);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_DOPPLER,         i), sv.doppler_ms);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_WHOLE_CHIPS,     i), sv.whole_chips);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_FRAC_CHIPS,      i), sv.frac_chips);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_CODE_PHASE,      i), sv.code_phase);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_INT_CODE_PHASE,  i), sv.int_code_phase);
            snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_RXM_MEASX_PSEU_RANGE_RMS_ERR, i), sv.pseu_range_rms_err);
        }
        raw.valid = true;

        {
            std::lock_guard<std::mutex> lk(raw_meas_mutex_);
            raw_measurements_ = raw;
        }
        data_updated.emit(gnss::buffer_type::raw_measurements);
    }

    // ── Monitor buffer (UBX-MON-IO + UBX-MON-TXBUF) ──────────────────────────
    {
        gnss::monitor_data mon{};
        std::memset(&mon, 0, sizeof(mon));

        // ── MON-IO ───────────────────────────────────────────────────────────
        if (snap.is_valid(ubx::database::DATA_UBX_MON_IO_NUM_PORTS))
        {
            uint32_t np = 0u;
            snap.get(ubx::database::DATA_UBX_MON_IO_NUM_PORTS, np);
            mon.num_ports = static_cast<uint8_t>(
                std::min(static_cast<uint32_t>(gnss::GNSS_MAX_MON_IO_PORTS), np));

            for (uint8_t i = 0u; i < mon.num_ports; ++i)
            {
                gnss::mon_io_port_record& p = mon.ports[i];
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_RX_BYTES,      i), p.rx_bytes);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_TX_BYTES,      i), p.tx_bytes);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_PARITY_ERRS,   i), p.parity_errs);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_FRAMING_ERRS,  i), p.framing_errs);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_OVERRUN_ERRS,  i), p.overrun_errs);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_BREAK_COND,    i), p.break_cond);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_RX_BUSY,       i), p.rx_busy);
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_IO_TX_BUSY,       i), p.tx_busy);
            }
            mon.io_valid = true;
        }

        // ── MON-TXBUF ────────────────────────────────────────────────────────
        if (snap.is_valid(ubx::database::DATA_UBX_MON_TXBUF_ERRORS))
        {
            gnss::mon_txbuf_data& tb = mon.txbuf;
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_AVAIL_0,      tb.usage[0]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_AVAIL_1,      tb.usage[1]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_AVAIL_2,      tb.usage[2]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_AVAIL_3,      tb.usage[3]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_AVAIL_4,      tb.usage[4]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_AVAIL_5,      tb.usage[5]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_PENDING_0,    tb.pending[0]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_PENDING_1,    tb.pending[1]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_PENDING_2,    tb.pending[2]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_PENDING_3,    tb.pending[3]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_PENDING_4,    tb.pending[4]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_PENDING_5,    tb.pending[5]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_USAGE_0,      tb.peak_usage[0]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_USAGE_1,      tb.peak_usage[1]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_USAGE_2,      tb.peak_usage[2]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_USAGE_3,      tb.peak_usage[3]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_USAGE_4,      tb.peak_usage[4]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_USAGE_5,      tb.peak_usage[5]);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_TUSED,        tb.t_used);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_TPEAK,        tb.t_peak);
            snap.get(ubx::database::DATA_UBX_MON_TXBUF_ERRORS,       tb.errors);
            mon.txbuf_valid = true;
        }

        // ── MON-SPAN ─────────────────────────────────────────────────────────
        if (snap.is_valid(ubx::database::DATA_UBX_MON_SPAN_NUM_RF_BLOCKS))
        {
            uint32_t nr = 0u;
            snap.get(ubx::database::DATA_UBX_MON_SPAN_NUM_RF_BLOCKS, nr);
            mon.span.num_rf_blocks = static_cast<uint8_t>(
                std::min(static_cast<uint32_t>(gnss::GNSS_MON_SPAN_RF_BLOCKS), nr));

            for (uint8_t i = 0u; i < mon.span.num_rf_blocks; ++i)
            {
                snap.get(ubx::database::db_index_with_offset(ubx::database::DATA_UBX_MON_SPAN_PGA, i), mon.span.pga[i]);
            }
            mon.span_valid = true;
        }

        if (mon.io_valid || mon.txbuf_valid || mon.span_valid)
        {
            mon.valid = true;
            {
                std::lock_guard<std::mutex> lk(mon_mutex_);
                monitor_ = mon;
            }
            data_updated.emit(gnss::buffer_type::monitor);
        }
    }
}

} // namespace gnss
