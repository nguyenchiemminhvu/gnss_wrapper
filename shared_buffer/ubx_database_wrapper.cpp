#include "ubx_database_wrapper.h"

// Handler headers — registered in the adapter to cover the standard epoch.
#include "ubx_parser/include/database/handlers/db_nav_pvt_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_sat_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_dop_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_timegps_handler.h"
#include "ubx_parser/include/database/handlers/db_nav_status_handler.h"
#include "ubx_parser/include/database/handlers/db_tim_tp_handler.h"

// Database field constants for snapshot extraction.
#include "ubx_parser/include/database/ubx_data_fields.h"
#include "ubx_parser/include/database/i_commit_policy.h"

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

    // Wire commit callbacks: HP → location + timing, LP → satellites.
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

// ─── base_database getters ─────────────────────────────────────────────────────

const location_data& ubx_database_wrapper::get_location() const
{
    std::lock_guard<std::mutex> lk(loc_mutex_);
    return location_;
}

const satellites_data& ubx_database_wrapper::get_satellites() const
{
    std::lock_guard<std::mutex> lk(sat_mutex_);
    return satellites_;
}

const timing_data& ubx_database_wrapper::get_timing() const
{
    std::lock_guard<std::mutex> lk(tim_mutex_);
    return timing_;
}

// ─── on_hp_commit ──────────────────────────────────────────────────────────────
//
// Called on every NAV-PVT epoch (high-priority commit).
// Extracts position, velocity, fix quality, and time fields from the snapshot,
// then refreshes location_data and timing_data.

void ubx_database_wrapper::on_hp_commit(const ubx::database::ubx_location_snapshot& snap)
{
    using namespace ubx::database;

    // ── Location buffer ──────────────────────────────────────────────────────
    {
        gnss::location_data loc{};

        // Position
        snap.get(DATA_UBX_NAV_PVT_LAT,  loc.latitude_deg);
        snap.get(DATA_UBX_NAV_PVT_LON,  loc.longitude_deg);
        snap.get(DATA_UBX_NAV_PVT_ALT,  loc.altitude_ellipsoid_mm);
        snap.get(DATA_UBX_NAV_PVT_HMSL, loc.altitude_msl_mm);

        // Accuracy
        snap.get(DATA_UBX_NAV_PVT_H_ACC,    loc.h_accuracy_mm);
        snap.get(DATA_UBX_NAV_PVT_V_ACC,    loc.v_accuracy_mm);
        snap.get(DATA_UBX_NAV_PVT_S_ACC,    loc.speed_accuracy_mmps);
        snap.get(DATA_UBX_NAV_PVT_HEAD_ACC, loc.heading_accuracy_deg);

        // Velocity
        snap.get(DATA_UBX_NAV_PVT_VEL_N,   loc.vel_n_mmps);
        snap.get(DATA_UBX_NAV_PVT_VEL_E,   loc.vel_e_mmps);
        snap.get(DATA_UBX_NAV_PVT_VEL_D,   loc.vel_d_mmps);
        snap.get(DATA_UBX_NAV_PVT_GSPEED,  loc.ground_speed_mmps);

        // Heading (already in degrees, converted by handler)
        snap.get(DATA_UBX_NAV_PVT_VEHICLE_HEADING_DEGREE,   loc.vehicle_heading_deg);
        snap.get(DATA_UBX_NAV_PVT_MOTION_HEADING_DEGREE,    loc.motion_heading_deg);
        snap.get(DATA_UBX_NAV_PVT_MAGNETIC_DECLINATION,     loc.magnetic_declination_deg);

        // Fix info
        snap.get(DATA_UBX_NAV_PVT_FIXTYPE, loc.fix_type);
        snap.get(DATA_UBX_NAV_PVT_NUMSV,   loc.num_sv);
        snap.get(DATA_UBX_NAV_PVT_FLAGS,   loc.flags);
        snap.get(DATA_UBX_NAV_PVT_FLAGS2,  loc.flags2);

        // Time
        snap.get(DATA_UBX_NAV_PVT_YEAR,         loc.year);
        snap.get(DATA_UBX_NAV_PVT_MONTH,         loc.month);
        snap.get(DATA_UBX_NAV_PVT_DAY,           loc.day);
        snap.get(DATA_UBX_NAV_PVT_HOUR,          loc.hour);
        snap.get(DATA_UBX_NAV_PVT_MINUTE,        loc.minute);
        snap.get(DATA_UBX_NAV_PVT_SECOND,        loc.second);
        snap.get(DATA_UBX_NAV_PVT_TIME_VALID,    loc.time_valid);
        snap.get(DATA_UBX_NAV_PVT_MILI_SECOND,  loc.millisecond);
        snap.get(DATA_UBX_NAV_PVT_MICRO_SECOND, loc.microsecond);
        snap.get(DATA_UBX_NAV_PVT_ITOW,         loc.i_tow_ms);
        snap.get(DATA_UBX_NAV_PVT_TIMESTAMP,    loc.utc_timestamp_ms);

        loc.valid = true;

        std::lock_guard<std::mutex> lk(loc_mutex_);
        location_ = loc;
    }
    data_updated.emit(gnss::buffer_type::location);

    // ── Timing buffer ────────────────────────────────────────────────────────
    {
        gnss::timing_data tim{};
        bool has_tp = snap.is_valid(DATA_TIME_PULSE_TIMESTAMP);
        if (has_tp)
        {
            snap.get(DATA_TIME_PULSE_TIMESTAMP,          tim.tow_ms);
            snap.get(DATA_TIME_PULSE_QUANTIZATION_ERROR, tim.q_err_ps);
            snap.get(DATA_TIME_PULSE_UTC_TIME_BASE,      tim.time_base);
            snap.get(DATA_TIME_PULSE_TIME_REF,           tim.time_ref);
            snap.get(DATA_TIME_PULSE_UTC_STANDARD,       tim.utc_standard);
            snap.get(DATA_TIME_PULSE_UTC_AVAILABLE,      tim.utc_available);
            snap.get(DATA_TIME_PULSE_Q_ERROR_VALID,      tim.q_err_valid);
        }

        bool has_gps = snap.is_valid(DATA_UBX_NAV_GPSTIME_TOW);
        if (has_gps)
        {
            snap.get(DATA_UBX_NAV_GPSTIME_TOW,          tim.gps_tow_ms);
            snap.get(DATA_UBX_NAV_GPSTIME_WEEK,         tim.gps_week);
            snap.get(DATA_UBX_NAV_GPSTIME_LEAP_SECONDS, tim.leap_seconds);
        }

        bool has_ttff = snap.is_valid(DATA_UBX_TTFF);
        if (has_ttff)
        {
            snap.get(DATA_UBX_TTFF, tim.ttff_ms);
        }

        tim.valid = (has_tp || has_gps || has_ttff);

        std::lock_guard<std::mutex> lk(tim_mutex_);
        timing_ = tim;
    }
    if (timing_.valid)
    {
        data_updated.emit(gnss::buffer_type::timing);
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
    snap.get(DATA_SATELLITES_IN_VIEW, count);
    sats.num_svs = static_cast<uint8_t>(
        std::min(static_cast<uint32_t>(GNSS_MAX_SV_COUNT), count));

    for (uint8_t i = 0u; i < sats.num_svs; ++i)
    {
        satellite_record& sv = sats.svs[i];

        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_GNSSID,     i), sv.gnss_id);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_SVID,       i), sv.sv_id);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_CN0_RATIO,  i), sv.cno);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_ELEVATION,  i), sv.elevation_deg);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_AZIMUTH,    i), sv.azimuth_deg);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_USED,       i), sv.used);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_ALMANAC,    i), sv.has_almanac);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_EPHEMERIS,  i), sv.has_ephemeris);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_HEALTH,     i), sv.healthy);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_DIFF_CORR,  i), sv.diff_correction);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_SBAS_USED,  i), sv.sbas_correction);
        snap.get(db_index_with_offset(DATA_SATELLITES_IN_VIEW_RTCM_USED,  i), sv.rtcm_correction);
    }

    sats.valid = true;

    {
        std::lock_guard<std::mutex> lk(sat_mutex_);
        satellites_ = sats;
    }
    data_updated.emit(gnss::buffer_type::satellites);
}

} // namespace gnss
