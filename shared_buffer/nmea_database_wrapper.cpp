#ifdef NMEA_PARSER_ENABLED

#include "nmea_data_buffer.h"
#include "nmea_database_wrapper.h"

#include "database/handlers/db_gga_handler.h"
#include "database/handlers/db_rmc_handler.h"
#include "database/handlers/db_gsa_handler.h"
#include "database/handlers/db_gsv_handler.h"
#include "database/handlers/db_vtg_handler.h"
#include "database/nmea_data_fields.h"

#include <cstring>
#include <memory>

namespace gnss
{

// ─── Constructor ───────────────────────────────────────────────────────────────

nmea_database_wrapper::nmea_database_wrapper()
    : db_(std::make_shared<nmea::database::nmea_database>())
    , adapter_(std::make_unique<nmea::database::nmea_database_adapter>(db_))
{
    // Register all standard NMEA sentence handlers.
    adapter_->add_handler(std::make_unique<nmea::database::db_gga_handler>());
    adapter_->add_handler(std::make_unique<nmea::database::db_rmc_handler>());
    adapter_->add_handler(std::make_unique<nmea::database::db_gsa_handler>());
    adapter_->add_handler(std::make_unique<nmea::database::db_gsv_handler>());
    adapter_->add_handler(std::make_unique<nmea::database::db_vtg_handler>());

    // Single HP commit callback
    db_->set_commit_callback(
        nmea::database::commit_kind::high_priority,
        [this](const nmea::database::nmea_snapshot& snap) {
            on_hp_commit(snap);
        });
    
    // Single LP commit callback
    db_->set_commit_callback(
        nmea::database::commit_kind::low_priority,
        [this](const nmea::database::nmea_snapshot& snap) {
            on_lp_commit(snap);
        });
}

// ─── register_with_parser ─────────────────────────────────────────────────────

void nmea_database_wrapper::register_with_parser(
    nmea::parser::nmea_sentence_registry& registry)
{
    adapter_->register_with_parser(registry);
}

// ─── get_nmea_data ────────────────────────────────────────────────────────────

const gnss::nmea_data& nmea_database_wrapper::get_nmea_data() const
{
    std::lock_guard<std::mutex> lk(data_mutex_);
    return data_;
}

// ─── on_hp_commit ────────────────────────────────────────────────────────────────
//
// Called synchronously from the nmea_database write path (parser thread).
// Extracts all available fields from the snapshot into the nmea_data buffer,
// then emits data_updated.

void nmea_database_wrapper::on_hp_commit(const nmea::database::nmea_snapshot& snap)
{
    {
        gnss::nmea_data local_data{};

        // ── GGA ──────────────────────────────────────────────────────────────────
        snap.get(nmea::database::NMEA_GGA_LATITUDE,        local_data.latitude_deg);
        snap.get(nmea::database::NMEA_GGA_LONGITUDE,       local_data.longitude_deg);
        snap.get(nmea::database::NMEA_GGA_ALTITUDE_MSL,    local_data.altitude_msl_m);
        snap.get(nmea::database::NMEA_GGA_GEOID_SEP,       local_data.geoid_separation_m);
        snap.get(nmea::database::NMEA_GGA_HDOP,            local_data.hdop);
        snap.get(nmea::database::NMEA_GGA_FIX_QUALITY,     local_data.fix_quality);
        snap.get(nmea::database::NMEA_GGA_NUM_SATELLITES,  local_data.num_satellites);

        // ── RMC ──────────────────────────────────────────────────────────────────
        snap.get(nmea::database::NMEA_RMC_SPEED_KNOTS,     local_data.speed_knots);
        snap.get(nmea::database::NMEA_RMC_COURSE_TRUE,     local_data.course_true_deg);
        {
            float status = 0.0f;
            if (snap.get(nmea::database::NMEA_RMC_STATUS_ACTIVE, status))
            {
                local_data.rmc_status_active = (status >= 1.0f);
            }
        }

        // ── GSA ──────────────────────────────────────────────────────────────────
        snap.get(nmea::database::NMEA_GSA_NAV_MODE,        local_data.nav_mode);
        snap.get(nmea::database::NMEA_GSA_PDOP,            local_data.pdop);
        snap.get(nmea::database::NMEA_GSA_HDOP,            local_data.hdop); // GSA HDOP overrides GGA HDOP
        snap.get(nmea::database::NMEA_GSA_VDOP,            local_data.vdop);

        // ── VTG ──────────────────────────────────────────────────────────────────
        snap.get(nmea::database::NMEA_VTG_SPEED_KMPH,      local_data.speed_kmph);
        snap.get(nmea::database::NMEA_VTG_COURSE_MAG,      local_data.course_mag_deg);
        // ── GSV ──────────────────────────────────────────────────────────────────
        {
            uint8_t n = 0u;
            if (snap.get(nmea::database::NMEA_GSV_NUM_SATS_IN_VIEW, n))
            {
                local_data.num_sats_in_view = n;
                if (n > gnss::NMEA_MAX_SAT_VIEW)
                {
                    n = gnss::NMEA_MAX_SAT_VIEW;
                }
                for (uint8_t i = 0u; i < n; ++i)
                {
                    snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_SVID,      i),
                            local_data.sats[i].sv_id);
                    snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_ELEVATION, i),
                            local_data.sats[i].elevation_deg);
                    snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_AZIMUTH,   i),
                            local_data.sats[i].azimuth_deg);
                    snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_SNR,       i),
                            local_data.sats[i].snr);
                }
            }
        }

        // ── GSV ──────────────────────────────────────────────────────────────────
        uint8_t n = 0u;
        if (snap.get(nmea::database::NMEA_GSV_NUM_SATS_IN_VIEW, n))
        {
            local_data.num_sats_in_view = n;
            if (n > gnss::NMEA_MAX_SAT_VIEW)
            {
                n = gnss::NMEA_MAX_SAT_VIEW;
            }
            for (uint8_t i = 0u; i < n; ++i)
            {
                snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_SVID,      i),
                            local_data.sats[i].sv_id);
                snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_ELEVATION, i),
                            local_data.sats[i].elevation_deg);
                snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_AZIMUTH,   i),
                            local_data.sats[i].azimuth_deg);
                snap.get(nmea::database::nmea_gsv_sat_field(
                                nmea::database::NMEA_GSV_SAT_SNR,       i),
                            local_data.sats[i].snr);
            }
        }

        local_data.valid = true;

        std::lock_guard<std::mutex> lk(data_mutex_);
        data_ = local_data;
    }

    data_updated.emit(gnss::buffer_type::nmea_data);
}

// ─── on_lp_commit ────────────────────────────────────────────────────────────────
//
// Called synchronously from the nmea_database write path (parser thread).
// Extracts all available fields from the snapshot into the nmea_data buffer,
// then emits data_updated.

void nmea_database_wrapper::on_lp_commit(const nmea::database::nmea_snapshot& snap)
{
    // No LP fields for now, but we could extract satellite view data here if desired.
}

} // namespace gnss

#endif // NMEA_PARSER_ENABLED
