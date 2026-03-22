#pragma once

#include "../adapter/ubx_base_database.h"

#include "ubx_parser/include/ubx_decoder_registry.h"
#include "ubx_parser/include/database/ubx_database.h"
#include "ubx_parser/include/database/ubx_database_adapter.h"
#include "ubx_parser/include/database/ubx_location_snapshot.h"

#include <memory>
#include <mutex>

namespace gnss
{

// ─── ubx_database_wrapper ────────────────────────────────────────────────────────────
//
// Concrete ubx_base_database implementation that wires the ubx_parser database
// layer to the three shared_buffer types exposed by ubx_base_database.
//
// Lifecycle:
//   1. Construct ubx_database_wrapper.
//   2. Call register_with_parser(registry) to install decoder stubs in the
//      ubx_decoder_registry that the gnss_receiver will hand to the parser.
//   3. After parser construction, HP/LP commit callbacks fire automatically.
//
// Data flow:
//   ubx_parser → ubx_database_adapter decoder stubs
//              → ubx_database::apply_update()
//              → epoch_commit_policy HP/LP decision
//              → on_hp_commit / on_lp_commit (this class)
//              → shared-buffer update + data_updated.emit()
//
// Thread-safety: the commit callbacks are called from the parser (receiver
// worker) thread; the getters may be called from any thread.  Each buffer
// is protected by its own std::mutex.

class ubx_database_wrapper : public gnss::ubx_base_database
{
public:
    ubx_database_wrapper();
    ~ubx_database_wrapper() override = default;

    // Non-copyable / non-movable (owns mutexes)
    ubx_database_wrapper(const ubx_database_wrapper&)            = delete;
    ubx_database_wrapper& operator=(const ubx_database_wrapper&) = delete;

    // ── Parser wiring ─────────────────────────────────────────────────────────
    //
    // Must be called once before the parser is constructed (i.e., before gnss_receiver::start()).
    // Installs decoder stubs for all registered message handlers.
    void register_with_parser(ubx::parser::ubx_decoder_registry& registry);

    // ── ubx_base_database interface ───────────────────────────────────────────────
    const gnss::location_data&          get_location()          const override;
    const gnss::satellites_data&        get_satellites()        const override;
    const gnss::timing_data&            get_timing()            const override;
    const gnss::attitude_data&          get_attitude()          const override;
    const gnss::dop_data&               get_dop()               const override;
    const gnss::raw_measurements_data&  get_raw_measurements()  const override;
    const gnss::monitor_data&           get_monitor()           const override;
    const gnss::security_data&          get_security()          const override;
    const ubx::parser::datum_data&      get_datum()             const override;

    /// Update the datum buffer (called from gnss_controller datum_resolved callback).
    void update_datum(const ubx::parser::datum_data& data);

private:
    // Commit callbacks called from the ubx_database commit path (parser thread).
    void on_hp_commit(const ubx::database::ubx_location_snapshot& snap);
    void on_lp_commit(const ubx::database::ubx_location_snapshot& snap);

    std::shared_ptr<ubx::database::ubx_database>         db_;
    std::unique_ptr<ubx::database::ubx_database_adapter> adapter_;

    mutable std::mutex loc_mutex_;
    mutable std::mutex sat_mutex_;
    mutable std::mutex tim_mutex_;
    mutable std::mutex att_mutex_;
    mutable std::mutex dop_mutex_;
    mutable std::mutex raw_meas_mutex_;
    mutable std::mutex mon_mutex_;
    mutable std::mutex sec_mutex_;
    mutable std::mutex datum_mutex_;

    gnss::location_data          location_{};
    gnss::satellites_data        satellites_{};
    gnss::timing_data            timing_{};
    gnss::attitude_data          attitude_{};
    gnss::dop_data               dop_{};
    gnss::raw_measurements_data  raw_measurements_{};
    gnss::monitor_data           monitor_{};
    gnss::security_data          security_{};
    ubx::parser::datum_data             datum_{};};

} // namespace gnss
