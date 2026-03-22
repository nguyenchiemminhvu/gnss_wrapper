#pragma once

#include "../adapter/base_database.h"

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
// Concrete base_database implementation that wires the ubx_parser database
// layer to the three shared_buffer types exposed by base_database.
//
// Lifecycle:
//   1. Construct ubx_database_wrapper.
//   2. Call register_with_parser(registry) to install decoder stubs in the
//      ubx_decoder_registry that the ubx_gnss_receiver will hand to the parser.
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

class ubx_database_wrapper : public base_database
{
public:
    ubx_database_wrapper();
    ~ubx_database_wrapper() override = default;

    // Non-copyable / non-movable (owns mutexes)
    ubx_database_wrapper(const ubx_database_wrapper&)            = delete;
    ubx_database_wrapper& operator=(const ubx_database_wrapper&) = delete;

    // ── Parser wiring ─────────────────────────────────────────────────────────
    //
    // Must be called once before the parser is constructed (i.e., before ubx_gnss_receiver::start()).
    // Installs decoder stubs for NAV-PVT, NAV-SAT, NAV-DOP, NAV-TIMEGPS, NAV-STATUS, and TIM-TP...
    void register_with_parser(ubx::parser::ubx_decoder_registry& registry);

    // ── base_database interface ───────────────────────────────────────────────
    const location_data&   get_location()   const override;
    const satellites_data& get_satellites() const override;
    const timing_data&     get_timing()     const override;

private:
    // Commit callbacks called from the ubx_database commit path (parser thread).
    void on_hp_commit(const ubx::database::ubx_location_snapshot& snap);
    void on_lp_commit(const ubx::database::ubx_location_snapshot& snap);

    std::shared_ptr<ubx::database::ubx_database>         db_;
    std::unique_ptr<ubx::database::ubx_database_adapter> adapter_;

    mutable std::mutex loc_mutex_;
    mutable std::mutex sat_mutex_;
    mutable std::mutex tim_mutex_;

    location_data   location_{};
    satellites_data satellites_{};
    timing_data     timing_{};
};

} // namespace gnss
