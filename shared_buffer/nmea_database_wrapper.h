#pragma once

#ifdef NMEA_PARSER_ENABLED

#include "../adapter/nmea_base_database.h"

#include "database/nmea_database.h"
#include "database/nmea_database_adapter.h"
#include "database/nmea_snapshot.h"
#include "database/i_update_policy.h"
#include "nmea_sentence_registry.h"

#include <memory>
#include <mutex>

namespace gnss
{

// ─── nmea_uniform_policy ─────────────────────────────────────────────────────
//
// Maps every NMEA message type to high_priority so a single commit callback
// handles all updates.  This avoids the need for a separate low_priority
// callback and keeps the wrapper simple.

class nmea_uniform_policy : public nmea::database::i_update_policy
{
public:
    nmea::database::commit_kind should_commit(nmea::database::msg_type /*msg*/) const override
    {
        return nmea::database::commit_kind::high_priority;
    }
};

// ─── nmea_database_wrapper ────────────────────────────────────────────────────
//
// Concrete nmea_base_database implementation that wires the nmea_parser database
// layer to the nmea_data shared buffer exposed by nmea_base_database.
//
// Role / scope mirrors ubx_database_wrapper:
//   1. Construct nmea_database_wrapper.
//   2. Call register_with_parser(registry) each time the receiver starts so that
//      sentence handler stubs are installed in the nmea_sentence_registry.
//   3. After the nmea_parser is constructed with that registry, HP commit
//      callbacks fire automatically and fill the local nmea_data buffer.
//
// Data flow:
//   nmea_parser → nmea_database_adapter sentence stubs
//              → nmea_database::apply_update()
//              → default_commit_policy: should_commit() → GGA, RMC for HP and GSV for LP
//              → on_hp_commit(snapshot) (this class)
//              → nmea_data buffer update + data_updated.emit()
//
// Design notes:
//   • Uses default_commit_policy for a single callback path.
//   • data_mutex_ protects the nmea_data buffer for concurrent reads.
//   • Each receiver calls register_with_parser() on every start(), producing a
//     fresh registry.  All receivers that share this wrapper will write to the
//     same buffer — last writer wins, which is the correct merge strategy for
//     a single physical GNSS chipset with multiple UART output streams.
//
// Thread-safety: the commit callback is called from the parser (receiver worker)
// thread; get_nmea_data() may be called from any thread.

class nmea_database_wrapper : public gnss::nmea_base_database
{
public:
    nmea_database_wrapper();
    ~nmea_database_wrapper() = default;

    // Non-copyable / non-movable (owns mutexes)
    nmea_database_wrapper(const nmea_database_wrapper&)            = delete;
    nmea_database_wrapper& operator=(const nmea_database_wrapper&) = delete;

    // ── Parser wiring ─────────────────────────────────────────────────────────
    //
    // Must be called once before the nmea_parser is constructed
    // (i.e., before gnss_receiver::start()).  May be called again on each
    // start() — each call produces a fresh set of sentence parser stubs.
    void register_with_parser(nmea::parser::nmea_sentence_registry& registry);

    // ── nmea_base_database interface ──────────────────────────────────────────

    /// Thread-safe read access to the last committed NMEA data buffer.
    const gnss::nmea_data& get_nmea_data() const override;

private:
    // Commit callback — called synchronously from the nmea_database write path
    // (parser thread).  Extracts fields from the snapshot into nmea_data_.
    void on_hp_commit(const nmea::database::nmea_snapshot& snap);
    void on_lp_commit(const nmea::database::nmea_snapshot& snap);

    std::shared_ptr<nmea::database::nmea_database>         db_;
    std::unique_ptr<nmea::database::nmea_database_adapter> adapter_;

    mutable std::mutex  data_mutex_;
    gnss::nmea_data     data_{};
};

} // namespace gnss

#endif // NMEA_PARSER_ENABLED
