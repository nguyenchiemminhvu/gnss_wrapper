#pragma once

#include "global_constants.h"
#include "libevent.h"
#include "../shared_buffer/location_data_buffer.h"
#include "../shared_buffer/satellites_data_buffer.h"
#include "../shared_buffer/timing_data_buffer.h"
#include "../shared_buffer/attitude_data_buffer.h"
#include "../shared_buffer/dop_data_buffer.h"
#include "../shared_buffer/dop_data_buffer.h"
#include "../shared_buffer/raw_measurements_data_buffer.h"
#include "../shared_buffer/monitor_data_buffer.h"
#include "../shared_buffer/security_data_buffer.h"
#include "../shared_buffer/datum_data_buffer.h"
#include "../shared_buffer/dead_reckoning_data_buffer.h"

namespace gnss
{

// ─── ubx_base_database ─────────────────────────────────────────────────────────────
//
// Abstract base providing:
//   1. Thread-safe read access to the shared_buffer types.
//   2. A sigslot::signal<buffer_type> that derived classes emit after updating
//      a buffer, allowing Location Manager to subscribe to incremental updates.
//
// Derived classes override the const getters and are responsible for calling
// data_updated.emit(type) after each successful buffer refresh.
//
// Thread-safety: data_updated is a thread-safe sigslot signal; the getters must
// be implemented with appropriate locking by derived classes.

class ubx_base_database
{
public:
    virtual ~ubx_base_database() = default;

    // ── Subscriber notification ───────────────────────────────────────────────
    //
    // Signal emitted whenever a buffer is refreshed.
    // Location Manager connects its slot here:
    //
    //   class LocationManager : public sigslot::base_slot { ... };
    //   db.data_updated.connect(this, &LocationManager::on_data_updated);
    //
    sigslot::signal<gnss::buffer_type> data_updated; // output data flow

    // ── Data accessors (thread-safe in derived classes) ───────────────────────
    virtual const gnss::location_data&              get_location()           const = 0;
    virtual const gnss::satellites_data&            get_satellites()         const = 0;
    virtual const gnss::timing_data&                get_timing()             const = 0;
    virtual const gnss::attitude_data&              get_attitude()           const = 0;
    virtual const gnss::dop_data&                   get_dop()                const = 0;
    virtual const gnss::raw_measurements_data&      get_raw_measurements()   const = 0;
    virtual const gnss::monitor_data&               get_monitor()            const = 0;
    virtual const gnss::security_data&              get_security()           const = 0;
    virtual const gnss::dead_reckoning_data_out&    get_dead_reckoning_out() const = 0;
    virtual const gnss::dead_reckoning_data_in&     get_dead_reckoning_in()  const = 0;

    // ── Data update methods ───────────────────────────────────────────────────
    virtual void update_dead_reckoning_in(const gnss::dead_reckoning_data_in& dr_input_params) = 0;
};

} // namespace gnss
