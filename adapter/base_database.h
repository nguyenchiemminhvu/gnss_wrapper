#pragma once

#include "global_constants.h"
#include "libevent.h"
#include "../shared_buffer/location_data_buffer.h"
#include "../shared_buffer/satellites_data_buffer.h"
#include "../shared_buffer/timing_data_buffer.h"

namespace gnss
{

// ─── base_database ─────────────────────────────────────────────────────────────
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

class base_database
{
public:
    virtual ~base_database() = default;

    // ── Subscriber notification ───────────────────────────────────────────────
    //
    // Signal emitted whenever a buffer is refreshed.
    // Location Manager connects its slot here:
    //
    //   class LocationManager : public sigslot::base_slot { ... };
    //   db.data_updated.connect(this, &LocationManager::on_data_updated);
    //
    sigslot::signal<gnss::buffer_type> data_updated;

    // ── Data accessors (thread-safe in derived classes) ───────────────────────
    virtual const location_data&   get_location()   const = 0;
    virtual const satellites_data& get_satellites() const = 0;
    virtual const timing_data&     get_timing()     const = 0;
};

} // namespace gnss
