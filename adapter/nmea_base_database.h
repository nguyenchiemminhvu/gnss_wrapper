#pragma once

#ifdef NMEA_PARSER_ENABLED

#include "global_constants.h"
#include "libevent.h"
#include "../shared_buffer/nmea_data_buffer.h"

namespace gnss
{

// ─── nmea_base_database ────────────────────────────────────────────────────────
//
// Abstract base for NMEA database wrappers.  Mirrors ubx_base_database.
//
// Provides:
//   1. Thread-safe read access to the nmea_data shared buffer.
//   2. A sigslot::signal<buffer_type> that derived classes emit after each
//      NMEA commit, allowing the Location Manager to subscribe to updates.
//
// Derived classes override get_nmea_data() with appropriate locking and are
// responsible for calling data_updated.emit(buffer_type::nmea_data) after each
// successful buffer refresh.
//
// Thread-safety: data_updated is a thread-safe sigslot signal; get_nmea_data()
// must be implemented with appropriate locking by derived classes.

class nmea_base_database
{
public:
    virtual ~nmea_base_database() = default;

    // ── Subscriber notification ───────────────────────────────────────────────
    //
    // Signal emitted whenever the NMEA data buffer is refreshed.
    // Location Manager connects its slot here before calling init():
    //
    //   class LocationManager : public sigslot::base_slot { ... };
    //   nmea_db.data_updated.connect(this, &LocationManager::on_nmea_data_updated);
    //
    sigslot::signal<gnss::buffer_type> data_updated;

    // ── Data accessor (thread-safe in derived classes) ────────────────────────
    virtual const gnss::nmea_data& get_nmea_data() const = 0;
};

} // namespace gnss

#endif // NMEA_PARSER_ENABLED
