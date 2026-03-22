#pragma once

#include "serial_helper/serial_config.h"
#include "ubx_parser/include/ubx_decoder_registry.h"
#include "ubx_parser/include/messages/ubx_raw_message.h"
#include "../shared_buffer/ubx_database_wrapper.h"

#ifdef NMEA_PARSER_ENABLED
#include "../shared_buffer/nmea_database_wrapper.h"
#endif

#include <functional>
#include <memory>
#include <string>

namespace gnss
{

// ─── i_gnss_receiver ──────────────────────────────────────────────────────────
//
// Interface for the UART receiver component.
// Implementations own the serial port and UBX parser worker thread.

class i_gnss_receiver
{
public:
    virtual ~i_gnss_receiver() = default;

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /// Open and configure the serial port.  Must be called before start().
    /// @return true on success.
    virtual bool init(const serial::serial_config& config) = 0;

    /// Build the UBX parser and start the UART reader thread.
    /// Requires init() to have succeeded.
    /// @return true if the thread was launched.
    virtual bool start() = 0;

    /// Stop the UART reader thread.  The serial port remains open.
    /// After stop(), start() may be called again.
    virtual void stop() = 0;

    /// Stop the reader thread and close the serial port.
    /// Returns the receiver to the state before init() was called.
    virtual void terminate() = 0;

    /// @return true if the reader worker thread is currently running.
    virtual bool is_running() const = 0;

    // ── Parser wiring ─────────────────────────────────────────────────────────

    /// Register a database adapter so that nav-message decoders are installed
    /// into the parser registry before each start().
    /// Must be called before the first start().
    virtual void setup_ubx(std::shared_ptr<gnss::ubx_database_wrapper> ubx_db) = 0;

#ifdef NMEA_PARSER_ENABLED
    /// Wire the shared nmea_database_wrapper so that NMEA sentence handler stubs
    /// are registered into the parser registry on every start().
    /// Must be called before the first start().
    virtual void setup_nmea(std::shared_ptr<gnss::nmea_database_wrapper> nmea_db) = 0;
#endif

    /// Register an arbitrary extra decoder setup_ubx callback that is invoked on
    /// every start() before the parser is created.  Use this to add decoders
    /// for non-navigation messages (e.g. CFG-VALGET response).
    virtual void set_extra_parser_setup(
        std::function<void(ubx::parser::ubx_decoder_registry&)> cb) = 0;

    /// Set a callback for UBX frames that have no registered decoder.
    /// The callback is installed onto the parser after each start().
    virtual void set_raw_message_callback(
        ubx::parser::raw_message_callback_t cb) = 0;
};

} // namespace gnss
