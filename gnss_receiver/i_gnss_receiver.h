#pragma once

#include "io_helper/i_io_config.h"
#include "io_helper/i_io_port.h"
#include "ubx_parser/include/ubx_decoder_registry.h"
#include "ubx_parser/include/messages/ubx_raw_message.h"
#include "../shared_buffer/ubx_database_wrapper.h"

#ifdef NMEA_PARSER_ENABLED
#include "../shared_buffer/nmea_database_wrapper.h"
#endif

#include <functional>
#include <memory>
#include <string>
#include <iostream>

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

    /// Open and configure the I/O channel.  Must be called before start().
    /// @return true on success.
    virtual bool init(std::shared_ptr<io::i_io_config> config) = 0;

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

    /// Replace the underlying I/O port with a new one.
    ///
    /// Must only be called when the receiver is not running (i.e. after
    /// terminate() has returned the receiver to the idle state).  The caller
    /// is responsible for opening the new port and calling init() with the
    /// matching config before the next start().
    ///
    /// @param port  New I/O port to use for subsequent init()/start() cycles.
    virtual void set_port(std::shared_ptr<io::i_io_port> port) = 0;
};

} // namespace gnss
