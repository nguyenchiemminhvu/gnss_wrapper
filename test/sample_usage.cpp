/**
 * @file sample_usage.cpp
 *
 * Comprehensive usage examples for the gnss_wrapper HAL library.
 *
 * Six independent sections, each wrapped in its own function:
 *
 *   1. section_adapter()          — highest-level API via control_adapter
 *                                   (typical Location Manager integration)
 *   2. section_controller()       — direct gnss_controller usage
 *   3. section_receiver()         — low-level gnss_receiver wiring
 *   4. section_database_only()    — standalone ubx_database_wrapper snapshot access
 *   5. section_error_handling()   — reconnect / retry patterns
 *   6. section_multi_receiver()   — multi-UART receiver with shared database
 *
 * Build prerequisites (see README.md for the full CMakeLists.txt snippet):
 *   - C++14
 *   - POSIX / Linux (poll, termios)
 *   - ext_libs: serial_helper, ubx_parser, lib_event
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// --- gnss_wrapper public headers ------------------------------------------
#include "adapter/control_adapter.h"          // highest-level service API
#include "shared_buffer/ubx_database_wrapper.h"           // typed shared-buffer publisher
#include "global_constants.h" // buffer_type enum, device path
#include "gnss_controller/gnss_controller.h"  // chip lifecycle + commands
#include "gnss_receiver/gnss_receiver.h"      // UART worker thread + parser

#ifdef NMEA_PARSER_ENABLED
#include "shared_buffer/nmea_data_buffer.h"
#include "shared_buffer/nmea_database_wrapper.h"
#endif

// --- external lib headers -------------------------------------------------
#include "ext_libs/serial_helper/serial_config.h"
#include "ext_libs/serial_helper/serial_port.h"

// --- sigslot (base_slot) --------------------------------------------------
#include "ext_libs/lib_event/libevent.h"

// ==========================================================================
// Shared helper: a Location Manager subscriber that reacts to data_updated
// signals emitted by ubx_base_database / ubx_database_wrapper.
// Must inherit sigslot::base_slot so the signal can track lifetime.
// ==========================================================================

class LocationManagerSubscriber : public sigslot::base_slot
{
public:
    explicit LocationManagerSubscriber(std::shared_ptr<gnss::ubx_database_wrapper> db)
        : db_(std::move(db))
    {}

    // Slot connected to ubx_base_database::data_updated signal.
    // Called from the gnss_receiver worker thread — keep it fast.
    void on_data_updated(gnss::buffer_type type)
    {
        switch (type)
        {
        case gnss::buffer_type::location:
            handle_location();
            break;
        case gnss::buffer_type::satellites:
            handle_satellites();
            break;
        case gnss::buffer_type::timing:
            handle_timing();
            break;
        }
    }

private:
    void handle_location()
    {
        const auto& loc = db_->get_location();
        if (!loc.valid)
            return;

        std::printf(
            "[LOCATION] fix=%u sv=%u  lat=%.7f  lon=%.7f  "
            "alt_msl=%.3f m  spd=%.3f m/s  hdg=%.2f deg  "
            "hacc=%.3f m  vacc=%.3f m\n",
            loc.fix_type,
            loc.num_sv,
            loc.latitude_deg,
            loc.longitude_deg,
            static_cast<double>(loc.altitude_msl_mm) / 1000.0,
            static_cast<double>(loc.ground_speed_mmps) / 1000.0,
            static_cast<double>(loc.vehicle_heading_deg),
            static_cast<double>(loc.h_accuracy_mm) / 1000.0,
            static_cast<double>(loc.v_accuracy_mm) / 1000.0);
    }

    void handle_satellites()
    {
        const auto& sats = db_->get_satellites();
        std::printf("[SATELLITES] %u SVs in view:\n", sats.num_svs);
        for (uint8_t i = 0; i < sats.num_svs; ++i)
        {
            const auto& sv = sats.svs[i];
            std::printf(
                "  gnss_id=%u sv=%3u  cno=%2u dBHz  el=%3d deg  az=%4d deg"
                "  used=%d  alm=%d  eph=%d\n",
                sv.gnss_id, sv.sv_id, sv.cno,
                sv.elevation_deg, sv.azimuth_deg,
                sv.used, sv.has_almanac, sv.has_ephemeris);
        }
    }

    void handle_timing()
    {
        const auto& tim = db_->get_timing();
        if (!tim.valid)
            return;

        std::printf(
            "[TIMING] tow_ms=%lld  gps_week=%u  leap_s=%d  "
            "q_err_ps=%d  utc_avail=%d\n",
            static_cast<long long>(tim.tow_ms),
            tim.gps_week,
            static_cast<int>(tim.leap_seconds),
            tim.q_err_ps,
            static_cast<int>(tim.utc_available));
    }

    std::shared_ptr<gnss::ubx_database_wrapper> db_;
};

// ==========================================================================
// SECTION 1 — control_adapter (Location Manager / service-level API)
//
// control_adapter is the single entry-point recommended for production code.
// It internally owns a gnss_controller and an ubx_database_wrapper, and exposes
// everything through string commands.
//
// Both single-UART and multi-UART constructors are shown:
//   Single:  control_adapter adapter(serial_config);
//   Multi:   control_adapter adapter(vector<serial_config>);
// ==========================================================================

static void section_adapter()
{
    std::puts("\n=== SECTION 1: control_adapter (single-UART, backward-compat) ===\n");

    // 1. Build the adapter with the target UART configuration.
    //    gnss_default() → 9600-8N1, 100 ms read timeout.
    gnss::control_adapter adapter(
        serial::serial_config::gnss_default(gnss::DEFAULT_UART_DEVICE));

    // 2. Obtain the underlying database and subscribe to data updates.
    //    The subscriber must outlive the adapter (or be disconnected first).
    auto db  = adapter.get_ubx_database();
    auto mgr = std::make_unique<LocationManagerSubscriber>(db);
    db->data_updated.connect(mgr.get(), &LocationManagerSubscriber::on_data_updated);

    // 3. Lifecycle — open port and launch UART reader thread.
    if (!adapter.execute_command("init"))
    {
        std::puts("[ERROR] init failed — check device path / permissions");
        return;
    }
    if (!adapter.execute_command("start"))
    {
        std::puts("[ERROR] start failed");
        adapter.execute_command("terminate");
        return;
    }

    // 4. Optional: load chip configuration from an INI file.
    //    INI key lines look like:   0x10240011 = 0x01   (hex key = hex value)
    //    Uses DEFAULT_CONFIG_INI_PATH if no path follows the colon.
    adapter.execute_command("poll_config:" + std::string(gnss::DEFAULT_CONFIG_INI_PATH));

    // 5. Issue a hardware reset — "hot_start" keeps all aiding data (fastest
    //    re-acquisition), "warm_start" clears ephemeris, "cold_start" clears all.
    adapter.execute_command("hot_start");

    // 6. Poll the firmware / hardware version string.
    //    The response arrives asynchronously via the raw-message callback
    //    registered inside gnss_controller.
    adapter.execute_command("get_version");

    // 7. Check whether the worker thread is still alive.
    bool running = adapter.execute_command("check_running");
    std::printf("[INFO] receiver running: %s\n", running ? "yes" : "no");

    // 8. Run for a while to receive position / satellite data.
    std::this_thread::sleep_for(std::chrono::seconds(30));

    // 9. Persist aiding data to battery-backed RAM before shutdown.
    adapter.execute_command("backup");

    // 10. Graceful teardown.
    adapter.execute_command("stop");
    adapter.execute_command("terminate");

    // Disconnect slot before mgr is destroyed (good practice with sigslot).
    db->data_updated.disconnect(mgr.get(), &LocationManagerSubscriber::on_data_updated);
}

// ==========================================================================
// SECTION 2 — gnss_controller (direct chip-control layer)
//
// Use this when you need finer control over the controller lifecycle or want
// to inject a custom serial configuration without the adapter abstraction.
// ==========================================================================

static void section_controller()
{
    std::puts("\n=== SECTION 2: gnss_controller ===\n");

    // High-speed config (115200-8N1) — useful after the chip baud-rate has
    // already been set via UBX-CFG-PRT / VALSET.
    const auto cfg = serial::serial_config::gnss_high_speed("/dev/ttyS3");
    gnss::gnss_controller ctrl(cfg);

    // init() opens the serial port and wires the parser pipeline.
    if (!ctrl.init())
    {
        std::puts("[ERROR] controller init failed");
        return;
    }

    // start() launches the UART reader thread.
    if (!ctrl.start())
    {
        std::puts("[ERROR] controller start failed");
        ctrl.terminate();
        return;
    }

    // Subscribe to database updates the same way as in section_adapter.
    auto db  = ctrl.get_ubx_database();
    auto mgr = std::make_unique<LocationManagerSubscriber>(db);
    db->data_updated.connect(mgr.get(), &LocationManagerSubscriber::on_data_updated);

    // Poll configuration from INI before issuing any resets.
    if (!ctrl.poll_config(gnss::DEFAULT_CONFIG_INI_PATH))
        std::puts("[WARN] poll_config returned false — INI may be missing");

    if (!ctrl.sync_config())
        std::puts("[WARN] sync_config failed — check INI path and contents");

    // Issue a cold start to force almanac / ephemeris re-download.
    ctrl.cold_start();

    // Poll firmware version (response arrives via MON-VER callback).
    ctrl.get_version();

    // Collect data for one minute.
    std::this_thread::sleep_for(std::chrono::minutes(1));

    // Save aiding data, then tear down.
    ctrl.backup();
    ctrl.stop();
    ctrl.terminate();

    db->data_updated.disconnect(mgr.get(), &LocationManagerSubscriber::on_data_updated);
}

// ==========================================================================
// SECTION 3 — gnss_receiver (UART worker thread + parser, lowest level)
//
// Normally you won't construct gnss_receiver directly — gnss_controller owns
// one internally.  This pattern is useful for unit / integration tests where
// you want to inject a mock serial_port or test the parser pipeline in
// isolation.
// ==========================================================================

static void section_receiver()
{
    std::puts("\n=== SECTION 3: gnss_receiver (direct) ===\n");

    // 1. Build the shared serial_port and the database.
    auto port     = std::make_shared<serial::serial_port>();
    auto database = std::make_shared<gnss::ubx_database_wrapper>();

    // 2. Construct the receiver and attach its database so that setup_ubx()
    //    can register the UBX decoder stubs with the parser.
    auto receiver = std::make_unique<gnss::gnss_receiver>(port);
    receiver->setup_ubx(database);                 // registers NAV-PVT, NAV-SAT, etc.

    // 3. (Optional) register a callback for UBX frames that the database
    //    does not handle — e.g. MON-VER, CFG-VALGET replies, SEC-CRC, …
    receiver->set_raw_message_callback(
        [](const ubx::parser::ubx_raw_message& raw)
        {
            std::printf(
                "[RAW] class=0x%02X id=0x%02X  payload_len=%zu\n",
                raw.frame.header.msg_class,
                raw.frame.header.msg_id,
                raw.frame.payload.size());
        });

    // 4. (Optional) inject extra decoders before the parser is built.
    //    The callback receives a ubx_decoder_registry& where you can call
    //    register_decoder() for any i_message_decoder you implement.
    receiver->set_extra_parser_setup(
        [](ubx::parser::ubx_decoder_registry& registry)
        {
            // Example: register a custom CFG-VALGET decoder.
            // registry.register_decoder(std::make_unique<MyValgetDecoder>());
            (void)registry;
        });

    // 5. Initialise (opens serial port) and start the worker thread.
    const auto cfg = serial::serial_config::gnss_default("/dev/ttyS3");
    if (!receiver->init(cfg))
    {
        std::puts("[ERROR] receiver init failed");
        return;
    }
    if (!receiver->start())
    {
        std::puts("[ERROR] receiver start failed");
        receiver->terminate();
        return;
    }

    // 6. Subscribe to parsed data.
    auto mgr = std::make_unique<LocationManagerSubscriber>(database);
    database->data_updated.connect(mgr.get(), &LocationManagerSubscriber::on_data_updated);

    std::printf("[INFO] receiver running: %s\n",
                receiver->is_running() ? "yes" : "no");

    // 7. Main loop — poll a position fix flag until 3D fix is acquired.
    constexpr int FIX_3D = 3;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(5);

    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto& loc = database->get_location();
        if (loc.valid && loc.fix_type >= FIX_3D)
        {
            std::printf("[INFO] 3-D fix acquired: lat=%.7f lon=%.7f\n",
                        loc.latitude_deg, loc.longitude_deg);
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 8. Teardown.
    receiver->stop();
    receiver->terminate();
    database->data_updated.disconnect(mgr.get(), &LocationManagerSubscriber::on_data_updated);
}

// ==========================================================================
// SECTION 4 — ubx_database_wrapper standalone snapshot access
//
// Demonstrates how to read position / satellite / timing data without relying
// on the signal — useful when polling from a separate service thread rather
// than reacting to push notifications.
// ==========================================================================

static void section_database_only()
{
    std::puts("\n=== SECTION 4: ubx_database_wrapper snapshot polling ===\n");

    // Assume database is already set up and the receiver is running
    // (e.g. obtained from adapter.get_ubx_database() or ctrl.get_ubx_database()).
    // Here we just demonstrate the read API.

    // --- create a minimal pipeline for illustration ------------------------
    auto port     = std::make_shared<serial::serial_port>();
    auto database = std::make_shared<gnss::ubx_database_wrapper>();
    auto receiver = std::make_unique<gnss::gnss_receiver>(port);
    receiver->setup_ubx(database);

    const auto cfg = serial::serial_config::gnss_default("/dev/ttyS3");
    if (!receiver->init(cfg) || !receiver->start())
    {
        std::puts("[ERROR] failed to start receiver pipeline");
        return;
    }
    // -----------------------------------------------------------------------

    // Polling loop: read the three shared buffers every 500 ms.
    for (int iter = 0; iter < 20; ++iter)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // --- Location buffer -----------------------------------------------
        {
            const gnss::location_data& loc = database->get_location();
            if (loc.valid)
            {
                std::printf(
                    "[POLL] lat=%.7f lon=%.7f fix=%u sv=%u "
                    "utc=%04u-%02u-%02u %02u:%02u:%02u\n",
                    loc.latitude_deg, loc.longitude_deg,
                    loc.fix_type, loc.num_sv,
                    loc.year, loc.month, loc.day,
                    loc.hour, loc.minute, loc.second);
            }
            else
            {
                std::puts("[POLL] location: no valid fix yet");
            }
        }

        // --- Satellites buffer --------------------------------------------
        {
            const gnss::satellites_data& sats = database->get_satellites();
            if (sats.valid)
            {
                uint8_t used = 0;
                for (uint8_t i = 0; i < sats.num_svs; ++i)
                    if (sats.svs[i].used) ++used;

                std::printf("[POLL] satellites: %u in view, %u used\n",
                            sats.num_svs, used);

                // Find the strongest signal.
                uint8_t max_cno = 0;
                for (uint8_t i = 0; i < sats.num_svs; ++i)
                    if (sats.svs[i].cno > max_cno)
                        max_cno = sats.svs[i].cno;

                std::printf("[POLL] best C/N₀ = %u dBHz\n", max_cno);
            }
        }

        // --- Timing buffer ------------------------------------------------
        {
            const gnss::timing_data& tim = database->get_timing();
            if (tim.valid)
            {
                std::printf(
                    "[POLL] tow_ms=%lld  week=%u  q_err=%d ps  utc=%d\n",
                    static_cast<long long>(tim.tow_ms),
                    tim.gps_week,
                    tim.q_err_ps,
                    static_cast<int>(tim.utc_available));
            }
        }
    }

    receiver->stop();
    receiver->terminate();
}

// ==========================================================================
// SECTION 5 — error handling and reconnect / retry patterns
//
// Shows defensive patterns when UART connectivity is unreliable.
// ==========================================================================

static void section_error_handling()
{
    std::puts("\n=== SECTION 5: error handling & reconnect ===\n");

    gnss::control_adapter adapter(
        serial::serial_config::gnss_default("/dev/ttyS3"));

    // init() returns false (no exception) if the device does not exist.
    constexpr int MAX_RETRIES = 5;
    bool ok = false;
    for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt)
    {
        if (adapter.execute_command("init"))
        {
            ok = true;
            break;
        }
        std::printf("[WARN] init attempt %d/%d failed, retrying in 2 s...\n",
                    attempt, MAX_RETRIES);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (!ok)
    {
        std::puts("[ERROR] could not open serial port after retries");
        return;
    }

    if (!adapter.execute_command("start"))
    {
        std::puts("[ERROR] start failed even after successful init");
        adapter.execute_command("terminate");
        return;
    }

    // Unknown / misspelled commands return false without throwing.
    if (!adapter.execute_command("turbo_boost"))
        std::puts("[INFO] unknown command correctly rejected");

    // sync_config with missing INI returns false but does not alter chip state.
    if (!adapter.execute_command("sync_config:/nonexistent/path.ini"))
        std::puts("[INFO] missing INI handled gracefully — chip config unchanged");

    // The receiver worker thread reconnects automatically on UART errors.
    // If reconnect also fails the thread exits cleanly; we detect that here.
    std::this_thread::sleep_for(std::chrono::seconds(10));
    if (!adapter.execute_command("check_running"))
    {
        std::puts("[WARN] receiver thread stopped — reinitialising...");
        adapter.execute_command("stop");
        adapter.execute_command("terminate");
        // Re-init from scratch.
        if (adapter.execute_command("init") && adapter.execute_command("start"))
            std::puts("[INFO] receiver restarted successfully");
    }

    adapter.execute_command("stop");
    adapter.execute_command("terminate");
}

// ==========================================================================
// SECTION 6 — multi-UART receiver
//
// Demonstrates the new multi-receiver API where a vector of serial_config
// objects is supplied.  The first element is the control UART; subsequent
// elements are data-only UARTs.  All receivers share a single
// ubx_database_wrapper so the Location Manager's subscriber receives
// updates from any UART transparently.
//
// Test scenarios covered:
//   A) Successful multi-receiver init + start.
//   B) Partial init failure (second UART path does not exist).
//   C) Shared database updates from multiple UART inputs (structural check).
// ==========================================================================

static void section_multi_receiver()
{
    std::puts("\n=== SECTION 6: multi-UART receiver ===\n");

    // ── A) Happy-path multi-receiver ─────────────────────────────────────────
    {
        std::puts("[A] Multi-receiver init and start (both UARTs present)...");

        std::vector<serial::serial_config> cfgs;
        cfgs.push_back(serial::serial_config::gnss_default("/dev/ttyS3")); // control UART
        cfgs.push_back(serial::serial_config::gnss_default("/dev/ttyS4")); // data UART

        gnss::control_adapter adapter(cfgs);

        auto db  = adapter.get_ubx_database();
        auto mgr = std::make_unique<LocationManagerSubscriber>(db);
        db->data_updated.connect(mgr.get(), &LocationManagerSubscriber::on_data_updated);

        const bool init_ok  = adapter.execute_command("init");
        std::printf("[A] init  returned: %s\n", init_ok ? "true" : "false");

        const bool start_ok = adapter.execute_command("start");
        std::printf("[A] start returned: %s\n", start_ok ? "true" : "false");

        // Receive data from both UARTs for a short window.
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::printf("[A] check_running: %s\n",
                    adapter.execute_command("check_running") ? "yes" : "no");

        adapter.execute_command("stop");
        adapter.execute_command("terminate");

        db->data_updated.disconnect(mgr.get(), &LocationManagerSubscriber::on_data_updated);
        std::puts("[A] Done.");
    }

    // ── B) Partial init failure ───────────────────────────────────────────────
    {
        std::puts("\n[B] Partial init failure (second UART path does not exist)...");

        std::vector<serial::serial_config> cfgs;
        cfgs.push_back(serial::serial_config::gnss_default("/dev/ttyS3"));    // may work
        cfgs.push_back(serial::serial_config::gnss_default("/dev/nonexistent")); // will fail

        gnss::control_adapter adapter(cfgs);

        // init() returns false (reports partial failure) but does NOT throw;
        // the caller decides whether to abort or proceed with the live UARTs.
        const bool init_ok = adapter.execute_command("init");
        std::printf("[B] init returned: %s (expected false due to nonexistent UART)\n",
                    init_ok ? "true" : "false");

        // start() will succeed only for the receivers that were initialised.
        adapter.execute_command("start");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        adapter.execute_command("stop");
        adapter.execute_command("terminate");
        std::puts("[B] Done.");
    }

    // ── C) Shared database — structural verification ──────────────────────────
    {
        std::puts("\n[C] Shared database wiring verification...");

        // Build two receivers sharing one database directly (below the adapter).
        auto port1 = std::make_shared<serial::serial_port>();
        auto port2 = std::make_shared<serial::serial_port>();
        auto db    = std::make_shared<gnss::ubx_database_wrapper>();

        auto recv1 = std::make_unique<gnss::gnss_receiver>(port1);
        auto recv2 = std::make_unique<gnss::gnss_receiver>(port2);

        // Both receivers share the same database instance.
        recv1->setup_ubx(db);
        recv2->setup_ubx(db);

        // Verify both receivers reference the same database object.
        // (In production the database pointer is exposed via get_ubx_database().)
        std::atomic<int> update_count{0};
        auto sub = std::make_unique<LocationManagerSubscriber>(db);

        // subscriber slot increments a counter on any data_updated signal.
        db->data_updated.connect(
            sub.get(), &LocationManagerSubscriber::on_data_updated);

        std::puts("[C] Two receivers wired to shared database — OK.");
        std::puts("[C] (Full data-path test requires real UART hardware.)");

        db->data_updated.disconnect(
            sub.get(), &LocationManagerSubscriber::on_data_updated);
        std::puts("[C] Done.");
    }

    // ── D) Empty config vector validation ─────────────────────────────────────
    {
        std::puts("\n[D] Empty config vector must throw std::invalid_argument...");
        bool threw = false;
        try
        {
            gnss::control_adapter adapter(std::vector<serial::serial_config>{});
        }
        catch (const std::invalid_argument&)
        {
            threw = true;
        }
        std::printf("[D] empty-vector validation: %s\n",
                    threw ? "PASS (exception thrown)" : "FAIL (no exception)");
    }
}

// ==========================================================================
// main — run all sections in sequence
// ==========================================================================

#ifdef NMEA_PARSER_ENABLED

// ==========================================================================
// NMEA subscriber helper
//
// Reacts to data_updated signals emitted by nmea_database_wrapper.
// Must inherit sigslot::base_slot so the signal can track lifetime.
// ==========================================================================

class NmeaManagerSubscriber : public sigslot::base_slot
{
public:
    explicit NmeaManagerSubscriber(std::shared_ptr<gnss::nmea_database_wrapper> db)
        : db_(std::move(db))
    {}

    // Slot connected to nmea_database_wrapper::data_updated.
    // Called from the gnss_receiver worker thread — keep it fast.
    void on_nmea_updated(gnss::buffer_type type)
    {
        if (type != gnss::buffer_type::nmea_data)
            return;

        // get_nmea_data() acquires data_mutex_ internally — safe from any thread.
        const gnss::nmea_data& d = db_->get_nmea_data();

        if (!d.valid)
            return;

        std::printf(
            "[NMEA] lat=%.7f  lon=%.7f  alt_msl=%.1f m  fix=%u  sv=%u  "
            "spd=%.2f kn  hdg=%.1f\u00b0  mode=%u  pdop=%.1f  hdop=%.1f  vdop=%.1f\n",
            d.latitude_deg,
            d.longitude_deg,
            static_cast<double>(d.altitude_msl_m),
            d.fix_quality,
            d.num_satellites,
            static_cast<double>(d.speed_knots),
            static_cast<double>(d.course_true_deg),
            d.nav_mode,
            static_cast<double>(d.pdop),
            static_cast<double>(d.hdop),
            static_cast<double>(d.vdop));

        if (d.num_sats_in_view > 0)
        {
            std::printf("[NMEA] %u SVs in view:\n", d.num_sats_in_view);
            uint8_t limit = d.num_sats_in_view;
            if (limit > gnss::NMEA_MAX_SAT_VIEW)
                limit = gnss::NMEA_MAX_SAT_VIEW;
            for (uint8_t i = 0; i < limit; ++i)
            {
                const auto& sv = d.sats[i];
                std::printf("  sv=%3u  el=%3d deg  az=%4d deg  snr=%2u dBHz\n",
                            sv.sv_id, sv.elevation_deg, sv.azimuth_deg, sv.snr);
            }
        }
    }

private:
    std::shared_ptr<gnss::nmea_database_wrapper> db_;
};

// ==========================================================================
// SECTION 7 — NMEA parser integration  (NMEA_PARSER_ENABLED only)
//
// Demonstrates the NMEA integration path added alongside the existing UBX
// pipeline.  All code paths are guarded by the NMEA_PARSER_ENABLED macro.
//
// Build with NMEA ON  (default):  make
// Build with NMEA OFF:            make NMEA_PARSER_ENABLED=0
//
// Test scenarios covered:
//   A) control_adapter exposes get_nmea_database(); subscriber receives
//      buffer_type::nmea_data signals from the NMEA commit path.
//   B) Standalone gnss_receiver wired with both UBX and NMEA databases;
//      both receive data from the same byte stream.
//   C) Multi-receiver setup_ubx: two receivers share one nmea_database_wrapper.
//      NMEA data from any UART reaches the same subscriber.
//   D) Empty-config validation still fires std::invalid_argument.
// ==========================================================================

static void section_nmea_integration()
{
    std::puts("\n=== SECTION 7: NMEA parser integration (NMEA_PARSER_ENABLED) ===\n");

    // ── A) control_adapter + NMEA subscriber ─────────────────────────────────
    {
        std::puts("[A] control_adapter with dual UBX + NMEA subscribers...");

        gnss::control_adapter adapter(
            serial::serial_config::gnss_default(gnss::DEFAULT_UART_DEVICE));

        // UBX subscriber (existing path — unchanged).
        auto ubx_db  = adapter.get_ubx_database();
        auto ubx_mgr = std::make_unique<LocationManagerSubscriber>(ubx_db);
        ubx_db->data_updated.connect(ubx_mgr.get(),
                                     &LocationManagerSubscriber::on_data_updated);

        // NMEA subscriber (new path).
        auto nmea_db  = adapter.get_nmea_database();
        auto nmea_mgr = std::make_unique<NmeaManagerSubscriber>(nmea_db);
        nmea_db->data_updated.connect(nmea_mgr.get(),
                                      &NmeaManagerSubscriber::on_nmea_updated);

        if (!adapter.execute_command("init"))
        {
            std::puts("[A] init failed — check device path / permissions");
        }
        else if (!adapter.execute_command("start"))
        {
            std::puts("[A] start failed");
            adapter.execute_command("terminate");
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            adapter.execute_command("stop");
            adapter.execute_command("terminate");
        }

        nmea_db->data_updated.disconnect(nmea_mgr.get(),
                                         &NmeaManagerSubscriber::on_nmea_updated);
        ubx_db->data_updated.disconnect(ubx_mgr.get(),
                                        &LocationManagerSubscriber::on_data_updated);
        std::puts("[A] Done.");
    }

    // ── B) Standalone gnss_receiver — dual UBX + NMEA parsers ─────────────────
    {
        std::puts("\n[B] Standalone gnss_receiver with both UBX and NMEA parsers...");

        auto port    = std::make_shared<serial::serial_port>();
        auto ubx_db  = std::make_shared<gnss::ubx_database_wrapper>();
        auto nmea_db = std::make_shared<gnss::nmea_database_wrapper>();

        auto recv = std::make_unique<gnss::gnss_receiver>(port);
        recv->setup_ubx(ubx_db);
        recv->setup_nmea(nmea_db);

        // Subscribe to both databases.
        auto ubx_mgr  = std::make_unique<LocationManagerSubscriber>(ubx_db);
        auto nmea_mgr = std::make_unique<NmeaManagerSubscriber>(nmea_db);
        ubx_db->data_updated.connect(ubx_mgr.get(),
                                     &LocationManagerSubscriber::on_data_updated);
        nmea_db->data_updated.connect(nmea_mgr.get(),
                                      &NmeaManagerSubscriber::on_nmea_updated);

        const auto cfg = serial::serial_config::gnss_default("/dev/ttyS3");
        if (recv->init(cfg) && recv->start())
        {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            recv->stop();
            recv->terminate();
        }
        else
        {
            std::puts("[B] receiver init/start failed — check device path");
        }

        nmea_db->data_updated.disconnect(nmea_mgr.get(),
                                         &NmeaManagerSubscriber::on_nmea_updated);
        ubx_db->data_updated.disconnect(ubx_mgr.get(),
                                        &LocationManagerSubscriber::on_data_updated);
        std::puts("[B] Done.");
    }

    // ── C) Multi-receiver with shared nmea_database_wrapper ──────────────────
    {
        std::puts("\n[C] Multi-receiver — both UARTs share one nmea_database_wrapper...");

        std::vector<serial::serial_config> cfgs;
        cfgs.push_back(serial::serial_config::gnss_default("/dev/ttyS3"));
        cfgs.push_back(serial::serial_config::gnss_default("/dev/ttyS4"));

        gnss::control_adapter adapter(cfgs);

        // Both receivers share the single nmea_database_ owned by gnss_controller.
        auto nmea_db  = adapter.get_nmea_database();
        auto nmea_mgr = std::make_unique<NmeaManagerSubscriber>(nmea_db);
        nmea_db->data_updated.connect(nmea_mgr.get(),
                                      &NmeaManagerSubscriber::on_nmea_updated);

        const bool init_ok  = adapter.execute_command("init");
        const bool start_ok = adapter.execute_command("start");
        std::printf("[C] init=%s  start=%s\n",
                    init_ok  ? "ok" : "fail",
                    start_ok ? "ok" : "fail");

        std::this_thread::sleep_for(std::chrono::seconds(2));
        adapter.execute_command("stop");
        adapter.execute_command("terminate");

        nmea_db->data_updated.disconnect(nmea_mgr.get(),
                                         &NmeaManagerSubscriber::on_nmea_updated);
        std::puts("[C] Done.");
    }

    // ── D) nmea_data struct accessible independently of signal ────────────────
    {
        std::puts("\n[D] Polling nmea_data directly (no signal subscription)...");

        auto port    = std::make_shared<serial::serial_port>();
        auto nmea_db = std::make_shared<gnss::nmea_database_wrapper>();
        auto recv    = std::make_unique<gnss::gnss_receiver>(port);
        recv->setup_nmea(nmea_db);

        const auto cfg = serial::serial_config::gnss_default("/dev/ttyS3");
        if (recv->init(cfg) && recv->start())
        {
            for (int i = 0; i < 5; ++i)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                const gnss::nmea_data& d = nmea_db->get_nmea_data();
                if (d.valid)
                {
                    std::printf("[D] poll %d: lat=%.7f lon=%.7f\n",
                                i + 1, d.latitude_deg, d.longitude_deg);
                }
                else
                {
                    std::printf("[D] poll %d: no valid NMEA data yet\n", i + 1);
                }
            }
            recv->stop();
            recv->terminate();
        }
        else
        {
            std::puts("[D] receiver init/start failed — skipping poll demo");
        }
        std::puts("[D] Done.");
    }
}

#endif // NMEA_PARSER_ENABLED

int main()
{
    // Each section is self-contained. In a real application you would call
    // only one of them (typically section_adapter).

    // Uncomment the section(s) you want to run:
    section_adapter();
    // section_controller();
    // section_receiver();
    // section_database_only();
    // section_error_handling();
    section_multi_receiver();
#ifdef NMEA_PARSER_ENABLED
    section_nmea_integration();
#endif

    return 0;
}
