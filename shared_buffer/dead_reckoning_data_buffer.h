// shared_buffer/dead_reckoning_data_buffer.h
//
// Dead Reckoning output data buffer.
//
// Aggregates compensated IMU data (UBX-ESF-INS) and sensor fusion status
// (UBX-ESF-STATUS) reported by the u-blox GNSS chipset.  The chip performs
// dead-reckoning internally; this buffer carries the chip's outputs to the
// higher-level location service layers.
//
// Field conventions:
//   • Angular rates  : degrees per second [deg/s]
//   • Accelerations  : metres per second squared [m/s^2]
//   • Times          : GPS time of week in milliseconds [ms]
//   • fusion_mode    : 0=init, 1=fusion, 2=suspended, 3=disabled
//   • calib_status   : 0=not calibrated, 1=calibrating, 2/3=calibrated
//   • time_status    : 0=no time, 1=first byte, 2=event, 3=time tag

#pragma once

#include <cstdint>

namespace gnss
{

/// Maximum number of ESF sensor records stored in dead_reckoning_data_out.
static constexpr uint8_t GNSS_MAX_DR_SENSORS = 32u;

// ─── dr_sensor_out_record ─────────────────────────────────────────────────────────
//
// Per-sensor calibration and status information from UBX-ESF-STATUS.

struct dr_sensor_out_record
{
    uint8_t type;          ///< Sensor type (matches ubx::parser::esf_sensor_type values)
    bool    used;          ///< Sensor data used in navigation solution
    bool    ready;         ///< Sensor data ready to be used
    uint8_t calib_status;  ///< Calibration status (0=not calibrated, 1=calibrating, 2/3=calibrated)
    uint8_t time_status;   ///< Time tag status (0=no time, 1=first byte, 2=event, 3=time tag)
    uint8_t freq;          ///< Received sensor data frequency [Hz]
    uint8_t faults;        ///< Sensor fault flags
};

// ─── dr_sensor_in_record ─────────────────────────────────────────────────────────
//
// Per-sensor measurement record from UBX-ESF-MEAS.

struct dr_sensor_in_record
{
    uint8_t type;          ///< Sensor type (matches ubx::parser::esf_sensor_type values)
    double  value;         ///< Sensor measurement value (units depend on sensor type; see UBX-ESF-MEAS documentation)
};

// ─── dead_reckoning_data_out ──────────────────────────────────────────────────────
//
// Aggregated dead reckoning output populated from UBX-ESF-INS (compensated
// angular rates and accelerations) and UBX-ESF-STATUS (fusion mode and
// per-sensor calibration).

struct dead_reckoning_data_out
{
    // ── UBX-ESF-INS: compensated IMU outputs ─────────────────────────────────
    uint32_t ins_i_tow_ms;    ///< GPS time of week [ms]
    uint32_t ins_bitfield0;   ///< Validity bitfield (see ubx_esf_ins.h BF0_* masks)
    double   x_ang_rate_dps;  ///< X-axis angular rate [deg/s]
    double   y_ang_rate_dps;  ///< Y-axis angular rate [deg/s]
    double   z_ang_rate_dps;  ///< Z-axis angular rate [deg/s]
    double   x_accel_mss;     ///< X-axis acceleration [m/s^2]
    double   y_accel_mss;     ///< Y-axis acceleration [m/s^2]
    double   z_accel_mss;     ///< Z-axis acceleration [m/s^2]
    bool     ins_valid;       ///< True when ESF-INS fields are populated

    // ── UBX-ESF-STATUS: sensor fusion mode and calibration ───────────────────
    uint32_t         status_i_tow_ms;            ///< GPS time of week [ms]
    uint8_t          fusion_mode;                ///< Fusion mode (0=init, 1=fusion, 2=suspended, 3=disabled)
    uint8_t          num_sens;                   ///< Number of sensor blocks in payload
    dr_sensor_out_record sensors[GNSS_MAX_DR_SENSORS]; ///< Per-sensor info; valid entries: [0, num_sens)
    bool             status_valid;               ///< True when ESF-STATUS fields are populated

    // ── Aggregate validity ────────────────────────────────────────────────────
    bool valid; ///< True when at least one of ins_valid or status_valid is true
};

// ─── dead_reckoning_data_in ───────────────────────────────────────────────────────
//
// Input structure for dead reckoning sensor measurements.

struct dead_reckoning_data_in
{
    // UBX-ESF-MEAS fields
    uint32_t time_tag_ms;  ///< External sensor data time tag [ms]
    bool wheeltick_dir_backward; ///< Direction of wheeltick sensor (true=reverse, false=forward)
    uint8_t num_sens; ///< Number of sensor blocks in payload
    dr_sensor_in_record sensors[GNSS_MAX_DR_SENSORS]; ///< Per-sensor measurements; valid entries: [0, num_sens)
};

} // namespace gnss
