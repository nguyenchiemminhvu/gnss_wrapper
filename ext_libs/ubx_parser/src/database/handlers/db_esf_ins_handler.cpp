// src/database/handlers/db_esf_ins_handler.cpp
//
// Maps ubx_esf_ins fields to database data_field entries.
// Angular rates in [1e-3 deg/s], accelerations in [1e-3 m/s^2] (stored raw).

#include "database/handlers/db_esf_ins_handler.h"
#include "database/ubx_field_store.h"
#include "database/ubx_msg_mask.h"
#include "database/ubx_data_fields.h"

namespace ubx
{
namespace database
{

void db_esf_ins_handler::update(const parser::ubx_esf_ins& msg)
{
    msg_ = msg;
}

void db_esf_ins_handler::handle(ubx_field_store& store, ubx_msg_mask& mask)
{
    store.set(DATA_UBX_ESF_INS_ITOW,      msg_.i_tow);
    store.set(DATA_UBX_ESF_INS_BITFIELD0, msg_.bitfield0);

    // Angular rates: raw [1e-3 deg/s]; store raw for fidelity.
    store.set(DATA_UBX_ESF_INS_X_ANG_RATE, msg_.x_ang_rate);
    store.set(DATA_UBX_ESF_INS_Y_ANG_RATE, msg_.y_ang_rate);
    store.set(DATA_UBX_ESF_INS_Z_ANG_RATE, msg_.z_ang_rate);

    // Accelerations: raw [1e-3 m/s^2]; store raw for fidelity.
    store.set(DATA_UBX_ESF_INS_X_ACCEL, msg_.x_accel);
    store.set(DATA_UBX_ESF_INS_Y_ACCEL, msg_.y_accel);
    store.set(DATA_UBX_ESF_INS_Z_ACCEL, msg_.z_accel);

    mask.set(MSG_UBX_ESF_INS);
}

} // namespace database
} // namespace ubx
