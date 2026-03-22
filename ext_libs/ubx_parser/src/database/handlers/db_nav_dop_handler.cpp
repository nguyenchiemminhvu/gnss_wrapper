// src/database/handlers/db_nav_dop_handler.cpp

#include "database/handlers/db_nav_dop_handler.h"
#include "database/ubx_field_store.h"
#include "database/ubx_msg_mask.h"
#include "database/ubx_data_fields.h"

namespace ubx
{
namespace database
{

void db_nav_dop_handler::update(const parser::ubx_nav_dop& msg)
{
    msg_ = msg;
}

void db_nav_dop_handler::handle(ubx_field_store& store, ubx_msg_mask& mask)
{
    // Raw values scaled ×0.01 per spec; store raw values for fidelity —
    // consumers scale as needed (same as old CDatabase::getData behaviour).
    store.set(DATA_UBX_NAV_GEOMETRIC_DILUTION_OF_PRECISION, msg_.g_dop);
    store.set(DATA_UBX_NAV_POSITION_DILUTION_OF_PRECISION,  msg_.p_dop);
    store.set(DATA_UBX_NAV_TIME_DILUTION_OF_PRECISION,      msg_.t_dop);
    store.set(DATA_UBX_NAV_VERTICAL_DILUTION_OF_PRECISION,  msg_.v_dop);
    store.set(DATA_UBX_NAV_HORIZONAL_DILUTION_OF_PRECISION, msg_.h_dop);

    mask.set(MSG_UBX_NAV_DOP);
}

} // namespace database
} // namespace ubx
