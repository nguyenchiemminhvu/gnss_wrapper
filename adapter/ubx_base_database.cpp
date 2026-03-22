#include "ubx_base_database.h"

// ubx_base_database is an abstract class; all implementation lives in derived classes.
// This translation unit is intentionally minimal — it exists so the vtable and
// virtual destructor are emitted in a single object file rather than in every
// translation unit that includes ubx_ubx_base_database.h.
