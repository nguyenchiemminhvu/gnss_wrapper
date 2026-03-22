#ifdef NMEA_PARSER_ENABLED

#include "nmea_base_database.h"

// nmea_base_database is an abstract class; all implementation lives in derived classes.
// This translation unit is intentionally minimal — it exists so the vtable and
// virtual destructor are emitted in a single object file rather than in every
// translation unit that includes nmea_base_database.h.

#endif // NMEA_PARSER_ENABLED
