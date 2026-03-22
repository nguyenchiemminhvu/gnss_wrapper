#include "global_constants.h"

namespace gnss
{

const char* DEFAULT_UART_DEVICE     = "/dev/ttyS3";
const char* DEFAULT_CONFIG_INI_PATH = "/etc/default_ubx_configs.ini";

// ── Command name token definitions ───────────────────────────────────────────

const char* CMD_INIT          = "init";
const char* CMD_START         = "start";
const char* CMD_STOP          = "stop";
const char* CMD_TERMINATE     = "terminate";
const char* CMD_CHECK_RUNNING = "check_running";

const char* CMD_BACKUP        = "backup";
const char* CMD_CLEAR_BACKUP  = "clear_backup";
const char* CMD_HOT_STOP      = "hot_stop";
const char* CMD_WARM_STOP     = "warm_stop";
const char* CMD_COLD_STOP     = "cold_stop";
const char* CMD_HOT_START     = "hot_start";
const char* CMD_WARM_START    = "warm_start";
const char* CMD_COLD_START    = "cold_start";
const char* CMD_GET_VERSION   = "get_version";
const char* CMD_SYNC_CONFIG   = "sync_config";

} // namespace gnss
