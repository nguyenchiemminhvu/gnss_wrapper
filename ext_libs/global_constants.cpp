#include "global_constants.h"

namespace gnss
{

const char* DEFAULT_UART_DEVICE     = "/dev/ttyS3";
const char* DEFAULT_CONFIG_INI_PATH = "/etc/default_ubx_configs.ini";
const char* DEFAULT_UBX_RECORD_PATH = "/var/log/gnss/record.ubx";
const char* DEFAULT_RESET_N_GPIO_PATH = "/sys/class/gpio/gpio0/value";

// ── Command name token definitions ───────────────────────────────────────────

const char* CMD_HARDWARE_RESET = "hardware_reset";

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
const char* CMD_HOT_RESET     = "hot_reset";
const char* CMD_WARM_RESET    = "warm_reset";
const char* CMD_COLD_RESET    = "cold_reset";
const char* CMD_GET_VERSION   = "get_version";
const char* CMD_POLL_CONFIG   = "poll_config";
const char* CMD_SYNC_CONFIG   = "sync_config";

const char* CMD_QUERY_DATUM     = "query_datum";
const char* CMD_SET_DATUM       = "set_datum";
const char* CMD_SET_DATUM_WGS84 = "set_datum 1";
const char* CMD_SET_DATUM_PZ90  = "set_datum 2";
const char* CMD_RESET_DATUM     = "reset_datum";

const char* CMD_START_RECORD = "start_record";
const char* CMD_STOP_RECORD  = "stop_record";
const char* CMD_START_REPLAY = "start_replay";
const char* CMD_STOP_REPLAY  = "stop_replay";

} // namespace gnss
