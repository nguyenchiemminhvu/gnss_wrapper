#pragma once

#include "ubx_parser/include/config/i_ini_config_provider.h"

#include <string>
#include <vector>

namespace gnss
{

// ─── ini_config_provider_impl ─────────────────────────────────────────────────
//
// Loads a named INI configuration file via ini_parser and maps the entries
// to UBX configuration key IDs using ubx_cfg_key_registry.
//
// The INI file supports both flat and sectioned key layouts:
//
//   Flat (key name matches registry directly):
//     rate_meas = 1000
//     uart1_baudrate = 115200
//
//   Sectioned (section + key joined by '_' for registry lookup):
//     [rate]
//     rate_meas = 1000          ; stored as "rate_meas" in registry
//
//     [uart1]
//     uart1_baudrate = 115200   ; stored as "uart1_baudrate" in registry
//
// Values can be only integer type (1000 for example). Keys unknown to ubx_cfg_key_registry are silently skipped.

class ini_config_provider_impl : public ubx::config::i_ini_config_provider
{
public:
    bool load(const std::string& path) override;
    std::vector<ubx::config::config_entry> get_all_entries() const override;

private:
    std::vector<ubx::config::config_entry> entries_;
};

} // namespace gnss
