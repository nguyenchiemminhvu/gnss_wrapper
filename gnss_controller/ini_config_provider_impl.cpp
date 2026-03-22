#include "ini_config_provider_impl.h"

#include "ini_parser.h"
#include "ubx_parser/include/config/ubx_cfg_key_registry.h"
#include "ubx_parser/include/config/ubx_config_types.h"

#include <stdexcept>
#include <string>

namespace gnss
{

bool ini_config_provider_impl::load(const std::string& path)
{
    entries_.clear();

    ini_parser::ini_parser parser;
    try
    {
        parser.load(path);
    }
    catch (const ini_parser::config_error&)
    {
        return false;
    }

    std::vector<std::string> all_keys = parser.get_keys();
    uint32_t key_id = 0;
    for (const std::string& key : all_keys)
    {
        try
        {
            // Strip the "section." prefix produced by the INI parser for
            // sectioned keys while keeping flat keys unchanged (e.g. "uart1_baudrate" → "uart1_baudrate" or "rate.rate_meas" → "rate_meas").
            const std::string::size_type dot_pos = key.find('.');
            const std::string flat_key = (dot_pos != std::string::npos)
                ? key.substr(dot_pos + 1)
                : key;

            ini_parser::config_value value = parser.get(key);
            if (ubx::config::ubx_cfg_key_registry::lookup_by_name(flat_key.c_str(), key_id))
            {
                ubx::config::config_entry entry {
                    .key_id = key_id,
                    .value  = ubx::config::config_value(value.as_int()),
                };
                entries_.push_back(entry);
            }
        }
        catch (const ini_parser::config_error& e)
        {
            // Log error and skip invalid entry
            // (In production code, consider using a logging framework instead of std::cerr)
        }
    }
    return true;
}

std::vector<ubx::config::config_entry>
ini_config_provider_impl::get_all_entries() const
{
    return entries_;
}

} // namespace gnss
