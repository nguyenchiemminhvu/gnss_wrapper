#include "gpio_pin_impl.h"

#include "logging.h"

#include <fstream>
#include <sstream>

namespace gnss
{

// ─── gpio_pin_impl ────────────────────────────────────────────────────────────

gpio_pin_impl::gpio_pin_impl(std::string gpio_path, std::string pin_name)
    : gpio_path_(std::move(gpio_path))
    , name_(std::move(pin_name))
{
}

bool gpio_pin_impl::write_value(bool high)
{
    std::lock_guard<std::mutex> lk(mutex_);

    std::ofstream fs(gpio_path_);
    if (!fs.is_open())
    {
        LOG_ERROR("[gpio_pin_impl] [", name_, "] failed to open GPIO path for write: ", gpio_path_);
        return false;
    }

    fs << (high ? '1' : '0') << '\n';

    if (!fs.good())
    {
        LOG_ERROR("[gpio_pin_impl] [", name_, "] write error on GPIO path: ", gpio_path_);
        return false;
    }

    LOG_DEBUG("[gpio_pin_impl] [", name_, "] set to ", (high ? "HIGH" : "LOW"));
    return true;
}

bool gpio_pin_impl::read_value(bool& out_high) const
{
    std::lock_guard<std::mutex> lk(mutex_);

    std::ifstream fs(gpio_path_);
    if (!fs.is_open())
    {
        LOG_ERROR("[gpio_pin_impl] [", name_, "] failed to open GPIO path for read: ", gpio_path_);
        return false;
    }

    char ch = '\0';
    fs >> ch;

    if (!fs.good() && !fs.eof())
    {
        LOG_ERROR("[gpio_pin_impl] [", name_, "] read error on GPIO path: ", gpio_path_);
        return false;
    }

    if (ch != '0' && ch != '1')
    {
        LOG_ERROR("[gpio_pin_impl] [", name_, "] unexpected GPIO value '", ch, "' from: ", gpio_path_);
        return false;
    }

    out_high = (ch == '1');
    return true;
}

const std::string& gpio_pin_impl::name() const
{
    return name_;
}

} // namespace gnss
