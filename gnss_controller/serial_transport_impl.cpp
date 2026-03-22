#include "serial_transport_impl.h"

#include "serial_helper/serial_wrapper.h"
#include "serial_helper/serial_exception.h"

namespace gnss
{

serial_transport_impl::serial_transport_impl(std::shared_ptr<serial::serial_port> port)
    : port_(port)
{}

bool serial_transport_impl::send(const std::vector<uint8_t>& msg)
{
    if (!port_)
    {
        return false;
    }
    try
    {
        serial::serial_wrapper w(port_);
        ssize_t written = w.write_bytes(msg);
        return (written == static_cast<ssize_t>(msg.size()));
    }
    catch (const serial::serial_error&)
    {
        return false;
    }
}

} // namespace gnss
