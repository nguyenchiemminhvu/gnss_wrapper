#pragma once

#include "ubx_parser/include/config/i_ubx_transport.h"
#include "serial_helper/serial_port.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace gnss
{

// ─── serial_transport_impl ────────────────────────────────────────────────────
//
// Implements i_ubx_transport by forwarding outgoing UBX frames through a
// shared serial_port.  A transient serial_wrapper is constructed on each
// send() call so that the underlying fd-write is performed without holding
// any long-lived wrapper state.

class serial_transport_impl : public ubx::config::i_ubx_transport
{
public:
    explicit serial_transport_impl(std::shared_ptr<serial::serial_port> port);

    bool send(const std::vector<uint8_t>& msg) override;

private:
    std::shared_ptr<serial::serial_port> port_;
};

} // namespace gnss
