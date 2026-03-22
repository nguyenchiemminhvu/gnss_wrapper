#pragma once

#include "ubx_parser/include/config/i_ubx_transport.h"
#include "io_helper/i_io_port.h"

#include "libevent.h"
#include "global_constants.h"

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace gnss
{

// ─── serial_transport_impl ────────────────────────────────────────────────────
//
// Implements i_ubx_transport by forwarding outgoing UBX frames through a
// shared serial_port.  A transient serial_wrapper is constructed on each
// send() call so that the underlying fd-write is performed without holding
// any long-lived wrapper state.

class serial_transport_impl
    : public ubx::config::i_ubx_transport
    , public event_handler::event_handler
{
public:
    explicit serial_transport_impl(std::shared_ptr<io::i_io_port> port);

    bool send_async(const std::vector<uint8_t>& msg) override;

    bool send_sync(const std::vector<uint8_t>& msg) override;
    void handle_ack(uint8_t msg_class, uint8_t msg_id, bool is_ack) override;

private:
    std::shared_ptr<io::i_io_port> port_;
    toggle_event::toggle_event ack_toggle_;
    std::mutex send_mutex_;
    uint8_t last_sent_class_id_ = 0xFFU;
    uint8_t last_sent_msg_id_ = 0xFFU;
};

} // namespace gnss
