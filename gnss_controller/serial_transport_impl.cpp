#include "serial_transport_impl.h"

#include "serial_helper/serial_wrapper.h"
#include "serial_helper/serial_exception.h"

#include "ubx_parser/include/ubx_protocol.h"

#include "logging.h"

#include <iomanip>

namespace gnss
{

serial_transport_impl::serial_transport_impl(std::shared_ptr<serial::serial_port> port)
    : port_(port)
{}

bool serial_transport_impl::send_async(const std::vector<uint8_t>& msg)
{
    if (!port_)
    {
        LOG_ERROR("[serial_transport_impl] Serial port not initialized");
        return false;
    }
    try
    {
        serial::serial_wrapper w(port_);
        ssize_t written = w.write_bytes(msg);
        return (written == static_cast<ssize_t>(msg.size()));
    }
    catch (const serial::serial_error& e)
    {
        LOG_ERROR("[serial_transport_impl] Failed to send message over serial port: ", e.what());
        return false;
    }
}

bool serial_transport_impl::send_sync(const std::vector<uint8_t>& msg)
{
    // Precondition:
    //  1. Ensure ubx database signal is connected to the internal slot for ACK/NAK handling before sending any message.
    // Algorithm:
    //  1. verify if the frame is a completed and valid UBX message, using ubx_protocol::verify_frame_checksum()
    //  2. if valid:
    //     a. reset the toggle_event_ to unsignaled state
    //     b. send the message over UART using send_async()
    //        b1. if send fails, return false immediately
    //        b2. if send succeeds, proceed to wait for ACK/NAK response
    //     c. wait for toggle_event_ to be signaled, indicating the ACK/NAK response has been received
    //        c1. if ACK received, return true
    //        c2. if timeout occurs before toggle_event_ is signaled, return false
    //  3. if invalid, return false immediately without sending

    std::lock_guard<std::mutex> lock(send_mutex_);

    if (!ubx::parser::protocol::verify_frame_checksum(msg))
    {
        LOG_ERROR("[serial_transport_impl] Invalid UBX message: checksum verification failed");
        return false;
    }

    uint8_t cls_id = ubx::parser::protocol::get_frame_msg_class(msg);
    uint8_t msg_id = ubx::parser::protocol::get_frame_msg_id(msg);
    LOG_INFO("[serial_transport_impl] Sending message with class=", std::hex, std::setfill('0'), std::setw(2), static_cast<uint32_t>(cls_id), ", id=", std::setw(2), static_cast<uint32_t>(msg_id));

    ack_toggle_.reset();
    last_sent_class_id_ = cls_id;
    last_sent_msg_id_ = msg_id;
    if (!send_async(msg))
    {
        return false;
    }

    LOG_INFO("[serial_transport_impl] Message sent, waiting for ACK/NAK response...");
    if (!ack_toggle_.wait_for(gnss::UART_POLL_TIMEOUT_MS))
    {
        LOG_ERROR("[serial_transport_impl] Timeout waiting for ACK/NAK response");
        last_sent_class_id_ = 0xFFU;
        last_sent_msg_id_ = 0xFFU;
        return false;
    }

    return true;
}

void serial_transport_impl::handle_ack(uint8_t msg_class, uint8_t msg_id, bool is_ack)
{
    if (is_ack && msg_class == last_sent_class_id_ && msg_id == last_sent_msg_id_)
    {
        ack_toggle_.set();
    }
}

} // namespace gnss
