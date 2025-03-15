#pragma once


#include <mavlink/crater/crater.hpp>
#include <nonstd/span.hpp>
#include <optional>

namespace crt::mavlink {

template<typename MsgT>
static void mavlink_encode_msg(
    nonstd::span<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer,
    const MsgT& message,
    uint8_t sys_id,
    uint8_t comp_id
) {
    ::mavlink::mavlink_message_t raw_msg {};
    ::mavlink::MsgMap map(&raw_msg);
    message.serialize(map);

    ::mavlink::mavlink_finalize_message(
        &raw_msg,
        sys_id,
        comp_id,
        MsgT::MIN_LENGTH,
        MsgT::LENGTH,
        MsgT::CRC_EXTRA
    );
    ::mavlink::mavlink_msg_to_send_buffer(buffer.data(), &raw_msg);
}

template<typename MsgT>
static std::optional<MsgT> mavlink_decode_msg(const ::mavlink::mavlink_message_t& raw_msg) {
    if (raw_msg.msgid == MsgT::MSG_ID) {
        ::mavlink::MsgMap map(&raw_msg);
        MsgT msg {};
        msg.deserialize(map);

        return msg;
    } else {
        return std::nullopt;
    }
}

} // namespace crt::mavlink