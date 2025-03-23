#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "crater/core/Channel.hpp"
#include "crater/core/collections/utils/Hash.hpp"
#include "crater/core/io/Mavlink.hpp"
#include "crater/core/io/Reader.hpp"

namespace crt::io {
struct ChannelReaderKey {
    ::mavlink::msgid_t msg_id;
    uint8_t sys_id;
    uint8_t comp_id;

    bool operator==(const ChannelReaderKey& other) const {
        return msg_id == other.msg_id && sys_id == other.sys_id && comp_id == other.comp_id;
    }
};
} // namespace crt::io

namespace std {
template<>
struct hash<crt::io::ChannelReaderKey> {
    auto operator()(const crt::io::ChannelReaderKey& k) const -> size_t {
        size_t h = 0;

        crt::collections::hash_combine(h, k.msg_id);
        crt::collections::hash_combine(h, k.sys_id);
        crt::collections::hash_combine(h, k.comp_id);

        return h;
    }
};
}; // namespace std

namespace crt::io {

/**
 * @brief Encodes samples from a list of eterogeneous channels (but data type must be mavlink messages) to a single output stream
 */
class ChannelReader {
private:
    // Function receiving samples from a buffer and encoding them to the output stream
    using DispatchFn = std::function<bool(const ::mavlink::mavlink_message_t&)>;

public:
    ChannelReader(mavlink::MavlinkChannel mavlink_chan) : mavlink_chan_(mavlink_chan) {}

    /**
     * @brief Add a channel for encoding
     * 
     * @tparam T Must be a Mavlink message
     */
    template<typename T>
    void add_channel(Sender<T>&& sender, uint8_t sys_id, uint8_t comp_id) {
        std::shared_ptr<Sender<T>> tx_shared = std::make_shared<Sender<T>>(std::move(sender));

        ::mavlink::msgid_t msg_id = T::MSG_ID;

        auto dispatch_fn = [tx_shared](const ::mavlink::mavlink_message_t& raw_msg) {
            std::optional<T> msg = mavlink::mavlink_decode_msg<T>(raw_msg);
            if (!msg) {
                return false;
            }

            tx_shared->send(*msg);

            return true;
        };

        channels_.insert(
            {ChannelReaderKey {msg_id, sys_id, comp_id}, Channel {.dispatch_fn = dispatch_fn}}
        );
    }

    void process(Reader& reader);

    struct Channel {
        DispatchFn dispatch_fn;
    };

    std::array<uint8_t, crt::mavlink::MaxPacketLen> buf_ {};

    std::unordered_map<ChannelReaderKey, Channel> channels_;
    mavlink::MavlinkChannel mavlink_chan_;
};

} // namespace crt::io