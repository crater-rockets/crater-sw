#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "crater/core/Channel.hpp"
#include "crater/core/hal/OutputStream.hpp"
#include "crater/telemetry/Mavlink.hpp"

namespace crt::io {

/**
 * @brief Encodes samples from a list of etherogeneous channels (data type must be mavlink messages) to a single output stream
 */
class ChannelToOutputStream {
  private:
    // Function receiving samples from a buffer and encoding them to the output stream
    using ConsumeFn = std::function<
        void(hal::OutputStream& os, nonstd::span<uint8_t, crt::mavlink::MavlinkMaxPacketLen> buf)>;

  public:
    ChannelToOutputStream(std::shared_ptr<hal::OutputStream> os) : ostream_(os) {}

    /**
     * @brief Add a channel for encoding
     * 
     * @tparam T Must be a Mavlink message
     */
    template<typename T>
    void add_channel(Receiver<T>&& receiver, uint8_t sys_id, uint8_t comp_id) {
        std::shared_ptr<Receiver<T>> rx_shared = std::make_shared<Receiver<T>>(std::move(receiver));

        auto consume_fn = [rx_shared, sys_id, comp_id](
                              hal::OutputStream& os,
                              nonstd::span<uint8_t, crt::mavlink::MavlinkMaxPacketLen> buf
                          ) {
            while (auto val = rx_shared->try_receive()) {
                uint16_t len = crt::mavlink::mavlink_encode_msg(buf, *val, sys_id, comp_id);

                os.write(buf.first(len));
            }
        };
        channels_.push_back(Channel {.consume_fn = consume_fn});
    }

    void process();

    struct Channel {
        ConsumeFn consume_fn;
    };

    std::array<uint8_t, crt::mavlink::MavlinkMaxPacketLen> buf_ {};

    std::shared_ptr<hal::OutputStream> ostream_;
    std::vector<Channel> channels_;
};

} // namespace crt::io