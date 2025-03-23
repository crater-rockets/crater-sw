#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "crater/core/Channel.hpp"
#include "crater/core/io/Mavlink.hpp"
#include "crater/core/io/Writer.hpp"

namespace crt::io {

/**
 * @brief Encodes samples from a list of eterogeneous channels (but data type must be mavlink messages) to a single output stream
 */
class ChannelWriter {
  private:
    // Function receiving samples from a channel and encoding them to the output writer
    using ConsumeFn = std::function<
        void(io::Writer& writer, nonstd::span<uint8_t, crt::mavlink::MaxPacketLen> buf)>;

  public:
    ChannelWriter() {}

    /**
     * @brief Add a channel for encoding
     * 
     * @tparam T Must be a Mavlink message
     */
    template<typename T>
    void add_channel(Receiver<T>&& receiver, uint8_t sys_id, uint8_t comp_id) {
        std::shared_ptr<Receiver<T>> rx_shared = std::make_shared<Receiver<T>>(std::move(receiver));

        auto consume_fn =
            [rx_shared,
             sys_id,
             comp_id](io::Writer& writer, nonstd::span<uint8_t, crt::mavlink::MaxPacketLen> buf) {
                while (auto val = rx_shared->try_receive()) {
                    uint16_t len = crt::mavlink::mavlink_encode_msg(buf, *val, sys_id, comp_id);

                    writer.write(buf.first(len));
                }
            };
        channels_.push_back(Channel {.consume_fn = consume_fn});
    }

    void process(io::Writer& writer);

    struct Channel {
        ConsumeFn consume_fn;
    };

    std::array<uint8_t, crt::mavlink::MaxPacketLen> buf_ {};

    std::vector<Channel> channels_;
};

} // namespace crt::io