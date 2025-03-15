#include "crater/core/io/ChannelToOutputStream.hpp"

namespace crt::io {

void ChannelToOutputStream::process() {
    for (Channel& ch : channels_) {
        ch.consume_fn(*ostream_, buf_);
    }
}

} // namespace crt::io