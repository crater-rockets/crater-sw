#include "crater/main/io/ChannelWriter.hpp"

namespace crt::io {

void ChannelWriter::process(io::Writer& writer) {
    for (Channel& ch : channels_) {
        ch.consume_fn(writer, buf_);
    }
}

} // namespace crt::io