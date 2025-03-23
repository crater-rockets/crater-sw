#include "crater/main/io/ChannelReader.hpp"

namespace crt::io {

void ChannelReader::process(Reader& reader) {
    ::mavlink::mavlink_message_t raw_msg {};
    ::mavlink::mavlink_status_t status {};

    while (true) {
        nonstd::span bytes = reader.read(buf_);
        if (bytes.size() == 0) {
            break;
        }

        for (uint8_t c : bytes) {
            uint8_t res = ::mavlink::mavlink_parse_char(
                static_cast<uint8_t>(mavlink_chan_),
                c,
                &raw_msg,
                &status
            );
            if (res == 1) {
                ChannelReaderKey key {
                    .msg_id = raw_msg.msgid,
                    .sys_id = raw_msg.sysid,
                    .comp_id = raw_msg.compid
                };

                if (channels_.count(key) > 0) {
                    // TODO: Error reporting
                    channels_[key].dispatch_fn(raw_msg);
                }
            }
        }
    }
}

} // namespace crt::io