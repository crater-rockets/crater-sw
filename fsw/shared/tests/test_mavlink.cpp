#include <fmt/base.h>
#include <fmt/core.h>

#include <crater/telemetry/Mavlink.hpp>
#include <cstdint>
#include <lest/lest.hpp>
#include <nonstd/span.hpp>

#define CASE(name) lest_CASE(specification(), name)

extern lest::tests& specification();

CASE(
    "Basic functionality"
    "[Mavlink]"
) {
    mavlink::crater::msg::TestMessage test_msg {};
    test_msg.field1 = 123;
    test_msg.field2 = 456.789F;

    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf {};
    crt::mavlink::mavlink_encode_msg(buf, test_msg, 1, 1);
}
