#pragma once

#include <nonstd/span.hpp>

namespace crt::io {
class Reader {
public:
    Reader() = default;

    virtual ~Reader() = default;

    virtual nonstd::span<uint8_t> read(nonstd::span<uint8_t> buf) = 0;
};

} // namespace crt::io