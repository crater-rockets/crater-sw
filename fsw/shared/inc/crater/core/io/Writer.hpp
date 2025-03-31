#pragma once

#include <nonstd/span.hpp>

namespace crt::io {
class Writer {
  public:
    virtual ~Writer() = default;

    virtual void write(nonstd::span<const uint8_t> data) = 0;
};

} // namespace crt::hal