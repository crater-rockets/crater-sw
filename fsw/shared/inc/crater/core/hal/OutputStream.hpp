#pragma once

#include <nonstd/span.hpp>

namespace crt::hal {
class OutputStream {
  public:
    virtual ~OutputStream() = default;

    virtual void write(nonstd::span<uint8_t> data);
};

} // namespace crt::hal