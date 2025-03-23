#pragma once

#include "Reader.hpp"

namespace crt::io {
class SpanReader: public Reader {
public:
    SpanReader(nonstd::span<const uint8_t> data) : data_(data) {}

    nonstd::span<uint8_t> read(nonstd::span<uint8_t> buf) override {
        size_t n_read = std::min(buf.size(), data_.size());

        std::copy_n(data_.begin(), n_read, buf.begin());

        data_ = data_.subspan(n_read);

        return buf.first(n_read);
    }

private:
    nonstd::span<const uint8_t> data_;
};
} // namespace crt::io