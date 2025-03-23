#pragma once

#include <vector>

#include "Writer.hpp"

namespace crt::io {

class VecWriter: public Writer {
public:
    VecWriter(std::vector<uint8_t>& data) : data_(&data) {}

    void write(nonstd::span<const uint8_t> buf) override {
        data_->insert(data_->end(), buf.begin(), buf.end());
    }

private:
    std::vector<uint8_t>* data_;
};

} // namespace crt::io