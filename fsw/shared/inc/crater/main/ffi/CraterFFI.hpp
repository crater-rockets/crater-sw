#pragma once

namespace crt::ffi {

struct Buffer {
    unsigned char* data;
    unsigned long long length;
};

/// <div rustbindgen opaque></div>
class CraterCpp {
public:
    CraterCpp();
    ~CraterCpp();

    void step(Buffer input, Buffer* output);

private:
    // Use PIMPL pattern to avoid having complex headers included in this file, that Rust wouldn't be able to understand
    /// <div rustbindgen hide></div>
    struct Data;
    Data* data_;
};

} // namespace crt::ffi