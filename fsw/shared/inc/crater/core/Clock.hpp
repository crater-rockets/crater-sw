#pragma once

#include <cstdint>

namespace crt {

static constexpr int64_t S_TO_NS = 1'000'000'000;
static constexpr int64_t MS_TO_NS = 1'000'000;
static constexpr int64_t US_TO_NS = 1'000;

class Timestamp {
public:
    Timestamp(int64_t timestamp_ns) : timestamp_ns_(timestamp_ns) {}

    int64_t num_ns() {
        return timestamp_ns_;
    }

    int64_t num_us() {
        return timestamp_ns_ / US_TO_NS;
    }

    int64_t num_ms() {
        return timestamp_ns_ / MS_TO_NS;
    }

    int64_t num_s() {
        return timestamp_ns_ / S_TO_NS;
    }

    double us() {
        return as_double(US_TO_NS);
    }

    double ms() {
        return as_double(US_TO_NS);
    }

    double s() {
        return as_double(S_TO_NS);
    }

private:
    double as_double(int64_t factor) {
        int64_t integral_part = timestamp_ns_ / factor;
        int64_t decimal_part = timestamp_ns_ % factor;

        return static_cast<double>(integral_part)
            + static_cast<double>(decimal_part) / static_cast<double>(factor);
    }

    int64_t timestamp_ns_;
};

class Clock {
public:
    virtual ~Clock() = default;

    Timestamp timestamp();
};

class VirtualClock: public Clock {
public:
    VirtualClock(int64_t initial_ts_ns) : timestamp_ns_(initial_ts_ns) {}

    Timestamp timestamp() {
        return Timestamp(timestamp_ns_);
    }

    void set_time(int64_t timestamp_ns) {
        timestamp_ns_ = timestamp_ns;
    }

private:
    int64_t timestamp_ns_;
};
} // namespace crt

// ns = us * US_TO_NS + sub_us
//