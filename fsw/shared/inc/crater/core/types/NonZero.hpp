#pragma once

#include "crater/core/errors/Asserts.hpp"

namespace crt
{
template <typename T>
struct NonZero {
    static_assert(std::is_integral_v<T>, "T must be an integral type");

    constexpr NonZero(const T value)
        : v(value)
    {
        CR_ASSERT((v != 0), "Initializing NonZero instance with zero value");
    }

    operator T() const
    {
        return v;
    }

    const T v;
};

}  // namespace crt
