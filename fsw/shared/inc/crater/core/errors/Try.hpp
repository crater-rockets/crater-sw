/*
 * Copyright (c) 2021, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <crater/core/errors/Asserts.hpp>
#include <crater/core/errors/Diagnostics.hpp>
#include <nonstd/expected.hpp>
#include <optional>

namespace crt::error_detail {
template<typename T>
static inline __attribute__((always_inline)) std::optional<T> extract_error(std::optional<T>&&) {
    return std::nullopt;
}

template<typename T, typename E>
static inline __attribute__((always_inline)) nonstd::expected<T, E>
extract_error(nonstd::expected<T, E>&& err) {
    return err.get_unexpected();
}

} // namespace crt::error_detail

// NOTE: This macro works with any result type that has the expected APIs.
//
//       It depends on a non-standard C++ extension, specifically
//       on statement expressions [1]. This is known to be implemented
//       by at least clang and gcc.
//       [1] https://gcc.gnu.org/onlinedocs/gcc/Statement-Exprs.html
//
//       If the static_assert below is triggered, it means you tried to return a
//       reference from a fallible expression. This will not do what you want;
//       the statement expression will create a copy regardless, so it is
//       explicitly disallowed.
#define TRY(expression) \
    ({ \
        /* Ignore -Wshadow to allow nesting the macro. */ \
        CRT_IGNORE_DIAGNOSTIC("-Wshadow", auto&& _temporary_result = (expression)); \
        if (!_temporary_result.has_value()) [[unlikely]] \
            return crt::error_detail::extract_error(std::move(_temporary_result)); \
        std::move(_temporary_result.value()); \
    })

#define MUST(expression) \
    ({ \
        /* Ignore -Wshadow to allow nesting the macro. */ \
        CRT_IGNORE_DIAGNOSTIC("-Wshadow", auto&& _temporary_result = (expression)); \
        CRT_ASSERT(_temporary_result.has_value(), "Expected contains an error"); \
        _temporary_result.value(); \
    })
