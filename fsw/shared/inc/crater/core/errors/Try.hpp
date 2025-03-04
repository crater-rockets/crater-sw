/*
 * Copyright (c) 2021, Andreas Kling <kling@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <crater/core/errors/Asserts.hpp>
#include <crater/core/errors/Diagnostics.hpp>

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

#define TRY(expression)                                                 \
    ({                                                                  \
        /* Ignore -Wshadow to allow nesting the macro. */               \
        CRT_IGNORE_DIAGNOSTIC("-Wshadow",                               \
                              auto&& _temporary_result = (expression)); \
        if (!_temporary_result.has_value()) [[unlikely]]                \
            return std::move(_temporary_result.get_unexpected());       \
        std::move(_temporary_result.value());                           \
    })

#define MUST(expression)                                                       \
    ({                                                                         \
        /* Ignore -Wshadow to allow nesting the macro. */                      \
        CRT_IGNORE_DIAGNOSTIC("-Wshadow",                                      \
                             auto&& _temporary_result = (expression));         \
        VERIFY(_temporary_result.has_value());                                 \
        _temporary_result.value();                                             \
    })
