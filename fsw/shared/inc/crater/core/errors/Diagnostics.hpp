/*
 * Copyright (c) 2022, Linus Groh <linusg@serenityos.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

// Needed to turn the 'name' token and the preceding 'GCC diagnostic ignored'
// into a single string literal, it won't accept "foo"#bar concatenation.
#define _CRT_PRAGMA(x) _Pragma(#x)
#define CRT_PRAGMA(x) _CRT_PRAGMA(x)

// Helper macro to temporarily disable a diagnostic for the given statement.
// Using _Pragma() makes it possible to use this in other macros as well (and
// allows us to define it as a macro in the first place).
// NOTE: 'GCC' is also recognized by clang.
#define CRT_IGNORE_DIAGNOSTIC(name, statement) \
    CRT_PRAGMA(GCC diagnostic push);           \
    CRT_PRAGMA(GCC diagnostic ignored name);   \
    statement;                                 \
    CRT_PRAGMA(GCC diagnostic pop);

#define CRT_BEGIN_NO_DEFAULT_CASE()  \
    CRT_PRAGMA(GCC diagnostic push); \
    CRT_PRAGMA(GCC diagnostic ignored "-Wswitch-default");

#define CRT_END_NO_DEFAULT_CASE() CRT_PRAGMA(GCC diagnostic pop);
