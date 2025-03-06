#pragma once

#include <fmt/core.h>

#include <exception>
#include <string>

template <typename... Args>
static void crater_assert(const char* file, int line, const char* assertion,
                          fmt::format_string<Args...> fmt, Args... args)
{
    std::string msg = fmt::format(fmt, std::forward<Args>(args)...);

    fmt::println("{}:{} - Assertion '{}' failed: {}", file, line, assertion,
                 msg);
    std::terminate();
}

#define CR_ASSERT(expression, text, args...)                                  \
    if (!expression)                                                          \
    {                                                                         \
        crater_assert(__FILE__, __LINE__, #expression, FMT_STRING(text), \
                      ##args);                                                \
    }

