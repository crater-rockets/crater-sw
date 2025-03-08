#pragma once

#include "Diagnostics.hpp"

namespace crt
{
enum class ErrorCode { Error1 = 1, Error2 = 2 };

inline const char* error_code_string(ErrorCode err)
{
    CRT_BEGIN_NO_DEFAULT_CASE();
    switch (err) {
        case ErrorCode::Error1:
            return "Error1";
        case crt::ErrorCode::Error2:
            return "Error2";
    }
    CRT_END_NO_DEFAULT_CASE();
    return "";
}

}  // namespace crt
