#include "crater/core/errors/Error.hpp"

namespace crt
{

const char* ErrorBase::code_str() const
{
    return error_code_string(code());
}

}  // namespace crt
