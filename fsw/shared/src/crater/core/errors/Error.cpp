#include "crater/core/errors/Error.hpp"
#include <crater_ext/ErrorCode.hpp>

namespace crt
{

const char* ErrorBase::code_str() const
{
    return crt_ext::error_code_str(code());
}

}  // namespace crt
