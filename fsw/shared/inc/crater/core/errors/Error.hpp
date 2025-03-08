#pragma once

#include <fmt/format.h>

#include <functional>
#include <memory>
#include <nonstd/expected.hpp>
#include <optional>
#include <string>
#include <utility>

#include "ErrorCodes.hpp"
#include "crater/core/types/DynamicCast.hpp"

namespace crt
{

template <typename DataT>
struct ErrorDataToString;

class ErrorBase
{
public:
    virtual ErrorCode code() const      = 0;
    virtual std::string message() const = 0;

    const char* code_str() const;
};

template <typename DataT>
class Error : public ErrorBase
{
public:
    explicit Error(ErrorCode err_code, DataT data)
        : err_code_(err_code),
          data_(data)
    {
    }

    ErrorCode code() const override
    {
        return err_code_;
    }

    std::string message() const override
    {
        return fmt::format("Error {}:{}. {}", code_str(), static_cast<int>(err_code_), data_string());
    }

    const DataT& data() const
    {
        return data_;
    }

    std::string data_string() const
    {
        return ErrorDataToString<DataT>::data_string(data_);
    }

private:
    ErrorCode err_code_;
    DataT data_;
};

template <>
class Error<void> : public ErrorBase
{
public:
    explicit Error(ErrorCode err_code)
        : err_code_(err_code)
    {
    }

    ErrorCode code() const override
    {
        return err_code_;
    }

    std::string message() const override
    {
        return fmt::format("Error {}:{}", code_str(), static_cast<int>(err_code_));
    }

private:
    ErrorCode err_code_;
};

template <>
class Error<std::string> : public ErrorBase
{
public:
    explicit Error(ErrorCode err_code, std::string msg)
        : err_code_(err_code),
          msg_(msg)
    {
    }

    ErrorCode code() const override
    {
        return err_code_;
    }

    std::string message() const override
    {
        return fmt::format("Error {}:{}. {}", code_str(), static_cast<int>(err_code_), msg_);
    }

private:
    ErrorCode err_code_;
    std::string msg_;
};

class AnyError : public ErrorBase
{
public:
    template <typename DataT>
    AnyError(Error<DataT> error)
        : error_(std::make_unique<Error<DataT>>(error))
    {
    }

    ErrorCode code() const override
    {
        return error_->code();
    }

    std::string message() const override
    {
        return error_->message();
    }

    template <typename ErrorT>
    std::optional<std::reference_wrapper<ErrorT>> downcast()
    {
        return crt::dyn_cast<ErrorT>(*error_.get());
    }

private:
    std::unique_ptr<ErrorBase> error_;
};

template <typename T, typename ET = void>
using Expected = nonstd::expected<T, Error<ET>>;

template <typename T>
using AnyExpected = nonstd::expected<T, AnyError>;

template <typename ET>
static inline nonstd::unexpected<Error<ET>> make_error(ErrorCode ecode, ET&& data)
{
    return nonstd::make_unexpected(Error<ET>(ecode, std::forward<ET>(data)));
}

static inline nonstd::unexpected<Error<std::string>> make_error(ErrorCode ecode, const char* data)
{
    return nonstd::make_unexpected(Error<std::string>(ecode, data));
}

static inline nonstd::unexpected<Error<void>> make_error(ErrorCode ecode)
{
    return nonstd::make_unexpected(Error<void>(ecode));
}

// Deduction guides
Error(ErrorCode) -> Error<void>;
Error(ErrorCode, const char*) -> Error<std::string>;

}  // namespace crt
