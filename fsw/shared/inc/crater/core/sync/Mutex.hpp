#pragma once

#include <mutex>

namespace crt::sync
{

template <typename T>
class Mutex
{
public:
    Mutex(T&& data)
        : data_(std::move(data)),
          mutex_()
    {
    }

    template <typename Args...>
    Mutex(Args&&..args)
        : data_(std::forward<Args>(args)...)
    {
    }

    Locked<T> lock()
    {
        return Locked(&data_);
    }

private:
    std::mutex mutex_;
    T data_;
};

template <typename T>
class Locked
{
public:
    friend class Mutex<T>;

    T* operator->()
    {
        return locked_data_;
    }

    T& operator*()
    {
        return *locked_data_;
    }

private:
    Locked(T* locked_data, std::mutex& mutex)
        : lock_(mutex),
          locked_data_(locked_data)
    {
    }

    std::unique_lock lock_;
    T* locked_data_;
};

}  // namespace crt::sync
