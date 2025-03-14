#pragma once

#include <mutex>
#include <type_traits>

namespace crt::sync
{

template <typename T, typename MutexT>
class Locked;

template <typename T, typename MutexT = std::mutex>
class Mutex
{
public:
    Mutex(T data) noexcept
        : data_(data)
    {
    }

    // Prevent ambiguities on which constructor to call
    template <typename A = T, typename... Args,
              std::enable_if_t<std::is_constructible_v<T, Args...> && !std::is_same_v<std::decay_t<A>, Mutex>, int> = 0>
    Mutex(Args&&... args) noexcept
        : data_(std::forward<Args>(args)...)
    {
    }

    Locked<T, MutexT> lock() noexcept
    {
        return Locked<T, MutexT>(&data_, mutex_);
    }

    MutexT& mutex()
    {
        return mutex_;
    }

    ~Mutex() = default;

    Mutex(const Mutex& s) = delete;
    Mutex(Mutex&& s)      = delete;

    Mutex& operator=(const Mutex& other) = delete;
    Mutex& operator=(Mutex&& other)      = delete;

private:
    T data_;
    mutable MutexT mutex_;
};

template <typename T, typename MutexT>
class Locked
{
public:
    friend class Mutex<T, MutexT>;

    T* operator->() noexcept
    {
        return locked_data_;
    }

    T& operator*() noexcept
    {
        return *locked_data_;
    }

    const T* operator->() const noexcept
    {
        return locked_data_;
    }

    const T& operator*() const noexcept
    {
        return *locked_data_;
    }

private:
    Locked(T* locked_data, MutexT& mutex) noexcept
        : lock_(mutex),
          locked_data_(locked_data)
    {
    }

    mutable std::unique_lock<MutexT> lock_;
    T* locked_data_;
};

}  // namespace crt::sync
