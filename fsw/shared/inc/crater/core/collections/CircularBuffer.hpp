#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "crater/core/types/NonZero.hpp"

namespace crater::collections
{

template <typename T>
class CircularBuffer
{
public:
    CircularBuffer(NonZero<size_t> size)
    {
        buffer_.resize(size);
    }

    void push(const T& value)
    {
        buffer_[head_] = value;
        post_push();
    }

    void push(T&& value)
    {
        buffer_[head_] = std::move(value);
        post_push();
    }

    std::optional<T> pop()
    {
        if (empty_) {
            return std::nullopt;
        }

        size_t old_tail = tail_;
        increment_index(tail_);

        empty_ = tail_ == head_;

        return buffer_[old_tail];
    }

    size_t size() const
    {
        return buffer_.size();
    }

    size_t count() const
    {
        if (empty_) {
            return 0;
        } else {
            return ((head_ + size() - tail_ - 1) % size()) + 1;
        }
    }

    bool empty() const
    {
        return empty_;
    }

private:
    void post_push()
    {
        increment_index(tail_, !empty_ && head_ == tail_);
        empty_ = false;
        increment_index(head_);
    }

    void increment_index(size_t& index, size_t amount = 1)
    {
        index = (index + amount) % buffer_.size();
    }

    size_t head_ = 0;
    size_t tail_ = 0;
    bool empty_  = true;

    std::vector<T> buffer_;
};

}  // namespace crater::collections
