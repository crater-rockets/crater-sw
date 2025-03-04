#pragma once

#include <cstddef>
#include <vector>

namespace crater::collections
{

template <typename T>
class CircularBuffer
{
public:
    CircularBuffer(size_t size)
    {
        buffer_.resize(size);
    }

    void push(const T& value)
    {
        buffer_[head_] = value;
        update_push_indices();
    }

    void push(T&& value)
    {
        buffer_[head_] = std::move(value);
        update_push_indices();
    }

    void pop() {
        
    }
private:
    void update_push_indices()
    {
        if (!empty_ && head_ == tail_)
        {
            increment_index(tail_);
        }

        increment_index(head_);
    }

    void increment_index(size_t& index)
    {
        index = (index + 1) % buffer_.size();
    }

    size_t head_ = 0;
    size_t tail_ = 0;
    bool empty_ = true;

    std::vector<T> buffer_;
};
}  // namespace crater::collections
