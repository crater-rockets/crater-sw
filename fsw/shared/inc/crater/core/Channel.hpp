#pragma once

#include <crater/core/collections/CircularBuffer.hpp>
#include <crater/core/sync/Mutex.hpp>
#include <memory>

#include "crater/core/types/NonZero.hpp"

namespace crt {

template<typename T>
class Sender;

template<typename T>
class Channel;

template<typename T>
class Receiver {
    friend class Channel<T>;

  public:
    std::optional<T> try_receive() {
        auto buf = inner_->buf_.lock();
        return buf->pop();
    }

    size_t count() {
        auto buf = inner_->buf_.lock();
        return buf->count();
    }

    size_t size() {
        return size;
    }

  private:
    struct ReceiverInner {
        crt::sync::Mutex<crt::collections::CircularBuffer<T>> buf_;

        ReceiverInner(crt::collections::CircularBuffer<T> buf) : buf_(crt::sync::Mutex(buf)) {}
    };

    Receiver(std::shared_ptr<ReceiverInner> inner, size_t size) : inner_(inner), size_(size) {}

    std::shared_ptr<ReceiverInner> inner_;
    size_t size_;
};

template<typename T>
class Channel {
    friend class Receiver<T>;
    friend class Sender<T>;

    struct ChannelInner {
        using RxInner = typename Receiver<T>::ReceiverInner;
        std::vector<std::shared_ptr<RxInner>> receivers;

        void write(const T& v) {
            for (std::shared_ptr<RxInner>& rx : receivers) {
                sync::Locked buf = rx->buf_.lock();
                buf->push(v);
            }
        }
    };

  public:
    Channel() : inner_(std::make_shared<ChannelInner>()) {};

    Sender<T> sender() {
        return Sender<T>(inner_);
    }

    Receiver<T> receiver(NonZero<size_t> buf_size) {
        using RxInner = typename Receiver<T>::ReceiverInner;
        std::shared_ptr<RxInner> rx_inner =
            std::make_shared<RxInner>(crt::collections::CircularBuffer<T>(buf_size));

        inner_->receivers.push_back(rx_inner);

        return Receiver<T>(rx_inner, buf_size);
    }

    ~Channel() = default;

    Channel(const Channel& s) = delete;
    Channel(Channel&& s) = default;

    Channel& operator=(const Channel& other) = delete;
    Channel& operator=(Channel&& other) = default;

  private:
    std::shared_ptr<ChannelInner> inner_;
};

template<typename T>
class Sender {
    friend class Channel<T>;

  public:
    void send(const T& v) {
        channel_->write(v);
    }

    ~Sender() = default;

    Sender(const Sender& s) = delete;
    Sender(Sender&& s) = default;

    Sender& operator=(const Sender& other) = delete;
    Sender& operator=(Sender&& other) = default;

  private:
    Sender(std::shared_ptr<typename Channel<T>::ChannelInner> channel) : channel_(channel) {}

    std::shared_ptr<typename Channel<T>::ChannelInner> channel_;
};

} // namespace crt
