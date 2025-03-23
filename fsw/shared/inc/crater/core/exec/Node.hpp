#pragma once

#include <string>

#include "Context.hpp"

namespace crt::exec {

class Node {
public:
    Node(std::string name, uint8_t id, Context ctx) : name_(name), id_(id), ctx_(ctx) {}

    virtual ~Node() = default;

    const Context& context() const {
        return ctx_;
    }

    const std::string& name() const {
        return name_;
    }

    uint8_t id() {
        return id_;
    }

    virtual void step() = 0;

private:
    std::string name_;
    uint8_t id_;
    mutable Context ctx_;
};

} // namespace crt::exec