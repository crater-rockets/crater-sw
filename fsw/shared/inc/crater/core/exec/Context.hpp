#pragma once

#include <memory>

#include "crater/core/Clock.hpp"

namespace crt::exec {

class Context {
public:
    Context(std::shared_ptr<Clock> clock) :
        inner_(std::make_shared<ContextSharedData>(ContextSharedData {.clock = clock})) {}

    std::shared_ptr<Clock> clock() {
        return inner_->clock;
    }

private:
    struct ContextSharedData {
        std::shared_ptr<Clock> clock;
    };

    std::shared_ptr<ContextSharedData> inner_;
};

} // namespace crt::exec