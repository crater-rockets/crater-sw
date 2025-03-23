#pragma once

#include <memory>
#include <vector>

#include "crater/core/exec/Node.hpp"

namespace crt::exec {

class Executor {
public:
    void add_node(std::unique_ptr<Node>&& node);

    void step();

private:
    std::vector<std::unique_ptr<Node>> nodes_;
};

} // namespace crt::exec