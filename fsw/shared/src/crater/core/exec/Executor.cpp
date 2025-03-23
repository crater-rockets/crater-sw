#include "crater/core/exec/Executor.hpp"

namespace crt::exec {

void Executor::add_node(std::unique_ptr<Node>&& node) {
    nodes_.emplace_back(std::move(node));
}

void Executor::step() {
    for(std::unique_ptr<Node>& node: nodes_) {
        node->step();
    }
}

} // namespace crt::exec