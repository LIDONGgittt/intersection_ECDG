#include "graph_utility.h"

namespace intersection_management {
    void Node::print() {
        std::cout << "Node " << id_ << " connects to: ";
        for (int i = 0; i < edges_.size(); i++) {
            std::cout << edges_[i]->to_.lock()->id_ << ", ";
        }
        std::cout << std::endl;
    }
}