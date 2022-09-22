#include "graph_utility.h"
#include <iostream>

namespace intersection_management {
    void Node::printDetail() {
        std::cout << "Node " << id_ << " has weight: " << node_weight_;
        std::cout << ". Connects to: ";
        for (auto p_edge : edges_) {
            std::cout << "--" << p_edge->edge_weight_ << "--> " << p_edge->to_.lock()->id_ << ",  ";
        }
        std::cout << std::endl;
    }

    bool Node::isSameAs(int id) {
        return id_ == id;
    }
    bool Node::isSameAs(Node n) {
        return id_ == n.id_;
    }
    bool Node::isSameAs(std::shared_ptr<Node> p_n) {
        return id_ == p_n->id_;
    }
    bool Node::isConnectedTo(int id) {
        for (auto p_edge : edges_) {
            if (p_edge->to_.lock()->id_ == id) {
                return true;
            }
        }
        return false;
    }
    bool Node::isConnectedTo(Node n) {
        return isConnectedTo(n.id_);
    }
    bool Node::isConnectedTo(std::shared_ptr<Node> p_n) {
        return isConnectedTo(p_n->id_);
    }
}