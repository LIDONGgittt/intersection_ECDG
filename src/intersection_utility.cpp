#include "intersection_utility.h"

#include <iostream>
#include "parameters.h"

namespace intersection_management {

extern Parameters param;

void Intersection::initializeFromParam() {
    num_directions_ = param.num_directions;
    num_lanes_in_ = param.num_lanes_in;
    num_lanes_out_ = param.num_lanes_out;
}

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
bool Node::isSameAs(Node &n) {
    return id_ == n.id_;
}
bool Node::isSameAs(std::shared_ptr<Node> &p_n) {
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
bool Node::isConnectedTo(Node &n) {
    return isConnectedTo(n.id_);
}
bool Node::isConnectedTo(std::shared_ptr<Node> &p_n) {
    return isConnectedTo(p_n->id_);
}

std::shared_ptr<Edge> Node::getEdgeTo(int id) {
    for (auto p_edge : edges_) {
        if (p_edge->to_.lock()->id_ == id) {
            return p_edge;
        }
    }
    return nullptr;
}
std::shared_ptr<Edge> Node::getEdgeTo(Node &n) {
    return getEdgeTo(n.id_);
}
std::shared_ptr<Edge> Node::getEdgeTo(std::shared_ptr<Node> &p_n) {
    return getEdgeTo(p_n->id_);
}
} // namespace intersection_management