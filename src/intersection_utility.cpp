#include "intersection_utility.h"

#include <iostream>

namespace intersection_management {

void Node::printWeightAndEdge() {
    std::cout << "Node " << id_ << " has weight: " << estimate_travel_time_;
    std::cout << ". Connects to: ";
    for (auto p_edge : edges_) {
        std::cout << "--" << p_edge->edge_weight_ << "--> " << p_edge->node2_.lock()->id_ << ",  ";
    }
    std::cout << std::endl;
}

void Node::printDetail() {
    std::cout << "Node id: " << id_ << ", ett: " << estimate_travel_time_ << ", eat: " << estimate_arrival_time_;
    std::cout << ", in-bound info: " << in_leg_id_ << "_" << in_lane_id_ << ", out_bound info: " << out_leg_id_ << "_" << out_lane_id_;
    std::cout << ", time window: [" << time_window_[0] << "," << time_window_[1] << "].\n";
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

//directional, node with id should be as the node2 in the edge
bool Node::isConnectedTo(int id) {
    return isConnectedWith(id, false);
}
bool Node::isConnectedTo(Node &n) {
    return isConnectedTo(n.id_);
}
bool Node::isConnectedTo(std::shared_ptr<Node> &p_n) {
    return isConnectedTo(p_n->id_);
}
// directional, node with id should be as the node2 in the edge
std::shared_ptr<Edge> Node::getEdgeTo(int id) {
    return getEdgeWith(id, false);
}
std::shared_ptr<Edge> Node::getEdgeTo(Node &n) {
    return getEdgeTo(n.id_);
}
std::shared_ptr<Edge> Node::getEdgeTo(std::shared_ptr<Node> &p_n) {
    return getEdgeTo(p_n->id_);
}

// undirectional, any edge connect with node with id
bool Node::isConnectedWith(int id, bool undirectional) {
    for (auto p_edge : edges_) {
        if ((p_edge->node2_.lock()->id_ == id) || (undirectional && p_edge->node1_.lock()->id_ == id)) {
            return true;
        }
    }
    return false;
}
bool Node::isConnectedWith(Node &n) {
    return isConnectedWith(n.id_);
}
bool Node::isConnectedWith(std::shared_ptr<Node> &p_n) {
    return isConnectedWith(p_n->id_);
}

// undirectional, any edge connect with node with id
std::shared_ptr<Edge> Node::getEdgeWith(int id, bool undirectional) {
    for (auto p_edge : edges_) {
        if ((p_edge->node2_.lock()->id_ == id) || (undirectional && p_edge->node1_.lock()->id_ == id)) {
            return p_edge;
        }
    }
    return nullptr;
}
std::shared_ptr<Edge> Node::getEdgeWith(Node &n) {
    return getEdgeWith(n.id_);
}
std::shared_ptr<Edge> Node::getEdgeWith(std::shared_ptr<Node> &p_n) {
    return getEdgeWith(p_n->id_);
}

int CriticalResource::getLegId() {
    return leg_id_;
}
int CriticalResource::getNumResources() {
    return num_resources_;
}


} // namespace intersection_management