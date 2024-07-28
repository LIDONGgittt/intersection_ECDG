#include "intersection_utility.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>

namespace intersection_management {

Node::Node(int id, double ett, double d, double ewd, double enwd)
    : id_(id), estimate_travel_time_(ett), depth_(d), edge_weighted_depth_(ewd), edge_node_weighted_depth_(enwd) {
    edges_.clear();

    time_window_ = std::vector<double>({-1, -1});
    estimate_arrival_time_ = -1;
    in_lane_id_ = -1;
    in_leg_id_ = -1;
    out_lane_id_ = -1;
    out_leg_id_ = -1;
    assigned_lane_id_ = -1;
    critical_resource_ = nullptr;
    route_ = nullptr;
}
Node::Node(int id, double ett, int in_leg_id, int in_lane_id, int out_leg_id, int out_lane_id)
    : Node(id, ett, -1, -1, -1) {
    in_leg_id_ = in_leg_id;
    in_lane_id_ = in_lane_id;
    out_leg_id_ = out_leg_id;
    out_lane_id_ = out_lane_id;
}
Node::Node(int id, double ett, int in_leg_id, int in_lane_id, int out_leg_id, int out_lane_id, double eat)
    : Node(id, ett, in_leg_id, in_lane_id, out_leg_id, out_lane_id) {
    estimate_arrival_time_ = eat;
}
Node::Node(const Node &node) {
    id_ = node.id_;
    estimate_travel_time_ = node.estimate_travel_time_;
    depth_ = node.depth_;
    edge_weighted_depth_ = node.edge_weighted_depth_;
    edge_node_weighted_depth_ = node.edge_node_weighted_depth_;
    // edges_ = node.edges_; // do not copy edges
    time_window_ = node.time_window_;
    estimate_arrival_time_ = node.estimate_arrival_time_;
    in_lane_id_ = node.in_lane_id_;
    in_leg_id_ = node.in_leg_id_;
    out_lane_id_ = node.out_lane_id_;
    out_leg_id_ = node.out_leg_id_;
    assigned_lane_id_ = node.assigned_lane_id_;
    critical_resource_ = node.critical_resource_;
    route_ = node.route_;
}

void Node::printWeightAndEdge() {
    std::cout << "Node " << id_ << " has weight: " << estimate_travel_time_;
    std::cout << ". Connects with: ";
    for (auto p_edge : edges_) {
        std::cout << p_edge->node1_.lock()->id_ << " --" << p_edge->edge_weight_ << "--> " << p_edge->node2_.lock()->id_ << ",  ";
    }
    std::cout << std::endl;
}

void Node::printDetail() {
    std::cout << "Node id: " << std::setw(3) << id_ << ", ett: " << estimate_travel_time_ << ", eat: " << estimate_arrival_time_;
    std::cout << ", in-bound info: " << in_leg_id_ << "_" << in_lane_id_ << ", out_bound info: " << out_leg_id_ << "_" << out_lane_id_;
    std::cout << ", time window: [" << std::setw(3) << time_window_[0] << "," << std::setw(3) << time_window_[1] << "], scheduled out_bound leg_lane: " << out_leg_id_;
    for (auto l : possible_lane_id_) std::cout << "_" << l;
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

// bidirectional, any edge connect with node with id, bidirectional has a default value of true
bool Node::isConnectedWith(int id, bool bidirectional) {
    for (auto p_edge : edges_) {
        if ((p_edge->node2_.lock()->id_ == id) || (bidirectional && p_edge->node1_.lock()->id_ == id)) {
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

// bidirectional, any edge connect with node with id
std::shared_ptr<Edge> Node::getEdgeWith(int id, bool bidirectional) {
    for (auto p_edge : edges_) {
        if ((p_edge->node2_.lock()->id_ == id) || (bidirectional && p_edge->node1_.lock()->id_ == id)) {
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

ConflictType Route::FindConflictTypeWithRoute(std::shared_ptr<Route> other_route) {
    ConflictType ct;

    // Diverging relationship
    if (getLaneIn()->getUniqueId() == other_route->getLaneIn()->getUniqueId()) {
        ct.setDiverging();
        ct.setPrecedence();
    }

    // Converging relationship
    if (getLaneOut()->getUniqueId() == other_route->getLaneOut()->getUniqueId()) {
        ct.setConverging();
    }

    // Competing relationship
    if (getLaneOut()->getLegId() == other_route->getLaneOut()->getLegId() && getLaneOut()->leg_.lock()->critical_resource_) {
        ct.setCompeting();
    }

    // Crossing relationship only when not diverging nor converging
    if (!(ct.isDiverging() || ct.isConverging())) {
        std::vector<std::pair<int, char>> lane_id_route_pair; // 's' for self, 'o' for other
        lane_id_route_pair.push_back(std::make_pair(getLaneIn()->getUniqueId(), 's'));
        lane_id_route_pair.push_back(std::make_pair(getLaneOut()->getUniqueId(), 's'));
        lane_id_route_pair.push_back(std::make_pair(other_route->getLaneIn()->getUniqueId(), 'o'));
        lane_id_route_pair.push_back(std::make_pair(other_route->getLaneOut()->getUniqueId(), 'o'));
        std::sort(lane_id_route_pair.begin(), lane_id_route_pair.end(),
                  [](std::pair<int, char> a, std::pair<int, char> b) {return a.first < b.first; });
        char route_code_minimum_id = lane_id_route_pair[0].second;
        if (route_code_minimum_id == lane_id_route_pair[2].second
            && lane_id_route_pair[2].first != lane_id_route_pair[1].first
            && lane_id_route_pair[2].first != lane_id_route_pair[3].first) {
            ct.setCrossing();
        }
    }

    return ct;
}

} // namespace intersection_management