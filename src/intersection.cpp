#include "intersection.h"

#include <iostream>
#include <random>
#include "parameters.h"

namespace intersection_management {

void Intersection::reset() {
    nodes_.clear();
    edges_.clear();
    critical_resource_map_.clear();
    leg_map_.clear();
    lane_map_.clear();
    num_nodes_ = 0;
    auto leading_node = std::make_shared<Node>(0); // virtual leading vehicle
    leading_node->time_window_ = std::vector<double>({0, 0});
    AddNode(leading_node);
}

void Intersection::InitializeFromParam() {
    num_legs_ = param.num_legs;
    num_lanes_in_vec_ = param.num_lanes_in_vec;
    num_lanes_out_vec_ = param.num_lanes_out_vec;
    num_lanes_ = 0;
    for (auto num_in : num_lanes_in_vec_) num_lanes_ += num_in;
    for (auto num_out : num_lanes_in_vec_) num_lanes_ += num_out;
    arrival_interval_avg_ = param.arrival_interval_avg;
    travel_time_range_ = param.travel_time_range;


    // initialize random seed
    if (param.random_seed < 0) {
        std::random_device rd;
        mt_.seed(rd());
    }
    else {
        mt_.seed(param.random_seed);
    }
}

void Intersection::InitializeFromLocalParam(Parameters local_param) {
    num_legs_ = local_param.num_legs;
    num_lanes_in_vec_ = local_param.num_lanes_in_vec;
    num_lanes_out_vec_ = local_param.num_lanes_out_vec;
    num_lanes_ = 0;
    for (auto num_in : num_lanes_in_vec_) num_lanes_ += num_in;
    for (auto num_out : num_lanes_in_vec_) num_lanes_ += num_out;
    arrival_interval_avg_ = local_param.arrival_interval_avg;
    travel_time_range_ = local_param.travel_time_range;


    // initialize random seed
    if (local_param.random_seed < 0) {
        std::random_device rd;
        mt_.seed(rd());
    }
    else {
        mt_.seed(local_param.random_seed);
    }
}

void Intersection::AddIntersectionUtilitiesFromGeometry() {
    AddLegsAndLanesFromGeometry();
    AddCriticalResourcesFromGeometry();
    UpdateReferencesOfCriticalResoucesAndLegs();
}

void Intersection::AddCriticalResourcesFromGeometry() {
    critical_resource_map_.clear();
    for (int leg_id = 0; leg_id < num_legs_; leg_id++) {
        if (num_lanes_out_vec_[leg_id] > 1) {
            auto cr = std::make_shared<CriticalResource>(leg_id, num_lanes_out_vec_[leg_id]);
            critical_resource_map_[leg_id] = cr;
        }
    }
}

void Intersection::AddLegsAndLanesFromGeometry() {
    leg_map_.clear();
    lane_map_.clear();
    int lane_unique_id = 0;
    for (int leg_id = 0; leg_id < num_legs_; leg_id++) {
        leg_map_[leg_id] = std::make_shared<Leg>(leg_id);
        for (int lane_in_id = 0; lane_in_id < num_lanes_in_vec_[leg_id]; lane_in_id++) {
            auto lane_in = std::make_shared<Lane>(lane_in_id, 'i', lane_unique_id, leg_map_[leg_id]);
            lane_map_[lane_unique_id++] = lane_in;
            leg_map_[leg_id]->lanes_in_map_[lane_in_id] = lane_in;
        }
        for (int lane_out_id = 0; lane_out_id < num_lanes_out_vec_[leg_id]; lane_out_id++) {
            auto lane_out = std::make_shared<Lane>(lane_out_id, 'o', lane_unique_id, leg_map_[leg_id]);
            lane_map_[lane_unique_id++] = lane_out;
            leg_map_[leg_id]->lanes_out_map_[lane_out_id] = lane_out;
        }
    }
}

void Intersection::UpdateReferencesOfCriticalResoucesAndLegs() {
    for (auto cr_pair : critical_resource_map_) {
        cr_pair.second->leg_ = leg_map_[cr_pair.second->leg_id_];
    }
    for (auto leg_pair : leg_map_) {
        if (critical_resource_map_.find(leg_pair.first) != critical_resource_map_.end()) {
            leg_pair.second->critical_resource_ = critical_resource_map_[leg_pair.first];
        }
    }
}

void Intersection::AddNode(std::shared_ptr<Node> node) {
    nodes_.push_back(node);
    num_nodes_++;
}

void Intersection::AddEdge(std::shared_ptr<Edge> edge) {
    edges_.push_back(edge);
}

void Intersection::AddRandomVehicleNodes(int count, bool verbose) {
    std::uniform_int_distribution<int> travel_time_dist(travel_time_range_[0], travel_time_range_[1]);
    std::poisson_distribution<int> arrival_interval_dist(arrival_interval_avg_);
    int num_total_lanes = lane_map_.size();
    // for (auto num_in : num_lanes_in_vec_) num_total_lanes += num_in;
    // for (auto num_out : num_lanes_out_vec_) num_total_lanes += num_out;

    int last_arrival_time = 0;
    for (auto n : nodes_) {
        if (n->estimate_arrival_time_ > last_arrival_time)
            last_arrival_time = n->estimate_arrival_time_;
    }

    for (int id = 1; id <= count; id++) { // id 0 is automatically created for the virtual leading vehicle on initialization or reset
        auto in_lane = lane_map_[mt_() % num_total_lanes];
        while (in_lane->isOutBound()) in_lane = lane_map_[mt_() % num_total_lanes];
        auto out_lane = lane_map_[mt_() % num_total_lanes];
        while (out_lane->isInBound() || out_lane->getLegId() == in_lane->getLegId()) { // in_leg and out_leg should be different
            out_lane = lane_map_[mt_() % num_total_lanes];
        }
        if (getNumNodes() > 1) {
            last_arrival_time += arrival_interval_dist(mt_);
        }
        auto node = std::make_shared<Node>(id, travel_time_dist(mt_), in_lane->getLegId(), in_lane->getId(),
                                           out_lane->getLegId(), out_lane->getId(), last_arrival_time);
        AddNode(node);
        if (verbose)
            node->printDetail();
    }
}

// add random vehicle according to travel_time_choice
// travel_time_choice must have 3 items, representing estimated travel time for [right-turn, straight, left-turn]
void Intersection::AddRandomVehicleNodesWithTravelTime(int count, std::vector<double> travel_time_choice, bool verbose) {
    std::poisson_distribution<int> arrival_interval_dist(arrival_interval_avg_);
    int num_total_lanes = lane_map_.size();
    // for (auto num_in : num_lanes_in_vec_) num_total_lanes += num_in;
    // for (auto num_out : num_lanes_out_vec_) num_total_lanes += num_out;

    int last_arrival_time = 0;
    for (auto n : nodes_) {
        if (n->estimate_arrival_time_ > last_arrival_time)
            last_arrival_time = n->estimate_arrival_time_;
    }

    for (int id = 1; id <= count; id++) { // id 0 is automatically created for the virtual leading vehicle on initialization or reset
        auto in_lane = lane_map_[mt_() % num_total_lanes];
        while (in_lane->isOutBound()) in_lane = lane_map_[mt_() % num_total_lanes];
        auto out_lane = lane_map_[mt_() % num_total_lanes];
        while (out_lane->isInBound() || out_lane->getLegId() == in_lane->getLegId()) { // in_leg and out_leg should be different
            out_lane = lane_map_[mt_() % num_total_lanes];
        }
        if (getNumNodes() > 1) {
            last_arrival_time += arrival_interval_dist(mt_);
        }
        double estimate_travel_time;
        if ((in_lane->getLegId() - 1 + 4) % 4 == out_lane->getLegId()) { // right-turn
            estimate_travel_time = travel_time_choice[0];
        }
        else if ((in_lane->getLegId() + 1) % 4 == out_lane->getLegId()) { // left-turn
            estimate_travel_time = travel_time_choice[2];
        }
        else {
            estimate_travel_time = travel_time_choice[1]; // straight
        }
        auto node = std::make_shared<Node>(id, estimate_travel_time, in_lane->getLegId(), in_lane->getId(),
                                           out_lane->getLegId(), out_lane->getId(), last_arrival_time);
        AddNode(node);
        if (verbose)
            node->printDetail();
    }
}

void Intersection::AssignRoutesToNodes() {
    for (int id = 1; id < nodes_.size(); id++) {
        auto &node = nodes_[id];
        std::shared_ptr<Lane> &lane_in = leg_map_[node->in_leg_id_]->lanes_in_map_[node->in_lane_id_];
        std::shared_ptr<Lane> &lane_out = leg_map_[node->out_leg_id_]->lanes_out_map_[node->out_lane_id_];
        node->route_ = std::make_shared<Route>(lane_in, lane_out);
    }
}

void Intersection::AssignCriticalResourcesToNodes() {
    for (auto node : nodes_) {
        auto iter_cr = critical_resource_map_.find(node->out_leg_id_);
        if (iter_cr != critical_resource_map_.end()) {
            node->critical_resource_ = iter_cr->second;
            iter_cr->second->nodes_.push_back(node);
        }
        else {
            node->critical_resource_ = nullptr;
        }
    }
}

void Intersection::AssignEdgesWithSafetyOffsetToNodes() {
    ConflictType ct_precedence;
    ct_precedence.setPrecedence();
    for (int i = 1; i < nodes_.size(); i++) {
        auto edge_to_virtual_leading = std::make_shared<Edge>(nodes_[0], nodes_[i], 0, ct_precedence, 0);
        nodes_[0]->edges_.push_back(edge_to_virtual_leading);
        nodes_[i]->edges_.push_back(edge_to_virtual_leading);
        AddEdge(edge_to_virtual_leading);
    }

    for (int i = 1; i < nodes_.size(); i++) {
        for (int j = i + 1; j < nodes_.size(); j++) {
            ConflictType ct = nodes_[i]->route_->FindConflictTypeWithRoute(nodes_[j]->route_);
            int predecessor_id = -1;
            double offset = 0;
            if (ct.isDiverging()) {
                predecessor_id = i;
                if (param.activate_precedent_offset) {
                    offset = -1;
                } else {
                    offset = 0;
                }
                
            }
            else if (!ct.isNotConflicting()) {
                offset = 0;
            }
            else {
                continue; // non-conflict relation don't need edges
            }
            auto edge = std::make_shared<Edge>(nodes_[i], nodes_[j], offset, ct, predecessor_id);
            nodes_[i]->edges_.push_back(edge);
            nodes_[j]->edges_.push_back(edge);
            AddEdge(edge);
        }
    }
}

bool Intersection::isRightmostTurningRoute(std::shared_ptr<Route> route) {
    auto lane_in = route->getLaneIn();
    auto lane_out = route->getLaneOut();

    if (lane_in->getUniqueId() == lane_out->getUniqueId() + 1 || lane_in->getUniqueId() == 0 && lane_out->getUniqueId() == num_lanes_ - 1)
        // rightmost turning
        return true;
    return false;
}
bool Intersection::isLeftmostTurningRoute(std::shared_ptr<Route> route) {
    auto lane_in = route->getLaneIn();
    auto lane_out = route->getLaneOut();

    if (lane_in->getLegId() == lane_out->getLegId() - 1 || lane_in->getLegId() == lane_out->getLegId() - 1 + num_legs_) {
        // left hand turning
        if (lane_in->getId() == num_lanes_in_vec_[lane_in->getLegId()] - 1 && lane_out->getId() == 0) {
            // leftmost turning
            return true;
        }
    }
    return false;
}
bool Intersection::isOutermostTurningRoute(std::shared_ptr<Route> route) {
    return isRightmostTurningRoute(route) || isLeftmostTurningRoute(route);
}

} // namespace intersection_management