#include "intersection.h"

#include <iostream>
#include <random>
#include "parameters.h"

namespace intersection_management {

extern Parameters param;

void Intersection::InitializeFromParam() {
    num_legs_ = param.num_legs;
    num_lanes_in_vec_ = param.num_lanes_in_vec;
    num_lanes_out_vec_ = param.num_lanes_out_vec;

    // initialize random seed
    if (param.random_seed < 0) {
        std::random_device rd;
        mt_.seed(rd());
    }
    else {
        mt_.seed(param.random_seed);
    }
}

void Intersection::reset() {
    nodes_.clear();
    edges_.clear();
    critical_resources_map_.clear();
    auto leading_node = std::make_shared<Node>(0); // virtual leading vehicle
    leading_node->time_window_ = std::vector<double>({0, 0});
    AddNode(leading_node);
}

void Intersection::AddNode(std::shared_ptr<Node> node) {
    nodes_.push_back(node);
}

int Intersection::getNumNodes() {
    return nodes_.size();
}

void Intersection::AddEdge(std::shared_ptr<Edge> edge) {
    edges_.push_back(edge);
}

int Intersection::getNumEdges() {
    return edges_.size();
}

int Intersection::getNumCriticalResources() {
    return critical_resources_map_.size();
}

void Intersection::AddRandomVehicleNodes(int count) {
    std::uniform_int_distribution<int> travel_time_dist(param.travel_time_range[0], param.travel_time_range[1]);
    std::poisson_distribution<int> arrival_interval_dist(param.arrival_interval_avg);

    int last_arrival_time = 0;
    for (auto n : nodes_) {
        if (n->estimate_arrival_time_ > last_arrival_time)
            last_arrival_time = n->estimate_arrival_time_;
    }

    for (int id = 1; id <= count; id++) { // id 0 reserved for virtual leading vehicle
        int in_leg = mt_() % num_legs_;
        int out_leg = mt_() % num_legs_;
        while (out_leg == in_leg) { // in_leg and out_leg should be different
            out_leg = mt_() % num_legs_;
        }
        if (getNumNodes() > 1) {
            last_arrival_time += arrival_interval_dist(mt_);
        }
        auto node = std::make_shared<Node>(id, travel_time_dist(mt_), in_leg, mt_() % num_lanes_in_vec_[in_leg],
                                           out_leg, mt_() % num_lanes_out_vec_[out_leg], last_arrival_time);
        AddNode(node);
        // node->printDetail();
    }
}

void Intersection::AddCriticalResourcesFromGeometric() {
    critical_resources_map_.clear();
    for (int i = 0; i < num_legs_; i++) {
        if (num_lanes_out_vec_[i] > 1) {
            auto cr = std::make_shared<CriticalResource>(i, num_lanes_out_vec_[i]);
            critical_resources_map_[i] = cr;
        }
    }
}

void Intersection::ConnectCriticalResourcesToNodes() {
    for (auto node : nodes_) {
        auto iter_cr = critical_resources_map_.find(node->out_leg_id_);
        if (iter_cr != critical_resources_map_.end()) {
            node->critical_resource_ = iter_cr->second;
            iter_cr->second->nodes_.push_back(node);
        }
        else {
            node->critical_resource_ = nullptr;
        }
    }
}
void Intersection::ConnectEdgesToNodes() {
    

}
} // namespace intersection_management