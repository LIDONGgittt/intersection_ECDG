#include "intersection.h"

#include <iostream>
#include "parameters.h"

namespace intersection_management {

extern Parameters param;

void Intersection::InitializeFromParam() {
    num_directions_ = param.num_directions;
    num_lanes_in_ = param.num_lanes_in;
    num_lanes_out_ = param.num_lanes_out;
}

void Intersection::reset() {
    nodes_.clear();
    edges_.clear();
    critical_resources_.clear();
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

void Intersection::AddCriticalResourceFromGeometric() {
    for (int i = 0; i < num_directions_; i++) {
        if (num_lanes_out_[i] > 1) {
            auto cr = std::make_shared<CriticalResource>(i, num_lanes_out_[i]);
            critical_resources_.push_back(cr);
        }
    }
}

int Intersection::getNumCriticalResource() {
    return critical_resources_.size();
}

} // namespace intersection_management