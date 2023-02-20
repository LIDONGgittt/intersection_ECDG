#include "intersection.h"

#include <iostream>
#include "parameters.h"

namespace intersection_management {

extern Parameters param;

void Intersection::initializeFromParam() {
    num_directions_ = param.num_directions;
    num_lanes_in_ = param.num_lanes_in;
    num_lanes_out_ = param.num_lanes_out;
}

void Intersection::reset() {
    nodes_.clear();
    edges_.clear();
    critical_resources_.clear();
}
} // namespace intersection_management