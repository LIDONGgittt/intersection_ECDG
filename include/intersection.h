#ifndef INTERSECTION_MANAGEMENT_INTERSECTION_H_
#define INTERSECTION_MANAGEMENT_INTERSECTION_H_

#include "intersection_utility.h"
#include <vector>
#include <memory>

namespace intersection_management {
class Node;
class Edge;
class CriticalResource;

class Intersection {
public:
    Intersection() {
        reset();
        initializeFromParam();
    }

    void initializeFromParam();
    void reset();

    int num_directions_;
    std::vector<int> num_lanes_in_;
    std::vector<int> num_lanes_out_;
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Edge>> edges_;
    std::vector<std::shared_ptr<CriticalResource>> critical_resources_;
}; // class Intersection

} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_INTERSECTION_H_