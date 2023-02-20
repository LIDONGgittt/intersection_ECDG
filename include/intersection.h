#ifndef INTERSECTION_MANAGEMENT_INTERSECTION_H_
#define INTERSECTION_MANAGEMENT_INTERSECTION_H_

#include "intersection_utility.h"
#include <vector>
#include <memory>

namespace intersection_management {

class Intersection {
public:
    Intersection() {
        reset();
        InitializeFromParam();
    }

    void InitializeFromParam();
    void reset();
    void AddNode(std::shared_ptr<Node> node);
    int getNumNodes();
    void AddEdge(std::shared_ptr<Edge> edge);
    int getNumEdges();
    void AddCriticalResourceFromGeometric();
    int getNumCriticalResource();

    int num_directions_;
    std::vector<int> num_lanes_in_;
    std::vector<int> num_lanes_out_;
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Edge>> edges_;
    std::vector<std::shared_ptr<CriticalResource>> critical_resources_;
}; // class Intersection

} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_INTERSECTION_H_