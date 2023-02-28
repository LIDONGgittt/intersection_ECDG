#ifndef INTERSECTION_MANAGEMENT_INTERSECTION_H_
#define INTERSECTION_MANAGEMENT_INTERSECTION_H_

#include "intersection_utility.h"
#include <vector>
#include <memory>
#include <random>
#include <unordered_map>

namespace intersection_management {

class Intersection {
public:
    Intersection() {
        reset();
        InitializeFromParam();
    }

    void reset();
    void InitializeFromParam();
    void AddIntersectionUtilitiesFromGeometric();
    void AddCriticalResourcesFromGeometric();
    void AddLegsAndLanesFromGeometric();
    void UpdateReferencesOfCriticalResoucesAndLegs();
    void AddNode(std::shared_ptr<Node> node);
    void AddEdge(std::shared_ptr<Edge> edge);
    void AddRandomVehicleNodes(int count);
    void AssignRoutesToNodes();
    void AssignCriticalResourcesToNodes();
    void AssignEdgesWithSafetyOffsetToNodes();

    inline int getNumNodes() { return nodes_.size(); }
    inline int getNumEdges() { return edges_.size(); }
    inline int getNumCriticalResources() { return critical_resource_map_.size(); }
    inline int getNumLegs() { return leg_map_.size(); }
    inline int getNumLanes() { return lane_map_.size(); }
    inline void setSeed(int seed) {
        if (seed < 0) {
            std::random_device rd;
            mt_.seed(rd());
        }
        else {
            mt_.seed(seed);
        }
    }

    int num_legs_;
    std::vector<int> num_lanes_in_vec_;
    std::vector<int> num_lanes_out_vec_;
    int num_nodes_;
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Edge>> edges_;
    std::unordered_map<int, std::shared_ptr<CriticalResource>> critical_resource_map_;
    std::unordered_map<int, std::shared_ptr<Leg>> leg_map_;
    std::unordered_map<int, std::shared_ptr<Lane>> lane_map_;
    std::mt19937 mt_;
}; // class Intersection

} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_INTERSECTION_H_