#ifndef INTERSECTION_MANAGEMENT_INTERSECTION_H_
#define INTERSECTION_MANAGEMENT_INTERSECTION_H_

#include "intersection_utility.h"
#include <vector>
#include <memory>
#include <random>
#include <unordered_map>

#include "parameters.h"

namespace intersection_management {

class Intersection {
public:
    Intersection() {
        reset();
        InitializeFromParam();
        AddIntersectionUtilitiesFromGeometry();
    }
    Intersection(Parameters local_param) {
        reset();
        InitializeFromLocalParam(local_param);
        AddIntersectionUtilitiesFromGeometry();
    }

    void reset();
    void InitializeFromParam();
    void InitializeFromLocalParam(Parameters local_param);
    void AddIntersectionUtilitiesFromGeometry();
    void AddCriticalResourcesFromGeometry();
    void AddLegsAndLanesFromGeometry();
    void UpdateReferencesOfCriticalResoucesAndLegs();
    void AddNode(std::shared_ptr<Node> node);
    void AddEdge(std::shared_ptr<Edge> edge);

    void AddRandomVehicleNodes(int count, bool verbose = false);
    void AddRandomVehicleNodesWithTravelTime(int count, std::vector<double> travel_time_choice = {6.0, 6.5, 7.0}, bool verbose = false);
    void AssignRoutesToNodes();
    void AssignCriticalResourcesToNodes();
    void AssignEdgesWithSafetyOffsetToNodes();

    inline int getNumNodes() { return nodes_.size(); }
    inline int getNumEdges() { return edges_.size(); }
    inline int getNumCriticalResources() { return critical_resource_map_.size(); }
    inline int getNumLegs() { return leg_map_.size(); }
    inline int getNumLanes() { return lane_map_.size(); }
    inline void setSeed(int seed) {
        if (seed < 0) { std::random_device rd; mt_.seed(rd()); }
        else { mt_.seed(seed); }
    }

    bool isRightmostTurningRoute(std::shared_ptr<Route> route);
    bool isLeftmostTurningRoute(std::shared_ptr<Route> route);
    bool isOutermostTurningRoute(std::shared_ptr<Route> route);

    int num_legs_;
    int num_lanes_;
    std::vector<int> num_lanes_in_vec_;
    std::vector<int> num_lanes_out_vec_;
    double arrival_interval_avg_;
    std::vector<int> travel_time_range_;
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