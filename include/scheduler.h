#ifndef INTERSECTION_MANAGEMENT_SCHEDULER_H_
#define INTERSECTION_MANAGEMENT_SCHEDULER_H_

#include "intersection.h"
#include "spanning_tree.h"
#include <iostream>
#include <algorithm>

namespace intersection_management {

class Candidate {
public:
    Candidate(int id, double depth, int parent, double estimate_offset = 0, int out_leg_id = -1,
              int out_lane_id = -1, double estimate_travel_time = 0)
        :id_(id), possible_depth_(depth), id_possible_parent_(parent), estimate_offset_(estimate_offset),
        out_leg_id_(out_leg_id), out_lane_id_(out_lane_id), estimate_travel_time_(estimate_travel_time) {
        split_flexible_critical_resource_ = false;
        num_critical_resource_splitted_ = 0;
        competing_node_id_ = 0;
    }

    int id_;
    double possible_depth_;
    int id_possible_parent_;
    double estimate_offset_;
    double estimate_travel_time_;
    int out_leg_id_; // same as critical resource id
    int out_lane_id_;
    bool split_flexible_critical_resource_;
    int num_critical_resource_splitted_;
    int competing_node_id_;
};

class Scheduler {

public:
    Scheduler();
    SpanningTree ScheduleWithFIFO(Intersection &intersection);

    void PrepareForTreeSchedule(Intersection &intersection);
    void GenerateUniparentTable(Intersection &intersection);
    void GenerateBineighborTable(Intersection &intersection);

    void SortReadyListAscendingly(std::vector<Candidate> &ready_list, Intersection &intersection);

    static bool isInList(int id, std::vector<Candidate> &ready_list);
    static bool StillHasUnscheduledPredecessor(std::vector<std::shared_ptr<Node>> &pre, std::vector<bool> &added_to_tree);
    static void printDepthVector(std::vector<double> &depth_vector);
    static void printOrder(std::vector<int> &order);

    SpanningTree result_tree_;
    std::vector<std::vector<std::shared_ptr<Node>>> unidirectional_parent_table_;
    std::vector<std::vector<std::shared_ptr<Node>>> bidirectional_neighbor_table_;
    std::vector<int> remaining_demand_per_lane_;
};

} // namespace intersection_management

#endif // INTERSECTION_MANAGEMENT_SCHEDULER_H_