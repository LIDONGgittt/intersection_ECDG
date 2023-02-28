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
    SpanningTree ScheduleWithDynamicLaneAssignment(Intersection &intersection);
    SpanningTree ScheduleWithModifiedDfst(const Intersection &intersection);
    SpanningTree ScheduleWithBfstWeightedEdgeOnly(const Intersection &intersection);
    SpanningTree ScheduleWithBfstMultiWeight(const Intersection &intersection);
    std::vector<int> ScheduleBruteForceSearch(const Intersection &intersection);

    void PrepareForTreeSchedule(const Intersection &intersection);
    void GenerateUniparentTable(const Intersection &intersection);
    void GenerateBineighborTable(const Intersection &intersection);

    void SearchOrderPermutationRecursively(std::vector<int> &vehicle_order, int num_nodes,
                                           std::vector<bool> &is_in_order_list,
                                           double &minimum_evacuation_time, std::vector<int> &best_order,
                                           const Intersection &intersection);
    double GetEvacuationTimeFromOrder(const std::vector<int> &vehicle_order,
                                      const Intersection &intersection);
    std::vector<double> GetDepthVectorFromOrder(const std::vector<int> &vehicle_order,
                                                const Intersection &intersection);

    static inline void SortReadyListAscendingly(std::vector<Candidate> &ready_list) {
        std::sort(ready_list.begin(), ready_list.end(),
                  [](const Candidate &a, const Candidate &b) {
                      if (a.possible_depth_ == b.possible_depth_) {
                          if (a.split_flexible_critical_resource_ && !b.split_flexible_critical_resource_)
                              return false;
                          if (!a.split_flexible_critical_resource_ && b.split_flexible_critical_resource_)
                              return true;
                          if (a.split_flexible_critical_resource_ && b.split_flexible_critical_resource_ &&
                              (a.num_critical_resource_splitted_ != b.num_critical_resource_splitted_))
                              return a.num_critical_resource_splitted_ < b.num_critical_resource_splitted_;
                          return a.id_ < b.id_;

                      }
                      return a.possible_depth_ < b.possible_depth_;
                  });
    }
    static inline bool isInList(int id, std::vector<Candidate> &ready_list) {
        for (auto &item : ready_list) {
            if (id == item.id_) {
                return true;
            }
        }
        return false;
    }
    static inline bool StillHasUnscheduledPredecessor(std::vector<std::shared_ptr<Node>> &pre, std::vector<bool> &added_to_tree) {
        for (auto node : pre) {
            if (added_to_tree[node->id_] == false) {
                return true;
            }
        }
        return false;
    }
    static void printDepthVector(std::vector<double> &depth_vector) {
        int tmp_cnt = 0;
        for (int i = 0; i < depth_vector.size(); i++) {
            std::cout << "Node " << i << ": " << depth_vector[i] << ", ";
            if (++tmp_cnt % 6 == 0)
                std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    static void printOrder(std::vector<int> &order) {
        int tmp_cnt = 0;
        for (auto id : order) {
            std::cout << "Node " << id << " -> ";
            if (++tmp_cnt % 10 == 0)
                std::cout << std::endl;
        }
        std::cout << std::endl;
    }

    SpanningTree result_tree_;
    std::vector<std::vector<std::shared_ptr<Node>>> unidirectional_parent_table_;
    std::vector<std::vector<std::shared_ptr<Node>>> bidirectional_neighbor_table_;
    bool enable_optimized_precedence_offset_;
};

} // namespace intersection_management

#endif // INTERSECTION_MANAGEMENT_SCHEDULER_H_