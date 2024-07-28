#ifndef INTERSECTION_MANAGEMENT_CDG_SCHEDULER_H_
#define INTERSECTION_MANAGEMENT_CDG_SCHEDULER_H_

#include "conflict_directed_graph.h"
#include "cdg_conflict_spanning_tree.h"
#include <iostream>
#include <algorithm>

namespace intersection_management {

class CDGCandidate {
public:
    CDGCandidate(int id, double depth, int parent, double edge_weight, double estimate_travel_time = -1.0) :
        id_(id), possible_depth_(depth), id_possible_parent_(parent), edge_weight_(edge_weight), estimate_travel_time_(estimate_travel_time) {}

    int id_;
    double possible_depth_;
    int id_possible_parent_;
    double edge_weight_;
    double estimate_travel_time_;
};

class CDGScheduler {
public:
    CDGScheduler();
    CDGConflictSpanningTree ScheduleWithModifiedDfst(const ConflictDirectedGraph &cdg);
    CDGConflictSpanningTree ScheduleWithBfstWeightedEdgeOnly(const ConflictDirectedGraph &cdg);
    CDGConflictSpanningTree ScheduleWithBfstMultiWeight(const ConflictDirectedGraph &cdg);
    CDGConflictSpanningTree ScheduleWithDfstMultiWeight(const ConflictDirectedGraph &cdg);
    std::vector<int> ScheduleBruteForceSearch(const ConflictDirectedGraph &cdg);

    void PrepareForTreeSchedule(const ConflictDirectedGraph &cdg);
    void GenerateUniparentTable(const ConflictDirectedGraph &cdg);
    void GenerateBineighborTable(const ConflictDirectedGraph &cdg);

    void SearchOrderPermutationRecursively(std::vector<int> &vehicle_order, int num_nodes,
                                           std::vector<bool> &is_in_order_list,
                                           double &minimum_evacuation_time, std::vector<int> &best_order,
                                           const ConflictDirectedGraph &cdg);
    double GetEvacuationTimeFromOrder(const std::vector<int> &vehicle_order,
                                      const ConflictDirectedGraph &cdg);
    std::vector<double> GetDepthVectorFromOrder(const std::vector<int> &vehicle_order,
                                                const ConflictDirectedGraph &cdg);
    
    static inline void SortReadyListAscendingly(std::vector<CDGCandidate> &ready_list) {
        std::sort(ready_list.begin(), ready_list.end(),
                  [](const CDGCandidate &a, const CDGCandidate &b) {
                      if (a.possible_depth_ == b.possible_depth_) {
                          return a.id_ < b.id_;
                      }
                      return a.possible_depth_ < b.possible_depth_;
                  });
    }
    static inline bool isInList(int id, std::vector<CDGCandidate> &ready_list) {
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

    CDGConflictSpanningTree result_tree_;
    std::vector<std::vector<std::shared_ptr<Node>>> unidirectional_parent_table_;
    std::vector<std::vector<std::shared_ptr<Node>>> bidirectional_neighbor_table_;
};

} // namespace intersection_management

#endif // INTERSECTION_MANAGEMENT_CDG_SCHEDULER_H_