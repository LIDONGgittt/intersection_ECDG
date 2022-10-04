#ifndef INTERSECTION_MANAGEMENT_SCHEDULER_H_
#define INTERSECTION_MANAGEMENT_SCHEDULER_H_

#include "conflict_directed_graph.h"
#include "conflict_spanning_tree.h"

namespace intersection_management {

class Candidate {
public:
    Candidate(int id, double depth, int parent, double edge_weight, double node_weight = -1.0) :
        id_(id), possible_depth_(depth), id_possible_parent_(parent), edge_weight_(edge_weight), node_weight_(node_weight) {}

    int id_;
    double possible_depth_;
    int id_possible_parent_;
    double edge_weight_;
    double node_weight_;
};

class Scheduler {
public:
    Scheduler();
    ConflictSpanningTree ScheduleWithModifiedDfst(const ConflictDirectedGraph &cdg);
    ConflictSpanningTree ScheduleWithBfstWeightedEdgeOnly(const ConflictDirectedGraph &cdg);
    ConflictSpanningTree ScheduleWithBfstMultiWeight(const ConflictDirectedGraph &cdg);
    std::vector<int> ScheduleBruteForceSearch(const ConflictDirectedGraph &cdg);

    void GenerateUniparentTable(const ConflictDirectedGraph &cdg);
    void GenerateBineighborTable(const ConflictDirectedGraph &cdg);
    bool isInList(int id, std::vector<Candidate> &ready_list);
    bool StillHasPredecessor(std::vector<std::shared_ptr<Node>> &pre, std::vector<bool> &added_to_tree);

    std::vector<double> GetDepthVectorFromOrder(const std::vector<int> &vehicle_order,
                                                const ConflictDirectedGraph &cdg);
    void printDepthVector(std::vector<double> &depth_vector);
    void printOrder(std::vector<int> &order);
    double GetEvacuationTimeFromOrder(const std::vector<int> &vehicle_order,
                                      const ConflictDirectedGraph &cdg);
    void SearchRecursively(std::vector<int> &vehicle_order, int num_nodes,
                           std::vector<bool> &is_in_order_list,
                           double &minimum_evacuation_time, std::vector<int> &best_order,
                           const ConflictDirectedGraph &cdg);

    ConflictSpanningTree result_tree_;
    std::vector<std::vector<std::shared_ptr<Node>>> unidirectional_parent_table_;
    std::vector<std::vector<std::shared_ptr<Node>>> bidirectional_neighbor_table_;
};

} // namespace intersection_management

#endif // INTERSECTION_MANAGEMENT_SCHEDULER_H_