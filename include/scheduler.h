#ifndef intersection_management_scheduler_h
#define intersection_management_scheduler_h

#include "conflict_directed_graph.h"
#include "conflict_spanning_tree.h"

namespace intersection_management {

    class Candidate {
    public:
        Candidate(int id, double depth, int parent, double edge_weight) :
            id_(id), possible_depth_(depth), id_possible_parent_(parent), edge_weight_(edge_weight) {}

        int id_;
        double possible_depth_;
        int id_possible_parent_;
        double edge_weight_;
    };

    class Scheduler {
    public:
        Scheduler();
        ConflictSpanningTree ScheduleWithModifiedDfst(const ConflictDirectedGraph &cdg);
        ConflictSpanningTree ScheduleWithBfstWeightedEdgeOnly(const ConflictDirectedGraph &cdg);
        ConflictSpanningTree ScheduleWithBfstMultiWeight(const ConflictDirectedGraph &cdg);
        bool isInList(int id, std::vector<Candidate> &ready_list);
        bool StillHasPredecessor(std::vector<std::shared_ptr<Node>> &pre, std::vector<bool> &added_to_tree);

        ConflictSpanningTree result_tree_;

    };

}

#endif