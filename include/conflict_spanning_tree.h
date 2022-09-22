#ifndef intersection_management_conflict_spanning_tree_h
#define intersection_management_conflict_spanning_tree_h

#include "graph_utility.h"

namespace intersection_management {
    class ConflictSpanningTree {
    public:
        ConflictSpanningTree();

        void reset();

        std::shared_ptr<Node> p_root_;
        std::vector<std::shared_ptr<Node>> nodes_;
        std::vector<std::shared_ptr<Edge>> edges_;
        int count_node_;
    };
}
#endif