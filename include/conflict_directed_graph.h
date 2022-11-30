#ifndef INTERSECTION_MANAGEMENT_CONFLICT_DIRECTED_GRAPH_H_
#define INTERSECTION_MANAGEMENT_CONFLICT_DIRECTED_GRAPH_H_

#include "graph_utility.h"

namespace intersection_management {
class ConflictDirectedGraph {
public:
    ConflictDirectedGraph();

    void reset(bool verbose = true);

    void AddNode(double weight = 1.0);

    void AddEdge(int from, int to, double weight = 1.0, bool bidirectional = false);

    void GenerateRandomGraph(int total_nodes,
                             double node_weight_range = 5.0,
                             double edge_weight_range = 2.0,
                             double node_weight_offset = 1.0,
                             double edge_weight_offset = 1.0,
                             bool int_weight_only = true);
    
    void AddFairnessConflicts();

    bool isFullyConnected();

    void PrintGraph();

    std::shared_ptr<Node> p_root_;
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Edge>> edges_;
    int num_nodes_;
};

} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_CONFLICT_DIRECTED_GRAPH_H_