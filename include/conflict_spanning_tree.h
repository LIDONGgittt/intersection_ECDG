#ifndef INTERSECTION_MANAGEMENT_CONFLICT_SPANNING_TREE_H_
#define INTERSECTION_MANAGEMENT_CONFLICT_SPANNING_TREE_H_

#include "graph_utility.h"
#include "conflict_directed_graph.h"

namespace intersection_management {
class ConflictSpanningTree {
public:
    ConflictSpanningTree();

    void reset(bool verbose = true);

    void AddNodesFromGraph(const ConflictDirectedGraph &graph);

    void AddNode(std::shared_ptr<Node> node);

    void AddNode(double weight = 1.0);

    void AddEdge(std::shared_ptr<Edge> edge);

    void AddEdge(int from, int to, double weight = 1.0, bool bidirectional = false);

    void UpdateDepth(int id, double depth);
    void UpdateEdgeWeightedDepth(int id, double depth);
    void UpdateEdgeNodeWeightedDepth(int id, double depth);

    void PrintTree(bool verbose = true);

    std::shared_ptr<Node> p_root_;
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Edge>> edges_;
    int num_nodes_;
    double depth_;
    double edge_weighted_depth_;
    double edge_node_weighted_depth_;
};
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_CONFLICT_SPANNING_TREE_H_