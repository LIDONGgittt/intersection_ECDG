#ifndef intersection_management_conflict_directed_graph_h
#define intersection_management_conflict_directed_graph_h

#include "graph_utility.h"

namespace intersection_management {
    class ConflictDirectedGraph {
    public:
        ConflictDirectedGraph();

        void reset(bool verbose = true);

        void AddNode(double weight = 1.0);

        void AddEdge(int from, int to, double weight = 1.0, bool bidirectional = false);

        void GenerateRandomGraph(
            int total_nodes,
            unsigned int seed = 0,
            double max_node_weight = 5,
            double max_edge_weight = 2,
            bool int_weight_only = true);

        bool isFullyConnected();

        void PrintGraph();

        std::shared_ptr<Node> p_root_;
        std::vector<std::shared_ptr<Node>> nodes_;
        std::vector<std::shared_ptr<Edge>> edges_;
        int num_nodes_;
    };
}
#endif