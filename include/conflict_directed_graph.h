#ifndef intersection_management_conflict_directed_graph_h
#define intersection_management_conflict_directed_graph_h

#include "graph_utility.h"

namespace intersection_management {
    class ConflictDirectedGraph {
    public:
        ConflictDirectedGraph() {
            root_ = std::shared_ptr<Node>(new Node(0, 0.0, 0.0, 0.0, 0.0));
            nodes_.clear();
            nodes_.push_back(root_);
            edges_.clear();
            count_node_ = 1;
        }

        void AddNode(double weight = 1.0) {
            auto node = std::shared_ptr<Node>(new Node(count_node_++, weight, -1, -1, -1));
            nodes_.push_back(node);
        }

        void AddEdge(int from, int to, double weight = 1.0) {
            if (from < 0 || from >= nodes_.size() || to < 0 || to >= nodes_.size())
            {
                return;
            }
            auto edge = std::shared_ptr<Edge>(new Edge(nodes_[from], nodes_[to], weight));
        }

        void GenerateRandomGraph() {
            return;
        }

        void reset() {
            root_ = std::shared_ptr<Node>(new Node(0, 0.0, 0.0, 0.0, 0.0));
            nodes_.clear();
            nodes_.push_back(root_);
            edges_.clear();
            count_node_ = 1;
            std::cout << "The CDG is reset to a new root-only graph!\n";
        }

        std::shared_ptr<Node> root_;
        std::vector<std::shared_ptr<Node>> nodes_;
        std::vector<std::shared_ptr<Edge>> edges_;
        int count_node_;
    };
}
#endif