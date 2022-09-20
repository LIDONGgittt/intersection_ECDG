#ifndef intersection_management_graph_utility_h
#define intersection_management_graph_utility_h

#include <vector>
#include <memory>

namespace intersection_management {
    class Edge;

    class Node {
    public:
        Node() : Node(-1, 1.0, -1.0, -1.0, -1.0) {}

        Node(int id, double nw, double d, double ewd, double enwd) : id_(id), node_weight_(nw), depth_(d), edge_weighted_depth_(ewd), edge_node_weighted_depth_(enwd) {
            edges_.clear();
        }

        int id_;
        double node_weight_;
        double depth_;
        double edge_weighted_depth_;
        double edge_node_weighted_depth_;
        std::vector<std::shared_ptr<Edge>> edges_;
    };

    class Edge {
    public:
        Edge() {
            from_.reset();
            to_.reset();
            edge_weight_ = 1.0;
        }
        Edge(std::weak_ptr<Node> from, std::weak_ptr<Node> to, double weight = 1.0) : from_(from), to_(to), edge_weight_(weight) {}
        std::weak_ptr<Node> from_;
        std::weak_ptr<Node> to_;
        double edge_weight_;
    };
}
#endif