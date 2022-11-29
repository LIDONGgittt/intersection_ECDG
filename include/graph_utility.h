#ifndef INTERSECTION_MANAGEMENT_GRAPH_UTILITY_H_
#define INTERSECTION_MANAGEMENT_GRAPH_UTILITY_H_

#include <vector>
#include <memory>

namespace intersection_management {
class Edge;

class Node {
public:
    Node() : Node(-1, 1.0, -1.0, -1.0, -1.0) {}

    Node(int id) : Node(id, 1.0, -1.0, -1.0, -1.0) {}

    Node(int id, double nw, double d, double ewd, double enwd)
        : id_(id), node_weight_(nw), depth_(d), edge_weighted_depth_(ewd), edge_node_weighted_depth_(enwd) {
        edges_.clear();
    }

    void printDetail();
    bool isSameAs(int id);
    bool isSameAs(Node &n);
    bool isSameAs(std::shared_ptr<Node> &p_n);
    bool isConnectedTo(int id);
    bool isConnectedTo(Node &n);
    bool isConnectedTo(std::shared_ptr<Node> &p_n);
    std::shared_ptr<Edge> getEdgeTo(int id);
    std::shared_ptr<Edge> getEdgeTo(Node &n);
    std::shared_ptr<Edge> getEdgeTo(std::shared_ptr<Node> &p_n);

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
        bidirectional_ = false;
    }
    Edge(std::shared_ptr<Node> p_from, std::shared_ptr<Node> p_to, double weight = 1.0, bool bidirectional = false)
        : from_(p_from), to_(p_to), edge_weight_(weight), bidirectional_(bidirectional) {}

    inline bool isBidirectional() {
        return bidirectional_;
    }
    std::weak_ptr<Node> from_;
    std::weak_ptr<Node> to_;
    double edge_weight_;
    bool bidirectional_;
};
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_GRAPH_UTILITY_H_