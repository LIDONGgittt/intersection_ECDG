#ifndef INTERSECTION_MANAGEMENT_INTERSECTION_UTILITY_H_
#define INTERSECTION_MANAGEMENT_INTERSECTION_UTILITY_H_

#include <vector>
#include <memory>

namespace intersection_management {
class Edge;
class CriticalResource;

class ConflictType{
    public:
    ConflictType(){
        diverging_ =false;
        converging_ = false;
        crossing_ = false;
        competing_ = false;
    }
    bool diverging_;
    bool converging_;
    bool crossing_;
    bool competing_;
};

class Node {
public:
    Node(): Node(-1, 1.0, -1.0, -1.0, -1.0) {}

    Node(int id): Node(id, 1.0, -1.0, -1.0, -1.0) {}

    Node(int id, double nw, double d, double ewd, double enwd)
        : id_(id), estimate_travel_time_(nw), depth_(d), edge_weighted_depth_(ewd), edge_node_weighted_depth_(enwd) {
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
    double estimate_travel_time_;
    double depth_;
    double edge_weighted_depth_;
    double edge_node_weighted_depth_;
    std::vector<std::shared_ptr<Edge>> edges_;

    // new attributes
    std::vector<double> time_window_;
    double estimate_arrival_time_;
    int in_bound_lane_id_;
    int in_bound_direction_id_;
    int out_bound_lane_id_;
    int out_bound_direction_id_;
    std::shared_ptr<CriticalResource> critican_resource_;
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

    // new attributes
    int predecessor_id_;
    double estimate_offset_;
    std::shared_ptr<CriticalResource> critical_resource_;
    ConflictType conflict_type_;
};

class CriticalResource {
public:
    CriticalResource(int direction_id, int num_resources): direction_id_(direction_id), num_resources_(num_resources) {}
    int direction_id_;
    int num_resources_;
    int getDirectionID();
    int getNumResources();
};
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_INTERSECTION_UTILITY_H_