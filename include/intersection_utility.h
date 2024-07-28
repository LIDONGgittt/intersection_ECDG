#ifndef INTERSECTION_MANAGEMENT_INTERSECTION_UTILITY_H_
#define INTERSECTION_MANAGEMENT_INTERSECTION_UTILITY_H_

#include <vector>
#include <memory>
#include <cmath>
#include <unordered_map>

namespace intersection_management {
class Route;
class Leg;
class CriticalResource;
class Lane;
class Node;
class Edge;
class ConflictType;

class ConflictType {
public:
    ConflictType() {
        diverging_ = false;
        converging_ = false;
        crossing_ = false;
        competing_ = false;
        precedence_ = false;
    }
    inline void setDiverging() { diverging_ = true; }
    inline void unsetDiverging() { diverging_ = false; }
    inline void setConverging() { converging_ = true; }
    inline void unsetConverging() { converging_ = false; }
    inline void setCrossing() { crossing_ = true; }
    inline void unsetCrossing() { crossing_ = false; }
    inline void setCompeting() { competing_ = true; }
    inline void unsetCompeting() { competing_ = false; }
    inline void setPrecedence() { precedence_ = true; }
    inline void unsetPrecedence() { precedence_ = false; }
    inline bool isDiverging() { return diverging_; }
    inline bool isConverging() { return converging_; }
    inline bool isCrossing() { return crossing_; }
    inline bool isCompeting() { return competing_; }
    inline bool isPrecedence() { return precedence_; }
    inline bool isNotConflicting() { return !(diverging_ || converging_ || crossing_ || competing_ || precedence_); }

    bool diverging_;
    bool converging_;
    bool crossing_;
    bool competing_;
    bool precedence_;
};

class Node {
public:
    Node(): Node(-1, 1.0, -1.0, -1.0, -1.0) {}
    Node(int id): Node(id, 1.0, -1.0, -1.0, -1.0) {}
    Node(int id, double ett, double d, double ewd, double enwd);
    Node(int id, double ett, int in_leg_id, int in_lane_id, int out_leg_id, int out_lane_id);
    Node(int id, double ett, int in_leg_id, int in_lane_id, int out_leg_id, int out_lane_id, double eat);
    Node(std::shared_ptr<Node> &p_node): Node(*p_node) {}
    Node(const Node &node);

    void printWeightAndEdge();
    void printDetail();
    bool isSameAs(int id);
    bool isSameAs(Node &n);
    bool isSameAs(std::shared_ptr<Node> &p_n);
    bool isConnectedTo(int id); // unidirectional, node with id should be as the node2 in the edge
    bool isConnectedTo(Node &n);
    bool isConnectedTo(std::shared_ptr<Node> &p_n);
    std::shared_ptr<Edge> getEdgeTo(int id); // unidirectional, node with id should be as the node2 in the edge
    std::shared_ptr<Edge> getEdgeTo(Node &n);
    std::shared_ptr<Edge> getEdgeTo(std::shared_ptr<Node> &p_n);
    bool isConnectedWith(int id, bool bidirectional = true); // bidirectional, any edge connect with node with id
    bool isConnectedWith(Node &n);
    bool isConnectedWith(std::shared_ptr<Node> &p_n);
    std::shared_ptr<Edge> getEdgeWith(int id, bool bidirectional = true); // bidirectional, any edge connect with node with id
    std::shared_ptr<Edge> getEdgeWith(Node &n);
    std::shared_ptr<Edge> getEdgeWith(std::shared_ptr<Node> &p_n);

    int id_;
    double estimate_travel_time_;
    double depth_;
    double edge_weighted_depth_;
    double edge_node_weighted_depth_;
    std::vector<std::shared_ptr<Edge>> edges_;

    // new attributes
    std::vector<double> time_window_;
    double estimate_arrival_time_;
    int in_lane_id_;
    int in_leg_id_;
    int out_lane_id_;
    int out_leg_id_;
    int assigned_lane_id_;
    std::vector<int> possible_lane_id_;
    std::shared_ptr<CriticalResource> critical_resource_;
    std::shared_ptr<Route> route_;
};

class Edge {
public:
    Edge() {
        node1_.reset();
        node2_.reset();
        edge_weight_ = 1.0;
        bidirectional_ = false;
    }
    Edge(std::shared_ptr<Node> p_node1, std::shared_ptr<Node> p_node2, double weight = 1.0, bool bidirectional = false)
        : node1_(p_node1), node2_(p_node2), edge_weight_(weight), bidirectional_(bidirectional) {
        estimate_offset_ = weight;
        predecessor_id_ = -1;
        critical_resource_.reset();
    }

    Edge(std::shared_ptr<Node> p_node1, std::shared_ptr<Node> p_node2, double eo, ConflictType ct, int pred_id = -1)
        : Edge(p_node1, p_node2, std::max(eo, 1.0), false) {
        estimate_offset_ = eo;
        predecessor_id_ = pred_id;
        conflict_type_ = ct;
    }

    inline bool isBidirectional() {
        return bidirectional_;
    }
    std::weak_ptr<Node> node1_;
    std::weak_ptr<Node> node2_;
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
    CriticalResource(int leg_id, int num_resources): leg_id_(leg_id), num_resources_(num_resources) {
        nodes_.clear();
    }
    int getLegId();
    int getNumResources();
    int leg_id_;
    int num_resources_;
    std::vector<std::weak_ptr<Node>> nodes_;
    std::weak_ptr<Leg> leg_;
};

class Route {
public:
    Route(std::shared_ptr<Lane> lane_in = nullptr, std::shared_ptr<Lane> lane_out = nullptr) {
        lane_in_ = lane_in;
        lane_out_ = lane_out;
    }
    ConflictType FindConflictTypeWithRoute(std::shared_ptr<Route> other_route);

    inline std::shared_ptr<Lane> getLaneIn() { return lane_in_; };
    inline std::shared_ptr<Lane> getLaneOut() { return lane_out_; };
    std::shared_ptr<Lane> lane_in_;
    std::shared_ptr<Lane> lane_out_;
};

class Leg {
public:
    Leg(int id):id_(id) {
        lanes_in_map_.clear();
        lanes_out_map_.clear();
        critical_resource_.reset();
    }
    inline int getId() { return id_; }
    inline int getNumLanesIn() { return lanes_in_map_.size(); }
    inline int getNumLanesOut() { return lanes_out_map_.size(); }
    inline std::shared_ptr<CriticalResource> getCriticalResource() { return critical_resource_; }
    
    int id_;
    std::unordered_map<int, std::shared_ptr<Lane>> lanes_in_map_;
    std::unordered_map<int, std::shared_ptr<Lane>> lanes_out_map_;
    std::shared_ptr<CriticalResource> critical_resource_;
};

class Lane {
public:
    Lane(int id, char sd, int unique_id = -1, std::shared_ptr<Leg> leg = nullptr)
        : id_(id), stream_direction_(sd), unique_id_(unique_id), leg_(leg) {}
    inline int getId() { return id_; }
    inline int getUniqueId() { return unique_id_; }
    inline int getLegId() { return leg_.lock()->getId(); }
    inline char getStreamDirection() { return stream_direction_; }
    inline bool isInBound() { return (stream_direction_ == 'i' || stream_direction_ == 'I'); }
    inline bool isOutBound() { return (stream_direction_ == 'o' || stream_direction_ == 'O'); }

    int id_;
    int unique_id_;
    char stream_direction_;
    std::weak_ptr<Leg> leg_;
};

// class 
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_INTERSECTION_UTILITY_H_