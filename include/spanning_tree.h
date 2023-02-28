#ifndef INTERSECTION_MANAGEMENT_SPANNING_TREE_H_
#define INTERSECTION_MANAGEMENT_SPANNING_TREE_H_

#include "intersection_utility.h"
#include "intersection.h"

namespace intersection_management {
enum class DepthType {
    Type_RegularDepth,
    Type_EdgeWeightedDepth,
    Type_EdgeNodeWeightedDepth
};
enum class FairnessType {
    Type_OrderStandardDeviation,
    Type_JainIndex
};

class SpanningTree {
public:
    SpanningTree();

    void reset(bool verbose = false);

    void AddNodesFromIntersection(const Intersection &intersection);

    void AddNode(std::shared_ptr<Node> node);

    void AddEdge(std::shared_ptr<Edge> edge);

    void AddEdge(int from, int to, double weight = 1.0);

    void UpdateDepth(int id, double depth, DepthType depth_type = DepthType::Type_RegularDepth);

    void UpdateTimeWindow(int id, double depth, double offset);
    void UpdateTimeWindow(int id, double depth, double offset, double estimate_travel_time);

    void PrintTree(bool verbose = false);
    double CalculateFairnessIndex(DepthType depth_type = DepthType::Type_RegularDepth,
                                  FairnessType fairness_type = FairnessType::Type_OrderStandardDeviation);

    std::shared_ptr<Node> p_root_;
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Edge>> edges_;
    int num_nodes_;
    double depth_;
    double edge_weighted_depth_;
    double edge_node_weighted_depth_;
};
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_SPANNING_TREE_H_