#include "cdg_conflict_spanning_tree.h"

#include <cstdlib>
#include <time.h>
#include <iostream>
#include <cmath>
#include <queue>
#include <algorithm>
#include <limits>

namespace intersection_management {
CDGConflictSpanningTree::CDGConflictSpanningTree() {
    reset(false);
}

void CDGConflictSpanningTree::reset(bool verbose) {
    p_root_ = nullptr;
    nodes_.clear();
    edges_.clear();
    num_nodes_ = 0;
    depth_ = -1;
    edge_weighted_depth_ = -1;
    edge_node_weighted_depth_ = -1;
    if (verbose) {
        std::cout << "The Tree is reset to empty!\n";
    }
}

void CDGConflictSpanningTree::AddNodesFromGraph(const ConflictDirectedGraph &graph) {
    for (auto pn : graph.nodes_) {
        AddNode(pn);
        nodes_.back()->in_lane_id_ = pn->in_lane_id_;
        nodes_.back()->in_leg_id_ = pn->in_leg_id_;
        nodes_.back()->out_lane_id_ = pn->out_lane_id_;
        nodes_.back()->out_leg_id_ = pn->out_leg_id_;
        nodes_.back()->estimate_arrival_time_ = pn->estimate_arrival_time_;
    }
    p_root_ = nodes_[0];
    p_root_->depth_ = 0;
    p_root_->edge_weighted_depth_ = 0;
    p_root_->edge_node_weighted_depth_ = 0;
}

void CDGConflictSpanningTree::AddNode(std::shared_ptr<Node> node) {
    AddNode(node->estimate_travel_time_);
}

void CDGConflictSpanningTree::AddNode(double weight) {
    auto node = std::shared_ptr<Node>(new Node(num_nodes_++, weight, -1, -1, -1));
    nodes_.push_back(node);
}

void CDGConflictSpanningTree::AddEdge(std::shared_ptr<Edge> edge) {
    AddEdge(edge->node1_.lock()->id_, edge->node2_.lock()->id_, edge->edge_weight_);
}

void CDGConflictSpanningTree::AddEdge(int from, int to, double weight) {
    if (from < 0 || from >= num_nodes_ || to < 0 || to >= num_nodes_)
    {
        return;
    }
    if (nodes_[from]->isConnectedTo(to)) {
        return;
    }

    if (to != 0) {
        auto edge = std::shared_ptr<Edge>(new Edge(nodes_[from], nodes_[to], weight, false));
        nodes_[from]->edges_.push_back(edge);
        edges_.push_back(edge);
    }
}

void CDGConflictSpanningTree::UpdateDepth(int id, double depth, CDGDepthType depth_type) {
    switch (depth_type)
    {
    case Type_RegularDepth:
        nodes_[id]->depth_ = depth;
        if (depth > depth_) {
            depth_ = depth;
        }
        break;
    case Type_EdgeWeightedDepth:
        nodes_[id]->edge_weighted_depth_ = depth;
        if (depth > edge_weighted_depth_) {
            edge_weighted_depth_ = depth;
        }
        break;
    case Type_EdgeNodeWeightedDepth:
        nodes_[id]->edge_node_weighted_depth_ = depth;
        if (depth > edge_node_weighted_depth_) {
            edge_node_weighted_depth_ = depth;
        }
        break;
    }
}

void CDGConflictSpanningTree::PrintTree(bool verbose) {
    std::cout << "***********Begin of Tree details***********\n";
    if (verbose) {
        std::cout << "Total nodes: " << num_nodes_ << std::endl;
        for (auto node : nodes_) {
            node->printWeightAndEdge();
        }
    }
    std::cout << "@@Tree Depth: " << depth_;
    std::cout << "\n@@Tree Edge-Weighted Depth: " << edge_weighted_depth_;
    std::cout << "\n@@Tree Edge-Node-Weighted Depth: " << edge_node_weighted_depth_;
    std::cout << "\n***********End of Tree details*************\n";
}

double CDGConflictSpanningTree::CalculateFairnessIndex(CDGDepthType depth_type, CDGFairnessType fairness_type) {
    std::vector<std::pair<int, double>> id_order_vec;
    for (auto i = 1u; i < nodes_.size(); i++) {
        auto &node = nodes_[i];
        double depth;
        switch (depth_type)
        {
        case Type_RegularDepth:
            depth = node->depth_;
            break;
        case Type_EdgeWeightedDepth:
            depth = node->edge_weighted_depth_;
            break;
        case Type_EdgeNodeWeightedDepth:
            depth = node->edge_node_weighted_depth_;
            break;
        }
        id_order_vec.push_back(std::pair<int, double>{node->id_, depth});
    }
    sort(id_order_vec.begin(), id_order_vec.end(),
         [](const std::pair<int, double> &p1, const std::pair<int, double> &p2) { return p1.second < p2.second; });
    std::vector<double> order_diff_vec;
    for (auto i = 0u; i < id_order_vec.size(); i++) {
        order_diff_vec.push_back(id_order_vec[i].first - (int)(i + 1));
    }

    double fairness_index = 0;
    switch (fairness_type)
    {
    case Type_OrderStandardDeviation:
        fairness_index = 0;
        for (auto &diff : order_diff_vec)
            fairness_index += std::pow(diff, 2);
        fairness_index = fairness_index / order_diff_vec.size();
        break;
    case Type_JainIndex: {
        double minimum_diff = std::numeric_limits<double>::max();
        for (auto &diff : order_diff_vec)
            if (minimum_diff > diff)
                minimum_diff = diff;
        if (minimum_diff < 0) {
            double numerator = 0;
            double denominator = 0;
            for (auto &diff : order_diff_vec) {
                numerator += diff - minimum_diff;
                denominator += std::pow(diff - minimum_diff, 2);
            }
            numerator = std::pow(numerator, 2);
            denominator *= order_diff_vec.size();
            fairness_index = numerator / denominator;
        }
        else {
            fairness_index = 1.0;
        }
        break;
    }
    default:
        fairness_index = -1;
        break;
    }
    return fairness_index;
}
} // namespace intersection_management