#include "conflict_spanning_tree.h"

#include <cstdlib>
#include <time.h>
#include <iostream>
#include <cmath>
#include <queue>

namespace intersection_management {
ConflictSpanningTree::ConflictSpanningTree() {
    reset(false);
}

void ConflictSpanningTree::reset(bool verbose) {
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

void ConflictSpanningTree::AddNodesFromGraph(const ConflictDirectedGraph &graph) {
    for (auto pn : graph.nodes_) {
        AddNode(pn);
    }
    p_root_ = nodes_[0];
    p_root_->depth_ = 0;
    p_root_->edge_weighted_depth_ = 0;
    p_root_->edge_node_weighted_depth_ = 0;
}

void ConflictSpanningTree::AddNode(std::shared_ptr<Node> node) {
    AddNode(node->node_weight_);
}

void ConflictSpanningTree::AddNode(double weight) {
    auto node = std::shared_ptr<Node>(new Node(num_nodes_++, weight, -1, -1, -1));
    nodes_.push_back(node);
}

void ConflictSpanningTree::AddEdge(std::shared_ptr<Edge> edge) {
    AddEdge(edge->from_.lock()->id_, edge->to_.lock()->id_, edge->edge_weight_);
}

void ConflictSpanningTree::AddEdge(int from, int to, double weight) {
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

void ConflictSpanningTree::UpdateDepth(int id, double depth, DepthType depth_type) {
    switch (depth_type)
    {
    case RegularDepth:
        nodes_[id]->depth_ = depth;
        if (depth > depth_) {
            depth_ = depth;
        }
        break;
    case EdgeWeightedDepth:
        nodes_[id]->edge_weighted_depth_ = depth;
        if (depth > edge_weighted_depth_) {
            edge_weighted_depth_ = depth;
        }
        break;
    case EdgeNodeWeightedDepth:
        nodes_[id]->edge_node_weighted_depth_ = depth;
        if (depth > edge_node_weighted_depth_) {
            edge_node_weighted_depth_ = depth;
        }
        break;
    }
}

void ConflictSpanningTree::PrintTree(bool verbose) {
    std::cout << "***********Begin of Tree details***********\n";
    if (verbose) {
        std::cout << "Total nodes: " << num_nodes_ << std::endl;
        for (auto node : nodes_) {
            node->printDetail();
        }
    }
    std::cout << "@@Tree Depth: " << depth_;
    std::cout << "\n@@Tree Edge-Weighted Depth: " << edge_weighted_depth_;
    std::cout << "\n@@Tree Edge-Node-Weighted Depth: " << edge_node_weighted_depth_;
    std::cout << "\n***********End of Tree details*************\n";
}
} // namespace intersection_management