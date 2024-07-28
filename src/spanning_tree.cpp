#include "spanning_tree.h"

#include <iostream>
#include <vector>

namespace intersection_management {
SpanningTree::SpanningTree() {
    reset(false);
}

void SpanningTree::reset(bool verbose) {
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

void SpanningTree::AddNodesFromIntersection(const Intersection &intersection) {
    for (auto node : intersection.nodes_) {
        AddNode(node);
    }
    num_nodes_ = intersection.num_nodes_;
    p_root_ = nodes_[0];
    p_root_->depth_ = 0;
    p_root_->edge_weighted_depth_ = 0;
    p_root_->edge_node_weighted_depth_ = 0;
}

void SpanningTree::AddNode(std::shared_ptr<Node> node) {
    auto node_copy = std::make_shared<Node>(node);
    nodes_.push_back(node_copy);
}

void SpanningTree::AddEdge(std::shared_ptr<Edge> edge) {
    AddEdge(edge->node1_.lock()->id_, edge->node2_.lock()->id_, edge->edge_weight_);
}

void SpanningTree::AddEdge(int from, int to, double weight) {
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

void SpanningTree::UpdateDepth(int id, double depth, DepthType depth_type) {
    switch (depth_type)
    {
    case DepthType::Type_RegularDepth:
        nodes_[id]->depth_ = depth;
        if (depth > depth_) {
            depth_ = depth;
        }
        break;
    case DepthType::Type_EdgeWeightedDepth:
        nodes_[id]->edge_weighted_depth_ = depth;
        if (depth > edge_weighted_depth_) {
            edge_weighted_depth_ = depth;
        }
        break;
    case DepthType::Type_EdgeNodeWeightedDepth:
        nodes_[id]->edge_node_weighted_depth_ = depth;
        if (depth > edge_node_weighted_depth_) {
            edge_node_weighted_depth_ = depth;
        }
        break;
    }
}

void SpanningTree::UpdateTimeWindow(int id, double depth, double offset) {
    UpdateTimeWindow(id, depth, offset, nodes_[id]->estimate_travel_time_);
}
void SpanningTree::UpdateTimeWindow(int id, double depth, double offset, double estimate_travel_time) {
    nodes_[id]->depth_ = depth;
    nodes_[id]->time_window_[1] = depth;
    nodes_[id]->time_window_[0] = depth - estimate_travel_time - offset;
    nodes_[id]->estimate_travel_time_ = estimate_travel_time;
    if (depth > depth_)
        depth_ = depth;
}

void SpanningTree::PrintTree(bool verbose) {
    std::cout << "***********Begin of Spanning Tree details***********\n";
    if (verbose) {
        std::cout << "Total nodes: " << num_nodes_ << std::endl;
        for (auto node : nodes_) {
            node->printWeightAndEdge();
        }
    }
    std::cout << "@@Tree Depth: " << depth_;
    std::cout << "\n@@Tree Edge-Weighted Depth: " << edge_weighted_depth_;
    std::cout << "\n@@Tree Edge-Node-Weighted Depth: " << edge_node_weighted_depth_;
    std::cout << "\n***********End of Spanning Tree details*************\n";
}

} // namespace intersection_management