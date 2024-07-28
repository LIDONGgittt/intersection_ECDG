#include "conflict_directed_graph.h"


#include <iostream>
#include <cmath>
#include <queue>

namespace intersection_management {

ConflictDirectedGraph::ConflictDirectedGraph() {
    reset(false);
}

void ConflictDirectedGraph::reset(bool verbose) {
    p_root_ = std::shared_ptr<Node>(new Node(0, 0.0, 0.0, 0.0, 0.0));
    nodes_.clear();
    nodes_.push_back(p_root_);
    edges_.clear();
    num_nodes_ = 1;
    if (verbose) {
        std::cout << "The CDG is reset to a new root-only graph!\n";
    }
}

void ConflictDirectedGraph::AddNode(double weight) {
    auto node = std::shared_ptr<Node>(new Node(num_nodes_++, weight, -1, -1, -1));
    nodes_.push_back(node);
}

void ConflictDirectedGraph::AddEdge(int from, int to, double weight, bool bidirectional) {
    if (from < 0 || from >= num_nodes_ || to < 0 || to >= num_nodes_)
    {
        return;
    }
    if (nodes_[from]->isConnectedTo(to)) {
        return;
    }
    if (bidirectional) {
        if (nodes_[to]->isConnectedTo(from)) {
            return;
        }
    }

    if (to != 0) {
        auto edge = std::shared_ptr<Edge>(new Edge(nodes_[from], nodes_[to], weight, bidirectional));
        if (from == 0) {
            edge->bidirectional_ = false;
        }
        nodes_[from]->edges_.push_back(edge);
        edges_.push_back(edge);
    }
    if (bidirectional && from != 0) {
        auto edge = std::shared_ptr<Edge>(new Edge(nodes_[to], nodes_[from], weight, bidirectional));
        if (to == 0) {
            edge->bidirectional_ = false;
        }
        nodes_[to]->edges_.push_back(edge);
        edges_.push_back(edge);
    }
}

void ConflictDirectedGraph::GenerateRandomGraph(
    int total_nodes,
    double estimate_travel_time_range,
    double edge_weight_range,
    double estimate_travel_time_offset,
    double edge_weight_offset,
    bool int_weight_only) {

    this->reset(false);
    double estimate_travel_time;
    for (int id = 1; id <= total_nodes; id++) {
        estimate_travel_time = ((double)rand()) / RAND_MAX * estimate_travel_time_range + estimate_travel_time_offset;
        if (int_weight_only) {
            estimate_travel_time = std::floor(estimate_travel_time);
        }
        AddNode(estimate_travel_time);
    }

    int num_edges_to_add = (int)(num_nodes_ * num_nodes_ * (rand() % 100) * 0.01);
    int from, to;
    double edge_weight;
    while (num_edges_to_add > 0) {
        edge_weight = ((double)rand()) / RAND_MAX * edge_weight_range + edge_weight_offset;
        if (int_weight_only) {
            edge_weight = std::floor(edge_weight);
        }
        if (rand() % 2) { // add bidirectional edge
            do {
                from = rand() % num_nodes_;
                to = rand() % num_nodes_;
            } while (from == to);
            AddEdge(from, to, edge_weight, true);
            num_edges_to_add -= 2;
        }
        else { // add unidirectional edge, from.id < to.id is ensured
            do {
                from = rand() % num_nodes_;
                to = rand() % num_nodes_;
            } while (from >= to);
            AddEdge(from, to, edge_weight, false);
            num_edges_to_add--;
        }
    }

    // link root to every other node so that every node could be scheduled
    p_root_->edges_.clear();
    for (int to = 1; to <= total_nodes; to++) {
        AddEdge(0, to, 1.0, false);
    }
}


void ConflictDirectedGraph::GenerateGraphFromIntersection(Intersection &intersection) {
    // num_nodes_ = intersection.num_nodes_;
    // for (int i = 1; i < intersection.nodes_.size(); i++) {
    //     auto node = std::shared_ptr<Node>(intersection.nodes_[i]);
    //     node->depth_ = -1;
    //     node->edge_weighted_depth_ = -1;
    //     node->edge_node_weighted_depth_ = -1;
    //     nodes_.push_back(node);
    // }
    for (int i = 1; i < intersection.nodes_.size(); i++) {
        AddNode(intersection.nodes_[i]->estimate_travel_time_);
        nodes_.back()->in_lane_id_ = intersection.nodes_[i]->in_lane_id_;
        nodes_.back()->in_leg_id_ = intersection.nodes_[i]->in_leg_id_;
        nodes_.back()->out_lane_id_ = intersection.nodes_[i]->out_lane_id_;
        nodes_.back()->out_leg_id_ = intersection.nodes_[i]->out_leg_id_;
        nodes_.back()->estimate_arrival_time_ = intersection.nodes_[i]->estimate_arrival_time_;
    }
    p_root_->edges_.clear();
    for (auto edge : intersection.edges_) {
        if (edge->conflict_type_.isPrecedence()) {
            if (edge->predecessor_id_ == edge->node1_.lock()->id_) {

                AddEdge(edge->node1_.lock()->id_, edge->node2_.lock()->id_, edge->edge_weight_, false);
                edges_.back()->conflict_type_ = edge->conflict_type_;
                edges_.back()->estimate_offset_ = edge->estimate_offset_;
            }
            else {
                AddEdge(edge->node2_.lock()->id_, edge->node1_.lock()->id_, edge->edge_weight_, false);
                edges_.back()->conflict_type_ = edge->conflict_type_;
                edges_.back()->estimate_offset_ = edge->estimate_offset_;
            }
        }
        else {
            AddEdge(edge->node1_.lock()->id_, edge->node2_.lock()->id_, edge->edge_weight_, true);
            edges_.back()->conflict_type_ = edge->conflict_type_;
            edges_.back()->estimate_offset_ = edge->estimate_offset_;
            edges_[edges_.size()-2]->conflict_type_ = edge->conflict_type_;
            edges_[edges_.size()-2]->estimate_offset_ = edge->estimate_offset_;
        }
    }
}

void ConflictDirectedGraph::AddFairnessConflicts() {
    int fairness_order_diff_threshold = 50;
    double fairness_order_diff_rate = 0.5;
    if (num_nodes_ * fairness_order_diff_rate < fairness_order_diff_threshold) {
        fairness_order_diff_threshold = std::floor(num_nodes_ * fairness_order_diff_rate);
    }

    for (int from = 0; from < num_nodes_; from++) {
        for (int to = from + fairness_order_diff_threshold; to < num_nodes_; to++) {
            AddEdge(from, to, 1, false);
        }
    }
}

bool ConflictDirectedGraph::isFullyConnected() {
    std::queue<int> visit_queue;
    std::vector<bool> is_visited(num_nodes_, false);
    visit_queue.push(0);
    is_visited[0] = true;
    int from, to;
    while (!visit_queue.empty()) {
        from = visit_queue.front();
        visit_queue.pop();
        for (auto p_edge : nodes_[from]->edges_) {
            to = p_edge->node2_.lock()->id_;
            if (!is_visited[to]) {
                visit_queue.push(to);
                is_visited[to] = true;
            }
        }
    }
    for (int id = 0; id < num_nodes_; id++) {
        if (!is_visited[id]) {
            return false;
        }
    }
    return true;
}

void ConflictDirectedGraph::PrintGraph() {
    std::cout << "***********Begin of CDG details***********\n";
    std::cout << "Total nodes: " << num_nodes_ << std::endl;
    for (auto node : nodes_) {
        node->printWeightAndEdge();
    }
    std::cout << "***********End of CDG details*************\n";
}
} // namespace intersection_management