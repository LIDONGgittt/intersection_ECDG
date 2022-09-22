#include "conflict_directed_graph.h"
#include <cstdlib>
#include <time.h>
#include <iostream>
#include <cmath>
#include <queue>

namespace intersection_management {

    ConflictDirectedGraph::ConflictDirectedGraph() {
        p_root_ = std::shared_ptr<Node>(new Node(0, 0.0, 0.0, 0.0, 0.0));
        nodes_.clear();
        nodes_.push_back(p_root_);
        edges_.clear();
        count_node_ = 1;
    }

    void ConflictDirectedGraph::AddNode(double weight) {
        auto node = std::shared_ptr<Node>(new Node(count_node_++, weight, -1, -1, -1));
        nodes_.push_back(node);
    }

    void ConflictDirectedGraph::AddEdge(int from, int to, double weight, bool bidirectional) {
        if (from < 0 || from >= count_node_ || to < 0 || to >= count_node_)
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
            auto edge = std::shared_ptr<Edge>(new Edge(nodes_[from], nodes_[to], weight));
            nodes_[from]->edges_.push_back(edge);
            edges_.push_back(edge);
        }
        if (bidirectional && from != 0) {
            auto edge = std::shared_ptr<Edge>(new Edge(nodes_[to], nodes_[from], weight));
            nodes_[to]->edges_.push_back(edge);
            edges_.push_back(edge);
        }
    }

    void ConflictDirectedGraph::GenerateRandomGraph(
        int total_nodes,
        unsigned int seed,
        double max_node_weight,
        double max_edge_weight,
        bool int_weight_only) {

        srand(seed);
        this->reset();
        double node_weight;
        for (int id = 1; id <= total_nodes; id++) {
            node_weight = ((double)rand()) / RAND_MAX * max_node_weight + 1.0;
            if (int_weight_only) {
                node_weight = std::floor(node_weight);
            }
            AddNode(node_weight);
        }

        int num_edges_to_add = (int)(count_node_ * count_node_ * (rand() % 100) * 0.01);
        int from, to;
        double edge_weight;
        while (num_edges_to_add > 0) {
            edge_weight = ((double)rand()) / RAND_MAX * max_edge_weight + 1.0;
            if (int_weight_only) {
                edge_weight = std::floor(edge_weight);
            }
            if (rand() % 2) { // add bidirectional edge
                do {
                    from = rand() % count_node_;
                    to = rand() % count_node_;
                } while (from == to);
                AddEdge(from, to, edge_weight, true);
                num_edges_to_add -= 2;
            }
            else { // add unidirectional edge
                do {
                    from = rand() % count_node_;
                    to = rand() % count_node_;
                } while (from >= to);
                AddEdge(from, to, edge_weight, false);
                num_edges_to_add--;
            }
        }
    }

    bool ConflictDirectedGraph::isFullyConnected() {
        std::queue<int> visit_queue;
        std::vector<bool> is_visited(count_node_, false);
        visit_queue.push(0);
        is_visited[0] = true;
        int from, to;
        while (!visit_queue.empty()) {
            from = visit_queue.front();
            visit_queue.pop();
            for (auto p_edge : nodes_[from]->edges_) {
                to = p_edge->to_.lock()->id_;
                if (!is_visited[to]) {
                    visit_queue.push(to);
                    is_visited[to] = true;
                }
            }
        }
        for (int id = 0; id < count_node_; id++) {
            if (!is_visited[id]) {
                return false;
            }
        }
        return true;
    }

    void ConflictDirectedGraph::PrintGraph() {
        std::cout << "***********Begin of CDG details***********\n" \
            << "Total nodes: " << count_node_ << std::endl;
        for (int i = 0; i < nodes_.size(); i++) {
            nodes_[i]->printDetail();
        }
        std::cout << "***********End of CDG details*************\n";
    }

    void ConflictDirectedGraph::reset() {
        p_root_ = std::shared_ptr<Node>(new Node(0, 0.0, 0.0, 0.0, 0.0));
        nodes_.clear();
        nodes_.push_back(p_root_);
        edges_.clear();
        count_node_ = 1;
        std::cout << "The CDG is reset to a new root-only graph!\n";
    }
}