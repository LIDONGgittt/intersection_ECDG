#ifndef intersection_management_conflict_directed_graph_h
#define intersection_management_conflict_directed_graph_h

#include <cstdlib>
#include <time.h>
#include "graph_utility.h"

namespace intersection_management {
    class ConflictDirectedGraph {
    public:
        ConflictDirectedGraph() {
            root_ = std::shared_ptr<Node>(new Node(0, 0.0, 0.0, 0.0, 0.0));
            nodes_.clear();
            nodes_.push_back(root_);
            edges_.clear();
            count_node_ = 1;
        }

        void AddNode(double weight = 1.0) {
            auto node = std::shared_ptr<Node>(new Node(count_node_++, weight, -1, -1, -1));
            nodes_.push_back(node);
        }

        void AddEdge(int from, int to, double weight = 1.0) {
            if (from < 0 || from >= nodes_.size() || to < 0 || to >= nodes_.size())
            {
                return;
            }
            auto edge = std::shared_ptr<Edge>(new Edge(nodes_[from], nodes_[to], weight));
            nodes_[from]->edges_.push_back(edge);
            edges_.push_back(edge);
        }

        void GenerateRandomGraph(int total_nodes, bool use_time_seed = false) {
            this->reset();
            for (int id = 1; id <= total_nodes; id++) {
                nodes_.push_back(std::shared_ptr<Node>(new Node(id)));
            }
            this->AddEdge(0,1);
            this->AddEdge(0,2);
            this->AddEdge(0,3);
            this->AddEdge(0,4);
            this->AddEdge(1,2);
            this->AddEdge(1,3);
            this->AddEdge(1,4);
            this->AddEdge(2,5);
            this->AddEdge(3,5);
            this->AddEdge(4,5);
        }

        void PrintGraph() {
            std::cout << "***********Begin of CDG details***********\n" \
                << "Total nodes: " << count_node_ << std::endl;
            for (int i = 0; i < nodes_.size(); i++) {
                nodes_[i]->print();
            }
            std::cout << "***********End of CDG details*************\n";
        }

        void reset() {
            root_ = std::shared_ptr<Node>(new Node(0, 0.0, 0.0, 0.0, 0.0));
            nodes_.clear();
            nodes_.push_back(root_);
            edges_.clear();
            count_node_ = 1;
            std::cout << "The CDG is reset to a new root-only graph!\n";
        }

        std::shared_ptr<Node> root_;
        std::vector<std::shared_ptr<Node>> nodes_;
        std::vector<std::shared_ptr<Edge>> edges_;
        int count_node_;
    };
}
#endif