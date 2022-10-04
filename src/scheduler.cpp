#include "scheduler.h"

#include <iostream>
#include <algorithm>

namespace intersection_management {
Scheduler::Scheduler() {}

ConflictSpanningTree Scheduler::ScheduleWithModifiedDfst(const ConflictDirectedGraph &cdg) {
    result_tree_.reset(false);
    result_tree_.AddNodesFromGraph(cdg);
    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    added_to_tree[0] = true;


    std::vector<std::shared_ptr<Node>> unidirectional_parent;
    std::vector<std::shared_ptr<Node>> bidirectional_parent;
    int id_possible_parent;
    int possible_depth;
    std::shared_ptr<Edge> edge_from_possible_parent;

    for (int id = 1; id < result_tree_.num_nodes_; id++) {
        unidirectional_parent.clear();
        bidirectional_parent.clear();
        id_possible_parent = -1;
        possible_depth = -1;
        edge_from_possible_parent = nullptr;
        for (int from = 0; from < result_tree_.num_nodes_; from++) {
            if (!added_to_tree[from]) {
                continue;
            }
            if (cdg.nodes_[from]->isConnectedTo(id)) {
                auto edge = cdg.nodes_[from]->getEdgeTo(id);
                if (edge->bidirectional_) {
                    bidirectional_parent.push_back(result_tree_.nodes_[from]);
                }
                else {
                    unidirectional_parent.push_back(result_tree_.nodes_[from]);
                    if (edge->edge_weight_ + result_tree_.nodes_[from]->edge_weighted_depth_ > possible_depth) {
                        possible_depth = edge->edge_weight_ + result_tree_.nodes_[from]->edge_weighted_depth_;
                        id_possible_parent = from;
                        edge_from_possible_parent = edge;
                    }
                }
            }
        }

        // if only connected by bidirectional edges, initiate possible depth with the smallest one
        if (id_possible_parent == -1) {
            int min_depth_parent_id;
            int min_depth;
            id_possible_parent = bidirectional_parent.front()->id_;
            edge_from_possible_parent = cdg.nodes_[id_possible_parent]->getEdgeTo(id);
            possible_depth = bidirectional_parent.front()->edge_weighted_depth_ + edge_from_possible_parent->edge_weight_;
            for (auto parent : bidirectional_parent) {
                auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
                if (parent->edge_weighted_depth_ + edge->edge_weight_ < possible_depth) {
                    id_possible_parent = parent->id_;
                    possible_depth = parent->edge_weighted_depth_ + edge->edge_weight_;
                    edge_from_possible_parent = edge;
                }
            }
        }

        // update conflicts with other bidirectional edges
        bool flag;
        do {
            flag = false;
            for (auto parent : bidirectional_parent) {
                auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
                if (parent->edge_weighted_depth_ - edge->edge_weight_ < possible_depth && \
                    possible_depth < parent->edge_weighted_depth_ + edge->edge_weight_) {
                    id_possible_parent = parent->id_;
                    possible_depth = parent->edge_weighted_depth_ + edge->edge_weight_;
                    edge_from_possible_parent = edge;
                    flag = true;
                }
            }
        } while (flag);

        added_to_tree[id] = true;
        result_tree_.AddEdge(id_possible_parent, id, edge_from_possible_parent->edge_weight_, false);
        result_tree_.UpdateEdgeWeightedDepth(id, possible_depth);
    }

    return result_tree_;
}

void Scheduler::GenerateUniparentTable(const ConflictDirectedGraph &cdg) {
    unidirectional_parent_table_.clear();
    for (int id = 0; id < cdg.num_nodes_; id++) {
        std::vector<std::shared_ptr<Node>> uni_parent;
        for (int from = 0; from < cdg.num_nodes_; from++) {
            if (cdg.nodes_[from]->isConnectedTo(id)) {
                auto edge = cdg.nodes_[from]->getEdgeTo(id);
                if (!edge->bidirectional_) {
                    uni_parent.push_back(cdg.nodes_[from]);
                }
            }
        }
        unidirectional_parent_table_.push_back(uni_parent);
    }
}


void Scheduler::GenerateBineighborTable(const ConflictDirectedGraph &cdg) {
    bidirectional_neighbor_table_.clear();
    for (int id = 0; id < cdg.num_nodes_; id++) {
        std::vector<std::shared_ptr<Node>> neighbors;
        for (int from = 0; from < cdg.num_nodes_; from++) {
            if (cdg.nodes_[from]->isConnectedTo(id)) {
                auto edge = cdg.nodes_[from]->getEdgeTo(id);
                if (edge->bidirectional_) {
                    neighbors.push_back(cdg.nodes_[from]);
                }
            }
        }
        bidirectional_neighbor_table_.push_back(neighbors);
    }
}

ConflictSpanningTree Scheduler::ScheduleWithBfstWeightedEdgeOnly(const ConflictDirectedGraph &cdg) {
    result_tree_.reset(false);
    result_tree_.AddNodesFromGraph(cdg);
    std::vector<Candidate> ready_list;
    std::vector<std::vector<std::shared_ptr<Node>>> unidirectional_parent_table;
    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    Candidate initial_root(0, 0, -1, -1);
    ready_list.push_back(initial_root);

    GenerateUniparentTable(cdg);
    unidirectional_parent_table = this->unidirectional_parent_table_;

    while (!ready_list.empty()) {
        Candidate chosen_candidate = ready_list[0];
        added_to_tree[chosen_candidate.id_] = true;
        result_tree_.UpdateEdgeWeightedDepth(chosen_candidate.id_, chosen_candidate.possible_depth_);
        if (chosen_candidate.id_ > 0) {
            result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.edge_weight_, false);
        }
        ready_list.erase(ready_list.begin());

        auto iter = ready_list.begin();
        while (iter != ready_list.end()) {
            if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(iter->id_)) {
                if (chosen_candidate.possible_depth_ + cdg.nodes_[chosen_candidate.id_]->getEdgeTo(iter->id_)->edge_weight_ > \
                    iter->possible_depth_) {
                    iter = ready_list.erase(iter);
                    continue;
                }
            }
            iter++;
        }

        for (int to = 1; to < result_tree_.num_nodes_; to++) {
            if (added_to_tree[to] || isInList(to, ready_list)) {
                continue;
            }
            if (StillHasPredecessor(unidirectional_parent_table[to], added_to_tree)) {
                continue;
            }
            if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(to)) {
                double edge_weight = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(to)->edge_weight_;
                Candidate new_candidate(to, chosen_candidate.possible_depth_ + edge_weight, chosen_candidate.id_, edge_weight);
                ready_list.push_back(new_candidate);
            }
        }
        std::sort(ready_list.begin(), ready_list.end(),
                  [](const Candidate &a, const Candidate &b) {
                      if (a.possible_depth_ == b.possible_depth_) {
                          return a.id_ < b.id_;
                      }
                      return a.possible_depth_ < b.possible_depth_;
                  });
    }

    return result_tree_;
}

bool Scheduler::isInList(int id, std::vector<Candidate> &ready_list) {
    for (auto &item : ready_list) {
        if (id == item.id_) {
            return true;
        }
    }
    return false;
}

bool Scheduler::StillHasPredecessor(std::vector<std::shared_ptr<Node>> &pre, std::vector<bool> &added_to_tree) {
    for (auto node : pre) {
        if (added_to_tree[node->id_] == false) {
            return true;
        }
    }
    return false;
}

ConflictSpanningTree Scheduler::ScheduleWithBfstMultiWeight(const ConflictDirectedGraph &cdg) {
    result_tree_.reset(false);
    result_tree_.AddNodesFromGraph(cdg);
    std::vector<Candidate> ready_list;
    std::vector<std::vector<std::shared_ptr<Node>>> unidirectional_parent_table;
    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    // root depth is set to -1 to offset the edge_weight from root to every other node
    Candidate initial_root(0, 0, -1, -1, -1);
    ready_list.push_back(initial_root);

    GenerateUniparentTable(cdg);
    unidirectional_parent_table = this->unidirectional_parent_table_;

    while (!ready_list.empty()) {
        Candidate chosen_candidate = ready_list[0];
        added_to_tree[chosen_candidate.id_] = true;
        result_tree_.UpdateEdgeNodeWeightedDepth(chosen_candidate.id_, chosen_candidate.possible_depth_);
        if (chosen_candidate.id_ > 0) {
            result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.edge_weight_, false);
        }
        ready_list.erase(ready_list.begin());

        auto iter = ready_list.begin();
        while (iter != ready_list.end()) {
            if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(iter->id_)) {
                double edge_weight = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(iter->id_)->edge_weight_;
                // non-conflict edges don't delay time windows
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (chosen_candidate.possible_depth_ + edge_weight + cdg.nodes_[iter->id_]->node_weight_ > iter->possible_depth_) {
                    iter = ready_list.erase(iter);
                    continue;
                }
            }
            iter++;
        }

        for (int to = 1; to < result_tree_.num_nodes_; to++) {
            if (added_to_tree[to] || isInList(to, ready_list)) {
                continue;
            }
            if (StillHasPredecessor(unidirectional_parent_table[to], added_to_tree)) {
                continue;
            }
            if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(to)) {
                double edge_weight = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(to)->edge_weight_;
                // non-conflict edges don't delay time windows
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                double node_weight = cdg.nodes_[to]->node_weight_;
                Candidate new_candidate(to, chosen_candidate.possible_depth_ + edge_weight + node_weight, chosen_candidate.id_, edge_weight);
                ready_list.push_back(new_candidate);
            }
        }
        std::sort(ready_list.begin(), ready_list.end(),
                  [](const Candidate &a, const Candidate &b) {
                      if (a.possible_depth_ == b.possible_depth_) {
                          return a.id_ < b.id_;
                      }
                      return a.possible_depth_ < b.possible_depth_;
                  });
    }

    return result_tree_;
}

std::vector<double> Scheduler::GetDepthVectorFromOrder(const std::vector<int> &vehicle_order,
                                                       const ConflictDirectedGraph &cdg) {
    std::vector<bool> vehicle_scheduled(vehicle_order.size(), false);
    std::vector<double> depth_of_the_order(vehicle_order.size(), -1.0);
    double cur_node_weight;
    double edge_weight;
    double possible_start_time;
    double possible_end_time;

    for (int cur_id : vehicle_order) {
        cur_node_weight = cdg.nodes_[cur_id]->node_weight_;
        possible_start_time = 0;
        for (auto parent : this->unidirectional_parent_table_[cur_id]) {
            if (!vehicle_scheduled[parent->id_]) {
                depth_of_the_order.clear();
                return depth_of_the_order;
            }
            edge_weight = parent->getEdgeTo(cur_id)->edge_weight_;
            if (edge_weight <= 1.0) {
                edge_weight = 0.0;
            }
            if (parent->edge_node_weighted_depth_ + edge_weight > possible_start_time) {
                possible_start_time = parent->edge_node_weighted_depth_ + edge_weight;
            }
        }
        possible_end_time = possible_start_time + cur_node_weight;
        bool flag;
        do {
            flag = false;
            for (auto neighbor : this->bidirectional_neighbor_table_[cur_id]) {
                if (!vehicle_scheduled[neighbor->id_]) {
                    continue;
                }
                edge_weight = neighbor->getEdgeTo(cur_id)->edge_weight_;
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (possible_end_time > neighbor->edge_node_weighted_depth_ - neighbor->node_weight_ - edge_weight &&
                    possible_start_time < neighbor->edge_node_weighted_depth_ + edge_weight) {
                    flag = true;
                    possible_start_time = neighbor->edge_node_weighted_depth_ + edge_weight;
                    possible_end_time = possible_start_time + cur_node_weight;
                }

            }
        } while (flag);
        depth_of_the_order[cur_id] = possible_end_time;
        vehicle_scheduled[cur_id] = true;
    }
    return depth_of_the_order;
}

void Scheduler::printDepthVector(std::vector<double> &depth_vector) {
    int tmp_cnt = 0;
    for (int i = 0; i < depth_vector.size(); i++) {
        std::cout << "Node " << i << ": " << depth_vector[i] << ",";
        if (++tmp_cnt % 6 == 0)
            std::cout << std::endl;
    }
    std::cout << std::endl;
}
void Scheduler::printOrder(std::vector<int> &order) {
    int tmp_cnt = 0;
    for (auto id: order) {
        std::cout << "Node " << id << " -> ";
        if (++tmp_cnt % 10 == 0)
            std::cout << std::endl;
    }
    std::cout << std::endl;
}

double Scheduler::GetEvacuationTimeFromOrder(const std::vector<int> &vehicle_order,
                                             const ConflictDirectedGraph &cdg) {
    std::vector<double> depth_of_the_order = GetDepthVectorFromOrder(vehicle_order, cdg);
    if (depth_of_the_order.empty()) {
        return -1.0;
    }

    double evacuation_time = -1.0;
    for (auto depth : depth_of_the_order) {
        if (depth > evacuation_time) {
            evacuation_time = depth;
        }
    }
    return evacuation_time;
}

void Scheduler::SearchRecursively(std::vector<int> &vehicle_order, int num_nodes,
                                  std::vector<bool> &is_in_order_list,
                                  double &minimum_evacuation_time, std::vector<int> &best_order,
                                  const ConflictDirectedGraph &cdg) {
    if (vehicle_order.size() >= num_nodes) {
        double evacuation_time;
        evacuation_time = GetEvacuationTimeFromOrder(vehicle_order, cdg);
        if (evacuation_time > 0) {
            if (minimum_evacuation_time < 0 || evacuation_time < minimum_evacuation_time) {
                minimum_evacuation_time = evacuation_time;
                best_order.clear();
                best_order.insert(best_order.end(), vehicle_order.begin(), vehicle_order.end());
            }
        }
        return;
    }

    for (int i = 1; i < num_nodes; i++) {
        if (is_in_order_list[i]) {
            continue;
        }
        vehicle_order.push_back(i);
        is_in_order_list[i] = true;
        SearchRecursively(vehicle_order, num_nodes, is_in_order_list, minimum_evacuation_time, best_order, cdg);
        is_in_order_list[i] = false;
        vehicle_order.pop_back();
    }
}

std::vector<int> Scheduler::ScheduleBruteForceSearch(const ConflictDirectedGraph &cdg) {
    int num_nodes = cdg.num_nodes_;
    double minimum_evacuation_time = -1.0;
    std::vector<int> vehicle_order;
    std::vector<bool> is_in_order_list(num_nodes, false);
    vehicle_order.push_back(0);
    is_in_order_list[0] = true;
    std::vector<int> best_order;

    GenerateUniparentTable(cdg);
    GenerateBineighborTable(cdg);
    SearchRecursively(vehicle_order, num_nodes, is_in_order_list, minimum_evacuation_time, best_order, cdg);
    return best_order;
}

} // namespace intersection_management