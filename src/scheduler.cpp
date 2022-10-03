#include "scheduler.h"

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
ConflictSpanningTree Scheduler::ScheduleWithBfstWeightedEdgeOnly(const ConflictDirectedGraph &cdg) {
    result_tree_.reset(false);
    result_tree_.AddNodesFromGraph(cdg);
    std::vector<Candidate> ready_list;
    std::vector<std::vector<std::shared_ptr<Node>>> unidirectional_parent_table;
    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    Candidate initial_root(0, 0, -1, -1);
    ready_list.push_back(initial_root);

    for (int id = 0; id < result_tree_.num_nodes_; id++) {
        std::vector<std::shared_ptr<Node>> uni_parent;
        for (int from = 0; from < result_tree_.num_nodes_; from++) {
            if (cdg.nodes_[from]->isConnectedTo(id)) {
                auto edge = cdg.nodes_[from]->getEdgeTo(id);
                if (!edge->bidirectional_) {
                    uni_parent.push_back(result_tree_.nodes_[from]);
                }
            }
        }
        unidirectional_parent_table.push_back(uni_parent);
    }

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

    for (int id = 0; id < result_tree_.num_nodes_; id++) {
        std::vector<std::shared_ptr<Node>> uni_parent;
        for (int from = 0; from < result_tree_.num_nodes_; from++) {
            if (cdg.nodes_[from]->isConnectedTo(id)) {
                auto edge = cdg.nodes_[from]->getEdgeTo(id);
                if (!edge->bidirectional_) {
                    uni_parent.push_back(result_tree_.nodes_[from]);
                }
            }
        }
        unidirectional_parent_table.push_back(uni_parent);
    }

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
} // namespace intersection_management