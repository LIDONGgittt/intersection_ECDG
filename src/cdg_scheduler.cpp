#include "cdg_scheduler.h"

#include <iostream>

#include "parameters.h"

namespace intersection_management {
CDGScheduler::CDGScheduler() {}

CDGConflictSpanningTree CDGScheduler::ScheduleWithModifiedDfst(const ConflictDirectedGraph &cdg) {
    PrepareForTreeSchedule(cdg);

    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    added_to_tree[0] = true;

    std::vector<std::shared_ptr<Node>> unidirectional_scheduled_parent;
    std::vector<std::shared_ptr<Node>> bidirectional_scheduled_parent;
    int id_possible_parent;
    int possible_depth;
    std::shared_ptr<Edge> edge_from_possible_parent;

    for (int id = 1; id < result_tree_.num_nodes_; id++) {
        unidirectional_scheduled_parent.clear();
        bidirectional_scheduled_parent.clear();
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
                    bidirectional_scheduled_parent.push_back(result_tree_.nodes_[from]);
                }
                else {
                    unidirectional_scheduled_parent.push_back(result_tree_.nodes_[from]);
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
            id_possible_parent = bidirectional_scheduled_parent.front()->id_;
            edge_from_possible_parent = cdg.nodes_[id_possible_parent]->getEdgeTo(id);
            possible_depth = bidirectional_scheduled_parent.front()->edge_weighted_depth_ + edge_from_possible_parent->edge_weight_;
            for (auto parent : bidirectional_scheduled_parent) {
                auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
                if (parent->edge_weighted_depth_ + edge->edge_weight_ < possible_depth) {
                    id_possible_parent = parent->id_;
                    possible_depth = parent->edge_weighted_depth_ + edge->edge_weight_;
                    edge_from_possible_parent = edge;
                }
            }
        }

        // update conflicts with other bidirectional edges
        bool flag_still_conflict_with_bidire_scheduled_neighbor;
        do {
            flag_still_conflict_with_bidire_scheduled_neighbor = false;
            for (auto parent : bidirectional_scheduled_parent) {
                auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
                if (parent->edge_weighted_depth_ - edge->edge_weight_ < possible_depth && \
                    possible_depth < parent->edge_weighted_depth_ + edge->edge_weight_) {
                    id_possible_parent = parent->id_;
                    possible_depth = parent->edge_weighted_depth_ + edge->edge_weight_;
                    edge_from_possible_parent = edge;
                    flag_still_conflict_with_bidire_scheduled_neighbor = true;
                }
            }
        } while (flag_still_conflict_with_bidire_scheduled_neighbor);

        added_to_tree[id] = true;
        result_tree_.AddEdge(id_possible_parent, id, edge_from_possible_parent->edge_weight_);
        result_tree_.UpdateDepth(id, possible_depth, Type_EdgeWeightedDepth);
    }

    return result_tree_;
}

CDGConflictSpanningTree CDGScheduler::ScheduleWithBfstWeightedEdgeOnly(const ConflictDirectedGraph &cdg) {
    PrepareForTreeSchedule(cdg);

    std::vector<CDGCandidate> ready_list;
    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    CDGCandidate initial_root(0, 0, -1, -1);
    ready_list.push_back(initial_root);

    while (!ready_list.empty()) {
        CDGCandidate chosen_candidate = ready_list[0];
        added_to_tree[chosen_candidate.id_] = true;
        result_tree_.UpdateDepth(chosen_candidate.id_, chosen_candidate.possible_depth_, Type_EdgeWeightedDepth);
        if (chosen_candidate.id_ > 0) {
            result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.edge_weight_);
        }
        ready_list.erase(ready_list.begin());

        auto iter_ready_candidate = ready_list.begin();
        while (iter_ready_candidate != ready_list.end()) {
            if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(iter_ready_candidate->id_)) {
                if (chosen_candidate.possible_depth_ + cdg.nodes_[chosen_candidate.id_]->getEdgeTo(iter_ready_candidate->id_)->edge_weight_ > \
                    iter_ready_candidate->possible_depth_) {
                    iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                    continue;
                }
            }
            iter_ready_candidate++;
        }

        for (int to = 1; to < result_tree_.num_nodes_; to++) {
            if (added_to_tree[to] || isInList(to, ready_list) || !cdg.nodes_[chosen_candidate.id_]->isConnectedTo(to)) {
                continue;
            }
            if (StillHasUnscheduledPredecessor(unidirectional_parent_table_[to], added_to_tree)) {
                continue;
            }

            double edge_weight = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(to)->edge_weight_;
            CDGCandidate new_candidate(to, chosen_candidate.possible_depth_ + edge_weight, chosen_candidate.id_, edge_weight);

            // update new_candidate and solve conflict with already scheduled nodes (both uni- and bi-directional)
            for (auto parent : unidirectional_parent_table_[to]) {
                edge_weight = parent->getEdgeTo(to)->edge_weight_;
                if (result_tree_.nodes_[parent->id_]->edge_weighted_depth_ + edge_weight > new_candidate.possible_depth_) {
                    new_candidate.possible_depth_ = result_tree_.nodes_[parent->id_]->edge_weighted_depth_ + edge_weight;
                    new_candidate.id_possible_parent_ = parent->id_;
                    new_candidate.edge_weight_ = edge_weight;
                }
            }
            bool flag_still_conflict_with_bidire_scheduled_neighbor;
            do {
                flag_still_conflict_with_bidire_scheduled_neighbor = false;
                for (auto neighbor : bidirectional_neighbor_table_[to]) {
                    if (!added_to_tree[neighbor->id_]) {
                        continue;
                    }
                    edge_weight = neighbor->getEdgeTo(to)->edge_weight_;
                    if (new_candidate.possible_depth_ > result_tree_.nodes_[neighbor->id_]->edge_weighted_depth_ - edge_weight &&
                        new_candidate.possible_depth_ < result_tree_.nodes_[neighbor->id_]->edge_weighted_depth_ + edge_weight) {
                        flag_still_conflict_with_bidire_scheduled_neighbor = true;
                        new_candidate.possible_depth_ = result_tree_.nodes_[neighbor->id_]->edge_weighted_depth_ + edge_weight;
                        new_candidate.id_possible_parent_ = neighbor->id_;
                        new_candidate.edge_weight_ = edge_weight;
                    }
                }
            } while (flag_still_conflict_with_bidire_scheduled_neighbor);
            ready_list.push_back(new_candidate);

        }
        SortReadyListAscendingly(ready_list);
    }

    return result_tree_;
}

CDGConflictSpanningTree CDGScheduler::ScheduleWithBfstMultiWeight(const ConflictDirectedGraph &cdg) {
    PrepareForTreeSchedule(cdg);

    std::vector<CDGCandidate> ready_list;
    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    CDGCandidate initial_root(0, 0, -1, -1, -1);
    ready_list.push_back(initial_root);

    while (!ready_list.empty()) {
        CDGCandidate chosen_candidate = ready_list[0];
        added_to_tree[chosen_candidate.id_] = true;
        result_tree_.UpdateDepth(chosen_candidate.id_, chosen_candidate.possible_depth_, Type_EdgeNodeWeightedDepth);
        if (chosen_candidate.id_ > 0) {
            result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.edge_weight_);
        }
        ready_list.erase(ready_list.begin());

        auto iter_ready_candidate = ready_list.begin();
        while (iter_ready_candidate != ready_list.end()) {
            if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(iter_ready_candidate->id_)) {
                auto the_edge = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(iter_ready_candidate->id_);
                double edge_weight = the_edge->edge_weight_;
                double edge_offset = the_edge->estimate_offset_;
                // non-conflict edges don't delay time windows
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (param.activate_precedent_offset && edge_offset < 0) {
                    edge_weight = edge_offset;
                }
                if (chosen_candidate.possible_depth_ + edge_weight + cdg.nodes_[iter_ready_candidate->id_]->estimate_travel_time_ > iter_ready_candidate->possible_depth_) {
                    iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                    continue;
                }
            }
            iter_ready_candidate++;
        }

        for (int to = 1; to < result_tree_.num_nodes_; to++) {
            if (added_to_tree[to] || isInList(to, ready_list) || !cdg.nodes_[chosen_candidate.id_]->isConnectedTo(to)) {
                continue;
            }
            if (StillHasUnscheduledPredecessor(unidirectional_parent_table_[to], added_to_tree)) {
                continue;
            }
            auto the_edge = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(to);
            double edge_weight = the_edge->edge_weight_;
            double edge_offset = the_edge->estimate_offset_;
            // non-conflict edges don't delay time windows
            if (edge_weight <= 1.0) {
                edge_weight = 0.0;
            }
            if (param.activate_precedent_offset && edge_offset < 0) {
                edge_weight = edge_offset;
            }
            double estimate_travel_time = cdg.nodes_[to]->estimate_travel_time_;
            CDGCandidate new_candidate(to, chosen_candidate.possible_depth_ + edge_weight + estimate_travel_time, chosen_candidate.id_, edge_weight, estimate_travel_time);

            // update new_candidate and solve conflict with already scheduled nodes
            for (auto parent : unidirectional_parent_table_[to]) {
                edge_weight = parent->getEdgeTo(to)->edge_weight_;
                edge_offset = parent->getEdgeTo(to)->estimate_offset_;
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (param.activate_precedent_offset && edge_offset < 0) {
                    edge_weight = edge_offset;
                }
                if (result_tree_.nodes_[parent->id_]->edge_node_weighted_depth_ + edge_weight + new_candidate.estimate_travel_time_ > new_candidate.possible_depth_) {
                    new_candidate.possible_depth_ = result_tree_.nodes_[parent->id_]->edge_node_weighted_depth_ + edge_weight + new_candidate.estimate_travel_time_;
                    new_candidate.id_possible_parent_ = parent->id_;
                    new_candidate.edge_weight_ = edge_weight;
                }
            }
            bool flag_still_conflict_with_bidire_scheduled_neighbor;
            do {
                flag_still_conflict_with_bidire_scheduled_neighbor = false;
                for (auto neighbor : bidirectional_neighbor_table_[to]) {
                    if (!added_to_tree[neighbor->id_]) {
                        continue;
                    }
                    edge_weight = neighbor->getEdgeTo(to)->edge_weight_;
                    edge_offset = neighbor->getEdgeTo(to)->estimate_offset_;
                    if (edge_weight <= 1.0) {
                        edge_weight = 0.0;
                    }
                    if (param.activate_precedent_offset && edge_offset < 0) {
                        edge_weight = edge_offset;
                    }
                    if (new_candidate.possible_depth_ > result_tree_.nodes_[neighbor->id_]->edge_node_weighted_depth_ - neighbor->estimate_travel_time_ - edge_weight &&
                        new_candidate.possible_depth_ - new_candidate.estimate_travel_time_ < result_tree_.nodes_[neighbor->id_]->edge_node_weighted_depth_ + edge_weight) {
                        flag_still_conflict_with_bidire_scheduled_neighbor = true;
                        new_candidate.possible_depth_ = result_tree_.nodes_[neighbor->id_]->edge_node_weighted_depth_ + edge_weight + new_candidate.estimate_travel_time_;
                        new_candidate.id_possible_parent_ = neighbor->id_;
                        new_candidate.edge_weight_ = edge_weight;
                    }
                }
            } while (flag_still_conflict_with_bidire_scheduled_neighbor);
            ready_list.push_back(new_candidate);

        }
        SortReadyListAscendingly(ready_list);
    }

    return result_tree_;
}

CDGConflictSpanningTree CDGScheduler::ScheduleWithDfstMultiWeight(const ConflictDirectedGraph &cdg) {
    PrepareForTreeSchedule(cdg);

    std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    added_to_tree[0] = true;

    std::vector<std::shared_ptr<Node>> unidirectional_scheduled_parent;
    std::vector<std::shared_ptr<Node>> bidirectional_scheduled_parent;
    int id_possible_parent;
    int possible_depth;
    std::shared_ptr<Edge> edge_from_possible_parent;

    for (int id = 1; id < result_tree_.num_nodes_; id++) {
        unidirectional_scheduled_parent.clear();
        bidirectional_scheduled_parent.clear();
        id_possible_parent = -1;
        possible_depth = -1;
        edge_from_possible_parent = nullptr;
        auto current_estimate_travel_time = result_tree_.nodes_[id]->estimate_travel_time_;
        for (int from = 0; from < result_tree_.num_nodes_; from++) {
            if (!added_to_tree[from]) {
                continue;
            }
            if (cdg.nodes_[from]->isConnectedTo(id)) {
                auto edge = cdg.nodes_[from]->getEdgeTo(id);
                auto edge_weight = edge->edge_weight_;
                auto edge_offset = edge->estimate_offset_;
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (param.activate_precedent_offset && edge_offset < 0) {
                    edge_weight = edge_offset;
                }
                if (edge->bidirectional_) {
                    bidirectional_scheduled_parent.push_back(result_tree_.nodes_[from]);
                }
                else {
                    unidirectional_scheduled_parent.push_back(result_tree_.nodes_[from]);
                    if (result_tree_.nodes_[from]->edge_node_weighted_depth_ + edge_weight + current_estimate_travel_time > possible_depth) {
                        possible_depth = result_tree_.nodes_[from]->edge_node_weighted_depth_ + edge_weight + current_estimate_travel_time;
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
            id_possible_parent = bidirectional_scheduled_parent.front()->id_;
            edge_from_possible_parent = cdg.nodes_[id_possible_parent]->getEdgeTo(id);
            auto edge_weight = edge_from_possible_parent->edge_weight_;
            auto edge_offset = edge_from_possible_parent->estimate_offset_;
            if (edge_weight <= 1.0) {
                edge_weight = 0.0;
            }
            if (param.activate_precedent_offset && edge_offset < 0) {
                edge_weight = edge_offset;
            }
            possible_depth = bidirectional_scheduled_parent.front()->edge_node_weighted_depth_ + edge_weight + current_estimate_travel_time;
            for (auto parent : bidirectional_scheduled_parent) {
                auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
                auto edge_weight = edge->edge_weight_;
                auto edge_offset = edge->estimate_offset_;
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (param.activate_precedent_offset && edge_offset < 0) {
                    edge_weight = edge_offset;
                }
                if (parent->edge_node_weighted_depth_ + edge_weight + current_estimate_travel_time < possible_depth) {
                    id_possible_parent = parent->id_;
                    possible_depth = parent->edge_node_weighted_depth_ + edge_weight + current_estimate_travel_time;
                    edge_from_possible_parent = edge;
                }
            }
        }

        // update conflicts with other bidirectional edges
        bool flag_still_conflict_with_bidire_scheduled_neighbor;
        do {
            flag_still_conflict_with_bidire_scheduled_neighbor = false;
            for (auto parent : bidirectional_scheduled_parent) {
                auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
                auto edge_weight = edge->edge_weight_;
                auto edge_offset = edge->estimate_offset_;
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (param.activate_precedent_offset && edge_offset < 0) {
                    edge_weight = edge_offset;
                }
                if (parent->edge_node_weighted_depth_ - edge_weight - parent->estimate_travel_time_ < possible_depth && \
                    possible_depth < parent->edge_node_weighted_depth_ + edge_weight + current_estimate_travel_time) {
                    id_possible_parent = parent->id_;
                    possible_depth = parent->edge_node_weighted_depth_ + edge_weight + current_estimate_travel_time;
                    edge_from_possible_parent = edge;
                    flag_still_conflict_with_bidire_scheduled_neighbor = true;
                }
            }
        } while (flag_still_conflict_with_bidire_scheduled_neighbor);

        added_to_tree[id] = true;
        result_tree_.AddEdge(id_possible_parent, id, edge_from_possible_parent->edge_weight_);
        result_tree_.UpdateDepth(id, possible_depth, Type_EdgeNodeWeightedDepth);
    }

    return result_tree_;
}

std::vector<int> CDGScheduler::ScheduleBruteForceSearch(const ConflictDirectedGraph &cdg) {
    int num_nodes = cdg.num_nodes_;
    double minimum_evacuation_time = -1.0;
    std::vector<int> vehicle_order;
    std::vector<bool> is_in_order_list(num_nodes, false);
    vehicle_order.push_back(0);
    is_in_order_list[0] = true;
    std::vector<int> best_order;

    GenerateUniparentTable(cdg);
    GenerateBineighborTable(cdg);
    SearchOrderPermutationRecursively(vehicle_order, num_nodes, is_in_order_list, minimum_evacuation_time, best_order, cdg);
    return best_order;
}

void CDGScheduler::PrepareForTreeSchedule(const ConflictDirectedGraph &cdg) {
    result_tree_.reset(false);
    result_tree_.AddNodesFromGraph(cdg);
    GenerateUniparentTable(cdg);
    GenerateBineighborTable(cdg);
}

void CDGScheduler::GenerateUniparentTable(const ConflictDirectedGraph &cdg) {
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

void CDGScheduler::GenerateBineighborTable(const ConflictDirectedGraph &cdg) {
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

void CDGScheduler::SearchOrderPermutationRecursively(std::vector<int> &vehicle_order, int num_nodes,
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
        SearchOrderPermutationRecursively(vehicle_order, num_nodes, is_in_order_list, minimum_evacuation_time, best_order, cdg);
        is_in_order_list[i] = false;
        vehicle_order.pop_back();
    }
}

double CDGScheduler::GetEvacuationTimeFromOrder(const std::vector<int> &vehicle_order,
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

std::vector<double> CDGScheduler::GetDepthVectorFromOrder(const std::vector<int> &vehicle_order,
                                                          const ConflictDirectedGraph &cdg) {
    std::vector<bool> vehicle_scheduled(vehicle_order.size(), false);
    std::vector<double> depth_of_the_order(vehicle_order.size(), -1.0);
    double cur_estimate_travel_time;
    double edge_weight;
    double edge_offset;
    double possible_start_time;
    double possible_end_time;

    for (int cur_id : vehicle_order) {
        cur_estimate_travel_time = cdg.nodes_[cur_id]->estimate_travel_time_;
        possible_start_time = 0;
        for (auto parent : unidirectional_parent_table_[cur_id]) {
            if (!vehicle_scheduled[parent->id_]) {
                depth_of_the_order.clear();
                return depth_of_the_order;
            }
            edge_weight = parent->getEdgeTo(cur_id)->edge_weight_;
            edge_offset = parent->getEdgeTo(cur_id)->estimate_offset_;
            if (edge_weight <= 1.0) {
                edge_weight = 0.0;
            }
            if (param.activate_precedent_offset && edge_offset < 0) {
                edge_weight = edge_offset;
            }
            if (depth_of_the_order[parent->id_] + edge_weight > possible_start_time) {
                possible_start_time = depth_of_the_order[parent->id_] + edge_weight;
            }
        }
        possible_end_time = possible_start_time + cur_estimate_travel_time;
        bool flag;
        do {
            flag = false;
            for (auto neighbor : bidirectional_neighbor_table_[cur_id]) {
                if (!vehicle_scheduled[neighbor->id_]) {
                    continue;
                }
                edge_weight = neighbor->getEdgeTo(cur_id)->edge_weight_;
                edge_offset = neighbor->getEdgeTo(cur_id)->estimate_offset_;
                if (edge_weight <= 1.0) {
                    edge_weight = 0.0;
                }
                if (param.activate_precedent_offset && edge_offset < 0) {
                    edge_weight = edge_offset;
                }
                if (possible_end_time > depth_of_the_order[neighbor->id_] - neighbor->estimate_travel_time_ - edge_weight &&
                    possible_start_time < depth_of_the_order[neighbor->id_] + edge_weight) {
                    flag = true;
                    possible_start_time = depth_of_the_order[neighbor->id_] + edge_weight;
                    possible_end_time = possible_start_time + cur_estimate_travel_time;
                }
            }
        } while (flag);
        depth_of_the_order[cur_id] = possible_end_time;
        vehicle_scheduled[cur_id] = true;
    }
    return depth_of_the_order;
}

} // namespace intersection_management