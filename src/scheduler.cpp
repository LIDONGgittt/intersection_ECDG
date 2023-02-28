#include "scheduler.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>

namespace intersection_management {
Scheduler::Scheduler() {
    result_tree_.reset();
}


SpanningTree Scheduler::ScheduleWithDynamicLaneAssignment(Intersection &intersection) {
    PrepareForTreeSchedule(intersection);
    std::vector<Candidate> ready_list;
    std::vector<bool> added_to_tree(intersection.num_nodes_, false);
    std::vector<bool> lane_fixed(intersection.num_nodes_, false);
    lane_fixed[0] = true;
    std::unordered_map<int, std::vector<double>> leg_lane_last_occupation;
    std::unordered_map<int, std::vector<int>> leg_lane_last_node_id;
    for (int leg_id = 0; leg_id < intersection.num_legs_; leg_id++) {
        leg_lane_last_occupation[leg_id] = std::vector<double>(1, 0.0);
        leg_lane_last_node_id[leg_id] = std::vector<int>(1, 0);
    }
    for (auto crm : intersection.critical_resource_map_) { // overwrite default value
        leg_lane_last_occupation[crm.first] = std::vector<double>(crm.second->getNumResources(), 0.0);
        leg_lane_last_node_id[crm.first] = std::vector<int>(crm.second->getNumResources(), 0);
    }
    Candidate initial_root(0, 0, -1, 0, -1, -1, 0);
    ready_list.push_back(initial_root);

    while (!ready_list.empty()) {
        // add the node from the first candidate to the tree
        // TODO: update lanes last occupation based on the added candidate
        auto chosen_candidate = ready_list[0];
        auto &chosen_node = result_tree_.nodes_[chosen_candidate.id_];
        added_to_tree[chosen_candidate.id_] = true;
        result_tree_.UpdateTimeWindow(chosen_candidate.id_, chosen_candidate.possible_depth_, chosen_candidate.estimate_offset_, chosen_candidate.estimate_travel_time_);
        if (chosen_candidate.id_ > 0) {
            // update edge in the tree
            result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.estimate_offset_);
            // add the node with all possible lanes to the tree
            chosen_node->possible_lane_id_.clear();
            chosen_node->possible_lane_id_.push_back(chosen_candidate.out_lane_id_);
            if (!chosen_candidate.split_flexible_critical_resource_) { // assign all possible lanes
                for (int i = 1; i < ready_list.size(); i++) {
                    if (ready_list[i].possible_depth_ <= chosen_candidate.possible_depth_) {
                        if (ready_list[i].id_ == chosen_candidate.id_) {
                            chosen_node->possible_lane_id_.push_back(ready_list[i].out_lane_id_);
                        }
                    }
                    else {
                        break;
                    }
                }
            }
            else { // chosen_candidate split critical resource, then needs to update lane assignments in the tree
                std::unordered_set<int> conflict_lanes;
                auto &competing_node = result_tree_.nodes_[chosen_candidate.competing_node_id_];
                std::shared_ptr<Route> route_chosen_candidate = std::make_shared<Route>(
                    intersection.leg_map_[chosen_node->in_leg_id_]->lanes_in_map_[chosen_node->in_lane_id_],
                    intersection.leg_map_[chosen_candidate.out_leg_id_]->lanes_out_map_[chosen_candidate.out_lane_id_]);
                for (int lane_id : competing_node->possible_lane_id_) {
                    auto route_competing = std::make_shared<Route>(
                        intersection.leg_map_[competing_node->in_leg_id_]->lanes_in_map_[competing_node->in_lane_id_],
                        intersection.leg_map_[competing_node->out_leg_id_]->lanes_out_map_[lane_id]);
                    auto ct = route_chosen_candidate->FindConflictTypeWithRoute(route_competing);
                    if (!ct.isConverging() && !ct.isCrossing()) {
                        conflict_lanes.insert(lane_id);
                    }
                }
                auto iter = competing_node->possible_lane_id_.begin();
                while (iter != competing_node->possible_lane_id_.end()) {
                    if (conflict_lanes.find(*iter) != conflict_lanes.end()) {
                        iter = competing_node->possible_lane_id_.erase(iter);
                        continue;
                    }
                    iter++;
                }
            }
            for (auto out_lane_id : chosen_node->possible_lane_id_) {
                leg_lane_last_occupation[chosen_candidate.out_leg_id_][out_lane_id] = chosen_candidate.possible_depth_;
                leg_lane_last_node_id[chosen_candidate.out_leg_id_][out_lane_id] = chosen_candidate.id_;
            }
            if (chosen_node->possible_lane_id_.size() == 1) {
                lane_fixed[chosen_candidate.id_] = true;
            }
        }

        // remove chosen candidate and all disabled candidates
        ready_list.erase(ready_list.begin());
        auto iter_ready_candidate = ready_list.begin();
        std::unordered_set<int> crossing_conflict_node_set;
        while (iter_ready_candidate != ready_list.end()) {
            // candidates for the same node is disabled
            if (iter_ready_candidate->id_ == chosen_candidate.id_) {
                iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                continue;
            }
            if (intersection.nodes_[chosen_candidate.id_]->isConnectedWith(iter_ready_candidate->id_)) {
                auto edge = intersection.nodes_[chosen_candidate.id_]->getEdgeWith(iter_ready_candidate->id_);
                // competing nodes in all lanes are disabled
                if (edge->conflict_type_.isCompeting()) {
                    iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                    continue;
                }
                // crossing nodes with conflicting candidate window will be removed, 
                // all candidates for the node will be removed
                if (edge->conflict_type_.isCrossing()) {
                    double edge_weight = edge->edge_weight_;
                    if (chosen_candidate.possible_depth_ + edge_weight + intersection.nodes_[iter_ready_candidate->id_]->estimate_travel_time_ > iter_ready_candidate->possible_depth_) {
                        crossing_conflict_node_set.insert(iter_ready_candidate->id_);
                        iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                        continue;
                    }
                }
            }
            iter_ready_candidate++;
        }
        // clean up all candidates that belong to crossing and conflicting nodes
        iter_ready_candidate = ready_list.begin();
        while (iter_ready_candidate != ready_list.end()) {
            if (crossing_conflict_node_set.find(iter_ready_candidate->id_) != crossing_conflict_node_set.end()) {
                iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                continue;
            }
            iter_ready_candidate++;
        }

        // add all possible candidates
        for (int new_node_id = 1; new_node_id < result_tree_.num_nodes_; new_node_id++) {
            if (added_to_tree[new_node_id] || isInList(new_node_id, ready_list) || !intersection.nodes_[chosen_candidate.id_]->isConnectedWith(new_node_id)) {
                continue;
            }
            if (StillHasUnscheduledPredecessor(unidirectional_parent_table_[new_node_id], added_to_tree)) {
                continue;
            }

            auto &new_node = intersection.nodes_[new_node_id];
            int out_leg_id = new_node->out_leg_id_;
            double estimate_travel_time = new_node->estimate_travel_time_;
            double estimate_arrival_time = new_node->estimate_arrival_time_;

            double earliest_start_time = -1;
            int possible_precedent_parent_id = 0;
            for (auto parent : unidirectional_parent_table_[new_node_id]) {
                if (result_tree_.nodes_[parent->id_]->depth_ > earliest_start_time) {
                    earliest_start_time = result_tree_.nodes_[parent->id_]->depth_;
                    possible_precedent_parent_id = parent->id_;
                }
            }
            double precedence_offset = 0;
            if (new_node->getEdgeWith(possible_precedent_parent_id)->conflict_type_.isDiverging())
                precedence_offset = -1;

            // update new_candidate for each possible lane and solve conflicts with already scheduled nodes
            for (int out_lane_id = 0; out_lane_id < leg_lane_last_occupation[out_leg_id].size(); out_lane_id++) {
                int best_parent_id = possible_precedent_parent_id;
                double best_start_time = earliest_start_time;
                double best_estimate_offset = precedence_offset;
                double best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                std::shared_ptr<Route> route_new_candidate = std::make_shared<Route>(intersection.leg_map_[new_node->in_leg_id_]->lanes_in_map_[new_node->in_lane_id_],
                                                                                     intersection.leg_map_[out_leg_id]->lanes_out_map_[out_lane_id]);
                bool split_flexible_critical_resource = false;
                int num_critical_resource_splitted = 0;
                int competing_node_id = 0;

                if (best_start_time < leg_lane_last_occupation[out_leg_id][out_lane_id]) {
                    best_parent_id = leg_lane_last_node_id[out_leg_id][out_lane_id];
                    best_start_time = leg_lane_last_occupation[out_leg_id][out_lane_id];
                    best_estimate_offset = 0;
                    best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                }

                bool flag_still_conflict_with_bidire_scheduled_neighbor;
                do {
                    flag_still_conflict_with_bidire_scheduled_neighbor = false;
                    for (auto neighbor_in_intersection : bidirectional_neighbor_table_[new_node_id]) {
                        if (!added_to_tree[neighbor_in_intersection->id_]) {
                            continue;
                        }
                        auto &neighbor = result_tree_.nodes_[neighbor_in_intersection->id_];
                        if (!new_node->getEdgeWith(neighbor->id_)->conflict_type_.isCompeting()) { // not competing, then must crossing
                            if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1]) {
                                flag_still_conflict_with_bidire_scheduled_neighbor = true;
                                best_parent_id = neighbor->id_;
                                best_start_time = neighbor->time_window_[1];
                                best_estimate_offset = 0;
                                best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                                split_flexible_critical_resource = false;
                            }
                        }
                        else { // competing, need to dynamicly assign lanes
                            if (lane_fixed[neighbor->id_]) {
                                std::shared_ptr<Route> route_neighbor = std::make_shared<Route>(
                                    intersection.leg_map_[neighbor->in_leg_id_]->lanes_in_map_[neighbor->in_lane_id_],
                                    intersection.leg_map_[neighbor->out_leg_id_]->lanes_out_map_[neighbor->possible_lane_id_[0]]);
                                auto ct = route_new_candidate->FindConflictTypeWithRoute(route_neighbor);
                                if (ct.isConverging() || ct.isCrossing()) { // only resolve conflict when converging or crossing
                                    if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1]) {
                                        flag_still_conflict_with_bidire_scheduled_neighbor = true;
                                        best_parent_id = neighbor->id_;
                                        best_start_time = neighbor->time_window_[1];
                                        best_estimate_offset = 0;
                                        best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                                        split_flexible_critical_resource = false;
                                    }
                                }
                            }
                            else { // neighbor's lane is not fixed
                                int non_conflict_count = 0;
                                for (int neighbor_lane_id : neighbor->possible_lane_id_) {
                                    auto route_neighbor = std::make_shared<Route>(
                                        intersection.leg_map_[neighbor->in_leg_id_]->lanes_in_map_[neighbor->in_lane_id_],
                                        intersection.leg_map_[neighbor->out_leg_id_]->lanes_out_map_[neighbor_lane_id]);
                                    auto ct = route_new_candidate->FindConflictTypeWithRoute(route_neighbor);
                                    if (!ct.isConverging() && !ct.isCrossing()) {
                                        non_conflict_count++;
                                    }
                                }
                                if (non_conflict_count < neighbor->possible_lane_id_.size()) { // otherwise will complete non-conflict
                                    if (non_conflict_count > 0) { // may split critical resources
                                        if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1]) {
                                            split_flexible_critical_resource = true;
                                            num_critical_resource_splitted = neighbor->possible_lane_id_.size() - non_conflict_count;
                                            competing_node_id = neighbor->id_;
                                        }
                                    }
                                    else { // conflict with all possible lanes, thus will not split critical resources
                                        if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1]) {
                                            flag_still_conflict_with_bidire_scheduled_neighbor = true;
                                            best_parent_id = neighbor->id_;
                                            best_start_time = neighbor->time_window_[1];
                                            best_estimate_offset = 0;
                                            best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                                            split_flexible_critical_resource = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                } while (flag_still_conflict_with_bidire_scheduled_neighbor);

                Candidate new_candidate(new_node_id, best_depth, best_parent_id, best_estimate_offset, out_leg_id, out_lane_id, estimate_travel_time);
                new_candidate.split_flexible_critical_resource_ = split_flexible_critical_resource;
                new_candidate.num_critical_resource_splitted_ = num_critical_resource_splitted;
                ready_list.push_back(new_candidate);
            }
        }
        SortReadyListAscendingly(ready_list);
    }
    return result_tree_;
}

SpanningTree Scheduler::ScheduleWithModifiedDfst(const Intersection &intersection) {
    // PrepareForTreeSchedule(cdg);

    // std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    // added_to_tree[0] = true;

    // std::vector<std::shared_ptr<Node>> unidirectional_scheduled_parent;
    // std::vector<std::shared_ptr<Node>> bidirectional_scheduled_parent;
    // int id_possible_parent;
    // int possible_depth;
    // std::shared_ptr<Edge> edge_from_possible_parent;

    // for (int id = 1; id < result_tree_.num_nodes_; id++) {
    //     unidirectional_scheduled_parent.clear();
    //     bidirectional_scheduled_parent.clear();
    //     id_possible_parent = -1;
    //     possible_depth = -1;
    //     edge_from_possible_parent = nullptr;
    //     for (int from = 0; from < result_tree_.num_nodes_; from++) {
    //         if (!added_to_tree[from]) {
    //             continue;
    //         }
    //         if (cdg.nodes_[from]->isConnectedTo(id)) {
    //             auto edge = cdg.nodes_[from]->getEdgeTo(id);
    //             if (edge->bidirectional_) {
    //                 bidirectional_scheduled_parent.push_back(result_tree_.nodes_[from]);
    //             }
    //             else {
    //                 unidirectional_scheduled_parent.push_back(result_tree_.nodes_[from]);
    //                 if (edge->edge_weight_ + result_tree_.nodes_[from]->edge_weighted_depth_ > possible_depth) {
    //                     possible_depth = edge->edge_weight_ + result_tree_.nodes_[from]->edge_weighted_depth_;
    //                     id_possible_parent = from;
    //                     edge_from_possible_parent = edge;
    //                 }
    //             }
    //         }
    //     }

    //     // if only connected by bidirectional edges, initiate possible depth with the smallest one
    //     if (id_possible_parent == -1) {
    //         int min_depth_parent_id;
    //         int min_depth;
    //         id_possible_parent = bidirectional_scheduled_parent.front()->id_;
    //         edge_from_possible_parent = cdg.nodes_[id_possible_parent]->getEdgeTo(id);
    //         possible_depth = bidirectional_scheduled_parent.front()->edge_weighted_depth_ + edge_from_possible_parent->edge_weight_;
    //         for (auto parent : bidirectional_scheduled_parent) {
    //             auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
    //             if (parent->edge_weighted_depth_ + edge->edge_weight_ < possible_depth) {
    //                 id_possible_parent = parent->id_;
    //                 possible_depth = parent->edge_weighted_depth_ + edge->edge_weight_;
    //                 edge_from_possible_parent = edge;
    //             }
    //         }
    //     }

    //     // update conflicts with other bidirectional edges
    //     bool flag_still_conflict_with_bidire_scheduled_neighbor;
    //     do {
    //         flag_still_conflict_with_bidire_scheduled_neighbor = false;
    //         for (auto parent : bidirectional_scheduled_parent) {
    //             auto edge = cdg.nodes_[parent->id_]->getEdgeTo(id);
    //             if (parent->edge_weighted_depth_ - edge->edge_weight_ < possible_depth && \
            //                 possible_depth < parent->edge_weighted_depth_ + edge->edge_weight_) {
            //                 id_possible_parent = parent->id_;
            //                 possible_depth = parent->edge_weighted_depth_ + edge->edge_weight_;
            //                 edge_from_possible_parent = edge;
            //                 flag_still_conflict_with_bidire_scheduled_neighbor = true;
            //             }
            //         }
            //     } while (flag_still_conflict_with_bidire_scheduled_neighbor);

            //     added_to_tree[id] = true;
            //     result_tree_.AddEdge(id_possible_parent, id, edge_from_possible_parent->edge_weight_);
            //     result_tree_.UpdateDepth(id, possible_depth, Type_EdgeWeightedDepth);
            // }

            // return result_tree_;
}

SpanningTree Scheduler::ScheduleWithBfstWeightedEdgeOnly(const Intersection &intersection) {
    // PrepareForTreeSchedule(cdg);

    // std::vector<Candidate> ready_list;
    // std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    // Candidate initial_root(0, 0, -1, -1);
    // ready_list.push_back(initial_root);

    // while (!ready_list.empty()) {
    //     Candidate chosen_candidate = ready_list[0];
    //     added_to_tree[chosen_candidate.id_] = true;
    //     result_tree_.UpdateDepth(chosen_candidate.id_, chosen_candidate.possible_depth_, Type_EdgeWeightedDepth);
    //     if (chosen_candidate.id_ > 0) {
    //         result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.edge_weight_);
    //     }
    //     ready_list.erase(ready_list.begin());

    //     auto iter_ready_candidate = ready_list.begin();
    //     while (iter_ready_candidate != ready_list.end()) {
    //         if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(iter_ready_candidate->id_)) {
    //             if (chosen_candidate.possible_depth_ + cdg.nodes_[chosen_candidate.id_]->getEdgeTo(iter_ready_candidate->id_)->edge_weight_ > \
            //                 iter_ready_candidate->possible_depth_) {
            //                 iter_ready_candidate = ready_list.erase(iter_ready_candidate);
            //                 continue;
            //             }
            //         }
            //         iter_ready_candidate++;
            //     }

            //     for (int to = 1; to < result_tree_.num_nodes_; to++) {
            //         if (added_to_tree[to] || isInList(to, ready_list) || !cdg.nodes_[chosen_candidate.id_]->isConnectedTo(to)) {
            //             continue;
            //         }
            //         if (StillHasUnscheduledPredecessor(unidirectional_parent_table_[to], added_to_tree)) {
            //             continue;
            //         }

            //         double edge_weight = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(to)->edge_weight_;
            //         Candidate new_candidate(to, chosen_candidate.possible_depth_ + edge_weight, chosen_candidate.id_, edge_weight);

            //         // update new_candidate and solve conflict with already scheduled nodes (both uni- and bi-directional)
            //         for (auto parent : unidirectional_parent_table_[to]) {
            //             edge_weight = parent->getEdgeTo(to)->edge_weight_;
            //             if (result_tree_.nodes_[parent->id_]->edge_weighted_depth_ + edge_weight > new_candidate.possible_depth_) {
            //                 new_candidate.possible_depth_ = result_tree_.nodes_[parent->id_]->edge_weighted_depth_ + edge_weight;
            //                 new_candidate.id_possible_parent_ = parent->id_;
            //                 new_candidate.edge_weight_ = edge_weight;
            //             }
            //         }
            //         bool flag_still_conflict_with_bidire_scheduled_neighbor;
            //         do {
            //             flag_still_conflict_with_bidire_scheduled_neighbor = false;
            //             for (auto neighbor : bidirectional_neighbor_table_[to]) {
            //                 if (!added_to_tree[neighbor->id_]) {
            //                     continue;
            //                 }
            //                 edge_weight = neighbor->getEdgeTo(to)->edge_weight_;
            //                 if (new_candidate.possible_depth_ > result_tree_.nodes_[neighbor->id_]->edge_weighted_depth_ - edge_weight &&
            //                     new_candidate.possible_depth_ < result_tree_.nodes_[neighbor->id_]->edge_weighted_depth_ + edge_weight) {
            //                     flag_still_conflict_with_bidire_scheduled_neighbor = true;
            //                     new_candidate.possible_depth_ = result_tree_.nodes_[neighbor->id_]->edge_weighted_depth_ + edge_weight;
            //                     new_candidate.id_possible_parent_ = neighbor->id_;
            //                     new_candidate.edge_weight_ = edge_weight;
            //                 }
            //             }
            //         } while (flag_still_conflict_with_bidire_scheduled_neighbor);
            //         ready_list.push_back(new_candidate);

            //     }
            //     SortReadyListAscendingly(ready_list);
            // }

            // return result_tree_;
}

SpanningTree Scheduler::ScheduleWithBfstMultiWeight(const Intersection &intersection) {
    // PrepareForTreeSchedule(cdg);

    // std::vector<Candidate> ready_list;
    // std::vector<bool> added_to_tree(cdg.num_nodes_, false);
    // Candidate initial_root(0, 0, -1, -1, -1);
    // ready_list.push_back(initial_root);

    // while (!ready_list.empty()) {
    //     Candidate chosen_candidate = ready_list[0];
    //     added_to_tree[chosen_candidate.id_] = true;
    //     result_tree_.UpdateDepth(chosen_candidate.id_, chosen_candidate.possible_depth_, Type_EdgeNodeWeightedDepth);
    //     if (chosen_candidate.id_ > 0) {
    //         result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.edge_weight_);
    //     }
    //     ready_list.erase(ready_list.begin());

    //     auto iter_ready_candidate = ready_list.begin();
    //     while (iter_ready_candidate != ready_list.end()) {
    //         if (cdg.nodes_[chosen_candidate.id_]->isConnectedTo(iter_ready_candidate->id_)) {
    //             double edge_weight = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(iter_ready_candidate->id_)->edge_weight_;
    //             // non-conflict edges don't delay time windows
    //             if (edge_weight <= 1.0) {
    //                 edge_weight = 0.0;
    //             }
    //             if (chosen_candidate.possible_depth_ + edge_weight + cdg.nodes_[iter_ready_candidate->id_]->estimate_travel_time_ > iter_ready_candidate->possible_depth_) {
    //                 iter_ready_candidate = ready_list.erase(iter_ready_candidate);
    //                 continue;
    //             }
    //         }
    //         iter_ready_candidate++;
    //     }

    //     for (int to = 1; to < result_tree_.num_nodes_; to++) {
    //         if (added_to_tree[to] || isInList(to, ready_list) || !cdg.nodes_[chosen_candidate.id_]->isConnectedTo(to)) {
    //             continue;
    //         }
    //         if (StillHasUnscheduledPredecessor(unidirectional_parent_table_[to], added_to_tree)) {
    //             continue;
    //         }
    //         double edge_weight = cdg.nodes_[chosen_candidate.id_]->getEdgeTo(to)->edge_weight_;
    //         // non-conflict edges don't delay time windows
    //         if (edge_weight <= 1.0) {
    //             edge_weight = 0.0;
    //         }
    //         double estimate_travel_time = cdg.nodes_[to]->estimate_travel_time_;
    //         Candidate new_candidate(to, chosen_candidate.possible_depth_ + edge_weight + estimate_travel_time, chosen_candidate.id_, edge_weight, estimate_travel_time);

    //         // update new_candidate and solve conflict with already scheduled nodes
    //         for (auto parent : unidirectional_parent_table_[to]) {
    //             edge_weight = parent->getEdgeTo(to)->edge_weight_;
    //             if (edge_weight <= 1.0) {
    //                 edge_weight = 0.0;
    //             }
    //             if (result_tree_.nodes_[parent->id_]->edge_node_weighted_depth_ + edge_weight + new_candidate.estimate_travel_time_ > new_candidate.possible_depth_) {
    //                 new_candidate.possible_depth_ = result_tree_.nodes_[parent->id_]->edge_node_weighted_depth_ + edge_weight + new_candidate.estimate_travel_time_;
    //                 new_candidate.id_possible_parent_ = parent->id_;
    //                 new_candidate.edge_weight_ = edge_weight;
    //             }
    //         }
    //         bool flag_still_conflict_with_bidire_scheduled_neighbor;
    //         do {
    //             flag_still_conflict_with_bidire_scheduled_neighbor = false;
    //             for (auto neighbor : bidirectional_neighbor_table_[to]) {
    //                 if (!added_to_tree[neighbor->id_]) {
    //                     continue;
    //                 }
    //                 edge_weight = neighbor->getEdgeTo(to)->edge_weight_;
    //                 if (edge_weight <= 1.0) {
    //                     edge_weight = 0.0;
    //                 }
    //                 if (new_candidate.possible_depth_ > result_tree_.nodes_[neighbor->id_]->edge_node_weighted_depth_ - neighbor->estimate_travel_time_ - edge_weight &&
    //                     new_candidate.possible_depth_ - new_candidate.estimate_travel_time_ < result_tree_.nodes_[neighbor->id_]->edge_node_weighted_depth_ + edge_weight) {
    //                     flag_still_conflict_with_bidire_scheduled_neighbor = true;
    //                     new_candidate.possible_depth_ = result_tree_.nodes_[neighbor->id_]->edge_node_weighted_depth_ + edge_weight + new_candidate.estimate_travel_time_;
    //                     new_candidate.id_possible_parent_ = neighbor->id_;
    //                     new_candidate.edge_weight_ = edge_weight;
    //                 }
    //             }
    //         } while (flag_still_conflict_with_bidire_scheduled_neighbor);
    //         ready_list.push_back(new_candidate);

    //     }
    //     SortReadyListAscendingly(ready_list);
    // }

    // return result_tree_;
}

std::vector<int> Scheduler::ScheduleBruteForceSearch(const Intersection &intersection) {
    // int num_nodes = cdg.num_nodes_;
    // double minimum_evacuation_time = -1.0;
    // std::vector<int> vehicle_order;
    // std::vector<bool> is_in_order_list(num_nodes, false);
    // vehicle_order.push_back(0);
    // is_in_order_list[0] = true;
    // std::vector<int> best_order;

    // GenerateUniparentTable(cdg);
    // GenerateBineighborTable(cdg);
    // SearchOrderPermutationRecursively(vehicle_order, num_nodes, is_in_order_list, minimum_evacuation_time, best_order, cdg);
    // return best_order;
}

void Scheduler::PrepareForTreeSchedule(const Intersection &intersection) {
    result_tree_.reset(false);
    result_tree_.AddNodesFromIntersection(intersection);
    GenerateUniparentTable(intersection);
    GenerateBineighborTable(intersection);
}

void Scheduler::GenerateUniparentTable(const Intersection &intersection) {
    unidirectional_parent_table_.clear();
    for (int id = 0; id < intersection.num_nodes_; id++) {
        std::vector<std::shared_ptr<Node>> uni_parent;
        for (int from = 0; from < intersection.num_nodes_; from++) {
            if (intersection.nodes_[from]->isConnectedWith(id)) {
                auto edge = intersection.nodes_[from]->getEdgeWith(id);
                auto &ct = edge->conflict_type_;
                if (ct.isPrecedence() && edge->predecessor_id_ == from) {
                    uni_parent.push_back(intersection.nodes_[from]);
                }
            }
        }
        unidirectional_parent_table_.push_back(uni_parent);
    }
}

void Scheduler::GenerateBineighborTable(const Intersection &intersection) {
    bidirectional_neighbor_table_.clear();
    for (int id = 0; id < intersection.num_nodes_; id++) {
        std::vector<std::shared_ptr<Node>> neighbors;
        for (int from = 0; from < intersection.num_nodes_; from++) {
            if (intersection.nodes_[from]->isConnectedWith(id)) {
                auto &ct = intersection.nodes_[from]->getEdgeWith(id)->conflict_type_;
                if (!ct.isPrecedence()) {
                    neighbors.push_back(intersection.nodes_[from]);
                }
            }
        }
        bidirectional_neighbor_table_.push_back(neighbors);
    }
}

void Scheduler::SearchOrderPermutationRecursively(std::vector<int> &vehicle_order, int num_nodes,
                                                  std::vector<bool> &is_in_order_list,
                                                  double &minimum_evacuation_time, std::vector<int> &best_order,
                                                  const Intersection &intersection) {
    // if (vehicle_order.size() >= num_nodes) {
    //     double evacuation_time;
    //     evacuation_time = GetEvacuationTimeFromOrder(vehicle_order, cdg);
    //     if (evacuation_time > 0) {
    //         if (minimum_evacuation_time < 0 || evacuation_time < minimum_evacuation_time) {
    //             minimum_evacuation_time = evacuation_time;
    //             best_order.clear();
    //             best_order.insert(best_order.end(), vehicle_order.begin(), vehicle_order.end());
    //         }
    //     }
    //     return;
    // }

    // for (int i = 1; i < num_nodes; i++) {
    //     if (is_in_order_list[i]) {
    //         continue;
    //     }
    //     vehicle_order.push_back(i);
    //     is_in_order_list[i] = true;
    //     SearchOrderPermutationRecursively(vehicle_order, num_nodes, is_in_order_list, minimum_evacuation_time, best_order, cdg);
    //     is_in_order_list[i] = false;
    //     vehicle_order.pop_back();
    // }
}

double Scheduler::GetEvacuationTimeFromOrder(const std::vector<int> &vehicle_order,
                                             const Intersection &intersection) {
    // std::vector<double> depth_of_the_order = GetDepthVectorFromOrder(vehicle_order, cdg);
    // if (depth_of_the_order.empty()) {
    //     return -1.0;
    // }

    // double evacuation_time = -1.0;
    // for (auto depth : depth_of_the_order) {
    //     if (depth > evacuation_time) {
    //         evacuation_time = depth;
    //     }
    // }
    // return evacuation_time;
}

std::vector<double> Scheduler::GetDepthVectorFromOrder(const std::vector<int> &vehicle_order,
                                                       const Intersection &intersection) {
    // std::vector<bool> vehicle_scheduled(vehicle_order.size(), false);
    // std::vector<double> depth_of_the_order(vehicle_order.size(), -1.0);
    // double cur_estimate_travel_time;
    // double edge_weight;
    // double possible_start_time;
    // double possible_end_time;

    // for (int cur_id : vehicle_order) {
    //     cur_estimate_travel_time = cdg.nodes_[cur_id]->estimate_travel_time_;
    //     possible_start_time = 0;
    //     for (auto parent : unidirectional_parent_table_[cur_id]) {
    //         if (!vehicle_scheduled[parent->id_]) {
    //             depth_of_the_order.clear();
    //             return depth_of_the_order;
    //         }
    //         edge_weight = parent->getEdgeTo(cur_id)->edge_weight_;
    //         if (edge_weight <= 1.0) {
    //             edge_weight = 0.0;
    //         }
    //         if (depth_of_the_order[parent->id_] + edge_weight > possible_start_time) {
    //             possible_start_time = depth_of_the_order[parent->id_] + edge_weight;
    //         }
    //     }
    //     possible_end_time = possible_start_time + cur_estimate_travel_time;
    //     bool flag;
    //     do {
    //         flag = false;
    //         for (auto neighbor : bidirectional_neighbor_table_[cur_id]) {
    //             if (!vehicle_scheduled[neighbor->id_]) {
    //                 continue;
    //             }
    //             edge_weight = neighbor->getEdgeTo(cur_id)->edge_weight_;
    //             if (edge_weight <= 1.0) {
    //                 edge_weight = 0.0;
    //             }
    //             if (possible_end_time > depth_of_the_order[neighbor->id_] - neighbor->estimate_travel_time_ - edge_weight &&
    //                 possible_start_time < depth_of_the_order[neighbor->id_] + edge_weight) {
    //                 flag = true;
    //                 possible_start_time = depth_of_the_order[neighbor->id_] + edge_weight;
    //                 possible_end_time = possible_start_time + cur_estimate_travel_time;
    //             }
    //         }
    //     } while (flag);
    //     depth_of_the_order[cur_id] = possible_end_time;
    //     vehicle_scheduled[cur_id] = true;
    // }
    // return depth_of_the_order;
}

} // namespace intersection_management