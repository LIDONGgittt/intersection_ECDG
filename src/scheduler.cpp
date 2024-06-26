#include "scheduler.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <parameters.h>
#include "colormod.h"

namespace intersection_management {

Scheduler::Scheduler()
{
    result_tree_.reset();
}

SpanningTree Scheduler::ScheduleWithDynamicLaneAssignment(Intersection &intersection)
{
    PrepareForTreeSchedule(intersection);
    std::vector<Candidate> ready_list;
    std::vector<bool> added_to_tree(intersection.num_nodes_, false);
    std::vector<bool> lane_fixed(intersection.num_nodes_, false);
    lane_fixed[0] = true;
    std::unordered_map<int, std::vector<double>> leg_lane_last_occupation;
    std::unordered_map<int, std::vector<int>> leg_lane_last_node_id;
    for (int leg_id = 0; leg_id < intersection.num_legs_; leg_id++)
    {
        leg_lane_last_occupation[leg_id] = std::vector<double>(1, 0.0);
        leg_lane_last_node_id[leg_id] = std::vector<int>(1, 0);
    }
    for (auto crm : intersection.critical_resource_map_)
    { // overwrite default value
        leg_lane_last_occupation[crm.first] = std::vector<double>(crm.second->getNumResources(), 0.0);
        leg_lane_last_node_id[crm.first] = std::vector<int>(crm.second->getNumResources(), 0);
    }
    Candidate initial_root(0, 0, -1, 0, -1, -1, 0);
    ready_list.push_back(initial_root);

    while (!ready_list.empty())
    {
        // add the node from the first candidate to the tree
        auto chosen_candidate = ready_list[0];
        auto &chosen_node = result_tree_.nodes_[chosen_candidate.id_];
        added_to_tree[chosen_candidate.id_] = true;
        result_tree_.UpdateTimeWindow(chosen_candidate.id_, chosen_candidate.possible_depth_, chosen_candidate.estimate_offset_, chosen_candidate.estimate_travel_time_);
        if (chosen_candidate.id_ > 0)
        {
            // update edge in the tree
            result_tree_.AddEdge(chosen_candidate.id_possible_parent_, chosen_candidate.id_, chosen_candidate.estimate_offset_);
            // update remaining demands
            remaining_demand_per_lane_[intersection.nodes_[chosen_candidate.id_]->route_->getLaneIn()->getUniqueId()]--;
            // add the node with all potentially available lanes to the tree
            chosen_node->possible_lane_id_.clear();
            chosen_node->possible_lane_id_.push_back(chosen_candidate.out_lane_id_);
            if (!chosen_candidate.split_flexible_critical_resource_)
            { // assign all possible lanes
                for (int i = 1; i < ready_list.size(); i++)
                {
                    if (ready_list[i].possible_depth_ <= chosen_candidate.possible_depth_)
                    {
                        if (ready_list[i].id_ == chosen_candidate.id_)
                        {
                            chosen_node->possible_lane_id_.push_back(ready_list[i].out_lane_id_);
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
            else
            { // chosen_candidate split critical resource, then needs to update lane assignments in the tree
                std::unordered_set<int> conflict_competing_lanes;
                auto &competing_node = result_tree_.nodes_[chosen_candidate.competing_node_id_];
                std::shared_ptr<Route> route_chosen_candidate = std::make_shared<Route>(
                    intersection.leg_map_[chosen_node->in_leg_id_]->lanes_in_map_[chosen_node->in_lane_id_],
                    intersection.leg_map_[chosen_candidate.out_leg_id_]->lanes_out_map_[chosen_candidate.out_lane_id_]);
                for (int lane_id : competing_node->possible_lane_id_)
                {
                    auto route_competing = std::make_shared<Route>(
                        intersection.leg_map_[competing_node->in_leg_id_]->lanes_in_map_[competing_node->in_lane_id_],
                        intersection.leg_map_[competing_node->out_leg_id_]->lanes_out_map_[lane_id]);
                    auto ct = route_chosen_candidate->FindConflictTypeWithRoute(route_competing);
                    if (ct.isConverging() || ct.isCrossing())
                    {
                        conflict_competing_lanes.insert(lane_id);
                    }
                }
                auto iter = competing_node->possible_lane_id_.begin();
                while (iter != competing_node->possible_lane_id_.end())
                {
                    if (conflict_competing_lanes.find(*iter) != conflict_competing_lanes.end())
                    {
                        iter = competing_node->possible_lane_id_.erase(iter);
                        continue;
                    }
                    iter++;
                }

                if (competing_node->possible_lane_id_.size() == 1)
                {
                    lane_fixed[competing_node->id_] = true;
                    leg_lane_last_occupation[competing_node->out_leg_id_][competing_node->possible_lane_id_[0]] = competing_node->depth_;
                    leg_lane_last_node_id[competing_node->out_leg_id_][competing_node->possible_lane_id_[0]] = competing_node->id_;
                }
            }
            if (chosen_node->possible_lane_id_.size() == 1)
            {
                lane_fixed[chosen_node->id_] = true;
                leg_lane_last_occupation[chosen_node->out_leg_id_][chosen_node->possible_lane_id_[0]] = chosen_node->depth_;
                leg_lane_last_node_id[chosen_node->out_leg_id_][chosen_node->possible_lane_id_[0]] = chosen_node->id_;
            }
        }

        // remove chosen candidate and all disabled candidates
        ready_list.erase(ready_list.begin());
        auto iter_ready_candidate = ready_list.begin();
        std::unordered_set<int> crossing_conflict_node_set;
        while (iter_ready_candidate != ready_list.end())
        {
            // candidates for the same node is disabled
            if (iter_ready_candidate->id_ == chosen_candidate.id_)
            {
                iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                continue;
            }
            if (intersection.nodes_[chosen_candidate.id_]->isConnectedWith(iter_ready_candidate->id_))
            {
                auto edge = intersection.nodes_[chosen_candidate.id_]->getEdgeWith(iter_ready_candidate->id_);
                // competing nodes in all lanes are disabled
                if (edge->conflict_type_.isCompeting())
                {
                    iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                    continue;
                }
                // crossing and converging but not competing (only one out lane in the leg) nodes 
                // with conflicting candidate window will be removed,
                // all candidates for the crossing node will be removed
                if (edge->conflict_type_.isCrossing() || edge->conflict_type_.isConverging())
                {
                    double edge_weight = edge->edge_weight_;
                    if (chosen_candidate.possible_depth_ > iter_ready_candidate->possible_depth_ - edge_weight - intersection.nodes_[iter_ready_candidate->id_]->estimate_travel_time_)
                    {
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
        while (iter_ready_candidate != ready_list.end())
        {
            if (crossing_conflict_node_set.find(iter_ready_candidate->id_) != crossing_conflict_node_set.end())
            {
                iter_ready_candidate = ready_list.erase(iter_ready_candidate);
                continue;
            }
            iter_ready_candidate++;
        }

        // add all possible candidates
        for (int new_node_id = 1; new_node_id < result_tree_.num_nodes_; new_node_id++)
        {
            if (added_to_tree[new_node_id] || isInList(new_node_id, ready_list) || !intersection.nodes_[chosen_candidate.id_]->isConnectedWith(new_node_id))
            {
                continue;
            }
            if (StillHasUnscheduledPredecessor(unidirectional_parent_table_[new_node_id], added_to_tree))
            {
                continue;
            }

            auto &new_node = intersection.nodes_[new_node_id];
            int out_leg_id = new_node->out_leg_id_;
            double estimate_travel_time = new_node->estimate_travel_time_;
            double estimate_arrival_time = new_node->estimate_arrival_time_;

            double earliest_start_time;
            if (param.activate_arrival_time) {
                earliest_start_time = new_node->estimate_arrival_time_;
            }
            else {
                earliest_start_time = -1;
            }

            int possible_precedent_parent_id = 0;
            for (auto parent : unidirectional_parent_table_[new_node_id])
            {
                if (result_tree_.nodes_[parent->id_]->depth_ > earliest_start_time)
                {
                    earliest_start_time = result_tree_.nodes_[parent->id_]->depth_;
                    possible_precedent_parent_id = parent->id_;
                }
            }
            double precedence_offset = 0;
            if (param.activate_precedent_offset && new_node->getEdgeWith(possible_precedent_parent_id)->conflict_type_.isDiverging())
            {
                precedence_offset = -1;
            }

            // update new_candidate for each possible lane and solve conflicts with already scheduled nodes
            for (int out_lane_id = 0; out_lane_id < leg_lane_last_occupation[out_leg_id].size(); out_lane_id++)
            {
                int best_parent_id = possible_precedent_parent_id;
                double best_start_time = earliest_start_time;
                double best_estimate_offset = precedence_offset;
                double best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                std::shared_ptr<Route> route_new_candidate = std::make_shared<Route>(intersection.leg_map_[new_node->in_leg_id_]->lanes_in_map_[new_node->in_lane_id_],
                                                                                     intersection.leg_map_[out_leg_id]->lanes_out_map_[out_lane_id]);
                bool split_flexible_critical_resource = false;
                int num_critical_resource_splitted = 0;
                int competing_node_id = 0;

                if (best_start_time < leg_lane_last_occupation[out_leg_id][out_lane_id])
                {
                    best_parent_id = leg_lane_last_node_id[out_leg_id][out_lane_id];
                    best_start_time = leg_lane_last_occupation[out_leg_id][out_lane_id];
                    best_estimate_offset = 0;
                    best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                }

                bool flag_still_conflict_with_bidire_scheduled_neighbor;
                do
                {
                    flag_still_conflict_with_bidire_scheduled_neighbor = false;
                    for (auto neighbor_in_intersection : bidirectional_neighbor_table_[new_node_id])
                    {
                        if (!added_to_tree[neighbor_in_intersection->id_])
                        {
                            continue;
                        }
                        auto &neighbor = result_tree_.nodes_[neighbor_in_intersection->id_];
                        if (!new_node->getEdgeWith(neighbor->id_)->conflict_type_.isCompeting())
                        { // not competing, then must crossing
                            if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1])
                            {
                                flag_still_conflict_with_bidire_scheduled_neighbor = true;
                                best_parent_id = neighbor->id_;
                                best_start_time = neighbor->time_window_[1];
                                best_estimate_offset = 0;
                                best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                                split_flexible_critical_resource = false;
                            }
                        }
                        else
                        { // competing, need to dynamicly assign lanes
                            if (lane_fixed[neighbor->id_])
                            {
                                std::shared_ptr<Route> route_neighbor = std::make_shared<Route>(
                                    intersection.leg_map_[neighbor->in_leg_id_]->lanes_in_map_[neighbor->in_lane_id_],
                                    intersection.leg_map_[neighbor->out_leg_id_]->lanes_out_map_[neighbor->possible_lane_id_[0]]);
                                auto ct = route_new_candidate->FindConflictTypeWithRoute(route_neighbor);
                                if (ct.isConverging() || ct.isCrossing())
                                { // only resolve conflict when converging or crossing
                                    if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1])
                                    {
                                        flag_still_conflict_with_bidire_scheduled_neighbor = true;
                                        best_parent_id = neighbor->id_;
                                        best_start_time = neighbor->time_window_[1];
                                        best_estimate_offset = 0;
                                        best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                                        split_flexible_critical_resource = false;
                                    }
                                }
                            }
                            else
                            { // neighbor's lane is not fixed
                                int non_conflict_count = 0;
                                for (int neighbor_lane_id : neighbor->possible_lane_id_)
                                {
                                    auto route_neighbor = std::make_shared<Route>(
                                        intersection.leg_map_[neighbor->in_leg_id_]->lanes_in_map_[neighbor->in_lane_id_],
                                        intersection.leg_map_[neighbor->out_leg_id_]->lanes_out_map_[neighbor_lane_id]);
                                    auto ct = route_new_candidate->FindConflictTypeWithRoute(route_neighbor);
                                    if (!ct.isConverging() && !ct.isCrossing())
                                    {
                                        non_conflict_count++;
                                    }
                                }
                                if (non_conflict_count < neighbor->possible_lane_id_.size())
                                { // otherwise will complete non-conflict, which is safe
                                    if (non_conflict_count > 0)
                                    { // may split critical resources
                                        if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1])
                                        {
                                            split_flexible_critical_resource = true;
                                            num_critical_resource_splitted = neighbor->possible_lane_id_.size() - non_conflict_count;
                                            competing_node_id = neighbor->id_;
                                        }
                                    }
                                    else
                                    { // conflict with all possible lanes, thus have to be scheduled later, and will not split critical resources
                                        if (best_depth > neighbor->time_window_[0] && best_start_time < neighbor->time_window_[1])
                                        {
                                            flag_still_conflict_with_bidire_scheduled_neighbor = true;
                                            best_parent_id = neighbor->id_;
                                            best_start_time = neighbor->time_window_[1];
                                            best_estimate_offset = 0;
                                            best_depth = best_start_time + estimate_travel_time + best_estimate_offset;
                                            split_flexible_critical_resource = false;
                                            num_critical_resource_splitted = 0;
                                            competing_node_id = 0;
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
                new_candidate.competing_node_id_ = competing_node_id;
                ready_list.push_back(new_candidate);
            }
        }
        SortReadyListAscendingly(ready_list, intersection);
    }
    return result_tree_;
}

SpanningTree Scheduler::ScheduleWithFIFO(Intersection &intersection) {
    PrepareForTreeSchedule(intersection);
    std::vector<bool> added_to_tree(intersection.num_nodes_, false);

    result_tree_.UpdateTimeWindow(0, 0, 0, 0);
    added_to_tree[0] = true;

    for (int id = 1; id < intersection.num_nodes_; id++)
    {
        auto &chosen_node = result_tree_.nodes_[id];
        if (StillHasUnscheduledPredecessor(unidirectional_parent_table_[id], added_to_tree)) {
            std::cerr << Color::red << "Error in FIFO, precendence vehicle arrives later than following vehicle.\n" << Color::def;
            throw;
        }

        double estimate_travel_time = chosen_node->estimate_travel_time_;
        double estimate_arrival_time = chosen_node->estimate_arrival_time_;
        double earliest_start_time;
        if (param.activate_arrival_time) {
            earliest_start_time = chosen_node->estimate_arrival_time_;
        }
        else {
            earliest_start_time = -1;
        }
        if (earliest_start_time < result_tree_.nodes_[id - 1]->time_window_[0]) {
            earliest_start_time = result_tree_.nodes_[id - 1]->time_window_[0];
        }

        for (int pre_id = id - 1; pre_id > 0; pre_id--) {
            if (intersection.nodes_[id]->isConnectedWith(pre_id)) {
                auto edge = intersection.nodes_[id]->getEdgeWith(pre_id);
                if (edge->conflict_type_.isConverging() || edge->conflict_type_.isCrossing() || edge->conflict_type_.isDiverging() || edge->conflict_type_.isPrecedence()) {
                    if (earliest_start_time < result_tree_.nodes_[pre_id]->time_window_[1]) {
                        earliest_start_time = result_tree_.nodes_[pre_id]->time_window_[1];
                    }
                }
            }
        }

        added_to_tree[id] = true;
        result_tree_.nodes_[id]->possible_lane_id_.clear();
        result_tree_.nodes_[id]->possible_lane_id_.push_back(intersection.nodes_[id]->out_lane_id_);
        result_tree_.UpdateTimeWindow(id, earliest_start_time + estimate_travel_time, 0, estimate_travel_time);
    }
    return result_tree_;
}

void Scheduler::PrepareForTreeSchedule(Intersection &intersection)
{
    result_tree_.reset(false);
    result_tree_.AddNodesFromIntersection(intersection);
    GenerateUniparentTable(intersection);
    GenerateBineighborTable(intersection);

    remaining_demand_per_lane_.clear();
    remaining_demand_per_lane_.resize(intersection.lane_map_.size(), 0);
    for (int id = 1; id < intersection.nodes_.size(); id++)
    {
        remaining_demand_per_lane_[intersection.nodes_[id]->route_->getLaneIn()->getUniqueId()]++;
    }
}

void Scheduler::GenerateUniparentTable(Intersection &intersection)
{
    unidirectional_parent_table_.clear();
    for (int id = 0; id < intersection.num_nodes_; id++)
    {
        std::vector<std::shared_ptr<Node>> uni_parent;
        for (int from = 0; from < intersection.num_nodes_; from++)
        {
            if (intersection.nodes_[from]->isConnectedWith(id))
            {
                auto edge = intersection.nodes_[from]->getEdgeWith(id);
                auto &ct = edge->conflict_type_;
                if (ct.isPrecedence() && edge->predecessor_id_ == from)
                {
                    uni_parent.push_back(intersection.nodes_[from]);
                }
            }
        }
        unidirectional_parent_table_.push_back(uni_parent);
    }
}

void Scheduler::GenerateBineighborTable(Intersection &intersection)
{
    bidirectional_neighbor_table_.clear();
    for (int id = 0; id < intersection.num_nodes_; id++)
    {
        std::vector<std::shared_ptr<Node>> neighbors;
        for (int from = 0; from < intersection.num_nodes_; from++)
        {
            if (intersection.nodes_[from]->isConnectedWith(id))
            {
                auto &ct = intersection.nodes_[from]->getEdgeWith(id)->conflict_type_;
                if (!ct.isPrecedence())
                {
                    neighbors.push_back(intersection.nodes_[from]);
                }
            }
        }
        bidirectional_neighbor_table_.push_back(neighbors);
    }
}

void Scheduler::SortReadyListAscendingly(std::vector<Candidate> &ready_list, Intersection &intersection)
{
    std::sort(ready_list.begin(), ready_list.end(),
              [&](const Candidate &candidate1, const Candidate &candidate2)
              {
                  if (candidate1.possible_depth_ != candidate2.possible_depth_)
                  {
                      return candidate1.possible_depth_ < candidate2.possible_depth_;
                  }

                  // Break the tie
                  if (param.tie_high_demand_first)
                  {
                      if (candidate1.id_ != 0 && candidate2.id_ != 0)
                      {
                          int demand1 = remaining_demand_per_lane_[intersection.nodes_[candidate1.id_]->route_->getLaneIn()->getUniqueId()];
                          int demand2 = remaining_demand_per_lane_[intersection.nodes_[candidate2.id_]->route_->getLaneIn()->getUniqueId()];
                          if (demand1 != demand2)
                          {
                              return demand1 > demand2;
                          }
                      }
                  }
                  if (param.tie_minimum_resource_waste_first)
                  {
                      bool isCompetingRightmost1 = intersection.isRightmostTurningRoute(intersection.nodes_[candidate1.id_]->route_);
                      bool isCompetingRightmost2 = intersection.isRightmostTurningRoute(intersection.nodes_[candidate2.id_]->route_);
                      if (isCompetingRightmost1 && intersection.critical_resource_map_.find(candidate1.out_leg_id_) == intersection.critical_resource_map_.end())
                          isCompetingRightmost1 = false;
                      if (isCompetingRightmost2 && intersection.critical_resource_map_.find(candidate2.out_leg_id_) == intersection.critical_resource_map_.end())
                          isCompetingRightmost2 = false;

                      if (isCompetingRightmost1 && !isCompetingRightmost2)
                      {
                          return true;
                      }
                      if (!isCompetingRightmost1 && isCompetingRightmost2)
                      {
                          return false;
                      }
                  }
                  if (param.tie_consider_splitting_resource)
                  {
                      if (candidate1.split_flexible_critical_resource_ && !candidate2.split_flexible_critical_resource_)
                          return true;
                      if (!candidate1.split_flexible_critical_resource_ && candidate2.split_flexible_critical_resource_)
                          return false;
                      if (candidate1.split_flexible_critical_resource_ && candidate2.split_flexible_critical_resource_ &&
                          (candidate1.num_critical_resource_splitted_ != candidate2.num_critical_resource_splitted_))
                          if (param.tie_more_splitted_resource_first)
                          {
                              return candidate1.num_critical_resource_splitted_ > candidate2.num_critical_resource_splitted_;
                          }
                          else
                          {
                              return candidate1.num_critical_resource_splitted_ < candidate2.num_critical_resource_splitted_;
                          }
                  }
                  return candidate1.id_ < candidate2.id_;
              });
}

bool Scheduler::isInList(int id, std::vector<Candidate> &ready_list)
{
    for (auto &item : ready_list)
    {
        if (id == item.id_)
        {
            return true;
        }
    }
    return false;
}

bool Scheduler::StillHasUnscheduledPredecessor(std::vector<std::shared_ptr<Node>> &pre, std::vector<bool> &added_to_tree)
{
    for (auto node : pre)
    {
        if (added_to_tree[node->id_] == false)
        {
            return true;
        }
    }
    return false;
}

void Scheduler::printDepthVector(std::vector<double> &depth_vector)
{
    int tmp_cnt = 0;
    for (int i = 0; i < depth_vector.size(); i++)
    {
        std::cout << "Node " << i << ": " << depth_vector[i] << ", ";
        if (++tmp_cnt % 6 == 0)
            std::cout << std::endl;
    }
    std::cout << std::endl;
}

void Scheduler::printOrder(std::vector<int> &order)
{
    int tmp_cnt = 0;
    for (auto id : order)
    {
        std::cout << "Node " << id << " -> ";
        if (++tmp_cnt % 10 == 0)
            std::cout << std::endl;
    }
    std::cout << std::endl;
}

} // namespace intersection_management