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