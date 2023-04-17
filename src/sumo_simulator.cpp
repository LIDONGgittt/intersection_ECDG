#include "sumo_simulator.h"
#include "sumo_sim_utility.h"

#include <iostream>
#include <libsumo/libtraci.h>

#include "scheduler.h"
#include "conflict_directed_graph.h"
#include "cdg_scheduler.h"
#include "time_profiler/time_profiler.h"
#include "parameters.h"

using namespace libtraci;

namespace intersection_management {

extern Parameters param;

void SumoSimulator::addVehicles(std::string schedule_method) {
    std::vector<std::shared_ptr<Node>> scheduled_vehicles;
    if (schedule_method == "mdbfs") {
        auto &target_tree = mdbfst_;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            auto &node = target_tree.nodes_[i];
            node->time_window_[1] = node->edge_node_weighted_depth_;
            node->time_window_[0] = node->edge_node_weighted_depth_ - node->estimate_travel_time_;
            node->possible_lane_id_ = std::vector<int>({node->out_lane_id_});
            scheduled_vehicles.push_back(node);
        }
    }
    else if (schedule_method == "dfs") {
        auto &target_tree = modified_dfst_;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            auto &node = target_tree.nodes_[i];
            node->time_window_[1] = node->edge_weighted_depth_ * travel_time_choice_[2];
            node->time_window_[0] = (node->edge_weighted_depth_ - 1) * travel_time_choice_[2];
            node->possible_lane_id_ = std::vector<int>({node->out_lane_id_});
            scheduled_vehicles.push_back(node);
        }
    }
    else if (schedule_method == "bfs") {
        auto &target_tree = bfst_;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            auto &node = target_tree.nodes_[i];
            node->time_window_[1] = node->edge_weighted_depth_ * travel_time_choice_[2];
            node->time_window_[0] = (node->edge_weighted_depth_ - 1) * travel_time_choice_[2];
            node->possible_lane_id_ = std::vector<int>({node->out_lane_id_});
            scheduled_vehicles.push_back(node);
        }
    }
    else { // default is dynamic lane assignment scheduler
        auto &target_tree = result_tree_;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            scheduled_vehicles.push_back(target_tree.nodes_[i]);
        }
    }
    for (int i = 0; i < scheduled_vehicles.size(); i++) {
        auto &node = scheduled_vehicles[i];
        std::string routeId = "r";
        routeId.push_back('0' + node->in_leg_id_);
        routeId.push_back('0' + node->out_leg_id_);
        std::string departLaneID = std::to_string(intersectionLaneIdToSumoLaneId(node->in_leg_id_, node->in_lane_id_, "in"));
        std::string arrivalLaneID = std::to_string(intersectionLaneIdToSumoLaneId(node->out_leg_id_, node->possible_lane_id_[0], "out"));
        localVehicles_.push_back(LocalVehicle(std::to_string(node->id_), routeId, "Vtype1", "now", departLaneID, "0", "0", arrivalLaneID));

        localVehicles_.back().color_ = sumoColorVec[i % sumoColorVec.size()];
        localVehicles_.back().arrival_time_ = node->estimate_arrival_time_;
        localVehicles_.back().timewindow_[0] = kTimeWindowOffset_ + node->time_window_[0];
        localVehicles_.back().timewindow_[1] = kTimeWindowOffset_ + node->time_window_[1];
    }

}

void SumoSimulator::startSimulation() {
    Simulation::start(sumo_cmd_);

    double currentTime = 0;
    double nextPrintTime = 10.0;
    for (int i = 1; i < 400000; i++) {
        currentTime = i * 0.001;
        for (auto &veh : localVehicles_) {
            if (currentTime >= veh.arrival_time_ && !veh.addedToSumo_) {
                Vehicle::add(veh.vehID_, veh.routeID_, veh.typeID_, veh.depart_, veh.departLaneID_, veh.departPos_,
                             veh.departSpeed_, veh.arrivalLaneID_, veh.arrivalPos_, veh.arrivalSpeed_);
                Vehicle::setColor(veh.vehID_, veh.color_);
                Vehicle::setStop(veh.vehID_, veh.departEdgeID_, libtraci::Lane::getLength(veh.departEdgeID_ + "_" + veh.departLaneID_),
                                 veh.departLaneIDNum_, 0, libsumo::STOP_DEFAULT,
                                 libtraci::Lane::getLength(veh.departEdgeID_ + "_" + veh.departLaneID_) - 0.1, veh.timewindow_[0]);
                veh.addedToSumo_ = true;
            }
        }

        Simulation::step(currentTime);

        for (auto &veh : localVehicles_) {
            if (veh.addedToSumo_ && !veh.hasFinished()) {
                // enforce lanes
                if (!veh.hasPassedIntersectionStopLine()) {
                    Vehicle::changeLane(veh.vehID_, veh.departLaneIDNum_, 0.1);
                }
                else {
                    Vehicle::changeLane(veh.vehID_, veh.arrivalLaneIDNum_, 0.1);
                }
                // cumulate fuel consumption
                // std::cout << Vehicle::getFuelConsumption(veh.vehID_) << std::endl;
                veh.fuelConsumed_ += ((Vehicle::getFuelConsumption(veh.vehID_) > 0) ? Vehicle::getFuelConsumption(veh.vehID_) : 0) * 0.001;
            }
        }

        if (currentTime >= nextPrintTime) {
            nextPrintTime += 10;
            printFuelConsumption(localVehicles_, currentTime);
        }

        bool allPassed = true;
        for (auto &veh : localVehicles_) {
            if (veh.addedToSumo_ && !veh.hasFinished()) {
                if (!veh.hasLeftIntersectionAndEnterArrivalLane()) {
                    allPassed = false;
                }
            }
        }
        if (allPassed && stopSimAfterClearanceFlag_) {
            std::cout << "The intersection is evacuated at time: " << i << " ms.\n";
            printFuelConsumption(localVehicles_, currentTime);
            getchar();
            stopSimAfterClearanceFlag_ = false;
        }
    }
    Simulation::close();
}

void SumoSimulator::simulateOneRandomCase(int num_nodes, std::string schedule_method, bool verbose, int seed) {
    generateSchedulingResults(num_nodes, schedule_method, verbose, seed);
    addVehicles();
    startSimulation();
}

void SumoSimulator::generateSchedulingResults(int num_nodes, std::string schedule_method, bool verbose, int seed) {
    param.num_lanes_in_vec = {3, 3, 3, 3};
    param.num_lanes_out_vec = {3, 3, 3, 3};

    Intersection intersection;
    ConflictDirectedGraph cdg;
    Scheduler scheduler;
    CDGScheduler scheduler_dfs;
    CDGScheduler scheduler_bfs;
    CDGScheduler scheduler_mdbfs;
    CDGScheduler scheduler_bruteforce;

    intersection.setSeed(seed);
    intersection.AddRandomVehicleNodesWithTravelTime(num_nodes, travel_time_choice_, verbose);
    intersection.AssignCriticalResourcesToNodes();
    intersection.AssignRoutesToNodes();
    intersection.AssignEdgesWithSafetyOffsetToNodes();

    cdg.GenerateGraphFromIntersection(intersection);

    result_tree_ = scheduler.ScheduleWithDynamicLaneAssignment(intersection);
    modified_dfst_ = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
    bfst_ = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
    mdbfst_ = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
    global_optimal_ = 0;
    if (cdg.num_nodes_ <= 16) { // only calculate global_optimal for small number of nodes
        auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
        auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
        global_optimal_ = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
    }

    if (verbose) {
        std::cout << "seed: " << seed << "\n";
        std::cout << "dynamin lane assignment: " << result_tree_.depth_ << "\n";
        std::cout << "modified dfs: " << modified_dfst_.edge_weighted_depth_ << "\n";
        std::cout << "edge_weighted bfs: " << bfst_.edge_weighted_depth_ << "\n";
        std::cout << "multi_weighted bfs: " << mdbfst_.edge_node_weighted_depth_ << "\n";
        std::cout << "global_optimal: " << global_optimal_ << "\n";
        std::cout << "=========================================\n";
        std::cout << "Dynamic lane schedule:\n";
        for (auto &node : result_tree_.nodes_)
            node->printDetail();
        std::cout << "=========================================\n";
    }
}
} // namespace intersection_management 