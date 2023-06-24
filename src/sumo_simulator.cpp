#include "sumo_simulator.h"
#include "sumo_sim_utility.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <libsumo/libtraci.h>

#include "scheduler.h"
#include "conflict_directed_graph.h"
#include "cdg_scheduler.h"
#include "time_profiler/time_profiler.h"
#include "parameters.h"
#include "colormod.h"

using namespace libtraci;

namespace intersection_management {


void SumoSimulator::setSimulateOneMethod(int num_nodes, std::string schedule_method, double arrival_interval_avg, int geometryID, bool verbose, int seed) {
    schedule_method_ = schedule_method;
    arrival_interval_avg_ = arrival_interval_avg;
    seed_ = seed;

    localParam_ = geometryParamVec[geometryID];
    localParam_.arrival_interval_avg = arrival_interval_avg;
    localParam_.random_seed = seed;
    sumo_cmd_[2] = PROJECT_DIR + localParam_.sumo_config_file;
    travel_time_choice_ = localParam_.travel_time_choice;
    kTimeWindowOffset_ = localParam_.kTimeWindowOffset;

    generateSchedulingResults(num_nodes, verbose, seed_);
    addVehicles(schedule_method_);
    if (verbose)
        printTargetSimResults(schedule_method_);
}

void SumoSimulator::generateSchedulingResults(int num_nodes, bool verbose, int seed) {

    // intersection will be initialized based on the geomtry of localParam_
    Intersection intersection(localParam_);
    ConflictDirectedGraph cdg;
    Scheduler scheduler;
    CDGScheduler scheduler_dfs;
    CDGScheduler scheduler_bfs;
    CDGScheduler scheduler_mdbfs;
    CDGScheduler scheduler_bruteforce;
    CDGScheduler scheduler_mddfs;

    intersection.setSeed(seed);
    intersection.AddRandomVehicleNodesWithTravelTime(num_nodes, travel_time_choice_, verbose);
    intersection.AssignCriticalResourcesToNodes();
    intersection.AssignRoutesToNodes();
    intersection.AssignEdgesWithSafetyOffsetToNodes();

    cdg.GenerateGraphFromIntersection(intersection);

    dynamic_lane_tree_ = scheduler.ScheduleWithDynamicLaneAssignment(intersection);
    modified_dfst_ = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
    bfst_ = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
    mdbfst_ = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
    mddfst_ = scheduler_mddfs.ScheduleWithDfstMultiWeight(cdg);
    global_optimal_ = 0;
    if (schedule_method_ == "global_optimal") {
        if (cdg.num_nodes_ <= 16) { // only calculate global_optimal for small number of nodes
            auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
            auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
            global_optimal_ = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
        }
    }
    fifo_tree_ = scheduler.ScheduleWithFIFO(intersection);

    standard_depth_[0] = dynamic_lane_tree_.depth_;
    standard_depth_[1] = modified_dfst_.edge_weighted_depth_ * travel_time_choice_[2];
    standard_depth_[2] = bfst_.edge_weighted_depth_ * travel_time_choice_[2];
    standard_depth_[3] = mdbfst_.edge_node_weighted_depth_;
    standard_depth_[4] = global_optimal_;
    standard_depth_[5] = fifo_tree_.depth_;
    standard_depth_[6] = mddfst_.edge_node_weighted_depth_;

    if (verbose) {
        std::cout << "=========================================\n";
        std::cout << "seed: " << seed << "\n";
        std::cout << "dynamin_lane assignment: " << standard_depth_[0] << "\n";
        std::cout << "modified dfs: " << standard_depth_[1] << "\n";
        std::cout << "edge_weighted bfs: " << standard_depth_[2] << "\n";
        std::cout << "multi_weighted bfs: " << standard_depth_[3] << "\n";
        std::cout << "multi_weighted dfs: " << standard_depth_[6] << "\n";
        std::cout << "global_optimal: " << standard_depth_[4] << "\n";
        std::cout << "FIFO: " << standard_depth_[5] << "\n";
        std::cout << "=========================================\n";
    }
}

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
    else if (schedule_method == "mddfs") {
        auto &target_tree = mddfst_;
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
    else if (schedule_method == "fifo") {
        auto &target_tree = fifo_tree_;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            scheduled_vehicles.push_back(target_tree.nodes_[i]);
        }
    }
    else { // default is dynamic lane assignment scheduler
        auto &target_tree = dynamic_lane_tree_;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            scheduled_vehicles.push_back(target_tree.nodes_[i]);
        }
    }
    
    for (int i = 0; i < scheduled_vehicles.size(); i++) {
        auto &node = scheduled_vehicles[i];
        std::string routeId = "r";
        routeId.push_back('0' + node->in_leg_id_);
        routeId.push_back('0' + node->out_leg_id_);
        std::string departLaneID = std::to_string(intersectionLaneIdToSumoLaneId(node->in_leg_id_, node->in_lane_id_, localParam_, "in"));
        std::string arrivalLaneID = std::to_string(intersectionLaneIdToSumoLaneId(node->out_leg_id_, node->possible_lane_id_[0], localParam_, "out"));
        localVehicles_.push_back(LocalVehicle(std::to_string(node->id_), routeId, "Vtype1", "now", departLaneID, "0", "speedLimit", arrivalLaneID));

        localVehicles_.back().color_ = sumoColorVec[i % sumoColorVec.size()];
        localVehicles_.back().arrival_time_ = node->estimate_arrival_time_;
        localVehicles_.back().timewindow_[0] = kTimeWindowOffset_ + node->time_window_[0];
        localVehicles_.back().timewindow_[1] = kTimeWindowOffset_ + node->time_window_[1];
    }

}

void SumoSimulator::startSimulation(bool verbose) {
    Simulation::start(sumo_cmd_);

    double currentTime = 0;
    double nextPrintTime = 20.0;
    long long total_step = (int)(max_sim_time_ / step_length_);
    bool foundEvacuationTime = false;

    for (int i = 1; i < total_step; i++) {
        currentTime = i * step_length_;
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

        // std::cout << "Vehicle 0 has max speed of: " << Vehicle::getMaxSpeed("1") << "\n"
        // <<libtraci::Lane::getMaxSpeed(localVehicles_[0].departEdgeID_ + "_" + localVehicles_[0].departLaneID_) << "\n";
        // if (localVehicles_[0].addedToSumo_ && !localVehicles_[0].hasFinished()) {
        //     std::cout<<Vehicle::getStopDelay(localVehicles_[0].vehID_)<<std::endl;
        // }
        // if (!test && localVehicles_[0].hasPassedIntersectionStopLine()) {
        //     std::cout << localVehicles_[0].departEdgeID_ + " to " + localVehicles_[0].arrivalEdgeID_ << " : "
        //         << currentTime << "  -  ";
        //     test = true;
        // }
        // if (localVehicles_[0].hasLeftIntersectionAndEnterArrivalLane()) {
        //     std::cout << currentTime << std::endl;
        //     break;
        // }

        for (auto &veh : localVehicles_) {
            if (veh.addedToSumo_ && !veh.hasFinished()) {
                // if (veh.hasPassedIntersectionStopLine()) {
                //     getchar();
                // }
                // enforce lanes
                // if (!veh.hasPassedIntersectionStopLine()) {
                //     Vehicle::changeLane(veh.vehID_, veh.departLaneIDNum_, 0.1);
                // }
                // else {
                //     Vehicle::changeLane(veh.vehID_, veh.arrivalLaneIDNum_, 0.1);
                // }
                // cumulate fuel consumption
                veh.fuelConsumed_ += ((Vehicle::getFuelConsumption(veh.vehID_) > 0) ? Vehicle::getFuelConsumption(veh.vehID_) : 0) * step_length_;
                if (Vehicle::getSpeed(veh.vehID_) < 0.1)
                    veh.waitingTime_ += step_length_;

            }
        }

        if (currentTime >= nextPrintTime) {
            nextPrintTime += 20;
            if (verbose) {
                printFuelConsumptionSummary();
            }
        }

        bool allPassed = true;
        bool allFinished = true;
        for (auto &veh : localVehicles_) {
            if (veh.addedToSumo_ && !veh.hasFinished()) {
                if (!veh.hasLeftIntersectionAndEnterArrivalLane()) {
                    allPassed = false;
                }
                if (!veh.hasFinished()) {
                    allFinished = false;
                }
            }
        }
        if (allPassed && !foundEvacuationTime) {
            if (verbose) {
                std::cout << "The intersection is evacuated at time: " << i << " ms.\n";
            }
            evacuation_time_ = currentTime;
            foundEvacuationTime = true;
        }

        if (allPassed && allFinished) {
            updateStatistics();
            if (verbose) {
                std::cout << Color::green << "\n\n########    Summary of simulation:    ########\n" << Color::def;
                for (auto &veh : localVehicles_) {
                    veh.printSummary();
                }
                printSummary();
            }
            break;
        }
    }
    Simulation::close();
}

void SumoSimulator::printTargetSimResults(std::string schedule_method) {
    std::cout << "=========================================\n";
    std::cout << schedule_method << " schedule:\n";
    if (schedule_method == "mdbfs") {
        auto &target_tree = mdbfst_;
        for (auto &node : target_tree.nodes_)
            node->printDetail();
    }
    else if (schedule_method == "mddfs") {
        auto &target_tree = mddfst_;
        for (auto &node : target_tree.nodes_)
            node->printDetail();
    }
    else if (schedule_method == "dfs") {
        auto &target_tree = modified_dfst_;
        for (auto &node : target_tree.nodes_)
            node->printDetail();
    }
    else if (schedule_method == "bfs") {
        auto &target_tree = bfst_;
        for (auto &node : target_tree.nodes_)
            node->printDetail();
    }
    else { // default is dynamic lane assignment scheduler
        auto &target_tree = dynamic_lane_tree_;
        for (auto &node : target_tree.nodes_)
            node->printDetail();
    }
    std::cout << "=========================================\n";
}

void SumoSimulator::updateStatistics() {
    averageWaitingTime_ = 0;
    averageTimeDelay_ = 0;
    maxWaitingTime_ = 0;
    maxTimeDelay_ = 0;
    totalFuelComsumed_ = 0;
    averageFuelComsumed_ = 0;
    for (auto &veh : localVehicles_) {
        if (veh.waitingTime_ > 0) {
            averageWaitingTime_ += veh.waitingTime_;
            if (veh.waitingTime_ > maxWaitingTime_)
                maxWaitingTime_ = veh.waitingTime_;
        }
        if (veh.timeDelay_ > 0) {
            averageTimeDelay_ += veh.timeDelay_;
            if (veh.timeDelay_ > maxTimeDelay_)
                maxTimeDelay_ = veh.timeDelay_;
        }
        if (veh.fuelConsumed_ > 0) {
            totalFuelComsumed_ += veh.fuelConsumed_ * 0.001; // change unit to gram
        }
    }

    averageWaitingTime_ = averageWaitingTime_ / localVehicles_.size();
    averageTimeDelay_ = averageTimeDelay_ / localVehicles_.size();
    averageFuelComsumed_ = totalFuelComsumed_ / localVehicles_.size();
}

void SumoSimulator::printSummary() {
    std::cout << Color::blue << "====statistics of simulation====\n"
        << "Scheduling method: " << schedule_method_ << "\n" << Color::def
        << "Intersection evacuation time: " << evacuation_time_ << " s\n"
        << "Waiting (stop) time: Max " << maxWaitingTime_ << " s, Ave " << averageWaitingTime_ << " s\n"
        << "Trip delay: Max " << maxTimeDelay_ << " s, Ave " << averageTimeDelay_ << " s\n";
    printFuelConsumptionSummary();
}

void SumoSimulator::printFuelConsumptionSummary() {
    totalFuelComsumed_ = 0;
    averageFuelComsumed_ = 0;
    for (auto &veh : localVehicles_) {
        if (veh.fuelConsumed_ > 0) {
            totalFuelComsumed_ += veh.fuelConsumed_ * 0.001; // change unit to gram
        }
    }
    averageFuelComsumed_ = totalFuelComsumed_ / localVehicles_.size();
    std::cout << "==========================\n";
    std::cout << "Sumo simulation at time: " << Simulation::getTime() << " s\n";
    std::cout << "Average fuel consumed is : " << averageFuelComsumed_ << " g\n";
    std::cout << "Total fuel consumed is : " << totalFuelComsumed_ << " g\n";
}

} // namespace intersection_management 