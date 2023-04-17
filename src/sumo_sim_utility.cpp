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

int intersectionLaneIdToSumoLaneId(int leg_id, int lane_id, std::string type) {
    if (type == "in")
        return lane_id;
    if (type == "out")
        return param.num_lanes_out_vec[leg_id] - 1 - lane_id;
    return -1;
}

void sumoSimulationOneCase(int num_nodes, std::string schedule_method, bool verbose, int seed) {
    param.num_lanes_in_vec = {3, 3, 3, 3};
    param.num_lanes_out_vec = {3, 3, 3, 3};
    std::vector<double> travel_time_choice = {6.0, 6.5, 7.0};
    std::vector<std::string> SUMO_CMD({"sumo-gui", "-c", PROJECT_DIR + "/configs/sumo_intersection/intersection_unregulated.sumocfg"});
    // offset that vehicle depart at 0m
    double kTimeWindowOffset = 10.610;
    bool stopSimAfterClearanceFlag = true;

    Intersection intersection;
    ConflictDirectedGraph cdg;
    Scheduler scheduler;
    CDGScheduler scheduler_dfs;
    CDGScheduler scheduler_bfs;
    CDGScheduler scheduler_mdbfs;
    CDGScheduler scheduler_bruteforce;

    intersection.setSeed(seed);
    intersection.AddRandomVehicleNodesWithTravelTime(num_nodes, travel_time_choice, verbose);
    intersection.AssignCriticalResourcesToNodes();
    intersection.AssignRoutesToNodes();
    intersection.AssignEdgesWithSafetyOffsetToNodes();

    cdg.GenerateGraphFromIntersection(intersection);

    auto result_tree = scheduler.ScheduleWithDynamicLaneAssignment(intersection);
    auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
    auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
    auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
    double global_optimal = 0;
    if (cdg.num_nodes_ <= 16) { // only calculate global_optimal for small number of nodes
        auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
        auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
        global_optimal = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
    }

    if (verbose) {
        std::cout << "seed: " << seed << "\n";
        std::cout << "dynamin lane assignment: " << result_tree.depth_ << "\n";
        std::cout << "modified dfs: " << modified_dfst.edge_weighted_depth_ << "\n";
        std::cout << "edge_weighted bfs: " << bfst.edge_weighted_depth_ << "\n";
        std::cout << "multi_weighted bfs: " << mdbfst.edge_node_weighted_depth_ << "\n";
        std::cout << "global_optimal: " << global_optimal << "\n";
        std::cout << "=========================================\n";
        std::cout << "Dynamic lane schedule:\n";
        for (auto &node : result_tree.nodes_)
            node->printDetail();
        std::cout << "=========================================\n";
    }

    std::vector<std::shared_ptr<Node>> scheduled_vehicles;
    if (schedule_method == "mdbfs") {
        auto &target_tree = mdbfst;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            auto &node = target_tree.nodes_[i];
            node->time_window_[1] = node->edge_node_weighted_depth_;
            node->time_window_[0] = node->edge_node_weighted_depth_ - node->estimate_travel_time_;
            node->possible_lane_id_ = std::vector<int>({node->out_lane_id_});
            scheduled_vehicles.push_back(node);
        }
    }
    else if (schedule_method == "dfs") {
        auto &target_tree = modified_dfst;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            auto &node = target_tree.nodes_[i];
            node->time_window_[1] = node->edge_weighted_depth_ * travel_time_choice[2];
            node->time_window_[0] = (node->edge_weighted_depth_ - 1) * travel_time_choice[2];
            node->possible_lane_id_ = std::vector<int>({node->out_lane_id_});
            scheduled_vehicles.push_back(node);
        }
    }
    else if (schedule_method == "bfs") {
        auto &target_tree = bfst;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            auto &node = target_tree.nodes_[i];
            node->time_window_[1] = node->edge_weighted_depth_ * travel_time_choice[2];
            node->time_window_[0] = (node->edge_weighted_depth_ - 1) * travel_time_choice[2];
            node->possible_lane_id_ = std::vector<int>({node->out_lane_id_});
            scheduled_vehicles.push_back(node);
        }
    }
    else { // default is dynamic lane assignment scheduler
        auto &target_tree = result_tree;
        for (int i = 1; i < target_tree.nodes_.size(); i++) {
            scheduled_vehicles.push_back(target_tree.nodes_[i]);
        }
    }

    std::vector<LocalVehicle> localVehicles;
    for (int i = 0; i < scheduled_vehicles.size(); i++) {
        auto &node = scheduled_vehicles[i];
        std::string routeId = "r";
        routeId.push_back('0' + node->in_leg_id_);
        routeId.push_back('0' + node->out_leg_id_);
        std::string departLaneID = std::to_string(intersectionLaneIdToSumoLaneId(node->in_leg_id_, node->in_lane_id_, "in"));
        std::string arrivalLaneID = std::to_string(intersectionLaneIdToSumoLaneId(node->out_leg_id_, node->possible_lane_id_[0], "out"));
        localVehicles.push_back(LocalVehicle(std::to_string(node->id_), routeId, "Vtype1", "now", departLaneID, "0", "0", arrivalLaneID));

        localVehicles.back().color_ = sumoColorVec[i % sumoColorVec.size()];
        localVehicles.back().arrival_time_ = node->estimate_arrival_time_;
        localVehicles.back().timewindow_[0] = kTimeWindowOffset + node->time_window_[0];
        localVehicles.back().timewindow_[1] = kTimeWindowOffset + node->time_window_[1];
    }

    Simulation::start(SUMO_CMD);

    double currentTime = 0;
    double nextPrintTime = 10.0;
    for (int i = 0; i < 400000; i++) {
        currentTime = i * 0.001;
        for (auto &veh : localVehicles) {
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

        for (auto &veh : localVehicles) {
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
                veh.fuelConsumed_ += ((Vehicle::getFuelConsumption(veh.vehID_)>0)?Vehicle::getFuelConsumption(veh.vehID_):0) * 0.001;
            }
        }

        if (currentTime >= nextPrintTime) {
            nextPrintTime += 10;
            double averageFuelComsumed = 0;
            for (auto &veh : localVehicles) {
                // std::cout << veh.fuelConsumed_ << std::endl;
                if (veh.fuelConsumed_ > 0) {
                    averageFuelComsumed += veh.fuelConsumed_;
                }
            }
            averageFuelComsumed /= localVehicles.size();
            std::cout << "==========================";
            std::cout << "Sumo status as time: " << currentTime << std::endl;
            std::cout << "Average fuel consumed is : " << averageFuelComsumed << " mg\n";
        }

        bool allPassed = true;
        for (auto &veh : localVehicles) {
            if (veh.addedToSumo_ && !veh.hasFinished()) {
                if (!veh.hasLeftIntersectionAndEnterArrivalLane()) {
                    allPassed = false;
                }
            }
        }
        if (allPassed && stopSimAfterClearanceFlag) {
            std::cout << "The intersection is evacuated at time: " << i << " ms.\n";

            double averageFuelComsumed = 0;
            for (auto &veh : localVehicles) {
                // std::cout << veh.fuelConsumed_ << std::endl;
                if (veh.fuelConsumed_ > 0) {
                    averageFuelComsumed += veh.fuelConsumed_;
                }
            }
            averageFuelComsumed /= localVehicles.size();
            std::cout << "==========================";
            std::cout << "Sumo status as time: " << currentTime << std::endl;
            std::cout << "Average fuel consumed is : " << averageFuelComsumed << " mg\n";

            getchar();
            stopSimAfterClearanceFlag = false;
        }
    }
    Simulation::close();
}

LocalVehicle::LocalVehicle(std::string vehID, std::string routeID, std::string typeID, std::string depart,
                           std::string departLaneID, std::string departPos, std::string departSpeed,
                           std::string arrivalLaneID, std::string arrivalPos, std::string arrivalSpeed) {
    vehID_ = vehID;
    routeID_ = routeID;
    typeID_ = typeID;
    depart_ = depart;
    departLaneID_ = departLaneID;
    departPos_ = departPos;
    departSpeed_ = departSpeed;
    arrivalLaneID_ = arrivalLaneID;
    arrivalPos_ = arrivalPos;
    arrivalSpeed_ = arrivalSpeed;

    departEdgeID_ = routeID.substr(1, 1) + "tc";
    arrivalEdgeID_ = "ct" + routeID.substr(2, 1);
    color_ = libsumo::TraCIColor(255, 0, 0);
    departLaneIDNum_ = std::stoi(departLaneID_);
    arrivalLaneIDNum_ = std::stoi(arrivalLaneID_);
    finished_ = false;
    addedToSumo_ = false;
    fuelConsumed_ = 0.0;

    arrival_time_ = -1;
    timewindow_ = std::vector<double>{-1, -1};
}

bool LocalVehicle::hasPassedIntersectionStopLine() {
    std::string road = Vehicle::getRoadID(vehID_);
    if (road.empty())
        return false;
    return road != departEdgeID_;
}
bool LocalVehicle::hasLeftIntersectionAndEnterArrivalLane() {
    // std::cout<<vehID_ << " at lane: " << Vehicle::getRoadID(vehID_) << ", has enter arrival lane? : " << (Vehicle::getRoadID(vehID_).find("ct") != std::string::npos) << std::endl;
    std::string road = Vehicle::getRoadID(vehID_);
    if (road.empty())
        return false;
    if (road != arrivalEdgeID_)
        return false;
    if (Vehicle::getLanePosition(vehID_) < Vehicle::getLength(vehID_))
        return false;
    return true;
}
bool LocalVehicle::hasFinished() {
    if (!finished_) {
        // std::cout << vehID_ << " position " << Vehicle::getLanePosition(vehID_) << std::endl;
        // std::cout << "is finished? " << (hasPassedIntersectionStopLine() && Vehicle::getLanePosition(vehID_) > 80) << std::endl;
        if (hasPassedIntersectionStopLine() && Vehicle::getLanePosition(vehID_) > 80) {
            // std::cout<<"set " << vehID_ << " finished\n";
            finished_ = true;
            
        }
    }
    return finished_;
}
}