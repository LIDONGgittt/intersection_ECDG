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

const std::vector<Parameters> geometryParamVec = {
    Parameters(4, {3, 3, 3, 3}, {3, 3, 3, 3}, "/configs/sumo_intersection0/intersection_unregulated.sumocfg"),
    Parameters(4, {1, 2, 1, 2}, {1, 2, 1, 2}, "/configs/sumo_intersection1/intersection_unregulated.sumocfg")};

int intersectionLaneIdToSumoLaneId(int leg_id, int lane_id, Parameters local_param, std::string type) {
    if (type == "in")
        return lane_id;
    if (type == "out")
        return local_param.num_lanes_out_vec[leg_id] - 1 - lane_id;
    return -1;
}

// void printFuelConsumption(std::vector<LocalVehicle> &localVehicles, double currentTime) {
//     double totalFuelComsumed = 0;
//     double averageFuelComsumed = 0;
//     for (auto &veh : localVehicles) {
//         // std::cout << veh.fuelConsumed_ << std::endl;
//         if (veh.fuelConsumed_ > 0) {
//             totalFuelComsumed += veh.fuelConsumed_ * 0.001;
//         }
//     }
//     averageFuelComsumed = totalFuelComsumed / localVehicles.size();
//     std::cout << "==========================\n";
//     std::cout << "Sumo simulation at time: " << currentTime << " s\n";
//     std::cout << "Average fuel consumed is : " << averageFuelComsumed << " g\n";
//     std::cout << "Total fuel consumed is : " << totalFuelComsumed << " g\n";
// }

void LocalVehicle::printSummary() {
    std::cout << "Vehicle " << vehID_ << ": "
        << "Fuel Consumed " << fuelConsumed_ * 0.001 << " g, "
        << "Stop Time " << waitingTime_ << " s, "
        << "Travel Time " << actualClearingTime_ - arrival_time_ << " s, "
        << "Time Delay " << timeDelay_ << "s.\n";
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
    waitingTime_ = 0.0;
    actualClearingTime_ = 0.0;
    timeDelay_ = 0.0;

    // scheduled results, will be used to add constraints in the simulation
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
            actualClearingTime_ = Simulation::getTime();
            timeNeededForTheTrip_ = Vehicle::getDistance(vehID_) / libtraci::Lane::getMaxSpeed(departEdgeID_ + "_" + departLaneID_);
            // std::cout << "Vehicle " << vehID_ << " has driven a distance of: " << Vehicle::getDistance(vehID_)
            //     << ", with max speed of: " << libtraci::Lane::getMaxSpeed(departEdgeID_ + "_" + departLaneID_) << "\n";
            timeDelay_ = actualClearingTime_ - arrival_time_ - timeNeededForTheTrip_;
            printSummary();
        }
    }
    return finished_;
}
}