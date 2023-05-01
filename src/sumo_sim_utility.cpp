#include "parameters.h"
#include "sumo_sim_utility.h"

#include <iostream>
#include <libsumo/libtraci.h>

#include "scheduler.h"
#include "conflict_directed_graph.h"
#include "cdg_scheduler.h"
#include "time_profiler/time_profiler.h"

using namespace libtraci;

namespace intersection_management {

int intersectionLaneIdToSumoLaneId(int leg_id, int lane_id, Parameters &local_param, std::string type) {
    if (type == "in")
        return lane_id;
    if (type == "out")
        return local_param.num_lanes_out_vec[leg_id] - 1 - lane_id;
    return -1;
}

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
    passStopLineCnt_ = 0;
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
        if (hasPassedIntersectionStopLine() && Vehicle::getLanePosition(vehID_) > 80) {
            finished_ = true;
            actualClearingTime_ = Simulation::getTime();
            timeNeededForTheTrip_ = Vehicle::getDistance(vehID_) / libtraci::Lane::getMaxSpeed(departEdgeID_ + "_" + departLaneID_);
            timeDelay_ = actualClearingTime_ - arrival_time_ - timeNeededForTheTrip_;
            printSummary();
        }
    }
    return finished_;
}
}