#include "sumo_local_vehicle.h"

#include <iostream>
#include <libsumo/libtraci.h>

using namespace libtraci;

namespace intersection_management {
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