#ifndef INTERSECTION_MANAGEMENT_LOCALVEHICLE_H_
#define INTERSECTION_MANAGEMENT_LOCALVEHICLE_H_

#include <string>
#include <vector>
#include <libsumo/libtraci.h>

using namespace libtraci;

namespace intersection_management {
class LocalVehicle {
public:
    LocalVehicle():LocalVehicle("dummy_vehID", "dummy_routeID") {}
    LocalVehicle(std::string vehID, std::string routeID, std::string typeID = "DEFAULT_VEHTYPE", std::string depart = "now",
                 std::string departLaneID = "first", std::string departPos = "base", std::string departSpeed = "0",
                 std::string arrivalLaneID = "current", std::string arrivalPos = "max", std::string arrivalSpeed = "current");

    bool hasPassedIntersectionStopLine();
    bool hasLeftIntersectionAndEnterArrivalLane();
    bool hasFinished();

    std::string vehID_;
    std::string routeID_;
    std::string typeID_;
    std::string depart_;
    std::string departLaneID_;
    std::string departPos_;
    std::string departSpeed_;
    std::string arrivalLaneID_;
    std::string arrivalPos_;
    std::string arrivalSpeed_;

    std::string departEdgeID_;
    std::string arrivalEdgeID_;
    int departLaneIDNum_;
    int arrivalLaneIDNum_;
    bool finished_;
    libsumo::TraCIColor color_;

    std::vector<double> timewindow_;
};
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_LOCALVEHICLE_H_