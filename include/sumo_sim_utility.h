#ifndef INTERSECTION_MANAGEMENT_SUMO_SIM_UTILITY_
#define INTERSECTION_MANAGEMENT_SUMO_SIM_UTILITY_

#include <string>
#include <vector>
#include <libsumo/libtraci.h>
#include "parameters.h"

using namespace libtraci;

namespace intersection_management {

static const std::vector<Parameters> geometryParamVec = {
    Parameters(4, {3, 3, 3, 3}, {3, 3, 3, 3}),
    Parameters(4, {1, 2, 1, 2}, {1, 2, 1, 2})};

static const std::vector<libsumo::TraCIColor> sumoColorVec = {
    libsumo::TraCIColor(55, 255, 0), libsumo::TraCIColor(0, 255, 255),
    libsumo::TraCIColor(255, 155, 0), libsumo::TraCIColor(200, 15, 15),
    libsumo::TraCIColor(255, 0, 255), libsumo::TraCIColor(100, 100, 255)};

class LocalVehicle {
public:
    LocalVehicle():LocalVehicle("dummy_vehID", "dummy_routeID") {}
    LocalVehicle(std::string vehID, std::string routeID, std::string typeID = "DEFAULT_VEHTYPE", std::string depart = "now",
                 std::string departLaneID = "first", std::string departPos = "base", std::string departSpeed = "0",
                 std::string arrivalLaneID = "current", std::string arrivalPos = "max", std::string arrivalSpeed = "current");

    bool hasPassedIntersectionStopLine();
    bool hasLeftIntersectionAndEnterArrivalLane();
    bool hasFinished();
    void printSummary();

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
    bool addedToSumo_;
    double fuelConsumed_;
    double waitingTime_;
    double actualClearingTime_;
    double timeDelay_;
    double timeNeededForTheTrip_;

    // scheduled results, will be used to add constraints in the simulation
    double arrival_time_;
    std::vector<double> timewindow_;
};

int intersectionLaneIdToSumoLaneId(int leg_id, int lane_id, std::string type = "in");

// void sumoSimulationOneCase(int num_nodes, std::string schedule_method, bool verbose = false, int seed = -1);

void printFuelConsumption(std::vector<LocalVehicle> &localVehicles, double currentTime);
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_SUMO_SIM_UTILITY_