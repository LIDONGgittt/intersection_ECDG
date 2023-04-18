#ifndef INTERSECTION_MANAGEMENT_SUMO_SIMULATOR_H_
#define INTERSECTION_MANAGEMENT_SUMO_SIMULATOR_H_

#include <string>
#include <vector>
#include <libsumo/libtraci.h>

#include "sumo_sim_utility.h"
#include "parameters.h"
#include "spanning_tree.h"
#include "cdg_conflict_spanning_tree.h"

using namespace libtraci;

namespace intersection_management {
class SumoSimulator {
public:
    SumoSimulator(): SumoSimulator(0.001, 400) {}
    SumoSimulator(double step_length, double total_time): step_length_(step_length), total_time_(total_time) {
        sumo_cmd_ = {"sumo-gui", "-c", PROJECT_DIR + "/configs/sumo_intersection/intersection_unregulated.sumocfg"}; // {"--step-length", std::to_string(step_length_)};
        // offset that vehicle depart at 0m
        kTimeWindowOffset_ = 10.610;
        stopSimAfterClearanceFlag_ = true;
        travel_time_choice_ = {6.0, 6.5, 7.0};
    }

    void addVehicles(std::string schedule_method = "dynamic_lane");
    void startSimulation();
    void simulateOneRandomCase(int num_nodes, std::string schedule_method, bool verbose = false, int seed = -1);
    void generateSchedulingResults(int num_nodes, std::string schedule_method, bool verbose = false, int seed = -1);
    inline void setDynamicLaneResult(SpanningTree result_tree) { result_tree_ = result_tree; }
    inline void setDFSResult(CDGConflictSpanningTree modified_dfst) { modified_dfst_ = modified_dfst; }
    inline void setBFSResult(CDGConflictSpanningTree bfst) { bfst_ = bfst; }
    inline void setMDBFSResult(CDGConflictSpanningTree mdbfst) { mdbfst_ = mdbfst; }
    inline void setMDBFSResult(double global_optimal) { global_optimal_ = global_optimal; }
    inline void setSumoCmd(std::vector<std::string> sumo_cmd) { sumo_cmd_ = sumo_cmd; }
    inline void setTravelTimeChoice(std::vector<double> travel_time_choice) { travel_time_choice_ = travel_time_choice; }
    inline void setTimeWindowOffset(double kTimeWindowOffset) { kTimeWindowOffset_ = kTimeWindowOffset; }
    inline void setStopAtClearanceFlag(bool stopSimAfterClearanceFlag = true) { stopSimAfterClearanceFlag_ = stopSimAfterClearanceFlag; }
    inline void setStepLength(double step_length) { step_length_ = step_length; }

    SpanningTree result_tree_;
    CDGConflictSpanningTree modified_dfst_;
    CDGConflictSpanningTree bfst_;
    CDGConflictSpanningTree mdbfst_;
    double global_optimal_;

    std::vector<std::string> sumo_cmd_;
    // offset that vehicle depart at 0m
    double kTimeWindowOffset_ = 10.610;
    bool stopSimAfterClearanceFlag_ = true;
    std::vector<double> travel_time_choice_ = {6.0, 6.5, 7.0};
    std::vector<LocalVehicle> localVehicles_;
    double step_length_;
    double total_time_;
};
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_SUMO_SIMULATOR_H_