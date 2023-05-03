#ifndef INTERSECTION_MANAGEMENT_SUMO_SIMULATOR_H_
#define INTERSECTION_MANAGEMENT_SUMO_SIMULATOR_H_

#include <string>
#include <vector>
#include <libsumo/libtraci.h>

#include "parameters.h"
#include "sumo_sim_utility.h"
#include "spanning_tree.h"
#include "cdg_conflict_spanning_tree.h"

using namespace libtraci;

namespace intersection_management {
class SumoSimulator {
public:
    SumoSimulator(): SumoSimulator(0.01, 10000) {}
    SumoSimulator(double step_length, double total_time = 10000): step_length_(step_length), max_sim_time_(total_time) {
        schedule_method_ = "default";
        arrival_interval_avg_ = 2.0;
        seed_ = -1;
        localParam_ = geometryParamVec[0];

        sumo_cmd_ = {"sumo-gui", "-c", PROJECT_DIR + localParam_.sumo_config_file, "--collision.action", "warn", "--step-log.period", "100000"};
        // sumo_cmd_ = {"sumo", "-c", PROJECT_DIR + localParam_.sumo_config_file};

        kTimeWindowOffset_ = localParam_.kTimeWindowOffset;
        travel_time_choice_ = localParam_.travel_time_choice;
        scheduler_method_list_ = {"dynamic_lane", "dfs", "bfs", "mdbfs", "global_optimal"};
        standard_depth_ = {0, 0, 0, 0, 0};

        evacuation_time_ = 0;
        totalFuelComsumed_ = 0;
        averageFuelComsumed_ = 0;
        maxWaitingTime_ = 0;
        averageWaitingTime_ = 0;
        maxTimeDelay_ = 0;
        averageTimeDelay_ = 0;
    }

    void setSimulateOneMethod(int num_nodes, std::string schedule_method, double arrival_interval_avg = 2.0,
                              int geometryID = 0, bool verbose = false, int seed = -1);
    void generateSchedulingResults(int num_nodes, bool verbose = false, int seed = -1);
    void addVehicles(std::string schedule_method = "dynamic_lane");
    void startSimulation(bool verbose = true);

    inline void setSumoGUI(bool activate = true) { if (activate) sumo_cmd_[0] = "sumo-gui"; else sumo_cmd_[0] = "sumo"; }
    inline void setDynamicLaneResult(SpanningTree result_tree) { result_tree_ = result_tree; }
    inline void setDFSResult(CDGConflictSpanningTree modified_dfst) { modified_dfst_ = modified_dfst; }
    inline void setBFSResult(CDGConflictSpanningTree bfst) { bfst_ = bfst; }
    inline void setMDBFSResult(CDGConflictSpanningTree mdbfst) { mdbfst_ = mdbfst; }
    inline void setMDBFSResult(double global_optimal) { global_optimal_ = global_optimal; }
    inline void setTravelTimeChoice(std::vector<double> travel_time_choice) { travel_time_choice_ = travel_time_choice; }
    inline void setTimeWindowOffset(double kTimeWindowOffset) { kTimeWindowOffset_ = kTimeWindowOffset; }
    inline void setStepLength(double step_length) { step_length_ = step_length; }

    void printTargetSimResults(std::string schedule_method = "dynamic_lane");
    void updateStatistics();
    void printSummary();
    void printFuelConsumptionSummary();

    // schedule results
    SpanningTree result_tree_;
    CDGConflictSpanningTree modified_dfst_;
    CDGConflictSpanningTree bfst_;
    CDGConflictSpanningTree mdbfst_;
    double global_optimal_;
    std::vector<std::string> scheduler_method_list_;
    std::vector<double> standard_depth_;

    // simulation configs
    std::string schedule_method_;
    double arrival_interval_avg_;
    int seed_;
    Parameters localParam_;

    std::vector<std::string> sumo_cmd_;
    double kTimeWindowOffset_;
    std::vector<double> travel_time_choice_;
    std::vector<LocalVehicle> localVehicles_;
    double step_length_;
    double max_sim_time_;

    // statistics
    double evacuation_time_;
    double totalFuelComsumed_;
    double averageFuelComsumed_;
    double maxWaitingTime_;
    double averageWaitingTime_;
    double maxTimeDelay_;
    double averageTimeDelay_;
};
} // namespace intersection_management
#endif // INTERSECTION_MANAGEMENT_SUMO_SIMULATOR_H_