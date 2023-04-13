#include "scheduler.h"
#include "conflict_directed_graph.h"
#include "cdg_scheduler.h"
#include "time_profiler/time_profiler.h"
#include "parameters.h"
#include "sumo_sim_utility.h"


namespace intersection_management {
extern Parameters param;
}

using namespace intersection_management;

int main() {
    param.num_lanes_in_vec = {3, 3, 3, 3};
    param.num_lanes_out_vec = {3, 3, 3, 3};

    std::vector<double> depth;
    Intersection intersection;
    ConflictDirectedGraph cdg;
    Scheduler scheduler;
    CDGScheduler scheduler_dfs;
    CDGScheduler scheduler_bfs;
    CDGScheduler scheduler_mdbfs;
    CDGScheduler scheduler_bruteforce;


    bool verbose = true;
    int seed = 1;

    PROFILER_HOOK();
    intersection.setSeed(seed);
    intersection.AddRandomVehicleNodesWithTravelTime(20, {6.0, 6.5, 7.0}, verbose);
    intersection.AssignCriticalResourcesToNodes();
    intersection.AssignRoutesToNodes();
    intersection.AssignEdgesWithSafetyOffsetToNodes();

    PROFILER_HOOK();
    cdg.GenerateGraphFromIntersection(intersection);

    PROFILER_HOOK();
    auto result_tree = scheduler.ScheduleWithDynamicLaneAssignment(intersection);

    PROFILER_HOOK();
    auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);

    PROFILER_HOOK();
    auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);

    PROFILER_HOOK();
    auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);

    PROFILER_HOOK();
    double global_optimal = 0;
    if (cdg.num_nodes_ <= 16) { // only calculate global_optimal for small number of nodes
        auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
        auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
        global_optimal = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
    }

    PROFILER_HOOK();
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
    depth = std::vector<double>{result_tree.depth_, modified_dfst.edge_weighted_depth_, bfst.edge_weighted_depth_,
        mdbfst.edge_node_weighted_depth_, global_optimal};
    // return depth;

    std::vector<std::string> SUMO_CMD({"sumo-gui", "-c", PROJECT_DIR + "/configs/sumo_intersection/intersection_unregulated.sumocfg"});
    std::vector<LocalVehicle> localVehicles;

    // offset that vehicle depart at 0m
    double kTimeWindowOffset = 10.610;
    for (int i = 1; i < result_tree.nodes_.size(); i++) {
        auto node = result_tree.nodes_[i];
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
    // localVehicles.resize(1);


    bool stopSimAfterClearanceFlag = true;
    double currentTime = 0;
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
                if (!veh.hasPassedIntersectionStopLine()) {
                    Vehicle::changeLane(veh.vehID_, veh.departLaneIDNum_, 0.1);
                }
                else {
                    Vehicle::changeLane(veh.vehID_, veh.arrivalLaneIDNum_, 0.1);
                }
            }
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
            getchar();
            stopSimAfterClearanceFlag = false;
        }
    }
    Simulation::close();

}