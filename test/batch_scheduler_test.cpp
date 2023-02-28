#include <gtest/gtest.h>

#include "scheduler.h"
#include "conflict_directed_graph.h"
#include "cdg_scheduler.h"

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <chrono>

using namespace intersection_management;
std::vector<double> BatchTest(int num_nodes, int seed = 0) {
    Intersection intersection;
    ConflictDirectedGraph cdg;
    Scheduler scheduler;
    CDGScheduler scheduler_dfs;
    CDGScheduler scheduler_bfs;
    CDGScheduler scheduler_mdbfs;
    CDGScheduler scheduler_bruteforce;


    intersection.setSeed(seed);
    intersection.AddRandomVehicleNodes(5);
    intersection.AddIntersectionUtilitiesFromGeometric();
    intersection.AssignCriticalResourcesToNodes();
    intersection.AssignRoutesToNodes();
    intersection.AssignEdgesWithSafetyOffsetToNodes();

    cdg.GenerateGraphFromIntersection(intersection);

    auto result_tree = scheduler.ScheduleWithDynamicLaneAssignment(intersection);
    auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
    auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
    auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
    auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
    auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
    double global_optimal = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
    std::cout << "dynamin lane assignment: " << result_tree.depth_ << "\n";
    std::cout << "modified dfs: " << modified_dfst.edge_weighted_depth_ << "\n";
    std::cout << "edge_weighted bfs: " << bfst.edge_weighted_depth_ << "\n";
    std::cout << "multi_weighted bfs: " << mdbfst.edge_node_weighted_depth_ << "\n";
    std::cout << "global_optimal: " << global_optimal << "\n";
}

TEST(BatchSchedulerTest, CompareDynamicLaneAssignments) {
    
}