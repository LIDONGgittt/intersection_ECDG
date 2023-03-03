#include "batch_test_utility.h"
#include "time_profiler/time_profiler.h"

namespace intersection_management {
std::vector<double> BatchTest(int num_nodes, bool enable_precedence_offset, bool verbose, int seed) {
    PROFILER_HOOK();
    std::vector<double> depth;
    Intersection intersection;
    ConflictDirectedGraph cdg;
    Scheduler scheduler;
    CDGScheduler scheduler_dfs;
    CDGScheduler scheduler_bfs;
    CDGScheduler scheduler_mdbfs;
    CDGScheduler scheduler_bruteforce;

    PROFILER_HOOK();
    intersection.setSeed(seed);
    intersection.AddRandomVehicleNodes(num_nodes);
    intersection.AddIntersectionUtilitiesFromGeometric();
    intersection.AssignCriticalResourcesToNodes();
    intersection.AssignRoutesToNodes();
    intersection.AssignEdgesWithSafetyOffsetToNodes();

    PROFILER_HOOK();
    cdg.GenerateGraphFromIntersection(intersection);

    PROFILER_HOOK();
    scheduler.enable_optimized_precedence_offset_ = enable_precedence_offset;
    auto result_tree = scheduler.ScheduleWithDynamicLaneAssignment(intersection);

    PROFILER_HOOK();
    auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);

    PROFILER_HOOK();
    auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);

    PROFILER_HOOK();
    auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);

    PROFILER_HOOK();
    double global_optimal = 0;
    if (cdg.num_nodes_ <= 15) { // only calculate global_optimal for small number of nodes
        auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
        auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
        global_optimal = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
    }

    PROFILER_HOOK();
    if (verbose) {
        std::cout << "dynamin lane assignment: " << result_tree.depth_ << "\n";
        std::cout << "modified dfs: " << modified_dfst.edge_weighted_depth_ << "\n";
        std::cout << "edge_weighted bfs: " << bfst.edge_weighted_depth_ << "\n";
        std::cout << "multi_weighted bfs: " << mdbfst.edge_node_weighted_depth_ << "\n";
        std::cout << "global_optimal: " << global_optimal << "\n";
    }
    depth = std::vector<double>{result_tree.depth_, modified_dfst.edge_weighted_depth_, bfst.edge_weighted_depth_,
        mdbfst.edge_node_weighted_depth_, global_optimal};
    return depth;
}

} // namespace intersection_management