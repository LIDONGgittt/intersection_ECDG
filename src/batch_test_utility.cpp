#include "batch_test_utility.h"

#include <csignal>

#include "scheduler.h"
#include "conflict_directed_graph.h"
#include "cdg_scheduler.h"
#include "time_profiler/time_profiler.h"
#include "parameters.h"

namespace intersection_management {

extern Parameters param;

std::vector<double> BatchTestOneCase(int num_nodes, bool verbose, int seed) {
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
    intersection.AddRandomVehicleNodes(num_nodes, verbose);
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
    }
    depth = std::vector<double>{result_tree.depth_, modified_dfst.edge_weighted_depth_, bfst.edge_weighted_depth_,
        mdbfst.edge_node_weighted_depth_, global_optimal};
    return depth;
}

void BatchTest(int num_nodes, int test_count, int print_interval, int starting_seed) {
    std::signal(SIGINT, SIGINT_signal_handler);
    std::vector<double> sum(5, 0);
    std::vector<long> better_count(5, 0);
    std::vector<long> better_dfs(5, 0);
    long total_test = 0;
    if (test_count < 0) test_count = INT32_MAX;
    int seed_increment = 0;
    if (starting_seed >= 0)
        seed_increment = 1;

    while (total_test < test_count) {
        auto depths = BatchTestOneCase(num_nodes, false, starting_seed);
        starting_seed += seed_increment;
        double coefficient;
        for (int i = 0; i < 5; i++) {
            coefficient = 1;
            if (i == 1 || i == 2)
                coefficient = param.travel_time_range[1];
            sum[i] += depths[i] * coefficient;
            if (depths[i] * coefficient <= depths[4])
                better_count[i]++;
            if (depths[i] * coefficient <= depths[1] * param.travel_time_range[1])
                better_dfs[i]++;
        }
        total_test++;

        if (total_test % print_interval == 0) {
            std::cout << "\n\n##################### Updated result: ##################### \n";
            std::cout << "Total tests: " << total_test << ", Total node num: " << num_nodes << ". \n";
            std::cout << "Average depths: dynamic_lanes, dfs, bfs, mwbfs, global_optimal.\n";
            for (int i = 0; i < 5; i++) {
                std::cout << sum[i] / total_test << ", ";
            }
            std::cout << "\n Better than Global Optimal ratio: \n";
            for (int i = 0; i < 5; i++) {
                std::cout << (double)better_count[i] / total_test << ", ";
            }
            std::cout << "\n Better than DFS ratio: \n";
            for (int i = 0; i < 5; i++) {
                std::cout << (double)better_dfs[i] / total_test << ", ";
            }
            std::cout << "\n";
        }
    }
}

void SIGINT_signal_handler(int signal) {
    // ::time_profiler::TimeProfiler::print_statistics();
    exit(signal);
}
} // namespace intersection_management