#include <gtest/gtest.h>
#include "scheduler.h"

using namespace intersection_management;

TEST(SchedulerTest, MultipleScheduler_v1) {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    Scheduler scheduler_dfs = Scheduler();
    Scheduler scheduler_bfs = Scheduler();
    Scheduler scheduler_mdbfs = Scheduler();

    // cdg.reset();
    unsigned int seed = 0;
    int max_node = 100;
    int total_test = 1000;
    int bfs_failure_count = 0;
    int bfs_better_count = 0;
    bool verbose_mode = true;

    for (int repeat = 0; repeat < total_test; repeat++) {
        seed = std::time(NULL);
        do {
            // cdg.GenerateRandomGraph(std::rand() % max_node + 1);
            cdg.GenerateRandomGraph(5);
        } while (!cdg.isFullyConnected());

        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);

        if (modified_dfst.edge_weighted_depth_ > bfst.edge_weighted_depth_) {
            bfs_better_count++;
        }
        else if (modified_dfst.edge_weighted_depth_ < bfst.edge_weighted_depth_) {
            bfs_failure_count++;
            if (verbose_mode) {
                std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << seed << " # # # # # # # # # # # # # #\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst.PrintTree(true);
                std::cout << "BFST:\n";
                bfst.PrintTree(true);
                std::cout << "MDBFST:\n";
                mdbfst.PrintTree(true);
            }
        }
    }

    std::cout << "BFST generate better results ratio:  " << bfs_better_count << " / " << total_test << "\n";
    std::cout << "BFST failed ratio:  " << bfs_failure_count << " / " << total_test << "\n";
}
