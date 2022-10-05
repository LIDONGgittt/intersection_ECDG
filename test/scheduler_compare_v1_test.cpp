#include <gtest/gtest.h>
#include "scheduler.h"

using namespace intersection_management;

TEST(SchedulerTest, CompareMethods_v1) {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    Scheduler scheduler_dfs = Scheduler();
    Scheduler scheduler_bfs = Scheduler();
    Scheduler scheduler_mdbfs = Scheduler();
    Scheduler scheduler_bruteforce = Scheduler();

    // cdg generater parameters
    unsigned int seed = 0;
    int max_node = 1;
    double max_node_weight = 5.0;
    double min_node_weight = 2.0;
    // verbose flag
    bool verbose_mode_for_bfs = false;
    bool verbose_mode_for_mwbfs = false;
    bool verbose_mode_for_both_failure = false;
    // statistic variables
    int total_test = 0;
    int bfs_failure_count = 0;
    int bfs_better_count = 0;
    int mdbfs_better_count = 0;
    int mdbfs_failure_count = 0;
    bool mwbfs_failure_flag = false;
    bool mwbfs_better_flag = false;
    bool either_better_flag = false;
    int both_failure_count = 0;
    int both_better_count = 0;
    int either_better_count = 0;
    double depth_sum_dfst = 0.0;
    double depth_sum_bfst = 0.0;
    double depth_sum_mdbfs = 0.0;
    double depth_sum_optimal = 0.0;
    int mwbfs_get_optimal_count = 0;
    int mwbfs_bettter_than_optimal_count = 0;

    seed = std::time(NULL);
    // seed = 0;
    srand(seed);
    while (true) {
        total_test++;
        // srand(343);
        srand(total_test);
        do {
            cdg.GenerateRandomGraph(std::rand() % max_node + 5, seed, max_node_weight - min_node_weight + 1.0, 2.0, min_node_weight, 1.0, true);
            // cdg.GenerateRandomGraph(5, seed, max_node_weight - min_node_weight + 1.0, 2.0, min_node_weight, 1.0, true);
        } while (!cdg.isFullyConnected());

        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
        auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
        auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
        double global_optimal = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);

        depth_sum_dfst += modified_dfst.edge_weighted_depth_;
        depth_sum_bfst += bfst.edge_weighted_depth_;
        depth_sum_mdbfs += mdbfst.edge_node_weighted_depth_;
        depth_sum_optimal += global_optimal;

        mwbfs_better_flag = false;
        mwbfs_failure_flag = false;
        either_better_flag = false;
        if (mdbfst.edge_node_weighted_depth_ < (max_node_weight) * (modified_dfst.edge_weighted_depth_) && mdbfst.edge_node_weighted_depth_ < (max_node_weight) * (bfst.edge_weighted_depth_)) {
            mdbfs_better_count++;
            mwbfs_better_flag = true;
            either_better_flag = true;
        }
        else if (mdbfst.edge_node_weighted_depth_ > (max_node_weight) * (modified_dfst.edge_weighted_depth_) || mdbfst.edge_node_weighted_depth_ > (max_node_weight) * (bfst.edge_weighted_depth_)) {
            mdbfs_failure_count++;
            mwbfs_failure_flag = true;
            if (verbose_mode_for_mwbfs) {
                std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                std::cout << "!!!!!!   Multi depth MWBFS is not the best one   !!!!!!\n";
            }
        }

        if (modified_dfst.edge_weighted_depth_ > bfst.edge_weighted_depth_) {
            bfs_better_count++;
            either_better_flag = true;
            if (mwbfs_better_flag) {
                both_better_count++;
            }
        }
        else if (modified_dfst.edge_weighted_depth_ < bfst.edge_weighted_depth_) {
            bfs_failure_count++;
            if (mwbfs_failure_flag) {
                both_failure_count++;
                if (verbose_mode_for_both_failure) {
                    std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                    std::cout << "!!!!!!   Both BFS and MWBFS methods fail to win DFS   !!!!!!\n";
                }
            }
            if (verbose_mode_for_bfs) {
                std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << seed << " # # # # # # # # # # # # # #\n";
                std::cout << "!!!!!!   BFS lost to DFS   !!!!!!\n";
            }
        }

        if (mdbfst.edge_node_weighted_depth_ < global_optimal) {
            mwbfs_bettter_than_optimal_count++;
            std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << total_test << " # # # # # # # # # # # # # #\n";
            std::cout << "Max node nums: " << max_node << ", Max node weight: " << max_node_weight << ", Min node weight: " << min_node_weight << "\n";
            cdg.PrintGraph();
            std::cout << "Modified DFST:\n";
            modified_dfst.PrintTree(true);
            std::cout << "BFST:\n";
            bfst.PrintTree(true);
            std::cout << "MDBFST:\n";
            mdbfst.PrintTree(true);
            std::cout << "Global optimal evacuation time is: " << global_optimal << "\n";
            std::cout << "The optimal order is : \n";
            scheduler_bruteforce.printOrder(best_order);
            std::cout << "The optimal schedule is : \n";
            scheduler_bruteforce.printDepthVector(depth_vector);
            break;
        }
        else if (mdbfst.edge_node_weighted_depth_ == global_optimal) {
            mwbfs_get_optimal_count++;
        }

        if (verbose_mode_for_bfs || verbose_mode_for_both_failure || verbose_mode_for_mwbfs) {
            std::cout << "Max node nums: " << max_node << ", Max node weight: " << max_node_weight << ", Min node weight: " << min_node_weight << "\n";
            cdg.PrintGraph();
            std::cout << "Modified DFST:\n";
            modified_dfst.PrintTree(true);
            std::cout << "BFST:\n";
            bfst.PrintTree(true);
            std::cout << "MDBFST:\n";
            mdbfst.PrintTree(true);
            std::cout << "Global optimal evacuation time is: " << global_optimal << "\n";
            std::cout << "The optimal order is : \n";
            scheduler_bruteforce.printOrder(best_order);
            std::cout << "The optimal schedule is : \n";
            scheduler_bruteforce.printDepthVector(depth_vector);
        }

        if (total_test % 10000 == 0) {
            std::cout << "------ Updated result: ------ \n";
            std::cout << "BFST generate better results ratio:  " << bfs_better_count << " / " << total_test;
            std::cout << ". BFST failure ratio:  " << bfs_failure_count << " / " << total_test << ".\n";
            std::cout << "MWBFST generate better results ratio:  " << mdbfs_better_count << " / " << total_test;
            std::cout << ". MWBFST failure ratio:  " << mdbfs_failure_count << " / " << total_test << ".\n";
            std::cout << "One or more generate better result ratio:  " << either_better_count << " / " << total_test << ".\n";
            std::cout << "Both BFS generate better results ratio:  " << both_better_count << " / " << total_test;
            std::cout << ". Both failure ratio:  " << both_failure_count << " / " << total_test << ".\n";
            std::cout << "MWBFST same as optimal ratio:  " << mwbfs_get_optimal_count << " / " << total_test;
            std::cout << ". MWBFST better than optimal ratio:  " << mwbfs_bettter_than_optimal_count << " / " << total_test << ".\n";

            std::cout << "DFST average depth is: " << depth_sum_dfst / total_test;
            std::cout << ".\nBFST average depth is: " << depth_sum_bfst / total_test;
            std::cout << ".\nMWBFST average depth is: " << depth_sum_mdbfs / total_test;
            std::cout << ".\nOptimal average depth is: " << depth_sum_optimal / total_test << ".\n";
        }

    }
}

