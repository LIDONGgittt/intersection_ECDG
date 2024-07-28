#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <chrono>

#include "cdg_scheduler.h"

using namespace intersection_management;

void cdg_main_test1() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    CDGScheduler scheduler_dfs = CDGScheduler();
    CDGScheduler scheduler_bfs = CDGScheduler();
    CDGScheduler scheduler_mdbfs = CDGScheduler();

    // cdg.reset();
    unsigned int seed = 0;
    int max_node = 1;
    int bfs_failure_count = 0;
    int bfs_better_count = 0;
    int mdbfs_better_count = 0;
    int mdbfs_failure_count = 0;
    bool mwbfs_failure_flag = false;
    bool mwbfs_better_flag = false;
    int both_failure_count = 0;
    int both_better_count = 0;
    int either_better_count = 0;
    bool either_better_flag = false;
    bool verbose_mode_for_bfs = false;
    bool verbose_mode_for_mwbfs = false;
    double depth_sum_dfst = 0.0;
    double depth_sum_bfst = 0.0;
    double depth_sum_mdbfs = 0.0;
    double max_estimate_travel_time = 5.0;
    double min_estimate_travel_time = 2.0;

    int total_test = 100000;

    for (int repeat = 0; repeat < total_test; repeat++) {
        // seed = std::time(NULL);
        srand(seed++);
        // std::cout << seed << std::endl;
        // srand(28745);
        // srand(100);
        do {
            // cdg.GenerateRandomGraph(std::rand() % max_node + 1);
            // cdg.GenerateRandomGraph(5, 4.0, 2.0, 2.0, 1.0, true);
            cdg.GenerateRandomGraph(std::rand() % max_node + 5, max_estimate_travel_time - min_estimate_travel_time + 1.0, 2.0, min_estimate_travel_time, 1.0, true);
        } while (!cdg.isFullyConnected());
        // cdg.PrintGraph();
        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
        depth_sum_dfst += modified_dfst.edge_weighted_depth_;
        depth_sum_bfst += bfst.edge_weighted_depth_;
        depth_sum_mdbfs += mdbfst.edge_node_weighted_depth_;

        mwbfs_better_flag = false;
        mwbfs_failure_flag = false;
        either_better_flag = false;
        if (mdbfst.edge_node_weighted_depth_ < (max_estimate_travel_time)*modified_dfst.edge_weighted_depth_ && mdbfst.edge_node_weighted_depth_ < (max_estimate_travel_time)*bfst.edge_weighted_depth_) {
            mdbfs_better_count++;
            mwbfs_better_flag = true;
            either_better_flag = true;
        }
        else if (mdbfst.edge_node_weighted_depth_ > (max_estimate_travel_time) *modified_dfst.edge_weighted_depth_ || mdbfst.edge_node_weighted_depth_ > (max_estimate_travel_time) *bfst.edge_weighted_depth_) {
            mdbfs_failure_count++;
            mwbfs_failure_flag = true;
            if (verbose_mode_for_mwbfs) {
                std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                std::cout << "!!!!!!   Multi depth is not the best one   !!!!!!\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst.PrintTree(true);
                std::cout << "BFST:\n";
                bfst.PrintTree(true);
                std::cout << "MDBFST:\n";
                mdbfst.PrintTree(true);
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
                std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                std::cout << "!!!!!!   Both bfs methods fail to win dfs   !!!!!!\n";
                std::cout << "Max node nums: " << max_node << ", Max estimate travel time: " << max_estimate_travel_time << ", Min estimate travel time: " << min_estimate_travel_time << "\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst.PrintTree(true);
                std::cout << "BFST:\n";
                bfst.PrintTree(true);
                std::cout << "MDBFST:\n";
                mdbfst.PrintTree(true);
            }
            if (verbose_mode_for_bfs) {
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

        if (either_better_flag) {
            either_better_count++;
        }

    }
    std::cout << "------ Updated result: ------ \n";
    std::cout << "BFST generate better results ratio:  " << bfs_better_count << " / " << total_test;
    std::cout << ". BFST failure ratio:  " << bfs_failure_count << " / " << total_test << ".\n";
    std::cout << "MWBFST generate better results ratio:  " << mdbfs_better_count << " / " << total_test;
    std::cout << ". MWBFST failure ratio:  " << mdbfs_failure_count << " / " << total_test << ".\n";
    std::cout << "One or more generate better result ratio:  " << either_better_count << " / " << total_test << ".\n";
    std::cout << "Both BFS generate better results ratio:  " << both_better_count << " / " << total_test;
    std::cout << ". Both failure ratio:  " << both_failure_count << " / " << total_test << ".\n";
    std::cout << "DFST average depth is: " << depth_sum_dfst / total_test;
    std::cout << ".\nBFST average depth is: " << depth_sum_bfst / total_test;
    std::cout << ".\nMWBFST average depth is: " << depth_sum_mdbfs / total_test << ".\n";

}


void cdg_main_test2() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    CDGScheduler scheduler_dfs = CDGScheduler();
    CDGScheduler scheduler_bfs = CDGScheduler();
    // cdg.reset();
    unsigned int seed = 0;
    int max_node = 100;
    int total_test = 0;
    int failure_count = 0;
    int better_count = 0;
    bool verbose_mode = false;
    double depth_sum_dfst = 0.0;
    double depth_sum_bfst = 0.0;

    while (true) {
        total_test++;
        seed = std::time(NULL);
        do {
            cdg.GenerateRandomGraph(std::rand() % max_node + 5);
        } while (!cdg.isFullyConnected());

        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);

        depth_sum_dfst += modified_dfst.edge_weighted_depth_;
        depth_sum_bfst += bfst.edge_weighted_depth_;
        if (modified_dfst.edge_weighted_depth_ > bfst.edge_weighted_depth_) {
            better_count++;
        }
        else if (modified_dfst.edge_weighted_depth_ < bfst.edge_weighted_depth_) {
            failure_count++;
            if (verbose_mode) {
                std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << seed << " # # # # # # # # # # # # # #\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst.PrintTree(true);
                std::cout << "BFST:\n";
                bfst.PrintTree(true);
            }
        }

        if (total_test % 10000 == 0) {
            std::cout << "------ Updated result: ------ \n";
            std::cout << "BFST generate better results ratio:  " << better_count << " / " << total_test;
            std::cout << ". BFST failure ratio:  " << failure_count << " / " << total_test << "\n";
            std::cout << "BFST average depth is: " << depth_sum_bfst / total_test;
            std::cout << ". DFST average depth is: " << depth_sum_dfst / total_test << "\n";
        }
    }
}

int main() {
    cdg_main_test1();
    return 0;
}