#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <chrono>
#include "scheduler.h"

using namespace intersection_management;

void test1() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    Scheduler scheduler_dfs = Scheduler();
    Scheduler scheduler_bfs = Scheduler();
    Scheduler scheduler_mdbfs = Scheduler();

    // cdg.reset();
    unsigned int seed = 0;
    int max_node = 100;
    int total_test = 1000;
    int failure_count = 0;
    int better_count = 0;
    bool verbose_mode = true;

    for (int repeat = 0; repeat < total_test; repeat++) {
        seed = std::time(NULL);
        do {
            // cdg.GenerateRandomGraph(std::rand() % max_node + 1, seed);
            cdg.GenerateRandomGraph(5, repeat);
        } while (!cdg.isFullyConnected());

        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);

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
                std::cout << "MDBFST:\n";
                mdbfst.PrintTree(true);
            }
        }
    }

    std::cout << "BFST generate better results ratio:  " << better_count << " / " << total_test << "\n";
    std::cout << "BFST failed ratio:  " << failure_count << " / " << total_test << "\n";
}


void test2() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    Scheduler scheduler_dfs = Scheduler();
    Scheduler scheduler_bfs = Scheduler();
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
            cdg.GenerateRandomGraph(std::rand() % max_node + 5, seed);
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
    test1();
}