#include <gtest/gtest.h>

#include "batch_test_utility.h"

using namespace intersection_management;

TEST(BatchSchedulerTest, CompareDynamicLaneAssignments100NodesMultipleCase) {
    std::vector<double> sum(5, 0);
    std::vector<long> better_count(5, 0);
    long total_test = 0;
    while (true) {
        auto depths = BatchTest(100);
        for (int i = 0; i < 5; i++) {
            sum[i] += depths[i];
            if (depths[i] <= depths[4])
                better_count[i]++;
        }
        total_test++;

        if (total_test % 10 == 0) {
            std::cout << "\n\n##################### Updated result: ##################### \n";
            std::cout << "Total tests: " << total_test << ", Total node num: 100. \n";
            std::cout << "Average depths: dynamic_lanes, dfs, bfs, mwbfs, global_optimal.\n";
            for (int i = 0; i < 5; i++) {
                std::cout << sum[i] / total_test << ", ";
            }
            std::cout << "\n Better ratio: \n";
            for (int i = 0; i < 5; i++) {
                std::cout << (double)better_count[i] / total_test << ", ";
            }
            std::cout << "\n";
        }
    }
}