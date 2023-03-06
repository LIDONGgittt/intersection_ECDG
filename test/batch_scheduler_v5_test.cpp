#include <gtest/gtest.h>

#include "batch_test_utility.h"
#include "parameters.h"

namespace intersection_management {
extern Parameters param;
}

using namespace intersection_management;

TEST(BatchSchedulerTest, CompareDynamicLaneAssignmentsOneCase) {
    auto depths = BatchTest(5);
    std::cout << "Batch results are: ";
    for (auto d : depths) {
        std::cout << d << ", ";
    }
    std::cout << "\n";
}

TEST(BatchSchedulerTest, CompareDynamicLaneAssignments5NodesMultipleCase) {
    std::vector<double> sum(5, 0);
    std::vector<long> better_count(5, 0);
    std::vector<long> better_dfs(5, 0);
    long total_test = 0;
    // while (total_test < 20001) {
    while (true) {
        auto depths = BatchTest(5);
        for (int i = 0; i < 5; i++) {
            sum[i] += depths[i];
            if (depths[i] <= depths[4])
                better_count[i]++;
            if (depths[i] <= depths[1] * param.travel_time_range[1])
                better_dfs[i]++;
        }
        total_test++;

        if (total_test % 1000 == 0) {
            std::cout << "\n\n##################### Updated result: ##################### \n";
            std::cout << "Total tests: " << total_test << ", Total node num: 5. \n";
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