#include "batch_test_utility.h"

#include <iostream>

#include "parameters.h"


using namespace intersection_management;

int main() {
    // 5-135 5-182 5-314
    // 6-116
    // 7-91
    // 8-77
    int num_nodes = 5;
    int current_seed = 135;
    int seed_increment = 1;
    int number_of_methods = 7;
    double coefficient;
    bool found = false;

    while (!found) {
        auto depths = BatchTestOneCase(num_nodes, true, current_seed);

        if (depths[5] >= depths[1] * param.travel_time_range[1] && // fifo >= idfs
            depths[1] * param.travel_time_range[1] >= depths[2] * param.travel_time_range[1] && // idfs >= bfs
            depths[2] * param.travel_time_range[1] >= depths[6] && // bfs >= mwdsf
            depths[6] >= depths[3] && // mwdfs >= mwbfs
            depths[3] >= depths[0] // mwbfs >= dl
            ) {

            std::cout << "\n\n##################### Updated result: ##################### \n";
            std::cout << "Seed is: " << current_seed << ", Total node num: " << num_nodes << ". \n";
            std::cout << "Depths are: dynamic_lane, dfs, bfs, mdbfs, global_optimal, fifo, mddfs.\n";
            for (int i = 0; i < number_of_methods; i++) {
                coefficient = 1;
                if (i == 1 || i == 2)
                    coefficient = param.travel_time_range[1];
                std::cout << depths[i] * coefficient << ", ";
            }
            std::cout << "\n";

            if (depths[5] > depths[1] * param.travel_time_range[1] && // fifo >= idfs
                depths[1] * param.travel_time_range[1] >= depths[2] * param.travel_time_range[1] && // idfs >= bfs
                depths[2] * param.travel_time_range[1] > depths[6] && // bfs >= mwdsf
                depths[6] > depths[3] && // mwdfs >= mwbfs
                depths[3] > depths[0] // mwbfs >= dl
                ) {
                found = true;
                std::cout << "Found one strict monotonic example case with seed " << current_seed << " !\n";
            }
            else {
                std::cout << "Found one non-strict monotonic example case with seed " << current_seed << " !\n";
            }
        }

        current_seed += seed_increment;
    }
    return 0;
}