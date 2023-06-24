#include "batch_test_utility.h"

#include <iostream>

#include "parameters.h"


using namespace intersection_management;

int main() {
    // 5-20508 5-43615 5-117703 5-124674 5-195709 5-241997
    // 8-2956
    int num_nodes = 5;
    int current_seed = 20508;
    int seed_increment = 1;
    int number_of_methods = 7;
    double coefficient;
    bool found = false;

    while (!found) {
        param.tie_high_demand_first = true;
        param.activate_precedent_offset = true;
        auto depths = BatchTestOneCase(num_nodes, true, current_seed);
        param.tie_high_demand_first = false;
        auto depths_no_high_demand = BatchTestOneCase(num_nodes, false, current_seed);
        param.tie_high_demand_first = true;
        param.activate_precedent_offset = false;
        auto depths_no_offset = BatchTestOneCase(num_nodes, false, current_seed);
        param.tie_high_demand_first = false;
        param.activate_precedent_offset = false;
        auto depths_no_policy = BatchTestOneCase(num_nodes, false, current_seed);

        // found = true;
        if (depths[5] >= depths[1] * param.travel_time_range[1] && // fifo >= idfs
            depths[1] * param.travel_time_range[1] >= depths[2] * param.travel_time_range[1] && // idfs >= bfs
            depths[2] * param.travel_time_range[1] >= depths[6] && // bfs >= mwdsf
            depths[6] >= depths[3] && // mwdfs >= mwbfs
            depths[3] >= depths[0] && // mwbfs >= dl
            depths[0] <= depths_no_high_demand[0]) {

            std::cout << "\n\n##################### Updated result: ##################### \n";
            std::cout << "Seed is: " << current_seed << ", Total node num: " << num_nodes << ". \n";
            std::cout << "Depths are: dynamic_lane, dfs, bfs, mdbfs, global_optimal, fifo, mddfs. And dynamic lane without high demand first.\n";
            for (int i = 0; i < number_of_methods; i++) {
                coefficient = 1;
                if (i == 1 || i == 2)
                    coefficient = param.travel_time_range[1];
                std::cout << depths[i] * coefficient << ", ";
            }
            std::cout << depths_no_high_demand[0] << ", ";
            std::cout << "\n";

            std::cout << "Depths without high demand are: dynamic_lane, dfs, bfs, mdbfs, global_optimal, fifo, mddfs.\n";
            for (int i = 0; i < number_of_methods; i++) {
                coefficient = 1;
                if (i == 1 || i == 2)
                    coefficient = param.travel_time_range[1];
                std::cout << depths_no_high_demand[i] * coefficient << ", ";
            }
            std::cout << "\n";

            std::cout << "Depths without precedence offset are: dynamic_lane, dfs, bfs, mdbfs, global_optimal, fifo, mddfs.\n";
            for (int i = 0; i < number_of_methods; i++) {
                coefficient = 1;
                if (i == 1 || i == 2)
                    coefficient = param.travel_time_range[1];
                std::cout << depths_no_offset[i] * coefficient << ", ";
            }
            std::cout << "\n";

            std::cout << "Depths with no policy are: dynamic_lane, dfs, bfs, mdbfs, global_optimal, fifo, mddfs.\n";
            for (int i = 0; i < number_of_methods; i++) {
                coefficient = 1;
                if (i == 1 || i == 2)
                    coefficient = param.travel_time_range[1];
                std::cout << depths_no_policy[i] * coefficient << ", ";
            }
            std::cout << "\n";

            if (depths[5] > depths[1] * param.travel_time_range[1] && // fifo > idfs
                depths[1] * param.travel_time_range[1] >= depths[2] * param.travel_time_range[1] && // idfs >= bfs
                depths[2] * param.travel_time_range[1] > depths[6] && // bfs > mwdsf
                depths[6] > depths[3] && // mwdfs > mwbfs
                depths[3] > depths[0] && // mwbfs > dl
                depths[0] < depths_no_high_demand[0] &&
                depths[0] < depths_no_offset[0] &&
                depths[0] < depths_no_policy[0] &&
                depths_no_high_demand[0] < depths_no_policy[0] &&
                depths_no_offset[0] < depths_no_policy[0]
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