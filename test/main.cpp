#include "batch_test_utility.h"
#include "parameters.h"

using namespace intersection_management;

int main() {
    // Check case:
    // num_legs: 4
    // num_lanes_in_vec: [1, 1, 2, 1]
    // num_lanes_out_vec: [3, 1, 1, 1]
    // arrival_interval_avg: 2.0
    // travel_time_range: [3, 6]
    // vehicle = 5,  seed = 16/70
    // vehicle = 8,  seed = 4/5/9
    // vehicle = 10, seed = 9/23

    // int seed = 26;
    int seed = param.random_seed;
    while (true) {
        auto result = BatchTestOneCase(param.test_vehicle_number, true, seed++);
        if (param.test_one_instance)
            break;
        if (result[0] > result[4])
            break;
    }
}