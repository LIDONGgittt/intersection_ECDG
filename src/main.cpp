#include "batch_test_utility.h"
#include "parameters.h"

namespace intersection_management {
extern Parameters param;
} // namespace intersection_management

using namespace intersection_management;

int main() {
    // Check case:
    // num_legs: 4
    // num_lanes_in_vec: [1, 1, 2, 1]
    // num_lanes_out_vec: [3, 1, 1, 1]
    // arrival_interval_avg: 2.0
    // travel_time_range: [3, 6]
    // seed = 16/26/31, vehicle = 5
    // seed = 18/19/28/31/42, vehicle = 8
    // seed = 5/8/12/15, vehicle = 10

    // int seed = 26;
    int seed = param.random_seed;
    while (true) {
        auto result = BatchTest(param.test_vehicle_number, true, true, seed++);
        if (param.test_one_instance)
            break;
        if (result[0] > result[4])
            break;
    }
}