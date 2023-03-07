#ifndef INTERSECTION_MANAGEMENT_PARAMETERS_H_
#define INTERSECTION_MANAGEMENT_PARAMETERS_H_

#include <vector>

namespace intersection_management {
class Parameters {
public:
    Parameters() {
        readParametersFromYaml();
    }
    void readParametersFromYaml();

    // # intersection gemotry and configurations
    int num_legs;
    std::vector<int> num_lanes_in_vec;
    std::vector<int> num_lanes_out_vec;
    double arrival_interval_avg;
    std::vector<int> travel_time_range;

    // # scheduler configurations
    bool activate_precedent_offset;

    // # tiebreak strategy
    // # right/left most turns first
    bool tie_minimum_resource_waste_first;
    // # high demand lanes first
    bool tie_high_demand_first;
    // # splitting resource first
    bool tie_consider_splitting_resource;
    // # splitting more resource first
    bool tie_more_splitted_resource_first;

    // # testing configurations
    int random_seed;
    bool test_one_instance;
    int test_vehicle_number;
}; // class Parameters

} // namespace intersection_management
#endif // #define INTERSECTION_MANAGEMENT_PARAMETERS_H_