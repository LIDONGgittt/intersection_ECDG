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

    int num_legs;
    std::vector<int> num_lanes_in_vec;
    std::vector<int> num_lanes_out_vec;
    double arrival_interval_avg;
    std::vector<int> travel_time_range;
    int random_seed;
    bool activate_precedent_offset;
}; // class Parameters

} // namespace intersection_management
#endif // #define INTERSECTION_MANAGEMENT_PARAMETERS_H_