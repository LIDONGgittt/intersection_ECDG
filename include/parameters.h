#ifndef INTERSECTION_MANAGEMENT_PARAMETERS_H_
#define INTERSECTION_MANAGEMENT_PARAMETERS_H_

#include <vector>

namespace intersection_management {
class Parameters {
public:
    int num_directions;
    std::vector<int> num_lanes_in;
    std::vector<int> num_lanes_out;
    Parameters() {
        readParametersFromYaml();
    }

    void readParametersFromYaml();
}; // class Parameters

} // namespace intersection_management
#endif // #define INTERSECTION_MANAGEMENT_PARAMETERS_H_