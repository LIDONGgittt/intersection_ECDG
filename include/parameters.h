#ifndef INTERSECTION_MANAGEMENT_PARAMETERS_H_
#define INTERSECTION_MANAGEMENT_PARAMETERS_H_

#include <vector>

namespace intersection_management {
class Parameters {
public:
    int number_directions;
    std::vector<int> number_lanes_in;
    std::vector<int> number_lanes_out;
    Parameters() {
        readParameters();
    }

    void readParameters();
}; // class Parameters

} // namespace intersection_management
#endif // #define INTERSECTION_MANAGEMENT_PARAMETERS_H_