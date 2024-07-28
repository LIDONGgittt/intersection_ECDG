#ifndef INTERSECTION_MANAGEMENT_PARAMETERS_H_
#define INTERSECTION_MANAGEMENT_PARAMETERS_H_

#include <vector>
#include <string>

namespace intersection_management {
extern const std::string PROJECT_DIR;
extern const std::string CONFIG_FILE;

class Parameters {
public:
    Parameters() {
        readParametersFromYaml();
    }
    Parameters(int nl, std::vector<int> nliv, std::vector<int> nlov,
               std::string scf, std::vector<double> ttc = {6, 7, 8},
               double timeWindowOffset = 6.15, double aia = 2.0,
               std::vector<int> ttr = {6, 7}) {
        readParametersFromYaml();
        num_legs = nl;
        num_lanes_in_vec = nliv;
        num_lanes_out_vec = nlov;
        sumo_config_file = scf;
        arrival_interval_avg = aia;
        travel_time_range = ttr;
        travel_time_choice = ttc;
        kTimeWindowOffset = timeWindowOffset;
    }

    void readParametersFromYaml();

    // ## intersection gemotry and configurations ##
    int num_legs;
    std::vector<int> num_lanes_in_vec;
    std::vector<int> num_lanes_out_vec;
    double arrival_interval_avg;
    std::vector<int> travel_time_range;

    // ## scheduler configurations ##
    bool activate_precedent_offset;
    bool activate_arrival_time;

    // # tiebreak strategy
    // # right/left most turns first
    bool tie_minimum_resource_waste_first;
    // # high demand lanes first
    bool tie_high_demand_first;
    // # splitting resource first
    bool tie_consider_splitting_resource;
    // # splitting more resource first
    bool tie_more_splitted_resource_first;

    // ## testing configurations ##
    int random_seed;
    bool test_one_instance;
    int test_vehicle_number;

    // ## sumo related configurations ##
    std::string sumo_config_file;
    std::vector<double> travel_time_choice;
    double kTimeWindowOffset;

}; // class Parameters

extern Parameters param;
extern std::vector<Parameters> geometryParamVec;
} // namespace intersection_management
#endif // #define INTERSECTION_MANAGEMENT_PARAMETERS_H_