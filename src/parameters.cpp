#include "parameters.h"

#include <iostream>
#include <string>
#include "yaml-cpp/yaml.h"

namespace intersection_management {

#ifdef PROJECT_ROOT_DIR
const std::string PROJECT_DIR = PROJECT_ROOT_DIR;
#else
/* path for linux */
const std::string PROJECT_DIR = "/home/dong/workspace/intersection_CDG";
/* path for windows */
// static const std::string PROJECT_DIR = "d:\\program\\workspace\\intersection_CDG";
#endif

const std::string CONFIG_FILE = "/configs/config.yaml";
// static const std::string CONFIG_FILE = "\\src\\config.yaml";

void Parameters::readParametersFromYaml() {
    YAML::Node config = YAML::LoadFile(PROJECT_DIR + CONFIG_FILE);

    num_legs = config["num_legs"].as<int>();
    num_lanes_in_vec = config["num_lanes_in_vec"].as<std::vector<int>>();
    num_lanes_out_vec = config["num_lanes_out_vec"].as<std::vector<int>>();
    arrival_interval_avg = config["arrival_interval_avg"].as<double>();
    travel_time_range = config["travel_time_range"].as<std::vector<int>>();

    activate_precedent_offset = config["activate_precedent_offset"].as<bool>();
    activate_arrival_time = config["activate_arrival_time"].as<bool>();

    tie_minimum_resource_waste_first = config["tie_minimum_resource_waste_first"].as<bool>();
    tie_high_demand_first = config["tie_high_demand_first"].as<bool>();
    tie_consider_splitting_resource = config["tie_consider_splitting_resource"].as<bool>();
    tie_more_splitted_resource_first = config["tie_more_splitted_resource_first"].as<bool>();

    random_seed = config["random_seed"].as<int>();
    test_one_instance = config["test_one_instance"].as<bool>();
    test_vehicle_number = config["test_vehicle_number"].as<int>();

    sumo_config_file = config["sumo_config_file"].as<std::string>();
    travel_time_choice = config["travel_time_choice"].as<std::vector<double>>();
    kTimeWindowOffset = config["kTimeWindowOffset"].as<double>();
}

Parameters param;
std::vector<Parameters> geometryParamVec = {
    param,
    Parameters(4, {3, 3, 3, 3}, {3, 3, 3, 3}, "/configs/sumo_intersection1/intersection_unregulated.sumocfg", {6, 7, 8}, 5.15),
    Parameters(4, {1, 2, 1, 2}, {1, 2, 1, 2}, "/configs/sumo_intersection2/intersection_unregulated.sumocfg", {5, 6, 7}, 5.45)};
} // namespace intersection_management
