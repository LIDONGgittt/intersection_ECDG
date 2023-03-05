#include "parameters.h"

#include <iostream>
#include <string>
#include "yaml-cpp/yaml.h"

namespace intersection_management {

static const std::string PROJECT_DIR = "/home/dong/workspace/intersection_CDG";
static const std::string CONFIG_FILE = "/src/config.yaml";

void Parameters::readParametersFromYaml() {
    YAML::Node config = YAML::LoadFile(PROJECT_DIR + CONFIG_FILE);
    num_legs = config["num_legs"].as<int>();
    num_lanes_in_vec = config["num_lanes_in_vec"].as<std::vector<int>>();
    num_lanes_out_vec = config["num_lanes_out_vec"].as<std::vector<int>>();
    arrival_interval_avg = config["arrival_interval_avg"].as<double>();
    travel_time_range = config["travel_time_range"].as<std::vector<int>>();
    random_seed = config["random_seed"].as<int>();
    activate_precedent_offset = config["activate_precedent_offset"].as<bool>();
}

Parameters param;
} // namespace intersection_management