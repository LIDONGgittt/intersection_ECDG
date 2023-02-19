#include "parameters.h"

#include <iostream>
#include "yaml-cpp/yaml.h"

namespace intersection_management {
void Parameters::readParameters() {
    YAML::Node config = YAML::LoadFile("/home/dong/workspace/intersection_CDG/src/config.yaml");
    number_directions = config["number_directions"].as<int>();
    number_lanes_in = config["number_lanes_in"].as<std::vector<int>>();
    number_lanes_out = config["number_lanes_out"].as<std::vector<int>>();
}

Parameters param;
}