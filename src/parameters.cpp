#include "parameters.h"

#include <iostream>
#include <string>
#include "yaml-cpp/yaml.h"

namespace intersection_management {

static const std::string PROJECT_DIR = "/home/dong/workspace/intersection_CDG";
static const std::string CONFIG_FILE = "/src/config.yaml";

void Parameters::readParametersFromYaml() {
    YAML::Node config = YAML::LoadFile(PROJECT_DIR + CONFIG_FILE);
    num_directions = config["num_directions"].as<int>();
    num_lanes_in = config["num_lanes_in"].as<std::vector<int>>();
    num_lanes_out = config["num_lanes_out"].as<std::vector<int>>();
}

Parameters param;
} // namespace intersection_management