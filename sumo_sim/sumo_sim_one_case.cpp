#include "sumo_simulator.h"
#include "parameters.h"

namespace intersection_management {
extern Parameters param;
}

using namespace intersection_management;

int main() {
    SumoSimulator sumo_simulator;
    param.num_lanes_in_vec = {3, 3, 3, 3};
    param.num_lanes_out_vec = {3, 3, 3, 3};

    // sumo_simulator.simulateOneRandomCase(10, "dynamic_lane", true, 0);
    // sumo_simulator.simulateOneRandomCase(10, "dfs", true, 0);
    // sumo_simulator.simulateOneRandomCase(60, "dynamic_lane", true, 22);
    // sumo_simulator.simulateOneRandomCase(60, "dynamic_lane", true, 0);
    sumo_simulator.simulateOneRandomCase(5, "dynamic_lane", true, 0);
    // sumo_simulator.simulateOneRandomCase(20, "dfs", true, 0);
    // sumo_simulator.simulateOneRandomCase(20, "bfs", true, 0);
    // sumo_simulator.simulateOneRandomCase(20, "mdbfs", true, 0);
}