#include "sumo_sim_utility.h"
#include "parameters.h"

namespace intersection_management {
extern Parameters param;
}

using namespace intersection_management;

int main() {
    param.num_lanes_in_vec = {3, 3, 3, 3};
    param.num_lanes_out_vec = {3, 3, 3, 3};
    // sumoSimulationOneCase(10, "dynamic_lane", true, 0);
    // sumoSimulationOneCase(10, "dfs", true, 0);
    // sumoSimulationOneCase(60, "dynamic_lane", true, 22);
    // sumoSimulationOneCase(60, "dynamic_lane", true, 0);
    sumoSimulationOneCase(5, "dynamic_lane", true, 0);
    // sumoSimulationOneCase(20, "dfs", true, 0);
    // sumoSimulationOneCase(20, "bfs", true, 0);
    // sumoSimulationOneCase(20, "mdbfs", true, 0);
}