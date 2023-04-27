#include "sumo_simulator.h"
#include "parameters.h"

using namespace intersection_management;

int main() {

    SumoSimulator sumo_simulator(0.01, 10000);
    param.num_lanes_in_vec = {3, 3, 3, 3};
    param.num_lanes_out_vec = {3, 3, 3, 3};

    sumo_simulator.setSumoGUI(true);
    sumo_simulator.setSimulateOneRandomCase(100, "mdbfs", true, 1.0, 667);
    // sumo_simulator.setSimulateOneRandomCase(10, "dynamic_lane", true, 1.0, 10);
    // sumo_simulator.setSimulateOneRandomCase(40, "dfs", true, 2.0, 0);
    // sumo_simulator.setSimulateOneRandomCase(40, "mdbfs", true, 2.0, 0);
    // // low demand
    // sumo_simulator.setSimulateOneRandomCase(100, "dynamic_lane", true, 2.0, 31);
    // sumo_simulator.setSimulateOneRandomCase(100, "dfs", true, 2.0, 33);
    // sumo_simulator.setSimulateOneRandomCase(100, "bfs", true, 2.0, 33);
    // sumo_simulator.setSimulateOneRandomCase(100, "mdbfs", true, 2.0, 33);
    // // high demand
    // sumo_simulator.setSimulateOneRandomCase(100, "dynamic_lane", true, 1.0, 33);
    // sumo_simulator.setSimulateOneRandomCase(100, "dfs", true, 1.0, 33);
    // sumo_simulator.setSimulateOneRandomCase(100, "bfs", true, 1.0, 33);
    // sumo_simulator.setSimulateOneRandomCase(100, "mdbfs", true, 1.0, 33);

    // sumo_simulator.setSimulateOneRandomCase(5, "dynamic_lane", true, 2.0, 0);
    
    sumo_simulator.startSimulation();
}