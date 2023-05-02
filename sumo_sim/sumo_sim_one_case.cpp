#include "sumo_simulator.h"
#include "parameters.h"

using namespace intersection_management;

int main() {

    SumoSimulator sumo_simulator(0.01, 10000);

    sumo_simulator.setSumoGUI(true);
    sumo_simulator.setSimulateOneMethod(100, "mdbfs", 1.0, 2, true, -1);
    // sumo_simulator.setSimulateOneMethod(10, "dynamic_lane", 1.0, 1, true, 10);
    // sumo_simulator.setSimulateOneMethod(40, "dfs", 2.0, 1, true, 0);
    // sumo_simulator.setSimulateOneMethod(40, "mdbfs", 2.0, 1, true, 0);
    // // low demand
    // sumo_simulator.setSimulateOneMethod(100, "dynamic_lane", 2.0, 1, true, 31);
    // sumo_simulator.setSimulateOneMethod(100, "dfs", 2.0, 1, true, 33);
    // sumo_simulator.setSimulateOneMethod(100, "bfs", 2.0, 1, true, 33);
    // sumo_simulator.setSimulateOneMethod(100, "mdbfs", 2.0, 1, true, 33);
    // // high demand
    // sumo_simulator.setSimulateOneMethod(100, "dynamic_lane", 1.0, 1, true, 33);
    // sumo_simulator.setSimulateOneMethod(100, "dfs", 1.0, 1, true, 33);
    // sumo_simulator.setSimulateOneMethod(100, "bfs", 1.0, 1, true, 33);
    // sumo_simulator.setSimulateOneMethod(100, "mdbfs", 1.0, 1, true, 33);

    // sumo_simulator.setSimulateOneMethod(5, "dynamic_lane", 2.0, 1, true, 0);
    
    sumo_simulator.startSimulation();
}