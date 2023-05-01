#include "sumo_simulator.h"
#include "parameters.h"
#include "sumo_batch_utility.h"

#include <iostream>
#include <string>
#include <vector>

using namespace intersection_management;

int main() {
    sumoBatchTest(5, 10, {"dynamic_lane", "dfs", "bfs", "mdbfs"}, 2.0, 1, 1, -1);
    return 0;
}