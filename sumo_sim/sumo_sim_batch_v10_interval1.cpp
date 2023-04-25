#include "sumo_batch_utility.h"

#include <iostream>
#include <string>
#include <vector>

using namespace intersection_management;

int main() {
    sumoBatchTest(10, -1, {"dynamic_lane", "dfs", "bfs", "mdbfs"}, 1.0, 100, -1);
    return 0;
}