#include <iostream>
#include "conflict_directed_graph.h"

using namespace intersection_management;
int main() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();
    // cdg.reset();
    unsigned int seed = 0;
    do {
        cdg.GenerateRandomGraph(5, seed++);
    } while (!cdg.isFullyConnected());

    cdg.PrintGraph();
    // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
}