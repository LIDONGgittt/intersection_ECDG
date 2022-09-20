#include <iostream>
#include "conflict_directed_graph.h"

using namespace intersection_management;
int main() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();
    // cdg.reset();
    cdg.GenerateRandomGraph(5);
    cdg.PrintGraph();
}