#include <iostream>
#include "scheduler.h"

using namespace intersection_management;
int main() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    Scheduler scheduler_dfs = Scheduler();
    Scheduler scheduler_bfs = Scheduler();
    // cdg.reset();
    unsigned int seed = 0;
    for (int repeat = 0; repeat < 10; repeat++) {
        do {
            cdg.GenerateRandomGraph(5, seed++);
        } while (!cdg.isFullyConnected());

        std::cout << "# # # # # # # # # # # # # #  New Loop  # # # # # # # # # # # # # #\n";
        // cdg.PrintGraph();
        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        std::cout << "Modified DFST:\n";
        modified_dfst.PrintTree(false);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        std::cout << "BFST:\n";
        bfst.PrintTree(false);
    }

}