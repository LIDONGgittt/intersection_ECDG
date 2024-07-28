#include "cdg_scheduler.h"

using namespace intersection_management;

int main() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    CDGScheduler scheduler_dfs = CDGScheduler();
    CDGScheduler scheduler_bfs = CDGScheduler();
    CDGScheduler scheduler_mdbfs = CDGScheduler();
    CDGScheduler scheduler_bruteforce = CDGScheduler();

    // cdg generater parameters
    unsigned int seed = 0;
    int max_node = 1;
    double max_estimate_travel_time = 5.0;
    double min_estimate_travel_time = 2.0;
    // verbose flag
    bool verbose_mode_for_bfs = false;
    bool verbose_mode_for_mwbfs = false;
    // statistic variables
    int total_test = 0;
    int bfs_failure_count = 0;
    int bfs_better_count = 0;
    int mdbfs_better_count = 0;
    int mdbfs_failure_count = 0;
    bool mwbfs_failure_flag = false;
    bool mwbfs_better_flag = false;
    bool either_better_flag = false;
    int both_failure_count = 0;
    int both_better_count = 0;
    int either_better_count = 0;
    double depth_sum_dfst = 0.0;
    double depth_sum_bfst = 0.0;
    double depth_sum_mdbfs = 0.0;

    // seed = std::time(NULL);
    seed = 0;
    srand(seed);
    while (total_test < 1) {
        total_test++;
        do {
            cdg.GenerateRandomGraph(std::rand() % max_node + 5, max_estimate_travel_time - min_estimate_travel_time + 1.0, 2.0, min_estimate_travel_time, 1.0, true);
            // cdg.GenerateRandomGraph(5, max_estimate_travel_time - min_estimate_travel_time + 1.0, 2.0, min_estimate_travel_time, 1.0, true);
        } while (!cdg.isFullyConnected());

        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
        auto best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
        auto depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
        double global_optimal = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);

        cdg.PrintGraph();
        std::cout << "Modified DFST:\n";
        modified_dfst.PrintTree(true);
        std::cout << "BFST:\n";
        bfst.PrintTree(true);
        std::cout << "MDBFST:\n";
        mdbfst.PrintTree(true);
        std::cout << "Global optimal evacuation time is: " << global_optimal << "\n";
        std::cout << "The optimal order is : \n";
        scheduler_bruteforce.printOrder(best_order);
        std::cout << "The optimal schedule is : \n";
        scheduler_bruteforce.printDepthVector(depth_vector);


        // depth_sum_dfst += modified_dfst.edge_weighted_depth_;
        // depth_sum_bfst += bfst.edge_weighted_depth_;
        // depth_sum_mdbfs += mdbfst.edge_node_weighted_depth_;

        // mwbfs_better_flag = false;
        // mwbfs_failure_flag = false;
        // either_better_flag = false;
        // if (mdbfst.edge_node_weighted_depth_ < (max_estimate_travel_time)*modified_dfst.edge_weighted_depth_ && mdbfst.edge_node_weighted_depth_ < (max_estimate_travel_time)*bfst.edge_weighted_depth_) {
        //     mdbfs_better_count++;
        //     mwbfs_better_flag = true;
        //     either_better_flag = true;
        // }
        // else if (mdbfst.edge_node_weighted_depth_ > (max_estimate_travel_time) *modified_dfst.edge_weighted_depth_ || mdbfst.edge_node_weighted_depth_ > (max_estimate_travel_time) *bfst.edge_weighted_depth_) {
        //     mdbfs_failure_count++;
        //     mwbfs_failure_flag = true;
        //     if (verbose_mode_for_mwbfs) {
        //         std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
        //         std::cout << "!!!!!!   Multi depth is not the best one   !!!!!!\n";
        //         cdg.PrintGraph();
        //         std::cout << "Modified DFST:\n";
        //         modified_dfst.PrintTree(true);
        //         std::cout << "BFST:\n";
        //         bfst.PrintTree(true);
        //         std::cout << "MDBFST:\n";
        //         mdbfst.PrintTree(true);
        //     }
        // }

        // if (modified_dfst.edge_weighted_depth_ > bfst.edge_weighted_depth_) {
        //     bfs_better_count++;
        //     either_better_flag = true;
        //     if (mwbfs_better_flag) {
        //         both_better_count++;
        //     }
        // }
        // else if (modified_dfst.edge_weighted_depth_ < bfst.edge_weighted_depth_) {
        //     bfs_failure_count++;
        //     if (mwbfs_failure_flag) {
        //         both_failure_count++;
        //         // std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
        //         // std::cout << "!!!!!!   Both bfs methods fail to win dfs   !!!!!!\n";
        //         // std::cout << "Max node nums: " << max_node << ", Max estimate travel time: " << max_estimate_travel_time << ", Min estimate travel time: " << min_estimate_travel_time << "\n";
        //         // cdg.PrintGraph();
        //         // std::cout << "Modified DFST:\n";
        //         // modified_dfst.PrintTree(true);
        //         // std::cout << "BFST:\n";
        //         // bfst.PrintTree(true);
        //         // std::cout << "MDBFST:\n";
        //         // mdbfst.PrintTree(true);
        //     }
        //     if (verbose_mode_for_bfs) {
        //         std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << seed << " # # # # # # # # # # # # # #\n";
        //         cdg.PrintGraph();
        //         std::cout << "Modified DFST:\n";
        //         modified_dfst.PrintTree(true);
        //         std::cout << "BFST:\n";
        //         bfst.PrintTree(true);
        //         std::cout << "MDBFST:\n";
        //         mdbfst.PrintTree(true);
        //     }
        // }

    }

    return 0;
}

