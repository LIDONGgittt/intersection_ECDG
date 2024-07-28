#include "cdg_scheduler.h"

#include <chrono>

using namespace intersection_management;

// cdg scheduler compare v2
int main() {
    ConflictDirectedGraph cdg = ConflictDirectedGraph();

    CDGScheduler scheduler_dfs = CDGScheduler();
    CDGScheduler scheduler_bfs = CDGScheduler();
    CDGScheduler scheduler_mdbfs = CDGScheduler();
    CDGScheduler scheduler_bruteforce = CDGScheduler();

    // cdg generater parameters
    unsigned int seed = 0;
    int max_node = 96;
    double max_estimate_travel_time = 5.0;
    double min_estimate_travel_time = 2.0;
    // verbose flag
    bool verbose_mode_for_bfs = false;
    bool verbose_mode_for_mdbfs = false;
    bool verbose_mode_for_both_failure = false;
    // statistic variables
    int total_test = 0;
    int bfs_failure_count = 0;
    int bfs_better_count = 0;
    int mdbfs_failure_count = 0;
    int mdbfs_better_count = 0;
    bool mdbfs_failure_flag = false;
    bool mdbfs_better_flag = false;
    bool either_better_flag = false;
    int both_failure_count = 0;
    int both_better_count = 0;
    int either_better_count = 0;
    double depth_sum_dfst = 0.0;
    double depth_sum_bfst = 0.0;
    double depth_sum_mdbfs = 0.0;
    double depth_sum_optimal = 0.0;
    int mdbfs_get_optimal_count = 0;
    int mdbfs_bettter_than_optimal_count = 0;
    // statistic variables with fairness conflicts
    int bfs_failure_count_fairness = 0;
    int bfs_better_count_fairness = 0;
    int mdbfs_failure_count_fairness = 0;
    int mdbfs_better_count_fairness = 0;
    bool mdbfs_failure_flag_fairness = false;
    bool mdbfs_better_flag_fairness = false;
    bool either_better_flag_fairness = false;
    int both_failure_count_fairness = 0;
    int both_better_count_fairness = 0;
    int either_better_count_fairness = 0;
    double depth_sum_dfst_fairness = 0.0;
    double depth_sum_bfst_fairness = 0.0;
    double depth_sum_mdbfs_fairness = 0.0;
    double depth_sum_optimal_fairness = 0.0;
    int mdbfs_get_optimal_count_fairness = 0;
    int mdbfs_bettter_than_optimal_count_fairness = 0;

    std::vector<double> fairness_vec_ordersd(4, 0);
    std::vector<double> fairness_vec_jain(4, 0);
    std::vector<double> fairness_vec_ordersd_with_fairness_conflict(4, 0);
    std::vector<double> fairness_vec_jain_with_fairness_conflict(4, 0);

    seed = std::time(NULL);
    // seed = 0;
    srand(seed);
    while (true) {
        total_test++;
        // srand(343);
        // srand(total_test);
        do {
            cdg.GenerateRandomGraph(std::rand() % max_node + 5, max_estimate_travel_time - min_estimate_travel_time + 1.0, 2.0, min_estimate_travel_time, 1.0, true);
            // cdg.GenerateRandomGraph(5, max_estimate_travel_time - min_estimate_travel_time + 1.0, 2.0, min_estimate_travel_time, 1.0, true);
        } while (!cdg.isFullyConnected());

        // std::cout << "The CDG fully connected status is: " << cdg.isFullyConnected() << std::endl;
        auto modified_dfst = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        auto mdbfst = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
        std::vector<int> best_order;
        std::vector<double> depth_vector;
        double global_optimal;
        if (max_node <= 6) {
            best_order = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
            depth_vector = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
            global_optimal = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
        }
        else {
            global_optimal = 0;
        }

        // add fairness conflicts
        cdg.AddFairnessConflicts();
        auto modified_dfst_fairness = scheduler_dfs.ScheduleWithModifiedDfst(cdg);
        auto bfst_fairness = scheduler_bfs.ScheduleWithBfstWeightedEdgeOnly(cdg);
        auto mdbfst_fairness = scheduler_mdbfs.ScheduleWithBfstMultiWeight(cdg);
        std::vector<int> best_order_fairness;
        std::vector<double> depth_vector_fairness;
        double global_optimal_fairness;
        if (max_node <= 6) {
            best_order_fairness = scheduler_bruteforce.ScheduleBruteForceSearch(cdg);
            depth_vector_fairness = scheduler_bruteforce.GetDepthVectorFromOrder(best_order, cdg);
            global_optimal_fairness = scheduler_bruteforce.GetEvacuationTimeFromOrder(best_order, cdg);
        }
        else {
            global_optimal_fairness = 0;
        }

        depth_sum_dfst += modified_dfst.edge_weighted_depth_;
        depth_sum_bfst += bfst.edge_weighted_depth_;
        depth_sum_mdbfs += mdbfst.edge_node_weighted_depth_;
        depth_sum_optimal += global_optimal;
        depth_sum_dfst_fairness += modified_dfst_fairness.edge_weighted_depth_;
        depth_sum_bfst_fairness += bfst_fairness.edge_weighted_depth_;
        depth_sum_mdbfs_fairness += mdbfst_fairness.edge_node_weighted_depth_;
        depth_sum_optimal_fairness += global_optimal_fairness;

        fairness_vec_ordersd[0] += modified_dfst.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_OrderStandardDeviation);
        fairness_vec_ordersd[1] += bfst.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_OrderStandardDeviation);
        fairness_vec_ordersd[2] += mdbfst.CalculateFairnessIndex(Type_EdgeNodeWeightedDepth, Type_OrderStandardDeviation);
        fairness_vec_jain[0] += modified_dfst.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_JainIndex);
        fairness_vec_jain[1] += bfst.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_JainIndex);
        fairness_vec_jain[2] += mdbfst.CalculateFairnessIndex(Type_EdgeNodeWeightedDepth, Type_JainIndex);

        fairness_vec_ordersd_with_fairness_conflict[0] += modified_dfst_fairness.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_OrderStandardDeviation);
        fairness_vec_ordersd_with_fairness_conflict[1] += bfst_fairness.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_OrderStandardDeviation);
        fairness_vec_ordersd_with_fairness_conflict[2] += mdbfst_fairness.CalculateFairnessIndex(Type_EdgeNodeWeightedDepth, Type_OrderStandardDeviation);
        fairness_vec_jain_with_fairness_conflict[0] += modified_dfst_fairness.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_JainIndex);
        fairness_vec_jain_with_fairness_conflict[1] += bfst_fairness.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_JainIndex);
        fairness_vec_jain_with_fairness_conflict[2] += mdbfst_fairness.CalculateFairnessIndex(Type_EdgeNodeWeightedDepth, Type_JainIndex);

        { // statistic for normal cdg
            mdbfs_better_flag = false;
            mdbfs_failure_flag = false;
            either_better_flag = false;
            if (mdbfst.edge_node_weighted_depth_ < (max_estimate_travel_time) * (modified_dfst.edge_weighted_depth_) && mdbfst.edge_node_weighted_depth_ < (max_estimate_travel_time) * (bfst.edge_weighted_depth_)) {
                mdbfs_better_count++;
                mdbfs_better_flag = true;
                either_better_flag = true;
            }
            else if (mdbfst.edge_node_weighted_depth_ > (max_estimate_travel_time) * (modified_dfst.edge_weighted_depth_) || mdbfst.edge_node_weighted_depth_ > (max_estimate_travel_time) * (bfst.edge_weighted_depth_)) {
                mdbfs_failure_count++;
                mdbfs_failure_flag = true;
                if (verbose_mode_for_mdbfs) {
                    std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                    std::cout << "!!!!!!   Multi depth MDBFS is not the best one   !!!!!!\n";
                }
            }
            if (modified_dfst.edge_weighted_depth_ > bfst.edge_weighted_depth_) {
                bfs_better_count++;
                either_better_flag = true;
                if (mdbfs_better_flag) {
                    both_better_count++;
                }
            }
            else if (modified_dfst.edge_weighted_depth_ < bfst.edge_weighted_depth_) {
                bfs_failure_count++;
                if (mdbfs_failure_flag) {
                    both_failure_count++;
                    if (verbose_mode_for_both_failure) {
                        std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                        std::cout << "!!!!!!   Both BFS and MDBFS methods fail to win DFS   !!!!!!\n";
                    }
                }
                if (verbose_mode_for_bfs) {
                    std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << seed << " # # # # # # # # # # # # # #\n";
                    std::cout << "!!!!!!   BFS lost to DFS   !!!!!!\n";
                }
            }
            if (mdbfst.edge_node_weighted_depth_ < global_optimal ||
                (max_estimate_travel_time) * (modified_dfst.edge_weighted_depth_) < global_optimal ||
                (max_estimate_travel_time) * (bfst.edge_weighted_depth_) < global_optimal) {
                mdbfs_bettter_than_optimal_count++;
                std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << total_test << " # # # # # # # # # # # # # #\n";
                std::cout << "Max node nums: " << max_node << ", Max estimate travel time: " << max_estimate_travel_time << ", Min estimate travel time: " << min_estimate_travel_time << "\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst.PrintTree(true);
                std::cout << "BFST:\n";
                bfst.PrintTree(true);
                std::cout << "MDBFST:\n";
                mdbfst.PrintTree(true);
                std::cout << "Global optimal evacuation time is: " << global_optimal << "\n";
                std::cout << "The optimal order is : \n";
                CDGScheduler::printOrder(best_order);
                std::cout << "The optimal schedule is : \n";
                CDGScheduler::printDepthVector(depth_vector);
                break;
            }
            else if (mdbfst.edge_node_weighted_depth_ == global_optimal) {
                mdbfs_get_optimal_count++;
            }
            if (verbose_mode_for_bfs || verbose_mode_for_both_failure || verbose_mode_for_mdbfs) {
                std::cout << "Max node nums: " << max_node << ", Max estimate travel time: " << max_estimate_travel_time << ", Min estimate travel time: " << min_estimate_travel_time << "\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst.PrintTree(true);
                std::cout << "BFST:\n";
                bfst.PrintTree(true);
                std::cout << "MDBFST:\n";
                mdbfst.PrintTree(true);
                std::cout << "Global optimal evacuation time is: " << global_optimal << "\n";
                std::cout << "The optimal order is : \n";
                CDGScheduler::printOrder(best_order);
                std::cout << "The optimal schedule is : \n";
                CDGScheduler::printDepthVector(depth_vector);
            }
            if (either_better_flag) {
                either_better_count++;
            }
        } // end of statistic for normal cdg

        { // statistic for fairness cdg
            mdbfs_better_flag_fairness = false;
            mdbfs_failure_flag_fairness = false;
            either_better_flag_fairness = false;
            if (mdbfst_fairness.edge_node_weighted_depth_ < (max_estimate_travel_time) * (modified_dfst_fairness.edge_weighted_depth_) && mdbfst_fairness.edge_node_weighted_depth_ < (max_estimate_travel_time) * (bfst_fairness.edge_weighted_depth_)) {
                mdbfs_better_count_fairness++;
                mdbfs_better_flag_fairness = true;
                either_better_flag_fairness = true;
            }
            else if (mdbfst_fairness.edge_node_weighted_depth_ > (max_estimate_travel_time) * (modified_dfst_fairness.edge_weighted_depth_) || mdbfst_fairness.edge_node_weighted_depth_ > (max_estimate_travel_time) * (bfst_fairness.edge_weighted_depth_)) {
                mdbfs_failure_count_fairness++;
                mdbfs_failure_flag = true;
                if (verbose_mode_for_mdbfs) {
                    std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                    std::cout << "!!!!!!   Multi depth MDBFS is not the best one   !!!!!!\n";
                }
            }
            if (modified_dfst_fairness.edge_weighted_depth_ > bfst_fairness.edge_weighted_depth_) {
                bfs_better_count_fairness++;
                either_better_flag_fairness = true;
                if (mdbfs_better_flag_fairness) {
                    both_better_count_fairness++;
                }
            }
            else if (modified_dfst_fairness.edge_weighted_depth_ < bfst_fairness.edge_weighted_depth_) {
                bfs_failure_count_fairness++;
                if (mdbfs_failure_flag_fairness) {
                    both_failure_count_fairness++;
                    if (verbose_mode_for_both_failure) {
                        std::cout << "# # # # # # # # # # # # # #  Serious sample with seed: " << seed << " # # # # # # # # # # # # # #\n";
                        std::cout << "!!!!!!   Both BFS and MDBFS methods fail to win DFS   !!!!!!\n";
                    }
                }
                if (verbose_mode_for_bfs) {
                    std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << seed << " # # # # # # # # # # # # # #\n";
                    std::cout << "!!!!!!   BFS lost to DFS   !!!!!!\n";
                }
            }
            if (mdbfst_fairness.edge_node_weighted_depth_ < global_optimal_fairness ||
                (max_estimate_travel_time) * (modified_dfst_fairness.edge_weighted_depth_) < global_optimal_fairness ||
                (max_estimate_travel_time) * (bfst_fairness.edge_weighted_depth_) < global_optimal_fairness) {
                mdbfs_bettter_than_optimal_count_fairness++;
                std::cout << "# # # # # # # # # # # # # #  New Loop with seed: " << total_test << " # # # # # # # # # # # # # #\n";
                std::cout << "Max node nums: " << max_node << ", Max estimate travel time: " << max_estimate_travel_time << ", Min estimate travel time: " << min_estimate_travel_time << "\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst_fairness.PrintTree(true);
                std::cout << "BFST:\n";
                bfst_fairness.PrintTree(true);
                std::cout << "MDBFST:\n";
                mdbfst_fairness.PrintTree(true);
                std::cout << "Global optimal evacuation time is: " << global_optimal_fairness << "\n";
                std::cout << "The optimal order is : \n";
                CDGScheduler::printOrder(best_order_fairness);
                std::cout << "The optimal schedule is : \n";
                CDGScheduler::printDepthVector(depth_vector_fairness);
                break;
            }
            else if (mdbfst_fairness.edge_node_weighted_depth_ == global_optimal_fairness) {
                mdbfs_get_optimal_count_fairness++;
            }
            if (verbose_mode_for_bfs || verbose_mode_for_both_failure || verbose_mode_for_mdbfs) {
                std::cout << "Max node nums: " << max_node << ", Max estimate travel time: " << max_estimate_travel_time << ", Min estimate travel time: " << min_estimate_travel_time << "\n";
                cdg.PrintGraph();
                std::cout << "Modified DFST:\n";
                modified_dfst_fairness.PrintTree(true);
                std::cout << "BFST:\n";
                bfst_fairness.PrintTree(true);
                std::cout << "MDBFST:\n";
                mdbfst_fairness.PrintTree(true);
                std::cout << "Global optimal evacuation time is: " << global_optimal_fairness << "\n";
                std::cout << "The optimal order is : \n";
                CDGScheduler::printOrder(best_order);
                std::cout << "The optimal schedule is : \n";
                CDGScheduler::printDepthVector(depth_vector);
            }
            if (either_better_flag_fairness) {
                either_better_count_fairness++;
            }
        } // end of statistic for fairness cdg

        if (total_test % 100 == 0) {
            std::cout << "\n\n##################### Updated result: ##################### \n";
            std::cout << "Total tests: " << total_test << ", Max node nums: " << max_node + 5 - 1;
            std::cout << ", Max estimate travel time: " << max_estimate_travel_time << ", Min estimate travel time: " << min_estimate_travel_time << "\n";

            std::cout << "BFST generate better results ratio:  " << (double)bfs_better_count / total_test << " (" << bfs_better_count << " / " << total_test << ")\n";
            std::cout << "BFST failure ratio:  " << (double)bfs_failure_count / total_test << " (" << bfs_failure_count << " / " << total_test << ")\n";
            std::cout << "MDBFST generate better results ratio:  " << (double)mdbfs_better_count / total_test << " (" << mdbfs_better_count << " / " << total_test << ")\n";
            std::cout << "MDBFST failure ratio:  " << (double)mdbfs_failure_count / total_test << " (" << mdbfs_failure_count << " / " << total_test << ")\n";
            std::cout << "One or more generate better result ratio:  " << (double)either_better_count / total_test << " (" << either_better_count << " / " << total_test << ")\n";
            std::cout << "Both BFS generate better results ratio:  " << (double)both_better_count / total_test << " (" << both_better_count << " / " << total_test << ")\n";
            std::cout << "Both failure ratio:  " << (double)both_failure_count / total_test << " (" << both_failure_count << " / " << total_test << ")\n";
            std::cout << "MDBFST same as optimal ratio:  " << (double)mdbfs_get_optimal_count / total_test << " (" << mdbfs_get_optimal_count << " / " << total_test << ")\n";
            std::cout << "MDBFST better than optimal ratio:  " << (double)mdbfs_bettter_than_optimal_count / total_test << " (" << mdbfs_bettter_than_optimal_count << " / " << total_test << ")\n";

            std::cout << "DFST average depth is: " << depth_sum_dfst / total_test;
            std::cout << "\nBFST average depth is: " << depth_sum_bfst / total_test;
            std::cout << "\nMDBFST average depth is: " << depth_sum_mdbfs / total_test;
            std::cout << "\nOptimal average depth is: " << depth_sum_optimal / total_test << "\n";
            std::cout << "Average Fairness Index (Order Standard Deviation): " << fairness_vec_ordersd[0] / total_test << ", " << fairness_vec_ordersd[1] / total_test << ", " << fairness_vec_ordersd[2] / total_test << "\n";
            std::cout << "Average Fairness Index (Jain's index): " << fairness_vec_jain[0] / total_test << ", " << fairness_vec_jain[1] / total_test << ", " << fairness_vec_jain[2] / total_test << "\n";

            std::cout << "------------ After adding fairness conflicts: ------------ \n";
            std::cout << "BFST generate better results ratio:  " << (double)bfs_better_count_fairness / total_test << " (" << bfs_better_count_fairness << " / " << total_test << ")\n";
            std::cout << "BFST failure ratio:  " << (double)bfs_failure_count_fairness / total_test << " (" << bfs_failure_count_fairness << " / " << total_test << ")\n";
            std::cout << "MDBFST generate better results ratio:  " << (double)mdbfs_better_count_fairness / total_test << " (" << mdbfs_better_count_fairness << " / " << total_test << ")\n";
            std::cout << "MDBFST failure ratio:  " << (double)mdbfs_failure_count_fairness / total_test << " (" << mdbfs_failure_count_fairness << " / " << total_test << ")\n";
            std::cout << "One or more generate better result ratio:  " << (double)either_better_count_fairness / total_test << " (" << either_better_count_fairness << " / " << total_test << ")\n";
            std::cout << "Both BFS generate better results ratio:  " << (double)both_better_count_fairness / total_test << " (" << both_better_count_fairness << " / " << total_test << ")\n";
            std::cout << "Both failure ratio:  " << (double)both_failure_count_fairness / total_test << " (" << both_failure_count_fairness << " / " << total_test << ")\n";
            std::cout << "MDBFST same as optimal ratio:  " << (double)mdbfs_get_optimal_count_fairness / total_test << " (" << mdbfs_get_optimal_count_fairness << " / " << total_test << ")\n";
            std::cout << "MDBFST better than optimal ratio:  " << (double)mdbfs_bettter_than_optimal_count_fairness / total_test << " (" << mdbfs_bettter_than_optimal_count_fairness << " / " << total_test << ")\n";

            std::cout << "DFST average depth is: " << depth_sum_dfst_fairness / total_test;
            std::cout << "\nBFST average depth is: " << depth_sum_bfst_fairness / total_test;
            std::cout << "\nMDBFST average depth is: " << depth_sum_mdbfs_fairness / total_test;
            std::cout << "\nOptimal average depth is: " << depth_sum_optimal_fairness / total_test << "\n";
            std::cout << "Average Fairness Index (Order Standard Deviation): " << fairness_vec_ordersd_with_fairness_conflict[0] / total_test << ", " << fairness_vec_ordersd_with_fairness_conflict[1] / total_test << ", " << fairness_vec_ordersd_with_fairness_conflict[2] / total_test << "\n";
            std::cout << "Average Fairness Index (Jain's index): " << fairness_vec_jain_with_fairness_conflict[0] / total_test << ", " << fairness_vec_jain_with_fairness_conflict[1] / total_test << ", " << fairness_vec_jain_with_fairness_conflict[2] / total_test << "\n";
        }

    }
    return 0;
}

