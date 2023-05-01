#include "sumo_batch_utility.h"
#include "sumo_simulator.h"

#include <csignal>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>

#include "parameters.h"
#include "batch_test_utility.h"
#include "time_utility.h"

namespace intersection_management {

SumoResult sumoBatchTestOneCase(int num_nodes, std::vector<std::string> schedule_methods,
                                double arrival_interval_avg, int geometryID, bool verbose, int seed) {
    SumoResult localResult(schedule_methods);

    for (int i = 0; i < schedule_methods.size(); i++) {
        SumoSimulator sumo_simulator;
        sumo_simulator.setSumoGUI(false);
        sumo_simulator.setSimulateOneMethod(num_nodes, schedule_methods[i], arrival_interval_avg, geometryID, verbose, seed);
        sumo_simulator.startSimulation(verbose);
        for (int m = 0; m < sumo_simulator.scheduler_method_list_.size(); m++) {
            if (schedule_methods[i] == sumo_simulator.scheduler_method_list_[m]) {
                localResult.scheduledStandardDepths_[i] = sumo_simulator.standard_depth_[m];
                break;
            }
        }
        localResult.simEvacuationTime_[i] = sumo_simulator.evacuation_time_;
        localResult.simMaxDelay_[i] = sumo_simulator.maxTimeDelay_;
        localResult.simAveDelay_[i] = sumo_simulator.averageTimeDelay_;
        localResult.simMaxStopTime_[i] = sumo_simulator.maxWaitingTime_;
        localResult.simAveStopTime_[i] = sumo_simulator.averageWaitingTime_;
        localResult.simAveFuelConsumption_[i] = sumo_simulator.averageFuelComsumed_;
    }
    return localResult;
}


void sumoBatchTest(int num_nodes, int test_count, std::vector<std::string> schedule_methods,
                   double arrival_interval_avg, int geometryID, int print_interval, int starting_seed) {
    std::signal(SIGINT, SIGINT_signal_handler);

    SumoResult summationResult(schedule_methods);

    long total_test = 0;
    if (test_count < 0) test_count = INT32_MAX;
    int seed_increment = 1;
    if (starting_seed < 0) {
        starting_seed = 0;
    }

    while (total_test < test_count) {
        total_test++;
        auto sumoResult = sumoBatchTestOneCase(num_nodes, schedule_methods, arrival_interval_avg, geometryID, false, starting_seed);
        starting_seed += seed_increment;

        summationResult += sumoResult;

        if (total_test % print_interval == 0) {
            std::cout << "\n\n##################### Updated result: ##################### \n";
            std::cout << "Total tests: " << total_test << ", Total node num: " << num_nodes << ". \n";
            std::cout << "Schedule method, Standard Depth,  Evacuation Time, Delay (Ave),     Delay (Max),     Stop Time (Ave), Stop Time (Ave), Fuel Consumption \n";
            for (int i = 0; i < schedule_methods.size(); i++) {
                std::cout << std::setw(17) << std::left << schedule_methods[i]
                    << std::setw(17) << std::left << summationResult.scheduledStandardDepths_[i] / (double)total_test
                    << std::setw(17) << std::left << summationResult.simEvacuationTime_[i] / (double)total_test
                    << std::setw(17) << std::left << summationResult.simAveDelay_[i] / (double)total_test
                    << std::setw(17) << std::left << summationResult.simMaxDelay_[i] / (double)total_test
                    << std::setw(17) << std::left << summationResult.simAveStopTime_[i] / (double)total_test
                    << std::setw(17) << std::left << summationResult.simMaxStopTime_[i] / (double)total_test
                    << std::setw(17) << std::left << summationResult.simAveFuelConsumption_[i] / (double)total_test
                    << std::endl;
            }
            std::cout << "\n";
        }
    }



}
} // namespace intersection_management