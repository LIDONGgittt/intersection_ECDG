#ifndef INTERSECTION_MANAGEMENT_SUMO_BATCH_UTILITY_H_
#define INTERSECTION_MANAGEMENT_SUMO_BATCH_UTILITY_H_

#include <vector>
#include <string>

namespace intersection_management {

class SumoResult {
public:
    SumoResult(): SumoResult({"dynamic_lane", "dfs", "bfs", "mdbfs"}) {}
    SumoResult(std::vector<std::string> scheduler_methods)
        : numSchedulers_(scheduler_methods.size()), scheduler_methods_(scheduler_methods) {
        scheduledStandardDepths_.resize(numSchedulers_, 0);
        simEvacuationTime_.resize(numSchedulers_, 0);
        simMaxDelay_.resize(numSchedulers_, 0);
        simAveDelay_.resize(numSchedulers_, 0);
        simMaxStopTime_.resize(numSchedulers_, 0);
        simAveStopTime_.resize(numSchedulers_, 0);
        simAveFuelConsumption_.resize(numSchedulers_, 0);
    }

    SumoResult &operator+=(const SumoResult &rhs) {
        if (this->numSchedulers_ != rhs.numSchedulers_)
            return *this;
        for (int i = 0; i < this->numSchedulers_; i++) {
            if (this->scheduler_methods_[i] != rhs.scheduler_methods_[i])
                return *this;
        }
        for (int i = 0; i < this->numSchedulers_; i++) {
            this->scheduledStandardDepths_[i] += rhs.scheduledStandardDepths_[i];
            this->simEvacuationTime_[i] += rhs.simEvacuationTime_[i];
            this->simMaxDelay_[i] += rhs.simMaxDelay_[i];
            this->simAveDelay_[i] += rhs.simAveDelay_[i];
            this->simMaxStopTime_[i] += rhs.simMaxStopTime_[i];
            this->simAveStopTime_[i] += rhs.simAveStopTime_[i];
            this->simAveFuelConsumption_[i] += rhs.simAveFuelConsumption_[i];
        }
        return *this;
    }

    int numSchedulers_;
    std::vector<std::string> scheduler_methods_;
    std::vector<double> scheduledStandardDepths_;
    std::vector<double> simEvacuationTime_;
    std::vector<double> simMaxDelay_;
    std::vector<double> simAveDelay_;
    std::vector<double> simMaxStopTime_;
    std::vector<double> simAveStopTime_;
    std::vector<double> simAveFuelConsumption_;
};

SumoResult sumoBatchTestOneCase(int num_nodes, std::vector<std::string> schedule_methods, bool verbose = false, int seed = -1);

void sumoBatchTest(int num_nodes = 5, int test_count = -1,
                   std::vector<std::string> schedule_methods = {"dynamic_lane", "dfs", "bfs", "mdbfs"},
                   double arrival_interval_avg = 2.0, int print_interval = 10, int starting_seed = -1);

} // namespace intersection_management

#endif // INTERSECTION_MANAGEMENT_SUMO_BATCH_UTILITY_H_