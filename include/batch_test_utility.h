#ifndef INTERSECTION_MANAGEMENT_BATCH_TEST_UTILITY_H_
#define INTERSECTION_MANAGEMENT_BATCH_TEST_UTILITY_H_

#include <vector>

namespace intersection_management {
std::vector<double> BatchTestOneCase(int num_nodes, bool verbose = false, int seed = -1);

void BatchTest(int num_nodes = 5, int test_count = -1, int print_interval = 1000, int starting_seed = -1);

void SIGINT_signal_handler(int signal);
} // namespace intersection_management

#endif // INTERSECTION_MANAGEMENT_BATCH_TEST_UTILITY_H_