#ifndef INTERSECTION_MANAGEMENT_BATCH_TEST_UTILITY_H_
#define INTERSECTION_MANAGEMENT_BATCH_TEST_UTILITY_H_

#include "scheduler.h"
#include "conflict_directed_graph.h"
#include "cdg_scheduler.h"

#include <vector>

namespace intersection_management {
std::vector<double> BatchTest(int num_nodes, bool enable_precedence_offset = true, bool verbose = false, int seed = -1);
} // namespace intersection_management

#endif // INTERSECTION_MANAGEMENT_BATCH_TEST_UTILITY_H_