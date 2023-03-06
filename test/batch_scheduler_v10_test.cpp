#include <gtest/gtest.h>

#include "batch_test_utility.h"

using namespace intersection_management;

TEST(BatchSchedulerTest, CompareDynamicLaneAssignments10NodesMultipleCase) {
    BatchTest(10, -1, 10);
}