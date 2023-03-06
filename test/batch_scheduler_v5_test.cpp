#include <gtest/gtest.h>

#include "batch_test_utility.h"

using namespace intersection_management;

TEST(BatchSchedulerTest, CompareDynamicLaneAssignments5NodesMultipleCase) {
    BatchTest(5, -1, 1000);
}