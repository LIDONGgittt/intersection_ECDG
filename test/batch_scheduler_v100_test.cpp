#include <gtest/gtest.h>

#include "batch_test_utility.h"

using namespace intersection_management;

TEST(BatchSchedulerTest, CompareDynamicLaneAssignments100NodesMultipleCase) {
    BatchTest(100, -1, 10);
}