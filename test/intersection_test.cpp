#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "intersection.h"

using namespace intersection_management;
using namespace ::testing;

class TestIntersectionUtility: public Test {
public:
    Intersection intersection;
    void SetUp() override {
    }
};

TEST_F(TestIntersectionUtility, CanInitializeIntersectionObject) {
    EXPECT_NO_THROW(Intersection intersection);
}
TEST_F(TestIntersectionUtility, AllDirectionsHaveInAndOutLanes) {
    EXPECT_THAT(intersection.num_lanes_in_.size(), Eq(intersection.num_directions_));
    EXPECT_THAT(intersection.num_lanes_out_.size(), Eq(intersection.num_directions_));
}
