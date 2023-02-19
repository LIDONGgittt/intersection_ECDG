#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "intersection_utility.h"

using namespace intersection_management;
using namespace ::testing;

class TestIntersectionUtility : public Test {
public:
    Intersection intersection;
    void SetUp() override {
    }
};

TEST_F(TestIntersectionUtility, CanInitializeIntersectionObject) {
    EXPECT_NO_THROW(Intersection intersection);
}