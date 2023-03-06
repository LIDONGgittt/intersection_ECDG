#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "spanning_tree.h"
#include "intersection.h"
#include "intersection_utility.h"

using namespace intersection_management;
using namespace ::testing;

class TestSpanningTree: public Test {
public:
    Intersection intersection;
    SpanningTree spanning_tree;
    void SetUp() override {
        intersection.num_legs_ = 4;
        intersection.num_lanes_in_vec_ = std::vector<int>({1, 1, 2, 1});
        intersection.num_lanes_out_vec_ = std::vector<int>({2, 1, 1, 1});
        intersection.setSeed(0);
        intersection.AddIntersectionUtilitiesFromGeometric();
        intersection.AddRandomVehicleNodes(5);

        intersection.AssignCriticalResourcesToNodes();
        intersection.AssignRoutesToNodes();
        intersection.AssignEdgesWithSafetyOffsetToNodes();
    }
};
TEST_F(TestSpanningTree, CanAddNodesFromIntersection) {
    EXPECT_NO_THROW(spanning_tree.AddNodesFromIntersection(intersection));
}