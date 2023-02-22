#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "intersection.h"
#include "intersection_utility.h"

using namespace intersection_management;
using namespace ::testing;

class TestIntersection: public Test {
public:
    Intersection intersection;
    void SetUp() override {
        intersection.num_directions_ = 4;
        intersection.num_lanes_in_ = std::vector<int>({2, 1, 1, 1});
        intersection.num_lanes_out_ = std::vector<int>({1, 1, 2, 1});
    }
};

TEST_F(TestIntersection, CanInitializeIntersectionObject) {
    EXPECT_NO_THROW(Intersection intersection);
}
TEST_F(TestIntersection, AllDirectionsHaveInAndOutLanes) {
    EXPECT_THAT(intersection.num_lanes_in_.size(), Eq(intersection.num_directions_));
    EXPECT_THAT(intersection.num_lanes_out_.size(), Eq(intersection.num_directions_));
}
TEST_F(TestIntersection, CanAddNodes) {
    std::shared_ptr<Node> node = std::make_shared<Node>();
    intersection.AddNode(node);
    EXPECT_THAT(intersection.getNumNodes(), Eq(1));
}
TEST_F(TestIntersection, CanAddEdges) {
    std::shared_ptr<Edge> edge = std::make_shared<Edge>();
    intersection.AddEdge(edge);
    EXPECT_THAT(intersection.getNumEdges(), Eq(1));
}
TEST_F(TestIntersection, CanAddCriticalResourceFromGeometric) {
    intersection.AddCriticalResourceFromGeometric();
    EXPECT_THAT(intersection.getNumCriticalResource(), Eq(1));
}
TEST_F(TestIntersection, CanAddRandomVehicleNode) {
    intersection.AddRandomVehicleNodes(10);
    EXPECT_THAT(intersection.getNumNodes(), Eq(10));
}
