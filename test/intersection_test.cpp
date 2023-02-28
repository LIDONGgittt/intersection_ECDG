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
        intersection.num_legs_ = 4;
        intersection.num_lanes_in_vec_ = std::vector<int>({1, 1, 2, 1});
        intersection.num_lanes_out_vec_ = std::vector<int>({2, 1, 1, 1});
    }
};

TEST_F(TestIntersection, CanInitializeIntersectionObject) {
    EXPECT_NO_THROW(Intersection intersection);
}
TEST_F(TestIntersection, AllDirectionsHaveInAndOutLanes) {
    EXPECT_THAT(intersection.num_lanes_in_vec_.size(), Eq(intersection.num_legs_));
    EXPECT_THAT(intersection.num_lanes_out_vec_.size(), Eq(intersection.num_legs_));
}
TEST_F(TestIntersection, CanAddNodes) {
    std::shared_ptr<Node> node = std::make_shared<Node>();
    intersection.AddNode(node);
    EXPECT_THAT(intersection.getNumNodes(), Eq(2)); // include default virtual leading vehicle
}
TEST_F(TestIntersection, CanAddEdges) {
    std::shared_ptr<Edge> edge = std::make_shared<Edge>();
    intersection.AddEdge(edge);
    EXPECT_THAT(intersection.getNumEdges(), Eq(1));
}
TEST_F(TestIntersection, CanAddIntersectionUtilitiesFromGeometric) {
    intersection.AddIntersectionUtilitiesFromGeometric();
    EXPECT_THAT(intersection.getNumCriticalResources(), Eq(1));
    EXPECT_THAT(intersection.getNumLegs(), Eq(4));
    EXPECT_THAT(intersection.getNumLanes(), Eq(10));
}
TEST_F(TestIntersection, CanAddRandomVehicleNode) {
    intersection.AddRandomVehicleNodes(10);
    EXPECT_THAT(intersection.getNumNodes(), Eq(11)); // include default virtual leading vehicle
}

class TestIntersectionWithNodes: public Test {
public:
    Intersection intersection;
    void SetUp() override {
        intersection.num_legs_ = 4;
        intersection.num_lanes_in_vec_ = std::vector<int>({1, 1, 2, 1});
        intersection.num_lanes_out_vec_ = std::vector<int>({2, 1, 1, 1});
        intersection.mt_.seed(0);
        intersection.AddIntersectionUtilitiesFromGeometric();

        intersection.AddRandomVehicleNodes(5);
        intersection.AssignCriticalResourcesToNodes();
        intersection.AssignRoutesToNodes();
        intersection.AssignEdgesWithSafetyOffsetToNodes();
    }
};
TEST_F(TestIntersectionWithNodes, CanAssignCriticalResources) {
    EXPECT_THAT(intersection.critical_resource_map_.at(0)->nodes_.size(), Eq(2));
}
TEST_F(TestIntersectionWithNodes, CanAssignRouteToNodes) {
    EXPECT_THAT(intersection.nodes_[1]->route_->getLaneIn(), NotNull());
    EXPECT_THAT(intersection.nodes_[1]->route_->getLaneOut(), NotNull());
}
TEST_F(TestIntersectionWithNodes, CanAssignEdgesToNodes) {
    EXPECT_THAT(intersection.nodes_[1]->getEdgeWith(0)->node1_.lock()->id_, Eq(0));
}
TEST_F(TestIntersectionWithNodes, CanAssignCorrectEdgesToNodes) {
    EXPECT_THAT(intersection.nodes_[1]->getEdgeWith(5)->conflict_type_.isConverging(), IsTrue());
    EXPECT_THAT(intersection.nodes_[2]->getEdgeWith(4)->conflict_type_.isDiverging(), IsTrue());
    EXPECT_THAT(intersection.nodes_[2]->getEdgeWith(4)->predecessor_id_, Eq(2));
    EXPECT_THAT(intersection.nodes_[3]->getEdgeWith(5)->conflict_type_.isDiverging(), IsTrue());
    EXPECT_THAT(intersection.nodes_[3]->getEdgeWith(5)->predecessor_id_, Eq(3));
    EXPECT_THAT(intersection.nodes_[3]->getEdgeWith(4)->conflict_type_.isCompeting(), IsFalse());
    EXPECT_THAT(intersection.nodes_[4]->getEdgeWith(5)->conflict_type_.isCrossing(), IsTrue());
}