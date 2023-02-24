#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "intersection_utility.h"

using namespace intersection_management;
using namespace ::testing;

class TestNodeOfIntersectionUtility: public Test {
public:
    Node initial_node_;
    std::shared_ptr<Node> p_node_a_;
    std::shared_ptr<Node> p_node_b_;
    std::shared_ptr<Node> p_node_a_copy_;
    std::shared_ptr<Edge> p_edge_a_to_b_;
    void SetUp() override {
        p_node_a_ = std::make_shared<Node>(Node(0, 1, 0, 0, 0));
        p_node_a_copy_ = std::make_shared<Node>(Node(0, 1, 0, 0, 0));
        p_node_b_ = std::make_shared<Node>(Node(1, 1, 0, 0, 0));
        p_edge_a_to_b_ = std::make_shared<Edge>(Edge(p_node_a_, p_node_b_, 2, false));
        p_node_a_->edges_.push_back(p_edge_a_to_b_);
    }
};

TEST_F(TestNodeOfIntersectionUtility, InitialNodeIdIsNegativeOne) {
    EXPECT_THAT(initial_node_.id_, Eq(-1));
}
TEST_F(TestNodeOfIntersectionUtility, CanCheckSameNodeWithId) {
    EXPECT_THAT(p_node_a_->isSameAs(p_node_a_copy_->id_), IsTrue());
}
TEST_F(TestNodeOfIntersectionUtility, CanCheckSameNodeWithRef) {
    EXPECT_THAT(p_node_a_->isSameAs(*p_node_a_copy_), IsTrue());
}
TEST_F(TestNodeOfIntersectionUtility, CanCheckSameNodeWithSharedPrt) {
    EXPECT_THAT(p_node_a_->isSameAs(p_node_a_copy_), IsTrue());
}
TEST_F(TestNodeOfIntersectionUtility, CanCheckConnectionWithId) {
    EXPECT_THAT(p_node_a_->isConnectedTo(p_node_b_->id_), IsTrue());
}
TEST_F(TestNodeOfIntersectionUtility, CanCheckConnectionWithRef) {
    EXPECT_THAT(p_node_a_->isConnectedTo(*p_node_b_), IsTrue());
}
TEST_F(TestNodeOfIntersectionUtility, CanCheckConnectionWithSharedPtr) {
    EXPECT_THAT(p_node_a_->isConnectedTo(p_node_b_), IsTrue());
}
TEST_F(TestNodeOfIntersectionUtility, ReturnNullWhenGettingNonexistEdge) {
    EXPECT_THAT(p_node_a_->getEdgeTo(2), IsNull());
}
TEST_F(TestNodeOfIntersectionUtility, CanReturnCorrectEdgeOfNodes) {
    EXPECT_THAT(p_node_a_->getEdgeTo(1)->node2_.lock()->id_, Eq(p_node_b_->id_));
}

class TestIntersectinUtilytyOfNewAttributes: public Test {
public:
    Node node_;
    Edge edge_;
    std::shared_ptr<Node> p_node;
    std::shared_ptr<Edge> p_edge;
    ConflictType ct;
    void SetUp() override {
        p_node = std::make_shared<Node>(1, 5, 2, 2, 0, 1, 2);
        ct.setConverging();
        ct.setCompeting();
        p_edge = std::make_shared<Edge>(p_node, p_node, -1, ct, 5);
    }
};

TEST_F(TestIntersectinUtilytyOfNewAttributes, CanInitializeNodeCorrectly) {
    EXPECT_THAT(p_node->id_, Eq(1));
    EXPECT_THAT(p_node->estimate_travel_time_, Eq(5.0));
    EXPECT_THAT(p_node->in_leg_id_, Eq(2));
    EXPECT_THAT(p_node->in_lane_id_, Eq(2));
    EXPECT_THAT(p_node->out_leg_id_, Eq(0));
    EXPECT_THAT(p_node->out_lane_id_, Eq(1));
    EXPECT_THAT(p_node->time_window_, Eq(std::vector<double>({-1, -1})));
    EXPECT_THAT(p_node->critical_resource_, IsNull());
}
TEST_F(TestIntersectinUtilytyOfNewAttributes, HasConflictAttributesInEdges) {
    EXPECT_THAT(p_edge->node1_.lock(), Eq(p_node));
    EXPECT_THAT(p_edge->node2_.lock(), Eq(p_node));
    EXPECT_THAT(p_edge->edge_weight_, Eq(1));
    EXPECT_THAT(p_edge->estimate_offset_, Eq(-1));
    EXPECT_THAT(p_edge->predecessor_id_, Eq(5));
    EXPECT_THAT(p_edge->critical_resource_, IsNull());
    EXPECT_THAT(p_edge->conflict_type_.competing_, IsTrue());
    EXPECT_THAT(p_edge->conflict_type_.converging_, IsTrue());
    EXPECT_THAT(p_edge->conflict_type_.crossing_, IsFalse());
    EXPECT_THAT(p_edge->conflict_type_.diverging_, IsFalse());
}

class TestIntersectionUtility: public Test {
public:
    std::shared_ptr<Route> route;
    std::shared_ptr<Leg> leg;
    std::shared_ptr<Lane> lane1, lane2;
    void SetUp() override {
        lane1 = std::make_shared<Lane>(0, 'i');
        lane2 = std::make_shared<Lane>(1, 'o');
        leg = std::make_shared<Leg>(0);
        leg->lanes_in_map_[0] = lane1;
        leg->lanes_out_map_[0] = lane2;
        route = std::make_shared<Route>(lane1, lane2);
    }
};
TEST_F(TestIntersectionUtility, HaveLaneAttributes) {
    EXPECT_THAT(lane1->isInBound(), IsTrue());
    EXPECT_THAT(lane2->isOutBound(), IsTrue());
    EXPECT_THAT(lane1->getId(), Eq(0));
    EXPECT_THAT(lane1->getStreamDirection(), AnyOf(Eq('I'), Eq('i')));
    EXPECT_THAT(lane1->getUniqueId(), Eq(-1));
}
TEST_F(TestIntersectionUtility, HaveLegAttributes) {
    EXPECT_THAT(leg->getId(), Eq(0));
    EXPECT_THAT(leg->getNumLanesIn(), Eq(1));
    EXPECT_THAT(leg->getNumLanesOut(), Eq(1));
}
TEST_F(TestIntersectionUtility, HaveRouteAttributes) {
    EXPECT_THAT(route->getLaneIn(), Eq(lane1));
    EXPECT_THAT(route->getLaneOut(), Eq(lane2));
}
