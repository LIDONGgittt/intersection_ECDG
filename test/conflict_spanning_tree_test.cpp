#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "cdg_conflict_spanning_tree.h"

using namespace intersection_management;
using namespace ::testing;

class TestConflictSpanningTree : public Test {
public:
    CDGConflictSpanningTree cst_;
    void SetUp() override {
        cst_.AddNode(2);
        cst_.AddNode(3);
        cst_.AddNode(4);
        cst_.AddNode(5);
        cst_.AddEdge(0, 3, 1);
        cst_.AddEdge(3, 2, 1);
        cst_.AddEdge(2, 1, 1);
        cst_.UpdateDepth(0, 0, Type_EdgeWeightedDepth);
        cst_.UpdateDepth(1, 3, Type_EdgeWeightedDepth);
        cst_.UpdateDepth(2, 2, Type_EdgeWeightedDepth);
        cst_.UpdateDepth(3, 1, Type_EdgeWeightedDepth);
    }
};

TEST_F(TestConflictSpanningTree, RecordTreeNodeNumbers) {
    EXPECT_THAT(cst_.num_nodes_, Eq(4));
}
TEST_F(TestConflictSpanningTree, UpdateCorrectDepth) {
    cst_.UpdateDepth(1, 10, Type_EdgeWeightedDepth);
    EXPECT_THAT(cst_.nodes_[1]->edge_weighted_depth_, Eq(10));
    EXPECT_THAT(cst_.edge_weighted_depth_, Eq(10));
}
TEST_F(TestConflictSpanningTree, CalculateOrderStandardErrorFairnessIndex) {
    EXPECT_THAT(cst_.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_OrderStandardDeviation), Eq(8.0/3.0));
}
TEST_F(TestConflictSpanningTree, CalculateJainFairnessIndex) {
    EXPECT_THAT(cst_.CalculateFairnessIndex(Type_EdgeWeightedDepth, Type_JainIndex), Eq(0.6));
}