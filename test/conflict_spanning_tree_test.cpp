#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "conflict_spanning_tree.h"

using namespace intersection_management;
using namespace ::testing;

class TestConflictSpanningTree : public Test {
public:
    ConflictSpanningTree cst_;
    void SetUp() override {
        cst_.AddNode(2);
        cst_.AddNode(3);
        cst_.AddNode(4);
        cst_.AddNode(5);
        cst_.AddEdge(0,1,1);
        cst_.AddEdge(0,2,1);
        cst_.AddEdge(0,3,1);
    }
};

TEST_F(TestConflictSpanningTree, RecordTreeNodeNumbers) {
    EXPECT_THAT(cst_.num_nodes_, Eq(4));
}
TEST_F(TestConflictSpanningTree, UpdateCorrectDepth) {
    cst_.UpdateDepth(1, 10, EdgeWeightedDepth);
    EXPECT_THAT(cst_.nodes_[1]->edge_weighted_depth_, Eq(10));
    EXPECT_THAT(cst_.edge_weighted_depth_, Eq(10));
}