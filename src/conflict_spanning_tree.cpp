#include "conflict_spanning_tree.h"
#include <cstdlib>
#include <time.h>
#include <iostream>
#include <cmath>
#include <queue>

namespace intersection_management {
    ConflictSpanningTree::ConflictSpanningTree() {
        p_root_ = nullptr;
        nodes_.clear();
        edges_.clear();
        count_node_ = 0;
    }
}