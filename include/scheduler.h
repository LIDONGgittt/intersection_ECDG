#ifndef intersection_management_scheduler_h
#define intersection_management_scheduler_h

#include "conflict_directed_graph.h"
#include "conflict_spanning_tree.h"

namespace intersection_management {
    class Scheduler {
    public:
        Scheduler();
        ConflictSpanningTree ScheduleWithModifiedDfst();
        ConflictSpanningTree ScheduleWithBfstWeightedEdgeOnly();
        ConflictSpanningTree ScheduleWithBfstMultiWeight();

        int temp;
    };
}

#endif