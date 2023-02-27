#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <chrono>

#include "scheduler.h"

using namespace intersection_management;
void test1() {
    Intersection intersection;
    Scheduler scheduler;

    intersection.mt_.seed(0);
    intersection.AddRandomVehicleNodes(5);
    intersection.AddIntersectionUtilitiesFromGeometric();
    intersection.AssignCriticalResourcesToNodes();
    intersection.AssignRoutesToNodes();
    intersection.AssignEdgesWithSafetyOffsetToNodes();

    auto result_tree = scheduler.ScheduleWithDynamicLaneAssignment(intersection);

}

int main() {
    test1();
}