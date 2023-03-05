#include "batch_test_utility.h"

using namespace intersection_management;

int main() {
    // Check case:
    // seed = 16, vehicle = 5
    // seed = 4, vehicle = 10
    int seed = 16;
    while (true) {
        auto result = BatchTest(5, true, true, seed++);
        if (result[0] > result[4])
            break;

    }
}