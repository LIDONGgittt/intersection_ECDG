#include "batch_test_utility.h"
#include "parameters.h"

namespace intersection_management {
extern Parameters param;
}
using namespace intersection_management;

int main() {
    param.activate_precedent_offset = false;
    BatchTest(5, -1, 1000);
}