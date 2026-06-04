#include "test_framework.h"

// Test cases self-register via TEST_CASE across the other translation units;
// this entry point just runs them all and reports the aggregate result.
int main() {
    return runAllTests();
}
