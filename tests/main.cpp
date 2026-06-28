#include "test_framework.h"

#if defined(_WIN32) && defined(_DEBUG)
#include <crtdbg.h>
#include <cstdlib>
// Route debug-CRT and STL (_ITERATOR_DEBUG_LEVEL) assertions to stderr instead
// of a (suppressed) modal dialog, so a failing assert prints its message and
// the offending expression in headless/CI runs instead of silently abort()ing.
static void routeCrtAssertsToStderr() {
    for (int report : {_CRT_WARN, _CRT_ERROR, _CRT_ASSERT}) {
        _CrtSetReportMode(report, _CRTDBG_MODE_FILE);
        _CrtSetReportFile(report, _CRTDBG_FILE_STDERR);
    }
    _set_abort_behavior(0, _WRITE_ABORT_MSG);
}
#endif

// Test cases self-register via TEST_CASE across the other translation units;
// this entry point just runs them all and reports the aggregate result.
int main() {
#if defined(_WIN32) && defined(_DEBUG)
    routeCrtAssertsToStderr();
#endif
    return runAllTests();
}
