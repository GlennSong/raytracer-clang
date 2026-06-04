#ifndef RAYTRACER_TEST_FRAMEWORK_H
#define RAYTRACER_TEST_FRAMEWORK_H

// A deliberately tiny test harness: standard library only, no external deps
// (per AGENTS.md). Define cases with TEST_CASE; assert with CHECK / CHECK_APPROX.
// A case passes if it logs no failed checks. main() lives in tests/main.cpp and
// just calls runAllTests(), which returns nonzero if any check failed.

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& testRegistry() {
    static std::vector<TestCase> registry;
    return registry;
}

inline int& totalFailedChecks() {
    static int count = 0;
    return count;
}

inline int& currentCaseFailedChecks() {
    static int count = 0;
    return count;
}

struct TestRegistrar {
    TestRegistrar(const std::string& name, std::function<void()> fn) {
        testRegistry().push_back({name, std::move(fn)});
    }
};

#define TEST_CASE(name)                                                       \
    static void name();                                                       \
    static TestRegistrar registrar_##name(#name, name);                       \
    static void name()

#define CHECK(cond)                                                           \
    do {                                                                      \
        if (!(cond)) {                                                        \
            std::printf("    FAIL %s:%d  CHECK(%s)\n", __FILE__, __LINE__,     \
                        #cond);                                               \
            ++totalFailedChecks();                                            \
            ++currentCaseFailedChecks();                                      \
        }                                                                     \
    } while (0)

#define CHECK_APPROX(actual, expected, eps)                                   \
    do {                                                                      \
        double actualValue = (actual);                                        \
        double expectedValue = (expected);                                    \
        if (std::fabs(actualValue - expectedValue) > (eps)) {                 \
            std::printf("    FAIL %s:%d  CHECK_APPROX(%s, %s)  %g vs %g\n",     \
                        __FILE__, __LINE__, #actual, #expected, actualValue,  \
                        expectedValue);                                       \
            ++totalFailedChecks();                                            \
            ++currentCaseFailedChecks();                                      \
        }                                                                     \
    } while (0)

inline int runAllTests() {
    int passedCases = 0;
    const int totalCases = static_cast<int>(testRegistry().size());
    for (auto& testCase : testRegistry()) {
        currentCaseFailedChecks() = 0;
        std::printf("[ RUN  ] %s\n", testCase.name.c_str());
        testCase.fn();
        if (currentCaseFailedChecks() == 0) {
            std::printf("[  OK  ] %s\n", testCase.name.c_str());
            ++passedCases;
        } else {
            std::printf("[ FAIL ] %s (%d checks failed)\n", testCase.name.c_str(),
                        currentCaseFailedChecks());
        }
    }
    std::printf("\n%d/%d cases passed, %d checks failed\n", passedCases,
                totalCases, totalFailedChecks());
    return totalFailedChecks() == 0 ? 0 : 1;
}

#endif
