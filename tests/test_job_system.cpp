#include "test_framework.h"

#include "../src/job_system.h"

#include <atomic>
#include <vector>

namespace {

// A representative workload size: large enough to span many chunks/threads.
constexpr std::size_t N = 10000;

int compute(std::size_t i) {
    return static_cast<int>(i * 3 + 1);
}

}  // namespace

TEST_CASE(job_system_parallel_for_covers_every_index_once) {
    JobSystem jobs;  // auto worker count
    std::vector<int> hits(N, 0);

    jobs.parallelFor(0, N, [&](std::size_t i) { hits[i] += 1; }, 64);

    bool allHitOnce = true;
    for (std::size_t i = 0; i < N; ++i) {
        if (hits[i] != 1) {
            allHitOnce = false;
        }
    }
    CHECK(allHitOnce);
}

TEST_CASE(job_system_parallel_for_writes_disjoint_results) {
    // Determinism (ADR-0002): writing one value per index must match the serial
    // result regardless of how chunks are scheduled across threads.
    JobSystem jobs;
    std::vector<int> out(N, -1);

    jobs.parallelFor(0, N, [&](std::size_t i) { out[i] = compute(i); }, 128);

    bool matchesSerial = true;
    for (std::size_t i = 0; i < N; ++i) {
        if (out[i] != compute(i)) {
            matchesSerial = false;
        }
    }
    CHECK(matchesSerial);
}

TEST_CASE(job_system_synchronous_mode_runs_inline) {
    JobSystem jobs(0);  // 0 workers => synchronous
    CHECK(jobs.workerCount() == 0);

    std::vector<int> out(N, -1);
    jobs.parallelFor(0, N, [&](std::size_t i) { out[i] = compute(i); }, 256);

    bool correct = true;
    for (std::size_t i = 0; i < N; ++i) {
        if (out[i] != compute(i)) {
            correct = false;
        }
    }
    CHECK(correct);
}

TEST_CASE(job_system_run_and_wait_completes_every_task) {
    JobSystem jobs;
    std::atomic<int> completed{0};
    std::atomic<int> counter{0};

    const int taskCount = 500;
    for (int i = 0; i < taskCount; ++i) {
        jobs.run([&] { completed.fetch_add(1, std::memory_order_relaxed); }, &counter);
    }
    jobs.wait(&counter);

    CHECK(completed.load() == taskCount);
    CHECK(counter.load() == 0);
}

TEST_CASE(job_system_run_and_wait_works_synchronously) {
    JobSystem jobs(0);
    std::atomic<int> completed{0};
    std::atomic<int> counter{0};

    const int taskCount = 50;
    for (int i = 0; i < taskCount; ++i) {
        jobs.run([&] { completed.fetch_add(1, std::memory_order_relaxed); }, &counter);
    }
    jobs.wait(&counter);

    CHECK(completed.load() == taskCount);
    CHECK(counter.load() == 0);
}

TEST_CASE(job_system_parallel_for_empty_range_is_noop) {
    JobSystem jobs;
    int calls = 0;
    jobs.parallelFor(5, 5, [&](std::size_t) { calls += 1; });
    CHECK(calls == 0);
}

TEST_CASE(job_system_parallel_for_grain_larger_than_range) {
    // grainSize beyond the range runs the whole thing in one inline pass, but
    // must still visit every index exactly once.
    JobSystem jobs;
    std::vector<int> hits(10, 0);
    jobs.parallelFor(0, 10, [&](std::size_t i) { hits[i] += 1; }, 1000);

    bool allHitOnce = true;
    for (int h : hits) {
        if (h != 1) {
            allHitOnce = false;
        }
    }
    CHECK(allHitOnce);
}

TEST_CASE(job_system_parallel_for_ragged_chunks_cover_range) {
    // Range length not a multiple of the grain — the final short chunk must be
    // covered too.
    JobSystem jobs;
    const std::size_t count = 1003;
    std::vector<int> hits(count, 0);
    jobs.parallelFor(0, count, [&](std::size_t i) { hits[i] += 1; }, 64);

    bool allHitOnce = true;
    for (std::size_t i = 0; i < count; ++i) {
        if (hits[i] != 1) {
            allHitOnce = false;
        }
    }
    CHECK(allHitOnce);
}
