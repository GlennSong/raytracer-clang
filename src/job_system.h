#ifndef RAYTRACER_JOB_SYSTEM_H
#define RAYTRACER_JOB_SYSTEM_H

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace engine {

// A minimal task scheduler: a fixed pool of worker threads draining one shared
// queue. The point is to have a single place that owns threads, so the rest of
// the codebase expresses *what* may run in parallel (parallelFor over an image
// today, async asset loads later) without hand-rolling std::thread management.
//
// Deliberately simple (ADR-0014): one mutex-guarded queue, no work stealing.
// The public surface hides the queue, so the internals can grow into per-worker
// deques later without touching callers — but only once a profile asks for it
// (ADR-0008: measure before optimizing).
//
// Determinism (ADR-0002): the scheduler must never change *results*. parallelFor
// is only safe over independent work — each index touches its own data, with no
// order-dependent reductions. It does NOT make SlotMap / World thread-safe, and
// callbacks must not structurally mutate the ECS mid-iteration.
class JobSystem {
public:
    // Spawn this many background worker threads. AUTO picks one fewer than the
    // hardware thread count (leaving a core for the calling thread). 0 is a
    // valid explicit choice: synchronous mode — every task runs inline on the
    // caller and no threads are spawned, which keeps tests deterministic and
    // single-core machines correct.
    static constexpr unsigned AUTO = 0xFFFFFFFFu;
    explicit JobSystem(unsigned workerThreads = AUTO);
    ~JobSystem();

    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    // Split [begin, end) into grain-sized chunks, run them across the pool, and
    // block until all have finished. The calling thread helps run chunks while
    // it waits, so no worker sits idle behind the caller. Small ranges (and
    // synchronous mode) run inline to skip the scheduling overhead.
    void parallelFor(std::size_t begin, std::size_t end,
                     const std::function<void(std::size_t)>& body,
                     std::size_t grainSize = 1);

    // Fire one task. If counter is non-null it is incremented now and
    // decremented when the task finishes, so a later wait(counter) can block on
    // a whole batch of runs. This is the lower-level primitive behind async work.
    void run(std::function<void()> task, std::atomic<int>* counter = nullptr);

    // Block until *counter reaches zero, helping drain the queue meanwhile.
    void wait(std::atomic<int>* counter);

    // Number of background workers actually spawned (0 in synchronous mode).
    unsigned workerCount() const { return static_cast<unsigned>(workers.size()); }

private:
    struct Task {
        std::function<void()> fn;
        std::atomic<int>* counter;
    };

    void workerLoop();

    std::mutex queueMutex;
    std::condition_variable taskAvailable;
    std::condition_variable taskCompleted;
    std::queue<Task> tasks;
    std::vector<std::thread> workers;
    bool stopping = false;
};

// THE PROCESS'S POOL (ADR-0288, #117 item 10): the one JobSystem every part of the engine runs its parallel work
// on -- the app's frame jobs (FrameContext::jobs), the level load's procgen passes (the terrain maps, the weather,
// the forest, the lanes mesher), the pyramid bake, the citysim's population. Made on first use; synchronous (no
// threads) on the web build, which links without -pthread.
JobSystem& sharedJobs();

// Rows [0, n) across the shared pool, each exactly once, in no particular order: `f(row)` must write only its own
// row's data (ADR-0002: the scheduler never changes results). The calling thread helps, so a row loop inside another
// parallel loop cannot deadlock. Rows are batched so a pass over 16 k short rows is not 16 k queue round trips.
template <class F>
void parallelRows(int n, F&& f) {
    if (n <= 0) return;
    JobSystem& jobs = sharedJobs();
    const std::size_t per = std::max<std::size_t>(1, static_cast<std::size_t>(n) / (8 * (jobs.workerCount() + 1)));
    jobs.parallelFor(0, static_cast<std::size_t>(n), [&f](std::size_t r) { f(static_cast<int>(r)); }, per);
}

}  // namespace engine

#endif
