#ifndef RAYTRACER_JOB_SYSTEM_H
#define RAYTRACER_JOB_SYSTEM_H

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
// Deliberately simple (ADR-0013): one mutex-guarded queue, no work stealing.
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

}  // namespace engine

// Transitional global alias (ADR-0014); removed once all consumers are
// namespaced. (The physics files that forward-declare JobSystem already use the
// engine:: name, so they don't rely on this.)
using engine::JobSystem;

#endif
