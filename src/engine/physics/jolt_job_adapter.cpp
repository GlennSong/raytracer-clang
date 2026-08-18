#include "jolt_job_adapter.h"

#include "../../job_system.h"
#include "../../log.h"

#include <chrono>
#include <thread>

namespace engine {

JoltJobAdapter::JoltJobAdapter(engine::JobSystem& pool, JPH::uint maxJobs, JPH::uint maxBarriers)
    : JPH::JobSystemWithBarrier(maxBarriers), pool(pool) {
    jobs.Init(maxJobs, maxJobs);
}

JoltJobAdapter::~JoltJobAdapter() {
    // See the header: a worker can still be between Execute() (which released
    // the barrier our caller was waiting on) and Release() (which frees the job
    // out of `jobs`). Leaving before it gets there frees the storage underneath
    // it. wait() also drains the queue on this thread, so a job that has not
    // started yet runs here rather than against a half-destroyed adapter.
    pool.wait(&outstanding);
}

int JoltJobAdapter::GetMaxConcurrency() const {
    // Our background workers, plus the thread that calls WaitForJobs — the base
    // barrier's Wait() executes jobs on it while it waits, so it counts too.
    return static_cast<int>(pool.workerCount()) + 1;
}

JPH::JobSystem::JobHandle JoltJobAdapter::CreateJob(const char* inName,
        JPH::ColorArg inColor, const JobFunction& inJobFunction,
        JPH::uint32 inNumDependencies) {
    // Allocate the job, wrap it in a handle that holds a reference, and queue
    // it now if nothing blocks it. The free list CAN come up empty: the whole
    // 2048 budget in flight at once during a SimClock catch-up burst, plus a
    // transient window where finished jobs sit between Execute() and the
    // worker's Release(). The first cut guarded that with JPH_ASSERT only —
    // compiled out in release, so `Get(cInvalidObjectIndex)` shipped a WILD
    // Job* that crashed at 0x27ff0 three times in one evening (twice in
    // CreateJob's construct, once later in sRemoveDependencies through the
    // returned handle). Jolt's own JobSystemThreadPool wraps this exact case
    // in a wait-and-retry loop ("No jobs available!"); mirror it. Retrying is
    // correct, not merely safe: slots free as soon as workers finish Release,
    // which needs no cooperation from this thread.
    // CAVEAT on the retry: it only helps when a slot can actually free — the
    // cross-frame transient where finished jobs sit between Execute() and a
    // worker's Release(). Exhaustion INSIDE one Update's graph cannot be
    // waited out (the barrier holds every job's ref until WaitForJobs, which
    // the creating thread hasn't reached), which is why maxJobs carries 4x
    // headroom at the construction site — the retry is a backstop, and the
    // escalating WARN below is how a mis-sized pool announces itself as a
    // diagnosable stall instead of memory corruption.
    JPH::uint32 index;
    for (int attempt = 0;; ++attempt) {
        index = jobs.ConstructObject(inName, inColor, this, inJobFunction,
                                     inNumDependencies);
        if (index != AvailableJobs::cInvalidObjectIndex) break;
        if (attempt == 0)
            LOG_WARN << "JoltJobAdapter: job pool exhausted; waiting for a slot";
        else if (attempt == 1000)   // ~100 ms of spinning: this is not transient
            LOG_ERROR << "JoltJobAdapter: pool exhausted >100ms — one physics "
                         "step needs more jobs than maxJobs; raise it";
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    Job* job = &jobs.Get(index);

    JobHandle handle(job);
    if (inNumDependencies == 0) {
        QueueJob(job);
    }
    return handle;
}

void JoltJobAdapter::QueueJob(Job* inJob) {
    // The job is only guaranteed alive for the duration of this call, so take a
    // reference to carry it onto a worker thread (JobSystem.h contract); the
    // worker releases it after Execute(), which triggers FreeJob when it hits 0.
    inJob->AddRef();
    // Counted, so ~JoltJobAdapter can wait for it. The pool decrements only
    // after this lambda returns — i.e. after Release() — which is what makes the
    // wait sufficient rather than merely likely.
    pool.run([inJob] {
        inJob->Execute();
        inJob->Release();
    }, &outstanding);
}

void JoltJobAdapter::QueueJobs(Job** inJobs, JPH::uint inNumJobs) {
    for (JPH::uint i = 0; i < inNumJobs; ++i) {
        QueueJob(inJobs[i]);
    }
}

void JoltJobAdapter::FreeJob(Job* inJob) {
    jobs.DestructObject(inJob);
}

}  // namespace engine

