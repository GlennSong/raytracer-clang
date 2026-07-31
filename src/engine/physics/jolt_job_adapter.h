#ifndef RAYTRACER_ENGINE_PHYSICS_JOLT_JOB_ADAPTER_H
#define RAYTRACER_ENGINE_PHYSICS_JOLT_JOB_ADAPTER_H

// Lets Jolt run its simulation jobs on OUR JobSystem (ADR-0014) instead of
// spawning a second thread pool. Jolt owns the job *graph* (dependencies and
// barriers — handled by the JobSystemWithBarrier base); we own the *threads*.
// The only real integration point is QueueJob, which forwards a ready-to-run
// job to JobSystem::run; the calling thread also helps via the base barrier's
// Wait(), so concurrency is workers + 1.
//
// Internal to the physics module: it sits behind the ADR-0012 Jolt seal and is
// included only by physics_world.cpp (which already pulls in Jolt) and the
// physics tests — never by engine/game code.

#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemWithBarrier.h>
#include <Jolt/Core/FixedSizeFreeList.h>

#include <atomic>

namespace engine {

class JobSystem;   // our thread pool (src/job_system.h)

class JoltJobAdapter final : public JPH::JobSystemWithBarrier {
public:
    // Our pool is spelled engine::JobSystem throughout: inside a
    // JPH::JobSystem-derived class, an unqualified "JobSystem" would resolve to
    // the Jolt base. pool must outlive this adapter (owned by e.g. Application).
    JoltJobAdapter(engine::JobSystem& pool, JPH::uint maxJobs, JPH::uint maxBarriers);

    // Blocks until every job this adapter queued has finished.
    //
    // NOT optional, and not merely tidy. A queued job holds a raw Job* into the
    // `jobs` free list below, and after running it calls Release() -> FreeJob()
    // -> jobs.DestructObject(). Job::Execute() signals the barrier BEFORE the
    // worker reaches Release(), so a thread that passes the barrier can return
    // from PhysicsSystem::Update, drop PhysicsWorld, and destroy this adapter
    // while a worker is still inside that window — leaving Release() to write
    // into a destroyed free list. That is a use-after-free, and it crashed
    // physics_tests twice under CPU contention (SIGSEGV at null+0x48).
    //
    // Jolt's own JobSystemThreadPool upholds the same contract by joining its
    // threads and draining the queue in its destructor ("Ensure that there are
    // no lingering jobs in the queue"). We cannot join — the threads belong to
    // the shared engine::JobSystem, which outlives us — so we wait on a counter
    // instead. The pool decrements it only after the task body returns, which
    // includes Release(), so zero really does mean nothing can touch us again.
    ~JoltJobAdapter() override;

    int GetMaxConcurrency() const override;
    JobHandle CreateJob(const char* inName, JPH::ColorArg inColor,
                        const JobFunction& inJobFunction,
                        JPH::uint32 inNumDependencies = 0) override;

protected:
    void QueueJob(Job* inJob) override;
    void QueueJobs(Job** inJobs, JPH::uint inNumJobs) override;
    void FreeJob(Job* inJob) override;

private:
    engine::JobSystem& pool;

    // Jobs handed to the pool that have not finished running yet. The pool
    // increments this when a task is queued and decrements it after the task
    // body returns, so the destructor can wait for it to reach zero.
    std::atomic<int> outstanding{0};

    // Fixed-size storage for in-flight jobs, exactly as the stock Jolt job
    // systems use; sized to maxJobs at construction. Declared AFTER `outstanding`
    // so it is destroyed first in reverse-declaration order — irrelevant while
    // the destructor drains, but it keeps the ordering honest if that changes.
    using AvailableJobs = JPH::FixedSizeFreeList<Job>;
    AvailableJobs jobs;
};


}  // namespace engine

#endif
