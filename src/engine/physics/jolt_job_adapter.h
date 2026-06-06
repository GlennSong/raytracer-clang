#ifndef RAYTRACER_ENGINE_PHYSICS_JOLT_JOB_ADAPTER_H
#define RAYTRACER_ENGINE_PHYSICS_JOLT_JOB_ADAPTER_H

// Lets Jolt run its simulation jobs on OUR JobSystem (ADR-0013) instead of
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

namespace engine {

class JobSystem;   // our thread pool (src/job_system.h)

class JoltJobAdapter final : public JPH::JobSystemWithBarrier {
public:
    // Our pool is spelled engine::JobSystem throughout: inside a
    // JPH::JobSystem-derived class, an unqualified "JobSystem" would resolve to
    // the Jolt base. pool must outlive this adapter (owned by e.g. Application).
    JoltJobAdapter(engine::JobSystem& pool, JPH::uint maxJobs, JPH::uint maxBarriers);

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

    // Fixed-size storage for in-flight jobs, exactly as the stock Jolt job
    // systems use; sized to maxJobs at construction.
    using AvailableJobs = JPH::FixedSizeFreeList<Job>;
    AvailableJobs jobs;
};


}  // namespace engine

#endif
