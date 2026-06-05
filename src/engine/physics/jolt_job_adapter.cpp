#include "jolt_job_adapter.h"

#include "../../job_system.h"

JoltJobAdapter::JoltJobAdapter(engine::JobSystem& pool, JPH::uint maxJobs, JPH::uint maxBarriers)
    : JPH::JobSystemWithBarrier(maxBarriers), pool(pool) {
    jobs.Init(maxJobs, maxJobs);
}

int JoltJobAdapter::GetMaxConcurrency() const {
    // Our background workers, plus the thread that calls WaitForJobs — the base
    // barrier's Wait() executes jobs on it while it waits, so it counts too.
    return static_cast<int>(pool.workerCount()) + 1;
}

JPH::JobSystem::JobHandle JoltJobAdapter::CreateJob(const char* inName,
        JPH::ColorArg inColor, const JobFunction& inJobFunction,
        JPH::uint32 inNumDependencies) {
    // Mirror JobSystemSingleThreaded::CreateJob: allocate the job, wrap it in a
    // handle that holds a reference, and queue it now if nothing blocks it.
    JPH::uint32 index =
        jobs.ConstructObject(inName, inColor, this, inJobFunction, inNumDependencies);
    JPH_ASSERT(index != AvailableJobs::cInvalidObjectIndex);
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
    pool.run([inJob] {
        inJob->Execute();
        inJob->Release();
    });
}

void JoltJobAdapter::QueueJobs(Job** inJobs, JPH::uint inNumJobs) {
    for (JPH::uint i = 0; i < inNumJobs; ++i) {
        QueueJob(inJobs[i]);
    }
}

void JoltJobAdapter::FreeJob(Job* inJob) {
    jobs.DestructObject(inJob);
}
