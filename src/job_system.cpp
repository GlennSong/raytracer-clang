#include "job_system.h"
#include "profile.h"

#include <algorithm>

namespace engine {

JobSystem::JobSystem(unsigned workerThreads) {
    unsigned count = workerThreads;
    if (count == AUTO) {
        unsigned hardware = std::thread::hardware_concurrency();
        // Leave a core for the calling thread; hardware==0/1 falls back to
        // synchronous (no workers) rather than spawning a lone, contending one.
        count = hardware > 1 ? hardware - 1 : 0;
    }
    workers.reserve(count);
    for (unsigned i = 0; i < count; ++i) {
        workers.emplace_back([this] { workerLoop(); });
    }
}

JobSystem::~JobSystem() {
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        stopping = true;
    }
    taskAvailable.notify_all();
    for (std::thread& worker : workers) {
        worker.join();
    }
}

void JobSystem::workerLoop() {
    RT_PROFILE_THREAD("JobSystem worker");
    std::unique_lock<std::mutex> lock(queueMutex);
    while (true) {
        taskAvailable.wait(lock, [this] { return stopping || !tasks.empty(); });
        if (stopping && tasks.empty()) {
            return;
        }
        Task task = std::move(tasks.front());
        tasks.pop();

        lock.unlock();
        task.fn();
        lock.lock();

        if (task.counter) {
            task.counter->fetch_sub(1, std::memory_order_release);
            taskCompleted.notify_all();
        }
    }
}

void JobSystem::run(std::function<void()> task, std::atomic<int>* counter) {
    if (counter) {
        counter->fetch_add(1, std::memory_order_relaxed);
    }
    if (workers.empty()) {
        // Synchronous mode: nowhere to defer to, so run inline on the caller.
        task();
        if (counter) {
            counter->fetch_sub(1, std::memory_order_release);
        }
        return;
    }
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        tasks.push(Task{std::move(task), counter});
    }
    taskAvailable.notify_one();
}

void JobSystem::wait(std::atomic<int>* counter) {
    if (!counter) {
        return;
    }
    std::unique_lock<std::mutex> lock(queueMutex);
    while (counter->load(std::memory_order_acquire) > 0) {
        if (!tasks.empty()) {
            // Help drain the queue instead of idling behind the workers.
            Task task = std::move(tasks.front());
            tasks.pop();

            lock.unlock();
            task.fn();
            lock.lock();

            if (task.counter) {
                task.counter->fetch_sub(1, std::memory_order_release);
                taskCompleted.notify_all();
            }
        } else {
            // Nothing left to help with; a worker still holds an in-flight task
            // for this counter and will signal when it finishes. The decrement
            // and this wait both happen under queueMutex, so no wakeup is lost.
            taskCompleted.wait(lock);
        }
    }
}

void JobSystem::parallelFor(std::size_t begin, std::size_t end,
                            const std::function<void(std::size_t)>& body,
                            std::size_t grainSize) {
    RT_PROFILE_ZONE_NAMED("JobSystem::parallelFor");
    if (begin >= end) {
        return;
    }
    if (grainSize == 0) {
        grainSize = 1;
    }
    if (workers.empty() || end - begin <= grainSize) {
        for (std::size_t i = begin; i < end; ++i) {
            body(i);
        }
        return;
    }
    // body outlives the call: we block in wait() before returning, so the
    // by-reference capture stays valid for every chunk.
    std::atomic<int> counter{0};
    for (std::size_t chunkStart = begin; chunkStart < end; chunkStart += grainSize) {
        std::size_t chunkEnd = std::min(chunkStart + grainSize, end);
        run([&body, chunkStart, chunkEnd] {
            for (std::size_t i = chunkStart; i < chunkEnd; ++i) {
                body(i);
            }
        }, &counter);
    }
    wait(&counter);
}

}  // namespace engine
