#ifndef RAYTRACER_ENGINE_RESIDENCY_H
#define RAYTRACER_ENGINE_RESIDENCY_H

// THE RESIDENCY SERVICE (ADR-0095 §4, docs/world-streaming-plan.md).
//
// What is on the GPU is decided here, by where the camera is -- not by what the level
// happened to load. A client registers ITEMS: a place (centre + radius), the camera
// distances at which the item is wanted, and two callbacks -- load() makes it resident
// (reads it from the bundle, uploads it, creates its entities) and returns the bytes it
// took; unload() gives all of that back. Every frame the service drops what is out of
// range and loads the nearest wanted items within a time budget. The item nearest the
// camera always loads, so what is right in front of the player never waits on the budget.
//
// Items are expected to have a coarser stand-in that is always resident (a building
// cell's mass-box proxy, a terrain tile's parent), so an item arriving a frame late is a
// brief loss of detail, never a hole. First client: building part cells (level_loader).

#include "../rt_math.h"

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace engine {

class JobSystem;

class Residency {
public:
    struct Item {
        std::string client;             // for the per-client stats ("buildings", ...)
        Vec3 center{0, 0, 0};
        double radius = 0.0;            // the item's extent: distances are to its nearest point
        double loadWithin = 0.0;        // wanted once the camera is this close...
        double dropBeyond = 0.0;        // ...and let go past this (> loadWithin: hysteresis)
        double dropWithin = -1.0;       // an OUTER tier's inner edge: let go closer than this
        std::function<std::size_t()> load;
        std::function<void()> unload;
        // TWO-PHASE loading (preferred): prepare() runs on a worker thread -- read, decode,
        // drape: the slow part, never the frame's -- and commit() on the render thread with
        // what it made (upload, entities), returning the bytes. Leave both empty to use load().
        std::function<std::shared_ptr<void>()> prepare;
        std::function<std::size_t(std::shared_ptr<void>)> commit;
        bool resident = false;
        std::size_t bytes = 0;
        // two-phase state, render thread only
        enum class Phase : uint8_t { Idle, Preparing, Ready } phase = Phase::Idle;
        std::shared_ptr<void> prepared;
    };

    std::size_t add(Item item);
    std::size_t size() const { return items_.size(); }

    // Drop out-of-range residents, then load the nearest wanted items until `budgetMs` of
    // this call is spent (budgetMs <= 0: no limit -- a level's first frame). With `jobs`, a
    // two-phase item's prepare() runs there (a few at once) and its commit() lands in a later
    // update; without, or on a jump, both run here.
    void update(const Vec3& camera, double budgetMs, JobSystem* jobs = nullptr);
    void unloadAll();

    struct Stats {
        std::size_t items = 0, resident = 0, bytes = 0;
        long loads = 0, unloads = 0;
        std::size_t waiting = 0;        // wanted but not resident after the last update
        double loadMs = 0.0, maxLoadMs = 0.0;   // on the render thread: a synchronous load (a jump), total and slowest
        long commits = 0;                        // two-phase items that arrived prepared...
        double commitMs = 0.0, maxCommitMs = 0.0;   // ...and what committing them cost the render thread
    };
    const std::map<std::string, Stats>& stats() const { return stats_; }

    // A camera move longer than this in one update is a jump: everything in range loads at once.
    static constexpr double kJumpDistance = 200.0;
    // ...and of what is in range then, only items this near load in place; the rest stream as in movement.
    static constexpr double kJumpNear = 150.0;
    // At most this many prepare() jobs at once: enough to keep ahead of a car, few enough that
    // the thread pool still serves physics and terrain.
    static constexpr int kMaxPreparing = 3;

private:
    Vec3 last_{0, 0, 0};
    bool haveLast_ = false;
    int preparing_ = 0;
    // Finished prepare() results, handed from the workers to the render thread. Shared with the
    // jobs, so one still running when the level unloads writes into a live inbox, not freed memory.
    struct Inbox {
        std::mutex m;
        std::vector<std::pair<std::size_t, std::shared_ptr<void>>> done;
    };
    std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
    std::size_t loadNow(Item& it);   // synchronous: load(), or prepare() then commit()
    std::vector<Item> items_;
    std::map<std::string, Stats> stats_;
};

// One per level: the loader creates it (with the items it registers) and ResidencySystem
// drives it from the camera.
struct ResidencyService {
    std::shared_ptr<Residency> service;
};

}  // namespace engine

#endif
