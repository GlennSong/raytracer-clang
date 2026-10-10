#include "residency.h"
#include "../job_system.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace engine {

std::size_t Residency::add(Item item) {
    Stats& s = stats_[item.client];
    ++s.items;
    items_.push_back(std::move(item));
    return items_.size() - 1;
}

std::size_t Residency::loadNow(Item& it) {
    if (it.prepare && it.commit) {
        std::shared_ptr<void> payload = it.phase == Item::Phase::Ready ? std::move(it.prepared) : it.prepare();
        it.prepared.reset();
        it.phase = Item::Phase::Idle;
        return it.commit(std::move(payload));
    }
    return it.load ? it.load() : 0;
}

void Residency::update(const Vec3& camera, double budgetMs, JobSystem* jobs) {
    const auto t0 = std::chrono::steady_clock::now();
    // A JUMP (a teleport, a respawn, the first frame) is not movement: nothing it lands on was
    // loading ahead, so everything in range loads now -- a hitch, never a city with holes in it.
    bool jump = !haveLast_;
    if (haveLast_) {
        const double dx = camera.x - last_.x, dy = camera.y - last_.y, dz = camera.z - last_.z;
        jump = dx * dx + dy * dy + dz * dz > kJumpDistance * kJumpDistance;
    }
    if (jump) budgetMs = 0.0;
    // AHEAD OF A MOVING CAMERA (the city's next ten #10, a fly-around): where it will be in ~0.75 s at this frame's
    // step (45 frames), so a cell starts streaming before the camera is on it; an item counts as near when it is near
    // either point. And when moving fast (over ~90 m/s at 60 fps) twice the prepare jobs.
    Vec3 ahead = camera;
    double step = 0.0;
    if (haveLast_ && !jump) {
        const Vec3 d = camera - last_;
        step = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        ahead = camera + d * 45.0;
    }
    const int maxPreparing = step > 1.5 ? 2 * kMaxPreparing : kMaxPreparing;
    last_ = camera;
    haveLast_ = true;
    // Hand over what the workers finished since the last update.
    {
        std::lock_guard<std::mutex> lock(inbox_->m);
        for (auto& [i, payload] : inbox_->done) {
            Item& it = items_[i];
            it.phase = Item::Phase::Ready;
            it.prepared = std::move(payload);
            --preparing_;
        }
        inbox_->done.clear();
    }
    auto distanceTo = [&](const Item& it) {
        auto from = [&](const Vec3& c) {
            const double dx = it.center.x - c.x, dy = it.center.y - c.y, dz = it.center.z - c.z;
            return std::max(0.0, std::sqrt(dx * dx + dy * dy + dz * dz) - it.radius);
        };
        return std::min(from(camera), from(ahead));
    };
    // 1. Let go of what is out of range (retired by the renderer, never a stall), and of
    //    prepared results nobody wants any more.
    std::vector<std::pair<double, std::size_t>> wanted;
    for (std::size_t i = 0; i < items_.size(); ++i) {
        Item& it = items_[i];
        const double d = distanceTo(it);
        const bool tooFar = d > it.dropBeyond;
        const bool tooNear = it.dropWithin >= 0.0 && d < it.dropWithin;
        if (it.resident && (tooFar || tooNear)) {
            if (it.unload) it.unload();
            it.resident = false;
            Stats& s = stats_[it.client];
            --s.resident;
            s.bytes -= std::min(s.bytes, it.bytes);
            ++s.unloads;
            it.bytes = 0;
            continue;
        }
        const bool inRange = d <= it.loadWithin && !(it.dropWithin >= 0.0 && d < it.dropWithin);
        if (!it.resident && !inRange && it.phase == Item::Phase::Ready) {
            it.prepared.reset();
            it.phase = Item::Phase::Idle;
        }
        if (!it.resident && inRange) wanted.push_back({d, i});
    }
    std::sort(wanted.begin(), wanted.end());
    // 2. Make resident, nearest first: a prepared item commits (cheap), a jump or a
    //    one-phase item loads here and now; the nearest always, the rest within the budget.
    std::size_t done = 0;
    for (const auto& [d, i] : wanted) {
        Item& it = items_[i];
        const bool twoPhase = it.prepare && it.commit;
        // a jump loads in place only what it lands in (kJumpNear): the rest streams through the workers like any
        // movement -- every cell keeps a stand-in, so the far ring arriving over a second is no hole, and a teleport
        // across the island stopped freezing the viewer for seconds (3.8 s measured)
        const bool canNow = !twoPhase || it.phase == Item::Phase::Ready || (jump && d <= kJumpNear) || !jobs;
        if (!canNow) continue;
        if (it.phase == Item::Phase::Preparing) continue;   // a jump does not wait on a worker
        if (budgetMs > 0 && done > 0 &&
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() > budgetMs)
            break;
        const bool wasPrepared = twoPhase && it.phase == Item::Phase::Ready;
        const auto tl = std::chrono::steady_clock::now();
        it.bytes = loadNow(it);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tl).count();
        it.resident = true;
        Stats& s = stats_[it.client];
        if (wasPrepared) { ++s.commits; s.commitMs += ms; s.maxCommitMs = std::max(s.maxCommitMs, ms); }
        else { s.loadMs += ms; s.maxLoadMs = std::max(s.maxLoadMs, ms); }
        ++s.resident;
        s.bytes += it.bytes;
        ++s.loads;
        ++done;
    }
    // 3. Start preparing the nearest wanted items that are not yet, a few at a time.
    if (jobs) {
        for (const auto& [d, i] : wanted) {
            if (preparing_ >= maxPreparing) break;
            Item& it = items_[i];
            if (it.resident || !it.prepare || !it.commit || it.phase != Item::Phase::Idle) continue;
            it.phase = Item::Phase::Preparing;
            ++preparing_;
            auto prep = it.prepare;
            const std::size_t idx = i;
            std::shared_ptr<Inbox> inbox = inbox_;
            jobs->run([inbox, prep, idx] {
                std::shared_ptr<void> payload = prep();
                std::lock_guard<std::mutex> lock(inbox->m);
                inbox->done.push_back({idx, std::move(payload)});
            });
        }
    }
    for (auto& kv : stats_) kv.second.waiting = 0;
    for (const auto& [d, i] : wanted) if (!items_[i].resident) ++stats_[items_[i].client].waiting;
}

void Residency::unloadAll() {
    for (Item& it : items_) {
        if (!it.resident) continue;
        if (it.unload) it.unload();
        it.resident = false;
        it.bytes = 0;
    }
    for (auto& kv : stats_) { kv.second.resident = 0; kv.second.bytes = 0; }
}

}  // namespace engine
