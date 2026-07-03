#ifndef RAYTRACER_ENGINE_AI_AGENT_MEMORY_H
#define RAYTRACER_ENGINE_AI_AGENT_MEMORY_H

#include "perception.h"   // Vec2/Real (via polygon.h)

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace engine {

// WORKING MEMORY for an agent (ADR-0063: sense -> REMEMBER -> PREDICT -> decide
// -> act). A vision cone produces momentary snapshots; acting on snapshots makes
// a goldfish — no continuity, no anticipation, and flicker at the cone's edge.
// Memory turns sightings into TRACKS: each body the agent has seen keeps a
// smoothed velocity estimate (successive sightings differenced — a poor man's
// Kalman filter, which is all a game needs), persists for a few seconds after it
// leaves view with its position EXTRAPOLATED along that velocity (object
// permanence: the car that passed behind the truck still exists), and fades out
// on a confidence that decays to zero. Pure and deterministic; ~1.5 KB per agent
// at the default track cap, updated at think cadence — cheap at hundreds of
// agents.
struct TrackedBody {
    int id = -1;          // the caller's stable body id
    Vec2 pos;             // best current estimate (extrapolated while unseen)
    Vec2 vel;             // smoothed velocity estimate (m/s)
    Vec2 seenPos;         // raw position at the last actual sighting
    Real lastSeen = 0;    // sim-time of that sighting
    Real confidence = 0;  // 1 = just seen, decays to 0 = forgotten
    bool hasVelocity = false;   // one sighting can't give a velocity yet
};

class AgentMemory {
public:
    Real memorySeconds = 4.0;    // unseen tracks survive (and extrapolate) this long
    Real velocityBlend = 0.5;    // EMA weight of the newest velocity estimate
    std::size_t maxTracks = 32;  // hard cap: weakest track evicted when full

    // A sighting this think tick. Successive sightings of the same id build the
    // velocity estimate; a re-sighting of a faded track re-anchors it.
    void observe(int id, const Vec2& p, Real now) {
        TrackedBody* t = find(id);
        if (!t) {
            if (tracks_.size() >= maxTracks) evictWeakest();
            TrackedBody nb;
            nb.id = id;
            nb.pos = p;
            nb.seenPos = p;
            nb.lastSeen = now;
            nb.confidence = 1.0;
            tracks_.push_back(nb);
            return;
        }
        Real dt = now - t->lastSeen;
        if (dt > 1e-6) {
            Vec2 raw((p.x - t->seenPos.x) / dt, (p.y - t->seenPos.y) / dt);
            if (t->hasVelocity) {
                t->vel.x += (raw.x - t->vel.x) * velocityBlend;
                t->vel.y += (raw.y - t->vel.y) * velocityBlend;
            } else {
                t->vel = raw;
                t->hasVelocity = true;
            }
        }
        t->seenPos = p;
        t->pos = p;
        t->lastSeen = now;
        t->confidence = 1.0;
    }

    // Advance the memory: unseen tracks extrapolate along their velocity (the
    // agent EXPECTS them where they were heading) and fade; stale tracks drop.
    void update(Real now) {
        for (TrackedBody& t : tracks_) {
            Real age = std::max(Real(0), now - t.lastSeen);
            t.confidence = 1.0 - age / memorySeconds;
            t.pos = t.seenPos + t.vel * age;
        }
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                     [](const TrackedBody& t) { return t.confidence <= 0; }),
                      tracks_.end());
    }

    const std::vector<TrackedBody>& tracks() const { return tracks_; }
    const TrackedBody* track(int id) const {
        for (const TrackedBody& t : tracks_)
            if (t.id == id) return &t;
        return nullptr;
    }
    void clear() { tracks_.clear(); }

private:
    TrackedBody* find(int id) {
        for (TrackedBody& t : tracks_)
            if (t.id == id) return &t;
        return nullptr;
    }
    void evictWeakest() {
        if (tracks_.empty()) return;
        std::size_t weakest = 0;
        for (std::size_t i = 1; i < tracks_.size(); ++i)
            if (tracks_[i].confidence < tracks_[weakest].confidence) weakest = i;
        tracks_.erase(tracks_.begin() + weakest);
    }

    std::vector<TrackedBody> tracks_;
};

// --- PREDICT: anticipation from tracked velocities (dot products, not ML) -----

// Closest point of approach between two moving points: when, and how near.
// `time` is clamped to >= 0 (diverging bodies are closest right now).
struct Approach {
    Real time = 0;       // seconds until the minimum separation
    Real distance = 0;   // that minimum separation (m)
};

inline Approach closestApproach(const Vec2& pa, const Vec2& va,
                                const Vec2& pb, const Vec2& vb) {
    Vec2 dp = pb - pa, dv = vb - va;
    Real dvv = dv.x * dv.x + dv.y * dv.y;
    Real t = dvv < 1e-9 ? 0.0 : -(dp.x * dv.x + dp.y * dv.y) / dvv;
    if (t < 0) t = 0;
    Vec2 at(dp.x + dv.x * t, dp.y + dv.y * t);
    Approach a;
    a.time = t;
    a.distance = at.length();
    return a;
}

// Time until two moving discs of combined radius `radius` touch: 0 if already
// overlapping, +infinity if they never will (on current velocities).
inline Real timeToCollision(const Vec2& pa, const Vec2& va,
                            const Vec2& pb, const Vec2& vb, Real radius) {
    const Real kNever = std::numeric_limits<Real>::infinity();
    Vec2 dp = pb - pa, dv = vb - va;
    Real c = dp.x * dp.x + dp.y * dp.y - radius * radius;
    if (c <= 0) return 0;                                  // touching already
    Real a = dv.x * dv.x + dv.y * dv.y;
    if (a < 1e-9) return kNever;                           // no relative motion
    Real b = 2.0 * (dp.x * dv.x + dp.y * dv.y);
    Real disc = b * b - 4.0 * a * c;
    if (disc < 0) return kNever;                           // paths miss
    Real t = (-b - std::sqrt(disc)) / (2.0 * a);
    return t >= 0 ? t : kNever;                            // past collisions don't count
}

}  // namespace engine

#endif
