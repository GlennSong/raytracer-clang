#ifndef RAYTRACER_APPS_CITYSIM_AGENT_ID_H
#define RAYTRACER_APPS_CITYSIM_AGENT_ID_H

#include <cstdint>

namespace citysim {

// Agent identity (Living City, ADR-0066 Phase 1). Every agent — pedestrian,
// driver, or the player — carries a stable UID so later layers (relationship
// tables, per-agent memory of people/places, job assignment) can key on it
// without caring about the agent's slot index, which is only a storage detail.
//
// The UID is NOT the agent's array index. Indices are reused when agents are
// rebuilt or (later) recycled; a UID, once handed out, is never reused for a
// different agent in the same run. That distinction is the whole point: a
// relationship "A knows B" must survive A moving to a different slot.
//
// Allocation is a monotonic counter, so it is fully deterministic in seed +
// call order (ADR-0002): the same build sequence hands out the same UIDs. Kept
// header-only and dependency-free so it drops into the Lua-free test build.
using AgentId = uint32_t;

// The "no agent" sentinel — an unset uid, or a relationship endpoint that has
// gone away. All-bits-one so it never collides with a real dense id.
constexpr AgentId kNoAgent = ~AgentId(0);

// Hands out unique AgentIds in a deterministic order. One allocator per sim;
// reset() on rebuild so a fresh scene starts from 0 (reproducible), while a
// LIVE scene that spawns more agents keeps counting up — never recycling a UID
// a still-remembered agent might once have held.
class AgentIdAllocator {
public:
    // The next fresh id. Deterministic: depends only on how many times this has
    // been called, not on wall clock or address.
    AgentId next() { return next_++; }

    // How many ids have been handed out (also the value the next call returns).
    AgentId issued() const { return next_; }

    // Start over from 0 — call at build, before assigning the initial cohort, so
    // a reseeded sim reproduces the same UIDs bit for bit.
    void reset() { next_ = 0; }

private:
    AgentId next_ = 0;
};

}  // namespace citysim

#endif
