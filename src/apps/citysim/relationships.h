#ifndef RAYTRACER_APPS_CITYSIM_RELATIONSHIPS_H
#define RAYTRACER_APPS_CITYSIM_RELATIONSHIPS_H

#include "agent_id.h"
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace citysim {

// Surface-level social graph (Living City, ADR-0066). Agents know each other by
// a single TAG per pair — coworker / neighbor / acquaintance — keyed on their
// UIDs (not array slots), so a bond survives an agent moving to a different slot.
// Deliberately shallow (⟨A6⟩): a tag, not a scalar affinity model; the hooks are
// here to go deeper later without re-plumbing identity. Pure data + queries, no
// ECS/sim state, so it is unit-tested headless.
//
// A stronger tag wins: promoting an acquaintance to a coworker upgrades the bond;
// set() never downgrades. Symmetric — set(a,b) == set(b,a).
enum class Relationship : uint8_t {
    None = 0,        // strangers (never stored)
    Acquaintance,    // seen around / met in passing
    Neighbor,        // shares a home
    Coworker,        // shares a workplace
};

const char* relationshipName(Relationship r);

class RelationshipTable {
public:
    // Record (or strengthen) the bond between two agents. Ignores self-pairs and
    // kNoAgent; a weaker tag never overwrites a stronger existing one.
    void set(AgentId a, AgentId b, Relationship r);

    // The bond between two agents, or None if they don't know each other.
    Relationship get(AgentId a, AgentId b) const;

    // How many OTHER agents this one knows.
    int countFor(AgentId a) const;

    // Everyone this agent knows, with the tag — for the inspector. Insertion
    // order per agent; empty if it knows no one.
    const std::vector<std::pair<AgentId, Relationship>>& relationsOf(AgentId a) const;

    int pairCount() const { return static_cast<int>(bonds_.size()); }
    void clear() { bonds_.clear(); adj_.clear(); }

private:
    // Order-independent key for a UID pair (min in the high word).
    static uint64_t key(AgentId a, AgentId b) {
        AgentId lo = a < b ? a : b, hi = a < b ? b : a;
        return (static_cast<uint64_t>(lo) << 32) | hi;
    }
    std::unordered_map<uint64_t, Relationship> bonds_;
    std::unordered_map<AgentId, std::vector<std::pair<AgentId, Relationship>>> adj_;
};

}  // namespace citysim

#endif
