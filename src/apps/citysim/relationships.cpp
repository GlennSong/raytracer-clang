#include "relationships.h"

namespace citysim {

const char* relationshipName(Relationship r) {
    switch (r) {
        case Relationship::Household:    return "household";
        case Relationship::Coworker:     return "coworker";
        case Relationship::Neighbor:     return "neighbor";
        case Relationship::Acquaintance: return "acquaintance";
        default:                         return "stranger";
    }
}

void RelationshipTable::set(AgentId a, AgentId b, Relationship r) {
    if (a == b || a == kNoAgent || b == kNoAgent || r == Relationship::None) return;
    const uint64_t k = key(a, b);
    auto it = bonds_.find(k);
    if (it != bonds_.end()) {
        if (static_cast<uint8_t>(r) <= static_cast<uint8_t>(it->second)) return;  // no downgrade
        it->second = r;
        // Refresh the tag in both adjacency lists.
        for (AgentId who : {a, b}) {
            AgentId other = who == a ? b : a;
            for (auto& e : adj_[who])
                if (e.first == other) { e.second = r; break; }
        }
        return;
    }
    bonds_[k] = r;
    adj_[a].push_back({b, r});
    adj_[b].push_back({a, r});
}

Relationship RelationshipTable::get(AgentId a, AgentId b) const {
    if (a == b) return Relationship::None;
    auto it = bonds_.find(key(a, b));
    return it == bonds_.end() ? Relationship::None : it->second;
}

int RelationshipTable::countFor(AgentId a) const {
    auto it = adj_.find(a);
    return it == adj_.end() ? 0 : static_cast<int>(it->second.size());
}

const std::vector<std::pair<AgentId, Relationship>>&
RelationshipTable::relationsOf(AgentId a) const {
    static const std::vector<std::pair<AgentId, Relationship>> kEmpty;
    auto it = adj_.find(a);
    return it == adj_.end() ? kEmpty : it->second;
}

void RelationshipTable::setTie(AgentId a, AgentId b, int aToB, int bToA) {
    if (a == b || a == kNoAgent || b == kNoAgent || aToB < 0 || bToA < 0) return;
    set(a, b, Relationship::Household);
    ties_[(static_cast<uint64_t>(a) << 32) | b] = static_cast<uint16_t>(aToB);
    ties_[(static_cast<uint64_t>(b) << 32) | a] = static_cast<uint16_t>(bToA);
}

int RelationshipTable::tieOf(AgentId a, AgentId b) const {
    auto it = ties_.find((static_cast<uint64_t>(a) << 32) | b);
    return it == ties_.end() ? -1 : it->second;
}

}  // namespace citysim
