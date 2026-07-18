#include "traffic_signal.h"

#include <algorithm>
#include <cmath>

namespace citysim {

void SignalController::build(const engine::NavGraph& graph, double greenTime,
                             double yellowTime, double allRedTime,
                             double walkTime) {
    green_ = greenTime;
    yellow_ = yellowTime;
    allRed_ = allRedTime;
    walk_ = walkTime;
    clock_ = 0.0;
    phase_.assign(graph.linkCount(), -1);
    nodeOf_.assign(graph.linkCount(), -1);
    nodes_.clear();

    // Inbound street approaches per junction node.
    std::unordered_map<int, std::vector<int>> inbound;
    for (int li = 0; li < graph.linkCount(); ++li) {
        const int to = graph.links[li].to;
        // A freeway merge/gore is a junction in the unified graph (§10) but
        // never a stoplight: mainline and ramp approaches stay uncontrolled.
        if (graph.links[li].klass == engine::RoadClass::Freeway ||
            graph.links[li].klass == engine::RoadClass::Ramp)
            continue;
        if (!graph.isJunction(to)) continue;            // open road: no signal
        // Only signalise real crossings — 4+ approaches. T-junctions and minor
        // merges go uncontrolled, so a dense district isn't a forest of
        // stoplights (user feedback).
        if (graph.outLinks[to].size() < 4) continue;
        inbound[to].push_back(li);
    }

    // CONFLICT-BASED grouping (roads-v2.1 R3): two approaches conflict unless
    // near-parallel (same stream / a collinear duplicate) or near-opposing
    // (the classic shared phase) — i.e. whenever their straight-through
    // streams actually CROSS the box. Greedy-color into the fewest groups,
    // approaches ordered by bearing so opposing arms pair deterministically.
    // The old bearing-parity bins put all four arms of an organic X in ONE
    // phase (metropolis: "the lights on all four sides turned green at the
    // same time") and split perpendicular pairs into the SAME green on
    // diagonal crossings — both impossible here by construction.
    constexpr double kSin20 = 0.34202014332;
    for (auto& [node, links] : inbound) {
        std::sort(links.begin(), links.end(), [&](int a, int b) {
            const engine::Vec2 da = graph.direction(a), db = graph.direction(b);
            return std::atan2(da.y, da.x) < std::atan2(db.y, db.x);
        });
        std::vector<int> groupOf(links.size(), -1);
        int groups = 0;
        for (std::size_t i = 0; i < links.size(); ++i) {
            const engine::Vec2 di = graph.direction(links[i]);
            for (int g = 0; g < groups && groupOf[i] < 0; ++g) {
                bool ok = true;
                for (std::size_t j = 0; j < i && ok; ++j) {
                    if (groupOf[j] != g) continue;
                    const engine::Vec2 dj = graph.direction(links[j]);
                    if (std::fabs(di.x * dj.y - di.y * dj.x) >= kSin20)
                        ok = false;   // streams cross: cannot share a green
                }
                if (ok) groupOf[i] = g;
            }
            if (groupOf[i] < 0) groupOf[i] = groups++;
        }
        for (std::size_t i = 0; i < links.size(); ++i) {
            phase_[links[i]] = groupOf[i];
            nodeOf_[links[i]] = node;
        }
        // Per-junction clock OFFSET (golden-ratio spread): one shared clock
        // synchronised every junction's WALK window citywide — with few
        // drivers, the whole city stood still together every cycle (the
        // gridlock gate measured 55+ s of EVERYONE stopped). Real signal
        // networks stagger; so do we, deterministically by node id.
        const double slot2 = green_ + yellow_ + allRed_;
        const double period2 = groups * slot2 + walk_;
        const double frac = std::fmod(node * 0.6180339887498949, 1.0);
        nodes_[node] = { groups, frac * period2 };
    }
}

void SignalController::update(double dt) { clock_ += dt; }

bool SignalController::hasSignal(int link) const {
    return link >= 0 && link < static_cast<int>(phase_.size()) && phase_[link] >= 0;
}

// Local time within this junction's cycle. The cycle: each group takes
// [green][yellow][all-red], then one WALK window (all red, pedestrians own
// the box) closes the cycle.
double SignalController::cycleAt(const NodePlan& plan, double& period) const {
    const double slot = green_ + yellow_ + allRed_;
    period = plan.groups * slot + walk_;
    double t = std::fmod(clock_ + plan.offset, period);
    if (t < 0) t += period;
    return t;
}

SignalState SignalController::stateForLink(int link) const {
    if (!hasSignal(link)) return SignalState::Green;   // open road
    const auto it = nodes_.find(nodeOf_[link]);
    if (it == nodes_.end()) return SignalState::Green;
    double period = 0;
    const double t = cycleAt(it->second, period);
    const double slot = green_ + yellow_ + allRed_;
    const double local = t - phase_[link] * slot;
    if (local >= 0 && local < green_) return SignalState::Green;
    if (local >= 0 && local < green_ + yellow_) return SignalState::Yellow;
    return SignalState::Red;   // own clearance, other groups' turns, or WALK
}

bool SignalController::walkWindowAt(int node) const {
    return walkRemainingAt(node) > 0.0;
}

double SignalController::walkRemainingAt(int node) const {
    const auto it = nodes_.find(node);
    if (it == nodes_.end()) return 0.0;
    double period = 0;
    const double t = cycleAt(it->second, period);
    return t >= period - walk_ ? period - t : 0.0;
}

}  // namespace citysim
