#include "traffic_signal.h"

#include <algorithm>
#include <cmath>

namespace citysim {

Move moveOf(const engine::Vec2& a, const engine::Vec2& b) {
    const double c = a.x * b.y - a.y * b.x, d = a.x * b.x + a.y * b.y;
    if (d > 0.85) return Move::Straight;       // the sim's own "a real bend" threshold
    return c > 0 ? Move::Left : Move::Right;  // counter-clockwise crosses the oncoming stream
}

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
    leftArrow_.assign(graph.linkCount(), 0);
    nodes_.clear();

    // Inbound street approaches per junction node.
    std::unordered_map<int, std::vector<int>> inbound;
    for (int li = 0; li < graph.linkCount(); ++li) {
        const engine::NavLink& L = graph.links[li];
        // Semantic signal predicate (#17/S5): signalise a node ONLY if it is
        // a street INTERSECTION with >= 4 signalable approaches. This is the
        // ONE source of truth street_furniture + city_render also read, so
        // the heads, the phases, and the stop-bar markings can never
        // disagree. A gore/landing (Freeway/Ramp arms) is not an
        // Intersection, so it is never controlled — and a 3-street + 1-ramp
        // landing no longer sneaks past a raw outLinks >= 4 count.
        if (!(L.access & engine::road_access::kSignalable)) continue;   // ramp arm
        if (!graph.signalControlled(L.to)) continue;
        inbound[L.to].push_back(li);
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
        // PROTECTED LEFTS: an approach sharing its green with an OPPOSING one, with a lane to turn from
        std::vector<double> lead(static_cast<std::size_t>(groups), 0.0);
        for (std::size_t i = 0; i < links.size(); ++i) {
            const engine::Vec2 di = graph.direction(links[i]);
            bool opposed = false;
            for (std::size_t j = 0; j < links.size(); ++j) {
                if (j == i || groupOf[j] != groupOf[i]) continue;
                const engine::Vec2 dj = graph.direction(links[j]);
                if (di.x * dj.x + di.y * dj.y < -0.8) opposed = true;
            }
            if (opposed && graph.links[links[i]].lanes >= 2) {
                leftArrow_[links[i]] = 1;
                lead[static_cast<std::size_t>(groupOf[i])] = arrow_ + arrowYellow_;
            }
        }
        // Per-junction clock OFFSET (golden-ratio spread): one shared clock
        // synchronised every junction's WALK window citywide — with few
        // drivers, the whole city stood still together every cycle (the
        // gridlock gate measured 55+ s of EVERYONE stopped). Real signal
        // networks stagger; so do we, deterministically by node id.
        double period2 = walk_;
        for (double l : lead) period2 += l + green_ + yellow_ + allRed_;
        const double frac = std::fmod(node * 0.6180339887498949, 1.0);
        nodes_[node] = { groups, frac * period2, lead };
    }
}

void SignalController::update(double dt) { clock_ += dt; }

bool SignalController::hasSignal(int link) const {
    return link >= 0 && link < static_cast<int>(phase_.size()) && phase_[link] >= 0;
}

// Local time within this junction's cycle. The cycle: each group takes
// [green][yellow][all-red], then one WALK window (all red, pedestrians own
// the box) closes the cycle.
double SignalController::slotOf(const NodePlan& plan, int g) const {
    return (g >= 0 && g < static_cast<int>(plan.lead.size()) ? plan.lead[static_cast<std::size_t>(g)] : 0.0) + green_ + yellow_ + allRed_;
}

double SignalController::cycleAt(const NodePlan& plan, double& period) const {
    period = walk_;
    for (int g = 0; g < plan.groups; ++g) period += slotOf(plan, g);
    double t = std::fmod(clock_ + plan.offset, period);
    if (t < 0) t += period;
    return t;
}

// A group's slot: [lead arrow: left green, then its yellow; everything else red] [green] [yellow]
// [all-red]. Outside its own slot an approach is red (other groups' turns, or the WALK window).
SignalState SignalController::stateFor(int link, Move m) const {
    if (!hasSignal(link)) return SignalState::Green;   // open road
    const auto it = nodes_.find(nodeOf_[link]);
    if (it == nodes_.end()) return SignalState::Green;
    const NodePlan& plan = it->second;
    double period = 0;
    const double t = cycleAt(plan, period);
    double start = 0;
    for (int g = 0; g < phase_[link]; ++g) start += slotOf(plan, g);
    double local = t - start;
    if (local < 0) return SignalState::Red;
    const double lead = plan.lead[static_cast<std::size_t>(phase_[link])];
    if (local < lead) {
        if (m != Move::Left || !leftArrow_[link]) return SignalState::Red;   // the arrow is the lefts' alone
        return local < lead - arrowYellow_ ? SignalState::Green : SignalState::Yellow;
    }
    local -= lead;
    if (local < green_) return SignalState::Green;
    if (local < green_ + yellow_) return SignalState::Yellow;
    return SignalState::Red;   // own clearance, other groups' turns, or WALK
}

bool SignalController::protectedLeft(int link) const {
    if (!hasSignal(link) || !leftArrow_[link]) return false;
    const auto it = nodes_.find(nodeOf_[link]);
    if (it == nodes_.end()) return false;
    const NodePlan& plan = it->second;
    double period = 0;
    const double t = cycleAt(plan, period);
    double start = 0;
    for (int g = 0; g < phase_[link]; ++g) start += slotOf(plan, g);
    const double local = t - start, lead = plan.lead[static_cast<std::size_t>(phase_[link])];
    return local >= 0 && local < lead - arrowYellow_;
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
