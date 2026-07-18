#ifndef RAYTRACER_APPS_CITYSIM_TRAFFIC_SIGNAL_H
#define RAYTRACER_APPS_CITYSIM_TRAFFIC_SIGNAL_H

#include "../../engine/ai/nav_graph.h"
#include <unordered_map>
#include <vector>

namespace citysim {

enum class SignalState { Green, Yellow, Red };

// Stoplights over a NavGraph's junctions (ADR-0060, reworked roads-v2.1 R3).
// Phases come from the junction's CONFLICT GRAPH, not bearing parity: two
// approaches conflict unless they are near-parallel or near-opposing (their
// straight-through streams cross the box). Approaches greedy-color into the
// fewest non-conflicting groups — an orthodox 4-way gets the classic two, a
// skewed or five-arm junction gets three-plus, and the all-one-band organic
// X that put FOUR simultaneous greens on metropolis gets its crossing pairs
// separated (the drive-feedback regression this rework exists for). Each
// cycle walks the groups (green -> yellow -> all-red each) and ends with a
// WALK window: everything red, pedestrians own the box. Deterministic: state
// is a pure function of accumulated time.
class SignalController {
public:
    // Build per-junction phase groups. greenTime/yellowTime/allRedTime are
    // per-group durations; walkTime closes each full cycle.
    // Defaults keep a 2-group cycle near the old 34 s two-phase period —
    // longer cycles queue more cars per red than small city blocks hold,
    // and a saturated block RING is a physical deadlock no rule can undo.
    void build(const engine::NavGraph& graph, double greenTime = 10.0,
               double yellowTime = 2.0, double allRedTime = 1.5,
               double walkTime = 8.0);

    // Advance the shared signal clock.
    void update(double dt);

    // Signal facing traffic entering the junction along `link`.
    SignalState stateForLink(int link) const;

    // Does this link face a real signal (enters a junction)? Else it's open road.
    bool hasSignal(int link) const;

    // Is `node`'s signalled junction in its WALK window (all approaches red,
    // pedestrians cross)? False for unsignalled nodes — the kerb logic's
    // gap-acceptance owns those.
    bool walkWindowAt(int node) const;

    // Seconds of WALK remaining at `node` (0 outside the window) — a
    // pedestrian STARTS a crossing only with enough left to clear the box.
    double walkRemainingAt(int node) const;

private:
    struct NodePlan { int groups = 0; double offset = 0; };
    std::vector<int> phase_;    // per link: -1 = no signal, else group index
    std::vector<int> nodeOf_;   // per link: the junction node it enters
    std::unordered_map<int, NodePlan> nodes_;   // per signalled junction
    double green_ = 12.0, yellow_ = 2.5, allRed_ = 2.5, walk_ = 7.0;
    double clock_ = 0.0;
    double cycleAt(const NodePlan& plan, double& period) const;   // local time
};

}  // namespace citysim

#endif
