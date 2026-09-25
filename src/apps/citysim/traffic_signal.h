#ifndef RAYTRACER_APPS_CITYSIM_TRAFFIC_SIGNAL_H
#define RAYTRACER_APPS_CITYSIM_TRAFFIC_SIGNAL_H

#include "../../engine/ai/nav_graph.h"
#include <unordered_map>
#include <vector>

namespace citysim {

enum class SignalState { Green, Yellow, Red };

// A movement through a junction, in the nav graph's frame (traffic keeps right: a LEFT turn crosses
// the oncoming stream).
enum class Move { Left, Straight, Right };
Move moveOf(const engine::Vec2& in, const engine::Vec2& out);

// Stoplights over a NavGraph's junctions (ADR-0060, reworked roads-v2.1 R3).
// Phases come from the junction's CONFLICT GRAPH, not bearing parity: two
// approaches conflict unless they are near-parallel or near-opposing (their
// straight-through streams cross the box). Approaches greedy-color into the
// (PROTECTED LEFTS, ADR-0109: where a group's green is shared by OPPOSING approaches -- whose left
// turns cross each other's through traffic -- and an approach has a lane to turn from (two or more),
// the group's slot opens with a LEAD LEFT ARROW: those approaches' left turns go, protected, while
// everything else in the junction is red; then the arrow's yellow; then the shared green, when a left
// is permissive again and yields to oncoming. Glenn, 2026-09-25: "arrows ... to define if you have a
// protected turn".)
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

    // Signal facing traffic entering the junction along `link` -- for going straight on.
    SignalState stateForLink(int link) const { return stateFor(link, Move::Straight); }
    // ...for a given movement: during a lead arrow a left is Green while straight/right are Red.
    SignalState stateFor(int link, Move m) const;
    // Is `link`'s left turn protected right now (its green ARROW: no oncoming to yield to)?
    bool protectedLeft(int link) const;
    // Does `link`'s approach have a left-turn arrow at all (the head carries the arrow lamp)?
    bool hasLeftArrow(int link) const { return link >= 0 && link < static_cast<int>(leftArrow_.size()) && leftArrow_[link]; }

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
    struct NodePlan { int groups = 0; double offset = 0; std::vector<double> lead; };   // lead: per group, its arrow's seconds (0: none)
    std::vector<int> phase_;    // per link: -1 = no signal, else group index
    std::vector<char> leftArrow_;   // per link: its left turn has a lead arrow
    std::vector<int> nodeOf_;   // per link: the junction node it enters
    std::unordered_map<int, NodePlan> nodes_;   // per signalled junction
    double green_ = 12.0, yellow_ = 2.5, allRed_ = 2.5, walk_ = 7.0;
    double arrow_ = 7.0, arrowYellow_ = 2.5;   // a lead arrow's green and yellow
    double clock_ = 0.0;
    double cycleAt(const NodePlan& plan, double& period) const;   // local time
    double slotOf(const NodePlan& plan, int g) const;
};

}  // namespace citysim

#endif
