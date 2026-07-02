#ifndef RAYTRACER_ENGINE_AI_COMMITMENT_H
#define RAYTRACER_ENGINE_AI_COMMITMENT_H

#include "../procgen/city/polygon.h"   // Real

#include <vector>

namespace engine {

// DECISION COMMITMENT (ADR-0062: ...predict -> DECIDE -> act). A utility chooser
// with hysteresis: the agent scores a small menu of options (maneuvers) each
// think tick, but the INCUMBENT choice keeps its seat unless a challenger beats
// it by a clear margin — and never before a minimum hold time. So a decision is
// a choice the agent MAKES and keeps for a while, not a value re-derived every
// tick: near-tied scores can flip-flop forever without ever flapping the
// behaviour. One escape hatch: an EMERGENCY re-decide (the caller saw an
// imminent collision) preempts the hold instantly.
//
// Pure and deterministic; per-agent state is two scalars.
class Commitment {
public:
    Real minHold = 1.2;         // a decision stands at least this long (s)
    Real switchMargin = 0.15;   // a challenger must beat the incumbent by this

    // Score every option and return the committed choice. `scores` is the
    // caller's utility per option (same order every call); `now` is sim-time.
    int choose(const std::vector<Real>& scores, Real now, bool emergency = false) {
        if (scores.empty()) { current_ = -1; return -1; }
        int best = 0;
        for (int i = 1; i < static_cast<int>(scores.size()); ++i)
            if (scores[i] > scores[best]) best = i;

        bool invalid = current_ < 0 || current_ >= static_cast<int>(scores.size());
        if (invalid || emergency) { adopt(best, now); return current_; }
        if (now - since_ < minHold) return current_;           // committed
        if (scores[best] > scores[current_] + switchMargin) adopt(best, now);
        return current_;
    }

    int current() const { return current_; }
    Real heldFor(Real now) const { return current_ < 0 ? 0.0 : now - since_; }
    void reset() { current_ = -1; since_ = 0; }

private:
    void adopt(int option, Real now) {
        if (option != current_) { current_ = option; since_ = now; }
        // Re-adopting the same option (e.g. an emergency confirming the current
        // plan) does NOT reset the hold clock — the commitment is to the choice.
    }

    int current_ = -1;
    Real since_ = 0;
};

}  // namespace engine

#endif
