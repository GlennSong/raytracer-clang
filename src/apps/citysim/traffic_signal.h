#ifndef RAYTRACER_APPS_CITYSIM_TRAFFIC_SIGNAL_H
#define RAYTRACER_APPS_CITYSIM_TRAFFIC_SIGNAL_H

#include "../../engine/ai/nav_graph.h"
#include <vector>

namespace citysim {

enum class SignalState { Green, Yellow, Red };

// Stoplights over a NavGraph's junctions (ADR-0060). Every link that ENTERS a
// junction (its `to` is a node of degree >= 3) is assigned to one of two phases by
// its approach axis — opposing arms (N/S vs E/W) share a phase. The two phases
// alternate green -> yellow -> ALL-RED -> (other green): the clearance interval
// after each yellow lets cars already committed to the box finish their turn
// before cross traffic gets its green (device: without it, streams met inside the
// junction). Links that don't enter a junction have no signal (always Green).
// Deterministic: the state is a pure function of accumulated time.
class SignalController {
public:
    // Group every junction-entering link into a phase. greenTime/yellowTime/
    // allRedTime are the per-phase durations (seconds). Green defaults long
    // enough for a few queued cars to clear a turn at junction speed.
    void build(const engine::NavGraph& graph, double greenTime = 12.0,
               double yellowTime = 2.5, double allRedTime = 2.5);

    // Advance the shared signal clock.
    void update(double dt);

    // Signal facing traffic entering the junction along `link`.
    SignalState stateForLink(int link) const;

    // Does this link face a real signal (enters a junction)? Else it's open road.
    bool hasSignal(int link) const;

private:
    std::vector<int> phase_;   // per link: -1 = no signal, else phase 0 or 1
    double green_ = 12.0, yellow_ = 2.5, allRed_ = 2.5;
    double clock_ = 0.0;
};

}  // namespace citysim

#endif
