#include "traffic_signal.h"

#include <cmath>

namespace citysim {

void SignalController::build(const engine::NavGraph& graph, double greenTime,
                             double yellowTime) {
    green_ = greenTime;
    yellow_ = yellowTime;
    clock_ = 0.0;
    phase_.assign(graph.linkCount(), -1);

    for (int li = 0; li < graph.linkCount(); ++li) {
        int to = graph.links[li].to;
        if (!graph.isJunction(to)) continue;            // open road: no signal
        // Bin the approach direction to one of two perpendicular axes: angles a
        // and a+PI land in the same bin (opposing arms share a phase), a+PI/2 in
        // the other. quantise atan2 to quarter-turns, then mod 2.
        engine::Vec2 d = graph.direction(li);
        double ang = std::atan2(d.y, d.x);
        long q = std::lround(ang / (engine::PI * 0.5));
        phase_[li] = static_cast<int>(((q % 2) + 2) % 2);
    }
}

void SignalController::update(double dt) { clock_ += dt; }

bool SignalController::hasSignal(int link) const {
    return link >= 0 && link < static_cast<int>(phase_.size()) && phase_[link] >= 0;
}

SignalState SignalController::stateForLink(int link) const {
    if (!hasSignal(link)) return SignalState::Green;   // open road
    int bin = phase_[link];

    // The cycle: phase 0 green, phase 0 yellow, phase 1 green, phase 1 yellow.
    double half = green_ + yellow_;
    double period = 2.0 * half;
    double t = std::fmod(clock_, period);
    if (t < 0) t += period;

    // Window for THIS bin's green/yellow within the period.
    double base = bin * half;          // bin 0 starts at 0, bin 1 at `half`
    double local = t - base;
    if (local < 0) local += period;
    if (local < green_) return SignalState::Green;
    if (local < half) return SignalState::Yellow;
    return SignalState::Red;
}

}  // namespace citysim
