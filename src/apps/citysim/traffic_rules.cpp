#include "traffic_rules.h"

#include <algorithm>
#include <cmath>

namespace citysim {

double approachStop(double dist, double decel, double maxSpeed) {
    if (dist <= 0.0) return 0.0;
    double v = std::sqrt(2.0 * decel * dist);
    return std::min(v, maxSpeed);
}

double signalSpeedCap(SignalState state, double dist, double maxSpeed, double decel) {
    switch (state) {
        case SignalState::Green:  return maxSpeed;
        case SignalState::Yellow: return approachStop(dist, decel, maxSpeed);
        case SignalState::Red:    return approachStop(dist, decel, maxSpeed);
    }
    return maxSpeed;
}

double nearestObstacleAhead(const engine::VisionCone& cone,
                            const std::vector<engine::Vec2>& obstacles,
                            double noObstacle) {
    double best = noObstacle;
    for (const engine::Vec2& o : obstacles) {
        if (!engine::sees(cone, o)) continue;
        double fd = engine::forwardDistance(cone, o);
        if (fd > 0.0 && fd < best) best = fd;
    }
    return best;
}

}  // namespace citysim
