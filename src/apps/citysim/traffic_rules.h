#ifndef RAYTRACER_APPS_CITYSIM_TRAFFIC_RULES_H
#define RAYTRACER_APPS_CITYSIM_TRAFFIC_RULES_H

#include "../../engine/ai/perception.h"
#include "traffic_signal.h"
#include <vector>

namespace citysim {

// Pure driving-decision helpers (ADR-0059) — no state, easy to unit-test. All
// speeds m/s, distances m, decel m/s^2.

// Comfortable speed to come to rest in `dist` metres at `decel`: v = sqrt(2*decel*dist),
// capped to `maxSpeed`. Zero at/under the stop point.
double approachStop(double dist, double decel, double maxSpeed);

// Speed cap for a signal `dist` ahead. Green = free; Red = ease to a stop at the
// line; Yellow = also prepare to stop (conservative — commit-through is Phase 3).
double signalSpeedCap(SignalState state, double dist, double maxSpeed, double decel);

// Forward distance to the nearest obstacle the cone can see (cars, pedestrians),
// or `noObstacle` if the cone is clear. Only obstacles AHEAD (positive forward
// distance) count.
double nearestObstacleAhead(const engine::VisionCone& cone,
                            const std::vector<engine::Vec2>& obstacles,
                            double noObstacle = 1e9);

}  // namespace citysim

#endif
