#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_OFFSET_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_OFFSET_H

#include "polygon.h"   // Vec2, Poly2
#include <vector>

namespace engine {

// The unified road JOIN engine (docs/unified-road-plan.md), foundational layer: 2-D polygon
// offsetting. A road body is a centerline polyline swept by a half-width; this turns the
// centerline into clean side rails and a closed ribbon OUTLINE that the boolean-union step
// (next) welds at junctions. Replaces the per-segment trapezoid stroking + the analytic
// trim + the SDF — one exact, light, parametric source of road geometry. Pure + headless.

// Offset an OPEN polyline `cl` by signed distance `d` (the left normal, perp(tangent), is the
// +d direction). MITER joins at vertices, with the miter clamped to `miterLimit * |d|` so a
// sharp corner bevels instead of shooting a spike. Endpoints offset by their end-segment
// normal. Returns `cl.size()` points (one per input vertex). Empty for < 2 points.
std::vector<Vec2> offsetPolyline(const std::vector<Vec2>& cl, double d, double miterLimit = 4.0);

// A closed ribbon OUTLINE for centerline `cl` at the given half-width: the left rail forward
// then the right rail back, wound CCW. The polygon a single road edge contributes to the
// network before junction welding. `miterLimit` clamps corner spikes.
Poly2 ribbonOutline(const std::vector<Vec2>& cl, double halfWidth, double miterLimit = 4.0);

}  // namespace engine

#endif
