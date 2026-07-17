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

// PER-VERTEX variable offset: vertex i is offset by its own signed distance `d[i]` (miter is
// automatic — each vertex uses its own d). `d.size()` must equal `cl.size()`. Lets a road/deck
// WIDEN along its length — an aux-lane / gore flare (road-unification one-mesher P5).
std::vector<Vec2> offsetPolyline(const std::vector<Vec2>& cl, const std::vector<double>& d,
                                 double miterLimit = 4.0);

// A closed ribbon OUTLINE for centerline `cl` at the given half-width: the left rail forward
// then the right rail back, wound CCW. The polygon a single road edge contributes to the
// network before junction welding. `miterLimit` clamps corner spikes.
Poly2 ribbonOutline(const std::vector<Vec2>& cl, double halfWidth, double miterLimit = 4.0);

// Variable-width ribbon outline: half-width `halfWidth[i]` per centerline vertex (a tapering /
// flaring deck). `halfWidth.size()` must equal `cl.size()`. CCW, like the scalar overload.
Poly2 ribbonOutline(const std::vector<Vec2>& cl, const std::vector<double>& halfWidth,
                    double miterLimit = 4.0);

// Annular ribbon for a CLOSED loop centerline `cl` (a roundabout ring): the road band between an
// OUTER rail (centerline + halfWidth) and an INNER rail (centerline − halfWidth), each offset with
// wrap-around joins (no end caps). `outer` comes back CCW (the filled disk the spokes weld onto)
// and `inner` CW (the central island to punch as a hole). `cl` may repeat its first point as last.
void ringRibbon(const std::vector<Vec2>& cl, double halfWidth, Poly2& outer, Poly2& inner,
                double miterLimit = 4.0);

// Boolean UNION of CCW polygons into boundary loops — the weld at road junctions: overlapping
// ribbon outlines merge into one outline (exterior loops CCW; interior holes, e.g. a roundabout
// island, come back CW). Method: split every edge at its crossings with the others, drop the
// sub-edges whose midpoint lies inside another polygon (interior), then chain the surviving
// boundary sub-edges into loops. Robust for ribbons that cross at an angle (the road case);
// exactly-collinear shared edges are not specially deduped. Pure + headless.
std::vector<Poly2> polygonUnion(const std::vector<Poly2>& polys);

// Merge holes into an outer loop by cutting zero-width BRIDGES, yielding ONE simple polygon a
// plain ear-clipper can triangulate (the block interiors / roundabout islands the welded road
// surrounds). `outer` is forced CCW, `holes` CW; each hole bridges to the outer boundary at a
// mutually-visible vertex (Eberly's rightmost-vertex rule). Pure + headless.
Poly2 bridgeHoles(const Poly2& outer, const std::vector<Poly2>& holes);

}  // namespace engine

#endif
