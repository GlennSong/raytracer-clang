#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_TRIANGULATE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_TRIANGULATE_H

#include "polygon.h"
#include <array>
#include <vector>

namespace engine {

// Robust ear-clipping triangulation of a polygon WITH HOLES — a compact port of
// the mapbox/earcut.hpp algorithm (ISC). Holes are eliminated into one doubly-
// linked ring via bridges, then ears are clipped with a validity test over the
// whole ring — no force-clip, so it never paves across a hole and never stalls
// on a grid of blocks (the failure mode that sent the road deck back to
// overlapping per-spine strips; see road_mesh weldSolid).
//
// `outer` is the exterior ring, `holes` the interior loops to punch out. Winding
// is auto-corrected, so either orientation is accepted. Returns the filled
// triangles as vertex triples in the ground plane (CCW-up). Degenerate input
// yields no triangles.
std::vector<std::array<Vec2, 3>>
triangulateWithHoles(const Poly2& outer, const std::vector<Poly2>& holes);

}  // namespace engine

#endif
