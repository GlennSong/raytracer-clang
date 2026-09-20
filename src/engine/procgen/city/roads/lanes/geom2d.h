#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_GEOM2D_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_GEOM2D_H

// lanelab's 2-D geometry seam (ADR-0083). The ONLY lanelab translation unit that
// includes Clipper2 or CDT is geom2d.cpp; everything else speaks these types. If the
// dependency decision goes the other way, this file is the whole blast radius.
//
// Conventions: plan coordinates are engine (x, z) carried in Vec2 as (x, y); outer
// rings CCW, holes CW; areas in m²; booleans are exact on a 1 mm integer grid.

#include "engine/procgen/city/polygon.h"   // Vec2
#include <array>
#include <utility>
#include <vector>

namespace engine {
namespace roads::lanes {

using Ring = std::vector<Vec2>;             // closed, first point NOT repeated

struct Polygon2 {
    Ring outer;                             // CCW
    std::vector<Ring> holes;                // CW
};
using PolySet = std::vector<Polygon2>;

// --- booleans (robust, integer-snapped) -----------------------------------------
PolySet unionRings(const std::vector<Ring>& rings);        // non-zero union of simple rings
PolySet unionSets(const PolySet& a, const PolySet& b);
PolySet differenceSets(const PolySet& a, const PolySet& b);
PolySet intersectSets(const PolySet& a, const PolySet& b);

// --- offsets -----------------------------------------------------------------------
// Positive delta grows, negative shrinks; round joins. closing(r) = grow r, shrink r:
// convex corners return exactly, concave notches narrower than 2r fill with a fillet.
PolySet offsetSet(const PolySet& a, double delta);
PolySet closing(const PolySet& a, double r);

// --- queries -------------------------------------------------------------------------
double ringArea(const Ring& r);             // signed (+ CCW)
double setArea(const PolySet& s);
bool contains(const Polygon2& p, const Vec2& q);   // holes excluded; boundary counts as inside
bool contains(const PolySet& s, const Vec2& q);
struct Box2 { double minX = 0, minY = 0, maxX = 0, maxY = 0; };
Box2 bounds(const Polygon2& p);
Box2 bounds(const PolySet& ps);              // over EVERY polygon of the set (a footprint may carry a detached sliver first)
PolySet fromRing(const Ring& r);            // a single-ring polygon set (ring may be CW; normalised)

// --- constrained Delaunay ----------------------------------------------------------------
// One triangulation of every point, honouring every constraint edge; crossing
// constraints are split at their intersections (that split IS the planar arrangement
// lanelab wants). Returns the vertex list (input points first, in order, then any
// intersection points) and CCW triangles over the convex hull; callers classify and
// discard triangles outside the pavement.
struct Triangulation {
    std::vector<Vec2> verts;
    std::vector<std::array<int, 3>> tris;
};
Triangulation constrainedTriangulation(const std::vector<Vec2>& points,
                                       const std::vector<std::pair<int, int>>& edges);

}  // namespace roads::lanes
}  // namespace engine

#endif
