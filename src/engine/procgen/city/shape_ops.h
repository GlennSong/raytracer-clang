#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SHAPE_OPS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SHAPE_OPS_H

#include "shape2.h"
#include <vector>

namespace engine {

// Operations on Shape2 (docs/lot-system-plan.md §3.2, layer L0). The verbs the
// plan grammar is written in: a wing is a UNION, a light well is a SUBTRACT, a
// setback is an OFFSET, a rounded corner is a FILLET.
//
// Two implementations sit behind this interface, exactly as the plan decided:
//
//   EXACT loop algebra is the default. It is fast, preserves sharp corners, and
//   keeps true arcs as true arcs. It is fiddly at degeneracies (collinear
//   shared edges — the limitation `polygonUnion` already documents).
//
//   A SAMPLED FIELD is the robust complement. Rasterize, do the arithmetic on
//   distances, contour with marching squares. Offsets, fillets and TOPOLOGY
//   CHANGES are free; sharpness and exactness are lost.
//
// The exact path runs first and the field is the fallback, chosen automatically
// when the exact path cannot answer (or explicitly, when a caller wants an
// organic result). Callers that must know which ran can ask for it.

// --- boolean ---------------------------------------------------------------

enum class BoolOp { Unite, Subtract, Intersect };

// The keep-rule generalization of `polygonUnion` (road_offset.cpp), which
// already does arbitrary-angle UNION for road junction welding by splitting
// every edge at its crossings and keeping the sub-edges whose midpoint lies
// outside the other operand. Subtract and intersect are THE SAME ALGORITHM
// with a different keep rule (plus, for subtract, reversing the clip's
// survivors so the hole winds the other way).
//
// ARCS SURVIVE. Crossings are found on a fine tessellation, but every surviving
// run of chords is merged back onto its source edge as a TRUE SUB-ARC — exact,
// because a sub-arc of a circle is an arc on the same circle. An untouched
// curved wall comes back bit-identical; a cut one comes back as a real arc
// whose split point carries the chord tolerance. Without this every boolean
// would permanently flatten every curve it passed over.
//
// Returns zero, one, or MANY shapes: subtracting a bar from a plate yields two
// pieces, and the caller must be able to see that. Holes are assigned to the
// smallest outer loop containing them.
//
// PRECONDITION: each operand must be a WELL-FORMED REGION SET — its shapes may
// not overlap one another (holes nesting inside their own outer loop is fine).
// The classification only ever compares a fragment against the OTHER operand,
// never against its own, so overlapping members of one operand produce a
// self-overlapping boundary and one of them is silently dropped. Fold a pile of
// possibly-overlapping shapes with uniteAll() before passing it in.
std::vector<Shape2> shapeBool(const std::vector<Shape2>& a,
                              const std::vector<Shape2>& b, BoolOp op,
                              Real chordTol = 0.02);

// Two-shape convenience wrappers.
std::vector<Shape2> unite(const Shape2& a, const Shape2& b);
std::vector<Shape2> subtract(const Shape2& a, const Shape2& b);
std::vector<Shape2> intersect(const Shape2& a, const Shape2& b);
// Fold a pile together with one op (the plan grammar's "add these four wings").
std::vector<Shape2> uniteAll(const std::vector<Shape2>& shapes);

// --- offset ----------------------------------------------------------------

// TOPOLOGY-CHANGING offset. `d > 0` grows outward, `d < 0` shrinks inward.
// Unlike `inset()` / `offsetPlan()`, which return a fixed vertex count and
// therefore cannot represent a change of shape count, this returns a LIST:
// shrinking a U splits it into two shapes, growing two neighbours merges them
// into one, and shrinking anything far enough returns nothing at all. Those are
// the cases setbacks actually hit at the block edge.
//
// The exact miter path runs first; the sampled field takes over when the exact
// result self-intersects or changes topology.
std::vector<Shape2> offsetShape(const Shape2& shape, Real d);

// Per-edge offset: edge i moves by `dist[i]`. This is what makes a setback a
// PROGRAM property (front 7 m, side 3 m, rear 8 m, party 0) instead of one
// number per district. Miter-limited, so a rounded ring cannot grow spikes.
// Returns empty if the result collapses.
Shape2 offsetEdges(const Shape2& shape, const std::vector<Real>& dist);

// Per-TAG offset: every edge takes the distance registered for its EdgeTag.
// The form the site layer actually calls — a program says "front 7, side 3,
// rear 8" and never has to know edge indices.
struct TagOffsets {
    Real street = 0, side = 0, rear = 0, court = 0, party = 0, lane = 0,
         none = 0;
    Real forTag(EdgeTag t) const;
};
Shape2 offsetByTag(const Shape2& shape, const TagOffsets& offsets);

// --- corners ---------------------------------------------------------------

// Fillet corner `i` with a TRUE tangent arc of radius r (a real circle, not a
// bezier that looks like one at a distance and does not up close). `r <= 0`
// leaves the corner sharp, which is how one plan mixes crisp party walls with a
// rounded street corner.
Loop2 filletCorners(const Loop2& loop, const std::vector<Real>& radii);
Shape2 fillet(const Shape2& shape, std::size_t vertex, Real r);
Shape2 filletAll(const Shape2& shape, Real r);
// Straight cut instead of an arc — the chamfered corner that turns a right
// angle into the diagonal a corner entrance sits in.
Loop2 chamferCorners(const Loop2& loop, const std::vector<Real>& cuts);
Shape2 chamferAll(const Shape2& shape, Real cut);

// The plan's "square -> circle": interpolate every corner's fillet radius
// toward its inscribed limit. t = 0 is the original, t = 1 is as round as the
// shape can be (a square becomes a true circle). Falls out of filletCorners.
Shape2 roundToCircle(const Shape2& shape, Real t);

// --- sampled field (the robust complement) ---------------------------------

// A 2-D signed distance field over a bounding box. Negative inside. This is the
// path the plan wants for organic results and for anything the exact algebra
// chokes on; a 2-D field is orders of magnitude cheaper than the 3-D
// polygonizeSdf already used for rocks and trees.
struct Field2 {
    Vec2 lo, hi;
    Real cell = 1.0;
    int nx = 0, ny = 0;
    std::vector<Real> d;                  // nx*ny samples, row-major

    Real at(int x, int y) const;
    Real sample(const Vec2& p) const;     // bilinear
};

// Rasterize a shape's exact signed distance. `cell` is the sample spacing —
// the accuracy floor of everything done in field space. `pad` grows the box so
// an outward offset has room to land.
Field2 rasterize(const std::vector<Shape2>& shapes, Real cell, Real pad);

// Marching-squares contour at `level`, chained into closed loops and oriented
// by NESTING (a contour walk's winding carries no information — see
// orientByNesting). Straight-edged: the field cannot return arcs.
std::vector<Poly2> contour(const Field2& f, Real level);

// Field-space boolean and offset: min/max and an iso shift. Topology changes
// are free here, which is the whole reason this path exists.
std::vector<Shape2> fieldBool(const std::vector<Shape2>& a,
                              const std::vector<Shape2>& b, BoolOp op,
                              Real cell = 0.25);
std::vector<Shape2> fieldOffset(const Shape2& shape, Real d, Real cell = 0.25);

// Smooth (blended) union — the "SDF join a circle onto the end of this wing"
// the brief asked for. `k` is the blend radius in metres; 0 is a hard union.
// Only the field can do this: a blend is not expressible in loop algebra.
std::vector<Shape2> smoothUnion(const Shape2& a, const Shape2& b, Real k,
                                Real cell = 0.25);

// --- assembly --------------------------------------------------------------

// Group a flat list of loops into shapes by NESTING depth: even depth is an
// outer boundary, odd is a hole, and each hole is assigned to the smallest
// outer loop that contains it. Shared by every op that can change shape count.
std::vector<Shape2> assembleShapes(std::vector<Loop2> loops);

}  // namespace engine

#endif
