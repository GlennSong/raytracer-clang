#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SHAPE2_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SHAPE2_H

#include "polygon.h"   // Vec2, Poly2, signedArea, pointInPolygon
#include <cstdint>
#include <vector>

namespace engine {

// The Lot System's 2-D kernel (docs/lot-system-plan.md §3, layer L0). Everything
// the building/lot pipeline draws — floorplans, lot zones, wings, courtyards,
// setbacks — is a region in the ground plane, and this is the type that region
// is made of. `Poly2` (a bare ring of points) cannot express three things the
// plan requires, and each of them is load-bearing rather than cosmetic:
//
//   1. HOLES. A courtyard block, an atrium, a plaza with a planting bed, a
//      donut tower. A ring cannot say "and not this part". The hole machinery
//      downstream already exists and is unwired: `triangulateWithHoles`
//      (triangulate.h) is called with an empty hole list at its only site, and
//      `polygonUnion` (road_offset.h) already returns interior loops CW.
//   2. ARCS AS EDGES. "Round this corner", "bow this shopfront", "make this end
//      an apse". A curve approximated by chords at authoring time can never be
//      re-derived, so every later op (offset, boolean, facade splitting)
//      degrades it further and a rounded building ends up faceted. A bulge
//      keeps a curved wall as ONE TRUE CIRCULAR ARC through the whole grammar;
//      it tessellates exactly once, at the end, at a tolerance the consumer
//      picks. It is also the difference between a real circle and a bezier
//      standing in for one — visibly not the same shape at building scale.
//   3. EDGE IDENTITY. Today "which wall faces the street" is recomputed by dot
//      product wherever anything needs it, which means every consumer can
//      disagree. Tagging an edge once, when the plan is authored, is what lets
//      the facade grammar spend ornament on the street, keep the rear plain,
//      and leave a party wall blank BY CONSTRUCTION rather than by heuristic.
//
// Convention follows polygon.h exactly: Vec2 is world XZ, CCW is positive area.
// An outer loop is CCW, a hole is CW — the same convention `polygonUnion` and
// `triangulateWithHoles` already speak, so shapes interoperate with the road
// pipeline without translation.
//
// Pure and headless: no meshes, no assets, no engine state.

// What a wall FACES. Set once by whoever authors the plan (the parceller tags
// the lot, the plan grammar propagates onto the walls it cuts), read by the
// facade grammar. `None` means "not yet classified" — a consumer may fall back
// to geometry, but should prefer to ask why the tag is missing.
enum class EdgeTag : uint8_t {
    None = 0,
    Street,    // faces the primary frontage: entrances, ornament, shopfronts
    Side,      // faces a neighbour across a setback: windows, no entrance
    Rear,      // faces the back of the lot: service doors, plain
    Court,     // faces an interior court/atrium: windows, quiet
    Party,     // shared with the neighbouring building: blank by construction
    Lane,      // faces a service lane / mews: real access, but a BACK face —
               // bins and service doors, never the show frontage
};

// One vertex and the edge LEAVING it. The plan sketched `Edge2 {Vec2 a, b;}`,
// but storing both endpoints in a closed loop duplicates every vertex and lets
// the two copies drift; the loop is closed by definition, so the edge's end is
// simply the next entry's `a`. One vertex, one outgoing edge, no way to be
// inconsistent.
struct Edge2 {
    Vec2 a;                       // this vertex; the edge runs a -> next vertex
    Real bulge = 0;               // 0 = straight; see below
    EdgeTag tag = EdgeTag::None;
};

// BULGE, in the DXF convention: bulge = tan(sweep / 4), where `sweep` is the
// arc's signed included angle. 0 is a straight line, 1 is a semicircle (the
// stadium end / apse), positive sweeps counter-clockwise. Chosen over
// "centre + radius + flags" because it survives transformation and needs no
// consistency check: any bulge describes a valid arc on any chord.
struct Loop2 {
    std::vector<Edge2> edges;     // CLOSED: the last edge runs back to edges[0]

    std::size_t size() const { return edges.size(); }
    bool empty() const { return edges.empty(); }
    const Vec2& start(std::size_t i) const { return edges[i].a; }
    const Vec2& end(std::size_t i) const { return edges[(i + 1) % edges.size()].a; }
    Real bulge(std::size_t i) const { return edges[i].bulge; }
};

// A REGION: one outer boundary, any number of holes. Not a ring — the
// distinction the whole kernel exists to make.
struct Shape2 {
    Loop2 outer;
    std::vector<Loop2> holes;

    bool empty() const { return outer.size() < 3; }
};

// --- construction ----------------------------------------------------------

// A straight-edged loop from a point ring (the bridge from every Poly2 the
// existing pipeline already produces). All edges get `tag`.
Loop2 loopFromPoly(const Poly2& poly, EdgeTag tag = EdgeTag::None);
Shape2 shapeFromPoly(const Poly2& poly, EdgeTag tag = EdgeTag::None);
// A shape from an outer ring plus hole rings; winding is corrected.
Shape2 shapeFromPolys(const Poly2& outer, const std::vector<Poly2>& holes,
                      EdgeTag tag = EdgeTag::None);

// Axis-aligned rectangle, CCW, corners at (x0,y0)-(x1,y1).
Shape2 rectShape(Real x0, Real y0, Real x1, Real y1, EdgeTag tag = EdgeTag::None);
// A true circle as FOUR quarter-arc edges (bulge = tan(45°) = 1 would be a
// semicircle; a quarter turn is tan(22.5°)). Four edges, not 64 points.
Shape2 circleShape(const Vec2& centre, Real radius);

// --- arc geometry ----------------------------------------------------------

// The circle an arc edge rides. `straight` is true for a zero bulge, in which
// case no other field is meaningful.
struct ArcGeom {
    Vec2 centre;
    Real radius = 0;
    Real startAngle = 0;      // atan2 of A about `centre`
    Real sweep = 0;           // signed included angle (CCW positive)
    bool straight = true;
};
ArcGeom arcGeom(const Vec2& a, const Vec2& b, Real bulge);

// The point at parameter t in [0,1] ALONG THE ARC (not along the chord).
Vec2 arcPoint(const Vec2& a, const Vec2& b, Real bulge, Real t);

// The bulge of the SUB-ARC from t0 to t1 of the arc a->b. Exact: a sub-arc of a
// circle is an arc on the same circle, so splitting an arc never degrades it.
// This is what lets booleans cut a curved wall and hand back curves.
Real subBulge(Real bulge, Real t0, Real t1);

// --- measurement -----------------------------------------------------------

// Signed area INCLUDING the circular segments the chords cut off — so a circle
// built from four quarter-arcs measures pi*r^2, not the inscribed square. Every
// area-conservation check in the kernel depends on this being exact.
Real signedArea(const Loop2& loop);
inline Real area(const Loop2& loop) { return std::abs(signedArea(loop)); }
// Outer minus holes.
Real area(const Shape2& shape);

Real perimeter(const Loop2& loop);           // arc length, not chord length
Vec2 centroid(const Shape2& shape);          // area centroid of the tessellation
void bounds(const Shape2& shape, Vec2& lo, Vec2& hi);   // includes arc bulges

// Point containment, holes honoured (a point inside a hole is NOT in the shape).
bool contains(const Shape2& shape, const Vec2& p);

// --- normalization ---------------------------------------------------------

// Force the plan's winding convention: outer CCW, holes CW. Idempotent.
void orient(Shape2& shape);

// Reverse a loop's traversal direction. NOT "reverse the point list": bulge and
// tag belong to the EDGE, so they shift index and the bulge negates (a CCW
// sweep walked backwards is CW). Exposed because it is the single subtlest
// operation in the kernel and everything that reorients depends on it.
void reverseLoop(Loop2& loop);

// Orient a set of loops by NESTING depth rather than by their current winding:
// a loop inside an even number of others is an outer ring (CCW), odd makes it a
// hole (CW). Contour extraction (marching squares) hands back whatever
// direction its cell walk produced, so its winding sign carries no information
// — this is what makes a sampled-field result interchangeable with an
// exact-boolean result downstream.
std::vector<Poly2> orientByNesting(std::vector<Poly2> loops);

// Merge near-duplicate vertices and drop near-collinear ones. Straight runs
// only: an arc edge is never merged away, because "nearly collinear" is
// meaningless for a curve the caller asked for on purpose.
Loop2 simplify(const Loop2& loop, Real tol);
Shape2 simplify(const Shape2& shape, Real tol);

// The INVERSE of tessellate: refit a dense polyline as the few true arcs and
// straight runs it came from. A marching-squares contour (shape_ops' sampled
// field — the only way to get a smooth blend) hands back hundreds of chords a
// third of a metre long, and downstream that is three separate problems: a
// wall shorter than a door fails the plan invariant, a sixty-storey stack
// multiplies the vertex count by sixty, and the facade grammar has no wall to
// put a window on. Fitting arcs back on solves all three at once, and it is
// the operation this kernel is shaped for — a blended lobe genuinely IS a
// circular arc, so the fit is a recovery rather than an approximation.
//
// `tol` is the maximum distance from the fitted arc to the sampled points;
// pick it at the field's cell size, since resolving finer than the samples
// only fits the sampling noise. `minEdge`, when > 0, is a floor the fit is
// then pushed to meet: the shortest edge is absorbed into whichever neighbour
// refits with less error, repeatedly, so the result has no wall too short to
// build. Corners survive — a fit that would round one off exceeds `tol` long
// before it gets there, and existing arc edges are never spanned across.
Loop2 fitArcs(const Loop2& loop, Real tol, Real minEdge = 0);
Shape2 fitArcs(const Shape2& shape, Real tol, Real minEdge = 0);

// --- tessellation ----------------------------------------------------------

// Arcs -> chords, at the END of the pipeline, at the consumer's tolerance.
// `chordTol` is the maximum distance from a chord to the true arc, in metres:
// 0.02 for a facade you stand next to, 0.5 for a distant LOD.
Poly2 tessellate(const Loop2& loop, Real chordTol = 0.05);

struct TessShape {
    Poly2 outer;
    std::vector<Poly2> holes;
};
TessShape tessellate(const Shape2& shape, Real chordTol = 0.05);

// --- editing verbs ---------------------------------------------------------

// Bow edge `i` into an arc with the given SAGITTA (the bulge height at the
// middle of the chord), in metres. On a CCW loop a positive sagitta bows
// OUTWARD and a negative one scoops in: a bay window, a bowed shopfront.
void bowEdge(Loop2& loop, std::size_t i, Real sagitta);
// Turn edge `i` into a clean semicircular end — an apse, a stadium end.
void apseEdge(Loop2& loop, std::size_t i, Real sign = 1.0);

// Retag every edge whose outward normal is within `deg` of `dir` (used to mark
// the street frontage once the lot knows which way the road is).
void tagFacing(Loop2& loop, const Vec2& dir, Real deg, EdgeTag tag);

// --- the pen (plan §3.4) ---------------------------------------------------
//
// A 2-D turtle, sibling to the L-system's 3-D turtle (lsystem.cpp), producing a
// Shape2. Both an ABSOLUTE path API (moveTo/lineTo/arcTo) and a RELATIVE turtle
// API (forward/turn/sweep), because floorplans want both: a facade is a turtle
// walk ("18 m of wall, turn 90, 12 m"), while a boolean cut is absolute. This
// is what makes plans authorable DATA (and, at P9, Lua) rather than C++
// literals.
//
//   Pen p;
//   p.moveTo(0, 0).forward(18).turn(90).forward(12).sweep(6, 180).close();
//
// `sweep` emits ONE TRUE ARC edge, not a chord fan — the whole point of having
// arcs in the type.
class Pen {
public:
    Pen& moveTo(Real x, Real y);           // start (or restart) the path
    Pen& lineTo(Real x, Real y);           // straight edge to an absolute point
    Pen& arcTo(Real x, Real y, Real bulge);// arc edge to an absolute point
    Pen& forward(Real d);                  // walk along the current heading
    Pen& turn(Real deg);                   // rotate the heading (CCW positive)
    Pen& left(Real deg) { return turn(deg); }
    Pen& right(Real deg) { return turn(-deg); }
    // Curve through `deg` of a circle of the given radius, heading updated. The
    // turn direction follows the sign of `deg`; radius must be positive.
    Pen& sweep(Real radius, Real deg);
    Pen& tag(EdgeTag t);                   // tag subsequently emitted edges

    Loop2 closeLoop() const;               // the path as a closed loop, CCW
    Shape2 close() const;                  // ... as a Shape2 with no holes

    Vec2 position() const { return cur_; }
    Real heading() const { return dir_; }  // radians

private:
    std::vector<Edge2> edges_;
    Vec2 cur_{0, 0};
    Real dir_ = 0;
    EdgeTag tag_ = EdgeTag::None;
    bool started_ = false;
};

}  // namespace engine

#endif
