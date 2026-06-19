#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_POLYGON_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_POLYGON_H

#include "../../../rt_math.h"   // Real
#include <vector>

namespace engine {

// 2D geometry in the city ground plane. The whole road/block/parcel pipeline
// (city-generation-plan.md §3, ADR-0038) is planar — terrain height is sampled
// only when geometry is finally placed — so it runs on these types, not Vec3.
// Convention: a Vec2 maps to world XZ (.x = world x, .y = world z), and the
// "up" cross product points along +world-Y, so CCW winding in (x,z) is the
// canonical front-facing/positive-area orientation.

struct Vec2 {
    Real x = 0, y = 0;
    Vec2() = default;
    Vec2(Real x, Real y) : x(x), y(y) {}

    Vec2 operator+(const Vec2& v) const { return {x + v.x, y + v.y}; }
    Vec2 operator-(const Vec2& v) const { return {x - v.x, y - v.y}; }
    Vec2 operator*(Real t) const { return {x * t, y * t}; }
    Vec2 operator/(Real t) const { return {x / t, y / t}; }
    Vec2 operator-() const { return {-x, -y}; }
    Vec2& operator+=(const Vec2& v) { x += v.x; y += v.y; return *this; }

    Real length() const { return std::sqrt(x * x + y * y); }
    Real lengthSquared() const { return x * x + y * y; }
};

inline Vec2 operator*(Real t, const Vec2& v) { return v * t; }
inline Real dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
// 2D scalar cross (z-component of the 3D cross): >0 when b is CCW from a.
inline Real cross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }
inline Vec2 normalize(const Vec2& v) {
    Real len = v.length();
    return len > 0 ? v / len : v;
}
// Left normal (rotate +90°): for a CCW polygon edge this points outward... see
// inset(), which uses the inward direction explicitly.
inline Vec2 perp(const Vec2& v) { return {-v.y, v.x}; }
inline Real distance(const Vec2& a, const Vec2& b) { return (a - b).length(); }
inline Vec2 lerp(const Vec2& a, const Vec2& b, Real t) { return a + (b - a) * t; }

using Poly2 = std::vector<Vec2>;

// Signed area: positive when the ring is wound counter-clockwise in (x,y).
Real signedArea(const Poly2& poly);
inline Real area(const Poly2& poly) { return std::abs(signedArea(poly)); }
inline bool isCCW(const Poly2& poly) { return signedArea(poly) > 0; }
void ensureCCW(Poly2& poly);           // reverse in place if wound CW

Vec2 centroid(const Poly2& poly);      // area centroid (falls back to vertex mean)
Real perimeter(const Poly2& poly);
void bounds(const Poly2& poly, Vec2& lo, Vec2& hi);
bool pointInPolygon(const Poly2& poly, const Vec2& p);   // ray-cast, edge-inclusive-ish

// Oriented minimum-area bounding box, aligned to a convex-hull edge (the optimal
// rectangle is always edge-aligned). axis[0]/axis[1] are unit and perpendicular;
// half[i] is the half-extent along axis[i]. The parcel subdivision splits along
// the longer axis (city-generation-plan.md §3.3).
struct OBB2 {
    Vec2 center;
    Vec2 axis[2] = {{1, 0}, {0, 1}};
    Real half[2] = {0, 0};
    int longAxis() const { return half[0] >= half[1] ? 0 : 1; }
};
OBB2 orientedBoundingBox(const Poly2& poly);

// Andrew's monotone-chain convex hull (CCW, no collinear duplicates).
Poly2 convexHull(const Poly2& points);

// Sutherland–Hodgman clip of `poly` against the half-plane { p : dot(n,p) <= c }.
// Returns the kept (possibly empty) piece. Robust for convex and mildly concave
// blocks; the road→block faces this consumes are simple polygons.
Poly2 clipHalfPlane(const Poly2& poly, const Vec2& n, Real c);

// Split `poly` by the infinite line through `p` with direction `dir` into the two
// half-plane pieces (`left` = the dot(normal,·) <= side). Either piece may be empty.
void splitByLine(const Poly2& poly, const Vec2& p, const Vec2& dir,
                 Poly2& left, Poly2& right);

// Inset (offset inward) a polygon by `d` along each edge's inward normal,
// re-intersecting consecutive offset edges. Correct for convex polygons and good
// for the convex-ish blocks the pipeline produces; returns empty if the inset
// collapses or self-intersects (a sliver block, dropped by the caller).
Poly2 inset(const Poly2& poly, Real d);

}  // namespace engine

#endif
