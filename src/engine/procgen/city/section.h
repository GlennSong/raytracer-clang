#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SECTION_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SECTION_H

#include "polygon.h"
#include <cstdint>
#include <vector>

namespace engine {

// THE SECTION (architectural toolkit, issue #105, foundation B; ADR-0174): a building's outline as EDGES, not as a
// polygon -- a line, an arc, a spline -- with holes. It sits UPSTREAM of the storey view: everything downstream of
// storeyPlans keeps reading the Poly2 its tessellation gives, corner for corner, so a building drawn from a section
// is the building drawn from its plan. Step 2a (this): lines only, the rectangle, the n-gon the drum is drawn with,
// a lot's plan, the grammar's own offset. Step 2b: arcs and splines tessellated within a tolerance with the corners
// kept, holes, and booleans by Clipper2 with the chord runs re-tagged as arcs.
struct Edge {
    enum class Kind : uint8_t { Line, Arc, Spline };
    Kind kind = Kind::Line;
    Vec2 a, b;                 // the ends (world XZ)
    Real bulge = 0;            // Arc: the SAGITTA (m) the arc rises over its chord, signed: left of a->b positive
    std::vector<Vec2> ctrl;    // Spline: the control points between a and b (owed)
    Real length() const;       // along the edge (an arc's arc length)
};

// A tessellation that remembers its edges: `edgeOf[i]` is the section edge the Poly2 edge points[i] -> points[i+1]
// lies on, so the facade can lay one run of bays along an arc's chords.
struct Tessellation {
    Poly2 points;
    std::vector<int> edgeOf;
};

struct Section {
    std::vector<Edge> outer;                 // CCW, each edge's b the next edge's a
    std::vector<std::vector<Edge>> holes;    // as given

    bool empty() const { return outer.size() < 3; }
    // The outline as the grammar reads it: lines give their corners (every edge's `a`); an arc its chords, each
    // rising no more than `tol` off the arc, its ends kept as corners so the facade's bays know their edges.
    Poly2 tessellate(Real tol = 0.25) const;
    Tessellation tessellation(Real tol = 0.25) const;
    std::vector<Poly2> holesTessellated(Real tol = 0.25) const;

    // The words.
    static Section rect(Real w, Real d, const Vec2& centre = Vec2(0, 0));       // CCW from the (-w/2, -d/2) corner
    static Section ngon(int n, Real r, const Vec2& centre = Vec2(0, 0));         // corner k at angle 2*pi*k/n: the drum's circle
    static Section fromLot(const Poly2& plan);                                   // every edge a line; CCW
    Section offset(Real d) const;                                                // d > 0 inset, d < 0 outset: the grammar's offsetPlan
    Section withHole(const Section& hole) const;
    // THE ROUNDED CORNERS: every corner between two lines becomes an arc of radius `r` tangent to both (trimmed to
    // what the edges can give). A rectangle rounded by half its depth is a STADIUM: two semicircular ends.
    Section rounded(Real r) const;
    Section transformed(const Vec2& origin, const Vec2& u, const Vec2& v) const;   // local (x, y) -> origin + u x + v y
};

}  // namespace engine

#endif
