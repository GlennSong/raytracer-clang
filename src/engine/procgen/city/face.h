#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_FACE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_FACE_H

#include "polygon.h"
#include <cstdint>

namespace engine {

// THE FACE (architectural toolkit, issue #105, foundation B; ADR-0174): a facade surface the layout words lay their
// bays on, parametrised (u across, v up). Flat (step 2a, this): a storey edge's rectangle -- the FaceRect every
// facade word in shape_grammar.cpp takes, built here so the two are one arithmetic. Cylindrical (an arc edge) and
// Ruled (an edge whose outline differs above and below) are step 2b.
struct Face {
    enum class Kind : uint8_t { Flat, Cylindrical, Ruled };
    Kind kind = Kind::Flat;
    Vec3 bl;              // the bottom-left corner (world)
    Vec3 h;               // the unit horizontal axis
    Vec3 v;               // the unit vertical axis
    Vec3 n;               // the outward normal
    Real width = 0, height = 0;

    Vec3 at(Real u, Real vv) const { return bl + h * u + v * vv; }
    Vec3 normal(Real, Real) const { return n; }

    static Face flat(const Vec3& bl, const Vec3& h, const Vec3& v, const Vec3& n, Real width, Real height);
    // The face of plan edge `i` (CCW plan) from height `y` up `h`: outward to the right of a->b.
    static Face ofEdge(const Poly2& plan, std::size_t i, Real y, Real h);
};

}  // namespace engine

#endif
