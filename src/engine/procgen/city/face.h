#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_FACE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_FACE_H

#include "polygon.h"
#include <cstdint>

namespace engine {

// THE FACE (architectural toolkit, issue #105, foundation B; ADR-0174): a facade surface the layout words lay their
// bays on, parametrised (x across, y up) -- the one face type every facade word in shape_grammar.cpp takes. (It was
// two until #117 item 35: the toolkit's Face beside the grammar's private FaceRect, copied field by field.)
struct Face {
    Vec3 bl;              // the bottom-left corner (world)
    Vec3 h;               // the unit horizontal axis
    Vec3 v;               // the unit vertical axis
    Vec3 n;               // the outward normal
    Real width = 0, height = 0;
    // A RULED face (issue #105 step 4, Glenn: "the twisting ... should be continuous and look like one curve"):
    // the top edge is its own segment, tl -> tr, the storey above's outline; at(x, y) rules between the two edges,
    // so a twisted or bulging storey's glass runs from its floor outline to its ceiling's in one surface.
    bool ruled = false;
    Vec3 tl, tr;

    Vec3 at(Real x, Real y) const {
        if (!ruled || width < 1e-9 || height < 1e-9) return bl + h * x + v * y;
        const Vec3 b = bl + h * x;
        const Vec3 t = tl + (tr - tl) * (x / width);
        return b + (t - b) * (y / height);
    }

    static Face flat(const Vec3& bl, const Vec3& h, const Vec3& v, const Vec3& n, Real width, Real height);
    // The face of plan edge `i` (CCW plan) from height `y` up `h`: outward to the right of a->b.
    static Face ofEdge(const Poly2& plan, std::size_t i, Real y, Real h);
    // The RULED face of edge `i` between a storey's floor outline `lower` (at y) and its ceiling's `upper` (at
    // y + h), the two outlines corner for corner (one section under a keyed transform); the normal from the face's
    // middle. Flat (ofEdge) when the outlines do not match corner for corner.
    static Face ofEdgeRuled(const Poly2& lower, const Poly2& upper, std::size_t i, Real y, Real h);
};

}  // namespace engine

#endif
