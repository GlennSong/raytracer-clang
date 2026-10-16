#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_IMPLICIT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_IMPLICIT_H

// THE IMPLICIT MASS (architectural toolkit, issue #105, step 6; ADR-0177): a building whose form is a signed
// distance field -- the engine's own SDF kit (procgen/sdf.h: spheres, boxes, capsules, unions, blends) with the
// words a building needs on top: a PLAN EXTRUDED into a prism, and WARPS of the space the field is read in (a
// wave along the facade, running up the storeys). Its slice at a height is the field's zero contour on that
// plane, traced by marching squares and simplified to the polygon every grower reads: a storey outline that no
// affine transform of a section could give (an undulating glass skin, Aqua Tower's waves) feeds the same facade,
// planners, collider and census as a drawn plan, storey by storey, the slab bands closing the seams.

#include "mass.h"
#include "../sdf.h"
#include <vector>

namespace engine {

// The 2D distance to a CCW plan in the XZ plane (negative inside), the same at every height.
Sdf sdfPlan(const Poly2& plan);
// A plan extruded between two heights.
Sdf sdfExtrude(const Poly2& plan, Real y0, Real y1);
// THE WAVE: the field read at a point displaced along `along` (a unit plan direction) by
// amplitude * sin(2 pi s / wavelength + 2 pi y / riseLength), s the plan coordinate across `along`: the skin
// undulates along the facade and the undulation climbs the building (riseLength 0: it does not climb).
Sdf sdfWave(Sdf field, Real amplitude, Real wavelength, const Vec2& along, Real riseLength);

// The field's zero contour at height `y` over `bounds` (XZ), sampled every `cell` metres (marching squares),
// the largest loop, simplified so no vertex is further than `tol` off the straight line replacing it. CCW.
Poly2 sliceSdf(const Sdf& field, Real y, const SdfBounds& bounds, Real cell = 0.5, Real tol = 0.3);
// Douglas-Peucker on a closed loop.
Poly2 simplifyLoop(const Poly2& loop, Real tol);

struct ImplicitMass : Mass {
    Sdf field;
    SdfBounds box;          // where to look for the form (XZ; y is the storeys')
    Poly2 base;             // the ground storeys' outline (storeys 0..baseStoreys are the plan as drawn)
    int baseStoreys = 1;
    Real groundH = 4.0, floorH = 3.4;
    int floors = 0;
    Real cell = 0.5, tol = 0.3;
    Slice slice(Real y) const override;
    Real height() const override { return groundH + floors * floorH; }
    MassBounds bounds() const override;
    RenderMesh shell(int lod, const Vec3& colour) const override;   // lod 1: the field polygonised; 0: the loft of slices
};

// THE WAVE ENVELOPE'S MASS: the plan extruded and waved by `params.wave` metres over `params.waveLen` along the
// plan's long axis, the wave climbing one cycle every `waveLen` of height; the ground storeys (to `baseFloors`)
// the plan itself.
ImplicitMass implicitOf(const Poly2& plan, const BuildingParams& params);

}  // namespace engine

#endif
