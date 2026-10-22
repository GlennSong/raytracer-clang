#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_MASS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_MASS_H

// THE MASS (architectural toolkit, issue #105, step 4; ADR-0174): anything that can give its outline at a height --
// a SLICE -- and its skin as a mesh. A building's storeys are slices of its mass at the storey heights, which is the
// bridge from foundation B (sections and masses) into foundation A (the storey view the planners, the facade, the
// collider and the census read): sliceStoreys. Mass sources sit behind this one interface: the STACK (mass_stack.h:
// sections over storey ranges under keyed transforms, today's tiers as the first preset), and later the sweep, the
// implicit form, the artist's mesh and the complex of cells.

#include "polygon.h"
#include "section.h"
#include "shape_grammar.h"   // StoreyPlan, BuildingParams, RenderMesh
#include <vector>

namespace engine {

// One height's outline(s): disjoint pieces, each a section with its holes.
struct Slice {
    std::vector<Section> pieces;
    int tier = 0;   // the mass's own tier at this height (a stack's entry): what the storey view calls `tier`
    bool empty() const { return pieces.empty() || pieces.front().empty(); }
    const Section& outline() const { return pieces.front(); }   // the (largest) piece
};

struct MassBounds {
    Vec2 lo, hi;        // in plan
    Real y0 = 0, y1 = 0;   // above the base
};

struct Mass {
    virtual ~Mass() = default;
    virtual Slice slice(Real y) const = 0;         // the outline at height y above the base (y < 0: the base's)
    virtual Real height() const = 0;               // to the top of the last storey
    virtual MassBounds bounds() const = 0;
    // The skin: lod 0 the loft of the storey slices (side faces between consecutive outlines), lod 1 the native
    // form where the source has one (a sweep's or an SDF's polygonised skin); a stack's is the loft either way.
    virtual RenderMesh shell(int lod, const Vec3& colour) const = 0;
};

// THE BRIDGE into the storey view: storey 0 the ground (the base slice, `groundHeight` tall), then `floors`
// storeys of `floorHeight`, each the slice at its own floor height. The tier is the mass's own (a stack's entry).
std::vector<StoreyPlan> sliceStoreys(const Mass& mass, const BuildingParams& params);

// The loft of a run of outlines at heights: side quads between consecutive outlines with the same corner count
// (ruled faces), a prism's walls where the outline holds; the last outline's cap when `cap`.
void loftSlices(RenderMesh& out, const std::vector<Poly2>& outlines, const std::vector<Real>& heights, const Vec3& colour, bool cap);

}  // namespace engine

#endif
