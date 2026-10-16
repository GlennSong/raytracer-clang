#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_MESH_MASS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_MESH_MASS_H

// THE MESH MASS (architectural toolkit, issue #105, step 6; ADR-0177): the artist's hatch. A closed mesh --
// modelled in Blender, exported as glTF -- is a building's form: its slice at a height is the mesh cut by that
// plane (every triangle's crossing segment, the segments linked into loops, the largest loop kept), simplified
// and RESAMPLED to a fixed corner count by arc length from a fixed bearing, so consecutive storeys correspond
// corner for corner and the glass is ruled between them (ADR-0175). The mesh is fitted to the lot: its footprint
// scaled onto the box inside the plan (the plan's own box when it is a rectangle; on an L, the box the twist turns
// in), its height onto the storeys'. The ground storeys stay the plan as drawn.

#include "mass.h"
#include <string>
#include <vector>

namespace engine {

// The mesh cut by the plane y: the largest closed loop, CCW, simplified within `tol` and, when `corners` > 0,
// resampled to that many corners by arc length starting from the point nearest the +x bearing from the centroid.
Poly2 sliceMesh(const RenderMesh& mesh, Real y, Real tol = 0.3, int corners = 0);
// A loop resampled to `n` points by arc length, the first exactly where the ray from the loop's centroid along
// `from` crosses it (so a turning form's start never snaps between vertices storey to storey); CCW kept.
Poly2 resampleLoop(const Poly2& loop, int n, const Vec2& from = Vec2(1, 0));

struct MeshMass : Mass {
    RenderMesh mesh;         // object space, fitted: the footprint on the plan's box, y from 0 to the height
    Poly2 base;              // the ground storeys' outline
    int baseStoreys = 1;
    Real groundH = 4.0, floorH = 3.4;
    int floors = 0;
    int corners = 32;        // every slice resampled to this many corners (0: as the cut gives)
    Real tol = 0.3;
    Slice slice(Real y) const override;
    Real height() const override { return groundH + floors * floorH; }
    MassBounds bounds() const override;
    RenderMesh shell(int lod, const Vec3& colour) const override;   // lod 1: the mesh itself; 0: the loft of slices
};

// THE MASS LIBRARY: the glTF files in `assets/masses`, sorted by name; `massMesh` on BuildingParams is 1 + the
// index. Empty when the folder is missing.
const std::vector<std::string>& massLibrary();
// The Mesh envelope's mass: library entry `params.massMesh` fitted to `plan` and the storeys; an empty mesh
// (no library, no file, or a lot whose inside box is too tight to hold the core) gives a MeshMass whose slices
// are the plan.
MeshMass meshOf(const Poly2& plan, const BuildingParams& params);
// Fit any mesh to a plan: translated and scaled so its footprint's box lies on the plan's oriented box and its
// height runs from 0 to `height`.
void fitMeshToPlan(RenderMesh& mesh, const Poly2& plan, Real height);
// No storey narrower than the core needs (kCoreTierMin + 3 m, the bulge's and the curves' floor): a fitted mesh
// whose narrowest storey slice (the storeys above `mm.baseStoreys`, sliced as `mm` slices) is too narrow has its
// footprint pushed toward the plan's box -- a point on the box stays, a point inside moves out by a share of the
// way -- until it is wide enough. A mass that cannot hold its core cannot be climbed.
void widenMeshForCore(RenderMesh& mesh, const Poly2& plan, const MeshMass& mm);

}  // namespace engine

#endif
