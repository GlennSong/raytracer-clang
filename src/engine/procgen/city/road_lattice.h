#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_LATTICE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_LATTICE_H

#include "road_mesh.h"                     // UnionSpine
#include "../../../renderer/renderer.h"    // RenderMesh
#include <functional>
#include <vector>

namespace engine {

// The swept-lattice road mesher (docs/road-mesher-research.md §2). A road is a
// swept surface: a fixed cross-section PROFILE dragged along the spine's
// centreline. Sweeping it as an indexed lattice (MeshBuilder::emitLattice) gives
// shared vertices, real topology, and per-road UV — the things the earcut union
// mesher throws away. This header defines the profile and the sweeper; the
// per-class profiles (freeway, street) live in the .cpp.

// One column of a cross-section profile — a point in the (lateral, height) plane
// that the sweeper places on every ring. The lattice is the columns x the rings.
struct ProfileCol {
    // Lateral offset from the centreline = edgeFrac * halfWidthHere + absOffset.
    // A deck edge is edgeFrac = +/-1 (it flares with the deck); a parapet's inner
    // face is edgeFrac = +/-1 with absOffset = -thickness (absolute inboard of the
    // verge). The centreline is edgeFrac = 0.
    double edgeFrac = 0.0;
    double absOffset = 0.0;
    // Height above the deck plane at this lateral (+ up). 0 = the drivable
    // surface; a parapet top is +0.85; a soffit is -thickness.
    double height = 0.0;
    // Cross-section normal, in the ring frame: cnLat along +left, cnVert along
    // +worldUp. A deck band is (0, 1) — the sweeper tilts it by the ring's
    // cross-slope. A wall face is (+/-1, 0). A soffit is (0, -1). Authored, not
    // guessed, so a crease is just two columns at one lateral with different
    // normals (the "split columns" of the design).
    double cnLat = 0.0;
    double cnVert = 1.0;
    float mu = 2.0f;                 // shader lateral paint coord (carriageway 1..3)
    Vec3 color{0.10, 0.10, 0.11};
};

struct RoadProfile {
    std::vector<ProfileCol> cols;   // ordered across the section, one side to the other
};

// An arc-length window [s0, s1] along the chain where a profile is SUPPRESSED —
// a parapet gapped over a ramp gore, say (the blocked-merge fix as data). The
// sweep simply does not place the profile's rings inside the window, so there is
// a clean opening, no zero-area geometry.
struct GapWindow { double s0 = 0, s1 = 0; };

// Sweep `spine` through `profile` into a lattice (several, if `gaps` splits it).
// Rings are placed by arc length, so the surface conforms without the earcut
// mesher's after-the-fact refinement. `ground(x,z)` sets the base deck Y where
// `spine.yAbs` is empty (a draped street); an authored deck (freeway/ramp) rides
// yAbs exactly. Banking from crossSlope, flare from per-point hw. UV: u = the
// column's mu, v = arc length. `gaps` (optional) opens the profile over each
// window — each surviving run is its own shared-vertex lattice.
RenderMesh sweepRoadLattice(const UnionSpine& spine, const RoadProfile& profile,
                            const std::function<double(double, double)>& ground,
                            double ringStep = 3.0,
                            const std::vector<GapWindow>* gaps = nullptr,
                            std::vector<Vec3>* ring0Out = nullptr,
                            std::vector<Vec3>* ringNOut = nullptr);

// The whole cross-section of a divided elevated freeway, meshed as a set of
// shared-vertex lattices: the drivable deck top + the viaduct underside
// (fascia + soffit) + edge parapets + the median wall. `gaps` gaps the edge
// parapets over each ramp gore so a car can merge (the blocked-merge fix).
// `deckThickness` is the slab depth; parapet/median heights follow the kit.
RenderMesh sweepFreewayDeck(const UnionSpine& spine,
                            const std::function<double(double, double)>& ground,
                            double ringStep = 3.0, double deckThickness = 0.5,
                            const std::vector<GapWindow>* gaps = nullptr);

// --- Component profiles (exposed for tests / reuse) --------------------------
// The drivable deck top: carriageway columns at mu in [1,3], centre 2, so the
// shader paints asphalt + lane lines instead of sidewalk concrete.
RoadProfile freewayDeckProfile(int lanesPerSide = 3, double laneWidth = 3.6);
// The viaduct underside: down the near fascia, across the soffit, up the far
// fascia — a down-facing structural band `thickness` below the deck.
RoadProfile freewayUndersideProfile(double thickness = 0.5);
// An edge parapet standing `height` above the deck at the `+1` (left) or `-1`
// (right) verge, `thickness` inboard of it.
RoadProfile parapetProfile(double side, double height = 0.85, double thickness = 0.28);
// The solid median wall down the centreline, `halfWidth` each way, `height` tall.
RoadProfile medianProfile(double halfWidth = 0.35, double height = 0.85);

// A draped STREET cross-section: raised sidewalk + curb on each side, then the
// carriageway (`lanesPerSide` each way) at mu in [1,3]. The carriageway edge ->
// curb-top is a vertical curb face; the sidewalk sits `curbHeight` above the
// road. Enough lateral columns (one per lane boundary) that, with rings placed
// near the lane width, the quads are ~square — so the surface has interior
// vertices to conform to terrain instead of a 2-triangle ribbon.
RoadProfile streetProfile(int lanesPerSide = 1, double sidewalkWidth = 3.0,
                          double curbHeight = 0.15);

// A one-way RAMP deck: carriageway + underside + edge parapets, no median. The
// ramp rides its spine's authored yAbs from the deck down to the street.
RenderMesh sweepRampDeck(const UnionSpine& spine,
                         const std::function<double(double, double)>& ground,
                         double ringStep = 3.0, double deckThickness = 0.5);

// The whole corridor as swept lattices: the divided mainline deck (parapets
// gapped over `deckGaps` so ramps can merge), every ramp, and the support piers
// under the elevated spans. `pierBasesOut`, if given, receives the pier
// footprints (world XZ) so the lot/vegetation passes can treat them as used
// ground. This is the one call the loader makes in place of weldSolid.
struct CorridorLatticeParams {
    double ringStep = 3.0;
    double deckThickness = 0.5;
    std::function<double(double, double)> ground;
    std::vector<GapWindow> deckGaps;
    std::vector<Vec2>* pierBasesOut = nullptr;
};
RenderMesh sweepCorridor(const UnionSpine& deckSpine,
                         const std::vector<UnionSpine>& rampSpines,
                         const CorridorLatticeParams& params);

// --- Junction patch (road-mesher-research.md §3) -----------------------------
// A Coons (transfinite bilinear) patch over four boundary curves — the drivable
// surface where roads meet. `bottom`/`top` share the u-direction (nu+1 points
// each); `left`/`right` the v-direction (nv+1 each); corners must coincide
// (bottom[0]=left[0], bottom.back()=right[0], top[0]=left.back(),
// top.back()=right.back()). Every interior vertex INTERPOLATES all four
// boundaries and the bilinear corner blend, so a junction between arms arriving
// at different heights ramps smoothly instead of STEPPING across the medial axis
// — the causal fix for weldSolid's 164%-grade pad faces. Emits one shared-vertex
// lattice (nv+1 rings x nu+1 columns), mu/color flat across the pad.
RenderMesh coonsPatch(const std::vector<Vec3>& bottom, const std::vector<Vec3>& right,
                      const std::vector<Vec3>& top, const std::vector<Vec3>& left,
                      float mu = 2.0f, const Vec3& color = Vec3(0.10, 0.10, 0.11));

// One road arriving at a junction node: its outward direction and its mouth ring
// — the carriageway cross-section (left verge -> right verge, looking OUTWARD
// along the arm) at the junction boundary. The mouth IS the body's end ring, so
// the patch and the body share it.
struct JunctionArm {
    Vec2 dir{1, 0};              // outward unit direction from the node
    std::vector<Vec3> mouth;     // K+1 CARRIAGEWAY points, left verge -> right verge
};

// The drivable junction pad for a node, from its incident arms (any order —
// sorted by bearing internally). A 4-arm node gets a Coons grid whose four sides
// are the arm mouths (corners snapped to the shared kerb points), so the height
// interpolates and the pad matches every arm exactly. Other degrees get a
// height-interpolated centroid fan for now (the T / N>=5 quad templates follow).
// Returns empty for degree <= 2 (the body runs straight through).
// ONE fill for every degree (roads-v2 plan, Part 2 step 4): sort arms CCW, join
// their carriageway mouths into a single closed boundary loop (adjacent verges
// meet at shared corner points), ear-clip the loop (adds no boundary points, so
// each arm's division count is honored exactly), lift by the boundary heights.
// Interim state of the real pipeline: corner ARCS and quad PAIRING land next;
// there are no per-degree templates and no side strips — those are deleted.
RenderMesh junctionPatch(std::vector<JunctionArm> arms,
                         float mu = 2.0f, const Vec3& color = Vec3(0.10, 0.10, 0.11));

}  // namespace engine

#endif
