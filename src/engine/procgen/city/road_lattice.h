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
    // Index into the per-station barrier-scale array (see sweepRoadLattice), or
    // -1 for a column whose height is fixed. A parapet/median column keys its
    // height off a channel so it can be GAPPED (scale 0) across a ramp gore —
    // the blocked-merge fix as data, not geometry surgery.
    int barrier = -1;
};

struct RoadProfile {
    std::vector<ProfileCol> cols;   // ordered across the section, one side to the other
    int barrierChannels = 0;        // how many independent gappable height channels
};

// Sweep `spine` through `profile` into ONE lattice. Rings are placed by arc
// length along the chain (dense in curves / where the deck sags), so the surface
// conforms without the earcut mesher's after-the-fact refinement. `ground(x,z)`
// sets the base deck Y where `spine.yAbs` is empty (a draped street); an authored
// deck (a freeway/ramp) rides yAbs exactly. `barrierScale`, if given, is indexed
// [ring][channel] and scales each barrier column's height per station — 0 opens a
// gap. UV: u = the column's mu, v = arc length in metres.
RenderMesh sweepRoadLattice(const UnionSpine& spine, const RoadProfile& profile,
                            const std::function<double(double, double)>& ground,
                            double ringStep = 3.0,
                            const std::vector<std::vector<double>>* barrierScale = nullptr);

// The drivable DECK-TOP profile of a divided freeway: two carriageways of
// `lanesPerSide` lanes with a paved median gap and outer verges, parameterised so
// the RoadMarkings shader paints asphalt + lane lines (mu in [1,3]) instead of
// sidewalk concrete. Structure (fascia/soffit) and furniture (parapets/median
// wall) are added as further profiles/columns in later stages.
RoadProfile freewayDeckProfile(int lanesPerSide = 3, double laneWidth = 3.6);

}  // namespace engine

#endif
