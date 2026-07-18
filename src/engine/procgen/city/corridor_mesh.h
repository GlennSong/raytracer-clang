#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_MESH_H

#include "alignment.h"
#include "road_network.h"           // RoadGraph (piers dodge the streets below)
#include "road_mesh.h"              // UnionSpine (fold the corridor into the one welder)
#include "../terrain.h"                    // TerrainFlatten
#include "../../../renderer/renderer.h"    // RenderMesh
#include <functional>
#include <vector>

namespace engine {

// §11 (lego parts): the DRAWN centreline of one ramp — gore band riding the deck
// edge + free run to the street — with ABSOLUTE heights. The loader builds the nav
// chain from THESE points AND the weld builds the ramp's geometry from them, so
// the mesh and the graph can never disagree. Points run in FLOW order (an exit:
// deck -> street; an on-ramp: street -> deck).
struct RampPath {
    std::vector<Vec3> pts;
    // The GORE-BAND portion of pts — the piece that rides the flared deck
    // edge. It is JUNCTION surface (deck flare + gore wedge), not ramp
    // ribbon: exits carry it as the first `bandFront` points, on-ramps as
    // the last `bandBack` (flow order street -> merge). The graph bake
    // starts the editable ramp chain PAST the band (roads-v2.1 2a); the
    // band's surface belongs to the gore junction (2c).
    int bandFront = 0;
    int bandBack = 0;
};

// Everything a corridor authors that ISN'T drawn geometry (one-mesher
// P8b): the ramp centrelines and the terrain-flatten windows. These are the two
// outputs the weld path still needs from the old mesher — the nav graph and the
// ramp spines are both built from `rampPaths`, and `flatten` carves the ground
// to the deck where it runs at grade. Splitting them out is what lets the
// corridor's geometry pass be deleted while the authoring survives.
struct CorridorAuthoring {
    // DRAWN ramp centrelines with ABSOLUTE heights, in FLOW order (an exit:
    // deck -> street; an on-ramp: street -> deck). STRICTLY index-parallel to
    // CorridorDef::exits — a dropped ramp leaves an EMPTY entry rather than
    // shrinking the vector, because the loader indexes rampPaths[exitIndex].
    std::vector<RampPath> rampPaths;
    // At-grade windows that carve terrain to the deck/ramp plane. Elevated
    // spans emit none — the ground must stay under a viaduct, not follow it.
    std::vector<TerrainFlatten> flatten;
};

// Author a corridor's ramp centrelines + flatten windows. This is the corridor's
// whole non-geometry contribution: the ramp drop rules (too-short / folds-or-hooks
// / overlaps-the-mainline) and the aux-lane synthesis live here, and weldSolid
// draws everything. No `avoidRoads`: that only ever steered pier bents, which are
// geometry the weld now owns (and slides clear of the streets itself).
CorridorAuthoring corridorAuthor(const CorridorDef& corridor,
                                 const std::function<Real(Real, Real)>& ground,
                                 Real step = 3.0);

}  // namespace engine

#endif
