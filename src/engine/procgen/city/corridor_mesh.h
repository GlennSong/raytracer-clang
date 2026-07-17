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
struct RampPath { std::vector<Vec3> pts; };

// Fold the corridor into the ONE welder (road-unification, one-mesher P3): sample
// the corridor's alignment + vertical profile into UnionSpine(s) that `weldSolid`
// meshes — the deck's centreline with per-point absolute Y (`yAbs`), per-point
// half-width (aux/gore flares, P5), cross-slope (superelevation, P6) and class
// Freeway. This adapter is what lets the corridor's authoring core drive the
// unified mesher; there is no second geometry pass any more.
std::vector<UnionSpine> corridorDeckSpines(
    const CorridorDef& corridor,
    const std::function<Real(Real, Real)>& ground, Real step = 3.0);

// Turn the corridor's authored ramp centrelines (rampPaths — the DRAWN gore-band
// + free-run polyline with absolute heights that the nav graph is also built
// from) into ramp UnionSpines the ONE welder meshes: class Ramp, per-point
// absolute Y (`yAbs`), constant half-width. Because the weld and the nav chain
// BOTH consume rampPaths, they can never disagree — no truth-source inversion.
// The grade-sep split then descends each ramp from the deck and welds its low
// foot into the street grid below. A dropped ramp (empty pts) yields no spine.
std::vector<UnionSpine> corridorRampSpines(
    const std::vector<RampPath>& rampPaths, Real halfWidth = 3.6);

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

// Overhead SIGN GANTRIES for a corridor (one-mesher P7): at each exit gore, two
// posts, a beam across the deck, a wide green placard over the through lanes and a
// smaller drop-arrow placard over the peeling exit lane. Fed by the CorridorDef
// alone (no ribs), so the ONE welder's deck — which builds its own parapets,
// median, and piers — wears the signage the corridor still owns. Returns a single
// furniture mesh (concrete + green + white).
RenderMesh corridorFurniture(const CorridorDef& corridor);

}  // namespace engine

#endif
