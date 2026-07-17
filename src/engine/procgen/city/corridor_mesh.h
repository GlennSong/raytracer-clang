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

// CORRIDOR SWEEP (plan §8 P8.2): extrude the cross-section template —
// shoulder | lanes | median barrier | lanes | shoulder — along the alignment,
// with superelevation banking the deck through curves. Real lane geometry:
// every lane edge gets its own painted line (solid edge/yellow median lines,
// dashed separators), so the mesh is honest about lane count and the marking
// look no longer guesses from width. Where the profile rides above the
// ground the deck becomes a viaduct: fascia sides, an underside, and piers;
// where it returns to grade the terrain is flattened to the deck.
struct CorridorMeshOut {
    RenderMesh deck;        // asphalt top (+ fascia/underside where elevated)
    RenderMesh markings;    // painted lines, floated just above the deck
    RenderMesh barrier;     // median barrier, edge railings, bents (concrete)
    std::vector<TerrainFlatten> flatten;   // at-grade windows carve to deck
    // Pier bent footprints (world XZ) — the lot/vegetation passes treat these
    // as USED ground (device: "turn those pylons into a used lot so nothing
    // else can be built there").
    std::vector<Vec2> pierBases;
    // §11 (lego parts): the DRAWN centreline of each ramp — gore band riding
    // the deck edge + free run to the street — with ABSOLUTE heights. The
    // loader builds the nav chain from THESE points: one source of truth, the
    // mesh and the graph can never disagree. Parallel to CorridorDef.exits;
    // a dropped ramp leaves an empty path. Points run in FLOW order (an
    // exit: deck -> street; an on-ramp: street -> deck).
    struct RampPath { std::vector<Vec3> pts; };
    std::vector<RampPath> rampPaths;
};

// `avoidRoads`: pier bents SLIDE along the corridor (or the span lengthens)
// so no column lands inside a street corridor below (device feedback).
CorridorMeshOut buildCorridorMesh(
    const CorridorDef& corridor,
    const std::function<Real(Real, Real)>& ground,
    Real step = 3.0, const RoadGraph* avoidRoads = nullptr);

// Fold the corridor into the ONE welder (road-unification, one-mesher P3): sample
// the corridor's alignment + vertical profile into UnionSpine(s) that `weldSolid`
// meshes — the deck's centreline with per-point absolute Y (`yAbs`) and class
// Freeway. P3 authors a CONSTANT half-width (the widest station's symmetric
// half); per-point width for aux/gore flares (P5) and banking (P6) come later.
// This is the adapter that lets the corridor's authoring core drive the unified
// mesher instead of buildCorridorMesh's geometry.
std::vector<UnionSpine> corridorDeckSpines(
    const CorridorDef& corridor,
    const std::function<Real(Real, Real)>& ground, Real step = 3.0);

}  // namespace engine

#endif
