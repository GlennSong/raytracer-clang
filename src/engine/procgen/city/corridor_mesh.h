#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_MESH_H

#include "alignment.h"
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
    RenderMesh barrier;     // median Jersey barrier + piers (concrete)
    std::vector<TerrainFlatten> flatten;   // at-grade windows carve to deck
};

CorridorMeshOut buildCorridorMesh(
    const CorridorDef& corridor,
    const std::function<Real(Real, Real)>& ground,
    Real step = 3.0);

}  // namespace engine

#endif
