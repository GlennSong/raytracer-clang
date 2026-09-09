#ifndef RAYTRACER_ENGINE_PROCGEN_LANELAB_DECK_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_LANELAB_DECK_MESH_H

// lanelab -> engine RenderMesh, through MeshBuilder's winding rule. World is (x, height, z):
// lanelab's plan Vec2 (x, y) lands on (x, z). Decks carry lane-local UVs (u = lateral metres
// from the owning lane's centreline, v = station along its parent spine).

#include "engine/procgen/lanelab/lanelab.h"
#include "renderer/renderer.h"
#include <string>
#include <vector>

namespace engine {
namespace lanelab {

struct NamedMesh { std::string name; RenderMesh mesh; Vec3 color; };

// One mesh per material: asphalt (deck tops and bottoms), concrete (walls, piers), sidewalk,
// shoulder, median, paint white, paint yellow, terrain. Empty meshes are omitted.
std::vector<NamedMesh> buildMeshes(const Result& r);

// The parapet runs the mesher sweeps: a chain of deck-boundary points on an elevated freeway or ramp lane.
// Exposed so a diagnostic can ask where a wall ENDS — a run whose cap sits on same-level pavement is a wall
// standing in the middle of drivable road ("the freeway walls end up blocking the car", Glenn, 2026-09-07).
// What a freeway or ramp deck edge FACES (2026-09-08). Today one geometric rule builds one product — a
// 0.9 m parapet wherever the deck stands 1.5 m clear of the ground — so an at-grade freeway beside a
// frontage road gets nothing at all (metro: 45 % of freeway edge carries anything). Naming the role is the
// first half of an edge grammar: the class table then says what to build for each (Glenn, 2026-09-08:
// "if it's a lane on the interior or exterior edge it should build some kind of wall/barrier/fence").
// Freeway and ramp classes only; streets are the lot pass's business.
enum class EdgeRole : uint8_t {
    Seam = 0,    // same-level pavement just beyond: not an edge of the structure at all, never walled
    Median,      // faces the other carriageway of the same freeway across the median
    VsStreet,    // faces a street, collector or arterial within reach — the freeway/city separation
    Elevated,    // outward edge of a deck standing clear of the ground: structure, needs a parapet
    AtGrade,     // outward edge sitting on the ground with nothing beyond: today gets nothing
    Count_
};
const char* edgeRoleName(EdgeRole role);
// Metres of freeway/ramp deck edge per role, and how much of it carries a barrier today.
struct ParapetCensus { double metres[static_cast<size_t>(EdgeRole::Count_)] = {}; double built[static_cast<size_t>(EdgeRole::Count_)] = {}; };

struct ParapetRun {
    std::vector<Vec2> pts;
    std::vector<double> z, offOuter, offInner;
    int lane = -1;          // the owning lane of the run's first edge
    bool closed = false;
    bool capOnPavement[2] = {false, false};   // start cap, end cap: same-level pavement just beyond it
    EdgeRole role = EdgeRole::Elevated;       // the role of the run's first edge
    BarrierKind kind = BarrierKind::Wall;     // what to build: one kind per run
    double height = 0.9;
};
std::vector<ParapetRun> parapetRuns(const Result& r, ParapetCensus* census = nullptr);

}  // namespace lanelab
}  // namespace engine

#endif
