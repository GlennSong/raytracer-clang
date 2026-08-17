#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_BAKE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_BAKE_H

// Roads-v2 S3 (plan §1.3): the corridor solver becomes a BAKE that emits
// GRAPH. The solved freeway (alignment + vertical profile + authored ramps)
// is appended to the editable street RoadEntity as ordinary nodes and edges:
//
//   - mainline -> nodes every ~mainlineStep m (absolute deck heights on the
//     nodes), SPLIT at every gore so the gore is a real degree>=3 node;
//     edges klass=Freeway, spec="freeway3".
//   - each ramp -> a chain from its GORE NODE (which IS a mainline node —
//     shared id, the proof of graph membership) down the authored descent to
//     a LANDING NODE that reuses the street's own node when one lies within
//     landingSnap; edges klass=Ramp, spec="ramp1".
//
// After the bake there is ONE editable graph containing streets, freeways and
// ramps; the editor's node handles and drags work on all of them, because
// they are all just RoadEntity nodes. `CorridorDef` remains only the transient
// solver input; nothing downstream needs it once the graph exists.

#include "corridor_mesh.h"   // CorridorDef, RampPath, CorridorAuthoring
#include "road_net.h"        // RoadEntity

namespace engine {

struct CorridorBakeParams {
    double mainlineStep = 40.0;   // graph node spacing along the mainline (m)
    // Reuse a street node within this radius (m). The authored ramp path stops
    // landingSetback (~20 m) SHORT of its street target — the street system owns
    // that last stretch — so the snap must cover setback + margin: the ramp's
    // final graph edge IS the graft the loader used to build as a stub.
    double landingSnap = 26.0;
    // Ramp node spacing (m). The bake RESAMPLES the authored 3 m polyline to
    // sparse spline nodes and stores TANGENTS from the authored curve, so the
    // net's Hermite sampling reproduces the clothoid between editable handles
    // (roads-v2.1 2a — "a few spline points with tangents", not hundreds).
    double rampStep = 25.0;
};

// Appends the corridor to `road.graph` (nodes with deck heights + tangents,
// baked edges, specs). `rampPaths` must be corridorAuthor's output for the SAME
// def (it is index-parallel to def.exits; empty paths = dropped exits, skipped).
// `heightAt` is the level terrain the planarity audits measure against (null =
// flat). Returns the baked mainline node indices in station order
// (tests/diagnostics).
std::vector<int> bakeCorridorIntoNet(RoadEntity& road, const CorridorDef& def,
                                     const std::vector<RampPath>& rampPaths,
                                     const CorridorBakeParams& params = {},
                                     const RoadGroundFn& heightAt = nullptr);

// The bake-time planarity audit (#18), exposed for the mainline warn + tests:
// does any edge in [e0, end) cross an earlier edge in XZ with less than
// `clearance` metres of vertical gap?
bool chainCrossesNet(const RoadEntity& road, std::size_t e0, double clearance,
                     const RoadGroundFn& heightAt = nullptr);

}  // namespace engine

#endif
