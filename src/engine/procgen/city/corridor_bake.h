#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_BAKE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_BAKE_H

// Roads-v2 S3 (plan §1.3): the corridor solver becomes a BAKE that emits
// GRAPH. The solved freeway (alignment + vertical profile + authored ramps)
// is appended to the editable street RoadNet as ordinary nodes and edges:
//
//   - mainline -> nodes every ~mainlineStep m (absolute deck heights in
//     nodeElev), SPLIT at every gore so the gore is a real degree>=3 node;
//     edges klass=Freeway, spec="freeway3".
//   - each ramp -> a chain from its GORE NODE (which IS a mainline node —
//     shared id, the proof of graph membership) down the authored descent to
//     a LANDING NODE that reuses the street's own node when one lies within
//     landingSnap; edges klass=Ramp, spec="ramp1".
//
// After the bake there is ONE editable graph containing streets, freeways and
// ramps; the editor's node handles and drags work on all of them, because
// they are all just RoadNet nodes. `CorridorDef` remains only the transient
// solver input; nothing downstream needs it once the graph exists.

#include "corridor_mesh.h"   // CorridorDef, RampPath, CorridorAuthoring
#include "road_net.h"        // RoadNet

namespace engine {

struct CorridorBakeParams {
    double mainlineStep = 40.0;   // graph node spacing along the mainline (m)
    // Reuse a street node within this radius (m). The authored ramp path stops
    // landingSetback (~20 m) SHORT of its street target — the street system owns
    // that last stretch — so the snap must cover setback + margin: the ramp's
    // final graph edge IS the graft the loader used to build as a stub.
    double landingSnap = 26.0;
};

// Appends the corridor to `net` (nodes, nodeElev, edges + parallel arrays,
// specs). `rampPaths` must be corridorAuthor's output for the SAME def (it is
// index-parallel to def.exits; empty paths = dropped exits, skipped). Returns
// the baked mainline node indices in station order (tests/diagnostics).
std::vector<int> bakeCorridorIntoNet(RoadNet& net, const CorridorDef& def,
                                     const std::vector<RampPath>& rampPaths,
                                     const CorridorBakeParams& params = {});

}  // namespace engine

#endif
