#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_PLAN_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CORRIDOR_PLAN_H

// Roads-v2 S3: the corridor PLANNING pipeline, extracted from the level loader
// so the editor's Regenerate can re-run it (plan §1.3.3: "dragging ... re-runs
// the bake — same regenerate flow streets use today"). Three stages, exactly
// the loader's §10.6/§12 network rules:
//
//   planCorridorRoutes    route plans -> engineered CorridorDefs (self-approach
//                         truncation R1.2b, crossing/parallel rule 2, the
//                         straightness + one-spine cull R1.4, feasibility-driven
//                         diamond placement R3f, the eased +9 m viaduct profile)
//   cutStreetsUnderCorridors  streets may pass UNDER a viaduct, never THROUGH
//                         an at-grade span (§10.6)
//   resolveCorridorLandings   each ramp claims a real street JUNCTION node on
//                         its own side (or splits the nearest street edge into
//                         a new T-junction), then the criss-cross guard drops
//                         any ramp pair whose runs still X (§10.3/§12)
//
// rebakeNetCorridors composes them with corridorAuthor + bakeCorridorIntoNet
// for ONE net — the editor's recipe-Regenerate entry point.

#include "alignment.h"     // CorridorDef
#include "road_net.h"      // RoadNet

#include <functional>
#include <utility>
#include <vector>

namespace engine {

// One freeway route plan (a metro `freewayPlans` polyline, or a rules-lab
// "freewayPlans" JSON plan) heading into the network-rules pipeline.
struct CorridorRouteInput {
    std::vector<Vec2> anchors;   // generator anchor polyline
    Real spacing = 700;          // desired interchange spacing (m)
    int srcNet = -1;             // which net grew this route (-1 = none/lab)
};

// A street node the diamond-placement feasibility scan may anchor a ramp to.
struct CorridorStreetAnchor {
    Vec2 pos;
    int degree = 0;
};

// A route that survived the rules: its engineered def + its dense centreline
// (pier-dodging guide for OTHER corridors) + the net it came from.
struct PlannedCorridor {
    CorridorDef def;
    std::vector<Vec2> guide;
    int srcNet = -1;
};

std::vector<PlannedCorridor> planCorridorRoutes(
    const std::vector<CorridorRouteInput>& routes,
    const std::vector<CorridorStreetAnchor>& anchors,
    const std::function<Real(Real, Real)>& ground);

// §10.6: cut street edges that cross a LOW span of any corridor (under-side
// clearance below the ~5 m highway standard). Mutates the nets' edge lists.
void cutStreetsUnderCorridors(const std::vector<RoadNet*>& nets,
                              const std::vector<CorridorDef>& defs,
                              const std::function<Real(Real, Real)>& ground);

// §10.3/§12: retarget every exit to a real street node (splitting an edge into
// a new T-junction when no junction is reachable — mutates the nets), drop the
// unreachable (station = -1), and run the criss-cross guard. Returns, per
// exit, the (net index, node index) street anchor it landed on ({-1,-1} =
// dropped or no anchor).
std::vector<std::pair<int, int>> resolveCorridorLandings(
    CorridorDef& def, const std::vector<RoadNet*>& nets,
    const std::function<Real(Real, Real)>& ground);

// The editor's recipe-Regenerate: re-run plan -> cut -> land -> author -> bake
// for one freshly regenerated net, using its own freewayPlans / nodes /
// heightAt. Returns the number of corridors baked (0 if the net has no plans
// or no ground sampler).
int rebakeNetCorridors(RoadNet& net, Real interchangeSpacing);

}  // namespace engine

#endif
