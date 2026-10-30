#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_ENTITY_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_ENTITY_H

#include "../road_mesh.h"          // UnionSpine, strokeRibbon, roadProfile, RenderMesh
#include "../road_spec.h"          // RoadSpec band model (roads-v2 Part 1)
#include "../road_network.h"       // RoadGraph — the ONE road graph
#include <nlohmann/json.hpp>
#include <functional>
#include <string>
#include <vector>

namespace engine {

// The road entity (ADR-0049, unified per docs/road-graph-unification-plan.md):
// ONE topology (`RoadGraph` — the same struct every generator returns and the
// mesher consumes), split from the two things that used to be welded onto it:
// how it is DRAWN (`RoadLook`) and what the generator KNEW (`RoadPlan`).
//
// The old `RoadNet` held topology as eight parallel arrays with fallback
// semantics (a short `edgeWidths` meant "use the default width"), which cost 54
// defensive size() guards across five files. Those fallbacks are now resolved
// ONCE at construction — `RoadEdge::width` is always the real width, `klass`/
// `layer`/`spec`/`baked` always present — and the JSON wire format keeps the
// parallel shape only inside roadNetFromJson/roadNetToJson, so no saved level
// changes. The terrain sampler (`heightAt`) is no longer stored on the entity
// at all: it is one load's ephemeral state, so every reader takes it as a
// parameter.

// ---- presentation: how the road is drawn, not what it is -------------------
// The road CARVE steps the terrain this far BELOW the drivable profile under
// the road footprint (the builder's carve); the curb skirt hides the step.
// The sidewalk slab top is deck + RoadLook::curb, so a lot's paving meets the
// sidewalk at ground + kRoadConformStep + curb — the lot pass's sidewalkRise.
constexpr double kRoadConformStep = 0.22;

struct RoadLook {
    double defaultWidth = 10.0;   // seeds RoadEdge::width where unspecified ("widen" control)
    double sidewalk = 3.5;        // raised sidewalk width per verge (m)
                                  // (device: "sidewalks should be wider")
    double curb = 0.16;           // curb height (m)
    double cornerRadius = 3.0;    // rounded kerb-return radius (m)
    double lift = 0.08;           // raise above the ground (m) — just enough to
                                  // clear z-fighting; the road should hug the
                                  // ground (device feedback)
    bool   markings = true;
    bool   crosswalks = true;
    // Junction policy (ADR-0075 P0): may the constraints pass PROMOTE a busy or
    // over-acute node to a roundabout ring? Generated nets (metro/district)
    // planarize + cap degree instead and set this false; the mesh and terrain-
    // conform passes must honour it. Hand-authored roads keep true.
    bool   autoRoundabout = true;
    // Vertical alignment by CLASS (DesignRules::maxGrade: freeway 5%, arterial
    // 8%, collector 10%, local 12%, ramp 6%) instead of one 8% for every road.
    // The table existed and was ignored — every profile solve was handed the
    // single kRoadMaxGrade — the same way lanesForClass was read by nav and
    // ignored by the mesher. Steeper side streets, less earth moved; freeways
    // tighten. Glenn chose this on 2026-08-29. false pins the single grade.
    bool   perClassGrade = true;
    Vec3   color{0.09, 0.09, 0.10};
};

// ---- provenance: what the generator knew, for downstream passes ------------
struct RoadPlan {
    // Freeway CORRIDOR plans: anchor polylines the loader builds as real corridors
    // (alignment, profile, interchanges), and the lanes twin's freeways for the lot
    // pass's fallback band. Never street edges.
    std::vector<std::vector<Vec2>> freewayPlans;
};

// ---- the editable road entity/component (replaces RoadNet) -----------------
// Notes carried over from the parallel-array era, still true of the graph form:
//  * EVERY edge is a Catmull-Rom spline sampled as a Hermite cubic through its
//    endpoints' tangents (`RoadNode::tangent`; zero = auto). A straight road is
//    the collinear case, which the sampler collapses back to one segment.
//  * `RoadNode::elevAbsolute` authors an ELEVATED road: the node rides at that
//    world Y and the ONE welder meshes it as a deck (UnionSpine.yAbs) — no
//    separate bridge mesher. false = at-grade, drape on terrain.
//  * `RoadNode::kind` is a semantic-layer HINT (roads-v2.2 #17): the corridor
//    bake stamps its gores and landings (it KNOWS which side of the interchange
//    it built); classifyRoadGraph honours a hint at degree >= 3. Not
//    serialized: the bake re-stamps on every load/regen.
//  * `RoadEdge::baked` marks an edge BAKED from a corridor solve (street passes
//    skip it); an AUTHORED freeway-class edge keeps false and meshes normally.
//  * `RoadEdge::spec` indexes `graph.specs` (-1 = legacy: synthesized from the
//    edge width + look. Only the `"metro"` path populates specs; district
//    levels are specless and MUST keep working through the -1 synthesis).
struct RoadEntity {
    RoadGraph graph;
    RoadLook  look;
    RoadPlan  plan;
};

// The terrain sampler roads drape on. One load's state — never stored, never
// serialized; pass what the loader (or the editor's conform pass) knows.
using RoadGroundFn = std::function<double(double, double)>;

// Diagnostic capture of the curb/sidewalk BAND's inputs (roads-v2 S5). The band
// is not a per-junction object — it is swept along the closed boundary loops of
// the whole asphalt union — so a probe cannot reconstruct it from the graph
// without re-implementing the mesher (which would then measure ITSELF, not the
// road). Filled only when a caller asks; costs nothing otherwise. Same contract
// as `chainTriEndsOut`: diagnostics read the mesher's own working state.
struct CurbBandAudit {
    std::vector<Poly2> loops;                          // boundary loops, post snap/de-spike
    std::vector<std::pair<Vec2, Vec2>> mouthGaps;      // where the band is suppressed
    std::vector<Vec2>  junctions;                      // deg >= 3 node positions
    std::vector<int>   junctionDegree;                 // parallel to `junctions`
    // Smallest angle (deg) between ADJACENT arms at each junction, from the
    // mesher's own arm directions — the number that says whether a junction is
    // a clean cross or a needle. -1 where the mesher built no arms for a node.
    std::vector<double> junctionMinAngle;
    double sidewalkWidth = 0.0;
    double curbHeight = 0.0;
};


// The sampled + constrained road graph the mesher builds from: every edge sampled
// to a fine polyline (a curved road becomes a chain of short straight edges; a
// straight run collapses back to one), with the local roundabout constraints
// applied — exactly the geometry the carriageway is meshed over. Runtime
// consumers (the navigation graph, ADR-0059) route on the SAME centrelines the
// asphalt is drawn on. This is real work (spline sampling + constraints +
// semantic classification), not a representation change — the entity's control
// graph is already a RoadGraph.
RoadGraph navRoadGraph(const RoadEntity& road, const RoadGroundFn& heightAt);

// The SPEC of edge `ei`: its entry in the graph's spec table when assigned, else a synthesis from the edge's
// (always-resolved) width + the look's sidewalk/curb -- so every edge answers the band model.
RoadSpec roadNetEdgeSpec(const RoadEntity& road, int ei);

// The road as JSON (nodes, sparse per-edge widths, layers, tangents, node elevations, the look): the lanes
// twin's debug dump into a bundle's GLB extras. (Roads are no longer authored as nodes in a level -- the
// editor's road drawing and the level reader retired with ADR-0267 -- so nothing reads this back.)
nlohmann::json roadNetToJson(const RoadEntity& road);

}  // namespace engine

#endif
