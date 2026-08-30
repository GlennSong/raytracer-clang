#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NET_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NET_H

#include "road_mesh.h"          // UnionSpine, strokeRibbon, roadProfile, RenderMesh
#include "road_spec.h"          // RoadSpec band model (roads-v2 Part 1)
#include "road_network.h"       // RoadGraph — the ONE road graph
#include "structure_set.h"      // StructureSet, StructureParams (buildRoadWalls)
#include "metro.h"              // CityHub (polycentric zoning handoff)
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
// parameter, exactly like buildRoadNetLattice always did.

// ---- presentation: how the road is drawn, not what it is -------------------
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
    // Hubs the metro recipe grew this net around (with district kinds), so lot
    // growth can zone polycentrically. Empty for hand-authored/district roads.
    std::vector<CityHub> cityHubs;
    // §10.6: freeway CORRIDOR plans the metro recipe routed hub-to-hub —
    // anchor polylines the loader builds as real corridors (alignment,
    // profile, interchanges). Never street edges.
    std::vector<std::vector<Vec2>> freewayPlans;
    // P8 footprint-first: the per-site footprint polygons (+ rim gates) the
    // skeleton was derived from. Empty unless the recipe sets
    // "skeleton": "footprint". The planner's Footprint overlay draws these;
    // the P8-G hand-edit surface writes back through them.
    std::vector<Footprint> siteFootprints;
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

// Build the road surface for `road` (its graph swept by buildRoadNetLattice).
// `heightAt` drapes it on the level terrain (null = flat).
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

// `deckOut` (optional) receives the DECK the mesh rode — see RoadDeckField —
// indexed and ready to query; the loader stores it beside the road entity.
RenderMesh buildRoadNetMesh(const RoadEntity& road, const RoadGroundFn& heightAt,
                            CurbBandAudit* auditOut = nullptr,
                            RoadDeckField* deckOut = nullptr);

// Swept-lattice street mesher (street-lattice-plan.md, stage 3): sweep each chain
// as a lattice body trimmed to the junction boundary, and fill each deg>=3 node
// with a Coons junction patch that shares the bodies' mouth rings — so the
// surface is quads with interior vertices (conforms to terrain) and junctions
// interpolate height (no medial-axis step). `heightAt` drapes the streets
// (null = flat). `chainTriEndsOut` (optional, diagnostics): index-buffer
// position after each swept chain BODY, in order; back() is where bodies end
// and junction PADS begin. The surface-scan tests use it to classify which
// surfaces stack (same-chain self-fold / cross-chain / body-pad / pad-pad).
RenderMesh buildRoadNetLattice(const RoadGraph& g,
                               const std::function<Real(Real, Real)>& heightAt,
                               std::vector<std::size_t>* chainTriEndsOut = nullptr,
                               double sidewalkWidth = 3.0, double curbHeight = 0.15,
                               bool crosswalks = true, CurbBandAudit* auditOut = nullptr,
                               // 0 = the old radius-less quadratic corner. The real
                               // value arrives from RoadLook::cornerRadius via
                               // buildRoadNetMesh; hand-built graphs keep the old shape.
                               double cornerRadius = 0.0,
                               // The deck: final chains (yAbs) + pad triangles.
                               RoadDeckField* deckOut = nullptr,
                               // Grade limit per class (RoadLook::perClassGrade) or
                               // the single kRoadMaxGrade. Must match what the
                               // terrain carve is told, or deck and ground disagree.
                               bool perClassGrade = true);

// The sampled + constrained road graph the mesher builds from: every edge sampled
// to a fine polyline (a curved road becomes a chain of short straight edges; a
// straight run collapses back to one), with the local roundabout constraints
// applied — exactly the geometry the carriageway is meshed over. Runtime
// consumers (the navigation graph, ADR-0059) route on the SAME centrelines the
// asphalt is drawn on. This is real work (spline sampling + constraints +
// semantic classification), not a representation change — the entity's control
// graph is already a RoadGraph.
RoadGraph navRoadGraph(const RoadEntity& road, const RoadGroundFn& heightAt);

// The full sampled+constrained graph, baked corridor edges INCLUDED — what the
// unified mesher (roads-v2.1 R1) and the bake-fidelity tests build from.
// (Since roads-v2.1 2e navRoadGraph keeps baked edges too; the two names are
// kept for their distinct call-site intents.)
RoadGraph roadNetFullGraph(const RoadEntity& road, const RoadGroundFn& heightAt);

// Diagnostic accessors: the mesher's exact constrained graph + weld-chain
// decomposition, so instruments (RT_POKE_REPORT) measure the deck the mesh
// actually rides — not a near-miss reconstruction.
RoadGraph roadNetConstrainedGraph(const RoadEntity& road, const RoadGroundFn& heightAt);
std::vector<UnionSpine> roadNetWeldSpines(const RoadGraph& g);

// The terrain cut/fill footprints that grade the ground to this road (ADR-0044 corridor
// conforming). Traces the graph's chains, gives each a smoothed, grade-limited vertical
// profile (roadProfile, over `heightAt`), and emits a flatten ramp per segment at the
// profile, half-width = carriageway + `shoulder`, feathered over `falloff`. The loader
// folds these into the level terrain before it builds, so the ground meets the road and no
// terrain pokes through. Empty if `heightAt` is null (a flat road needs no carving).
std::vector<TerrainFlatten> roadNetConformRegions(const RoadEntity& road,
                                                  const RoadGroundFn& heightAt,
                                                  double shoulder = 1.5,
                                                  double falloff = 8.0, double maxGrade = 0.10);

// Retaining / fill walls for a road whose `heightAt` is the NATURAL (pre-carve)
// ground (ADR-0075 Phase 1). Reuses the SAME spine + grade-limited profile as
// roadNetConformRegions, so a wall stands exactly where the terrain batter clamps
// at `p.reach` — capping the residual step a steep cut/fill can't daylight, never
// double-counting it. The 3-D grade-break geometry the 2.5-D ground defers to a
// StructureSet. Empty on flat ground or when `heightAt` is null.
StructureSet buildRoadWalls(const RoadEntity& road, const RoadGroundFn& heightAt,
                            const StructureParams& p = {});

// --- editor edit ops (each leaves the road ready for buildRoadNetMesh) ---------
// The SPEC of edge `ei`: its entry in the graph's spec table when assigned, else
// a legacy synthesis from the edge's (always-resolved) width + the look's
// sidewalk/curb — so every edge answers the band model even before its level is
// migrated (roads-v2 compat shim).
RoadSpec roadNetEdgeSpec(const RoadEntity& road, int ei);
// Roads-v2 S3b: the road with baked corridor edges (Freeway/Ramp) removed —
// what the street mesher/conform/nav consume while the corridor still draws
// itself. Nodes are preserved so indices stay stable.
RoadEntity roadNetStreetsOnly(const RoadEntity& road);
// Override edge `ei`'s width (the viewport per-edge widen). w <= 0 reverts to
// the look's default width. False if out of range.
bool roadNetSetEdgeWidth(RoadEntity& road, int ei, double w);
// Move control node `i` to `pos` (the viewport node drag). False if out of range.
bool roadNetMoveNode(RoadEntity& road, int i, const Vec2& pos);
// Set node `i`'s tangent (the viewport tangent-handle drag). A zero tangent reverts
// the knot to auto (Catmull-Rom). False if out of range.
bool roadNetSetTangent(RoadEntity& road, int i, const Vec2& tangent);
// Node `i`'s effective tangent (the stored override, or the auto Catmull-Rom/chord
// the curve actually uses) — what the viewport seeds the tangent handle from.
Vec2 roadNetTangentAt(const RoadEntity& road, int i);

// --- topology edits: the viewport's add / split / delete (ADR-0049) -----------
// Append a control node at `pos`; returns its index.
int  roadNetAddNode(RoadEntity& road, const Vec2& pos);
// Connect nodes a and b. Ignored (returns false) if invalid, equal, or already joined.
bool roadNetAddEdge(RoadEntity& road, int a, int b);
// Append a node at `pos` joined to `from` — grow a road from an end. -1 if `from` bad.
int  roadNetExtend(RoadEntity& road, int from, const Vec2& pos);
// Insert a node at `pos` into edge `edgeIndex`, splitting it in two; returns the new
// node index (the "click a road to add a point" op). -1 if the edge index is bad.
int  roadNetSplitEdge(RoadEntity& road, int edgeIndex, const Vec2& pos);
// Delete node `i` and its incident edges, reindexing the rest. False if out of range.
bool roadNetDeleteNode(RoadEntity& road, int i);
// The edge nearest `p` within `maxDist` (chord distance), or -1 — for viewport edge
// picking (which road segment did the user click to split?).
int  roadNetNearestEdge(const RoadEntity& road, const Vec2& p, double maxDist);

// --- level I/O: the `road` block of a shape:"road" entity ----------------------
// The wire format is UNCHANGED (parallel arrays, sparse overrides): these two
// functions are the ONLY code that still knows that shape — they transpose on
// the way in (resolving every fallback: a missing/zero edge width becomes the
// default width, a short class array becomes Local, a null node_elev becomes
// at-grade) and reconstruct the sparse form on the way out.
RoadEntity roadNetFromJson(const nlohmann::json& j);
nlohmann::json roadNetToJson(const RoadEntity& road);

// Build a generated road's graph from a "generate" recipe block (district kind): runs
// buildDistrict, caps junction degree to <=4, planarizes, and assigns the result to
// road.graph — leaving the look untouched. `heightAt` gates terrain-aware metro layout
// (buildability); pass what the loader knows (null = not terrain-aware). Shared by the
// loader (initial build) and the editor (regenerate from the tuning panel). No-op if
// `generate` isn't a recipe object. (road-network-v2-plan T2.1)
void applyGenerateRecipe(RoadEntity& road, const nlohmann::json& generate,
                         const RoadGroundFn& heightAt);

// The JSON to SAVE for an (edited) road, given its current recipe and live graph: a
// GENERATED road keeps its "generate" block with only the look refreshed (never baking the
// nodes — that lost the recipe, the grown.json "save changed" bug); a hand-authored road
// serialises the graph in full. The editor's regenerate writes this back to
// SourceSpec.recipe. (road-network-v2-plan T2.1)
nlohmann::json roadRecipeForSave(const std::string& currentRecipe, const RoadEntity& road);

}  // namespace engine

#endif
