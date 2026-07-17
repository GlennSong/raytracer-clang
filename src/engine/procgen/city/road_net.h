#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NET_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NET_H

#include "road_mesh.h"          // RoadMeshParams, buildRoadMesh, RenderMesh
#include "road_network.h"       // RoadGraph (constrainedNetGraph return type)
#include "structure_set.h"      // StructureSet, StructureParams (buildRoadWalls)
#include "metro.h"              // CityHub (polycentric zoning handoff)
#include <nlohmann/json.hpp>
#include <array>
#include <functional>
#include <vector>

namespace engine {

// An editor-authored road network (ADR-0049): a small graph of control nodes
// joined by edges, plus the look (width, sidewalk, markings, ...). The editable
// counterpart to the procedural city.road_mesh — promoted to a first-class entity
// so the inspector can WIDEN it and the viewport can DRAG its nodes, regenerating
// the carriageway live through buildRoadMesh (a fast pure function, no SDF grid).
// `heightAt` drapes it on the level terrain; it is set on load, not serialized.
struct RoadNet {
    std::vector<Vec2> nodes;
    std::vector<std::array<int, 2>> edges;     // node-index pairs (0-based)
    // Spline shape (ADR-0049): EVERY edge is a Catmull-Rom spline, sampled as a Hermite cubic through
    // its endpoints' tangents — a straight road is just the collinear case, which the sampler collapses
    // back to a single segment. `tangents` is one through-direction per node (parallel to `nodes`); a
    // zero (or missing) tangent is auto — Catmull-Rom on a through-road, straight into a junction/
    // dead-end. Editing a tangent overrides the auto for that knot, so any road is shapeable by
    // dragging its handles.
    std::vector<Vec2> tangents;
    // Optional per-node ABSOLUTE deck elevation (parallel to `nodes`; NaN or
    // missing = at-grade, drape on terrain). A finite value authors an ELEVATED
    // road: the node rides at that world Y and the ONE welder meshes it as a deck
    // (UnionSpine.yAbs) — the same welder that meshes streets, no separate bridge
    // mesher. Interior spline samples interpolate between two authored endpoints;
    // a chain with any at-grade node drapes (welder needs a homogeneous chain).
    std::vector<double> nodeElev;
    double width = 10.0;                        // default carriageway width (m) — widen control
    // Optional per-edge width override (parallel to `edges`; <= 0 or missing = use the
    // default `width`). Lets a road taper or a slip road run narrower than its trunk.
    std::vector<double> edgeWidths;
    // Optional per-edge grade-separation layer (parallel to `edges`; 0 or missing = ground,
    // ADR-0051). An edge on a higher layer than one it crosses is a bridge: it is lifted onto
    // a deck that clears the lower road (clearanceProfile) instead of forming an intersection.
    std::vector<int> edgeLayers;
    // Optional per-edge road CLASS (parallel to `edges`; missing/short = Local).
    // The generator writes the arterial/collector/local class its recipe grew so
    // the mesher, markings, and nav can vary by class instead of treating every
    // surface edge as a bare Local — the first step of the graph unification
    // (road-unification-plan P1). Hand-authored/edited edges default to Local.
    std::vector<RoadClass> edgeClasses;
    double sidewalk = 3.5;                      // raised sidewalk width per verge (m)
                                                // (device: "sidewalks should be wider")
    double curb = 0.16;                         // curb height (m)
    double cornerRadius = 3.0;                  // rounded kerb-return radius (m)
    double lift = 0.08;                         // raise above the ground (m) — just
                                                // enough to clear z-fighting; the road
                                                // should hug the ground (device feedback)
    bool   markings = true;
    bool   crosswalks = true;
    // Junction policy (ADR-0075 P0): may the constraints pass PROMOTE a busy or
    // over-acute node to a roundabout ring? Generated nets (metro/district)
    // planarize + cap degree instead and set this false; the mesh and terrain-
    // conform passes must honour it, or they silently re-promote roundabouts the
    // generator never intended (buildRoadNetMesh + constrainedNetGraph used to
    // re-run applyConstraints with default rules). Hand-authored nets keep true.
    bool   autoRoundabout = true;
    // Swept-lattice street mesher (street-lattice-plan.md stage 3): mesh streets as
    // quad-strip bodies + Coons junction patches instead of the earcut union weld.
    // Per-level opt-in while it reaches parity; the whole engine flips at 3d and
    // this field (and weldSolid) go away. `RT_LATTICE_STREETS` forces it globally.
    bool   latticeStreets = false;
    Vec3   color{0.09, 0.09, 0.10};
    std::function<double(double, double)> heightAt;   // terrain drape (flat if unset)
    // Hubs the metro recipe grew this net around (with district kinds), so lot
    // growth can zone polycentrically. Empty for hand-authored/district nets.
    std::vector<CityHub> cityHubs;
    // §10.6: freeway CORRIDOR plans the metro recipe routed hub-to-hub —
    // anchor polylines the loader builds as real corridors (alignment,
    // profile, interchanges). Never street edges.
    std::vector<std::vector<Vec2>> freewayPlans;
};

// Build the road surface for `net` (its graph fed to buildRoadMesh with the look).
RenderMesh buildRoadNetMesh(const RoadNet& net);

// Swept-lattice street mesher (street-lattice-plan.md, stage 3): sweep each chain
// as a lattice body trimmed to the junction boundary, and fill each deg>=3 node
// with a Coons junction patch that shares the bodies' mouth rings — so the
// surface is quads with interior vertices (conforms to terrain) and junctions
// interpolate height (no medial-axis step). Not yet the buildRoadNetMesh default;
// proven on its own first. `heightAt` drapes the streets (null = flat).
RenderMesh buildRoadNetLattice(const RoadGraph& g,
                               const std::function<Real(Real, Real)>& heightAt);

// The sampled + constrained road graph the mesher builds from: every edge sampled
// to a fine polyline (a curved road becomes a chain of short straight edges; a
// straight run collapses back to one), with the local roundabout constraints
// applied — exactly the geometry the carriageway is meshed over. Exposed so
// runtime consumers (the navigation graph, ADR-0059) route on the SAME centrelines
// the asphalt is drawn on. (Wraps the file-local builder; defined in road_net.cpp.)
RoadGraph navRoadGraph(const RoadNet& net);

// Diagnostic accessors: the mesher's exact constrained graph + weld-chain
// decomposition, so instruments (RT_POKE_REPORT) measure the deck the mesh
// actually rides — not a near-miss reconstruction.
RoadGraph roadNetConstrainedGraph(const RoadNet& net);
std::vector<UnionSpine> roadNetWeldSpines(const RoadGraph& g);

// The terrain cut/fill footprints that grade the ground to this road (ADR-0044 corridor
// conforming). Traces the net's chains, gives each a smoothed, grade-limited vertical
// profile (roadProfile, over `net.heightAt`), and emits a flatten ramp per segment at the
// profile, half-width = carriageway + `shoulder`, feathered over `falloff`. The loader
// folds these into the level terrain before it builds, so the ground meets the road and no
// terrain pokes through. Empty if `net.heightAt` is unset (a flat road needs no carving).
std::vector<TerrainFlatten> roadNetConformRegions(const RoadNet& net, double shoulder = 1.5,
                                                  double falloff = 8.0, double maxGrade = 0.10);

// Retaining / fill walls for a road net whose `heightAt` is the NATURAL (pre-carve)
// ground (ADR-0075 Phase 1). Reuses the SAME spine + grade-limited profile as
// roadNetConformRegions, so a wall stands exactly where the terrain batter clamps
// at `p.reach` — capping the residual step a steep cut/fill can't daylight, never
// double-counting it. The 3-D grade-break geometry the 2.5-D ground defers to a
// StructureSet. Empty on flat ground or when `net.heightAt` is unset.
StructureSet buildRoadWalls(const RoadNet& net, const StructureParams& p = {});

// --- editor edit ops (each leaves the net ready for buildRoadNetMesh) ----------
// Set the default carriageway width (the inspector "Width" control — "widen a road").
void roadNetSetWidth(RoadNet& net, double width);
// Width of edge `ei` (its per-edge override if set, else the default `width`).
double roadNetEdgeWidth(const RoadNet& net, int ei);
// Override edge `ei`'s width (the viewport per-edge widen). w <= 0 reverts to default.
bool roadNetSetEdgeWidth(RoadNet& net, int ei, double w);
// Move control node `i` to `pos` (the viewport node drag). False if out of range.
bool roadNetMoveNode(RoadNet& net, int i, const Vec2& pos);
// Set node `i`'s tangent (the viewport tangent-handle drag). A zero tangent reverts
// the knot to auto (Catmull-Rom). False if out of range.
bool roadNetSetTangent(RoadNet& net, int i, const Vec2& tangent);
// Node `i`'s effective tangent (the stored override, or the auto Catmull-Rom/chord
// the curve actually uses) — what the viewport seeds the tangent handle from.
Vec2 roadNetTangentAt(const RoadNet& net, int i);

// --- topology edits: the viewport's add / split / delete (ADR-0049) -----------
// Append a control node at `pos`; returns its index.
int  roadNetAddNode(RoadNet& net, const Vec2& pos);
// Connect nodes a and b. Ignored (returns false) if invalid, equal, or already joined.
bool roadNetAddEdge(RoadNet& net, int a, int b);
// Append a node at `pos` joined to `from` — grow a road from an end. -1 if `from` bad.
int  roadNetExtend(RoadNet& net, int from, const Vec2& pos);
// Insert a node at `pos` into edge `edgeIndex`, splitting it in two; returns the new
// node index (the "click a road to add a point" op). -1 if the edge index is bad.
int  roadNetSplitEdge(RoadNet& net, int edgeIndex, const Vec2& pos);
// Delete node `i` and its incident edges, reindexing the rest. False if out of range.
bool roadNetDeleteNode(RoadNet& net, int i);
// The edge nearest `p` within `maxDist` (chord distance), or -1 — for viewport edge
// picking (which road segment did the user click to split?).
int  roadNetNearestEdge(const RoadNet& net, const Vec2& p, double maxDist);

// --- level I/O: the `road` block of a shape:"road" entity ----------------------
RoadNet roadNetFromJson(const nlohmann::json& j);
nlohmann::json roadNetToJson(const RoadNet& net);

// Build a generated road's graph from a "generate" recipe block (district kind): runs buildDistrict,
// caps junction degree to <=4, planarizes, and fills net.nodes/edges/edgeWidths — leaving the look
// params and heightAt untouched. Shared by the loader (initial build) and the editor (regenerate
// from the tuning panel). No-op if `generate` isn't a recipe object. (road-network-v2-plan T2.1)
void applyGenerateRecipe(RoadNet& net, const nlohmann::json& generate);

// The JSON to SAVE for an (edited) road, given its current recipe and live net: a GENERATED road
// keeps its "generate" block with only the look refreshed from `net` (never baking the nodes — that
// lost the recipe, the grown.json "save changed" bug); a hand-authored road serialises the net in
// full. The editor's regenerate writes this back to SourceSpec.recipe. (road-network-v2-plan T2.1)
nlohmann::json roadRecipeForSave(const std::string& currentRecipe, const RoadNet& net);

}  // namespace engine

#endif
