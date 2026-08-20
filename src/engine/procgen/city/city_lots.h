#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CITY_LOTS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CITY_LOTS_H

#include "architect.h"      // ArchetypeBook (the Lua-authored selection layer)
#include "polygon.h"        // Poly2, Vec2
#include "shape_grammar.h"  // BuildingParams (the style-book hook's target)
#include "../terrain.h"      // TerrainFlatten (block grades computed in-pass)
#include "../../../rt_math.h"   // Vec3
#include "../../../renderer/renderer.h"   // RenderMesh (the grown building geometry)
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace engine {

// Grow BUILDINGS on the blocks a road network encloses (Living City, ADR-0066).
// This is the "real roads → city blocks → lots → buildings" pass: take the block
// faces (extractBlocks), parcel each block's interior into lots (subdivideBlock),
// and place one typed box building — set back from its lot lines — in each viable
// lot. Each building carries a place-type TAG (a string, so this stays engine-core
// with no citysim dependency) that the caller turns into a Place agents route to.
//
// Pure geometry — no ECS, no rendering — so it is unit-tested headless and stays
// deterministic (ADR-0002). The host (level_loader) spawns each LotBuilding as a
// box entity + collider and registers it as a place for the agents' schedules.

struct LotBuilding {
    Vec2 site;              // footprint centroid (world XZ)
    Real width = 0;         // OBB extent along the lot's long axis (m) — collider
    Real depth = 0;         // OBB extent along the short axis (m) — collider
    Real height = 0;        // building height (m); a park is a low green pad
    Real yaw = 0;           // OBB rotation about +Y (rad) — collider orientation
    std::string type;       // "home" | "shop" | "office" | "civic" | "park"
                            // | "green" (an UNBUILT lot — the caller plants
                            //   grass + trees on it; not a routable place)
    // Which BLOCK this came from, and the district that block resolved to.
    // Both are set on real buildings (PASS C); parks/greens/courts leave block
    // at -1. They exist so the one-block-one-district invariant is TESTABLE:
    // zoning boundaries are supposed to land on streets, and a block face is
    // bounded by streets, so every building in a block must share a district.
    // Without the block id that invariant can only be checked by
    // point-in-polygon against LotPlanDebug, which no test should have to do.
    int block = -1;
    std::string district;   // districtName(tag) — "financial", "oldtown", ...
    std::string recipe;     // the architect RECIPE that built it ("school",
                            // "glass_tower", "fire_station", ...) — debug/UI
    Real baseY = 0;         // world Y the building grows from (terrain-sampled
                            // when LotParams::ground is set; 0 on flat ground)
    Real groundY = 0;       // the building's graded PAD plane (terrain levels):
                            // the plan-centred grade the terrain is flattened to
                            // under the footprint. baseY sits a small plinth
                            // reveal above it. 0 on flat ground.
    // The building's grown PLAN polygon (world XZ) — the caller extrudes it
    // into a prism collider that matches the massing exactly, where an
    // oriented box would spill onto the sidewalk on L / courtyard / prow
    // plans (the old invisible-walls bug). Empty for parks/greens.
    Poly2 plan;
    Vec3 color{0.72, 0.70, 0.64};
    // Park / "green" lots only (device: "square green lots don't fit the
    // blocks"): the lot's OWN polygon (world XZ) and a low slab mesh built
    // from it, so the pad follows the parcel instead of its bounding box.
    // padMesh vertices are white — the caller tints via material albedo —
    // and world-space with ground at y=0. Empty for real buildings.
    // EXCEPTION: sculpted parks bake lawn/path colours into the vertices and
    // set `color` white, so the bake reads as-is under the tint.
    Poly2 pad;
    RenderMesh padMesh;
    // LANDSCAPING tree spots: deterministic planting positions the hosts grow
    // real trees at (parks: around the paths; yards: behind the house). Each
    // entry is (world x, trunk scale, world z). Empty = host's own scatter.
    std::vector<Vec3> treeSpots;
    // Fenced ring spans (world XZ pairs) sculptPark actually built — the
    // loader turns them into thin wall colliders (drive feedback: "Parks
    // also have no collision detection").
    std::vector<std::pair<Vec2, Vec2>> fenceSegs;
};

struct LotParams {
    Real roadMargin = 11.0;   // inset from the block edge to the buildable interior
                              // (road half-width + sidewalk) — wider = more sidewalk
    Real lotSetback = 1.4;    // building inset from its own lot lines
    Real minShort = 9.0;      // reject a building whose short side is under this (m)
    Real minShortUrban = 7.0; // ...relaxed in DENSE districts (OldTown/Commercial
                              // parcel small — an 8 m rowhouse plan is correct there)
    Real maxAspect = 3.5;     // ...or whose long/short exceeds this (no knife blades)
    Real minLotArea = 90.0;   // skip tiny leftover lots
    Real buildChance = 0.92;  // per-lot occupancy (rest become plazas/gaps)
    Vec2 center{0, 0};        // downtown centre for the radial zoning
    Real innerRadius = 55.0;  // < this: downtown (offices/shops)
    Real midRadius = 135.0;   // < this: mixed; beyond: residential
    // Polycentric zoning (metropolis tier): {position, kind} hubs forwarded to
    // DistrictMap::hubs (kind mirrors DistrictTag order). Empty = radial rings.
    std::vector<std::pair<Vec2, int>> hubs;
    Real hubRadius = 220.0;
    // Hub CLUSTER ids parallel to `hubs` (multi-site metros, 8km-city P3):
    // CityHub::site — 0 = the primary city, 1+ = satellite towns. The landmark
    // planner runs its quotas PER CLUSTER (the city keeps the full civic
    // table; each town is guaranteed its own school + church). Empty = every
    // hub is the city; growLotBuildingsOnNets recovers the ids from the nets'
    // own cityHubs when the caller leaves this unset.
    std::vector<int> hubClusters;
    // Level-authored PARCEL overrides (citysim.parcel — the 8km-metro knob:
    // 150 m+ blocks parcel bigger lots). <= 0 = unset: the parceler keeps its
    // compiled-in district tuning EXACTLY. When set, targetArea / frontWidth /
    // lotDepth RESCALE the per-district grain (each district's value is
    // multiplied by override/stock-default, so financial plates stay the
    // widest and old town the tightest — the whole city just parcels bigger),
    // while minArea / minEdge / courtMinArea replace their base values
    // outright (they are floors, not grain).
    Real parcelTargetArea = -1;     // stock 420 (ParcelParams::targetArea)
    Real parcelMinArea = -1;        // stock: minLotArea above
    Real parcelMinEdge = -1;        // stock: minShort above
    Real parcelFrontWidth = -1;     // stock 16 (ParcelParams::frontWidth)
    Real parcelLotDepth = -1;       // stock 28 (ParcelParams::lotDepth)
    Real parcelCourtMinArea = -1;   // stock 400 (ParcelParams::courtMinArea)
    // Stage-10 ALLEYS (courts-with-alleys round): when a block's parcelled lots
    // sit beyond the frontage reach (rim-block outer rows, deep interiors), cut
    // ONE service alley along the block's long axis so that land gains legal
    // frontage instead of going green. `citysim.alleys` / `citysim.alleyWidth`.
    bool alleys = true;
    // Pavement width (m). 2.8 = 2 x lotSetback: centred on the rows' shared
    // boundary, the lane's edges land exactly where building walls already
    // stop, so cutting it never shrinks an existing row.
    Real alleyWidth = 2.8;
    uint32_t seed = 1;
    // TERRAIN sampler (world y at x,z): buildings grow from their graded pad
    // plane (the ENTRANCE-side grade, so the front door sits level with the
    // sidewalk it faces), park/green pads drape per-vertex, and
    // LotBuilding::groundY/baseY record the result. Unset = flat ground at y 0.
    std::function<Real(Real, Real)> ground;
    // Plinth height (terrain levels): how far the wall base rises above the
    // graded pad, on a concrete foundation course — the knob for "how tall the
    // base is" (device feedback). Ignored on flat ground.
    Real plinth = 0.15;
    // STYLE BOOK hook (the Lua data layer): called with every recipe's NAME
    // so the host can overlay look overrides (cladding, windows, colours)
    // from assets/scripts/style_book.lua. The architect decides WHAT + WHERE
    // (C++, deterministic, tested); the style book owns HOW IT LOOKS (data,
    // hot-reloadable). Empty = the built-in looks. Row units get
    // "rowhouse_unit". Overrides must stay deterministic (pure data).
    std::function<void(const std::string& recipe, BuildingParams&)> styleHook;
    // ARCHETYPE BOOK (architect.h): the Lua-authored SELECTION layer — per-
    // district recipe weights, resolved to registry indices at load
    // (makeArchetypeBook). Empty = the architect's compiled ladders. The
    // style book restyles what stands; this book decides WHAT stands.
    ArchetypeBook archetypeBook;
};

// The intermediate planning geometry, exposed for debug visualization: the block
// interiors the parcel pass actually subdivided (post road-margin inset) and every
// lot it produced (built or not) — so "blocks → lots → buildings" can be SEEN.
struct LotPlanDebug {
    std::vector<Poly2> blocks;   // buildable block interiors (inset from the roads)
    std::vector<Poly2> lots;     // every parcelled lot
    // Stage-10 alleys cut this build: centreline segments (plan view + tests).
    std::vector<std::pair<Vec2, Vec2>> alleys;
    // Why lots did NOT build (each one became a green) — the density tuning
    // dials. A "small town" city is usually one of these counters running hot.
    int rejChance = 0;   // lost the occupancy roll (plaza / gap)
    int rejSliver = 0;   // site's OBB short side under minShort
    int rejAspect = 0;   // long/short over maxAspect (knife blade)
    int rejFill = 0;     // polygon fills too little of its OBB
    int rejPlan = 0;     // finished plan too pinched (inradius gauge) — was
                         // double-counted into rejFill
    int rejClear = 0;    // no inset of the plan cleared the road corridors
    int rejBox = 0;      // box fallback rejected (fill / shrink-fit too small)
    int rejFrontage = 0; // whole footprint too far from any road surface
};

// One building per viable lot across every block. Deterministic in seed.
// `debug`, when non-null, receives the intermediate blocks + lots.
//
// `outParts`, when non-null, receives the buildings' GEOMETRY merged by material
// class — one world-space RenderMesh per shape-grammar PartId (Wall / Glass /
// Brick / Concrete / Stucco / …), exactly like CityModel::parts — so the caller
// binds the SAME PBR recipes (materialFor + baked surface maps) the shape:"city"
// pipeline uses, and the whole district draws as a handful of textured meshes.
// Each entry's materialIndex is set to its PartId.
//
// `outFlatParts`, when non-null, additionally receives the SAME buildings grown
// at FacadeDetail::Flat — the middle LOD (city-render-perf R2): identical bay
// and opening layout, a fraction of the triangles. Same PartId merge scheme as
// `outParts`, so the loader binds the same materials and chunks it the same
// way, drawn only between the detail and mass-box distances.
//
// `roads` + `roadClearance` (device: "buildings overlapping sidewalks and poking
// out onto the street"): when given, every building box is additionally kept at
// least `edge width/2 + roadClearance` from every road centreline — pass the
// sidewalk width (+ margin) so towers on wide arterials stay behind the curb.
struct RoadGraph;   // road_network.h (kept light — see edgeBlocks below)
std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& params,
                                          LotPlanDebug* debug = nullptr,
                                          std::vector<RenderMesh>* outParts = nullptr,
                                          const RoadGraph* roads = nullptr,
                                          Real roadClearance = 0.0,
                                          std::vector<RenderMesh>* outFlatParts = nullptr,
                                          std::vector<TerrainFlatten>* outGrade = nullptr);

// EDGE blocks (ADR-0066, device feedback): only fully ENCLOSED faces become city
// blocks, which leaves the town rim bare. Synthesize rectangular blocks on the
// OPEN side of boundary roads: walk each road chain between junctions, subdivide
// it into [minLen, maxLen] pieces, and where a side faces open ground (inside no
// closed block, clear of other roads) emit a `depth`-wide rectangle set back by
// `margin`. The rectangles feed growLotBuildings like any other block. Pure
// geometry, deterministic.
struct EdgeBlockParams {
    Real margin = 9.0;     // setback from the road centreline to the block edge
    Real depth = 34.0;     // block width, outward from the road (m)
    Real minLen = 26.0;    // shortest block along the road (m)
    Real maxLen = 58.0;    // longest block along the road (m)
};
struct RoadGraph;   // road_network.h (forward-declared to keep this header light)
std::vector<Poly2> edgeBlocks(const RoadGraph& roads,
                              const std::vector<Poly2>& closedBlocks,
                              const EdgeBlockParams& params);

// The whole "road nets → blocks (+ rim) → lots → buildings" pass in one call,
// shared by the viewer's loader and the offline tracer so both grow the SAME
// city: combines every net into one planar graph, extracts the enclosed blocks,
// synthesizes rim blocks on the open sides, and grows the lot buildings against
// the SAMPLED centrelines (road clearance).
struct RoadEntity;   // road_net.h
struct NetLotResult {
    std::vector<LotBuilding> lots;
    LotPlanDebug plan;               // blocks + lots, for debug overlays
    std::vector<RenderMesh> parts;   // grown geometry merged by PartId
    std::vector<RenderMesh> flatParts;   // the LOD1 twin of `parts` (R2)
    // Block grades/terraces, computed BETWEEN parcelling and building growth
    // so every building's pad plane is sampled off the TERRACED ground it
    // will actually stand on (the old order graded after the buildings were
    // meshed — pads froze pre-terrace heights and hillside houses sat metres
    // under the bands: "buried to the eaves"). The host stamps these into the
    // terrain INSTEAD of re-deriving them.
    std::vector<TerrainFlatten> gradeFlatten;
};
// `freewayROW` (optional): the ACTUAL freeway right-of-way — the corridor's
// dual carriageways (RoadClass::Freeway) and its ramps (RoadClass::Ramp), in
// world XZ with real per-edge widths, gathered from the unified road graph
// AFTER the corridor is routed. When given it supersedes each net's
// freewayPlans mainline proxy, so the city clears (and, under a deck, re-zones)
// around the WHOLE freeway footprint — ramps and gores included, not just the
// mainline centreline.
// `wantFlatParts`: also grow every building's LOD1 twin into `flatParts`
// (city-render-perf R2). Off by default so the offline tracer and diagnostics
// don't pay for a detail tier they never draw.
NetLotResult growLotBuildingsOnNets(const std::vector<RoadEntity>& nets,
                                    const LotParams& params,
                                    const EdgeBlockParams& edgeParams,
                                    Real roadClearance,
                                    const std::function<double(double, double)>& ground = nullptr,
                                    const RoadGraph* freewayROW = nullptr,
                                    bool wantFlatParts = false);

// The HLOD mass box for one lot (metropolis-scale-plan P1.2): the building's
// oriented box, ground to roof, four walls + a roof cap. This is what the DISTANT
// city is actually made of — past `detailDistance` the full facades are dropped
// and a chunk becomes a handful of these — so its normals decide how the whole
// skyline shades. Appends to `out` (one mesh per render cell).
void appendLotMassBox(RenderMesh& out, const LotBuilding& lot,
                      const Vec3& sideColor, const Vec3& roofColor);

}  // namespace engine

#endif
