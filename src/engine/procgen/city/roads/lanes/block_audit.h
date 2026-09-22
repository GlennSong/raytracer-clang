#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_BLOCK_AUDIT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_BLOCK_AUDIT_H

// The city's block/lot pass, run HEADLESS on the lane lab's road twin, and audited. Calls the engine's
// own growLotBuildingsOnNets (pure: no ECS, no renderer) on exactly the RoadEntity the loader hands it,
// so the blocks and lots here are the ones the J overlay draws in-engine. Every block is then measured
// for what a person sees as "broken": thin protrusions (the area an 8 m morphological opening removes),
// built pavement under the block, and vertex bloat. Drawn over the lab's paved surface as one SVG so
// the whole thing is inspectable without starting the 3D engine.

#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/city_lots.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

struct BlockReport {
    Poly2 foot;              // the block interior the parceller used
    double area = 0, thinArea = 0, pavedArea = 0;   // thin = removed by an opening of 4 m radius (8 m thinner than)
    double convexity = 1;    // area / convex hull area
    double spikeArea = 0;    // thin pieces LONGER than 25 m: a dead-end finger, not the tip of a tapering wedge
    size_t vertices = 0;
    Vec2 centroid;
    std::string roads;       // the lab roads whose pavement lies under the foot
    bool ok() const { return pavedArea < 1.0 && spikeArea < std::max(20.0, 0.15 * area); }   // a hole cannot grow a spur; this catches slivers and anything on the road
};

struct DroppedHole { double area; size_t vertices; double shortestEdge; Vec2 centroid; std::string why; };

// A building pad (or block terrace) whose graded plane would stand above the pavement or sidewalk it
// reaches: the terrain flatten outranks the road, so the ground rises through the deck there.
struct PadConflict { int pad = -1; std::string what; double area = 0, rise = 0; Vec2 at; };

// The feather a lanelab level allows a pad: to the kerb, never into the road.
double lanesPadFalloff(double sidewalk);
// The sidewalk's top above the ground a lot samples beside it (ADR-0086): the
// conform skirt drops the ground `skirtDrop` under the road envelope and the
// slab stands `kSidewalkLift` over the deck. The default rule — an authored
// skirtDrop does not yet travel through the city bundle (open).
double lanesSidewalkRise();
// Building pads clipped to the block that holds each lot, feathered no further than the sidewalk. The loader
// grades a lanelab level with exactly these; the audit checks exactly these.
std::vector<TerrainFlatten> clipPadsToBlocks(const std::vector<LotBuilding>& lots, const std::vector<Poly2>& blocks, double sidewalk);
// Block planes and terraces the lot pass derived, shrunk by the feather and clipped to their block the same way.
std::vector<TerrainFlatten> lanesTerraces(const std::vector<TerrainFlatten>& grades, const std::vector<Poly2>& blocks, double sidewalk);
// Pads and terraces against the built scene: samples conform's own grid nodes that lie on pavement or
// sidewalk (nodeOwner set, nodeDist 0) and reports every flatten whose feathered plane stands more than
// `kerb` above the conformed ground there. Empty when the scene has no terrain (flat ground cannot conflict).
std::vector<PadConflict> padsOnPavement(const Result& r, const std::vector<TerrainFlatten>& pads, double kerb = 0.25);

struct BlockAudit {
    std::vector<PadConflict> padConflicts;      // with the pads the loader now builds (clipped, sidewalk feather)
    std::vector<PadConflict> padConflictsRaw;   // with the pads as built before (plan + apron, 5 m feather): the bug's size
    double padArea = 0, padAreaRaw = 0;         // m² of pavement/sidewalk those would lift
    NetLotResult grown;      // the pass's own output: plan.blocks, plan.lots, lots (built plans)
    std::vector<BlockReport> blocks;
    size_t holesGiven = 0;   // pavement holes handed to the pass
    std::vector<DroppedHole> dropped;   // holes the pass returned no block for, with the reason
    int broken = 0;          // blocks that fail ok()
    double thinTotal = 0, pavedTotal = 0, spikeTotal = 0;
};

// The city blocks straight from the SCENE: the holes of the paved surface (lanes + shoulders unioned),
// simplified at `simplify` metres. No graph, no face walk — a block is what the pavement encloses.
// `insetBy`: the sidewalk margin, applied here with Clipper (robust) — pass the result to the lot pass with
// roadMargin = 0, because its own miter inset rejects large holes over a single short kerb edge.
std::vector<Poly2> sceneBlocks(const Result& r, double simplify = 1.5, double minArea = 2000.0, double insetBy = 0.0);
// The two halves of sceneBlocks, for the bundle (ADR-0084): the un-inset holes of the paved surface are a
// property of the BUILD; the inset is the LEVEL's sidewalk, applied at load.
std::vector<Ring> pavementHoles(const Result& r, double minArea);
// `minWidth` (0 = the old 2 * (insetBy + 3)): a hole piece narrower than this is a verge or a
// median, not a block. The holes already stop at the back of the drawn sidewalk, so a lane-built
// level insets by kBlockMarginBehindSidewalk and passes the width a block must have.
std::vector<Poly2> blocksFromHoles(const std::vector<Ring>& holes, double simplify, double insetBy, double minWidth = 0.0);
constexpr double kBlockMarginBehindSidewalk = 0.3;   // the block begins just behind the drawn sidewalk
constexpr double kMinBlockWidth = 16.0;              // the old effective rule at a 5 m inset

// `citysim`: the level's citysim block (sidewalk, buildChance, parcel knobs, seed); empty -> defaults.
// `fromScene`: hand the pass sceneBlocks() (the loader's lanelab path) instead of letting it extract
// faces from the twin's graph; the twin then serves only as the road-clearance graph.
BlockAudit auditBlocks(const Result& r, const nlohmann::json& citysim, bool fromScene = true);
// Pavement (grey), blocks (tinted, red where broken), lots (thin outlines), built plans (dark).
void writeBlocksSvg(const Result& r, const BlockAudit& a, const std::string& path);
std::string summary(const BlockAudit& a);

}  // namespace roads::lanes
}  // namespace engine

#endif
