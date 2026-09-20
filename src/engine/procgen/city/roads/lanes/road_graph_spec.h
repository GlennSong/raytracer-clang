#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_GRAPH_SPEC_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_GRAPH_SPEC_H

// lanelab's input: a road graph as data (ADR-0083). Edges are classes + paths; a class's
// lane layout is expressed as engine RoadSpec bands (Travel per direction, Median gap,
// Shoulder, Sidewalk) so the lane model is the engine's. Ramps carry anchors on a host
// edge's outer lane; roads may carry pockets (lanes born out of a neighbour); connectors
// join a lane's end to a point on another lane. Everything downstream reads the resolved
// spines here, never the JSON.
//
// Plan coordinates: the scene files use (x, y); the engine world is (x, height, z). Vec2.x
// is world x and Vec2.y is world z throughout lanelab.

#include "engine/procgen/city/polygon.h"      // Vec2
#include "engine/procgen/city/road_spec.h"    // RoadSpec / RoadBand
#include <nlohmann/json.hpp>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

struct LaneLayout {
    double w = 3.5;          // lane width
    int fwd = 1, back = 0;   // lanes per direction; forward lanes lie on the RIGHT of the spine direction
    double gap = 0.0;        // median between directions (two-way roads)
    double dovetail = 60.0;  // ramps with fwd >= 2: station where the second lane starts dying into the first
};

struct PocketSpec {
    std::string id, kind = "turn", side = "right";
    int dir = +1;            // +1 forward lanes, -1 backward lanes
    double s0 = 0, s1 = 0, taper = 0, taperOut = 0;   // taper 0 -> 10 lane widths
};

struct RampAnchor {
    bool set = false;        // an anchor with geometry (station on the host); a bare edge id only supplies height
    std::string edge;        // host edge id
    bool hasStation = false; double s = 0;
    char axis = 0; double coord = 0;   // 'x' or 'y': nearest host vertex to that coordinate; 'p': nearest to (px, py)
    double px = 0, py = 0;
    std::string side = "right";
    double aux = 120, decel = 100, taper = -1, approach = 150;   // taper < 0 -> 30 lane widths
};

struct ConnectorSpec { std::string from, to; double toS = 0; };

// What a class builds at a deck edge, by what that edge faces (EdgeRole in deck_mesh.h). Authored per class
// in the graph JSON: "edges": { "median": {"kind": "wall", "h": 1.05}, "at_grade": {"kind": "guardrail"} }.
// kind: "none" | "wall" (solid slab, the parapet) | "guardrail" (beam on posts). Only freeway and ramp
// classes are consulted; a street's edges belong to the lot pass (Glenn, 2026-09-08).
enum class BarrierKind { None = 0, Wall, Guardrail };
// `set` distinguishes "the graph authored none here" from "the graph said nothing": without it a class
// could never switch a default OFF.
// `offset` is how far OUTBOARD of the road's own outline the barrier stands, `thick` its body. Offset 0 hugs
// the pavement edge, which is the only place the lab can guarantee ground under it; push it out only where
// the class's shoulder is really paved.
struct BarrierSpec { BarrierKind kind = BarrierKind::None; double h = 0.9, offset = 0.0, thick = 0.4; bool set = false; };

struct RoadClassSpec {
    std::string name;
    LaneLayout lanes;
    double shoulder = 0, sidewalk = 0, median = 0;   // rings outside the lanes (median: divider axes)
    int rank = 1;
    double gMax = 0.09, window = 40, thick = 0.5;
    // indexed by EdgeRole: seam, median, vs street, elevated, at grade
    std::array<BarrierSpec, 5> edges{};
};

struct Rules {
    double step = 1.0, sameLevelDz = 1.0, blendLen = 20.0, closing = 0.0, bridgeH = 4.0, pierSpacing = 24.0;
    double slope = 0.5, conformW = 14.0, skirt = 1.6, skirtDrop = 0.15, endpointTol = 0.5;
    double underClearance = 5.0, structureDepth = 1.6;   // free height under a deck's structure over any road it crosses, and the girder depth below the slab
    double rampLevelDz = 1.5;                  // a ramp within this of a street it crosses is at grade with it (the street meets it); above, it is structure (the ramp clears it)
};

struct TerrainSpec {
    std::string type = "flat", file;
    std::array<double, 4> bounds{0, 0, 0, 0};   // x0, x1, y0, y1
    bool hasBounds = false;
    double res = 2.0; int seed = 1;
    std::vector<std::pair<double, double>> octaves;             // (amplitude, cell)
    struct Valley { bool alongY = false; double c = 0, width = 1, depth = 0; };
    std::vector<Valley> valleys;
    struct Hill { double x = 0, y = 0, r = 1, h = 0; };
    std::vector<Hill> hills;
    bool hasTilt = false; double dzdx = 0, x0 = 0, dzdy = 0, y0 = 0;
};

struct RampInfo {
    bool valid = false;
    double departS = 0, touchS = 0, length = 0, climb = 0, freeLen = 0, requiredLen = 0;
    bool ok = true;
};

struct EdgeSpec {
    std::string id, cls;
    nlohmann::json path;                 // kept for ramp recomposition
    RampAnchor from, to;
    bool hasZMin = false; double zMin = 0;
    double lotsFrom = -1, lotsTo = -1;   // station range where the city may build lots along this edge (-1: all of it); the rest is right-of-way
    std::vector<std::array<double, 4>> floorPts;   // (x, y, z, half-span): the profile must clear z within half-span of that station, approached at design grade beyond
    LaneLayout lanes;
    std::vector<PocketSpec> pockets;
    // resolved geometry and profile
    std::vector<Vec2> xy;                // spine, resampled at rules.step
    std::vector<double> s, z, t;         // station, deck height, terrain height along the spine
    RampInfo ramp;
    int anchorIdx[2] = {-1, -1};         // spine index of the from-gore and the to-gore, when anchored
    std::map<std::string, std::string> anchorLanes;   // "from"/"to" -> host lane id
    bool isRamp() const { return !from.edge.empty() || !to.edge.empty(); }
    int laneCount() const { return lanes.fwd + lanes.back; }
    double length() const { return s.empty() ? 0.0 : s.back(); }
};

struct RoadLabGraph {
    std::string name = "roads";
    std::map<std::string, RoadClassSpec> classes;
    Rules rules;
    TerrainSpec terrain;
    std::vector<EdgeSpec> edges;
    std::vector<ConnectorSpec> connectors;
    std::map<std::string, int> index;    // edge id -> position in edges

    const EdgeSpec* find(const std::string& id) const;
    EdgeSpec* find(const std::string& id);
    const RoadClassSpec& cls(const EdgeSpec& e) const { return classes.at(e.cls); }
    int rank(const std::string& edgeId) const { return cls(*find(edgeId)).rank; }
    double hw(const EdgeSpec& e) const;      // half-width of the through lanes
    std::array<double, 4> bounds() const;    // terrain bounds, or the edges' extent plus a margin
    std::vector<const EdgeSpec*> paved() const;

    static RoadLabGraph fromJson(const nlohmann::json& spec, const std::string& baseDir = ".");
    static RoadLabGraph load(const std::string& path);
};

// The engine band list for a class: [Sidewalk][Shoulder][Travel dir -1 x back][Median gap][Travel dir +1 x fwd][Shoulder][Sidewalk],
// one-way roads centred. Lane offsets are derived from RoadSpec::bandSpan, so the lane model is the engine's.
RoadSpec bandsFor(const RoadClassSpec& c, const LaneLayout& layout);

// (name, left-normal offset from the spine, direction) of each Travel band, from the RoadSpec.
struct LaneSlot { std::string name; double offset; int dir; };
std::vector<LaneSlot> laneSlots(const RoadClassSpec& c, const LaneLayout& layout);

// --- path helpers shared with lane expansion (ramps are recomposed from their anchors) ---
std::vector<Vec2> bezier(const std::array<Vec2, 4>& p, int n = 200);
std::vector<Vec2> resample(const std::vector<Vec2>& pts, double step);
std::vector<double> stations(const std::vector<Vec2>& xy);
// A point spec: [x, y] or {"edge", s|t|x|y, along, normal, dx, dy}; edges must already be resolved.
Vec2 resolvePoint(const nlohmann::json& spec, const RoadLabGraph& g);
std::vector<Vec2> resolvePath(const nlohmann::json& path, const RoadLabGraph& g, double step);

}  // namespace roads::lanes
}  // namespace engine

#endif
