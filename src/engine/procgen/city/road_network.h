#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NETWORK_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NETWORK_H

#include "polygon.h"
#include <cstdint>
#include <vector>

namespace engine {

// The road network (ADR-0038 §3 / city-plan §3.1-§3.2): a planar graph whose
// *faces* are the city blocks. Roads are the lines; blocks are the holes between
// them. Get the graph right and blocks fall out via planar-face extraction.

enum class RoadClass : uint8_t { Arterial, Collector, Local };

struct RoadNode { Vec2 pos; };
struct RoadEdge { int a = 0, b = 0; Real width = 8; RoadClass klass = RoadClass::Local; };

struct RoadGraph {
    std::vector<RoadNode> nodes;
    std::vector<RoadEdge> edges;

    // Add a node, snapping to an existing one within `tol` (keeps the graph clean).
    int addNode(const Vec2& p, Real tol = 0.5);
    void addEdge(int a, int b, Real width = 8, RoadClass klass = RoadClass::Local);
    Real edgeWidth(int edgeIndex) const { return edges[edgeIndex].width; }
};

// Deformed-grid road generator (city-plan §3.1, the bootstrap). A grid of streets
// over a square region centred on `center`, vertices jittered by noise, with
// optional dropout of low-importance local streets. Deterministic for `seed`.
struct GridRoadParams {
    Vec2  center{0, 0};
    Real  extent = 400;     // half-size of the square region (m)
    Real  cellSize = 90;    // target block spacing (m)
    Real  jitter = 0.18;    // vertex jitter as a fraction of cellSize
    Real  dropout = 0.0;    // probability a local street segment is removed
    Real  arterialWidth = 16, collectorWidth = 11, localWidth = 8;
    uint32_t seed = 0;
};
RoadGraph gridRoads(const GridRoadParams& params);

// Radial road generator (ADR-0044): concentric ring roads + radial spokes from a
// centre — the "Paris/Canberra" pattern. Rings every `ringSpacing` out to
// `extent`, `spokes` radials from the middle, vertices jittered. The outer ring
// is arterial so the region stays enclosed. Deterministic for `seed`.
struct RadialParams {
    Vec2  center{0, 0};
    Real  extent = 400;      // outer radius (m)
    Real  ringSpacing = 70;  // distance between concentric rings (m)
    int   spokes = 12;       // radial roads from the centre
    Real  jitter = 0.08;     // node jitter as a fraction of ringSpacing
    Real  arterialWidth = 16, collectorWidth = 11, localWidth = 8;
    uint32_t seed = 0;
};
RoadGraph radialRoads(const RadialParams& params);

// Planarize: split edges wherever they cross and merge coincident nodes, so the
// only adjacencies are at shared endpoints (precondition for face extraction).
RoadGraph planarize(const RoadGraph& graph, Real tol = 0.5);

// Extract the minimal interior faces (city blocks) of an already-planar graph via
// a half-edge DCEL "next clockwise" traversal, discarding the unbounded outer
// face and degenerate slivers. Each returned polygon is CCW. (city-plan §3.2.)
std::vector<Poly2> extractBlocks(const RoadGraph& graph, Real minArea = 50);

}  // namespace engine

#endif
