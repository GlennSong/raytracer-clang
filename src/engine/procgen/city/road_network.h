#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NETWORK_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NETWORK_H

#include "polygon.h"
#include "../terrain_field.h"   // HeightField — terrain-aware routing/pruning (ADR-0046)
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

// A curved road centreline (the arc a ring/roundabout follows). Curves are the
// source of truth; a curve is SAMPLED into the planar graph as a fine polyline
// with a bounded chord error, so the planar pipeline (planarize/blocks/parcels)
// stays segment-based while the road reads as a true curve and lots line the arc.
// Straight roads are just ordinary edges. (Future: cubic-spline artist roads.)
struct RoadArc {
    Vec2 center;
    Real radius = 0;
    Real a0 = 0, a1 = 0;        // swept from a0 to a1 (radians, signed)
    Vec2 at(Real t) const;      // point on the arc, t in [0,1]
    Real length() const { return std::abs(a1 - a0) * radius; }
};
// Sample `arc` from node n0 (at a0) to n1 (at a1) into a chain of edges with
// intermediate nodes, keeping the chord error <= maxErr (so the polyline is
// indistinguishable from the true curve). The endpoints stay n0/n1 so spokes
// attach to them. `minSegs` is a floor for tiny arcs.
void sampleArc(RoadGraph& g, int n0, int n1, const RoadArc& arc, Real width,
               RoadClass klass, Real maxErr = 0.15, int minSegs = 1);

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

// Radial road generator (ADR-0044): concentric ring roads + radial avenues — the
// "Paris/Étoile" pattern. Rings every `ringSpacing` out to `extent`; each ring is
// a true circular ARC sampled to a fine polyline (bounded chord error), so it
// reads as a real curve and lots line it. `spokes` avenues radiate from the
// central ROUNDABOUT (the innermost ring) outward — they meet the ring, never a
// centre point, so there is no spike and the centre is an island. The outer ring
// is arterial so the region stays enclosed. Deterministic for `seed`.
struct RadialParams {
    Vec2  center{0, 0};
    Real  extent = 400;      // outer radius (m)
    Real  ringSpacing = 70;  // distance between concentric rings (m)
    int   spokes = 8;        // radial avenues from the roundabout
    int   ringSubdiv = 6;    // minimum chords per avenue gap (arc sampling adds more)
    Real  jitter = 0.03;     // avenue ANGLE jitter (kept on the circle so rings stay round)
    Real  arterialWidth = 16, collectorWidth = 11, localWidth = 8;
    uint32_t seed = 0;
};
RoadGraph radialRoads(const RadialParams& params);

// Tensor-field road generator (ADR-0044; after Chen et al., "Interactive
// Procedural Street Modeling", SIGGRAPH 2008). The single generator that
// *dovetails* radial and grid: instead of stamping one pattern, it builds a
// continuous field of road orientations and traces streets along it, so the
// layout morphs smoothly from radial in the core to a grid at the rim — no hard
// seam between the two.
//
// The field is a symmetric, traceless 2nd-order tensor T(p) = [[a,b],[b,-a]];
// its two eigenvectors are everywhere perpendicular and have no head/tail (a
// road has no direction), which is exactly why tensors — not vector fields —
// blend without the "cowlick" singularities a vector field gives. We sum two
// basis fields, each weighted by a smooth falloff:
//   * a RADIAL singularity at `center` whose major eigenvector points radially,
//     so its streamlines are avenues (major family) and ring roads (minor
//     family) — the Étoile core; its weight decays as exp(-(r/radialDecay)^2).
//   * a GRID field of constant orientation `gridAngle`, the rim lattice.
// Roads are then traced as evenly-spaced streamlines (Jobard & Lefebvre) of both
// eigenvector families at separation `spacing`, integrated with step `step`.
// Cross-family crossings become intersections in the usual planarize pass.
// Deterministic for `seed`.
struct TensorRoadParams {
    Vec2  center{0, 0};
    Real  extent = 400;        // half-size of the square region (m)
    Real  gridAngle = 0;       // orientation of the grid basis (radians)
    Real  radialStrength = 1;  // weight of the radial singularity at the core
    Real  radialDecay = 150;   // 1/e falloff radius of the radial field (m)
    Real  gridStrength = 1;    // weight of the constant grid basis
    Real  spacing = 70;        // target streamline separation = block size (m)
    Real  step = 8;            // streamline integration step (m)
    Real  arterialWidth = 16, collectorWidth = 11, localWidth = 8;
    uint32_t seed = 0;

    // Terrain coupling (ADR-0046, after Chen et al. §6). When `terrain` is set, the
    // field is blended toward the CONTOUR orientation (perpendicular to the terrain
    // gradient) where the ground is steep, so the avenues bend to follow the
    // hillside instead of marching across it. Flat ground is untouched. `slopeAlign`
    // scales how hard steep ground pulls the field; `maxGrade` is the slope at which
    // that pull saturates (so streets ease onto contours as the grade approaches it).
    const HeightField* terrain = nullptr;
    Real  slopeAlign = 1.0;
    Real  maxGrade = 0.12;
};
RoadGraph tensorRoads(const TensorRoadParams& params);

// Drop the streets a road can't legally climb (ADR-0046). For each edge, sample
// its grade |dh|/run from `terrain`; if it exceeds `maxGrade`, remove it — so the
// network only keeps streets gentle enough to walk and the steep ground falls into
// the (bigger) natural-hillside blocks. Two guards keep the result coherent:
// Arterials are never pruned (the structural skeleton stays connected), and an edge
// is kept when removing it would orphan an endpoint (no degree-0 nodes). Run on the
// PLANAR graph (so each short segment is judged on its own) before extractBlocks,
// and the blocks merge across the removed streets for free.
RoadGraph pruneSteepEdges(const RoadGraph& graph, const HeightField& terrain,
                          Real maxGrade, int samplesPerEdge = 3);

// Planarize: split edges wherever they cross and merge coincident nodes, so the
// only adjacencies are at shared endpoints (precondition for face extraction).
RoadGraph planarize(const RoadGraph& graph, Real tol = 0.5);

// Extract the minimal interior faces (city blocks) of an already-planar graph via
// a half-edge DCEL "next clockwise" traversal, discarding the unbounded outer
// face and degenerate slivers. Each returned polygon is CCW. (city-plan §3.2.)
std::vector<Poly2> extractBlocks(const RoadGraph& graph, Real minArea = 50);

}  // namespace engine

#endif
