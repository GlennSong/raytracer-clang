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

// Road classes, most major first. Freeway = limited-access, grade-separated, divided; Ramp =
// a one-way connector (on/off ramp). Arterial/Collector/Local are the surface hierarchy.
// (Comparisons on this are ternary "is Arterial? is Collector? else Local"-style, so the two
// new members fall through to sensible defaults in older code; DesignRules gives them real
// parameters — ADR-0052.)
enum class RoadClass : uint8_t { Freeway, Arterial, Collector, Local, Ramp };

// What MEETS at a node — not how many edges (roads-v2.2 semantic layer,
// issue #17). Auto = unclassified; classifyRoadGraph (road_semantics.h)
// fills it, and a non-Auto value on input is an authored/baked HINT that
// wins at degree >= 3 (bakeCorridorIntoNet stamps its gores and landings —
// the bake KNOWS which side of the interchange it built; geometry alone
// cannot tell a Merge from a mirror-image Diverge).
enum class JunctionKind : uint8_t {
    Auto = 0,       // not yet classified
    None,           // open road: degree 2 (through / curve sample) or isolated
    DeadEnd,        // degree 1
    Intersection,   // streets crossing streets — pads, zebras, signals live here
    Merge,          // gore: an on-ramp joins a freeway mainline
    Diverge,        // gore: an exit ramp leaves a freeway mainline
    Landing,        // a ramp foot meets street(s)
};
// Consumers that only care "is this a gore?" go through this — Merge vs
// Diverge is bake-only truth and no current consumer needs the difference.
inline bool isGore(JunctionKind k) {
    return k == JunctionKind::Merge || k == JunctionKind::Diverge;
}

// Per-edge ACCESS bits (semantic layer): derived once by classifyRoadGraph
// and STORED, so consumers stop re-deriving "freeway-ness" with their own
// class checks. kWalkable mirrors RoadEdge::walkable this round (the bool
// stays for nav/pathfind compat until a later cleanup).
namespace road_access {
constexpr uint8_t kWalkable   = 1;   // pedestrians travel along it
constexpr uint8_t kFrontage   = 2;   // lots/parking/sidewalk band may front it
constexpr uint8_t kCrossable  = 4;   // may host a zebra crossing
constexpr uint8_t kSignalable = 8;   // counts as a signal approach
constexpr uint8_t kAllStreet =
    kWalkable | kFrontage | kCrossable | kSignalable;
}  // namespace road_access

struct RoadNode {
    Vec2 pos;
    // Carriageway height here. elevAbsolute=false (default): height ABOVE
    // GROUND (0 = at grade). elevAbsolute=true (corridor decks/ramps): the
    // ABSOLUTE deck Y — ground varies between chain nodes on hills, so a
    // ground-relative value made deck traffic hover/sink between nodes.
    Real elev = 0;
    bool elevAbsolute = false;
    // Semantic layer: what meets here (see JunctionKind above). Auto until
    // classifyRoadGraph runs; appended defaulted member — every existing
    // brace-init keeps working.
    JunctionKind kind = JunctionKind::Auto;
};
// `layer` is the grade-separation level (ADR-0051): 0 = ground. Two edges that cross in XY
// are the same intersection only when they share a layer; different layers pass over/under
// (an overpass), so the crossing is NOT turned into a shared node. Default 0 = the old
// all-at-grade behaviour.
// Where an edge came from (§10): the unified level graph carries streets and
// corridor-derived chains side by side; meshing and furniture dispatch on it.
enum class RoadProvenance : uint8_t { Street = 0, CorridorMain, CorridorRamp };

struct RoadEdge {
    int a = 0, b = 0;
    Real width = 8;
    RoadClass klass = RoadClass::Local;
    int layer = 0;
    RoadProvenance provenance = RoadProvenance::Street;
    // One-way in stored direction (a -> b). A freeway CARRIAGEWAY is one-way
    // by construction (plan §8/§9): the corridor publishes two of these, one
    // per direction, so traffic can never take a link against the flow.
    bool oneWay = false;
    // Roads-v2: index into the owning net's spec table (-1 = legacy, synthesize
    // from width/oneWay). Carried through netGraph so the mesher/bake read the
    // band model off the graph edge directly.
    int spec = -1;
    // Roads-v2 S8: may pedestrians travel along this edge? Baked from the
    // edge's RoadSpec band list (any Sidewalk band => true) where a spec is
    // known; edges without specs keep the permissive default and rely on the
    // class rule (pathfind's onFoot skip of Freeway/Ramp) as the backstop.
    bool walkable = true;
    // Semantic layer: access bits (road_access::k*), derived by
    // classifyRoadGraph from class/spec/elevation/neighbours and stored.
    uint8_t access = road_access::kAllStreet;
};

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

// Thin out the streets a road can't legally climb, WITHOUT fragmenting the network
// (ADR-0046). A grade-only drop shatters the graph (steep edges are often the only
// bridge between two parts), so this is a connectivity-preserving prune / max-
// spanning-forest completion: keep every gentle edge (grade |dh|/run <= maxGrade
// from `terrain`) and every Arterial, then add back the GENTLEST steep edges needed
// to reconnect whatever that leaves split. A steep street survives only where
// there's no gentler way around; a steep street with a gentle detour is dropped. The
// result has exactly the connected components of the input — no new gaps, no orphans
// — and the steep ground falls into the (bigger) natural-hillside blocks. Run on the
// PLANAR graph (so each short segment is judged on its own) before extractBlocks,
// and the blocks merge across the removed streets for free.
RoadGraph pruneSteepEdges(const RoadGraph& graph, const HeightField& terrain,
                          Real maxGrade, int samplesPerEdge = 3);

// Stitch a fragmented network back into ONE connected graph (ADR-0046). Contour
// coupling can leave streamlines that never cross (parallel on a steep flank), and
// pruning preserves but can't heal such splits — so this is the coherence backstop:
// while more than one component remains, add the single shortest connector edge
// between the closest pair of nodes in different components. Returns a graph with
// exactly one connected component (re-planarize afterward so the new connectors
// split cleanly at any road they cross).
RoadGraph connectComponents(const RoadGraph& graph, Real connectorWidth = 8);

// Planarize: split edges wherever they cross and merge coincident nodes, so the
// only adjacencies are at shared endpoints (precondition for face extraction).
RoadGraph planarize(const RoadGraph& graph, Real tol = 0.5);

// Layer-aware planarize (ADR-0051, the grade-separation rule). Like planarize, but a
// crossing splits into a shared node ONLY when the two edges share a `layer`; a crossing
// between different layers is a grade separation (overpass/underpass) and is left intact —
// no node, the edges pass over/under each other. Edge `layer` is carried onto the splits.
// With every edge on layer 0 this is identical to planarize.
RoadGraph planarizeLayered(const RoadGraph& graph, Real tol = 0.5);

// How many XY crossings between edges on DIFFERENT layers exist — i.e. the number of grade
// separations the layer assignment implies. For tests / an editor read-out.
int gradeSeparationCount(const RoadGraph& graph);

// Extract the minimal interior faces (city blocks) of an already-planar graph via
// a half-edge DCEL "next clockwise" traversal, discarding the unbounded outer
// face and degenerate slivers. Each returned polygon is CCW. (city-plan §3.2.)
std::vector<Poly2> extractBlocks(const RoadGraph& graph, Real minArea = 50);

}  // namespace engine

#endif
