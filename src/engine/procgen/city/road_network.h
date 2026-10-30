#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NETWORK_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_NETWORK_H

#include "polygon.h"
#include "road_spec.h"          // RoadSpec — the band table RoadEdge::spec indexes
#include <cstdint>
#include <vector>

namespace engine {

// The road network (ADR-0038 §3 / city-plan §3.1-§3.2): a planar graph whose
// *faces* are the city blocks. Roads are the lines; blocks are the holes between
// them. Get the graph right and blocks fall out via planar-face extraction.

// Road classes, most major first. Freeway = limited-access, grade-separated, divided; Ramp =
// a one-way connector (on/off ramp). Arterial/Collector/Local are the surface hierarchy.
// Alley = a block-interior service lane (city-pipeline stage 10): today it lives only in the
// lot pipeline's clearance graph — the frontage gate counts it as street surface and
// buildings keep just off its pavement — it is never meshed as a carriageway or routed by
// nav. (Comparisons on this are ternary "is Arterial? is Collector? else Local"-style, so
// appended members fall through to sensible defaults in older code; DesignRules gives the
// real classes parameters — ADR-0052.)
enum class RoadClass : uint8_t { Freeway, Arterial, Collector, Local, Ramp, Alley };

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
constexpr uint8_t kFootpath   = 16;  // a FOOTPATH (a park's or the quad's walk): pedestrians only, never a vehicle
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
    // Spline shape (ADR-0049, was RoadNet::tangents): the through-direction at
    // this knot. Zero = auto — Catmull-Rom on a through-road, straight into a
    // junction or dead end. Editing one overrides the auto for that knot, which
    // is how a road is shaped by dragging its handles. Appended, like `kind`, so
    // positional brace-init keeps working.
    Vec2 tangent{0, 0};
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
    // Roads-v2 kerbside PARKING band, RESOLVED from the spec when the graph is
    // built (parkWidth 0 = this road has no parking). `parkOffset` is the
    // lateral distance from the carriageway centreline to the CENTRE of the
    // kerbside Parking band; `parkWidth` is that band's width. Baked onto the
    // edge — exactly like `walkable` — because the consumers downstream (the
    // nav graph, the city sim's parking bays) never see the owning net's spec
    // table. Everything that places a parked car reads these two numbers.
    Real parkOffset = 0;
    Real parkWidth = 0;
    // Roads-v2 S3 (was RoadNet::edgeBaked): this edge was BAKED from a corridor
    // solve. The corridor draws, carves and navigates itself, so street passes
    // skip these — and block extraction must never treat one as a block
    // boundary. An AUTHORED freeway-class edge keeps false and meshes normally.
    // Appended so positional brace-init ({a, b, width, klass, layer}) keeps
    // working — road_network.cpp and road_net.cpp both rely on that.
    bool baked = false;
};

struct RoadGraph {
    std::vector<RoadNode> nodes;
    std::vector<RoadEdge> edges;
    // Roads-v2 band table (road-graph-unification step 3): RoadEdge::spec
    // indexes into this. Empty for most graphs — only the metro street-parking
    // pass and baked corridors populate it; spec = -1 means "legacy, synthesize
    // a section from the edge width + the owning look's sidewalk/curb".
    std::vector<RoadSpec> specs;

    // Add a node, snapping to an existing one within `tol` (keeps the graph clean).
    int addNode(const Vec2& p, Real tol = 0.5);
    void addEdge(int a, int b, Real width = 8, RoadClass klass = RoadClass::Local);
    Real edgeWidth(int edgeIndex) const { return edges[edgeIndex].width; }
};

// DANGLING ENDS become T junctions (device: "can we properly join those
// roads"). A road whose dead end stops inside another road's corridor — the
// end's own half-width plus the other's plus the sidewalk — never got a
// junction: no pad, no crosswalk, and the through road's curb and sidewalk
// run straight across the stub (metro_v2_test had exactly two, the city map
// census's last two places). Each such end is joined: the other edge is
// split at the end's projection and an edge of the end's own class/width
// bridges the gap. Same layer only, streets only: freeway/ramp/baked edges
// and anything within 40 m of them are corridor territory and left alone
// (a street end joined INTO a ramp broke a merge on the freeway metro);
// one-way edges (a divided road's carriageways) and ends that end TOGETHER
// with another dead end within 30 m (twin carriageways, paired stubs at a
// city edge) are not stubs inside another road. Returns one record per join.
// An end that overlaps a road it is NOT heading into (the other road skirts
// past its cap obliquely — metro's arterial ending at the shore beside a
// local) would make an absurd acute T; it is TRIMMED instead: the end node
// pulled back along its last edge until its cap clears the other road's
// corridor and sidewalk (`trimmed`, `pulledBack`), still a dead end.
struct DanglingJoin {
    Vec2 end;          // the dead end that was joined (or trimmed: its old position)
    Vec2 at;           // the new junction node on the other road (or the end's new position)
    Real gap = 0;      // centreline-to-centreline distance bridged (or cleared)
    RoadClass endClass = RoadClass::Local, throughClass = RoadClass::Local;
    bool trimmed = false;
    Real pulledBack = 0;
};
std::vector<DanglingJoin> joinDanglingEnds(RoadGraph& g, Real sidewalk);

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

// Extract the minimal interior faces (city blocks) of an already-planar graph via
// a half-edge DCEL "next clockwise" traversal, discarding the unbounded outer
// face and degenerate slivers. Each returned polygon is CCW. (city-plan §3.2.)
std::vector<Poly2> extractBlocks(const RoadGraph& graph, Real minArea = 50);

}  // namespace engine

#endif
