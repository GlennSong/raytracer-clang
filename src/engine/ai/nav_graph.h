#ifndef RAYTRACER_ENGINE_AI_NAV_GRAPH_H
#define RAYTRACER_ENGINE_AI_NAV_GRAPH_H

#include "../procgen/city/road_network.h"   // RoadGraph, RoadEdge, RoadClass, Vec2
#include <vector>

namespace engine {

// A runtime, queryable navigation graph derived from the road network (ADR-0059).
// The procedural pipeline consumes the RoadGraph at mesh-build time and discards
// it; agents need it to live. NavGraph is the persistent, *directed* lane graph
// that vehicles and pedestrians route over and drive on. Pure data + pure
// builders — no ECS, no rendering, no physics — so it builds and is unit-tested
// headless (Linux/CI), like the rest of the procgen substrate.

// One directed traversal of a road edge. An undirected RoadEdge becomes two
// NavLinks (one per direction); a one-way class (Ramp) becomes one. The geometry
// is the straight segment from->to: the source RoadGraph is already finely
// sampled (netGraph collapses straight runs and keeps bends as chains of short
// edges), so a curved road is a *chain* of NavLinks and each link is straight.
struct NavLink {
    int from = 0;                       // NavGraph node index (tail)
    int to = 0;                         // NavGraph node index (head)
    Real length = 0;                    // segment length (m)
    Real width = 8;                     // carriageway width of the source road (m)
    RoadClass klass = RoadClass::Local;
    int lanes = 1;                      // lanes available in THIS direction
    int layer = 0;                      // grade-separation level (0 = ground, >0 = bridge)
    // Carriageway height above ground at each end (from RoadNode.elev):
    // agents lerp it along the link, so a corridor deck or a descending ramp
    // positions traffic continuously (layer stays for legacy bridges).
    Real elevA = 0, elevB = 0;
    bool elevAbsolute = false;          // elevA/B are absolute deck Y
    // One-way link (freeway carriageway / ramp): its LANES span the full
    // link width centred on the chain, not the right half of a two-way road.
    bool oneWay = false;
    // S8: pedestrians may travel this link (baked from the road spec's
    // Sidewalk bands; see RoadEdge.walkable). Routing (onFoot) and the sim's
    // walkers both honour it.
    bool walkable = true;
    // Semantic layer (#17): access bits (road_access::k*) copied from the
    // source RoadEdge. kWalkable mirrors `walkable`; the new consumers key
    // on kFrontage/kCrossable/kSignalable.
    uint8_t access = road_access::kAllStreet;
};

// Semantic signal predicate (roads-v2.2 #17/S5): a node is signal-CONTROLLED
// iff it is a street INTERSECTION with at least 4 SIGNALABLE approaches. This
// replaces three duplicated filters (skip Freeway/Ramp + isJunction +
// outLinks>=4) that all counted RAMP links toward the >=4 threshold — so a
// 3-street + 1-ramp landing wrongly got signal heads. A Landing is not an
// Intersection, so it now correctly gets none. Declared after NavGraph.
struct NavGraph {
    std::vector<Vec2> nodes;                // node positions (world XZ; Vec2.x=x, .y=z)
    std::vector<NavLink> links;             // directed links
    std::vector<std::vector<int>> outLinks; // node -> indices of links departing it
    std::vector<uint8_t> junction;          // 1 if the node is an intersection (deg >= 3)
    // How far a KNOT-MERGED junction's members sat from its centroid (m); 0 for
    // ordinary nodes. The drawn crossing spans this far around the merged node,
    // so anything PLACED relative to the node (signal poles) backs off by it.
    std::vector<Real> nodeSpread;
    // Semantic layer (#17): JunctionKind per node (values of the road
    // graph's JunctionKind enum). junction[] keeps its degree-based meaning
    // this round — every box/gridlock ratchet reads it unchanged; nodeKind
    // carries the NEW distinctions (Intersection vs gore vs landing).
    std::vector<uint8_t> nodeKind;

    JunctionKind kindOf(int node) const {
        return node >= 0 && node < static_cast<int>(nodeKind.size())
                   ? static_cast<JunctionKind>(nodeKind[node])
                   : JunctionKind::Auto;
    }

    int nodeCount() const { return static_cast<int>(nodes.size()); }
    int linkCount() const { return static_cast<int>(links.size()); }

    // True if `node` is an intersection (three or more distinct neighbours) — a
    // place agents slow for / will yield at (ADR-0059).
    bool isJunction(int node) const {
        return node >= 0 && node < static_cast<int>(junction.size()) && junction[node];
    }

    // The number of signalable (street-class) approaches departing `node`.
    int signalableApproaches(int node) const {
        if (node < 0 || node >= static_cast<int>(outLinks.size())) return 0;
        int k = 0;
        for (int ol : outLinks[node])
            if (links[ol].access & road_access::kSignalable) ++k;
        return k;
    }
    // See the comment above the struct.
    bool signalControlled(int node) const {
        return kindOf(node) == JunctionKind::Intersection &&
               signalableApproaches(node) >= 4;
    }

    // Unit travel direction of a link (zero-length links report {1,0}).
    Vec2 direction(int link) const;

    // Point at parameter t in [0,1] along a link's centreline.
    Vec2 pointOnLink(int link, Real t) const;

    // Centre of lane `lane` (0 = lane nearest the road centreline) at param t,
    // offset to the driving side. Right-hand traffic: lanes sit to the RIGHT of
    // the travel direction, lane i at (0.5 + i) * laneWidth from the centreline.
    Vec2 laneCenter(int link, int lane, Real t, Real laneWidth = 3.5) const;

    // A pedestrian point at param t: on the verge just outside the kerb, on the
    // right of travel. `verge` is the extra offset beyond the carriageway edge.
    Vec2 sidewalkPoint(int link, Real t, Real verge = 1.0) const;

    // Nearest node / nearest link to a world point — for snapping trip ends.
    // Return -1 if the graph is empty.
    int nearestNode(const Vec2& p) const;
    int nearestLink(const Vec2& p) const;
};

// Build a NavGraph from an already-sampled RoadGraph (e.g. netGraph(net), or any
// of the generators in road_network.h). Surface roads are two-way; Ramp is
// one-way in its stored direction when `oneWayRamps`. Lanes per direction are
// derived from width: max(1, round(width / laneWidth / 2)).
struct NavBuildParams {
    Real laneWidth = 3.5;
    bool oneWayRamps = true;
    // Junction KNOT merge (ADR-0066 device fix). A generated crossing — two curvy
    // arterials meeting — planarizes into several intersection nodes a few metres
    // apart, joined by car-length stub links. Treating each as an independent
    // junction gave every stub its own stop line and hold box; the boxes overlap,
    // every held car sits in another junction's box, and the centre gridlocks
    // permanently. Semantically that knot is ONE intersection, so: cluster
    // JUNCTION nodes (degree >= 3) closer than this radius into one node at their
    // centroid. Degree-2 nodes are curve samples and never merge (collapsing them
    // would straighten the drawn bends). 0 disables.
    Real junctionMergeRadius = 7.0;
};
NavGraph buildNavGraph(const RoadGraph& roads, const NavBuildParams& params = {});

}  // namespace engine

#endif
