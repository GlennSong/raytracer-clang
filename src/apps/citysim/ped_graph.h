#ifndef RAYTRACER_APPS_CITYSIM_PED_GRAPH_H
#define RAYTRACER_APPS_CITYSIM_PED_GRAPH_H

#include "../../engine/ai/nav_graph.h"   // engine::NavGraph, Vec2, Real
#include "places.h"                       // PlaceMap, PlaceId
#include <vector>

namespace citysim {

using engine::Real;
using engine::Vec2;

// The PEDESTRIAN navigation graph (Living City, ADR-0066 Phase 2). Cars and
// pedestrians share the road NavGraph today, so a walker can only route along
// road centrelines and stop at the nearest road node — never AT a building door,
// and a car would happily "drive" down any door connector we added to that shared
// graph. This is the separate walkable layer: corner nodes along the sidewalks +
// one ENTRANCE node per place, joined by UNDIRECTED edges (walkers go both ways).
// A route therefore begins and ends at real doors, house → corner → … → shop-door.
//
// Pure data + pure builders/queries — no ECS, no rendering, no sim state — so it
// builds and is unit-tested headless like the rest of the substrate; deterministic
// (ADR-0002). Crosswalk edges are implicit at shared corner nodes for now (a
// junction's incident sidewalks meet there); explicit mid-block crossings are a
// later refinement. Live-sim integration (routing walkers on this instead of the
// car graph) is a separate, careful step — this lands the tested foundation.

struct PedEdge {
    enum class Kind : uint8_t { Sidewalk, Entrance };
    int a = 0, b = 0;     // PedGraph node indices (undirected)
    Real length = 0;      // metres
    Kind kind = Kind::Sidewalk;
};

struct PedGraph {
    std::vector<Vec2> nodes;                // node positions (world XZ)
    std::vector<PedEdge> edges;
    std::vector<std::vector<int>> adj;      // node -> incident edge indices
    // The ped node that IS place `p`'s door, parallel to PlaceMap::places()
    // (entranceNode[p] indexes `nodes`); -1 if the place couldn't be attached.
    std::vector<int> entranceNode;

    int nodeCount() const { return static_cast<int>(nodes.size()); }
    int edgeCount() const { return static_cast<int>(edges.size()); }
    // The door node for a place, or -1.
    int doorOf(PlaceId place) const {
        return place < entranceNode.size() ? entranceNode[place] : -1;
    }
};

// Build the walkable graph from the road net + the places: a corner node per road
// node, a sidewalk edge per road link, and an entrance node + connector per place
// (snapped to the nearest road node). Deterministic.
PedGraph buildPedGraph(const engine::NavGraph& roads, const PlaceMap& places);

// Shortest walk (A*, Euclidean heuristic) from node `from` to node `to`, as a
// list of node indices inclusive of both ends. Empty if either is invalid or no
// path exists (`from == to` returns just that node).
std::vector<int> pedRoute(const PedGraph& g, int from, int to);

}  // namespace citysim

#endif
