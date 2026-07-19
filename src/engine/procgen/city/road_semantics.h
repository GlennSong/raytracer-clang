#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_SEMANTICS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_SEMANTICS_H

// The road graph SEMANTIC LAYER (roads-v2.2, issue #17). Nodes historically
// carried nothing — "junction" meant degree >= 3, re-derived independently
// in ~20 consumer sites, and freeway/ramp meaning was re-derived via class
// checks for four different purposes. Those heuristics disagreed exactly at
// freeway/street seams: zebra crosswalks at gores, sidewalk bands climbing
// ramp approaches, houses fronting the freeway. This module is the ONE
// classification pass: it stamps JunctionKind on every node and access bits
// on every edge; consumers read those instead of guessing.
//
// It also owns the graph AUDIT (issue #18): the planarity rule nobody ever
// asserted — two edges crossing in XZ must share a node or be properly
// grade-separated. "GradeSep" is deliberately a crossing RECORD from this
// audit, not a node kind: a legal overpass has no node at the crossing.

#include "road_network.h"

#include <functional>
#include <vector>

namespace engine {

using GroundFn = std::function<double(double, double)>;

// Stamp node kinds + edge access bits. Idempotent and cheap (one incidence
// build); safe to call on an already-classified graph. Non-Auto node kinds
// on input are authored/baked hints and win at degree >= 3. `groundAt`
// (optional) lets the approach-edge grade test see real terrain.
void classifyRoadGraph(RoadGraph& g, const GroundFn& groundAt = nullptr);

// True when any node is still JunctionKind::Auto — the self-heal test the
// direct consumers (buildNavGraph, buildRoadNetLattice) use so hand-built
// test graphs classify on entry.
bool roadGraphUnclassified(const RoadGraph& g);

// The planarity audit (issue #18). Every pair of edges whose segments cross
// in XZ while sharing neither a node index nor an endpoint position must be
// grade-separated by >= `clearance` metres at the crossing point; anything
// closer is a violation. elevAbsolute nodes resolve to their deck Y;
// relative nodes to groundAt(x,z)+elev (ground 0 when groundAt is null).
struct CrossingViolation {
    int edgeA = -1, edgeB = -1;
    Vec2 at;          // the XZ crossing point
    double dY = 0;    // vertical gap there (< clearance = the violation)
};
std::vector<CrossingViolation> auditRoadGraph(const RoadGraph& g,
                                              const GroundFn& groundAt = nullptr,
                                              double clearance = 4.5);

}  // namespace engine

#endif
