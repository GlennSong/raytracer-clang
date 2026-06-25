#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_CONSTRAINTS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_CONSTRAINTS_H

#include "road_network.h"   // RoadGraph

namespace engine {

// The road local-constraints pass (ADR-0052, first rules). Generate-and-constrain
// (after Parish & Müller, CityEngine 2001): a generator lays down candidate geometry
// from a soft goal field; THIS pass walks the graph and makes each crossing *legal*
// against its immediate neighbourhood — adjust / snap / rewrite, never a global solve.
//
// Two node rules are implemented here, and they share one piece of geometry:
//   * MIN ARM ANGLE — two roads must not leave a node closer than `minArmAngle`. A node
//     can therefore hold at most 2*pi/minArmAngle arms, so the "too many spokes" hub
//     (ADR-0044: the curb-corner trim diverges as the neighbour angle -> 0) is
//     unrepresentable rather than fixed up after the fact.
//   * MAX DEGREE — a node with more than `maxDegree` arms is too busy for a flat patch.
// A node that violates EITHER is PROMOTED to a roundabout: the dimensionless super-node
// is given extent — replaced by a ring of attach nodes (one per arm) joined by sampled
// ring arcs. Every node that survives is then degree <= 3 (two ring neighbours + one
// spoke), which the analytic junction pad (buildRoadMesh) already handles cleanly.
//
// Pure + headless. Degree-2 nodes (through-roads, curve samples) and dead ends are never
// touched. A 4-way grid crossing (degree 4, healthy angles) is left as a flat patch.
struct RoadRules {
    // Two arms closer than this (radians) can't share a flat junction -> promote. Default
    // ~30 deg: below it the curb corners would overlap even at a normal width.
    double minArmAngle = 0.52;
    // Above this degree a node is promoted to a roundabout. 4 keeps grid crossings flat.
    int    maxDegree = 4;
    // Roundabout ring radius (m). 0 = auto: the tightest adjacent arm pair sets it, so the
    // attach points clear each other (chord >= the two half-widths + margin) — the same
    // w/sin(theta) geometry the trim diverges on, used here to size the ring instead.
    double roundaboutRadius = 0.0;
    double islandRadius = 4.0;     // floor on the auto radius (a real centre island)
    double maxRadius = 60.0;       // cap so a near-parallel pair can't ask for an infinite ring
    double clearMargin = 1.0;      // extra spacing between adjacent attach points (m)
    double arcChordError = 0.3;    // ring-arc sampling tolerance (m)
};

// Run the pass: return a new graph with every over-busy / too-acute node promoted to a
// roundabout. Orphaned super-nodes are dropped and the node list compacted. A graph with
// no violating node is returned structurally unchanged.
RoadGraph applyConstraints(const RoadGraph& graph, const RoadRules& rules = {});

// Would node `v` be promoted under `rules`? (degree > maxDegree, or its tightest adjacent
// arm gap < minArmAngle). Exposed for the editor (warn / preview) and for tests.
bool nodeNeedsRoundabout(const RoadGraph& graph, int v, const RoadRules& rules);

}  // namespace engine

#endif
