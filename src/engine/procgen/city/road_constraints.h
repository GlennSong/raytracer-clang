#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_CONSTRAINTS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROAD_CONSTRAINTS_H

#include "road_network.h"

#include <functional>   // RoadGraph

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
// spoke), which the lattice junction pad already handles cleanly.
//
// Pure + headless. Degree-2 nodes (through-roads, curve samples) and dead ends are never
// touched. A 4-way grid crossing (degree 4, healthy angles) is left as a flat patch.
struct RoadRules {
    // Auto-promote a busy (> maxDegree) or acute (< minArmAngle) node to a roundabout ring (the
    // rule below). On by default for hand-authored nets and the mesh path; the procedural city
    // generator turns this OFF and instead caps degree (capDegree) + places roundabouts
    // deliberately, so a generated grid isn't tangled with auto-rings (road-network-v2-plan T1.1).
    bool   autoRoundabout = true;
    // NB: minArmAngle / maxDegree mirror DesignRules.minArmAngle / maxArmsAtGrade (road_rules.h),
    // the canonical junction policy; keep them in sync (they will fold into one source).
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
    double ringWidthFactor = 3.5;  // also floor the radius at widestArmHalfWidth * this, so the
                                   // ring is a drivable annulus around a visible island

    double maxRadius = 60.0;       // cap so a near-parallel pair can't ask for an infinite ring
    double clearMargin = 1.0;      // extra spacing between adjacent attach points (m)
    double arcChordError = 0.3;    // ring-arc sampling tolerance (m)
};

// Run the pass: return a new graph with every over-busy / too-acute node promoted to a
// roundabout. Orphaned super-nodes are dropped and the node list compacted. A graph with
// no violating node is returned structurally unchanged.
RoadGraph applyConstraints(const RoadGraph& graph, const RoadRules& rules = {});

// Would node `v` be promoted under `rules`? (degree > maxDegree, or its tightest adjacent
// arm gap < minArmAngle). False when rules.autoRoundabout is off. Exposed for the editor
// (warn / preview) and for tests.
bool nodeNeedsRoundabout(const RoadGraph& graph, int v, const RoadRules& rules);

// Cap node degree: split every node with more than `rules.maxDegree` arms into two nodes a
// short distance apart, joined by a short link, so the busiest crossing a flat junction has to
// mesh holds at most `maxDegree` arms — the procedural-city alternative to promoting it to a
// roundabout (road-network-v2-plan T1.2). Arms are partitioned by bearing; repeated until every
// node is within the cap. Pure + headless. A graph already within the cap is returned unchanged.
RoadGraph capDegree(const RoadGraph& graph, const RoadRules& rules = {});

// MAXIMUM bend at a through-node (device feedback: "sharp bends in the road ...
// creating some really bad overlap"). A degree-2 node whose two legs turn more
// than `maxTurn` (radians) folds the stroked carriageway over itself — roads do
// not hairpin mid-block. Relax every such node toward its neighbours' chord
// midpoint, repeatedly, until all through-bends are within the limit (or
// `iterations` passes elapse). Junctions and dead ends never move; degree-2
// geometry eases into a drivable curve. Deterministic and pure.
RoadGraph relaxSharpBends(const RoadGraph& graph, Real maxTurn = 0.9,
                          int iterations = 48);

// MINIMUM road length (device feedback: "we end up with some really short roads").
// Crossings that land close together (planarize) read as broken stubs — signals in
// the middle of the street, cramped junction pads. Repeatedly take the shortest
// edge under `minLen` and:
//   * MERGE its endpoints into one node (at their degree-weighted midpoint) when
//     the united node stays within `maxDegree` unique arms — two adjacent
//     crossings become one clean junction; or
//   * LENGTHEN it — push the endpoints apart along the edge axis to `minLen` —
//     when merging would over-crowd the junction (e.g. the staggered link a
//     capDegree split leaves can't merge back), so the short road becomes a real
//     drivable block face instead.
// Self-loops and duplicate parallel edges from a merge are dropped (widest wins);
// orphaned nodes are compacted away. Deterministic and pure.
RoadGraph mergeShortEdges(const RoadGraph& graph, Real minLen, int maxDegree = 4);

// Minimum INTERSECTION spacing (8km-city plan P2). mergeShortEdges works on raw
// edges, but a colonization-grown arterial is a chain of ~40 m curve segments
// between degree-2 nodes — raw edge length is curve sampling, not junction
// spacing. This walks each junction-to-junction SPAN (through degree-2 nodes,
// summing arc length) and fuses the two junctions of any span shorter than
// minSpan (degree-weighted midpoint, curve nodes of the short span dropped,
// every other chain re-targeted). Junctions whose fused degree would exceed
// maxDegree keep their span. Deterministic: shortest span first, ties by node
// index. Callers should re-planarize + re-cap afterwards — moving junctions
// can introduce crossings.
RoadGraph consolidateJunctionSpans(const RoadGraph& graph, Real minSpan,
                                   int maxDegree = 4);

// Region-aware overload (P7 density unlock): the span floor varies by
// position (evaluated at each span's midpoint) — a tight downtown floor and
// a big-block periphery floor coexist in one pass. The victim each round is
// the span most below its own floor (deterministic tie-breaks).
RoadGraph consolidateJunctionSpans(
    const RoadGraph& graph, const std::function<Real(const Vec2&)>& minSpanAt,
    int maxDegree = 4);

// Dissolve the redundant twin of a near-parallel junction arm pair (dir dot >
// minDot): delete the narrower/shorter arm's whole span when its far junction
// stays reachable within maxDetourSpans span-hops without it. The mesher's
// acute-pair trim cannot surface the sliver wedge such pairs enclose — the
// drive probe measured the gap. Deletion-only: planarity-safe, connectivity
// proven per cut.
RoadGraph dissolveAcuteArms(const RoadGraph& graph, Real minDot = 0.85,
                            int maxDetourSpans = 3);

// REALIGN an acute junction by bending its approaches (docs/curb-weld-analysis.md).
//
// Two arms leaving a node closer than `minAngle` are a sight-line problem and a
// handling problem before they are ever a paving problem: you cannot see around
// the corner, and you cannot take the turn at speed. Real practice fixes that in
// the LAYOUT — the approach is realigned to meet nearer square, with a straight
// run into the junction — not by paving cleverly around the angle.
//
// This is that move. For each offending pair the arms' departure bearings are
// pushed apart until every gap clears `minAngle`, and the change is realised by
// inserting a BEND NODE `runIn` metres along the arm and rotating only that
// stub. The far node never moves, so every block face hanging off it keeps its
// corners — which is the difference from `deAcute` (road_net.cpp), whose
// far-node rotation deformed the faces and is why the metro recipe skipped the
// district cleanup entirely.
//
// Junctions that cannot be opened (degree * minAngle > 2*pi — arithmetically
// impossible) are left alone for the next rung of the ladder: a roundabout, a
// stagger, or dissolveAcuteArms. Elevated nodes and baked corridor edges are
// never touched. Pure + headless; deterministic in node then arm order.
RoadGraph realignAcuteJunctions(const RoadGraph& graph, Real minAngle,
                                Real runIn = 18.0, int passes = 4);

// Corner-cut degree-2 vertices whose deflection exceeds maxTurn: delete the
// vertex and chord its neighbours. Strictly bend-reducing (terminates),
// face-preserving, junction-pinned — the backstop for bends relaxSharpBends
// cannot converge.
RoadGraph cutSharpCorners(const RoadGraph& graph, Real maxTurn);

// Drop exact parallel duplicate edges (same node pair), keeping the widest.
// A parallel pair is a two-edge loop: an un-relaxable 180-degree fold to any
// chain walk and a sliver face to the block extractor.
RoadGraph dropParallelEdges(const RoadGraph& graph);

}  // namespace engine

#endif
