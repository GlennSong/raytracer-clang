#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CONNECT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CONNECT_H

// THE PATHS THROUGH JUNCTIONS (issue #238, docs/driving-briefs/B2-connectors.md). A CONNECTOR is the path a car follows
// through a junction from the lane it arrives in to the lane it leaves by. This module:
//
// - says WHICH TURNS a junction offers (junctionTurns): which arriving lane may turn into which leaving lane, and what
//   kind of turn that is. Pure -- the junction's shape and its lanes in, the turns out -- so the road planner can call
//   the same function and store its answer in the road graph (owed: today the lane graph builder calls it);
// - gives each turn its SHAPE (connectorShape): straight, a circular arc as large as fits, straight; or, where no single
//   arc joins the two lane ends, two arcs. Never a free-form curve: its tightest point is what a vehicle must manage;
// - CHECKS each one against the vehicles passed in (connectJunctions): too tight for a vehicle, or so placed that a
//   vehicle of its width following it would leave the paved road, the connector is closed to that vehicle; when that is
//   every vehicle it is REFUSED -- not built, and a line in the problems with its place and the reason;
// - says how much of the graph a car CAN GET ROUND (reachReport): the largest set of paths that can all reach one
//   another, and why each lane outside it is outside.
//
// Words: NEAR TURN, the turn that stays by the kerb; FAR TURN, the one across the oncoming traffic. Which side the kerb
// is on comes from one place (lane_graph_build.h kerbPerpSign); nothing here names a hand.

#include "engine/procgen/city/roads/lane_graph.h"

#include <cstdint>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

struct LaneGraphBuild;   // lane_graph_build.h
struct LaneLookup;       // deck_mesh.h
struct Result;           // lanes.h
struct VehicleSize;      // lane_graph_build.h

// ---- which turns a junction offers ---------------------------------------------------------------------------------

// A turn through less than this (degrees between the arriving and the leaving direction) is straight on. To become
// rule-set data with the planner's turn list.
constexpr double kStraightOnDegrees = 30.0;

// ONE WAY INTO OR OUT OF A JUNCTION: one section's lanes where they meet it.
struct TurnLeg {
    int road = -1;                 // the caller's road
    int dir = 1;                   // which way along it its traffic runs (+1 or -1): an arriving and a leaving leg of
                                   // one road running opposite ways are a road and its way back (no U-turn)
    Vec2 heading;                  // the way its traffic runs at the junction (a unit vector)
    std::vector<uint32_t> lanes;   // the caller's lane ids, offside to kerb
    std::vector<Vec2> ends;        // where each lane ends (arriving) or begins (leaving), in the same order
};

struct TurnRules {
    double straightOnDegrees = kStraightOnDegrees;
    // Which side of a lane its kerb lies: perp(the lane's direction) times this points toward the kerb (+1 or -1).
    double kerbPerpSign = -1;
};

struct JunctionTurn {
    uint32_t from = 0, to = 0;           // lane ids
    Turn turn = Turn::None;
    uint32_t fromLeg = 0, toLeg = 0;     // indices into the arriving and the leaving legs
};

// THE TURNS A JUNCTION OFFERS, every pair of an arriving and a leaving leg of different roads (never back onto the road
// it came from), in the order (arriving leg, leaving leg, from lane):
// - the kind from the angle turned: less than straightOnDegrees is straight on, unless another leaving leg is straighter
//   (it is straight on; this one a turn); a turn toward the kerb side is a near turn, the other way a far turn;
// - straight on: each arriving lane into the leaving lane nearest it sideways (where the road beyond has fewer lanes,
//   two arrive in one);
// - a near turn: the kerb-most arriving lane into the kerb-most leaving lane; a far turn: the offside-most into the
//   offside-most. Where the arriving leg has no way straight on, its lanes share the turns -- the offside half the far
//   turns, the kerb half the near turns (all of them when it has only one kind) -- lane k from the turn's side into
//   lane k from the same side of the leaving leg (into its last where it has fewer).
std::vector<JunctionTurn> junctionTurns(const std::vector<TurnLeg>& arriving, const std::vector<TurnLeg>& leaving,
                                        const TurnRules& rules = TurnRules());

// ---- a connector's shape -------------------------------------------------------------------------------------------

// Every connector leaves its lane straight for at least this far, and joins its next lane straight for at least this
// far (metres): so every connector leaving one lane starts with the same piece, bit for bit, running exactly the lane's
// way, and every one into a lane ends so. Short, as every centimetre of it comes off the arc's radius. (The curve is
// tangent at its ends; LaneGraph::directionAt reads a chord kTangentHalfSpan long, so on a bend of radius R starting at
// a joint the direction it reads turns by about kTangentHalfSpan / 2R there.)
constexpr double kConnectorLeadIn = 0.05;

struct ConnectorShape {
    std::vector<Vec2> points;         // from the start point to the end point, both exactly as given
    double tightestRadius = kNoBend;  // its tightest arc (kNoBend: none)
    bool singleArc = false;           // straight, one arc, straight (so a smaller arc can be asked for)
    bool ok = false;                  // false: no arcs join the two ends going forward (the points are then empty)
};
// THE PATH FROM ONE LANE END TO ANOTHER: from `from` heading `fromDir`, to `to` heading `toDir` (unit vectors). After the
// lead-in: where the two lines cross ahead of the one and behind the other, straight, then the largest circular arc
// that fits between them (no larger than `maxRadius`), then straight; otherwise (the lines parallel or crossing
// elsewhere: a jog sideways, an S-bend), or where two arcs bend more gently than the one that fits (the lines crossing
// close by one end), two arcs meeting tangent halfway in reach (a biarc). Its tightest point is what a vehicle must
// manage, so of the two the gentler is taken (one arc always, when a radius is asked for). Arcs are drawn within
// kArcChordError of the true circle.
ConnectorShape connectorShape(const Vec2& from, const Vec2& fromDir, const Vec2& to, const Vec2& toDir,
                              double maxRadius = kNoBend);
// Where the largest arc that fits takes a vehicle off the paved road (a near turn's arc cuts the kerb's corner more the
// larger it is), this many smaller arcs are tried, evenly down to the tightest vehicle's turning radius: the largest
// that keeps the most vehicles on the road is the connector.
constexpr int kSmallerArcTries = 16;

// ---- making them ---------------------------------------------------------------------------------------------------

// How far apart the sweep of a connector is tested against the paved road (metres along it) ...
constexpr double kSweepStep = 0.1;
// ... and how much wider than the vehicle, each side (metres): between two samples the edge of the sweep moves on, and a
// kerb's corner can reach in between them. (A crack between two lanes' footprints narrower than the step -- the outside
// of a kink where two roads meet, where neither footprint reaches -- can still pass between two samples.)
constexpr double kSweepMargin = 0.1;

// THE CONNECTORS OF EVERY JUNCTION in the build's graph: for each junction (in id order) its stated turns
// (junctionTurns over its arriving and leaving lanes, a leg per section; a carry-on's all straight on), each given its
// shape and checked against `vehicles` (each vehicle tag's tightest turning radius and width; a tag with none is not
// limited): closed to each vehicle tighter than its turning radius, then to each whose width (and kSweepMargin each
// side), swept along it every kSweepStep, leaves the lanes' footprints at its level (the paved road as the builder made
// it, its kerb returns included), widest first -- trying smaller arcs where the largest cuts a corner; refused when no
// vehicle remains. A junction the build refused (out.junctionNotes) gets none. Adds the
// connectors to the graph (their junction lists them, their from-lane lists them in `next`, which the caller writes
// into the graph), the refusal lines to out.problems, the refusals to out.refusals.
void connectJunctions(const Result& r, const LaneLookup& look, const std::vector<VehicleSize>& vehicles,
                      std::vector<std::vector<PathId>>& next, LaneGraphBuild& out);

// HOW MUCH OF THE GRAPH A CAR CAN GET ROUND (reachableParts), with the lanes outside the largest part grouped by why:
// they cannot get back to it -- they lead into a junction with turns refused, into a lane end not joined, or into a
// road that just ends -- or nothing in it leads to them. Fills out.reach, out.reachableShare and out.outside.
void reachReport(LaneGraphBuild& out);

}  // namespace roads::lanes
}  // namespace engine

#endif
