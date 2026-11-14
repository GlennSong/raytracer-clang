#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CONNECT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CONNECT_H

// THE PATHS THROUGH JUNCTIONS (issue #238, docs/driving-briefs/B2-connectors.md). A CONNECTOR is the path a car follows
// through a junction from the lane it arrives in to the lane it leaves by. WHICH turns a junction offers, and which lane
// takes each, is stated by the road model (lane_turns.h, ADR-0426); this module is the builder's addition to a stated
// turn:
//
// - its SHAPE (connectorShape): straight, a circular arc as large as fits, straight; or, where no single arc joins the
//   two lane ends, two arcs. Never a free-form curve: its tightest point is what a vehicle must manage;
// - the CORNER FIT (fitConnector): checked against the vehicles passed in, too tight for a vehicle, or so placed that a
//   vehicle's body following it would leave the paved road, the connector is closed to that vehicle -- a corner fault
//   the lane level reports against the turn (lane_level.h);
// - and how much of the graph a car CAN GET ROUND (reachReport): the largest set of paths that can all reach one
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

// Two arms whose headings are within this of opposite ways (degrees) run straight through a junction, as the one-off
// layout tool states a junction's through pairs (road_layout.cpp) and the control planner reads its major road
// (control_plan.cpp). The lane level reads the stated pairs, never this.
constexpr double kStraightOnDegrees = 30.0;

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

// ---- fitting it to its corner --------------------------------------------------------------------------------------

// How far apart the sweep of a connector is tested against the paved road (metres along it) ...
constexpr double kSweepStep = 0.1;
// ... and how much wider than the vehicle, each side (metres): between two samples the edge of the sweep moves on, and a
// kerb's corner can reach in between them. (A crack between two lanes' footprints narrower than the step -- the outside
// of a kink where two roads meet, where neither footprint reaches -- can still pass between two samples.)
constexpr double kSweepMargin = 0.1;

// ONE TURN FITTED TO ITS CORNER: the path from the end of lane path `from` to the start of lane path `into` (the graph's;
// `laneFrom` / `laneInto` the builder lanes they were cut from, for the heights), shaped (connectorShape: the largest arc
// that fits, smaller ones where it cuts the kerb's corner) and checked against `vehicles` (each vehicle tag's turning
// radius, width and length; a tag with none is not limited): closed to each tag too tight for it, then to each whose
// body, swept along it every kSweepStep (its width and kSweepMargin each side, its length cutting inside the bends:
// swept_body.h, from where its front comes onto the connector to where its rear leaves it), leaves the lanes' footprints
// at its level (the paved road as the builder made it, its kerb returns included), widest first. A tag closed here is
// one the corner does not fit, with why (`whyClosed`, per tag index of the graph's vehicleTagNames): a corner fault.
struct ConnectorFit {
    ConnectorShape shape;                 // not ok: no arcs join the two lane ends going forward (`noArc`)
    std::vector<double> z;                // its heights, point by point
    TagMask open = 0, closed = 0;         // the vehicle tags it is open and closed to
    std::vector<std::string> whyClosed;   // per vehicle tag: why it is closed (empty: open, or not on both lanes)
    bool noArc = false;
    std::string why;                      // when it is open to nobody: why, in words
    Vec2 at;                              // where (the middle of the two lane ends, or where a body left the road)
};
ConnectorFit fitConnector(const Result& r, const LaneLookup& look, const LaneGraph& G, PathId from, PathId into, int laneFrom,
                          int laneInto, const std::vector<VehicleSize>& vehicles);

// HOW MUCH OF THE GRAPH A CAR CAN GET ROUND (reachableParts), with the lanes outside the largest part grouped by why:
// they cannot get back to it -- they lead into a junction whose turns fit no vehicle, or into a road that just ends --
// or nothing in it leads to them. Fills out.reach, out.reachableShare and out.outside.
void reachReport(LaneGraphBuild& out);

}  // namespace roads::lanes
}  // namespace engine

#endif
