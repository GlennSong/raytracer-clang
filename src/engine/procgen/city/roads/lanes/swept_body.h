#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_SWEPT_BODY_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_SWEPT_BODY_H

// THE GROUND A LONG BODY COVERS AS IT FOLLOWS A PATH (issue #284, review item 9, ADR-0412).
//
// A vehicle's body follows its path by the lane graph's one body rule (lane_graph.h bodyOnPlan): front bumper on the
// line, rear bumper on the line its length further back, the body a rectangle along the chord between them. Through a
// bend the chord cuts inside the line -- a 12 m bus on a 10 m turn lies about 1.7 m inside the line its front follows
// at its middle -- so a body reaches ground that half its width either side of the line does not. The road builder
// needs that ground twice: to ask whether a turn stays on the paved road (lane_graph_connect.h), and to find which
// paths' bodies can touch (lane_graph_crossings.h).
//
// HOW IT IS SWEPT. The place `along` metres along a path is covered by every body whose stretch of line holds it: its
// front anywhere from `along` to `along + length`. Each of those bodies has a point on its middle line level with
// `along` (as far from its rear end, as a share of its length, as `along` is from its rear bumper along the line); the
// body's ground near `along` is within half its width of that point. So the ground any body near `along` can cover is
// within half the width of a short run of points: the SWEPT POINTS at `along`, one per kSweepFrontStep of front. On a
// straight they are all the line's own point; round a bend they run from the line inwards to the chord's deepest
// point and back. A rear bumper may lie on the path before (a connector's arriving lane) and a front on the path after:
// a BODY LINE is the path's centre line carried on into those, and straight on beyond them.
//
// Who reads what: the paved-road test reads the swept points (the ground, whichever body covers it); the crossing
// points place whole bodies front by front (bodyAt), because there it matters WHICH body touches -- one standing
// straight at its line, or one half round its turn (lane_graph_crossings.h).

#include "engine/procgen/city/roads/lane_graph.h"

#include <vector>

namespace engine {
namespace roads::lanes {

// A sweep places a body's front at most this far apart along its stretch (metres). Between two fronts a metre apart the
// point level with one place moves a few tens of centimetres round a tight bend; the crossing points' margin and a
// sweep's own margin cover that.
constexpr double kSweepFrontStep = 1.0;
// Two swept points closer than this are one (metres): a body on a straight sweeps one point, not a pile of the same.
constexpr double kSweptSamePoint = 0.01;

// A LINE A BODY LIES ALONG: a path's centre line, carried on before its start into the path leading in and after its end
// into the path it leads to (each as far as it was asked to reach), and straight on beyond those. Distances along are
// the path's own: 0 at its start, negative before it, beyond its length after it.
struct BodyLine {
    std::vector<Vec2> xy;
    std::vector<double> s;
    double length = 0;   // the path's own length
    // Where the line is, and which way it runs (unit), `along` metres along; straight on past its two ends.
    Vec2 pointAt(double along) const;
    Vec2 directionAt(double along) const;
};
// The line through `path`'s points, carried on into `before` (the points of the path leading in, ending where `path`
// begins; empty: none) and `after` (the points of the path it leads to, starting where `path` ends; empty: none), each
// kept for `reach` metres from the joint.
BodyLine makeBodyLine(const std::vector<Vec2>& before, const std::vector<Vec2>& path, const std::vector<Vec2>& after,
                      double reach);
// A path's centre line points, from the graph (empty for an unknown path).
std::vector<Vec2> pathPoints(const LaneGraph& g, PathId path);

// THE BODY with its front `front` metres along the line, `length` long and `width` wide (bodyOnPlan: its rear bumper
// `length` back along the line).
PlanRectangle bodyAt(const BodyLine& line, double front, double length, double width);

// THE SWEPT POINTS at `along` of a body `length` long (see the top of this file), appended to `out`, near-repeats
// (kSweptSamePoint) dropped. A body of no length sweeps the line's own point.
void sweptPoints(const BodyLine& line, double along, double length, std::vector<Vec2>& out);

}  // namespace roads::lanes
}  // namespace engine

#endif
