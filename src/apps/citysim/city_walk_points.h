#ifndef RAYTRACER_APPS_CITYSIM_CITY_WALK_POINTS_H
#define RAYTRACER_APPS_CITYSIM_CITY_WALK_POINTS_H

// THE SIM'S WALK POINTS (#306, the street map's deletion): the points on the road model's walks where the city sim's
// people rest, set out from and go to. A walk point is a place on a walk (WalkPoint), where that is, and which part of
// the walks it is in; the sim numbers them, and an agent's home, work, rest and trip goal are walk-point numbers (the
// sim's code still calls them nodes). They are what the street map's nodes were to the sim, with nothing else of the
// street map: no links, no routing. A walk between two of them is the one router's (findWalkRoute,
// CitySim::pointRoute).
//
// Words: a PART is a set of walks a walker can get round (reachableWalks): two points in one part reach each other on
// foot, and two in different parts do not (another town: by train, city_rail_riders.h). A STREET POINT is one set out
// along a walk beside a street or along a walkway, every kWalkPointSpacing metres, in a part of at least
// kWalkPointPartLength metres of such walks (a sliver of sidewalk cut off by itself has none). A PLACE POINT is a
// place's door joined to the walks (CitySim::assignPlaces), added after the street points.

#include "../../engine/procgen/city/roads/lane_graph.h"   // WalkPoint, LaneGraph, ReachableParts
#include "../../rt_math.h"

#include <cstdint>
#include <vector>

namespace engine {
struct RoadModel;
}

namespace citysim {

using engine::Real;
using engine::Vec2;

// Metres between the street points along a walk.
constexpr Real kWalkPointSpacing = 30.0;
// The least length of street and walkway walks a part has for street points to be set out in it.
// (60 m: a 4-way cross of 40 m arms is a part of about 240 m and has its points; a sidewalk cut off by itself, a sliver
// between two roads, is shorter)
constexpr Real kWalkPointPartLength = 60.0;

struct WalkPointMap {
    std::vector<Vec2> nodes;               // per point: where it is (LaneGraph::pointAt)
    std::vector<engine::WalkPoint> at;     // per point: its place on the walks
    std::vector<Vec2> along;               // per point: the walk's direction there (unit)
    std::vector<int> part;                 // per point: its walk's part (ReachableParts::partOf)
    int streetPoints = 0;                  // the first this many are street points; the rest are places' points

    int nodeCount() const { return static_cast<int>(nodes.size()); }
    int streetNodeCount() const { return streetPoints; }
    bool empty() const { return nodes.empty(); }

    // The street points set out along the open walks (`open`: per walk, non-zero where a walker may use it) of the
    // parts `parts` (reachableWalks over the same) long enough. Clears what was there.
    void build(const engine::RoadModel& model, const std::vector<uint8_t>& open, const engine::ReachableParts& parts);
    // Drops the places' points, keeping the street points (before the places are given theirs again).
    void dropPlacePoints() {
        const std::size_t n = static_cast<std::size_t>(streetPoints);
        nodes.resize(n);
        at.resize(n);
        along.resize(n);
        part.resize(n);
    }
    // Adds a point at `at` on `model`'s walks, in part `inPart`; its number.
    int add(const engine::RoadModel& model, const engine::WalkPoint& at, int inPart);
    // The point nearest `p` (-1: none); the `k` nearest, nearest first.
    int nearestNode(Vec2 p) const;
    std::vector<int> nearestNodes(Vec2 p, int k) const;
    // Are points `a` and `b` in one part (each walked to from the other)?
    bool samePart(int a, int b) const {
        return a >= 0 && b >= 0 && a < nodeCount() && b < nodeCount() && part[static_cast<std::size_t>(a)] == part[static_cast<std::size_t>(b)];
    }
};

}  // namespace citysim

#endif
