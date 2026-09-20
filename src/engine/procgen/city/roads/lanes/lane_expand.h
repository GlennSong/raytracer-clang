#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_EXPAND_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_EXPAND_H

// Lanes are the atomic paved object (ADR-0083). A road expands into constant-width ribbons
// at the offsets its RoadSpec bands dictate. A lane is BORN by sweeping its centreline out of
// a neighbour's and DIES by converging into one; there are no width tapers. Ramps compose
// their polylines from a gore anchored on the host's outer lane: [birth + deceleration] +
// authored approach + [auxiliary + death]. Connectors are Hermite lanes from a lane's end to
// a point on another lane. Adjacency is derived from proximity, never authored.

#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include <map>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

struct Lane {
    std::string id, kind, cls;      // kind: through | turn | slip | aux | ramp | connector
    int parent = -1;                // index of the parent edge; -1 for connectors
    double w = 3.5; int dir = +1;   // dir: +1 along the parent spine, -1 against it
    std::vector<Vec2> xy, left, right;
    std::vector<double> s;
    int srcLane = -1, dstLane = -1; double z0 = 0, z1 = 0;   // connectors: the lanes joined and their end heights
    bool isConnector() const { return kind == "connector"; }
};

struct LaneSet {
    std::vector<Lane> lanes;
    std::map<std::string, int> byId;
    int find(const std::string& id) const { auto it = byId.find(id); return it == byId.end() ? -1 : it->second; }
    // Ownership rank: connectors yield to every lane; otherwise the parent class rank.
    double rank(int lane, const RoadLabGraph& g) const;
    std::string roadOf(int lane, const RoadLabGraph& g) const;
};

// Ramps get their composed geometry (mutates the graph's ramp edges), then every paved edge
// expands into lanes and the declared connectors are generated.
LaneSet expand(RoadLabGraph& g);

// Same travel direction and centrelines about one lane width apart over >= 20 m.
std::vector<std::pair<int, int>> adjacency(const LaneSet& L);

// Height of a lane at plan points, from its parent's profile (or a connector's end blend).
double laneHeightAt(const Lane& l, const RoadLabGraph& g, const Vec2& p);

}  // namespace roads::lanes
}  // namespace engine

#endif
