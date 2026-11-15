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

// THE BUILDER'S LANE: a lane as it is laid and drawn (its centre line and both edges). The road model's lane
// (road_model.h ModelLane) is the record made from it once it is built, and the only place that says which streets the
// lane runs along and which junction stands at each end (issue #292: this type once carried a copy of both).
struct Lane {
    std::string id, kind, cls;      // kind: through | turn | slip | aux | ramp | connector
    int parent = -1;                // index of the parent edge; -1 for connectors
    double w = 3.5; int dir = +1;   // dir: +1 along the parent spine, -1 against it
    std::vector<Vec2> xy, left, right;
    std::vector<double> s;
    int srcLane = -1, dstLane = -1; double z0 = 0, z1 = 0;   // connectors: the lanes joined and their end heights
    // WHERE IT LIES ON THE ROAD MODEL (ADR-0417), as laid -- none of it changes the lane's shape:
    //   `slot`: its place across its road, counted from the centre-line side to the kerb side among the lanes running its
    //   way (laneSlots: f0 / b0 is 0); a turn pocket born on the kerb side is one beyond the kerb lane (the count of lanes
    //   that way), one born on the centre-line side is -1; a ramp's lanes 0 and 1; a connector -1.
    //   `spineFrom` / `spineTo`: the stretch of its parent's spine it is laid along (stations, from <= to).
    //   (The streets it runs along and the junction at each end are the model's: ModelLane::streets, atStart, atEnd.)
    //   `sweptFrom` / `diesInto` (ADR-0426): a lane born along its road out of a neighbour, or dying into one -- a turn
    //   pocket swept out of the lane on its side (and back into it where it tapers out), a ramp's second lane dying into
    //   its first -- names that lane (an index into LaneSet::lanes); -1 otherwise. (A ramp's lane leaving or joining its
    //   host is stated by the ramp's anchors, EdgeSpec::anchorLanes.)
    int slot = -1;
    double spineFrom = 0, spineTo = 0;
    int sweptFrom = -1, diesInto = -1;
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
// expands into lanes and the declared connectors are generated. Each lane is given its slot and its stretch of spine.
LaneSet expand(RoadLabGraph& g);
// The distances from one end of a ramp's stretch along its host (deceleration or auxiliary lane and taper, `length`
// metres) at which the builder lays it: every whole metre, then the length itself, so the stretch ends exactly where the
// road file states (issue #292: it once stopped at the last whole metre).
std::vector<double> rampEndDistances(double length);
// The station along a ramp's host where its anchor stands (the gore), as the builder composes the ramp.
double anchorStation(const EdgeSpec& host, const RampAnchor& a);
// The host lane a ramp's anchor attaches to, by its side (as read, road_graph_spec.cpp readAnchor): "right" the forward
// lanes' kerb lane (their outer lane, on the spine's right in plan coordinates; written "kerb"), "back-right" the
// backward lanes' (written "back-kerb"), "left" the forward lanes' offside lane.
LaneSlot anchorSlot(const RoadLabGraph& g, const EdgeSpec& host, const std::string& side);

// Same travel direction and centrelines about one lane width apart over >= 20 m.
std::vector<std::pair<int, int>> adjacency(const LaneSet& L);

// Height of a lane at plan points, from its parent's profile (or a connector's end blend).
double laneHeightAt(const Lane& l, const RoadLabGraph& g, const Vec2& p);

}  // namespace roads::lanes
}  // namespace engine

#endif
