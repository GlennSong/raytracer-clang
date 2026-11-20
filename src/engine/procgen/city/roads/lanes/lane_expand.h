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
#include <memory>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// THE BUILDER'S LANE: a lane as it is laid and drawn. It IS the lane's facts (road_model_parts.h LaneFacts: id, kind,
// road, direction, slot, width, stretch of spine, its stretch of the centre lines -- declared once, ADR-0435) plus what
// only the builder needs while it works: both edges and the working data below. The road model's lane (road_model.h
// ModelLane) takes the facts whole once the road is built, and is the only place that says which streets the lane runs
// along and which junction stands at each end.
//
// ITS CENTRE LINE is not held here (ADR-0438): it is the lane's stretch (`curve`) of the one object every lane of the
// build is laid into (`curves`: road_model_parts.h LaneCurves, made by expand), which the road model made from the build
// holds too. `xy()` gives its points and `s()` the distance along the lane to each, read in place.
struct Lane : LaneFacts {
    std::string cls;                // its road's class
    std::shared_ptr<const LaneCurves> curves;   // where every lane's centre line is held (shared with the build's road model)
    Span<Vec2> xy() const { return curves ? Span<Vec2>(curves->points, curve) : Span<Vec2>(); }
    Span<double> s() const { return curves ? Span<double>(curves->along, curve) : Span<double>(); }
    std::vector<Vec2> left, right;
    int srcLane = -1, dstLane = -1; double z0 = 0, z1 = 0;   // connectors: the lanes joined and their end heights
    //   `sweptFrom` / `diesInto` (ADR-0426): a lane born along its road out of a neighbour, or dying into one -- a turn
    //   pocket swept out of the lane on its side (and back into it where it tapers out), a ramp's second lane dying into
    //   its first -- names that lane (an index into LaneSet::lanes); -1 otherwise. (A ramp's lane leaving or joining its
    //   host is stated by the ramp's anchors, EdgeSpec::anchorLanes.)
    int sweptFrom = -1, diesInto = -1;
    bool isConnector() const { return kind == "connector"; }
};

struct LaneSet {
    std::vector<Lane> lanes;
    // THE WALKS (walk_lanes.h, ADR-0465): every sidewalk lane and corner walk the build laid for people on foot, laid into
    // the same centre lines. Not lanes of the pavement: nothing is drawn from them (buildFootprints reads `lanes` only),
    // and the road model takes them as its own list (RoadModel::walks), which the lane level does not read yet.
    std::vector<Lane> walks;
    // every lane's centre line (each lane's `curves` is this object); the road model made from the build holds it too.
    // Written by expand (the lanes) and layWalks (the walks), each appending.
    std::shared_ptr<LaneCurves> curves;
    std::map<std::string, int> byId;
    int find(const std::string& id) const { auto it = byId.find(id); return it == byId.end() ? -1 : it->second; }
    // Ownership rank: connectors yield to every lane; otherwise the parent class rank.
    double rank(int lane, const RoadLabGraph& g) const;
    std::string roadOf(int lane, const RoadLabGraph& g) const;
};

// ONE LANE LAID BESIDE `spine` (`nrm` its unit normals, `off` the offset along each): its centre line is appended to
// `curves`, the one object every lane of the build is laid into (road_model_parts.h LaneCurves), and the lane names its
// stretch of it; nothing may hold a view of an earlier lane's points across this call (the arrays grow). Expand lays the
// car lanes by it, and layWalks (walk_lanes.h) the walks.
Lane ribbonLane(const std::shared_ptr<LaneCurves>& curves, const std::string& id, int parent, const std::string& kind,
                const std::string& cls, double w, int dir,
                const std::vector<Vec2>& spine, const std::vector<Vec2>& nrm, const std::vector<double>& off);

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
