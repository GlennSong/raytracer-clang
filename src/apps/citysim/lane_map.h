#ifndef RAYTRACER_APPS_CITYSIM_LANE_MAP_H
#define RAYTRACER_APPS_CITYSIM_LANE_MAP_H

// THE REAL LANES ON THE SIM's LINKS (#218). The sim routes on NavLinks: straight pieces of each road's simplified
// centreline. It drew a car at the link's line plus (lane + 0.5) x spacing, so where the lanes builder laid its lanes
// differently (a ramp's own lane, an avenue's median, a one-way's offset, a curve the link cuts across) the car sat
// between the painted lanes. The builder's lanes are published at load (engine::LevelLanes); this matches each link to
// the lanes running its way and keeps, per link, each lane's line over the link's span -- offside (by the centre line,
// lane 0) to kerb, the sim's own lane order (we drive on the LEFT: rightOf(direction) is the kerb side).
//
// THE ROAD UNDER EACH LINK, AND ITS LANE COUNT (ADR-0418). The nav graph is built at load from the road graph stored in
// the level's bake, whose edges do not say which road-file road they came from. So the match also says which road each
// link runs along: of the roads with lanes running the link's way at its middle, the one whose centre line lies nearest
// the middle (the link was cut from that line), and which way along it. A link's lane count is then the one its road file states that way
// (engine::StatedLanes, from the road's class or its own "lanes"), and stateLanes writes it into the nav graph. This is
// an INTERIM way to the statement (docs/road-model-design.md step 7 rebuilds the nav from the road model, with ids, and
// deletes the match): a link with no lane at its middle keeps the nav graph's own count (the class table's, nav_graph.cpp).
//
// A link is MATCHED when its stated lane count of one road's lanes run its whole length its way; a link the builder laid
// differently (a turn pocket, a lane drop, a ramp merging) keeps the old offsets, and is counted. Pure: no sim, no world
// -- built once at load and read by refreshPose.

#include "../../engine/ai/nav_graph.h"
#include "../../engine/components.h"   // TrafficLane, StatedLanes
#include "../../rt_math.h"

#include <map>
#include <utility>
#include <vector>

namespace citysim {

struct LinkLane {
    std::vector<engine::Vec2> xy;   // the lane's centreline over the link, from its `from` end to its `to` end
    std::vector<engine::Real> t;    // each point's place along the link, 0..1 (by projection onto the link)
    engine::Real w = 3.5;
    int road = -1;   // the lane's road (engine::LevelLanes::roads index)
};

class LaneMap {
public:
    // Match every drivable link of `nav` to `lanes` (travel lanes only: connectors are not on any link). `roads`: each
    // road's centre line and `stated`: its travel lanes as its road file states them, both indexed as TrafficLane::parent
    // (engine::LevelLanes::roads, ::stated); a link on a road `stated` covers is matched against that count, and
    // statedLanes says it.
    void build(const engine::NavGraph& nav, const std::vector<engine::TrafficLane>& lanes,
               const std::vector<std::vector<engine::Vec2>>& roads = {}, const std::vector<engine::StatedLanes>& stated = {});

    bool has(int link) const {
        return link >= 0 && static_cast<std::size_t>(link) < links_.size() && !links_[static_cast<std::size_t>(link)].empty();
    }
    const std::vector<LinkLane>& lanesOf(int link) const { return links_[static_cast<std::size_t>(link)]; }
    // The point on the link at `t` (0..1 along it) in fractional lane `laneF` (0 = offside): between the two lanes
    // either side, past the last lane by its spacing (the freeway's shoulder lane).
    engine::Vec2 point(int link, engine::Real t, engine::Real laneF) const;

    // The road under `link` (engine::LevelLanes::roads index; -1: no lane at its middle), the way the link runs along it
    // (+1 its spine's way, -1 against; 0 none), and the lanes its road file states that way (-1: not known).
    int roadOf(int link) const { return at(roadOf_, link, -1); }
    int wayOf(int link) const { return at(wayOf_, link, 0); }
    int statedLanes(int link) const { return at(stated_, link, -1); }
    // THE NAV'S LANE COUNTS FROM THE ROAD FILE: every link whose road and way are known gets its stated lanes. Returns
    // how many links it set.
    int stateLanes(engine::NavGraph& nav) const;

    // How each link came out: a footpath (not drivable), no lane found, its road's lanes starting or ending along it,
    // another count of them than it drives, or matched.
    enum class Outcome : uint8_t { NotDrivable, None, Partial, CountDiffers, Matched };
    Outcome outcomeOf(int link) const {
        return link >= 0 && static_cast<std::size_t>(link) < outcome_.size() ? outcome_[static_cast<std::size_t>(link)] : Outcome::NotDrivable;
    }

    // What the match found, over the drivable links.
    int drivable = 0, matched = 0, countDiffers = 0, partial = 0, none = 0;
    int resolved = 0;      // links whose road (and so its stated lanes) is known
    // (the sim's lane count, the lanes found) -> links: where the two disagree
    std::map<std::pair<int, int>, int> counts;

private:
    static int at(const std::vector<int>& v, int link, int none) {
        return link >= 0 && static_cast<std::size_t>(link) < v.size() ? v[static_cast<std::size_t>(link)] : none;
    }
    std::vector<std::vector<LinkLane>> links_;
    std::vector<int> roadOf_, wayOf_, stated_;
    std::vector<Outcome> outcome_;
};

}  // namespace citysim

#endif
