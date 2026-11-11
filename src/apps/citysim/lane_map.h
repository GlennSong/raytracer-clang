#ifndef RAYTRACER_APPS_CITYSIM_LANE_MAP_H
#define RAYTRACER_APPS_CITYSIM_LANE_MAP_H

// THE REAL LANES ON THE SIM's LINKS (#218). The sim routes on NavLinks: straight pieces of each road's simplified
// centreline, with a lane count from the road's width. It drew a car at the link's line plus (lane + 0.5) x spacing,
// so where the lanes builder laid its lanes differently (a ramp's own lane, an avenue's median, a one-way's offset,
// a curve the link cuts across) the car sat between the painted lanes. The builder's lanes are published at load
// (engine::LevelLanes); this matches each link to the lanes running its way and keeps, per link, each lane's line over
// the link's span -- offside (by the centre line, lane 0) to kerb, the sim's own lane order (we drive on the LEFT:
// rightOf(direction) is the kerb side).
//
// A link is MATCHED when its lane count of one road's lanes run its whole length its way -- or one more, the kerb-most
// being the lane the sim parks in; a link the builder laid differently (a turn pocket, a lane drop, a ramp merging)
// keeps the old offsets, and is counted. Pure: no sim, no
// world -- built once at load and read by refreshPose.

#include "../../engine/ai/nav_graph.h"
#include "../../engine/components.h"   // TrafficLane
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
    // Match every drivable link of `nav` to `lanes` (travel lanes only: connectors are not on any link).
    void build(const engine::NavGraph& nav, const std::vector<engine::TrafficLane>& lanes);

    bool has(int link) const {
        return link >= 0 && static_cast<std::size_t>(link) < links_.size() && !links_[static_cast<std::size_t>(link)].empty();
    }
    const std::vector<LinkLane>& lanesOf(int link) const { return links_[static_cast<std::size_t>(link)]; }
    // The point on the link at `t` (0..1 along it) in fractional lane `laneF` (0 = offside): between the two lanes
    // either side, past the last lane by its spacing (the freeway's shoulder lane).
    engine::Vec2 point(int link, engine::Real t, engine::Real laneF) const;

    // What the match found, over the drivable links.
    int drivable = 0, matched = 0, countDiffers = 0, partial = 0, none = 0;
    int kerbParking = 0;   // of the matched: one more lane than the sim drives, the kerb-most left to the parking
    // (the sim's lane count, the lanes found) -> links: where the two disagree
    std::map<std::pair<int, int>, int> counts;

private:
    std::vector<std::vector<LinkLane>> links_;
};

}  // namespace citysim

#endif
