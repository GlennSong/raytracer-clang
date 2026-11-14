#ifndef RAYTRACER_APPS_CITYSIM_LANE_MAP_H
#define RAYTRACER_APPS_CITYSIM_LANE_MAP_H

// THE REAL LANES ON THE SIM's LINKS (#218). The sim routes on NavLinks: straight pieces of each street's centre line. It
// drew a car at the link's line plus (lane + 0.5) x spacing, so where the lanes builder laid its lanes differently (a
// ramp's own lane, an avenue's median, a one-way's offset, a curve the link cuts across) the car sat between the painted
// lanes. The builder's lanes are published at load (engine::LevelLanes); this ties each link to the lanes running its way
// over its stretch of road and keeps, per link, each lane's line over the link's span -- offside (by the centre line,
// lane 0) to kerb, the sim's own lane order (we drive on the LEFT: rightOf(direction) is the kerb side).
//
// HOW A LINK IS TIED TO ITS LANES (ADR-0425: by ids, nothing found by where it lies). The street map (roads/street_map.h)
// gives every link its road (the road file's edge), its city, and the stations along that road's spine at its two ends;
// the builder gives every lane its road, the way it runs (+1 along the spine), its slot across the lanes running that way
// (0 by the centre line), the stretch of spine it is laid along, and each point's station. A link's lanes are its road's
// lanes running its way whose stretch covers the link's, slots 0 to the link's stated lane count less one (a turn pocket
// is a slot past them, or -1, and drives no link). Each lane's line over the link is its own points between the link's two
// stations.
//
// A link is MATCHED when every one of its stated lanes is laid over its whole length; a link where one starts or ends
// along it (a ramp's dovetail, a lane drop) keeps the old offsets and is counted; a link of no road (a hand-made map's,
// a walk) or whose road has no lanes is UNTIED and reported. Pure: no sim, no world -- built once at load and read by
// refreshPose.

#include "../../engine/ai/nav_graph.h"
#include "../../engine/components.h"   // TrafficLane, LevelLanes
#include "../../rt_math.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace citysim {

struct LinkLane {
    std::vector<engine::Vec2> xy;   // the lane's centreline over the link, from its `from` end to its `to` end
    std::vector<engine::Real> t;    // each point's place along the link, 0..1 (by its station along the road)
    engine::Real w = 3.5;
    int road = -1;   // the lane's road (engine::LevelLanes::roads index)
};

class LaneMap {
public:
    // Tie every drivable link of `nav` to the lanes of `lanes` (see the top of this file).
    void build(const engine::NavGraph& nav, const engine::LevelLanes& lanes);

    bool has(int link) const {
        return link >= 0 && static_cast<std::size_t>(link) < links_.size() && !links_[static_cast<std::size_t>(link)].empty();
    }
    const std::vector<LinkLane>& lanesOf(int link) const { return links_[static_cast<std::size_t>(link)]; }
    // The point on the link at `t` (0..1 along it) in fractional lane `laneF` (0 = offside): between the two lanes
    // either side, past the last lane by its spacing (the freeway's shoulder lane).
    engine::Vec2 point(int link, engine::Real t, engine::Real laneF) const;

    // How each link came out: a footpath (not drivable), no lane found, its lanes starting or ending along it, another
    // count of them than it drives, or matched.
    enum class Outcome : uint8_t { NotDrivable, None, Partial, CountDiffers, Matched };
    Outcome outcomeOf(int link) const {
        return link >= 0 && static_cast<std::size_t>(link) < outcome_.size() ? outcome_[static_cast<std::size_t>(link)] : Outcome::NotDrivable;
    }

    // What the match found, over the drivable links.
    int drivable = 0, matched = 0, countDiffers = 0, partial = 0, none = 0;
    // (the link's lane count, the lanes found) -> links: where the two disagree
    std::map<std::pair<int, int>, int> counts;
    // the links the match could not tie, in words (a link of no road, a road with no lanes): a few, then how many
    std::vector<std::string> untied;

private:
    std::vector<std::vector<LinkLane>> links_;
    std::vector<Outcome> outcome_;
};

}  // namespace citysim

#endif
