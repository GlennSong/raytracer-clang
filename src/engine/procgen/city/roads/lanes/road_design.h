#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_DESIGN_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_DESIGN_H

// THE ROAD DESIGN STAGE (Glenn, 2026-10-20: "neither system is guessing ... use the same models"): the vertical design
// of a road graph, in one place so the planner can run exactly what the builder runs --
//   each road's profile at its class's grade over the ground (open ends down to it), a track's holds chained across
//   its joints, ramps from their hosts, heights agreed at nodes and level crossings (iterated to the best round),
//   grade-separated crossings lifted to clear their structure, ramps and floors settled once more at the end.
// It writes each edge's z (and its floors); lanes::build calls it between expanding the lanes and the footprints.

#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/lanes/vertical_profile.h"   // HeightField

#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

struct DesignReport {
    double nodeMismatchBefore = 0, nodeMismatchAfter = 0;   // the worst node or level crossing, before and after agreeing (m)
    std::string nodeMismatchWhere;                          // the worst node, when it is worth naming
    int lifted = 0;                                         // grade-separated crossings lifted to clear their structure
    std::vector<std::string> undeclared;                    // a graph that declares its crossings: those it did not ("a / b at (x, y)")
};

DesignReport designRoads(RoadLabGraph& g, const HeightField& terrain);

// THE PLANNER'S CROSSING POLICY (ADR-0215 step 3): every place two of the graph's roads meet away from a shared node --
// their centrelines crossing, or one ending on the other's interior -- found with the design stage's own geometry, and
// what the plan means there, by what the two roads are:
//   a track and anything        a bridge (there are no level crossings)
//   the freeway and a road      a bridge (nothing meets the freeway at grade; its ramps join it at their gores)
//   a footway crossing a road   a bridge (a footbridge); a footway ENDING on a road meets it at grade (its way in)
//   any two streets             at grade (a junction: the planner made them meet)
// A bridge's upper road is left to the design stage, which names it in the designed graph. The result is the graph's
// "crossings" list (CrossingSpec as JSON). `g` needs its spines resolved (fromJson), not its heights.
nlohmann::json planCrossings(const RoadLabGraph& g);

}  // namespace roads::lanes
}  // namespace engine

#endif
