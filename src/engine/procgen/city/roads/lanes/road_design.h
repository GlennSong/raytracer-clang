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

}  // namespace roads::lanes
}  // namespace engine

#endif
