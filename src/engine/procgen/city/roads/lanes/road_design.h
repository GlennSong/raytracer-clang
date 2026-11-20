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

// WHETHER ANOTHER ROAD MEETS ROAD `e`'S END (`end` 0: its start, 1: its end): another road ends within the rules'
// endpointTol of it, or its paved band (the two roads' half widths) covers it. The design stage's test for an open end
// (one it brings down to the ground), in plan, as it always was (`atGradeOnly` false). With `atGradeOnly`, a road counts
// only where the crossing policy (meetAtGrade, below) lets the two meet at grade: a street ending under the freeway's
// band meets nothing there, since the freeway meets no road at grade (ADR-0469). The road model asks it so
// (road_model.h streetEndKind) to tell a road that just ends from one that meets others where the road file states no
// junction (reported, never joined).
bool roadEndMeetsAnother(const RoadLabGraph& g, const EdgeSpec& e, int end, bool atGradeOnly = false);

// THE PLANNER'S CROSSING POLICY, its one statement (ADR-0215 step 3; ADR-0469): whether roads `a` and `b` meet AT GRADE
// where they meet (true), or one bridges the other (false), by what the two roads are, from their classes' stated flags:
//   a track and anything        a bridge (there are no level crossings)
//   the freeway and a road      a bridge (a limited-access class meets nothing at grade but a ramp, at its gore)
//   a footway crossing a road   a bridge (a footbridge); a footway ENDING on a road meets it at grade (its way in)
//   any two streets             at grade (a junction: the planner made them meet)
// `aEndsOnB` / `bEndsOnA`: that road's end is where they meet. planCrossings writes it into a plan; roadEndMeetsAnother
// asks it for the road model.
bool meetAtGrade(const RoadLabGraph& g, const EdgeSpec& a, const EdgeSpec& b, bool aEndsOnB, bool bEndsOnA);

// EVERY PLACE TWO OF THE GRAPH'S ROADS MEET away from a shared node -- their centrelines crossing, or one ending on the
// other's interior -- found with the design stage's own geometry, and what the plan means there (the crossing policy,
// meetAtGrade above: "bridge" or "at_grade"; ADR-0215 step 3). A bridge's upper road is left to the design stage, which names it in the designed graph. The result is the graph's
// "crossings" list (CrossingSpec as JSON). `g` needs its spines resolved (fromJson), not its heights.
nlohmann::json planCrossings(const RoadLabGraph& g);

}  // namespace roads::lanes
}  // namespace engine

#endif
