#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_LEVEL_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_LEVEL_H

// THE LANE LEVEL, A TABLE BUILT FROM THE ROAD MODEL (docs/road-model-design.md step 8, ADR-0426): the lane graph the new
// driving drives on (lane_graph.h), filled from the road model's lanes, connections and turns by id. Design section 5:
// one network at two levels of detail -- a lane belongs to a street, so nothing is matched or guessed between the
// levels. Section 4: a table built from the model holds no fact of its own: every path carries its street and lane (a
// connector its stated turn), every junction its model junction.
//
// WHAT COMES FROM WHERE (nothing is found by a distance or an angle but the measurements the design keeps -- where two
// paths' bodies could touch, how tightly each path bends -- and the corner-fit test, which reports):
//   - a LANE PATH is a stretch of one of the builder's lanes along one street: the lane's curve between stations of its
//     road's spine (each point the builder laid stands beside a stated spine station), from the leaving arm's stop line
//     to the arriving arm's (or a street's end where it carries on, or the lane's own end);
//   - a STOP LINE is the builder's kept stop-line station of the stated arm (the paint's box, kStopLineBack outside it:
//     ModelArm::stopLine), where the paint's box holds that junction alone; else ADR-0419's rule read from the stated
//     cross-sections (control_signs.h stopLineFromMouth): where the paint found no box, or where its box holds two stated
//     junctions (the paint's own "boxes within 12 m are one" until step 9; each such place in the road model's problems,
//     their one home). The STATED junctions are followed, never the paint's grouping;
//   - SECTIONS are the lanes of one road running one way between cuts: where a lane path begins or ends, where a lane
//     leaves or joins its host (the connections' `meets`), and where the solid lines before a stop line begin. Lanes
//     side by side are ordered by their slot (the builder's, from the centre-line side), never by where they lie;
//   - WHAT FOLLOWS A LANE, all stated: the next stretch of the same lane; where a street CARRIES ON as another (no
//     junction: ModelStreet::carriesOn), the lane of the same slot on the next street, whatever the gap between their
//     ends (joined and reported, never a junction); where a lane LEAVES its host (a ramp's exit lane, a turn pocket) the
//     host lane, cut where their lines meet, leads on into it as well as on along itself; where a lane MERGES into its
//     host (a ramp's entry lane, a pocket tapering out) its end leads into the host lane from where their lines meet;
//     at a JUNCTION, a connector for each stated turn (RoadModel::turns);
//   - a CONNECTOR is the builder's addition: the path a stated turn follows, from the arriving lane's end at its stop line
//     to the leaving lane's start, shaped by the existing connector shaping and fitted to the drawn corner for each
//     vehicle class (lane_graph_connect.h fitConnector). A class it does not fit is closed on it and a CORNER FAULT is
//     reported against the turn; a turn that fits no class has no connector (its fault says why) -- it stays stated; a
//     turn with no lane path to start or end on (its lanes stop short of the junction) has none either: a Refusal of its
//     own kind (RefusalWhy::NoLanePath), no corner's fault. Every stated turn is built or refused, checked;
//   - WHAT IS NOT JOINED, SAID: a street's end with no junction and nothing it carries on into, where its road meets
//     another (the design stage's roadEndMeetsAnother; a road that just ends, or a ramp's anchored end, is no problem);
//     a street's end at a junction with no stop line for its arm (nothing joined across it); every stretch of a lane no
//     lane path covers, outside a junction (LaneGraphBuild::laneGaps, with why);
//   - WHICH LINES BETWEEN LANES MAY BE CROSSED: between two through lanes, solid within the road file's
//     `lane_rules.solid_before_stop_line` of the stop line they run to; every other line may be crossed both ways (beside
//     a lane that begins or ends, or of another kind);
//   - CONTROLS, SIGNS AND LIMITS: each junction's stated control; each arm's stated signs at the end of every lane
//     arriving by it; each lane its street's posted limit, each connector the limit of the lane it joins;
//   - the junction's OUTLINE (the area that must not be blocked) runs across each arm's lanes at its stop line, the
//     arms' lines joined in the junction's stated ring order (its arms' list, road_layout.h armRing, ADR-0427: nothing
//     sorted here); an outline that still crosses itself, where two arms' stop lines overlap, is reported (corners for
//     step 9). A road file whose arms are not in ring order gets no lane level (`error` names the command).
// Tidying, in the table only (the builder's lanes are not touched): a lane on the inside of a corner sharper than its
// offset folds back on itself; the fold is cut out (cutFolds) and reported.
//
// WHAT A ROAD FILE MUST STATE: its junctions and streets (`lanes_tool layout <file> --write`), every junction's control
// (`city_plan controls <file> rules/lab.lua --layout --write --controls-only`) and its lane rules (`lanes_tool layout
// <file> --lane-rules 20 --write`). A file lacking any gets no lane level: LaneGraphBuild::error names what to run.

#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/road_model.h"

namespace engine {
namespace roads::lanes {

// THE LANE LEVEL of a built road network, from its road model (assembleRoadModel of the same build): the graph, its
// problems, its corner faults, where each path came from.
LaneGraphBuild buildLaneLevel(const Result& r, const RoadModel& m, const LaneGraphOptions& opts);

}  // namespace roads::lanes
}  // namespace engine

#endif
