#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_CONTROL_PLAN_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_CONTROL_PLAN_H

// THE CONTROL PLAN (issue #240, task B4; driving-model-design.md 5.1 and 6.1): which junction gets traffic lights, an
// all-way stop, stop or give-way signs on its minor roads, or nothing, and what speed limit each road posts -- decided
// by the city planner from a PLACEMENT POLICY that is rule-set data, and written into the road graph as intent
// (road_graph_spec.h PlannedJunction, EdgeSpec::speedLimit). The lane graph builder carries the plan through and checks
// it; it never works a control out for itself ("roads: no guessing").
//
// Words used here:
//   - a NODE is a place where roads meet in the road graph: road ends at one point, or a ramp's free end on the side of
//     a street (a ramp whose `from` or `to` names the street it ends on);
//   - a JUNCTION, as planned, is one node, or several nodes so close together along a road that the builder will make
//     them one junction (streets meeting a few metres apart); the planner groups them by the same figures the builder
//     uses (each road's half-width, the stop line's setback, the merge gap: deck_mesh.h), so one junction gets one
//     control. The builder checks the grouping: a built junction holding two planned ones of different controls is
//     reported, never settled by picking one;
//   - an ARM is a road's way into a junction (road_graph_spec.h PlannedArm); an APPROACH is an arm with lanes arriving;
//   - the MAJOR approaches are those on the most important class of road there (the policy's `class_order`); where every
//     approach is of one class and exactly two of them run straight through (a T), those two are the major ones; every
//     other approach is MINOR. Where no approach is minor (a crossroads of equals), there is no minor road, and the
//     "minor" facts (class, lanes, limit) are read over every approach instead, so a row asking for a minor class of
//     `local` holds for a crossroads of local streets.
//
// THE POLICY (ControlPolicy, read from the rule set's `placement` table by apps/driving/scripting/rule_set_lua.h) is an
// ordered list of ROWS. Each row is a set of conditions on the junction's FACTS -- how many arms and approaches, the
// classes of the major and minor roads, the most lanes arriving on one major and one minor approach, their posted
// limits, whether a ramp ends there -- and a decision: the control, and the regulations (signs) to stand where the major
// and the minor approaches arrive. Optionally a row only applies where no junction already given its control lies
// within `apart` metres (traffic lights spaced along a road). The FIRST row that holds decides; the policy's reader
// insists the last row holds for every junction, so none is left without a control. Junctions are decided most
// important first (by their major road, then their minor road), so where `apart` matters the busier junction is
// served first. No class, control or regulation is named here: they are the policy's words, compared as text.
//
// Same plan every run: nodes and junctions are found in road order, sorted by fixed keys, never through an unordered
// container.
//
// Units: metres, metres per second.

#include "engine/procgen/city/roads/lanes/road_graph_spec.h"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace engine {
namespace roads::lanes {

// A RANGE a fact must lie in for a row to hold. Not set: any value. Set: the fact must have a value (a junction with no
// minor road has no minor-road facts) within [lo, hi].
struct FactRange {
    bool set = false;
    double lo = -std::numeric_limits<double>::infinity();
    double hi = std::numeric_limits<double>::infinity();
    bool holds(bool hasValue, double value) const;
};

// A YES/NO CONDITION: not asked, must be yes, must be no.
enum class YesNo : uint8_t { Any, Yes, No };

// ONE ROW OF THE POLICY.
struct PolicyRow {
    std::string name;                 // what the row is for, in words: written into the plan as the junction's `why`
    // The conditions; each left at its default asks nothing.
    FactRange arms, approaches;
    std::vector<std::string> major, minor;   // the major / minor road's class is one of these (empty: any)
    YesNo hasMinor = YesNo::Any;
    YesNo rampEnd = YesNo::Any;
    FactRange majorLanes, minorLanes;        // the most lanes arriving on one major / minor approach
    FactRange majorLimit, minorLimit;        // the posted limit on the major / minor road (m/s)
    double apart = 0;                        // > 0: no junction already given `control` within this many metres
    // The decision.
    std::string control;
    std::vector<std::string> majorSigns, minorSigns;   // regulation kinds standing where those approaches arrive
};

// THE PLACEMENT POLICY.
struct ControlPolicy {
    std::string name;                                          // the rule set it came from
    std::vector<std::string> classOrder;                       // road classes, the most important first
    std::vector<std::pair<std::string, double>> speedLimits;   // the limit posted on each class (m/s), by class name
    std::vector<PolicyRow> rows;                               // in written order: the first that holds decides

    // A class's place in classOrder (0 is the most important); classOrder's size for a class not listed.
    uint32_t classRank(const std::string& roadClass) const;
    // The limit posted on a class (m/s), or 0 when the policy posts none there.
    double postedLimit(const std::string& roadClass) const;
};

// ONE ARM, as the policy sees it.
struct ArmFacts {
    std::string road, roadClass;
    double at = 0;              // station along the road's spine
    bool spineIn = true;        // the road's spine runs into the junction here
    Vec2 heading;               // unit: the way traffic arriving by this arm is going as it reaches the junction
    int arriving = 0;           // lanes arriving by it
    int leaving = 0;            // lanes leaving by it
    double limit = 0;           // the limit posted on the road (m/s), 0 when none
    bool ramp = false;          // a ramp whose free end is in this junction
    bool major = false;         // settled by settleRoles
};

// ONE JUNCTION, as the policy sees it. Fill `arms` (and rampEnd), then settleRoles works out the rest.
struct JunctionFacts {
    std::string id;
    Vec2 at;
    std::vector<ArmFacts> arms;
    std::vector<PlannedStretch> inside;   // stretches of road lying between nodes grouped into it
    std::vector<Vec2> nodes;              // where its nodes are (the road-end points grouped into it), in node order
    bool rampEnd = false;              // a ramp's free end is in it
    // Worked out by settleRoles. The minor facts are over the minor approaches, or over every approach when no approach
    // is minor (minorClass is then the least important class there, which is the major one).
    int approaches = 0;
    bool hasMinor = false;
    std::string majorClass, minorClass;   // empty when nothing arrives
    int majorLanes = 0, minorLanes = 0;
    double majorLimit = 0, minorLimit = 0;   // 0: none posted
};

// WHICH APPROACHES ARE MAJOR, and the facts that follow (see the top of this file). Two approaches run straight
// through when they arrive within kStraightOnDegrees (lane_graph_connect.h) of head on.
void settleRoles(JunctionFacts& j, const ControlPolicy& policy);
// The facts in words, one line: "4 arms (4 approaches): major arterial (3 lanes, 15.6 m/s), minor local (1 lane, 11.2
// m/s)".
std::string describeFacts(const JunctionFacts& j);

// A JUNCTION ALREADY DECIDED, for rows that keep their control apart.
struct Decided {
    Vec2 at;
    std::string control;
};
// THE DECISION for one junction: the index of the row that held (-1: none did), its control and its name.
struct ControlChoice {
    int row = -1;
    std::string control, why;
};
// The first row of `policy` that holds for `j` (whose roles are settled), given the junctions decided before it.
ControlChoice chooseControl(const JunctionFacts& j, const ControlPolicy& policy, const std::vector<Decided>& before);

// HOW THE NODES ARE FOUND. The control plan's default: road ends within the graph's endpoint tolerance are one node. The
// road model's layout (road_layout.h) also asks for a tighter tolerance (the planner's: the ends it made from one point)
// and for the places two roads cars drive cross AT GRADE away from any shared end, which the caller names (the graph's
// declared crossings, or the planner's crossing policy): each is a node of both roads, at their stations there.
//
// An arm needs road beyond the junction: a node this close to a road's end has nothing of it on that side (metres).
constexpr double kArmReachesPast = 0.5;

// A PLACE WHERE A STREET ONLY CARRIES ON (ADR-0419): a node of two arms -- two road ends meeting with nothing else there,
// one road running on as another, or a bend -- is no junction: no area, no stop line, never grouped with one. The caller
// may ask for them (`carryOns`).
struct CarryOn {
    Vec2 at;
    std::array<std::string, 2> roads;     // the two roads
    std::array<double, 2> stations{};     // each one's station there (its start, 0, or its end)
};
struct JunctionFinding {
    double endTolerance = -1;                              // < 0: the graph's rules.endpointTol
    const std::vector<CrossingSpec>* atGrade = nullptr;    // crossings of kind "at_grade" to make nodes of (others skipped)
    const std::vector<DividedPair>* divided = nullptr;     // the divided roads' carriageway pairs; null: the graph's own list
    std::vector<CarryOn>* carryOns = nullptr;              // out: the places where a street only carries on
};

// THE NODES AND JUNCTIONS OF A ROAD GRAPH, with their facts (roles settled), in id order ("j0", "j1", ...: by their
// first node, nodes in road order). Roads cars do not drive (tracks, footways, roads with no lanes) take no part. The
// ramps' spines are composed from their anchors first, as the builder composes them, so stations agree with its.
//
// WHICH NODES ARE ONE JUNCTION (ADR-0419), the one rule: two nodes next to each other along a road are one junction when
// the stretch of that road between their stop lines is shorter than the graph's storage length (Rules::storage, one car
// and its gap): not even a car could wait between them. Along road `via` a node's stop line stands back from it by the
// widest other road there (half its width: the area the crossing traffic needs) and kStopLineBack. And by construction:
// where the two carriageways of a divided road (`divided`) meet the same street, their nodes, next to each other along it,
// are one junction however far apart. Nothing else groups them.
std::vector<JunctionFacts> findJunctions(const RoadLabGraph& g, const ControlPolicy& policy,
                                         const JunctionFinding& how = JunctionFinding());

// THE WHOLE PLAN for a road graph.
struct ControlPlan {
    std::vector<PlannedJunction> junctions;                 // in id order; hand-written ones kept first, as they were
    std::vector<JunctionFacts> facts;                       // the planned (not hand-written) ones' facts, same order
    std::vector<std::pair<std::string, double>> limits;     // per road (in road order): the posted limit, 0 none
    std::vector<std::string> problems;                      // a junction no row decided (a policy fault), and so on
    std::vector<std::pair<std::string, uint32_t>> byControl;   // junctions per control, by name
    std::vector<std::pair<std::string, uint32_t>> byRow;       // junctions per row, in the policy's order
    std::vector<std::pair<std::string, uint32_t>> signs;       // approaches signed, per regulation kind, by name
    uint32_t handKept = 0;                                  // hand-written junctions kept as they were
    // WHERE A STREET ONLY CARRIES ON (ADR-0419: no junction) the road builder may still lay lanes through a junction of
    // its own until it builds the stated ones (design step 9): the control the policy gives a place of two arms there,
    // the same at every such place (written as the graph's "carry_on_control"; empty: the graph has none)
    std::string carryOnControl;
    uint32_t carryOns = 0;                                  // places where a street only carries on
};
// Plans `g` with `policy`: every junction the graph's nodes make, decided most important first; hand-written junctions
// (PlannedJunction::hand) kept, and any junction sharing a road end with one is left to it; every road's posted limit
// from the policy (a road whose limit is hand-set keeps it); the places where a street only carries on, one control.
ControlPlan planControls(const RoadLabGraph& g, const ControlPolicy& policy);
// WRITES THE PLAN INTO A ROAD GRAPH'S JSON: its "junctions" list replaced, and each road's "speed_limit" set (or
// removed where none is posted), leaving everything else as it was. `scene` must be the graph `plan` was made from.
void writeControlPlan(nlohmann::json& scene, const ControlPlan& plan);
// The plan's counts in words, a few lines, and one line per problem.
std::string describeControlPlan(const ControlPlan& plan);

}  // namespace roads::lanes
}  // namespace engine

#endif
