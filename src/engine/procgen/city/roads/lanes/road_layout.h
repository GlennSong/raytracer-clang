#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_LAYOUT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_LAYOUT_H

// THE ROAD FILE'S LAYOUT: its junctions and streets, with ids (docs/road-model-design.md step 1, ADR-0417). The planned
// part of the one road model: where roads meet (a JUNCTION: its arms in order round it, each arm's street and heading,
// which pairs of arms are one street carrying on through it) and the stretches of road between them (a STREET: one
// road-file edge from one junction to the next, or to a dead end). Written into the road file's one "junctions" list
// and its "streets" list (road_graph_spec.h PlannedJunction, PlannedStreet).
//
// Words used here:
//   - a NODE is a place where road ends meet, a ramp's free end on a street's side, or two roads crossing at grade; a
//     JUNCTION is one node or several nodes so close together along a road that the builder makes them one junction
//     (control_plan.h findJunctions: the control plan's own grouping, used here so a file has ONE list of junctions);
//   - an ARM is one road's way out of a junction; its HEADING is the way it leaves, in degrees in plan coordinates
//     (counter-clockwise from +x), measured over the first kArmHeadingReach metres of the road;
//   - two arms are THROUGH (one street carrying on) at a junction of two arms always, and at a bigger junction where they
//     leave within kStraightOnDegrees (lane_graph_connect.h: the one "straight on") of opposite ways, the most nearly
//     opposite pairs first, each arm in one pair at most.
//
// WHO STATES IT. A city the planner makes carries its layout from birth: planToLanesScene and the island scene writer
// call deriveLayout(LayoutSource::Planner) on the scene they made. Its road ends meet where the planner put them at one
// point (it writes points to the centimetre, kPlannerSameEnd), ramps end where their anchors say, and two roads cross at
// grade where its crossing policy says (planCrossings: the scene's own "crossings" list once city_plan's design step has
// written it, the same policy before). A road file made before this has none: `lanes_tool layout --write` derives it ONCE
// with today's inference (LayoutSource::Inferred: ends within the graph's endpoint tolerance, half a metre, and the
// planner's crossing policy where the file declares no crossings -- for a hand-made scene that policy is a guess about
// what the author meant) and writes it into the file for a person to read and correct. A file whose "junctions" list
// already exists (the control plan's) keeps that list as stated: the layout adds each arm's street and heading and the
// through pairs to it. For a planned city the road file holds every fact the planner had, so the two derivations differ
// only in how near two road ends must be to meet; a test holds them to the same layout.
//
// THE BAKE KEY IGNORES IT. The city's bake key hashes the road file's bytes (city_producer.cpp). stripLayoutText is the
// file with the layout's own members taken out, byte for byte what it was before they were written in; the key hashes
// that, so writing a layout into a file re-bakes nothing (and a later edit to the layout re-bakes nothing either: no
// stage of the bake reads it).

#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/street_names.h"   // StreetNameBook: the planner's name generator

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

enum class LayoutSource : uint8_t { Planner, Inferred };

// The planner writes every point to the centimetre: two road ends it made from one point are within this of each other.
constexpr double kPlannerSameEnd = 0.01;
// An arm's heading is measured over this much of its road (metres): enough to read past the curve a junction's corner
// puts on a road's last metres.
constexpr double kArmHeadingReach = 15.0;
// A street shorter than this is no street: the stretch between two arms of one junction, or a road's last half metre.
constexpr double kShortestStreet = 0.5;

struct RoadLayout {
    std::vector<PlannedJunction> junctions;   // in id order
    std::vector<PlannedStreet> streets;       // in road order, then along each road
    std::vector<DividedPair> divided;         // the divided roads' carriageway pairs (the graph's own list, or found)
    bool keptList = false;                    // the junctions are the graph's own list, as it stated them
    bool armsInRing = false;                  // every junction's arms listed in ring order (found here, or a marked list kept)
    std::vector<std::string> notes;           // what could not be settled, in words (an arm with no street, ...)
};

// THE DIVIDED ROADS OF A FILE THAT STATES NONE (ADR-0419), from what it does state: two roads whose ids are one name with
// "_in" and "_out" (the planner's own naming of a boulevard pair, plan_scene.cpp), of one class, each one-way. A road
// whose id ends so but has no such partner is named in `unpaired`, with why.
std::vector<DividedPair> dividedFromIds(const RoadLabGraph& g, std::vector<std::string>* unpaired = nullptr);

// THE LAYOUT OF A ROAD GRAPH (see the top of this file). `g` needs its spines resolved (fromJson), not its heights.
RoadLayout deriveLayout(const RoadLabGraph& g, LayoutSource source);
// The junctions deriveLayout finds when the graph lists none (control_plan.h findJunctions, with the source's end
// tolerance, the graph's at-grade crossings and its divided roads -- found from its ids when it states none and the
// source is Inferred), with their facts and nodes; `carryOns` (optional, out): where a street only carries on.
struct JunctionFacts;
struct CarryOn;
std::vector<JunctionFacts> layoutJunctionFacts(const RoadLabGraph& g, LayoutSource source, std::vector<CarryOn>* carryOns = nullptr);
// THE CONTROLS ON THE LAYOUT'S OWN JUNCTIONS (ADR-0422): control_plan.h planControls, its junctions found as the layout
// finds them (`source`'s end tolerance, the graph's or the planner's crossings, the divided pairs), so a plan written
// into a scene whose layout is written already (writeControlPlan) names the same junctions its streets do.
struct ControlPlan;
struct ControlPolicy;
ControlPlan planLayoutControls(const RoadLabGraph& g, LayoutSource source, const ControlPolicy& policy);
// In words: counts by number of arms, streets, dead ends, the notes.
std::string describeLayout(const RoadLayout& layout);

// WRITES IT INTO A SCENE'S JSON: the "junctions" list replaced by the layout's (a junction's control-plan members kept
// as the layout carries them) and the "streets" list set; "arms_in_ring": true where the layout's arms are in their ring
// (armsInRing, ADR-0427). For a scene the planner is making.
void writeLayout(nlohmann::json& scene, const RoadLayout& layout);

// WRITES IT INTO A ROAD FILE'S TEXT IN PLACE: nothing else in the file moves by a byte. A file with no "junctions" list
// gets one (with "arms_in_ring", and a "streets" list) after its last member; a file whose list exists (the control plan's, whose junctions
// the layout kept: keptList) gets each arm's "street" and "heading" and each junction's "through" written into it. The
// result, stripped (stripLayoutText), is the text it was made from: checked here, and an empty string (with `err`) when it
// is not, or when the layout does not fit the file's list.
std::string insertLayoutText(const std::string& text, const RoadLayout& layout, std::string* err = nullptr);
// THE FILE WITHOUT ITS LAYOUT: the "streets" list taken out; a "junctions" list naming no control taken out whole (with
// the "arms_in_ring" mark); in one
// that names controls (the control plan's), each junction's "through" and each arm's "street" and "heading". Each member
// goes with the comma before it (or, the first of its object, the one after it), so text the layout was inserted into
// comes back byte for byte. Text with none of them is returned as it was.
std::string stripLayoutText(const std::string& text);
// WHAT THE CITY'S BAKE KEY HASHES (city_producer.cpp): the file with its "streets", "divided", "junctions",
// "arms_in_ring" and "lane_rules" members taken out whole -- the junctions with their controls and signs too (ADR-0421: no stage of the
// bake reads any of them; the lane graph, which reads them, keys them itself, laneLevelKey, ADR-0426).
std::string keyedRoadFileText(const std::string& text);
// WHAT THE LANE LEVEL READS OF A ROAD FILE THAT THE BAKE KEY LEAVES OUT (ADR-0426), hashed: its "junctions" (without
// their points, "nodes", which only the street map reads), "arms_in_ring", "streets", "divided" and "lane_rules"
// members, as dumped; 0
// for a file with none. Mixed into the lane graph's cache key.
uint64_t laneLevelKey(const nlohmann::json& spec);
// WRITES THE LANE LEVEL'S RULES INTO A ROAD FILE'S TEXT IN PLACE (ADR-0426): a "lane_rules" member after its last one,
// {"solid_before_stop_line": metres}. Nothing else moves; the bake key does not see it (checked). Empty (with `err`)
// when the file states them already.
std::string insertLaneRulesText(const std::string& text, double solidBeforeStopLine, std::string* err = nullptr);

// ---- the streets' names (ADR-0420) --------------------------------------------------------------------------------------
// A RUN is the streets that are one street carrying on, as the file states it: the two streets of a junction's `through`
// pair, a street and the one it `carries_on` into, and every street of a divided road's two carriageways. Only streets of
// a class that carries a name take part (`named`, per street); a run never passes through one that does not (a freeway
// that carries on as a boulevard). Runs in order of their first street; each run's streets in layout order.
std::vector<std::vector<std::size_t>> streetRuns(const std::vector<PlannedJunction>& junctions, const std::vector<PlannedStreet>& streets,
                                                 const std::vector<DividedPair>& divided, const std::vector<char>& named);
// Whether a street of graph `g` carries a name: its road's class is not one the book leaves unnamed, nor a track or a
// footway, and it has lanes.
std::vector<char> namedStreets(const RoadLabGraph& g, const std::vector<PlannedStreet>& streets, const StreetNameBook& book);
// A name a file already states, on a stretch of a road (what a re-derived layout keeps: the street of that road whose
// middle lies in the stretch takes it).
struct NamedStretch {
    std::string road;
    double from = 0, to = 0;
    std::string name;
};
std::vector<NamedStretch> namedStretches(const std::vector<PlannedStreet>& streets, const std::string& roadPrefix = std::string());
// NAMES EVERY RUN: a run with a street under a kept name takes that name (its longest such street's); every other run
// gets a new one from the book (street_names.h nameRuns: by the run's length, its widest carriageway -- a divided road's
// two together -- and the class of its longest street), never one already kept. Writes `name` into each street of each
// run (and clears it on a street that carries none).
struct NamingReport {
    uint32_t runs = 0, kept = 0, generated = 0, streets = 0;
};
NamingReport nameStreetRuns(std::vector<PlannedStreet>& streets, const std::vector<PlannedJunction>& junctions,
                            const std::vector<DividedPair>& divided, const RoadLabGraph& g, const StreetNameBook& book,
                            const std::vector<NamedStretch>& keep = {});
// The same for a layout, with the shipped book.
NamingReport nameLayout(RoadLayout& layout, const RoadLabGraph& g, const std::vector<NamedStretch>& keep = {});
// WRITES THE STREETS' NAMES INTO A ROAD FILE'S TEXT IN PLACE: a "name" in each street object of its "streets" list that has
// none, from `streets` (the file's own list, in its order). Nothing else moves; stripped, the text is as it was (the bake
// key does not see it). Empty (with `err`) when the list does not fit.
std::string insertStreetNamesText(const std::string& text, const std::vector<PlannedStreet>& streets, std::string* err = nullptr);
// WRITES EACH JUNCTION'S CONTROL INTO A ROAD FILE'S TEXT IN PLACE (ADR-0421): a "control" in each junction object of its
// "junctions" list that has none, from `controls` (each junction's id and control, in the list's order; empty: none).
// Nothing else moves; the bake key does not see it (keyedRoadFileText). Empty (with `err`) when the list does not fit.
std::string insertJunctionControlsText(const std::string& text, const std::vector<std::pair<std::string, std::string>>& controls,
                                       std::string* err = nullptr);

// ---- a junction's points (ADR-0425) -------------------------------------------------------------------------------------
// A junction the layout spread over several points (it has stretches of road inside it) states which roads meet at each
// (road_graph_spec.h PlannedNode): what the street map joins its arms and inside stretches by. The planner writes them as
// it lays out a city (deriveLayout, planControls). A file whose layout was written before them gets them ONCE from the
// one-off tool (`lanes_tool layout <file> --nodes --write`): the nodes as the layout finds them (layoutJunctionFacts:
// today's inference, `source`'s end tolerance), each placed in the one junction of the file that holds all its points (an
// arm there, or an inside stretch running past it, to the centimetre). `notes`: a node no junction holds, an arm at no
// node or at two, an inside stretch's end at no node.
struct JunctionNodes {
    std::string junction;
    std::vector<PlannedNode> nodes;
};
std::vector<JunctionNodes> junctionNodes(const RoadLabGraph& g, LayoutSource source, std::vector<std::string>* notes = nullptr);
// WRITES THEM INTO A ROAD FILE'S TEXT IN PLACE: a "nodes" member in each listed junction that has none and has nodes (the
// file's own list, `g` read from it). Nothing else moves; the bake key (keyedRoadFileText) and the lane graph's key
// (laneLevelKey) do not see it. Empty (with `err`) when the list does not fit.
std::string insertJunctionNodesText(const std::string& text, const RoadLabGraph& g, const std::vector<JunctionNodes>& nodes,
                                    std::string* err = nullptr);

// ---- the order of a junction's arms round it (ADR-0427) -----------------------------------------------------------------
// A JUNCTION'S ARMS ARE STATED IN RING ORDER: its `arms` list goes round it COUNTER-CLOCKWISE in plan coordinates (the
// way a heading in degrees grows), and the file says so with its top-level "arms_in_ring": true (RoadLabGraph::armsInRing).
// Which way the kerb side lies from an arm is the one statement lane_graph_build.h kerbPerpSign (lane_turns.h
// kerbSideCounterClockwise). Every reader -- the turns (lane_turns.h), the regions the streets enclose (road_model.h
// enclosedRegions), the junction's outline at the lane level, the street signs -- TRUSTS a marked file's list as it is:
// nothing sorts arms by heading or by angle, and nothing measures the ring again at load. A planner may state a ring the
// measure below would not give; the measure only checks (checkArmRings, `lanes_tool layout <file> --arm-order`).
//
// THE ONE MEASURE (armRing), used by the writers (deriveLayout, writeControlPlan, insertArmOrderText) and the check, never
// at load. It is taken on the graph with its ramps composed (ComposedRoads: an arm's `at` is a station along the composed
// spine). Each arm's MOUTH is the point at its station on its road's spine. Each arm is measured by a direction from the
// junction's middle (its stated `at`):
//   - at a junction over one point (every mouth within kArmRingSpreadMouths of every other), the arm's heading (stated,
//     else the road's way out over its first kArmHeadingReach): the mouths all stand at the middle, so only the way out
//     tells them apart;
//   - at a junction spread over several points, the point where the arm's road leaves the circle round the middle that
//     holds every mouth and kArmRingSpreadMargin more: the mouths stand apart, and an arm's heading at its mouth can point
//     across another arm (a road leaving its point sideways); where it leaves the junction's area is where its traffic
//     comes from and goes to.
// Arms measuring exactly the same keep their listed order, and the measure says so in a note (as it does when a road never
// leaves the circle -- measured by its far end -- or has no spine beyond its mouth -- measured by its heading).
// 1 m: the layout writes stations to the centimetre (written), so the mouths of one point agree within centimetres, while
// the points of a spread junction lie metres apart (a divided road's two carriageways, a skewed crossing's corners); a
// metre lies between the two.
constexpr double kArmRingSpreadMouths = 1.0;   // metres: mouths farther apart than this make a junction spread
// 5 m: about a lane and a half past the farthest mouth -- far enough that every arm's road has left the stretch the
// junction's other arms share, near enough that a road bending away soon after its mouth is still measured by the way it
// leaves, not where it ends up.
constexpr double kArmRingSpreadMargin = 5.0;   // metres beyond the farthest mouth: the circle a spread junction's arms leave
constexpr double kArmRingAtMiddle = 1e-9;      // metres: a point this near the middle has no direction from it
constexpr double kArmRingNoLength2 = 1e-12;    // square metres: a spine segment this short has no length to cross the circle on

// A ROAD GRAPH WITH ITS RAMPS COMPOSED from their anchors (lane_expand.h expand), as the builder, the loader and the layout
// compose it: the only graph an arm's station means what it says on, so the only one the ring is measured on. Made from
// the graph as read (fromJson); a copy.
class ComposedRoads {
public:
    explicit ComposedRoads(const RoadLabGraph& asRead);
    const RoadLabGraph& graph() const { return g_; }

private:
    RoadLabGraph g_;
};
// The arms of `pj` in ring order, by the measure above: indices into pj.arms. `notes` (optional): each arm measured
// otherwise than the rule says, and each pair measuring exactly the same, in words.
std::vector<std::size_t> armRing(const PlannedJunction& pj, const ComposedRoads& roads, std::vector<std::string>* notes = nullptr);
// Puts pj's arms in ring order (armRing), its `through` pairs renumbered to follow.
void orderArmsRound(PlannedJunction& pj, const ComposedRoads& roads, std::vector<std::string>* notes = nullptr);
// Whether pj's stated list, read as a ring (any arm first), differs from the measure's.
bool ringDiffers(const PlannedJunction& pj, const ComposedRoads& roads, std::vector<std::string>* notes = nullptr);
// THE CHECK (never a gate): the junctions of a graph as read whose stated ring differs from the measure (ids, in list
// order), and the measure's notes.
struct ArmRingCheck {
    std::vector<std::string> differ;
    std::vector<std::string> notes;
};
ArmRingCheck checkArmRings(const RoadLabGraph& asRead);
// WRITES THE RING ORDER INTO A ROAD FILE'S TEXT IN PLACE (the one-off layout tool, `lanes_tool layout <file> --arm-order
// --write`): each listed junction's `arms` put in ring order (the measure, on the file's graph composed), their texts
// moved whole, its `through` pairs renumbered, and the file marked "arms_in_ring": true. Nothing else moves (the bake key,
// which leaves the junctions and the mark out, does not see it; the lane graph's key does). `changed` (optional): the ids
// of the junctions whose order changed; `notes` (optional): the measure's. Empty (with `err`) when the list does not fit.
std::string insertArmOrderText(const std::string& text, const RoadLabGraph& asRead, std::vector<std::string>* changed = nullptr,
                               std::string* err = nullptr, std::vector<std::string>* notes = nullptr);
// The command that writes the ring order into a road file, for messages that ask for it.
constexpr const char* kArmOrderCommand = "lanes_tool layout <road file> --arm-order --write";

}  // namespace roads::lanes
}  // namespace engine

#endif
