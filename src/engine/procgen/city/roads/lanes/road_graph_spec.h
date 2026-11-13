#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_GRAPH_SPEC_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_GRAPH_SPEC_H

// lanelab's input: a road graph as data (ADR-0083). Edges are classes + paths; a class's
// lane layout is expressed as engine RoadSpec bands (Travel per direction, Median gap,
// Shoulder, Sidewalk) so the lane model is the engine's. Ramps carry anchors on a host
// edge's outer lane; roads may carry pockets (lanes born out of a neighbour); connectors
// join a lane's end to a point on another lane. Everything downstream reads the resolved
// spines here, never the JSON.
//
// Plan coordinates: the scene files use (x, y); the engine world is (x, height, z). Vec2.x
// is world x and Vec2.y is world z throughout lanelab.

#include "engine/procgen/city/polyline.h"   // stations
#include "engine/procgen/city/polygon.h"      // Vec2
#include "engine/procgen/city/road_spec.h"    // RoadSpec / RoadBand
#include <nlohmann/json.hpp>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// A road's lanes, as its class states them (and a road's own "lanes" may restate). `fwd` and `back` are the TRAVEL
// lanes each way: what the builder lays and what the city sim drives -- one lane count for every reader (ADR-0418).
//
// KERB PARKING (`parking`, ADR-0418): a band this many metres wide along each kerb edge of the carriageway (the paved
// width between the kerbs), where cars park; traffic keeps to the rest. It is not a lane of its own: the builder lays the
// lanes as stated, and the band lies over the kerb side of the kerb lane each way (a planned city's local street: one
// 6 m lane each way, the kerb 2.5 m of it the parking). 0: no parking. A band that would leave a travel lane narrower
// than kMinTravelLane is not built, and the road model says so. streetSection (below) is the one reader.
struct LaneLayout {
    double w = 3.5;          // lane width
    int fwd = 1, back = 0;   // lanes per direction; forward lanes lie on the RIGHT of the spine direction
    double gap = 0.0;        // median between directions (two-way roads)
    double dovetail = 60.0;  // ramps with fwd >= 2: station where the second lane starts dying into the first
    double parking = 0.0;    // kerb parking: the band's width along each kerb (above); 0 none
};

// The narrowest travel lane kerb parking may leave (metres): a band that would leave less is not built.
constexpr double kMinTravelLane = 3.2;

struct PocketSpec {
    std::string id, kind = "turn", side = "right";
    int dir = +1;            // +1 forward lanes, -1 backward lanes
    double s0 = 0, s1 = 0, taper = 0, taperOut = 0;   // taper 0 -> 10 lane widths
};

struct RampAnchor {
    bool set = false;        // an anchor with geometry (station on the host); a bare edge id only supplies height
    std::string edge;        // host edge id
    bool hasStation = false; double s = 0;
    char axis = 0; double coord = 0;   // 'x' or 'y': nearest host vertex to that coordinate; 'p': nearest to (px, py)
    double px = 0, py = 0;
    std::string side = "right";
    double aux = 120, decel = 100, taper = -1, approach = 150;   // taper < 0 -> 30 lane widths
};

struct ConnectorSpec { std::string from, to; double toS = 0; };

// A JUNCTION AS PLANNED: THE CONTROL PLAN (issue #240, driving-model-design.md 5.1; control_plan.h writes it). Which
// control a junction has (traffic lights, an all-way stop, stop or give-way signs on its minor roads, or nothing) and
// which regulations stand where each road arrives (a stop sign), decided by the city planner from the rule set's
// placement policy and written here as INTENT: the lane graph builder carries it through and checks it, never works a
// control out for itself ("roads: no guessing"). Authored at the graph's top level:
//   "junctions": [{"id": "j12", "at": [x, y], "control": "stop", "why": "a local street meets a collector",
//                  "arms": [{"road": "s4", "at": 87.3, "spine": "in", "regulations": ["stop"]}, ...]}]
// An ARM is one road's way into the junction: the road, the station along its spine where it meets the junction, and
// whether its spine runs INto the junction there (its forward lanes arrive) or OUT of it (its backward lanes arrive).
// A road that passes through (a ramp ending on a street's side) is two arms, one each way. A junction the planner
// spread over several nodes (streets meeting so close together that they make one junction) lists only the arms
// that lead out of it, and names the stretches of road lying between its nodes as `inside` ({"road", "from", "to"},
// stations along the road's spine): the builder may still make two junctions there, joined by such a stretch, and
// both take this one's control. `hand`: written by a person, so re-planning keeps it as it is.
//
// THE SAME LIST IS THE ROAD MODEL'S JUNCTIONS (docs/road-model-design.md step 1, ADR-0417): one list of junctions in a
// road file, never two. Besides the control plan's fields, a junction may say which STREET each arm is (`street`, an id
// from the graph's "streets" list), the way the arm leaves the junction (`heading`, degrees in plan coordinates,
// counter-clockwise from +x), and which pairs of its arms are one street carrying on through it (`through`: arm indices).
// A list that carries no control at all is the layout alone: it plans no control, and every junction says so (the lane
// graph builder then gives each junction the options' one control, as a graph with no list always did). A list where
// some junctions name a control and some do not is refused: a control plan that leaves a junction out is the plan's
// fault. The road model's own members (street, heading, through, and the "streets" list) are not part of the city's
// bake key (city_producer.cpp, road_layout.h stripLayoutText): writing them into a file re-bakes nothing.
struct PlannedArm {
    std::string road;
    double at = 0;                          // station along the road's spine
    bool spineIn = true;                    // the spine runs into the junction here
    std::vector<std::string> regulations;   // kinds standing at the end of each lane that arrives by this arm
    std::string street;                     // the street this arm is (PlannedStreet::id); empty: not stated
    bool hasHeading = false;
    double heading = 0;                     // degrees: the way the arm leaves the junction, plan coordinates
};
struct PlannedStretch {
    std::string road;
    double from = 0, to = 0;                // stations along the road's spine, from <= to
};
struct PlannedJunction {
    std::string id, control, why;
    Vec2 at;
    std::vector<PlannedArm> arms;
    std::vector<PlannedStretch> inside;     // the stretches of road lying between its nodes
    std::vector<std::array<int, 2>> through;   // pairs of arms (indices into arms) that are one street carrying on through
    bool hand = false;
};

// A STREET AS PLANNED (road-model-design.md step 1): a stretch of one road-file edge from one junction to the next, or to
// a dead end -- `from` and `to` stations along the edge's spine, and the junction at each end (a PlannedJunction id;
// empty: none -- a dead end, or a ramp's end anchored to a lane of its host, which the ramp itself states). An edge that
// passes through junctions is several streets; the stretches of it inside a junction belong to no street. Authored at the
// graph's top level: "streets": [{"id": "s12/1", "road": "s12", "from": 40.2, "to": 131.7, "junctions": ["j3", "j9"]}].
// WHERE A STREET ONLY CARRIES ON (ADR-0419): two road ends that meet with nothing else there (one road running on as
// another, or a bend) are no junction -- no junction area, no stop line. Each of the two streets names the other at that
// end: "carries_on": [null, "s13"] (at `from`, at `to`; written only where one is set). One street carrying on, in two
// road-file edges.
struct PlannedStreet {
    std::string id, road;
    double from = 0, to = 0;
    std::array<std::string, 2> junctions;   // at `from`, at `to`
    std::array<std::string, 2> carriesOn;   // the street this one carries on into at `from`, at `to` (empty: none)
};

// A DIVIDED ROAD'S TWO CARRIAGEWAYS AS PLANNED (ADR-0419): two one-way road-file edges that are one road, its two ways.
// Where both meet the same street, the planner built them as ONE junction there, however far apart along the street
// their two ends are. Authored at the graph's top level: "divided": [["bv0_in", "bv0_out"]]. Part of the layout: the
// bake key does not see it (road_layout.h stripLayoutText).
using DividedPair = std::array<std::string, 2>;

// A CROSSING AS PLANNED (ADR-0215, Glenn: "the planner and builder should know the intent of the roads so it's not
// guessing"): where two roads cross or one ends on the other away from a shared node, what the plan means there --
//   "bridge"    grade-separated: `over` goes over the other, clearing it by the rules' clearance (or `clearance` m);
//               `over` left empty: the design stage chooses by its own rule, and the designed graph names it
//   "at_grade"  the two meet level: their heights are made to agree there, however far apart they stand
// Authored at the graph's top level: "crossings": [{"a": "link9", "b": "p0_s191", "at": [x, y], "kind": "bridge",
// "over": "link9"}]. A graph that carries the list DECLARES its crossings: one the design stage finds that is not on
// it is a design error, reported with its place, not a kind guessed from the heights. Without the list the stage
// infers the kind as it always did (hand-made lab scenes).
struct CrossingSpec {
    std::string a, b, kind, over;
    Vec2 at;
    double clearance = -1;   // < 0: the rules' under_clearance
};

// What a class builds at a deck edge, by what that edge faces (EdgeRole in deck_mesh.h). Authored per class
// in the graph JSON: "edges": { "median": {"kind": "wall", "h": 1.05}, "at_grade": {"kind": "guardrail"} }.
// kind: "none" | "wall" (solid slab, the parapet) | "guardrail" (beam on posts). Only freeway and ramp
// classes are consulted; a street's edges belong to the lot pass (Glenn, 2026-09-08).
enum class BarrierKind { None = 0, Wall, Guardrail };
// `set` distinguishes "the graph authored none here" from "the graph said nothing": without it a class
// could never switch a default OFF.
// `offset` is how far OUTBOARD of the road's own outline the barrier stands, `thick` its body. Offset 0 hugs
// the pavement edge, which is the only place the lab can guarantee ground under it; push it out only where
// the class's shoulder is really paved.
// minDrop > 0: built only where the ground 5 m out falls at least that far below the deck -- a mountain road's
// guardrail stands where there is somewhere to fall, not along every verge (ADR-0117)
struct BarrierSpec { BarrierKind kind = BarrierKind::None; double h = 0.9, offset = 0.0, thick = 0.4, minDrop = 0.0; bool set = false; };

struct RoadClassSpec {
    std::string name;
    LaneLayout lanes;
    double shoulder = 0, sidewalk = 0, median = 0;   // rings outside the lanes (median: divider axes)
    int rank = 1;
    double gMax = 0.09, window = 40, thick = 0.5;
    // CUT AND FILL (ADR-0116): 0 = the profile only fills (the grade-limited envelope from above, every
    // city's roads); 0.5 = halfway between that and the envelope from below -- a mountain road that cuts
    // through the rise it cannot climb instead of riding a viaduct off it (the island pass was 2.7 km of
    // bridge in 4.2). Both envelopes keep the grade, so any blend does.
    double balance = 0.0;
    // indexed by EdgeRole: seam, median, vs street, elevated, at grade
    std::array<BarrierSpec, 5> edges{};
    // RAIL (Glenn, 2026-10-10: "it should define rail lines and we should probably build them like we build lanes of
    // the freeway"): a TRACK class is a road class whose lanes are tracks -- the same lanes, profiles, decks, bridges
    // and terrain conform, but the deck is a ballast bed with sleepers and rails on each lane's centreline, no paint,
    // and nothing in the road twin (no car, walker or lot ever routes onto it). Its lanes leave as rail tracks.
    bool track = false;
    // A FOOTWAY (Glenn, 2026-10-11, the stations: "make it accessible"): a paved path for people on foot -- a
    // station's way in from the nearest street, ramping up into the footbridge over the tracks. Built like any road
    // (profile, deck, bridge, piers, ground), paved as a sidewalk, never painted, and left out of the road twin: no
    // car routes onto it. An end held by the edge's own floor point stays up (a footbridge ends over a platform).
    // Steeper than kFootwayStairGrade, its deck is built as steps.
    bool footway = false;
};

// A station platform beside a track edge, authored on the edge: "platforms": [{"name": "Central", "s0": 300,
// "s1": 420, "side": "both", "w": 4.5}]. Built on the OUTSIDE of the edge's outermost track on that side, its
// edge `kPlatformGap` off the train body, its top at the height a carriage floor stands (railhead + h).
struct PlatformSpec {
    std::string name;
    double s0 = 0, s1 = 0;    // stations along the edge's spine
    int side = 0;             // 0 both, +1 the spine's left (plan), -1 its right
    double w = 4.5, h = 1.05; // width, and the top above the railhead
};

struct Rules {
    double step = 1.0, sameLevelDz = 1.0, blendLen = 20.0, closing = 0.0, bridgeH = 4.0, pierSpacing = 24.0;
    double slope = 0.5, conformW = 14.0, skirt = 1.6, skirtDrop = 0.15, endpointTol = 0.5;
    double underClearance = 5.0, structureDepth = 1.6;   // free height under a deck's structure over any road it crosses, and the girder depth below the slab
    bool freewaySeparates = false;             // a freeway is never met at grade by a road crossing it (not a ramp): structure at any height. Off by default -- authored scenes (freeway_cross, ring_city) cross streets over freeways at grade -- and on for the island (ADR-0114)
    double rampLevelDz = 1.5;                  // a ramp within this of a street it crosses is at grade with it (the street meets it); above, it is structure (the ramp clears it)
    // STORAGE ("storage", metres; ADR-0419): the street a car needs to wait in between two junctions -- a car's length
    // and the gap behind it (4.5 m + 2 m). Two junction points next to each other along a road whose stretch between
    // their stop lines is shorter than this are ONE junction: not even a car could wait between them. (A longer vehicle
    // that cannot wait there is the driving's to keep out: it checks the room beyond for each vehicle class.)
    double storage = 6.5;
};

constexpr double kFootwayStairGrade = 0.15;

struct TerrainSpec {
    std::string type = "flat", file;
    std::array<double, 4> bounds{0, 0, 0, 0};   // x0, x1, y0, y1
    bool hasBounds = false;
    double res = 2.0; int seed = 1;
    std::vector<std::pair<double, double>> octaves;             // (amplitude, cell)
    struct Valley { bool alongY = false; double c = 0, width = 1, depth = 0; };
    std::vector<Valley> valleys;
    struct Hill { double x = 0, y = 0, r = 1, h = 0; };
    std::vector<Hill> hills;
    bool hasTilt = false; double dzdx = 0, x0 = 0, dzdy = 0, y0 = 0;
    // THE WORLD UNDERNEATH: a level's own `terrain` block, evaluated by the engine's terrainHeight —
    // the same function the level renders outside this grid — with the relief above (octaves,
    // valleys, hills) added on top. Null: no base.
    nlohmann::json base;
    // Where that relief applies: fully within fadeR0 of the fade centre, not at all past fadeR1 —
    // so at the grid's edge the ground IS the level's and the seam cannot show — and calmed
    // toward a coast: fully before calmFrom along (calmDx, calmDy), not at all past calmTo.
    bool hasFade = false; double fadeX = 0, fadeY = 0, fadeR0 = 0, fadeR1 = 0;
    bool hasCalm = false; double calmDx = 1, calmDy = 0, calmFrom = 0, calmTo = 0;
};

// A RAMP'S LANDING AS PLANNED (#282, ADR-0411; interchange.h): how the planner meant the ramp's end on a street to meet
// it -- written into the ramp's "landing" for the builder to check (checkRampLandings), never to work out:
//   "landing": {"street": "s276", "joins": "merge", "carriageway": "backward", "half": "far",
//               "crossing": {"kind": "over", "street": "s340", "at": [x, y], "clearance": 7.6}}
// `joins`: "merge" -- by a lane of its own beside the kerb lane of the street's `carriageway` ("forward": the lanes
// running along the street's spine; "backward": the other way), its end anchored there; "junction" -- as one arm of a
// junction on the street, its end bare. `half`: the half of the street, seen from the freeway, the carriageway is on
// ("near" or "far"). `crossing`: how the ramp gets to that half -- "over" a street (holding `clearance` over it, deck
// to deck, at `at`), or "none". A ramp without one was planned before the rule (or by hand): the builder checks its
// geometry all the same, and only warns.
struct RampLanding {
    bool declared = false;
    std::string street, joins, carriageway, half;
    std::string crossingKind, crossingStreet;   // kind "over" or "none"
    Vec2 crossingAt;
    double clearance = 0;
};

struct RampInfo {
    bool valid = false;
    double departS = 0, touchS = 0, length = 0, climb = 0, freeLen = 0, requiredLen = 0;
    bool ok = true;
};

struct EdgeSpec {
    std::string id, cls;
    nlohmann::json path;                 // kept for ramp recomposition
    RampAnchor from, to;
    bool hasZMin = false; double zMin = 0;
    double lotsFrom = -1, lotsTo = -1;   // station range where the city may build lots along this edge (-1: all of it); the rest is right-of-way
    std::vector<std::array<double, 4>> floorPts;   // (x, y, z, half-span): the profile must clear z within half-span of that station, approached at design grade beyond
    // a TRACK's holds that belong to the section joined to one of its ends (chainTrackFloors): (station, z, half-span)
    // in this edge's stations -- before 0 or past its end -- so a lift's approach runs on across the joint
    std::vector<std::array<double, 3>> chainFloors;
    LaneLayout lanes;
    std::vector<PocketSpec> pockets;
    std::vector<PlatformSpec> platforms;   // track classes only
    // THE PLANNED ELEVATION (ADR-0215): the deck height along the resolved spine, every `profileStep` m from its start,
    // as the design stage made it ("profile": {"step": 1, "z": [...]}). A graph whose roads all carry one is DESIGNED:
    // the builder takes the heights as given and checks them, never re-designs them.
    std::vector<double> profile; double profileStep = 0.0;
    // THE POSTED SPEED LIMIT (issue #240, the control plan; "speed_limit": metres per second), on every lane of the road
    // both ways; 0: none posted (the rule set's default for the road's class holds). `speedLimitHand`
    // ("speed_limit_hand": true): set by a person, so re-planning keeps it.
    double speedLimit = 0.0;
    bool speedLimitHand = false;
    // A FRONTAGE ROAD AS PLANNED (ADR-0413; "frontage_of": the id of the freeway carriageway it runs beside): a one-way
    // road, every lane running the way that carriageway's traffic runs, so a ramp joins or leaves it at grade on its lane
    // nearest the freeway. The builder checks it (checkFrontageRoads); empty: not one.
    std::string frontageOf;
    // resolved geometry and profile
    std::vector<Vec2> xy;                // spine, resampled at rules.step
    std::vector<double> s, z, t;         // station, deck height, terrain height along the spine
    RampInfo ramp;
    RampLanding landing;                 // a ramp's end on a street, as planned (#282)
    // Does anything meet this end? An OPEN end is pinned to the ground by the profile
    // solve: gradeLimit only ever RAISES a profile, so a road that cannot follow its
    // terrain otherwise floats — steep_climb began 65 m in the air over a valley floor
    // it never came down to. Filled by build() before the profiles are solved.
    bool openEnd[2] = {false, false};
    int anchorIdx[2] = {-1, -1};         // spine index of the from-gore and the to-gore, when anchored
    std::map<std::string, std::string> anchorLanes;   // "from"/"to" -> host lane id
    bool isRamp() const { return !from.edge.empty() || !to.edge.empty(); }
    int laneCount() const { return lanes.fwd + lanes.back; }
    double length() const { return s.empty() ? 0.0 : s.back(); }
};

struct RoadLabGraph {
    std::string name = "roads";
    std::map<std::string, RoadClassSpec> classes;
    Rules rules;
    TerrainSpec terrain;
    std::vector<EdgeSpec> edges;
    std::vector<ConnectorSpec> connectors;
    std::vector<CrossingSpec> crossings;  // as planned (see CrossingSpec)
    bool declaresCrossings = false;       // the graph carried a "crossings" list: every crossing is on it
    // THE CONTROL PLAN (see PlannedJunction). A graph whose "junctions" list names controls DECLARES every junction's
    // control: the lane graph builder reports a junction it builds that no planned one names. Without the list, or with a
    // list that names no control (the road model's layout alone), each junction carries the lane graph options' one
    // control, as before.
    std::vector<PlannedJunction> junctions;
    bool declaresJunctions = false;       // the list names each junction's CONTROL (the control plan); see PlannedArm
    bool listsJunctions = false;          // the graph carried a "junctions" list at all, with controls or without
    // WHERE A STREET ONLY CARRIES ON (no junction, ADR-0419) the lane graph builder may still make a junction of its own
    // until the builder builds the stated ones: the control the plan gives it there ("carry_on_control"; control_plan.h)
    std::string carryOnControl;
    // THE DIVIDED ROADS AS PLANNED (DividedPair); `listsDivided`: the graph carried the list.
    std::vector<DividedPair> divided;
    bool listsDivided = false;
    // THE STREETS AS PLANNED (PlannedStreet); `listsStreets`: the graph carried the list.
    std::vector<PlannedStreet> streets;
    bool listsStreets = false;
    bool designed() const;               // every road with a spine carries its planned profile
    std::map<std::string, int> index;    // edge id -> position in edges
    // the planned crossing of edges a and b nearest p (within `within` m), or null
    const CrossingSpec* declaredCrossing(const std::string& a, const std::string& b, const Vec2& p, double within = 12.0) const;

    const EdgeSpec* find(const std::string& id) const;
    EdgeSpec* find(const std::string& id);
    const RoadClassSpec& cls(const EdgeSpec& e) const { return classes.at(e.cls); }
    int rank(const std::string& edgeId) const { return cls(*find(edgeId)).rank; }
    double hw(const EdgeSpec& e) const;      // half-width of the through lanes
    std::array<double, 4> bounds() const;    // terrain bounds, or the edges' extent plus a margin
    std::vector<const EdgeSpec*> paved() const;

    static RoadLabGraph fromJson(const nlohmann::json& spec, const std::string& baseDir = ".");
    static RoadLabGraph load(const std::string& path);
};

// The engine band list for a class: [Sidewalk][Shoulder][Travel dir -1 x back][Median gap][Travel dir +1 x fwd][Shoulder][Sidewalk],
// one-way roads centred. Lane offsets are derived from RoadSpec::bandSpan, so the lane model is the engine's.
RoadSpec bandsFor(const RoadClassSpec& c, const LaneLayout& layout);

// THE STREET'S CROSS-SECTION AS THE CITY READS IT (ADR-0418): the engine band list the road twin hands the city sim --
// the carriageway (the road's lanes and shoulders: 2 x (hw + shoulder)) split into travel bands and, where the road states
// kerb parking that fits (LaneLayout::parking), a Parking band that wide along each kerb, carved out of the carriageway's
// edges (so it lies over the kerb side of the kerb lane; the carriageway's width never moves). The one reader of the
// stated parking: the twin's band (which the sim lays its kerbside bays in) and the road model both come from here.
// `e` is a whole road-file edge.
RoadSpec streetSection(const RoadLabGraph& g, const EdgeSpec& e);
// The kerb parking `e` really has along each kerb: its stated width where it fits, else 0. `fits` (optional): false when
// it states parking that would leave a travel lane narrower than kMinTravelLane.
double kerbParking(const RoadLabGraph& g, const EdgeSpec& e, bool* fits = nullptr);

// (name, left-normal offset from the spine, direction) of each Travel band, from the RoadSpec.
struct LaneSlot { std::string name; double offset; int dir; };
// THE ROAD CLASS TABLE (ADR-0276): every class's section, grade and profile, once -- what a graph's classes start
// from, what the planner writes into the graphs it plans, and DesignRules' grades. `roadClass` falls back to local.
const std::map<std::string, RoadClassSpec>& roadClasses();
const RoadClassSpec& roadClass(const std::string& name);
// A table row as a graph's "classes" entry (what fromJson reads back to the same spec).
nlohmann::json classJson(const RoadClassSpec& c);
std::vector<LaneSlot> laneSlots(const RoadClassSpec& c, const LaneLayout& layout);

// --- path helpers shared with lane expansion (ramps are recomposed from their anchors) ---
std::vector<Vec2> bezier(const std::array<Vec2, 4>& p, int n = 200);
std::vector<Vec2> resample(const std::vector<Vec2>& pts, double step);
// A point spec: [x, y] or {"edge", s|t|x|y, along, normal, dx, dy}; edges must already be resolved.
Vec2 resolvePoint(const nlohmann::json& spec, const RoadLabGraph& g);
std::vector<Vec2> resolvePath(const nlohmann::json& path, const RoadLabGraph& g, double step);

}  // namespace roads::lanes
}  // namespace engine

#endif
