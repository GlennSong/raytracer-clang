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
#include "engine/procgen/city/roads/road_model_parts.h"   // ModelJunction, ModelStreet: the road file's junctions and streets ARE the model's
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
    // One-way, and the width the travel lanes take across: the cross-section's one answer to each (road_model_parts.h).
    bool oneWay() const { return lanesRunOneWay(fwd, back); }
    double travelWidth() const { return lanesTravelWidth(fwd, back, w, gap); }
};

// The narrowest travel lane kerb parking may leave (metres): a band that would leave less is not built.
constexpr double kMinTravelLane = 3.2;

// THE TAPER A ROAD FILE DOES NOT STATE, in lane widths: a turn pocket's (of its road's lanes) and a ramp anchor's (of its
// host's). Each is applied once, where the road file is read (RoadLabGraph::fromJson), so PocketSpec::taper and
// RampAnchor::taper always hold the metres the builder lays and the road model states.
constexpr double kPocketTaperLaneWidths = 10.0;
constexpr double kRampTaperLaneWidths = 30.0;

struct PocketSpec {
    std::string id, kind = "turn", side = "right";
    int dir = +1;            // +1 forward lanes, -1 backward lanes
    double s0 = 0, s1 = 0;
    double taper = 0;        // metres over which it is swept out of its lane (the file's, else kPocketTaperLaneWidths)
    double taperOut = 0;     // metres over which it runs back into it; 0: it does not
};

struct RampAnchor {
    bool set = false;        // an anchor with geometry (station on the host); a bare edge id only supplies height
    std::string edge;        // host edge id
    bool hasStation = false; double s = 0;
    char axis = 0; double coord = 0;   // 'x' or 'y': nearest host vertex to that coordinate; 'p': nearest to (px, py)
    double px = 0, py = 0;
    std::string side = "right";
    // metres: the auxiliary lane beside the host after an entry's gore, the deceleration lane before an exit's, the
    // taper beyond it (the file's, else kRampTaperLaneWidths of the host's lanes; 0 only on an anchor that is not set)
    double aux = 120, decel = 100, taper = 0, approach = 150;
};

struct ConnectorSpec { std::string from, to; double toS = 0; };

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
    // WHETHER ITS ARMS AT A JUNCTION ARE MARKED (issue #292): a crosswalk across the arm, and the stop bar and lane arrows
    // the road model states for it (deck_mesh.h paintsJunctionBoxes, the one reader). A street's are; a freeway's, a
    // ramp's, a track's, a footway's and a divider's are not. The class's own row says so ("junction_markings" in a
    // road file's `classes`); no code asks the class's name or rank. A class a file adds starts from the local row: marked.
    bool junctionMarkings = true;
    // WHAT THE BUILDER ONCE ASKED A CLASS'S NAME FOR, stated by its row instead (ADR-0438; assets/data/road_classes.json
    // says what each means):
    //   - `engineClass` ("engine_class"): which of the engine's six driving classes it is (classOf, the one mapping);
    //   - `limitedAccess` ("limited_access"): a road other roads never meet at one level (a freeway): a crossing with any
    //     road but a ramp is a bridge, two side by side share a median wall, a street close beside gets a noise wall;
    //   - `defaultBarriers` ("default_barriers"): its edges carry the builder's usual barriers where `edges` states none;
    //   - `planColour` ("plan_colour"): its lanes' fill on the plan picture (empty: the picture's usual one).
    RoadClass engineClass = RoadClass::Local;
    bool limitedAccess = false;
    bool defaultBarriers = false;
    std::string planColour;
    // Do cars drive on its roads (the cross-section's one answer, road_model_parts.h: not a track, not a footway)?
    bool carriesCars() const { return sectionCarriesCars(track, footway); }
};

// HOW FAR OUT FROM THE SPINE A SECTION'S BANDS REACH -- the one sum each (issue #292), for a class row and the lanes laid
// on it: to the carriageway's edge (half the travel lanes' width, then the shoulder), and to the sidewalk's outer edge
// (the sidewalk lies outside the shoulder). A road-file edge asks the same through RoadLabGraph::carriagewayHalfWidth /
// sidewalkOuterHalfWidth; a street that is not yet an edge of a graph (the interchange maker's) asks these.
inline double carriagewayHalfWidth(const RoadClassSpec& c, const LaneLayout& lanes) { return carriagewayHalfWidthOf(lanes.travelWidth(), c.shoulder); }
inline double sidewalkOuterHalfWidth(const RoadClassSpec& c, const LaneLayout& lanes) { return carriagewayHalfWidth(c, lanes) + c.sidewalk; }

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
    // WHERE THE RAMP'S LINE LEAVES ([0], its from-anchor) OR MEETS ([1], its to-anchor) ITS HOST LANE'S LINE: the station
    // along the host's spine where composeRamp laid the far end of the deceleration or auxiliary lane and its taper.
    // The builder's record; the road model copies it (ModelConnection::meets). NaN: no such anchor.
    double anchorMeets[2] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()};
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
    // THE CONTROL PLAN (road_model_parts.h ModelJunction). A graph whose "junctions" list names controls DECLARES every junction's
    // control: the lane graph builder reports a junction it builds that no planned one names. Without the list, or with a
    // list that names no control (the road model's layout alone), each junction carries the lane graph options' one
    // control, as before.
    std::vector<ModelJunction> junctions;
    bool declaresJunctions = false;       // the list names each junction's CONTROL (the control plan)
    bool listsJunctions = false;          // the graph carried a "junctions" list at all, with controls or without
    // THE JUNCTIONS' ARMS ARE LISTED IN THEIR RING (ADR-0427), stated at the graph's top level beside the junctions list,
    // "arms_in_ring": true -- the file's word that every junction's "arms" list goes round it counter-clockwise in plan
    // coordinates (road_layout.h armRing says how its writer worked that out). Its writers: the layout (deriveLayout,
    // writeLayout), the control plan (writeControlPlan) and the one-off `lanes_tool layout <file> --arm-order --write`.
    // Every reader trusts a marked file's list order and never measures it again; the lane level refuses an unmarked
    // file. (Outside the bake key, like the rest of the layout: road_layout.h keyedRoadFileText.)
    bool armsInRing = false;
    // WHERE A STREET ONLY CARRIES ON (no junction, ADR-0419) the lane graph builder may still make a junction of its own
    // until the builder builds the stated ones: the control the plan gives it there ("carry_on_control"; control_plan.h)
    std::string carryOnControl;
    // THE DIVIDED ROADS AS PLANNED (DividedPair); `listsDivided`: the graph carried the list.
    std::vector<DividedPair> divided;
    bool listsDivided = false;
    // THE STREETS AS PLANNED (road_model_parts.h ModelStreet); `listsStreets`: the graph carried the list.
    std::vector<ModelStreet> streets;
    bool listsStreets = false;
    // THE LANE LEVEL'S RULES (ADR-0426), stated at the graph's top level, "lane_rules": {"solid_before_stop_line": 20.0}:
    // how far back from a stop line the lines between lanes running the same way are solid (no changing lanes on the
    // approach to a junction). THE ROAD FILE IS THE NUMBER'S ONE HOME (ADR-0435): the lane level (lanes/lane_level.h) and
    // the paint read it here; the planner states it in the file it writes, from the number its ramps are laid by
    // (plan/plan_scene.h SceneOptions::solidBeforeStopLine). The city's own bake key leaves it out (road_layout.h
    // keyedRoadFileText); the paint's key holds it. `statesLaneRules`: the graph carried the member.
    struct LaneRules {
        double solidBeforeStopLine = 0;
    };
    LaneRules laneRules;
    bool statesLaneRules = false;
    bool designed() const;               // every road with a spine carries its planned profile
    std::map<std::string, int> index;    // edge id -> position in edges
    // the planned crossing of edges a and b nearest p (within `within` m), or null
    const CrossingSpec* declaredCrossing(const std::string& a, const std::string& b, const Vec2& p, double within = 12.0) const;

    const EdgeSpec* find(const std::string& id) const;
    EdgeSpec* find(const std::string& id);
    const RoadClassSpec& cls(const EdgeSpec& e) const { return classes.at(e.cls); }
    int rank(const std::string& edgeId) const { return cls(*find(edgeId)).rank; }
    // half the width of a road's travel lanes, and the spine to its carriageway's edge (with the class's shoulder): both
    // the cross-section's one answer (road_model_parts.h), asked for a road-file edge
    double hw(const EdgeSpec& e) const { return e.lanes.travelWidth() * 0.5; }
    double carriagewayHalfWidth(const EdgeSpec& e) const { return roads::lanes::carriagewayHalfWidth(cls(e), e.lanes); }
    // WHERE THE SIDEWALK STARTS AND ENDS, out from the spine (issue #292): it starts at the carriageway's edge, OUTSIDE the
    // shoulder (as bandsFor lays the bands: lanes, shoulder, sidewalk), and is the class's sidewalk wide. The pavement
    // draws it there and the ground conforms out to there. (The pavement once measured it from the lanes' edge, so a
    // class with a shoulder and a sidewalk had its sidewalk drawn under its shoulder: narrower than stated, or gone.)
    double sidewalkOuterHalfWidth(const EdgeSpec& e) const { return roads::lanes::sidewalkOuterHalfWidth(cls(e), e.lanes); }
    std::array<double, 4> bounds() const;    // terrain bounds, or the edges' extent plus a margin
    std::vector<const EdgeSpec*> paved() const;

    static RoadLabGraph fromJson(const nlohmann::json& spec, const std::string& baseDir = ".");
    static RoadLabGraph load(const std::string& path);
};

// The engine band list for a class: [Sidewalk][Shoulder][Travel dir -1 x back][Median gap][Travel dir +1 x fwd][Shoulder][Sidewalk],
// one-way roads centred. Lane offsets are derived from RoadSpec::bandSpan, so the lane model is the engine's.
RoadSpec bandsFor(const RoadClassSpec& c, const LaneLayout& layout);

// Which engine RoadClass a road-file edge carries. ONE mapping: the road model, the drawn deck and anything else that
// has to speak both vocabularies reads it here. A road laid as a ramp (anchored to a host's lane) is a ramp; any other
// is what its class's row states (RoadClassSpec::engineClass, "engine_class"). No class is asked by name.
RoadClass classOf(const RoadLabGraph& g, const EdgeSpec& e);

// THE STREET'S CROSS-SECTION AS THE CITY READS IT (ADR-0418): the engine band list of a road-file edge --
// the carriageway (the road's lanes and shoulders: 2 x (hw + shoulder)) split into travel bands and, where the road states
// kerb parking that fits (LaneLayout::parking), a Parking band that wide along each kerb, carved out of the carriageway's
// edges (so it lies over the kerb side of the kerb lane; the carriageway's width never moves). The one reader of the
// stated parking: the road model's kerb parking (which the sim lays its kerbside bays in) comes from here.
// `e` is a whole road-file edge.
RoadSpec streetSection(const RoadLabGraph& g, const EdgeSpec& e);
// The kerb parking `e` really has along each kerb: its stated width where it fits, else 0. `fits` (optional): false when
// it states parking that would leave a travel lane narrower than kMinTravelLane.
double kerbParking(const RoadLabGraph& g, const EdgeSpec& e, bool* fits = nullptr);

// (name, left-normal offset from the spine, direction) of each Travel band, from the RoadSpec.
struct LaneSlot { std::string name; double offset; int dir; };
// THE ROAD CLASS TABLE (ADR-0276; a data file since ADR-0438): every class's section, grade, profile and the builder's
// treatment of it, once -- what a graph's classes start from, what the planner writes into the graphs it plans, and
// DesignRules' grades. ITS ONE HOME IS assets/data/road_classes.json, read once by roadClasses(); there is no copy in
// code. A missing or unreadable file, or one without its default row, throws std::runtime_error naming the file: no
// road can be built without it. `roadClass` gives the default row (the file's "default": the local street) for a name
// the table does not have. A level's road file states its own `classes` on top (RoadLabGraph::fromJson), through the
// same row reader (readRoadClassRow).
constexpr const char* kRoadClassTableFile = "assets/data/road_classes.json";
const std::map<std::string, RoadClassSpec>& roadClasses();
const RoadClassSpec& roadClass(const std::string& name);
// The name of the row a class no table row names starts from (the file's "default").
const std::string& defaultRoadClassName();
// THE TABLE READ FROM A DOCUMENT (the file's own shape: {"default": name, "classes": {name: row}}); `defaultName`
// receives its default row's name. Throws std::runtime_error on a row that names a `like` it cannot follow, an
// `engine_class` the engine does not have, or a missing default row. roadClasses() is this over the shipped file.
std::map<std::string, RoadClassSpec> parseRoadClassTable(const nlohmann::json& doc, std::string* defaultName = nullptr);
// ONE ROW'S KEYS applied over `c` (each key present replaces c's value; `like` is the table's own and is not read
// here): the one reader of a class row, for the table's file and for a road file's `classes`.
void readRoadClassRow(RoadClassSpec& c, const nlohmann::json& row);
// The table file's bytes, hashed: part of the city's bake key (lanes/city_producer.cpp), so editing a row re-bakes.
uint64_t roadClassTableHash();
// A table row as a graph's "classes" entry (what fromJson reads back to the same spec).
nlohmann::json classJson(const RoadClassSpec& c);
std::vector<LaneSlot> laneSlots(const RoadClassSpec& c, const LaneLayout& layout);
// THE LINE A TRAVEL LANE IS DRIVEN ALONG (ADR-0450, #302): the middle of the part of the lane traffic keeps to. The kerb
// parking band lies over the kerb side of the kerb lane (LaneLayout::parking, above), and traffic keeps to the rest, so
// a lane the band overlaps is driven along the middle of what is left, that much narrower; any other lane along its
// own middle, at its width. From the band `e` states and that fits (streetSection), never a measurement. `offset` as
// LaneSlot's (left of the spine is +), `width` in metres. The lane level drives this line (lane_level.cpp); the builder
// lays and paints the lane as stated.
struct DrivenLine { double offset = 0, width = 0; };
DrivenLine drivenLine(const RoadLabGraph& g, const EdgeSpec& e, const LaneSlot& slot);

// --- path helpers shared with lane expansion (ramps are recomposed from their anchors) ---
std::vector<Vec2> bezier(const std::array<Vec2, 4>& p, int n = 200);
std::vector<Vec2> resample(const std::vector<Vec2>& pts, double step);
// A point spec: [x, y] or {"edge", s|t|x|y, along, normal, dx, dy}; edges must already be resolved.
Vec2 resolvePoint(const nlohmann::json& spec, const RoadLabGraph& g);
std::vector<Vec2> resolvePath(const nlohmann::json& path, const RoadLabGraph& g, double step);

}  // namespace roads::lanes
}  // namespace engine

#endif
