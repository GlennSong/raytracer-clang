#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_MODEL_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_MODEL_H

// THE ROAD MODEL (docs/road-model-design.md, ADR-0417): one description of a level's roads, with ids that never change,
// that later readers will query instead of each working the roads out again. Made in step 2 of the design beside the
// old forms; read since by the names and signs (step 3), the controls (step 4), the street map (step 7, ADR-0425) and
// the lane graph, which is now a table built from its lanes, connections and turns (step 8, ADR-0426, lane_level.h).
//
// Words used here:
//   - a JUNCTION is where roads meet at one level; its ARMS are the roads' ways out of it, in order round it;
//   - a STREET is one stretch of road from a junction to the next (or a dead end): a road-file edge cut at its junctions;
//   - a LANE is one line of travel along a road, with its real curve and heights; it runs along one or more streets;
//   - a STATION is distance along a road-file edge's centre line (its spine), in metres;
//   - a lane's SLOT is its place across the road among the lanes running its way, counted from the centre-line side
//     (0) to the kerb side;
//   - a CONNECTION is how one lane leads to another: a ramp's lane merging into or leaving its host's lane (with the
//     host station where they meet, `meets`), a pocket or a dropped lane swept from or dying into its host lane, or a lane
//     the road file joins to another by a declared connector;
//   - a TURN is a stated way through a junction from one arriving lane to one leaving lane, with its kind (straight on,
//     near turn: across no oncoming traffic, far turn: across it) and the turn table's row that states it (ADR-0426,
//     assets/data/lane_turns.json, lane_turns.h);
//   - a STOP LINE is where traffic arriving by an arm stops: the paint's, kStopLineBack outside its junction box.
//
// WHO STATES WHAT (design section 2). Everything here is copied from a stage that states it, never worked out:
//   - the road file (the planner, or the one-off layout tool): junctions and their arms, headings and through pairs,
//     streets, classes and cross-sections (with their kerb parking, ADR-0418), one-way, speed limits, controls and signs,
//     crossings, ramps and their anchors;
//   - the road builder: each lane's curve, heights and slot, the streets it runs along, the junction at each end, the
//     ramps' merges (host lane, station, length, taper), the paint's junction boxes and stop lines.
//   - the turn table (assets/data/lane_turns.json): which arriving lanes take which turn, applied to the stated arms,
//     through pairs and lane kinds in one function (turnsAtJunction).
//   - the kerb parking rules (assets/data/kerb_parking.json): how a street's parking band is cut into spaces, applied to
//     the stated streets and the arms' stop lines in one function (ADR-0439; RoadModel::kerbRuns).
// Where nothing states a part yet (the turns' drawn paths, which the lane level shapes; crosswalks, blocks, signal
// groups), it is EMPTY, and the model's `notStated` list says so. Street
// names are stated since ADR-0420 (the road file's streets[].name), as ATTACHMENTS (below). Where the build
// does not match what was stated (a box the paint found that no stated junction holds, an arm with no box), it is in
// `problems`, with its place.
//
// IDS. A junction or street is named by the road file's id; a lane by the builder's lane id (edge.f0, edge.pocket, ...).
// Inside the model they are indices into its arrays (GraphIndex, as the lane graph uses), stable for one road file.
//
// TWO LAYERS, ONE MODEL (ADR-0420). The PLANNED LAYER is everything the road file alone states -- junctions, arms,
// streets and their centre lines, cross-sections, one-way, carrying on, through pairs, divided roads, ramps, crossings,
// names -- read in milliseconds (plannedRoadModel): what the readers that only label the roads read (street names, name
// signs, the map). The BUILT LAYER is what the road build adds
// -- lanes with their curves and heights, connections, junction heights, the paint's boxes and stop lines -- assembled
// from a build (assembleRoadModel = the planned layer + the built parts) by whoever has the build. A LEVEL'S model is
// made in one place (ADR-0435): lanes/road_model_cache.h obtainLevelRoadModel, which the loader calls and publishes
// (components.h LevelRoadModel) -- the built model where the road file states enough, else the planned layer. A tool or
// a test with a road file and no level calls plannedRoadModel or assembleRoadModel itself. The built layer holds THE
// LANE LEVEL too (ADR-0429, `laneLevel`): the table of paths the driving drives on, filled as the last step of
// assembling, for the vehicles named (a level's: its own fleet, read by obtainLevelRoadModel).
// The built model is saved whole, lane level and all, in cache/roadmodel under one key; it is not in the
// level's bake (that would re-bake every level). A planned-only model is the same structure with its built parts empty,
// `built` false, and its `notStated` list says which.
//
// ATTACHMENTS (design section 1: "a bus stop is street 41, station 120, kerb side"). Anything that belongs to a street or
// a junction is attached to it by id: its KIND is data (assets/data/road_attachments.json: what it may attach to -- a
// street, a junction, or an arm of a junction), never a branch in code. The first users: a street's NAME (from the road
// file) and a NAME SIGN at a junction arm (the loader's sign plan).

#include "engine/procgen/city/roads/lane_graph.h"        // Turn; bundle::BinWriter
#include "engine/procgen/city/roads/road_model_parts.h"  // junctions, arms, streets: the types the road file is read into

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace engine {

struct TurnArm;   // lane_turns.h

namespace roads::lanes {
struct Result;
struct RoadLabGraph;
struct LaneGraphOptions;   // lanes/lane_graph_build.h: the vehicles the lane level's turns are checked against
struct LaneGraphBuild;     // ...and what building the lane level reports
}  // namespace roads::lanes

// WHAT A KIND OF ATTACHMENT IS (assets/data/road_attachments.json): its name, what it attaches to ("street", "junction"
// or "arm"), and whether it is the street's name (what a reader asks for when it wants one).
struct AttachmentKind {
    std::string name, on;
    bool names = false;
    // The junction controls this kind stands for (the rule sets' control kinds: a signal pole stands for "signal"),
    // empty for a kind that stands at any junction (ADR-0421)
    std::vector<std::string> controls;
    // A SIGN THAT FOLLOWS FROM A FACT (ADR-0424): the kind's placement row as the file writes it (null: the kind is not
    // placed from the facts). control_signs.h reads it.
    nlohmann::json sign;
    // A MARKING PAINTED ON THE ROAD THAT FOLLOWS FROM A FACT (a stop bar): the kind's `marking` row as the file writes it
    // (null: none). control_signs.h reads it (armMarkingRules), the paint draws it.
    nlohmann::json marking;
    bool standsFor(const std::string& control) const;
};
struct AttachmentKinds {
    std::vector<AttachmentKind> kinds;
    // every junction control the file knows (the names under its "controls"."map"): a junction stating any other is one
    // the paint has no row for, and says so
    std::vector<std::string> controlNames;
    std::vector<std::pair<std::string, std::string>> uses;   // a reader's role -> the kind it writes or reads
    const AttachmentKind* find(const std::string& name) const;
    std::string use(const std::string& role) const;          // the kind a role uses (empty: none)
};
AttachmentKinds parseAttachmentKinds(const nlohmann::json& doc);
// assets/data/road_attachments.json, read once; a built-in copy of the shipped kinds when the file is missing.
const AttachmentKinds& attachmentKinds();

// ONE THING ATTACHED TO THE ROADS. On a street: `street` (and `station` along its road; kNotStated: the whole street). On a
// junction: `junction`. On an arm: `junction` and `arm`, and `street` is the arm's street. `text`: what it says (a name).
// `at`: where it stands in plan, when the attaching reader placed it (a sign's post); kNotStated when not.
struct ModelAttachment {
    std::string kind;
    ModelStreetId street;
    ModelJunctionId junction;
    int arm = -1;
    Real station = kNotStated;
    std::string text;
    Vec2 at = Vec2(kNotStated, kNotStated);
};

// A REGULATION ON A STREET (ADR-0437): a rule that holds on a stretch of one street for the traffic running one way
// along it -- "top speed 15 m/s from station 30 to the street's end, for traffic running along the spine". Attached by
// street id and station, as everything else on the roads is. Its KIND is a name from the data files: the rule set's
// device of that name (assets/scripts/rules/*.lua `devices`) says what it means, and the driving refuses a rule set
// that lacks it when the rules are handed over (apps/driving/road_rules.h). `from` == `to`: it stands at one place (a
// sign). `value`: what the kind's rule makes of it (a top speed in metres per second); unused by a kind that needs none.
// THE ONE HOME of where a rule stands along a road and what value it carries. (A sign a junction's arm states --
// ModelArm::regulations, the road file's -- stands at the end of every lane arriving by that arm: stated there, once.)
// A path's regulations are ASKED: RoadModel::regulationsAlong. Nothing writes these into a road file yet.
struct ModelRegulation {
    std::string kind;
    ModelStreetId street;
    Real from = 0, to = 0;           // stations along the street's road's spine, from <= to
    int dir = 1;                     // whose traffic it holds for: +1 running along the spine, -1 against it
    Real value = 0;
};
// ONE REGULATION AS A PATH MEETS IT (RoadModel::regulationsAlong): the kind's name, the stretch of the path where it
// holds (metres along the path; equal: it stands at one place) and its value.
struct PathRegulation {
    std::string kind;
    Real from = 0, to = 0;
    Real value = 0;
};

// ---- KERB PARKING SPACES (ADR-0439) ------------------------------------------------------------------------------------
// HOW A PARKING BAND IS CUT INTO SPACES: assets/data/kerb_parking.json, read once (the file's comment says what each
// number is and where it came from). No copy in C++: a missing or unreadable file reads as `usable` false, and then no
// street is given a space (the log says why). `hash`: of the file's bytes, part of the saved road model's key
// (lanes/road_model_cache.h), so editing the file makes each saved model again.
struct KerbParkingRules {
    bool usable = false;
    Real spaceLength = 0;        // metres of kerb one space takes, along the street's centre line
    Real clearOfStopLine = 0;    // metres kept empty between an arm's stop line and the nearest space
    uint64_t hash = 0;
};
// The rules a parsed file states; false (and why in `err`) when a number is missing or not above zero (a clearance: below).
bool parseKerbParkingRules(const nlohmann::json& doc, KerbParkingRules& out, std::string* err = nullptr);
const KerbParkingRules& kerbParkingRules();

// A RUN OF PARKING SPACES ALONG ONE KERB OF ONE STREET: THE ONE HOME of where a car may be left at a kerb. `count`
// spaces, each `spaceLength` metres of the street's spine, one after another from station `from`. WHICH KERB is said by
// the traffic that runs beside it, never by a hand: `dir` +1 is the kerb side of the traffic running along the spine,
// -1 the kerb side of the traffic running against it. How WIDE a space is and where it lies across the road are the
// street's own cross-section (ModelSection::parking, parkingOffset: stated once, there). The planned layer states the
// runs (plannedRoadModel, from the stated streets, junction arms' stop lines and the rules above), by street, the +1
// kerb then the -1 kerb, then along the spine. A space is not a record: it is place `i` of its run, and everything
// about it is ASKED (RoadModel::kerbSpacesOf and the questions beside it).
struct ModelKerbRun {
    ModelStreetId street;
    int dir = 1;
    Real from = 0;               // the station where its first space begins (the lower station of the run)
    Real spaceLength = 0;
    uint32_t count = 0;
    Real to() const { return from + static_cast<Real>(count) * spaceLength; }   // the station where its last space ends
};
// ONE SPACE, AS ANSWERED. Its id is (street, dir, index): `index` counts the spaces of that kerb of that street along
// the spine from the street's `from` end, across its runs, so it is the same number for one road file and one rules
// file. `from` to `to`: its stretch of the spine. `length`, `width`: metres along and across.
struct KerbSpace {
    ModelStreetId street;
    int dir = 1;
    uint32_t index = 0;
    Real from = 0, to = 0;
    Real length = 0, width = 0;
    bool valid() const { return street.valid(); }
    Real middle() const { return (from + to) * Real(0.5); }
};
// WHERE A CAR STANDING IN A SPACE IS: its middle in plan and the way it faces (unit; the way the traffic beside it runs).
struct KerbSpacePose {
    Vec2 at;
    Vec2 facing = Vec2(1, 0);
};
// A PLACE IN THE KERB LANE: a lane path of the lane level and the distance along it (metres from its start), with the
// plan point there. Not valid: the model has no lane level, or no lane path of that street runs that way there.
struct KerbLanePlace {
    PathId path;
    Real along = 0;
    Vec2 at;
    bool valid() const { return path.valid(); }
};
// A SPACE FOUND FROM A PLACE: the space, how far it is (metres: in a straight line for nearestKerbSpace, on foot along
// the streets for kerbSpacesNear) and where a car in it stands.
struct KerbSpaceNear {
    KerbSpace space;
    Real distance = 0;
    Vec2 at;
};

// ---- FROM A DOOR TO THE ROAD (ADR-0439) --------------------------------------------------------------------------------
// ONE STRETCH OF ROAD WITH A CENTRE LINE that cars drive: a street, or a stretch of road inside a junction. The list of
// them (RoadModel::roadLines), in one order, is what "the road nearest this point" is searched over -- by the lot pass
// (lanes/lots_producer.h lotRoadInputs makes its look-up from exactly this list) and by the model's own questions
// (frontageAt), so the two cannot find different roads.
struct RoadLine {
    ModelStreetId street;            // the street it is (none: a stretch inside a junction)
    ModelJunctionId junction;        // the junction it lies inside (none: a street)
    Range line;                      // into RoadModel::linePoints / lineStations
    const ModelSection* section = nullptr;
};
// (What a building may front is one rule, road_network.h buildingMayFront: the lot pass and frontageAt both ask it.)
// WHERE A POINT (A DOOR) FRONTS THE ROAD: the street (or the junction, where the nearest road is a stretch inside
// one), the station along its road's spine level with the point, which kerb the point is on (`dir`: +1 the kerb side
// of the traffic running along the spine, -1 of the traffic running against it), how far the point is from the centre
// line and the point of the centre line nearest it (`on`). Not valid: no road a building may front has a centre line.
struct RoadFrontage {
    ModelStreetId street;
    ModelJunctionId junction;
    Real station = 0;
    int dir = 1;
    Real distance = 0;
    Vec2 on;
    Vec2 towards = Vec2(1, 0);       // unit, across the road from the centre line toward the point's kerb
    const ModelSection* section = nullptr;
    bool valid() const { return section != nullptr; }
};

// ---- WHERE A PLACE JOINS THE WALKS (ADR-0468, issue #300) --------------------------------------------------------------
// THE POINT ON THE WALKS A PLACE (A MARKER, A DOOR) JOINS: a place on a walk of the lane level (the walk and how far along
// it, LaneGraph::walks), the street it runs along and the kerb it runs beside. Where a walker's route to or from that place
// begins or ends (route.h findWalkRoute). Asked by RoadModel::walkJoinOn (stated facts) or walkJoinNear (the fallback, by
// position). Only a walk that runs ALONG a street is joined: a sidewalk's stretch, or a footway's. A corner, a crossing or
// a turn-round runs round or across a junction or a road end, and no place fronts one.
//
// Not valid (`at.walk` not valid): `why` says why. Valid, it may still need saying: `held` (the place lies past where the
// sidewalk on its kerb stops, so it joins at that sidewalk's end), `across` (its own kerb has no walk the mover may use,
// so it joins the other kerb's: the person must cross the road there), `cutOff` (the walk is not in the largest part of
// the walks the mover can get round, when the parts were handed in).
enum class WalkJoinWhy : uint8_t {
    None,       // joined
    NoStreet,   // the frontage names no street: no road a building may front is near, or the nearest is a stretch of road
                // inside a junction (a spread junction's), along which no walk is laid
    NoWalk,     // the lane level lays no walk along that street (its class has no sidewalk, a planned-only model, ...)
    NotOpen,    // it lays walks along that street (or that walkway), but none the mover may use
    NoWalkway,  // the frontage names a walkway the lane level does not hold (ADR-0475)
};
const char* walkJoinWhyWord(WalkJoinWhy why);
struct WalkJoin {
    WalkPoint at;                    // the walk and metres along it from its start (not valid: none; `why`)
    ModelStreetId street;            // the street that walk runs along (none: a walkway's)
    uint32_t walkway = kNoIndex;     // the walkway it is a stretch of (RoadModel::walkways; ADR-0475), else kNoIndex
    int dir = 1;                     // the kerb it runs beside (a footway's walk is its middle: the asked kerb)
    Real station = 0;                // where along the street's road's spine it joins (the asked station, held to the walk);
                                     // on a walkway, the metres along the walkway
    Vec2 point;                      // where that is on the walk (LaneGraph::pointAt)
    Real held = 0;                   // metres along the spine from the asked station to the walk's end it joins at (0: beside it)
    bool across = false;             // joined on the other kerb: the asked kerb has no walk the mover may use there
    bool searched = false;           // asked by position (walkJoinNear, the fallback): `distance` is from the asked point
    Real distance = 0;               // ...how far the asked point is from `point` (stated facts: 0)
    uint32_t part = kNoIndex;        // the walk's part on foot (ReachableParts::partOf), when the parts were handed in
    bool cutOff = false;             // ...and it is not the largest
    WalkJoinWhy why = WalkJoinWhy::None;
    bool valid() const { return at.walk.valid(); }
};

// One lane's stretch on one street.
struct ModelLaneSpan {
    ModelStreetId street;
    Real from = 0, to = 0;           // stations along the road's spine
};

// THE MODEL'S LANE: the lane's facts (road_model_parts.h LaneFacts: id, kind, road, direction, slot, width, stretch of
// spine, and the stretch of the lanes' centre lines that is its own (`curve`: into RoadModel::lanePoints / laneAlong /
// heights / spineStations) -- the builder's lane's, taken whole) and where it lies: the streets it runs along and the
// junction at each end.
struct ModelLane : LaneFacts {
    std::vector<ModelLaneSpan> streets;
    ModelJunctionId atStart, atEnd;  // the junction it begins and ends at, the way its traffic runs
};

// HOW ONE LANE LEADS TO ANOTHER, as stated: a ramp's lane leaving its host's lane (Diverge) or merging into it (Merge),
// at a station along the host, over a stated length and taper; a turn pocket leaving the lane it is swept out of
// (Diverge), or a lane dropping into its neighbour (Merge); or a connector the road file declares (Joins).
// A DIVERGE is "this lane leaves that lane over this stretch": the branch's line starts ON the host lane's line at
// `meets` and draws away from it over the taper (and, for a ramp, the deceleration length) to the gore. A MERGE is "this
// lane joins that lane over this stretch": from the gore it runs beside the host (a ramp's auxiliary length) and draws
// in over the taper, its line ending ON the host lane's at `meets`. So the lane level links the host lane, cut at
// `meets`, to the branch (a diverge) and the branch's end to the host lane from `meets` on (a merge).
enum class ConnectionKind : uint8_t { Diverge, Merge, Joins };
struct ModelConnection {
    ConnectionKind kind = ConnectionKind::Merge;
    ModelLaneId from, to;
    std::string ramp;                // the ramp it belongs to (Diverge, Merge of a ramp; empty for a pocket)
    Real hostStation = 0;            // the gore, along the host's spine (Diverge, Merge); the far lane's station (Joins)
    Real length = 0;                 // the lane of its own beside the host: deceleration (Diverge) or auxiliary (Merge)
    Real taper = 0;
    // where the two lanes' lines meet, along the host's spine (Diverge, Merge): the gore less the taper and the
    // deceleration length the way the host runs (a diverge), or the gore plus the auxiliary length and the taper (a
    // merge) -- the builder's own arithmetic on the stated numbers (lane_expand.cpp composeRamp, roadLanes)
    Real meets = kNotStated;
};

// ONE TURN THROUGH A JUNCTION, as stated (ADR-0426): from an arriving lane of one arm into a leaving lane of another, by
// the rows of assets/data/lane_turns.json over the junction's stated arms, through pairs, lanes and control
// (lane_turns.h). The path it follows is the lane level's (the builder's connector shaping); a turn that does not fit
// its corner for a vehicle class stays stated, and the lane level reports a corner fault against it.
struct ModelTurn {
    ModelJunctionId junction;
    uint32_t fromArm = 0, toArm = 0;  // indices into the junction's arms
    ModelLaneId from, to;            // the arriving lane and the leaving lane
    Turn turn = Turn::None;
    uint32_t row = 0;                // the table row that stated it
};

// A RAMP: the road it leaves and the road it joins (design section 1). Each end: anchored to a lane of its host (a
// merge), ending bare on a street's side (an arm of a junction there), or free (a road end like any other).
enum class RampEndKind : uint8_t { Free, Anchored, OnStreet };
struct ModelRampEnd {
    RampEndKind kind = RampEndKind::Free;
    std::string host, hostLane;      // the road and (anchored) its lane
    Real hostStation = kNotStated;   // the gore, along the host's spine (anchored)
    Real rampStation = kNotStated;   // the gore, along the ramp's own spine (anchored: where it leaves or meets its host)
    Real length = 0, taper = 0;      // (anchored)
};
struct ModelRamp {
    std::string edge;
    std::array<ModelRampEnd, 2> ends;   // its start, its end
    bool landingPlanned = false;     // the planner wrote how its end meets the street (#282)
    std::string landingStreet, landingJoins, landingCarriageway, landingCrossing;
};

// Two roads crossing without meeting, or meeting at grade away from a shared end, as the road file declares.
struct ModelCrossing {
    std::string a, b, kind, over;
    Vec2 at;
    Real clearance = kNotStated;
};

// ---- CROSSWALKS (ADR-0452) ----------------------------------------------------------------------------------------------
// A CROSSWALK is a place where people on foot walk across one arm of a junction, from one kerb to the other. No road
// file states one; the model STATES them once, in one function (stateCrosswalks in road_model.cpp, called by
// plannedRoadModel, from stated facts only): one across every arm of a stated junction whose road the paint marks (its
// class's `junction_markings`) or whose street has pavements, where its band lies on the road. The band is
// lanes/deck_mesh.h kCrosswalkNear to kCrosswalkFar back from the arm's box edge (ModelArm::boxEdge), the one source of
// those numbers. Everything about a crosswalk is read off its record: the paint draws its zebra there, the lane level
// lays its outline (so the road build's own crossing finder finds the paths that cross it), the driving times its walk
// signal and gives way to people on it, and the city's people on foot wait at its kerb end.
//
// HOW IT IS CONTROLLED, from the junction's stated control and the arm's stated signs: under the junction's LIGHTS
// (a walk signal: WALK, FLASHING DON'T WALK, DON'T WALK); at the arm's SIGNS (a stop or give-way sign: traffic stops or
// gives way at the line, then gives way to people crossing); or NONE (a marked crossing with no lights, or an unmarked
// one: drivers give way to people on it and to people at its kerb plainly about to cross).
enum class CrosswalkControl : uint8_t { Lights, Signs, None };
constexpr uint8_t kCrosswalkControlCount = 3;
struct ModelCrosswalk {
    ModelJunctionId junction;
    uint32_t arm = 0;                // the arm of that junction it lies across (its place in the stated list)
    ModelStreetId street;            // that arm's street (none: the junction states none for the arm)
    std::string road;                // the arm's road (the road-file edge)
    // ITS BAND along the road's spine: the edge nearer the junction and the edge farther from it (stations)
    Real nearStation = 0, farStation = 0;
    // ITS TWO KERB ENDS, in plan, level with the band's middle: [0] on the kerb of the traffic running along the spine
    // (+1), [1] on the kerb of the traffic running against it. The carriageway's edge, where a person steps off.
    std::array<Vec2, 2> ends{};
    Real length = 0;                 // kerb to kerb: the carriageway's width there (m)
    bool spineIn = true;             // the arm's road's spine runs into the junction (as ModelArm::spineIn)
    bool marked = false;             // the paint marks it (a zebra): the arm's road class marks its junction arms
    bool pavements = false;          // its street has pavements, so people on foot reach it
    CrosswalkControl control = CrosswalkControl::None;
    // ITS RECORD IN THE LANE LEVEL (LaneGraph::crosswalks; built layer): where the paths that cross it are found
    // (RoadModel::crossersOf). kNoIndex: none (a planned-only model, or a junction no lane path reaches).
    uint32_t laneCrosswalk = kNoIndex;
    // How wide the band is, along the road (m).
    Real bandWidth() const { return std::fabs(farStation - nearStation); }
    // The middle of its band, in plan, and the way across it from end 0 to end 1 (unit).
    Vec2 middle() const { return (ends[0] + ends[1]) * Real(0.5); }
    // WHERE A PERSON WAITS TO STEP OFF AT KERB END `end` (0 or 1): `back` metres behind it, on the pavement (away from
    // the crosswalk's middle, across the road's line).
    Vec2 kerbWait(int end, Real back) const {
        const Vec2 e = ends[static_cast<std::size_t>(end == 0 ? 0 : 1)];
        const Vec2 out = e - middle();
        return out.length() > 0 ? e + out * (back / out.length()) : e;
    }
    Vec2 across() const {
        const Vec2 d = ends[1] - ends[0];
        return d.length() > 0 ? d * (Real(1) / d.length()) : Vec2(1, 0);
    }
};
// ONE PATH THAT CROSSES A CROSSWALK, as the lane level found it (its crossing points with the crosswalk): the path,
// the stretch of it whose body could be on the crosswalk (metres along it), and the stretch of the walk across that
// it covers (metres from kerb end 0).
struct CrosswalkCrosser {
    PathId path;
    Real from = 0, to = 0;
    Real acrossFrom = 0, acrossTo = 0;
};
// ONE CROSSING ON A WALK ROUND A JUNCTION: the crosswalk crossed, and the kerb end a person steps off from (0 or 1).
struct WalkCrossing {
    uint32_t crosswalk = kNoIndex;
    int fromEnd = 0;
};
const char* crosswalkControlWord(CrosswalkControl c);
// ACROSS THE ROAD TOWARD A KERB (unit): the kerb of the traffic running `dir` (+1 along the spine, -1 against it), where
// the spine runs `spine` (unit). Which side a kerb lies is the one statement, lanes/lane_graph_build.h kerbPerpSign; the
// crosswalks' kerb ends and the walks (lanes/walk_lanes.h) are laid by it.
Vec2 towardKerb(const Vec2& spine, int dir);
// (where a point fronts a road's centre line, from a search's hit on it: line_frontage.h frontageOnLine, ADR-0474)

// ---- WHERE PEOPLE ON FOOT CAN WALK (ADR-0464, #306 task 4 piece 1) -----------------------------------------------------
// The facts the walking paths will be laid from (pieces 2 and 3): stated once, by the planned layer, from stated facts
// only. Nothing reads them yet but the tests and the tools.
//
// WHO MAY USE WHICH KIND OF PATH: assets/data/path_users.json, read once. Its MOVERS are named there with their
// properties (`vehicles`: the mover stands for every vehicle tag of the level's fleet, still checked against each body's
// turns); its KINDS are the kinds of path (a lane's kind as the builder names it, and the kinds the walking paths will
// be laid as), each listing the movers that may use it. A kind the file does not list is open to nobody, and asking
// says so. No C++ names a mover or a kind. `deadEndJoin`: the kind of path that joins a dead end's two sidewalks across
// the road's end (the file's `dead_end.joined_by`); empty: a dead end's sidewalks are not joined. `hash`: of the file's
// bytes, part of the saved road model's key (lanes/road_model_cache.h).
struct PathMover {
    std::string name;
    bool vehicles = false;           // stands for every vehicle tag of the level's fleet
    // `waits` (ADR-0467): the seconds a walking route adds for each walk of a kind the mover steps onto (the wait
    // expected at a crossing), in the file's order. Never on a vehicles mover (the car search does not read it).
    std::vector<std::pair<std::string, Real>> waits;
    // The wait for stepping onto a path of kind `kind`: 0 when none is stated.
    Real waitOn(const std::string& kind) const;
};
struct PathUsers {
    bool usable = false;
    std::vector<PathMover> movers;
    std::vector<std::pair<std::string, std::vector<uint32_t>>> kinds;   // kind -> movers (indices into `movers`), in the file's order
    std::string deadEndJoin;
    uint64_t hash = 0;
    // The movers that may use a path of kind `kind` (null: the file lists no such kind).
    const std::vector<uint32_t>* usersOf(const std::string& kind) const;
    // The mover named `name` (null: none).
    const PathMover* mover(const std::string& name) const;
    // May mover `name` use a path of kind `kind`?
    bool mayUse(const std::string& name, const std::string& kind) const;
};
// The table a parsed file states; false (and why in `err`) when it is not one: no movers or kinds, a kind naming a
// mover the file does not have, or a dead-end join naming a kind it does not list.
bool parsePathUsers(const nlohmann::json& doc, PathUsers& out, std::string* err = nullptr);
const PathUsers& pathUsers();

// A CORNER: the walk round a junction's kerb from one arm to the next one round its stated ring (ADR-0427:
// counter-clockwise), `armB` the arm after `armA` (wrapping), where people can walk along both arms on the kerbs that
// face that corner (ModelSection::pavementOnKerb). `kerbA` / `kerbB`: which kerb of each arm's street that is (+1 the
// kerb of the traffic running along its spine, -1 the other), from the arm's `spineIn` and the world's kerb side
// (lane_turns.h kerbSideCounterClockwise): the arriving traffic's kerb lies toward the next arm round the ring the way
// the kerb side turns. Stated by the planned layer (ringPairs below says which pairs got none, and why).
//
// ITS WALK (built layer, ADR-0465): the line along the middle of the drawn sidewalk between the two arms' kerb ends,
// laid by the road build (lanes/walk_lanes.h): `walk`, an index into RoadModel::walks (kNoIndex: none laid; a planned
// model has none). The build also lays a corner for each pair of a spread junction's arms that meet it at different
// points (`spread`: CornerGap::ApartAtSpread, a corner no road file states), from the drawn pavement between them.
struct ModelCorner {
    ModelJunctionId junction;
    uint32_t armA = 0, armB = 0;
    int kerbA = 1, kerbB = 1;
    uint32_t walk = kNoIndex;
    bool spread = false;
};
// A DEAD END: a street's end with no junction, nothing it carries on into, and no other road meeting it there (the
// road just ends: StreetEndKind::DeadEnd). `turnRound`: its two sidewalks are joined across the road's end by a path of
// the dead-end join's kind (pathUsers().deadEndJoin) -- where the file names one and the street has a sidewalk on both
// kerbs of a road cars drive.
struct ModelDeadEnd {
    ModelStreetId street;
    int end = 0;                     // 0: the street's `from`, 1: its `to`
    bool turnRound = false;
};

// ---- WALKWAYS (ADR-0475, #306 task 4 piece 9) -------------------------------------------------------------------------
// A WALKWAY is a path for people on foot that runs beside no street: a park's path, a plaza's, a cut-through between two
// buildings. Whoever draws it (the lot pass, S6b) STATES it, once: its line and where each of its two ends lands, looked
// up when it was drawn. The road model holds it, and the lane level lays it as walks (its kind's, from the path users'
// table) joined where the ends say -- never by distance.
//
// WHERE AN END LANDS: one of
//   - a street's sidewalk: the street's id, the station along its centre line and the kerb (`meets` as a StatedFrontage
//     names a street: the same three facts a door states, road_network.h, ADR-0474);
//   - another walkway: its id and the metres along it (`meets` naming a walkway);
//   - nothing: a dead end, stated (`deadEnd`), so it is no fault that nothing meets it.
// Stating neither, or both, is a fault the lane level reports.
struct WalkwayEnd {
    StatedFrontage meets;
    bool deadEnd = false;
};
struct ModelWalkway {
    std::string id;                  // unique among the model's walkways; what a door on it names (StatedFrontage::walkway)
    std::string kind;                // a kind of path of assets/data/path_users.json ("footway", "park_path"): who may use it
    Real width = 0;                  // metres
    std::vector<Vec2> line;          // its centre line in plan, from its end 0 to its end 1
    std::vector<Real> heights;       // ...each point's height (as many as `line`)
    std::array<WalkwayEnd, 2> ends{};
};
// WHAT IS WRONG WITH A STATED WALKWAY (or one of its ends), as the lane level found it laying it. A walkway wrong as a
// whole is not laid; an end that does not land is left meeting nothing, and said.
enum class WalkwayFault : uint8_t {
    None,
    NoLaneLevel,      // the model has no lane level to lay it in (a planned-only model)
    AlreadyLaid,      // the lane level already holds walkways: lay the lane level again to state them anew
    NoId,             // it has no id
    SameId,           // another walkway has its id (the first keeps it)
    NoLine,           // fewer than two points, a height missing, or no length
    NoWidth,          // no width
    KindNotListed,    // its kind is not a row of the path users' table (open to nobody)
    StatesNothing,    // the end names no street or walkway and is not stated a dead end
    StatesTwo,        // the end names more than one of a street, a walkway and a dead end
    NoSuchStreet,     // the end names a street the model does not have
    OffStreet,        // ...a station off that street (outside its `from` to `to`)
    NoSidewalk,       // ...a kerb with no sidewalk laid there
    PastSidewalk,     // ...a station past where the sidewalk on that kerb runs (nearer a junction than its kerb end)
    NoSuchWalkway,    // the end names a walkway the model does not have
    OwnWalkway,       // ...its own walkway
    WalkwayNotLaid,   // ...a walkway that was not laid (its own fault says why)
    OffWalkway,       // ...metres along it off its line
};
const char* walkwayFaultWord(WalkwayFault f);
struct WalkwayProblem {
    uint32_t walkway = kNoIndex;     // into RoadModel::walkways (kNoIndex: all of them)
    int end = -1;                    // 0 or 1; -1: the walkway as a whole
    WalkwayFault fault = WalkwayFault::None;
    std::string why;                 // in words, with the numbers
};

struct RoadModel {
    std::string name;
    bool built = false;                   // the built layer is here (assembleRoadModel); false: the planned layer alone
    std::vector<ModelJunction> junctions;
    std::vector<ModelStreet> streets;
    std::vector<ModelLane> lanes;
    std::vector<ModelConnection> connections;
    std::vector<ModelTurn> turns;          // junction by junction, then as lane_turns.h turnsAtJunction orders them
    std::vector<ModelRamp> ramps;
    std::vector<ModelCrossing> crossings;
    // THE LANES' CENTRE LINES (ModelLane::curve ranges into these): each point and its distance along the lane
    // (`curves`, read through lanePoints / laneAlong), its height (the drawn deck), and the station along its road's
    // spine it was laid at (a lane is laid point for point beside its spine's samples; a connector's: kNotStated).
    // `curves` is THE OBJECT THE ROAD BUILD LAID ITS LANES INTO (road_model_parts.h LaneCurves, ADR-0438), held with
    // the build, not copied from it: while the build exists its lanes (lanes/lane_expand.h Lane::xy, s) and this
    // model's read the same points. Null: no lane (a planned-only model).
    std::shared_ptr<const LaneCurves> curves;
    std::vector<Real> heights, spineStations;
    const std::vector<Vec2>& lanePoints() const { return curves ? curves->points : noLanePoints(); }
    const std::vector<Real>& laneAlong() const { return curves ? curves->along : noLaneAlong(); }
    static const std::vector<Vec2>& noLanePoints() { static const std::vector<Vec2> none; return none; }
    static const std::vector<Real>& noLaneAlong() { static const std::vector<Real> none; return none; }
    std::vector<Vec2> linePoints;         // the streets' centre lines (ModelStreet::line ranges into it), and inside stretches'
    std::vector<Real> lineStations;       // ...each point's station along its road's spine (ADR-0425)
    std::vector<ModelAttachment> attachments;   // in the order attached
    std::vector<ModelRegulation> regulations;   // the rules standing along the streets, in the order stated (ADR-0437)
    // THE KERB PARKING SPACES (ADR-0439): each run of spaces along one kerb of one street, by street, the +1 kerb then
    // the -1 kerb, then along the spine. The planned layer's (plannedRoadModel). Read through the questions below.
    std::vector<ModelKerbRun> kerbRuns;
    // THE CROSSWALKS (ADR-0452): junction by junction, each junction's in its arms' order. Stated by the planned layer
    // (see ModelCrosswalk); read through the questions below.
    std::vector<ModelCrosswalk> crosswalks;
    // WHERE PEOPLE WALK (ADR-0464): the corners, junction by junction in each one's ring order, and the dead ends, street
    // by street (its `from` end first). Stated by the planned layer (see ModelCorner, ModelDeadEnd).
    std::vector<ModelCorner> corners;
    std::vector<ModelDeadEnd> deadEnds;
    // THE WALKS (built layer, ADR-0465): every sidewalk lane (kind from the path users' table; `dir` the kerb it runs
    // beside; its streets and the junction at each end as a car lane's) and every corner's walk (no road; named by its
    // corner), as the road build laid them into the same centre lines (`curves`, with their heights and spine stations
    // beside them). A list of its own, beside `lanes`: nothing that reads the car lanes meets them. A footway's lane is a
    // car lane's record of kind "footway" in `lanes`, laid by expand. The lane level lays them as its walks (ADR-0466).
    std::vector<ModelLane> walks;
    // THE WALKWAYS (ADR-0475): the paths for people on foot beside no street, as their drawer stated them, in the order
    // stated. Supplied by stateWalkways (below), which lays them in the lane level. Empty until something states one.
    std::vector<ModelWalkway> walkways;
    std::vector<std::string> notStated;  // the parts of the design nothing states yet, in words
    std::vector<std::string> problems;    // where the build does not match what was stated, each with its place
    // the road file states that its junctions' arms are listed in their ring ("arms_in_ring", ADR-0427): the readers of the
    // order (the turns, the regions, the outline, the signs) read the lists as they are. False: a problem line names the
    // command, and the lane level refuses the file.
    bool armsInRing = false;
    // THE LANE LEVEL (ADR-0429; lanes/lane_level.h says where each part comes from): every curve a car can be on, how
    // lanes lie side by side, the paths through each junction, where paths meet, the rules standing along them. Each path
    // names its street, lane or turn of this model, each junction its junction here. Filled by assembleRoadModel; empty in
    // a planned-only model, and where the road file lacks a fact the lane level needs (the report's `error` says which).
    LaneGraph laneLevel;

    // ---- WHAT A READER OF THE LANE LEVEL ASKS OF THE MODEL (ADR-0436) -------------------------------------------------
    // The lane level holds where a car can be; what the road IS there is the model's, reached by the id each path,
    // junction and crossing point carries. The lane table keeps no copy of these (issue #291), so each has one answer.
    // A path, junction or crossing that names nothing of the model (a table made by hand with nothing stated for it)
    // reads as "none stated": no street, no limit, no class, no control, no turn, no merge.
    //
    // THE STREET A PATH IS ON: a lane's own (Path::street); a connector's, that of the lane it leads into. Null: none.
    const ModelStreet* streetOf(PathId path) const {
        if (path.index >= laneLevel.paths.size()) return nullptr;
        const Path* p = &laneLevel.paths[path.index];
        if (p->kind == PathKind::Connector) {
            if (p->next.count == 0 || p->next.first >= laneLevel.nextPaths.size()) return nullptr;
            const PathId into = laneLevel.nextPaths[p->next.first];
            if (into.index >= laneLevel.paths.size()) return nullptr;
            p = &laneLevel.paths[into.index];
        }
        return p->street < streets.size() ? &streets[p->street] : nullptr;
    }
    // THE LIMIT POSTED ON A PATH, metres per second (0: none posted): its street's (streetOf).
    Real postedLimit(PathId path) const {
        const ModelStreet* s = streetOf(path);
        return s ? s->speedLimit : Real(0);
    }
    // THE CLASS OF THE ROAD A PATH IS ON, by name (empty: none): its street's cross-section's.
    const std::string& roadClassOf(PathId path) const;
    // THE TURN A CONNECTOR TAKES: the kind of the stated turn it follows (Path::modelTurn). Turn::None: a lane, or a
    // connector following no stated turn.
    Turn turnOf(PathId path) const {
        if (path.index >= laneLevel.paths.size()) return Turn::None;
        const uint32_t t = laneLevel.paths[path.index].modelTurn;
        return t < turns.size() ? turns[t].turn : Turn::None;
    }
    // THE MODEL'S JUNCTION A JUNCTION OF THE LANE LEVEL IS (null: none), and the control it states (empty: none).
    const ModelJunction* junctionOf(JunctionId laneJunction) const {
        if (laneJunction.index >= laneLevel.junctions.size()) return nullptr;
        const uint32_t j = laneLevel.junctions[laneJunction.index].modelJunction;
        return j < junctions.size() ? &junctions[j] : nullptr;
    }
    const std::string& controlAt(JunctionId laneJunction) const;
    // THE JUNCTION OF THE LANE LEVEL A JUNCTION OF THE MODEL IS (not valid: none): the one whose `modelJunction` names it.
    // What a reader holding a model junction (a signal head attached to one of its arms, ADR-0456) asks to read the
    // lane level's paths there. A walk over the lane level's junctions: ask once, not per tick.
    JunctionId laneJunctionOf(ModelJunctionId junction) const;
    // DO TWO APPROACHES TO A JUNCTION FACE EACH OTHER (their traffic meets head on: "oncoming")? `a` and `b`: two lane
    // paths arriving at junction `laneJunction` of the lane level. Yes where the model's junction STATES their two arms
    // (Path::arm of each) as one road carrying on through (ModelJunction::through, read by throughPartner). The one
    // answer (ADR-0438): the builder places a far turn's waiting point inside the junction by it
    // (lanes/lane_graph_crossings.h placeWaitInsidePoints) and the driving's right-of-way table and signal programme
    // ask it (apps/driving/right_of_way.h approachesFace). Never the angle between the two; and the lane table keeps
    // no copy of the pair. No junction of the model, or an arm not stated: no.
    bool facing(JunctionId laneJunction, PathId a, PathId b) const {
        const ModelJunction* J = junctionOf(laneJunction);
        if (!J || a.index >= laneLevel.paths.size() || b.index >= laneLevel.paths.size()) return false;
        const uint32_t armA = laneLevel.paths[a.index].arm, armB = laneLevel.paths[b.index].arm;
        return armA != kNoIndex && armB != kNoIndex && throughPartner(J->through, armA) == armB &&
               throughPartner(J->through, armB) == armA;
    }
    // THE PATH THAT JOINS BY A STATED MERGE at a crossing point: where the crossing names a merge of this model
    // (CrossingPoint::modelConnection), the one of its two paths that is a stretch of the lane the connection says
    // merges (ModelConnection::from) -- the path that gives way. Not valid: no merge is stated there.
    PathId mergingAt(const CrossingPoint& x) const;
    // THE JUNCTION A LANE PATH ARRIVES AT (null: it arrives at none, or states no stretch of its street): the junction
    // its street names at the end the path runs to (its `to` end for a path running along the spine, its `from` end
    // for one running against it), for a path that carries an arm (Path::arm).
    const ModelJunction* arrivesAt(PathId path) const;
    // THE REGULATIONS ALONG A PATH (ADR-0437), in order along it (by where each begins, then where it ends, then by
    // kind name): what the driving binds to its rules. Two stated facts answer it, each stated once:
    //   - the signs the arm it arrives by states (ModelArm::regulations of arrivesAt's junction), each standing at
    //     the path's end (from == to == its length), one of each name;
    //   - the regulations stated on its street (`regulations`) for traffic running its way, whose stretch of the
    //     spine overlaps the path's own (Path::stationFrom, stationTo) or, for one standing at one place, lies on it:
    //     the part on this path, each station placed along the path in proportion (exact on a straight road; on a
    //     bend out by the difference between the lane's length and the spine's over that stretch).
    // A connector, or a lane stating no stretch of spine, has none.
    std::vector<PathRegulation> regulationsAlong(PathId path) const;

    // ---- WHAT A READER ASKS OF THE JUNCTIONS AND STREETS (ADR-0436) -------------------------------------------------
    // THE WAY TO READ THE ROADS: a reader asks these (and the questions below this struct: lanesOf, leadsTo,
    // streetName, namedArmsAt, streetNameNear, armMouth, laneSpanEnd) instead of walking the tables for itself, so
    // each question has one answer. Each was a loop two or more readers wrote out.
    //
    // THE STREET AN ARM IS (null: the junction states none for it, or one this model does not have).
    const ModelStreet* streetOf(const ModelArm& arm) const {
        return arm.street.index < streets.size() ? &streets[arm.street.index] : nullptr;
    }
    // THE ARM A STREET IS AT THE JUNCTION IT NAMES at one of its ends (`end` 0: its `from`, 1: its `to`): its place in
    // that junction's stated list of arms -- the arm of this street whose spine runs out of the junction at the
    // street's `from`, into it at its `to`. The first such arm listed. -1: the street names no junction there, or the
    // junction lists no arm for it.
    int armAt(ModelStreetId street, int end) const;
    // THE ARM REACHED AT A STATION OF A STREET (ADR-0437): where `station` (along the street's road's spine) is one of
    // the street's two ends, to the written centimetre, the arm the street is there (armAt); -1 anywhere else along
    // it. What a link of the street map asks to learn the arm it arrives by: it carries its street and the station at
    // its far end (NavLink::street, stationTo), and keeps no copy of the arm.
    int armReachedAt(ModelStreetId street, Real station) const;
    // A STREET'S CENTRE LINE (or a stretch inside a junction's: ModelStretch::line), `from` to `to`, as plan points.
    // Empty: none kept.
    std::vector<Vec2> centreLine(const Range& line) const;
    // WHERE AN END OF A STREET IS in plan (`end` 0: its `from`, 1: its `to`): the first or last point of its centre
    // line. False: the street has no centre line.
    bool streetEnd(const ModelStreet& street, int end, Vec2& out) const;

    // ---- KERB PARKING SPACES AND THE WAY FROM A DOOR TO THE ROAD (ADR-0439) --------------------------------------------
    // THE SPACES ALONG A STREET: the kerb of the traffic running along the spine first, then the other kerb; each kerb's
    // in order along the spine (so by index). Empty: the street's class does not park, or no space fits.
    std::vector<KerbSpace> kerbSpacesOf(ModelStreetId street) const;
    // ONE SPACE BY ITS ID (not valid: there is none).
    KerbSpace kerbSpace(ModelStreetId street, int dir, uint32_t index) const;
    // A SPACE'S NAME, for a person and for a log: "<street id> along <index>" for the kerb of the traffic running along
    // the spine, "<street id> against <index>" for the other. Empty for a space that is not valid.
    std::string kerbSpaceName(const KerbSpace& space) const;
    // WHERE A CAR STANDING IN A SPACE IS: on the parking band's middle line (the street's centre line at the space's
    // middle station, moved toward that kerb by the cross-section's parkingOffset), facing the way the traffic beside
    // it runs. False: the space is not valid, or its street has no centre line.
    bool kerbSpacePose(const KerbSpace& space, KerbSpacePose& out) const;
    // THE KERB LANE BESIDE A STATION OF A STREET, for the traffic running `dir`: the lane path of the lane level on that
    // street running that way whose stretch of the spine holds the station, the one nearest the kerb (the last of its
    // section), and where along it the station falls (alongPathAt). Where a car stops to turn into a space or to set
    // someone down, and where it joins the traffic again. The built layer's: not valid on a planned-only model.
    KerbLanePlace kerbLaneAt(ModelStreetId street, int dir, Real station) const;
    // ...beside a space: at its middle station.
    KerbLanePlace kerbLaneBeside(const KerbSpace& space) const { return kerbLaneAt(space.street, space.dir, space.middle()); }
    // HOW FAR ALONG A LANE PATH A STATION OF ITS ROAD'S SPINE FALLS (metres from the path's start, the way it is
    // driven): in proportion between the stations at the path's two ends, held to the path. Exact on a straight road;
    // on a bend out by the difference between the lane's length and the spine's over that stretch (ADR-0437). The one
    // placing of a station on a path: the regulations' and the kerb lane's. A path stating no stretch of spine: 0.
    Real alongPathAt(PathId path, Real station) const;
    // THE SPACE NEAREST A PLAN POINT, by where a car in it stands, no farther than `reach` metres in a straight line.
    // Not valid (its `space`): none is that near. A search over the streets' centre lines (their boxes first).
    KerbSpaceNear nearestKerbSpace(const Vec2& p, Real reach) const;

    // EVERY STRETCH OF ROAD CARS DRIVE THAT HAS A CENTRE LINE, in the one order the searches by position use: each
    // street, then each junction's inside stretches.
    std::vector<RoadLine> roadLines() const;
    // WHERE A PLAN POINT (A DOOR) FRONTS THE ROAD: the nearest road a building may front (buildingMayFront), by its
    // centre line -- the same search over the same lines, in the same order, as the lot pass faces its doors by
    // (LotRoads::nearestFrontable), so a door built facing a street is answered that street. The station and kerb come
    // from frontageOnLine, as the lot pass's do. The lot pass now records them on each building and door
    // (StatedFrontage, ADR-0474); a reader of a lot hands them to frontageOn below and nothing is searched. This search
    // stays for what states nothing.
    RoadFrontage frontageAt(const Vec2& door) const;
    // ...the same answer from stated facts, with no search: a street, a station along its spine and a kerb.
    RoadFrontage frontageOn(ModelStreetId street, Real station, int dir) const;
    // THE NEAREST PLACE TO WALK beside a frontage: the middle of the sidewalk on the point's side of the road, level
    // with it (the centre line moved toward that kerb by the carriageway's half width and half the sidewalk). False:
    // that road has no sidewalk.
    bool sidewalkBeside(const RoadFrontage& f, Vec2& out) const;
    // THE KERB LANE BESIDE A FRONTAGE: where a car sets someone down for that door (kerbLaneAt, on the door's side).
    KerbLanePlace kerbLaneBeside(const RoadFrontage& f) const {
        return f.street.valid() ? kerbLaneAt(f.street, f.dir, f.station) : KerbLanePlace();
    }
    // THE KERB SPACES NEAR A FRONTAGE, nearest first BY WALKING DISTANCE, no farther than `reach` metres on foot and at
    // most `most` of them. The walk is measured on the model's streets: along the frontage's street and on through the
    // junctions and carry-ons at its ends along every street with a sidewalk, to the space's middle station; a space
    // on the other kerb adds its street's travel width (the walk across). It is the model's own measure of nearness,
    // made from stated facts; the walker's real path comes from the street map when the walk is planned. Ties: by
    // street, kerb and index.
    std::vector<KerbSpaceNear> kerbSpacesNear(const RoadFrontage& f, Real reach, std::size_t most) const;

    // ---- WHERE A PLACE JOINS THE WALKS (ADR-0468, issue #300) -----------------------------------------------------------
    // THE ONE ANSWER to "where does this place join the sidewalks", for placing a place, starting a walk from it and
    // ending one at it (WalkJoin above). Built layer only (it reads the lane level's walks).
    //
    // FROM STATED FACTS: a frontage stated as a street, a station along its spine and a kerb (frontageOn). The walk
    // along that street on that kerb whose stretch of spine holds the station; past the stretch (a place by a junction,
    // nearer it than where a person steps off the kerb; a sidewalk that stops short), the nearest end of that kerb's
    // walk along the street, `held` saying how far. Where that kerb has no walk the mover may use, the other kerb's,
    // `across`. A footway's walk serves both kerbs. `openWalks`: per walk, non-zero where the mover may use it
    // (RoadModel::openTo(mover).walks; empty: every walk); `parts`, when given, the parts of the walks the mover can get
    // round (reachableWalks over the same open walks), so the answer says when it lies in a part cut off from the rest.
    // Ties: the lower walk id.
    WalkJoin walkJoinOn(const RoadFrontage& f, const std::vector<uint8_t>& openWalks = {},
                        const ReachableParts* parts = nullptr) const;
    // ...FROM WHAT A PLACE STATES (StatedFrontage: a lot's building or door, ADR-0474). On a walkway (ADR-0475): the
    // walk of that walkway whose stretch holds the stated metres along it (a walkway cut where others meet it is several
    // walks), `held` past its ends, `walkway` set and no street. On a street: the answer above, from frontageOn. Neither:
    // not joined, NoStreet; a walkway the lane level does not hold, NoWalkway.
    WalkJoin walkJoinOn(const StatedFrontage& f, const std::vector<uint8_t>& openWalks = {},
                        const ReachableParts* parts = nullptr) const;
    // THE FALLBACK, BY POSITION: only where nothing states a place's frontage (a lot's door: the lot pass records no
    // street; a person standing anywhere). The frontage is the one search by position (frontageAt: the nearest road a
    // building may front, never a freeway or a ramp), then walkJoinOn; `searched` set and `distance` the metres from
    // `p` to the point joined. It never joins a walk the mover may not use, and says when it is in a cut-off part.
    WalkJoin walkJoinNear(const Vec2& p, const std::vector<uint8_t>& openWalks = {},
                          const ReachableParts* parts = nullptr) const;
    // HOW FAR ALONG A WALK THAT RUNS ALONG A STREET a station of that street's road's spine falls (metres from the walk's
    // start, held to the walk): read off the stations of the sidewalk or footway lane it was cut from, point by point,
    // as the lane level cut it (lanes/lane_level.cpp). A walk along no street: 0.
    Real walkAlongAt(WalkId walk, Real station) const;

    // ---- CROSSWALKS (ADR-0452) ------------------------------------------------------------------------------------------
    // THE CROSSWALK ACROSS AN ARM of a junction (null: none stated there).
    const ModelCrosswalk* crosswalkAcross(ModelJunctionId junction, uint32_t arm) const;
    // ONE BY ITS ID (null: none).
    const ModelCrosswalk* crosswalk(uint32_t id) const { return id < crosswalks.size() ? &crosswalks[id] : nullptr; }
    // THE CROSSWALK A RECORD OF THE LANE LEVEL IS (kNoIndex: none; a lane table made by hand states it).
    uint32_t crosswalkOfLane(uint32_t laneCrosswalk) const {
        return laneCrosswalk < laneLevel.crosswalks.size() ? laneLevel.crosswalks[laneCrosswalk].modelCrosswalk : kNoIndex;
    }
    // THE PATHS THAT CROSS A CROSSWALK (built layer), in the lane level's order of its crossing points: what the road
    // build's crossing finder found for its outline. Empty: none, or a planned-only model.
    std::vector<CrosswalkCrosser> crossersOf(uint32_t crosswalk) const;
    // THE CROSSWALKS A WALK ROUND A JUNCTION CROSSES, in the order crossed, by a person who arrives by arm `armIn` on
    // the pavement on the kerb side of that arm's arriving traffic and leaves by arm `armOut` on the kerb side of its
    // leaving traffic -- the way the street map's people walk (rightOf their way: the kerb side). They go round the
    // junction's corners in its stated ring (ADR-0427) the way that crosses fewer arms (a tie: toward the arriving
    // traffic's kerb), crossing each arm between at its crosswalk; an arm with no crosswalk where people reach it is
    // passed over. The kerb side is the one statement of it (lane_turns.h kerbSideCounterClockwise). Empty: a turn round
    // the corner, an arm out of range, or a junction with no crosswalk on the way.
    std::vector<WalkCrossing> crosswalksOnWalk(ModelJunctionId junction, int armIn, int armOut) const;

    // ---- WHO MAY USE WHICH PATH OF THE LANE LEVEL (ADR-0466) -------------------------------------------------------------
    // THE KIND OF A LANE PATH, as the path users' table names it: a lane's, its model lane's kind (through, turn, ramp...);
    // a connector's, kConnectorKind. Empty: none stated (a hand-made table).
    const std::string& pathKind(PathId path) const;
    // THE KIND OF A WALK: a sidewalk's or corner's, its model walk's; a footway's, its lane's; a crossing, kCrossingKind;
    // a turn-round, the path users' dead-end join (pathUsers().deadEndJoin); a walkway's, its stated kind. Empty: none.
    const std::string& walkKind(WalkId walk) const;
    // EVERY LANE PATH AND WALK MOVER `mover` MAY USE, by its kind and the path users' table `users` (per lane path and per
    // walk, 1: open). A mover or kind the table does not name: closed. The one mover filter: a car's route sees no walk
    // unless the table opens a walk's kind to the vehicles, and a person's no lane path unless one is opened to walkers.
    struct OpenPaths {
        std::vector<uint8_t> paths, walks;
    };
    OpenPaths openTo(const std::string& mover, const PathUsers& users = pathUsers()) const;
};
// The kinds of path the lane level names that are not a lane's (assets/data/path_users.json lists who may use each): a
// connector through a junction, and a crossing over a crosswalk (ADR-0466).
extern const std::string kConnectorKind;
extern const std::string kCrossingKind;
// WHERE A PERSON STEPS OFF THE KERB AT A JUNCTION'S ARM, along its road's spine (ADR-0465, ADR-0466): the middle of its
// crosswalk's band, where the crosswalk's kerb ends lie (ModelCrosswalk::ends); with none, or on a footway (whose
// middle a person walks on), the arm's stated box edge, else its mouth. kNotStated: no such arm. The one statement: a
// corner walk ends there (lanes/walk_lanes.cpp) and the lane level cuts a sidewalk or footway there (lane_level.cpp).
Real kerbEndStation(const RoadModel& m, ModelJunctionId junction, uint32_t arm);

// STATES A CROSSWALK on the model by hand (a test's lane table, which has no road file): refused (false, why in `err`)
// when it names no junction of the model or an arm the junction does not list. Appended to `crosswalks`; returns its id
// in `id`. (A level's crosswalks are stated by plannedRoadModel, never by this.)
bool stateCrosswalk(RoadModel& m, const ModelCrosswalk& c, uint32_t* id = nullptr, std::string* err = nullptr);
// THE CROSSWALKS LAID INTO THE LANE LEVEL (ADR-0452): for every crosswalk of a junction of the lane level `g` (by
// Junction::modelJunction), its outline -- the band from its near to its far edge, kerb to kerb -- as a lane-level
// Crosswalk record naming it (Crosswalk::modelCrosswalk), filed under its junction, and the model's record told its
// place (laneCrosswalk). The road build's crossing finder then finds the paths that cross each (lanes/lane_graph_crossings.h
// findCrossingPoints). THE ONE PLACE a crosswalk becomes a lane-level record: buildLaneLevel calls it, and a test's lane
// table made by hand calls it the same way. Replaces any crosswalks the lane level held.
void layLaneCrosswalks(RoadModel& m);

// STATES THE WALKWAYS on a built model (ADR-0475): THE ONE PLACE they are supplied. Kept in RoadModel::walkways and laid
// in its lane level at once (lanes/lane_level.h layWalkways): each walkway as a walk of its kind, cut and joined where
// its ends say. Returns what did not land, with why; a walkway wrong as a whole is kept but not laid. Once per lane
// level: the lane level must be laid again (buildLaneLevel) before stating them anew.
std::vector<WalkwayProblem> stateWalkways(RoadModel& m, std::vector<ModelWalkway> walkways);

// Field for field, exactly (the saved bytes of each compared).
bool operator==(const RoadModel& a, const RoadModel& b);
inline bool operator!=(const RoadModel& a, const RoadModel& b) { return !(a == b); }

// THE PLANNED LAYER: everything the road file states, read from it alone (no road build: milliseconds). `g` with its ramps
// composed (lane_expand.h expand), so stations are the builder's. The built parts empty; `built` false.
RoadModel plannedRoadModel(const roads::lanes::RoadLabGraph& g);
// THE MODEL OF A BUILT ROAD NETWORK: the planned layer of the build's graph, everything the build states (see the top
// of this file), and the lane level built from those (lanes/lane_level.h), its turns checked against `opts`' vehicles.
// The one way to get a built model. `report`, when given, receives what building the lane level found (problems,
// corner faults, where each path came from: for a tool or a test; the model does not keep it).
RoadModel assembleRoadModel(const roads::lanes::Result& r, const roads::lanes::LaneGraphOptions& opts,
                            roads::lanes::LaneGraphBuild* report = nullptr);
// THE BUILT LAYER WITHOUT THE LANE LEVEL: the planned layer and everything the build states (lanes, connections, junction
// heights, the stated turns), `laneLevel` left empty. What assembleRoadModel builds the lane level from, and what the
// road paint draws from (lanes/deck_mesh.h buildMeshes: the arrows are RoadModel::turns, the stop bars the arms' stated
// control and signs, the lane lines the lanes' kinds and stop lines), so the paint and the lane level read one model.
RoadModel builtRoadModel(const roads::lanes::Result& r);
// WHICH ARM A LANE MEETS AT AN END OF A STREET it runs along (`end` 0: the street's `from`, 1: its `to`), and that arm's
// stop line. THE ONE PLACE this is read off the stated streets and junctions: the turns' arms (turnArmsOf), the lane
// level's cut at each stop line (lanes/lane_level.cpp) and the paint's lane lines (lanes/deck_mesh.cpp) all ask it.
struct LaneSpanEnd {
    bool reaches = false;            // the lane's stretch comes to that end of the street (sameWrittenStation)
    ModelJunctionId junction;        // the junction the street names there (none: it carries on, or the road just ends)
    int arm = -1;                    // the arm of that junction this street is there (-1: the junction lists none for it)
    Real stopLine = kNotStated;      // that arm's stop line, along the road's spine (not stated: its road is not in the file)
};
LaneSpanEnd laneSpanEnd(const RoadModel& m, const ModelLaneSpan& sp, int end);
// WHAT IS AT AN END OF A STREET (`end` 0: its `from`, 1: its `to`) -- THE ONE ANSWER (ADR-0464), from stated facts and
// the design stage's one test for a road end another road meets (lanes/road_design.h roadEndMeetsAnother): a junction
// the street names there; the street it carries on into; a ramp's end anchored to its host; the road just ends (a dead
// end); or the street stops where the road file states nothing though another road meets its road's end there, or part
// way along its road (both the planner's to state: the lane level reports them). `g`: the road file the street is of.
// "Meets" is at grade, by the planner's crossing policy (road_design.h meetAtGrade, ADR-0469): a street ending under
// the freeway's band (the freeway meets no road at grade) is a dead end, not a junction the planner left unstated.
enum class StreetEndKind : uint8_t { Junction, CarriesOn, RampAnchored, DeadEnd, MeetsAnother, PartWay };
StreetEndKind streetEndKind(const roads::lanes::RoadLabGraph& g, const ModelStreet& s, int end);
const char* streetEndKindWord(StreetEndKind k);
// EVERY PAIR OF ARMS NEXT TO EACH OTHER ROUND A JUNCTION'S STATED RING, and whether a corner is stated between them, or
// why not -- THE ONE DERIVATION of corners (ADR-0464): the planned layer keeps the stated ones (RoadModel::corners); a
// tool or a test asks for the rest. `g`: the road file `m` was read from (an arm's point of a spread junction is found
// by its road and station among the junction's stated points).
enum class CornerGap : uint8_t {
    None,              // stated: a corner
    NoPavement,        // an arm's street has no walk along the kerb facing the corner (a freeway, a ramp, a track)
    NoStreet,          // an arm names no street, so nothing says what it is made of
    OneArm,            // a junction of one arm has no corner
    RingNotStated,     // the road file does not state that its arms are listed in their ring (ADR-0427)
    ArmAtNoPoint,      // a spread junction states its points, and none holds the arm's road and station
    ApartAtSpread,     // the two arms meet the junction at different points of a spread junction: no walk between the
                       // points is stated, so nothing says where the kerb runs between them
};
constexpr uint8_t kCornerGapCount = 7;
const char* cornerGapWord(CornerGap g);
struct RingPair {
    ModelJunctionId junction;
    uint32_t armA = 0, armB = 0;
    int kerbA = 1, kerbB = 1;
    CornerGap gap = CornerGap::None;
};
std::vector<RingPair> ringPairs(const RoadModel& m, const roads::lanes::RoadLabGraph& g);
// THE DEAD ENDS OF `m`'s streets (streetEndKind DeadEnd), street by street, `from` first, each joined across where
// `users` names a dead-end join and the street is a road cars drive with a sidewalk on both kerbs -- THE ONE DERIVATION
// (ADR-0464): the planned layer states it with the shipped table (RoadModel::deadEnds); a test hands in another table.
std::vector<ModelDeadEnd> deadEndsOf(const RoadModel& m, const roads::lanes::RoadLabGraph& g, const PathUsers& users);
// EACH JUNCTION'S ARMS AS THE TURN TABLE READS THEM (lane_turns.h TurnArm), in the stated ring order (the list's own): each
// arm's heading and the built lanes arriving and leaving by it. What assembleRoadModel states the turns from.
std::vector<std::vector<TurnArm>> turnArmsOf(const RoadModel& m);

// ATTACHES `a` to the model: refused (false, and why in `err`) when its kind is not in `kinds`, or it is not on what its
// kind attaches to (a street kind needs a street of the model; a junction kind a junction; an arm kind a junction and one
// of its arms, and then its street is the arm's).
bool attach(RoadModel& m, ModelAttachment a, const AttachmentKinds& kinds, std::string* err = nullptr);
bool attach(RoadModel& m, ModelAttachment a, std::string* err = nullptr);   // the shipped kinds
// STATES A REGULATION on a street of the model (ADR-0437): refused (false, and why in `err`) when it names no kind, no
// street of the model, a way that is neither along the spine (+1) nor against it (-1), or a stretch that runs
// backwards or leaves its street (the street's `from` to `to`, to the written centimetre).
bool stateRegulation(RoadModel& m, const ModelRegulation& r, std::string* err = nullptr);

// ---- the questions (design section 6) ---------------------------------------------------------------------------------
// The lanes of a street, forward then backward, each by slot from the centre line.
std::vector<ModelLaneId> lanesOf(const RoadModel& m, ModelStreetId s);
// What a lane leads to, from stated facts only: the junction its traffic reaches at its end (none: it ends otherwise),
// the connections that leave it (a ramp leaving it, it merging into a host, a declared connector), and the turns it
// takes through that junction (ADR-0426).
struct LeadsTo {
    ModelJunctionId junction;
    std::vector<uint32_t> connections;   // indices into RoadModel::connections
    std::vector<uint32_t> turns;         // indices into RoadModel::turns
};
LeadsTo leadsTo(const RoadModel& m, ModelLaneId lane);
// WHAT IS ATTACHED to a street (on it, or on a junction arm that is it): indices into RoadModel::attachments, in the
// order attached.
std::vector<uint32_t> attachedToStreet(const RoadModel& m, ModelStreetId s);
// A STREET'S NAME: the text of its attachment whose kind names the street (empty: it carries none).
std::string streetName(const RoadModel& m, ModelStreetId s, const AttachmentKinds& kinds = attachmentKinds());
// THE NAMED STREETS AT A JUNCTION, arm by arm in order round it (by heading): each arm whose street has a name, with
// where it meets the junction (its street's centre line at that end), the way it leaves (its stated heading), and the
// name. What the name signs read. (How wide the street is is asked of the street: its section's travelWidth().)
struct NamedArm {
    int arm = -1;
    ModelStreetId street;
    Vec2 point, dir;
    std::string name;
};
std::vector<NamedArm> namedArmsAt(const RoadModel& m, ModelJunctionId j);
// THE NAMED STREET NEAREST a plan point, by the streets' centre lines (planned layer), within `reach` metres; its name,
// and `distance` (optional, out). Empty when none is in reach.
std::string streetNameNear(const RoadModel& m, const Vec2& p, Real reach = 40, Real* distance = nullptr);
// WHERE AN ARM MEETS ITS JUNCTION: its street's centre line at that end (false: the arm has no street with a line).
bool armMouth(const RoadModel& m, const ModelJunction& J, std::size_t arm, Vec2& out);

// Lookups by name.
ModelStreetId streetNamed(const RoadModel& m, const std::string& id);
ModelLaneId laneNamed(const RoadModel& m, const std::string& id);

// ---- in words and on a map ---------------------------------------------------------------------------------------------
std::string dumpRoadModel(const RoadModel& m);     // every junction, street, lane, connection, ramp and crossing
std::string reportRoadModel(const RoadModel& m);   // counts, what is not stated, the problems
std::string roadModelSvg(const RoadModel& m);      // junctions and streets with their ids, lanes, kerb parking spaces
// THE KERB PARKING SPACES IN WORDS (ADR-0439), street by street and kerb by kerb: each space's name, stations, where a
// car in it stands and faces, and the kerb lane path beside it (lanes_tool model writes them as parking.txt).
std::vector<std::string> kerbSpaceLines(const RoadModel& m);
// How much kerb parking a model holds: the streets whose cross-section states a parking band, how many of them hold at
// least one space, the spaces, and the metres of kerb they take.
struct KerbSpaceCounts {
    uint32_t streetsWithBand = 0, streets = 0, spaces = 0;
    Real kerbMetres = 0;
};
KerbSpaceCounts kerbSpaceCounts(const RoadModel& m);

// ---- saved and read back ------------------------------------------------------------------------------------------------
// The layout of the bytes (magic "RMDL"). Bump when a field changes.
constexpr uint32_t kRoadModelFormat = 18;   // 18: the walkways; a walk may be a walkway's (LGRF 17, ADR-0475)
                                            // 17: the lane level's walks and their joins (LGRF 16, ADR-0466)
                                            // 16: the walks; a corner's walk and whether it is across a spread junction (ADR-0465)
                                            // 15: the corners, the dead ends, a street's sidewalk per kerb (ADR-0464)
                                            // 14: the crosswalks; the lane level's name the model's (LGRF 15, ADR-0452)
                                            // 13:the kerb parking runs; the lane level holds no car spaces (LGRF 14, ADR-0439)
                                            // 12: the lane level holds no through partner (LGRF 13, ADR-0438)
                                            // 11: the regulations stated on streets; the lane level holds none (LGRF 12, ADR-0437)
                                            // 10: the lane level holds no copy of the model's facts (LGRF 11, ADR-0436)
                                            // 9: no stored one-way flag: the cross-section answers it (ADR-0432)
                                            // 8: the lane level, as the lane graph's own bytes (LGRF) after the rest (ADR-0429)
                                            // 7: whether the file states its arms in their ring (ADR-0427)
                                            // 2: a street's kerb parking (ADR-0418); 3: where a street carries on (ADR-0419);
                                            // 4: the planned layer's centre lines, engine class and attachments (ADR-0420);
                                            // 5: a spread junction's points, inside stretches' lines and sections, the centre
                                            //    lines' stations, a ramp's gore along itself, the parking band's middle (ADR-0425)
                                            // 6: the turns through each junction, where a diverge or merge meets its host (ADR-0426)
// The lane level is written last, by the lane graph's own writer, and read back by its reader, which checks it
// (validate): a model whose lane level fails that check does not read back. (What assembleRoadModel makes is versioned
// by kLaneGraphBuildVersion, lanes/lane_graph_build.h, which the cache key holds: lanes/road_model_cache.h.)
void putRoadModel(bundle::BinWriter& w, const RoadModel& m);
bool getRoadModel(bundle::BinReader& r, RoadModel& m, std::string* err);

}  // namespace engine

#endif
