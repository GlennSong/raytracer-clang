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
// Where nothing states a part yet (the turns' drawn paths, which the lane level shapes; crosswalks, the parking spaces
// along a kerb, blocks, signal groups), it is EMPTY, and the model's `notStated` list says so. Street
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

// One lane's stretch on one street.
struct ModelLaneSpan {
    ModelStreetId street;
    Real from = 0, to = 0;           // stations along the road's spine
};

// THE MODEL'S LANE: the lane's facts (road_model_parts.h LaneFacts: id, kind, road, direction, slot, width, stretch of
// spine -- the builder's lane's, taken whole) and where it lies: its centre line as kept, the streets it runs along and
// the junction at each end.
struct ModelLane : LaneFacts {
    Range curve;                     // into points / along / heights: its centre line, the way it is laid (spine order)
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
    // the lanes' centre lines (ModelLane::curve ranges into these): each point, its distance along the lane, its height
    // (the drawn deck), and the station along its road's spine it was laid at (a lane is laid point for point beside its
    // spine's samples; a connector's: kNotStated)
    std::vector<Vec2> points;
    std::vector<Real> along, heights, spineStations;
    std::vector<Vec2> linePoints;         // the streets' centre lines (ModelStreet::line ranges into it), and inside stretches'
    std::vector<Real> lineStations;       // ...each point's station along its road's spine (ADR-0425)
    std::vector<ModelAttachment> attachments;   // in the order attached
    std::vector<std::string> notStated;   // the parts of the design nothing states yet, in words
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
};

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
// EACH JUNCTION'S ARMS AS THE TURN TABLE READS THEM (lane_turns.h TurnArm), in the stated ring order (the list's own): each
// arm's heading and the built lanes arriving and leaving by it. What assembleRoadModel states the turns from.
std::vector<std::vector<TurnArm>> turnArmsOf(const RoadModel& m);

// ATTACHES `a` to the model: refused (false, and why in `err`) when its kind is not in `kinds`, or it is not on what its
// kind attaches to (a street kind needs a street of the model; a junction kind a junction; an arm kind a junction and one
// of its arms, and then its street is the arm's).
bool attach(RoadModel& m, ModelAttachment a, const AttachmentKinds& kinds, std::string* err = nullptr);
bool attach(RoadModel& m, ModelAttachment a, std::string* err = nullptr);   // the shipped kinds

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
std::string roadModelSvg(const RoadModel& m);      // junctions and streets with their ids, lanes

// ---- saved and read back ------------------------------------------------------------------------------------------------
// The layout of the bytes (magic "RMDL"). Bump when a field changes.
constexpr uint32_t kRoadModelFormat = 9;    // 9: no stored one-way flag: the cross-section answers it (ADR-0432)
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
