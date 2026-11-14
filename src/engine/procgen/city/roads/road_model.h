#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_MODEL_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROAD_MODEL_H

// THE ROAD MODEL (docs/road-model-design.md, ADR-0417): one description of a level's roads, with ids that never change,
// that later readers will query instead of each working the roads out again. This is step 2 of the design: the model
// exists BESIDE the old forms (the road twin, the lane match, the lane graph) and nothing reads it yet.
//
// Words used here:
//   - a JUNCTION is where roads meet at one level; its ARMS are the roads' ways out of it, in order round it;
//   - a STREET is one stretch of road from a junction to the next (or a dead end): a road-file edge cut at its junctions;
//   - a LANE is one line of travel along a road, with its real curve and heights; it runs along one or more streets;
//   - a STATION is distance along a road-file edge's centre line (its spine), in metres;
//   - a lane's SLOT is its place across the road among the lanes running its way, counted from the centre-line side
//     (0) to the kerb side;
//   - a CONNECTION is how one lane leads to another: a ramp's lane merging into or leaving its host's lane, or a lane the
//     road file joins to another by a declared connector;
//   - a STOP LINE is where traffic arriving by an arm stops: the paint's, kStopLineBack outside its junction box.
//
// WHO STATES WHAT (design section 2). Everything here is copied from a stage that states it, never worked out:
//   - the road file (the planner, or the one-off layout tool): junctions and their arms, headings and through pairs,
//     streets, classes and cross-sections (with their kerb parking, ADR-0418), one-way, speed limits, controls and signs,
//     crossings, ramps and their anchors;
//   - the road builder: each lane's curve, heights and slot, the streets it runs along, the junction at each end, the
//     ramps' merges (host lane, station, length, taper), the paint's junction boxes and stop lines.
// Where nothing states a part yet (turn paths through junctions, which lines between lanes may be crossed, crosswalks,
// the parking spaces along a kerb, blocks, signal groups), it is EMPTY, and the model's `notStated` list says so. Street
// names are stated since ADR-0420 (the road file's streets[].name), as ATTACHMENTS (below). Where the build
// does not match what was stated (a box the paint found that no stated junction holds, an arm with no box), it is in
// `problems`, with its place.
//
// IDS. A junction or street is named by the road file's id; a lane by the builder's lane id (edge.f0, edge.pocket, ...).
// Inside the model they are indices into its arrays (GraphIndex, as the lane graph uses), stable for one road file.
//
// TWO LAYERS, ONE MODEL (ADR-0420). The PLANNED LAYER is everything the road file alone states -- junctions, arms,
// streets and their centre lines, cross-sections, one-way, carrying on, through pairs, divided roads, ramps, crossings,
// names -- read in milliseconds (plannedRoadModel), and published when a level loads (components.h LevelRoadModel), for
// the readers that only label the roads (street names, name signs, the map). The BUILT LAYER is what the road build adds
// -- lanes with their curves and heights, connections, junction heights, the paint's boxes and stop lines -- assembled
// from a build (assembleRoadModel = the planned layer + the built parts), stored in its own cache file (cache/roadmodel,
// below), keyed by the city's bake key, the layout's bytes and kRoadModelVersion; not in the level's bake yet (that would
// re-bake every level). A planned-only model is the same structure with its built parts empty, `built` false, and its
// `notStated` list says which.
//
// ATTACHMENTS (design section 1: "a bus stop is street 41, station 120, kerb side"). Anything that belongs to a street or
// a junction is attached to it by id: its KIND is data (assets/data/road_attachments.json: what it may attach to -- a
// street, a junction, or an arm of a junction), never a branch in code. The first users: a street's NAME (from the road
// file) and a NAME SIGN at a junction arm (the loader's sign plan).

#include "engine/procgen/city/road_network.h"        // RoadClass: the engine's class of a street
#include "engine/procgen/city/roads/lane_graph.h"   // GraphIndex, Range, kNoIndex: the lane graph's index types

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace engine {

namespace roads::lanes {
struct Result;
struct RoadLabGraph;
}  // namespace roads::lanes
namespace bundle {
struct LevelInputs;
}  // namespace bundle

struct ModelJunctionTag {};
struct ModelStreetTag {};
struct ModelLaneTag {};
using ModelJunctionId = GraphIndex<ModelJunctionTag>;
using ModelStreetId = GraphIndex<ModelStreetTag>;
using ModelLaneId = GraphIndex<ModelLaneTag>;

// No station or height stated.
constexpr Real kNotStated = std::numeric_limits<Real>::quiet_NaN();
inline bool stated(Real v) { return !std::isnan(v); }

// ONE ARM: a road's way out of a junction.
struct ModelArm {
    ModelStreetId street;            // the street this arm is (none: the road file names none)
    std::string road;                // the road-file edge
    Real station = 0;                // where the arm meets the junction, along the road's spine
    bool spineIn = true;             // the road's spine runs INTO the junction here (the street ends here)
    Real heading = kNotStated;       // degrees, the way the arm leaves the junction, plan coordinates
    std::vector<std::string> regulations;   // signs standing at the end of each lane arriving by it (the control plan)
    Real boxEdge = kNotStated;       // the paint's: where the road enters the junction's box, along its spine
    Real stopLine = kNotStated;      // the paint's: where traffic arriving by this arm stops, along its spine
};

// A STRETCH OF ROAD INSIDE A JUNCTION (between nodes the planner grouped into it), or one of the paint's boxes along a
// road: an edge and stations from <= to.
struct ModelStretch {
    std::string road;
    Real from = 0, to = 0;
};

struct ModelJunction {
    std::string id;
    Vec2 at;
    Real height = kNotStated;        // the drawn deck at `at`, read off the builder's heights along its first arm's road
    std::vector<ModelArm> arms;      // in order round it (by heading) where the file wrote them so; as written otherwise
    std::vector<std::array<uint32_t, 2>> through;   // pairs of arms (indices into arms) that are one street carrying on
    std::string control, why;        // empty control: the road file plans none
    std::vector<ModelStretch> inside;   // stretches of road inside it
    std::vector<ModelStretch> boxes;    // the paint's junction boxes holding its arms: its extent along each road
};

// What a street is made of across its width (design section 1: the cross-section), from its road's class and the road
// file's lanes. The lanes each way are the TRAVEL lanes: the one lane count the builder lays and the city sim drives
// (ADR-0418). KERB PARKING is a band along each kerb, over the kerb side of the kerb lane each way (roads/lanes
// LaneLayout::parking): `parking` its width where the road states one that fits (streetSection), 0 none.
struct ModelSection {
    std::string roadClass;
    RoadClass klass = RoadClass::Local;        // the engine's class for it (roads/lanes classOf: the one mapping)
    int lanesForward = 0, lanesBackward = 0;   // forward: running along the spine
    Real laneWidth = 0, medianGap = 0, median = 0, shoulder = 0, sidewalk = 0;
    Real parking = 0;                          // the kerb parking band's width along each kerb (m); 0: none
    bool track = false, footway = false;
};

struct ModelStreet {
    std::string id, road;
    uint32_t edge = kNoIndex;        // the road-file edge's index
    Real from = 0, to = 0;           // stations along the road's spine
    std::array<ModelJunctionId, 2> junctions;   // at `from`, at `to` (none: a dead end, or a ramp's anchored end)
    // at `from`, at `to`: the street this one carries on into where no junction is (one road running on as another, or a
    // bend: ADR-0419); none: a junction, a dead end or a ramp's anchored end
    std::array<ModelStreetId, 2> carriesOn;
    ModelSection section;
    bool oneWay = false;
    Real speedLimit = 0;             // metres per second; 0: none posted
    Real lotsFrom = -1, lotsTo = -1; // where buildings may front it (the road's lots_range); -1: all of it
    std::string frontageOf;          // a frontage road: the freeway carriageway it runs beside
    Range line;                      // into linePoints: its centre line, `from` to `to` (planned: the road file's spine)
    std::vector<ModelLaneId> lanes;  // its lanes, forward then backward, each by slot from the centre line (built layer)
    // The width the travel lanes take across, both ways (lanes x width + the median gap): what the map draws.
    Real travelWidth() const {
        return (section.lanesForward + section.lanesBackward) * section.laneWidth + section.medianGap;
    }
};

// WHAT A KIND OF ATTACHMENT IS (assets/data/road_attachments.json): its name, what it attaches to ("street", "junction"
// or "arm"), and whether it is the street's name (what a reader asks for when it wants one).
struct AttachmentKind {
    std::string name, on;
    bool names = false;
};
struct AttachmentKinds {
    std::vector<AttachmentKind> kinds;
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

struct ModelLane {
    std::string id, kind;            // kind: the builder's (through, turn, ramp, connector, ...)
    uint32_t edge = kNoIndex;        // its road (none: a connector)
    int dir = 1;                     // +1 along the road's spine
    int slot = -1;                   // its place across (see the top of this file)
    Real width = 0;
    Range curve;                     // into points / along / heights: its centre line, the way it is laid (spine order)
    Real spineFrom = 0, spineTo = 0; // the stretch of its road's spine it is laid along
    std::vector<ModelLaneSpan> streets;
    ModelJunctionId atStart, atEnd;  // the junction it begins and ends at, the way its traffic runs
};

// HOW ONE LANE LEADS TO ANOTHER, as stated: a ramp's lane leaving its host's lane (Diverge) or merging into it (Merge),
// at a station along the host, over a stated length and taper; or a connector the road file declares (Joins).
enum class ConnectionKind : uint8_t { Diverge, Merge, Joins };
struct ModelConnection {
    ConnectionKind kind = ConnectionKind::Merge;
    ModelLaneId from, to;
    std::string ramp;                // the ramp it belongs to (Diverge, Merge)
    Real hostStation = 0;            // the gore, along the host's spine (Diverge, Merge); the far lane's station (Joins)
    Real length = 0;                 // the lane of its own beside the host: deceleration (Diverge) or auxiliary (Merge)
    Real taper = 0;
};

// A RAMP: the road it leaves and the road it joins (design section 1). Each end: anchored to a lane of its host (a
// merge), ending bare on a street's side (an arm of a junction there), or free (a road end like any other).
enum class RampEndKind : uint8_t { Free, Anchored, OnStreet };
struct ModelRampEnd {
    RampEndKind kind = RampEndKind::Free;
    std::string host, hostLane;      // the road and (anchored) its lane
    Real hostStation = kNotStated;   // the gore, along the host's spine (anchored)
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
    std::vector<ModelRamp> ramps;
    std::vector<ModelCrossing> crossings;
    // the lanes' centre lines (ModelLane::curve ranges into these): each point, its distance along the lane, its height
    // (the drawn deck), and the station along its road's spine it was laid at (a lane is laid point for point beside its
    // spine's samples; a connector's: kNotStated)
    std::vector<Vec2> points;
    std::vector<Real> along, heights, spineStations;
    std::vector<Vec2> linePoints;         // the streets' centre lines (ModelStreet::line ranges into it)
    std::vector<ModelAttachment> attachments;   // in the order attached
    std::vector<std::string> notStated;   // the parts of the design nothing states yet, in words
    std::vector<std::string> problems;    // where the build does not match what was stated, each with its place
};

// Field for field, exactly (the saved bytes of each compared).
bool operator==(const RoadModel& a, const RoadModel& b);
inline bool operator!=(const RoadModel& a, const RoadModel& b) { return !(a == b); }

// THE PLANNED LAYER: everything the road file states, read from it alone (no road build: milliseconds). `g` with its ramps
// composed (lane_expand.h expand), so stations are the builder's. The built parts empty; `built` false.
RoadModel plannedRoadModel(const roads::lanes::RoadLabGraph& g);
// THE MODEL OF A BUILT ROAD NETWORK: the planned layer of the build's graph and everything the build states (see the top
// of this file).
RoadModel assembleRoadModel(const roads::lanes::Result& r);

// ATTACHES `a` to the model: refused (false, and why in `err`) when its kind is not in `kinds`, or it is not on what its
// kind attaches to (a street kind needs a street of the model; a junction kind a junction; an arm kind a junction and one
// of its arms, and then its street is the arm's).
bool attach(RoadModel& m, ModelAttachment a, const AttachmentKinds& kinds, std::string* err = nullptr);
bool attach(RoadModel& m, ModelAttachment a, std::string* err = nullptr);   // the shipped kinds

// ---- the questions (design section 6) ---------------------------------------------------------------------------------
// The street, lane and station nearest a plan point (any lane on any street; within `within` metres, else none), and the
// lane's height there.
struct NearestOnRoad {
    ModelStreetId street;
    ModelLaneId lane;
    Real station = 0;                // along the street's road's spine
    Real distance = 0;               // plan metres from the point to the lane's centre line
    Real height = 0;
};
NearestOnRoad nearestOnRoad(const RoadModel& m, const Vec2& p, Real within = 50);
// The streets at a junction, in order round it (by the arms' headings, counter-clockwise from +x; an arm with no street
// or heading last, as listed).
std::vector<ModelStreetId> streetsAt(const RoadModel& m, ModelJunctionId j);
// The lanes of a street, forward then backward, each by slot from the centre line.
std::vector<ModelLaneId> lanesOf(const RoadModel& m, ModelStreetId s);
// What a lane leads to, from stated facts only: the junction its traffic reaches at its end (none: it ends otherwise),
// and the connections that leave it (a ramp leaving it, it merging into a host, a declared connector). Turns through the
// junction are not stated yet (notStated), so none are listed.
struct LeadsTo {
    ModelJunctionId junction;
    std::vector<uint32_t> connections;   // indices into RoadModel::connections
};
LeadsTo leadsTo(const RoadModel& m, ModelLaneId lane);
// WHAT IS ATTACHED to a street (on it, or on a junction arm that is it) or to a junction (on it, or on one of its arms):
// indices into RoadModel::attachments, in the order attached.
std::vector<uint32_t> attachedToStreet(const RoadModel& m, ModelStreetId s);
std::vector<uint32_t> attachedToJunction(const RoadModel& m, ModelJunctionId j);
// A STREET'S NAME: the text of its attachment whose kind names the street (empty: it carries none).
std::string streetName(const RoadModel& m, ModelStreetId s, const AttachmentKinds& kinds = attachmentKinds());
// THE NAMED STREETS AT A JUNCTION, arm by arm in order round it (by heading): each arm whose street has a name, with
// where it meets the junction (its street's centre line at that end), the way it leaves (its stated heading), its
// street's travel width, and the name. What the name signs read.
struct NamedArm {
    int arm = -1;
    ModelStreetId street;
    Vec2 point, dir;
    Real travelWidth = 0;
    std::string name;
};
std::vector<NamedArm> namedArmsAt(const RoadModel& m, ModelJunctionId j);
// THE NAMED STREET NEAREST a plan point, by the streets' centre lines (planned layer), within `reach` metres; its name,
// and `distance` (optional, out). Empty when none is in reach.
std::string streetNameNear(const RoadModel& m, const Vec2& p, Real reach = 40, Real* distance = nullptr);

// Lookups by name.
ModelJunctionId junctionNamed(const RoadModel& m, const std::string& id);
ModelStreetId streetNamed(const RoadModel& m, const std::string& id);
ModelLaneId laneNamed(const RoadModel& m, const std::string& id);

// ---- in words and on a map ---------------------------------------------------------------------------------------------
std::string dumpRoadModel(const RoadModel& m);     // every junction, street, lane, connection, ramp and crossing
std::string reportRoadModel(const RoadModel& m);   // counts, what is not stated, the problems
std::string roadModelSvg(const RoadModel& m);      // junctions and streets with their ids, lanes

// ---- saved and read back ------------------------------------------------------------------------------------------------
// The layout of the bytes (magic "RMDL"). Bump when a field changes.
constexpr uint32_t kRoadModelFormat = 4;    // 2: a street's kerb parking (ADR-0418); 3: where a street carries on (ADR-0419);
                                            // 4: the planned layer's centre lines, engine class and attachments (ADR-0420)
// The version of what assembleRoadModel makes: bump when it can make a different model from the same build.
constexpr uint32_t kRoadModelVersion = 4;   // 2: a street's kerb parking (ADR-0418); 3: carrying on, no two-arm junctions (ADR-0419);
                                            // 4: names as attachments, centre lines (ADR-0420)
void putRoadModel(bundle::BinWriter& w, const RoadModel& m);
bool getRoadModel(bundle::BinReader& r, RoadModel& m, std::string* err);

// ITS OWN CACHE FILE beside the level's other caches: cache/roadmodel/<key>.roadmodel, a bundle holding the model, keyed
// by the city's bake key (lane_graph_cache.h levelRoadsKey: the lanes tag and the road file without its layout), the road
// file's layout (stripped out of that key, so hashed here) and kRoadModelVersion.
constexpr const char* kRoadModelCacheDir = "cache/roadmodel";
uint64_t roadModelCacheKey(uint64_t roadsKey, uint64_t layoutHash, uint32_t version = kRoadModelVersion);
std::string roadModelCachePath(const std::string& dir, uint64_t key);
bool writeRoadModelCache(const std::string& path, uint64_t key, const RoadModel& m, std::string* err);
bool readRoadModelCache(const std::string& path, uint64_t key, RoadModel& m, std::string* err);
// A LEVEL'S ROAD MODEL, from the cache or built (the road build, then assembleRoadModel) and saved. The first city only,
// as the lane graph. Null and `err` when the level has no lane-built city or the build fails.
struct ObtainedRoadModel {
    std::shared_ptr<const RoadModel> model;
    uint64_t key = 0;
    std::string path;
    bool fromCache = false, saved = false;
};
ObtainedRoadModel obtainLevelRoadModel(const bundle::LevelInputs& in, const std::string& cacheDir, std::string* err);

}  // namespace engine

#endif
