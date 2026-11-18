#ifndef RAYTRACER_TESTS_DRIVING_TEST_ROADS_H
#define RAYTRACER_TESTS_DRIVING_TEST_ROADS_H

// SMALL REALISTIC ROAD NETWORKS FOR THE DRIVING TESTS (#254): a straight road cut into sections, and a grid of
// junctions, each built as a whole, valid lane graph (LaneGraph validate() finds nothing wrong with it) and with names
// a test can ask for: "the lane leaving junction (r, c) heading east, lane k".
//
// EVERY TEST ROAD IS A SMALL ROAD MODEL (ADR-0436), as a level's is: the driving is handed the model and reaches the
// lane table through it. The lane table (`model.laneLevel`) is laid by hand here; what the road IS -- the limit posted
// on a stretch and its road class, the control at a junction, the turn a connector takes, which lane joins by a merge
// -- and the rules standing along a lane (a stop sign at its end, a top speed part-way along) are stated ON THE MODEL,
// once, by the functions below (stateStreet, stateControl, stateThrough, stateTurn, stateMerge, stateRegulation), and
// read back by the same questions the driving asks (RoadModel::postedLimit, roadClassOf, controlAt, facing, turnOf,
// mergingAt, regulationsAlong). A test that makes its own little lane table states its facts through the same functions.
//
// The plan is laid out on compass HEADINGS: east is +x, north is +y (the plan point's second coordinate). Lanes are
// counted as the lane graph counts them: lane 0 is the OFFSIDE-most (nearest the middle of the road), the last is the
// KERB-most. Which geometric side of a road the kerb is on is the `drivesOn` parameter, turned into geometry by
// traffic_state.h's kerbwardPerpSign, the one place that does it; the builder works the same for both.
//
// At every junction of a grid there is a connector for every turn the lane layout allows:
//   - straight on: lane k into lane k, a straight line across the junction;
//   - the NEAR turn (the one that stays by the kerb): the kerb-most lane into the kerb-most lane, a quarter arc whose
//     radius is the kerb corner's radius plus half a lane;
//   - the FAR turn (across the oncoming traffic): the offside-most lane into the offside-most lane, a quarter arc round
//     the far corner of the junction box;
//   - no U-turns.
// A grid junction is a square box reaching `cornerRadius` past the roads' edges; each road's lanes run from the edge of
// one box to the edge of the next, `roadLength` metres. Every path gets the spec's speed limit, lane width and vehicle
// tags; lanes have no bend (kNoBend), turns their arc's radius.
//
// Later tasks extend this: add a builder beside these two (a T junction, a ramp, a turn pocket) rather than special
// cases inside them.

#include "engine/procgen/city/roads/road_model.h"
#include "apps/driving/rule_set.h"        // DrivesOn
#include "apps/driving/vehicle_class.h"

#include <cstdint>
#include <string>
#include <vector>

namespace driving_test {

using engine::Real;

// The vehicle tag every test road carries unless told otherwise, and the road class and junction control names.
constexpr const char* kTestVehicleTag = "test_general";
constexpr const char* kTestRoadClass = "test_street";
constexpr const char* kTestControl = "test_uncontrolled";

// ---- stating a road's facts on a model made by hand (the tests' one way; see the top of this file) ------------------
// THE ROAD MODEL OF A LANE TABLE LAID BY HAND, with nothing stated yet: no limit, class, control, turn or merge. For a
// test whose road is its shape alone, or that states its facts next with the functions below.
inline engine::RoadModel modelOf(engine::LaneGraph table) {
    engine::RoadModel m;
    m.laneLevel = std::move(table);
    return m;
}
// A STREET of the model that posts `limit` (metres per second; 0: none posted) and is of class `roadClass` (empty:
// none), with lane path `lane` on it. Returns the street's place in the model's list, for putting more lanes on it
// (onStreet).
uint32_t stateStreet(engine::RoadModel& m, engine::PathId lane, Real limit, const std::string& roadClass = kTestRoadClass);
// Puts another lane path on a street already stated.
void onStreet(engine::RoadModel& m, engine::PathId lane, uint32_t street);
// THE CONTROL at junction `junction` of the lane table: the model gets a junction stating it (or the one the lane
// table's junction already names is restated).
void stateControl(engine::RoadModel& m, engine::JunctionId junction, const std::string& control);
// A THROUGH PAIR at junction `junction` (ADR-0438): its arms `armA` and `armB` (places in the junction's list of arms,
// the numbers its arriving lanes carry as Path::arm) are one road carrying on through, so the approaches arriving by
// them face each other (RoadModel::facing). The model's junction gets the pair (and arms up to those places); any
// pair it stated for either arm is replaced. `stated` false: the two are no longer a pair.
void stateThrough(engine::RoadModel& m, engine::JunctionId junction, uint32_t armA, uint32_t armB, bool stated = true);
// THE TURN connector `connector` takes: the model gets a stated turn of that kind, which the connector follows.
void stateTurn(engine::RoadModel& m, engine::PathId connector, engine::Turn turn);
// A MERGE: lane path `joining` ends into lane path `host`'s lane and gives way. The model gets a lane for each (where
// the path names none yet) and the connection that says so, and every crossing point where the two paths join names
// that connection. Returns the connection's place in the model's list.
uint32_t stateMerge(engine::RoadModel& m, engine::PathId joining, engine::PathId host);
// A REGULATION ALONG A LANE (ADR-0437): a rule of kind `kind` (a device name of the test's rule set) from `from` to
// `to` metres along lane path `lane` (equal: a sign standing at one place), with `value` (a top speed in metres per
// second; unused by a kind that needs none). Stated on the MODEL, on the lane's street, as a level's is
// (engine::stateRegulation); the driving reads it back through RoadModel::regulationsAlong. The street's stations
// are the lane's own metres: every lane path on that street that states no stretch of spine yet is given 0 to its
// length, running the way the spine does, so the regulation holds for every lane of the street, as a sign beside a
// road does. A lane on no street yet gets one of its own, posting no limit and of no class. False (and a line
// printed): the model refused it.
bool stateRegulation(engine::RoadModel& m, engine::PathId lane, const std::string& kind, Real from, Real to, Real value = 0);
// A SIGN STANDING AT THE END OF A LANE (a stop sign where it arrives at a junction): a regulation at its last metre.
bool stateSignAtEnd(engine::RoadModel& m, engine::PathId lane, const std::string& kind);

// WHICH WAY A ROAD RUNS on the plan.
enum class Heading : uint8_t { East, North, West, South };
constexpr uint8_t kHeadingCount = 4;
// The unit vector of a heading.
engine::Vec2 headingVector(Heading h);
// The heading the other way.
Heading opposite(Heading h);
// The heading a car arriving on `h` leaves by when it takes the near turn (toward its kerb side), or the far turn.
Heading nearTurnFrom(Heading h, driving::DrivesOn drivesOn);
Heading farTurnFrom(Heading h, driving::DrivesOn drivesOn);

// HOW THE TEST ROADS ARE BUILT.
struct TestRoadSpec {
    uint32_t lanesEachWay = 1;
    Real roadLength = 100;        // metres of lane between two junction boxes (a straight road: each section's length)
    // A GRID'S EAST-WEST ROADS between columns c and c + 1 are this long, where given (more columns: roadLength): two
    // junctions a few metres apart, as the lab's junctions 11 and 14 are (issue #264).
    std::vector<Real> columnRoadLengths;
    Real laneWidth = 3.5;
    Real cornerRadius = 6;        // how far a junction box reaches past the roads' edges: the near turn's kerb corner
    Real speedLimit = 13.4;       // m/s (30 mph), on every lane and connector
    driving::DrivesOn drivesOn = driving::DrivesOn::Left;
    engine::LaneLine line;        // the line between neighbouring lanes; crossable both ways unless set otherwise
    // TURNS REFUSED at grid junctions: no connector is built for them, as the road builder refuses a turn it cannot draw.
    struct RefusedTurn {
        uint32_t row = 0, col = 0;
        Heading in = Heading::East, out = Heading::East;
    };
    std::vector<RefusedTurn> refused;
    // THE BODIES A GRID'S CROSSING POINTS ARE FOUND FOR (each tag's most demanding body, driving::tagSizes; the road
    // builder sweeps each path at the widest and longest allowed on it, ADR-0412). Empty: the test car's width alone, its
    // length not swept (the crossing points the driving tests were written against).
    std::vector<driving::TagSize> crossingBodies;
    // THE VEHICLE TAGS every path is open to (the graph's vehicle tag names, in this order). Default: the test tag alone.
    std::vector<std::string> vehicleTags{kTestVehicleTag};
    // A GRID'S CROSSWALKS (ADR-0452): stated on the model at every arm of every junction (engine::stateCrosswalk), as a
    // level's road model states them: across the arm's road, the band kCrosswalkNear to kCrosswalkFar in from the box's
    // edge (the grid's lanes end at the box, so its connectors cross the band), kerb to kerb across the road, with
    // pavements, of control `crosswalkControl`; then laid into the lane level by the model's own function
    // (engine::layLaneCrosswalks), so the road builder's crossing finder finds the connectors over each.
    bool crosswalks = false;
    engine::CrosswalkControl crosswalkControl = engine::CrosswalkControl::None;
    TestRoadSpec() {
        line.crossFromOffside = true;
        line.crossFromKerbside = true;
    }
};

// A BUILT NETWORK and the names into it.
struct TestRoads {
    engine::RoadModel model;       // the road: its lane table is model.laneLevel
    TestRoadSpec spec;
    uint32_t rows = 0, cols = 0;   // a grid's size in junctions (0 for a straight road)
    uint32_t sectionCount = 0;     // a straight road's sections each way

    // ---- grid names (an unknown junction or a road off the grid's edge: not valid) ----
    engine::JunctionId junction(uint32_t row, uint32_t col) const;
    // THE CROSSWALK across the arm by which traffic heading `h` arrives at junction (row, col) (kNoIndex: none stated):
    // the model's id (RoadModel::crosswalkAcross).
    uint32_t crosswalkAcross(uint32_t row, uint32_t col, Heading h) const;
    // The section of the road leaving junction (row, col) heading `h`.
    engine::SectionId sectionLeaving(uint32_t row, uint32_t col, Heading h) const;
    // Lane k of it (0 = offside-most).
    engine::PathId laneLeaving(uint32_t row, uint32_t col, Heading h, uint32_t k) const;
    // The section of the road ARRIVING at junction (row, col) heading `h` (it leaves the junction behind it).
    engine::SectionId sectionArriving(uint32_t row, uint32_t col, Heading h) const;
    engine::PathId laneArriving(uint32_t row, uint32_t col, Heading h, uint32_t k) const;
    // The connector through junction (row, col) from lane `kIn` of the road arriving heading `in` to lane `kOut` of the
    // road leaving heading `out`; not valid when there is none.
    engine::PathId connector(uint32_t row, uint32_t col, Heading in, uint32_t kIn, Heading out, uint32_t kOut) const;

    // ---- straight-road names ----
    // Lane k of section i (counted from the west end) of the road heading east (`eastward`) or west.
    engine::SectionId straightSection(uint32_t i, bool eastward) const;
    engine::PathId straightLane(uint32_t i, bool eastward, uint32_t k) const;

    // ---- ring names ----
    // Lane k of side i of a ring (counted the way traffic runs round it).
    engine::PathId ringLane(uint32_t i, uint32_t k) const;

    // ---- merge names (mergeRoads) ----
    engine::PathId mergeThrough;   // the through lane, up to where the joining lane has run into it
    engine::PathId mergeJoining;   // the lane that joins it: the one the roads state merges, and so gives way
    engine::PathId mergeBeyond;    // the one lane both lead into
    // Every crossing record where the two meet (one per pair of body sizes that meet differently), each marked with
    // the model's stated merge as the road builder marks one (where `stated`).
    std::vector<engine::CrossingId> mergeJoins;

    // Lane k of a section (0 = offside-most); not valid when there is none.
    engine::PathId laneOf(engine::SectionId s, uint32_t k) const;

    // Where things are, by index (filled by the builders).
    std::vector<engine::SectionId> leavingSections;   // grid: ((row * cols + col) * kHeadingCount + heading)
    std::vector<engine::SectionId> straightSections;  // straight road: i * 2 + (eastward ? 0 : 1)
    std::vector<engine::SectionId> ringSections;      // ring: side i
};

// A STRAIGHT ROAD of `sections` sections each way, laid west to east from x = 0, each `roadLength` long, the lanes of
// one section leading straight into the same lanes of the next (no junctions).
TestRoads straightRoad(uint32_t sections, const TestRoadSpec& spec = TestRoadSpec());
// The same, with each section its own length (west to east): a long approach onto a run of very short sections, as a
// junction cut into short paths gives (spec.roadLength is not read).
TestRoads straightRoadOf(const std::vector<Real>& sectionLengths, const TestRoadSpec& spec = TestRoadSpec());

// A CLOSED RING of one-way road: `sides` straight sections, each `roadLength` long, laid as a regular polygon
// anticlockwise seen from above, each side's lanes leading straight into the same lanes of the next and the last side's
// into the first's (no junctions). A car can drive round it for ever.
TestRoads ringRoad(uint32_t sides, const TestRoadSpec& spec = TestRoadSpec());

// A MERGE BETWEEN JUNCTIONS, as a ramp's lane or a lane that ends makes one: a THROUGH lane running east for
// kMergeThroughLength metres, and a JOINING lane that starts kMergeAlongside metres before the through lane's end, one
// lane width to its kerb side, runs beside it and then tapers into it over the last kMergeTaper metres. Both lead into
// one lane BEYOND (kMergeBeyondLength metres); there is no junction. The through lane is not cut where the joining lane
// starts, so a car on it meets nothing until the taper. The crossing records where the two lanes' bodies meet are found
// by the road builder's own finder, and each names the merge the model states (stateMerge: the joining lane joins the
// through lane's), as the road builder names one (ADR-0426); `stated` false states no merge, two lanes that just
// happen to end in one. `joiningLaneFirst`: the joining lane is made first and so has the LOWER path id; otherwise the
// through lane has (the same road either way: only the ids differ).
constexpr Real kMergeThroughLength = 400, kMergeAlongside = 150, kMergeTaper = 60, kMergeBeyondLength = 300;
TestRoads mergeRoads(bool joiningLaneFirst, bool stated = true, const TestRoadSpec& spec = TestRoadSpec());

// A GRID of `rows` by `cols` junctions, junction (r, c) at the plan point (c, r) times the junction spacing. Its
// junctions carry real crossing points: where their connectors cross, join and split, found by the road builder's own
// finder (lanes/lane_graph_crossings.h, findCrossingPoints, with the test car's width), and each far turn's place to wait
// inside as the builder places it (placeWaitInsidePoints), when this build has the lanes builder (RT_ROADS_LANES);
// without it, none.
TestRoads gridRoads(uint32_t rows, uint32_t cols, const TestRoadSpec& spec = TestRoadSpec());

// AN ORDINARY CAR that may use the test roads: 4.5 m, a 2.7 m wheelbase, 30 degrees of steering (a 4.7 m turn), the
// test vehicle tag. Give `tags` to make one that carries other tags instead.
driving::VehicleClass testCarClass(const std::vector<std::string>& tags = {kTestVehicleTag});

}  // namespace driving_test

#endif
