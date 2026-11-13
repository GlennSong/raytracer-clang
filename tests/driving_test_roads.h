#ifndef RAYTRACER_TESTS_DRIVING_TEST_ROADS_H
#define RAYTRACER_TESTS_DRIVING_TEST_ROADS_H

// SMALL REALISTIC ROAD NETWORKS FOR THE DRIVING TESTS (#254): a straight road cut into sections, and a grid of
// junctions, each built as a whole, valid lane graph (LaneGraph validate() finds nothing wrong with it) and with names
// a test can ask for: "the lane leaving junction (r, c) heading east, lane k".
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

#include "engine/procgen/city/roads/lane_graph.h"
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
    TestRoadSpec() {
        line.crossFromOffside = true;
        line.crossFromKerbside = true;
    }
};

// A BUILT NETWORK and the names into it.
struct TestRoads {
    engine::LaneGraph graph;
    TestRoadSpec spec;
    uint32_t rows = 0, cols = 0;   // a grid's size in junctions (0 for a straight road)
    uint32_t sectionCount = 0;     // a straight road's sections each way

    // ---- grid names (an unknown junction or a road off the grid's edge: not valid) ----
    engine::JunctionId junction(uint32_t row, uint32_t col) const;
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
