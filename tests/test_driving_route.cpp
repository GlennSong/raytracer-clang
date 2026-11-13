// The router (#254), on the shared test roads (driving_test_roads.h): the test roads are valid and their junctions
// carry the turns the lane layout allows; routes on a grid take the fewest junctions, join up, never start by turning
// round, and come out the same every run; the lane-changes table and the questions read from it (lanesToCross,
// lastChance, wayAhead, remainingFrom); what a vehicle class may use; learned section times; "no route", in the router
// and in Driving; and how long 1,000 routes on a 40 by 40 grid take.

#include "test_framework.h"

#include "driving_test_roads.h"

#include "apps/driving/driving.h"
#include "apps/driving/route.h"
#include "hash.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

using namespace driving;
using driving_test::Heading;
using driving_test::TestRoads;
using driving_test::TestRoadSpec;

namespace {

constexpr Real kEps = 1e-6;

PathPoint at(engine::PathId path, Real along) {
    PathPoint p;
    p.path = path;
    p.along = along;
    return p;
}

bool says(const std::string& line, const std::string& words) { return line.find(words) != std::string::npos; }

// Prints and fails on any problem validate() finds.
void checkValid(const engine::LaneGraph& g, const char* what) {
    const std::vector<std::string> problems = engine::validate(g);
    for (const std::string& p : problems) std::printf("    %s: %s\n", what, p.c_str());
    CHECK(problems.empty());
}

TestRoadSpec specWith(uint32_t lanes, DrivesOn side) {
    TestRoadSpec s;
    s.lanesEachWay = lanes;
    s.drivesOn = side;
    return s;
}

// The test car bound to a graph.
RouteClass carOn(const engine::LaneGraph& g) { return bindRouteClass(g, driving_test::testCarClass()); }

Route route(const TestRoads& t, const PathPoint& from, const PathPoint& to, const SectionSeconds* learned = nullptr) {
    return findRoute(t.graph, studyRoads(t.graph), carOn(t.graph), from, to, learned);
}

// The legs join up: each turn's connector leaves a lane of the leg before it and joins a lane of the leg after, and
// none of them is a U-turn.
void checkJoinsUp(const engine::LaneGraph& g, const Route& r) {
    CHECK(r.turns.size() + 1 == r.legs.size());
    for (std::size_t i = 0; i < r.turns.size(); ++i) {
        const RouteTurn& t = r.turns[i];
        CHECK(t.connector.valid());
        if (!t.connector.valid()) continue;
        const engine::Path& c = g.paths[t.connector.index];
        CHECK(t.turn != engine::Turn::UTurn);
        CHECK(t.fromSection == r.legs[i].section && t.toSection == r.legs[i + 1].section);
        CHECK(g.paths[c.from.index].section == r.legs[i].section);
        CHECK(g.paths[g.nextPaths[c.next.first].index].section == r.legs[i + 1].section);
    }
}

bool routeUses(const Route& r, engine::SectionId s) {
    for (const RouteLeg& l : r.legs)
        if (l.section == s) return true;
    return false;
}

const RouteLane& laneIn(const Route& r, std::size_t leg, uint32_t k) { return r.lanes[r.legs[leg].firstLane + k]; }

// Adds a one-lane section nothing leads into or out of, far from the rest; returns its lane.
engine::PathId addIsolatedLane(engine::LaneGraph& g) {
    const engine::SectionId sid(static_cast<uint32_t>(g.sections.size()));
    const engine::PathId lane(static_cast<uint32_t>(g.paths.size()));
    engine::Path p;
    p.curve = g.addCurve({engine::Vec2(-1000, -1000), engine::Vec2(-900, -1000)}, {0, 0});
    p.length = 100;
    p.width = 3.5;
    p.speedLimit = 13.4;
    p.vehicleTags = 1;
    p.section = sid;
    p.next.first = static_cast<uint32_t>(g.nextPaths.size());
    g.paths.push_back(p);
    engine::Section s;
    s.roadClass = 0;
    s.lanes.first = static_cast<uint32_t>(g.sectionLanes.size());
    s.lanes.count = 1;
    s.lines.first = static_cast<uint32_t>(g.laneLines.size());
    g.sectionLanes.push_back(lane);
    g.sections.push_back(s);
    return lane;
}

}  // namespace

// ---- 1. the test roads -----------------------------------------------------------------------------------------------

TEST_CASE(route_test_roads_are_valid) {
    for (int side = 0; side < 2; ++side) {
        const DrivesOn drivesOn = side == 0 ? DrivesOn::Left : DrivesOn::Right;
        for (uint32_t lanes = 1; lanes <= 2; ++lanes) {
            const TestRoadSpec spec = specWith(lanes, drivesOn);
            const TestRoads straight = driving_test::straightRoad(3, spec);
            checkValid(straight.graph, "straight road");
            CHECK(straight.graph.sections.size() == 6u);
            CHECK(straight.graph.paths.size() == 6u * lanes);
            const TestRoads small = driving_test::gridRoads(2, 2, spec);
            checkValid(small.graph, "2x2 grid");
            const TestRoads big = driving_test::gridRoads(4, 4, spec);
            checkValid(big.graph, "4x4 grid");
            // 4 by 4 junctions: 3 roads in each of 4 rows and 4 columns, two ways.
            CHECK(big.graph.sections.size() == 48u);
            CHECK(big.graph.junctions.size() == 16u);
        }
    }

    // One junction's connectors, against the rules: straight lane k into lane k, the near turn kerb-most into
    // kerb-most, the far turn offside-most into offside-most, no U-turn.
    const TestRoadSpec spec = specWith(2, DrivesOn::Left);
    const TestRoads t = driving_test::gridRoads(4, 4, spec);
    const Heading near = driving_test::nearTurnFrom(Heading::East, spec.drivesOn);
    const Heading far = driving_test::farTurnFrom(Heading::East, spec.drivesOn);
    CHECK(near != far && near != Heading::East && far != Heading::West);
    const engine::Junction& j = t.graph.junctions[t.junction(1, 1).index];
    CHECK(j.connectors.count == 16u);   // four ways in, each: two straight on, one near turn, one far turn
    const Real halfBox = 2 * spec.laneWidth + spec.cornerRadius;
    for (uint32_t k = 0; k < 2; ++k) {
        const engine::PathId s = t.connector(1, 1, Heading::East, k, Heading::East, k);
        CHECK(s.valid());
        if (s.valid()) {
            CHECK(t.graph.paths[s.index].turn == engine::Turn::Straight);
            CHECK_APPROX(t.graph.paths[s.index].length, 2 * halfBox, kEps);
            CHECK(t.graph.paths[s.index].tightestRadius == engine::kNoBend);
        }
        CHECK(!t.connector(1, 1, Heading::East, k, Heading::East, 1 - k).valid());
        CHECK(!t.connector(1, 1, Heading::East, k, Heading::West, k).valid());   // no U-turn
    }
    const engine::PathId nearTurn = t.connector(1, 1, Heading::East, 1, near, 1);
    const engine::PathId farTurn = t.connector(1, 1, Heading::East, 0, far, 0);
    CHECK(nearTurn.valid() && farTurn.valid());
    CHECK(!t.connector(1, 1, Heading::East, 0, near, 0).valid());
    CHECK(!t.connector(1, 1, Heading::East, 0, near, 1).valid());
    CHECK(!t.connector(1, 1, Heading::East, 1, far, 1).valid());
    CHECK(!t.connector(1, 1, Heading::East, 1, far, 0).valid());
    if (nearTurn.valid() && farTurn.valid()) {
        const engine::Path& n = t.graph.paths[nearTurn.index];
        const engine::Path& f = t.graph.paths[farTurn.index];
        CHECK(n.turn == engine::Turn::NearTurn && f.turn == engine::Turn::FarTurn);
        CHECK_APPROX(n.tightestRadius, spec.cornerRadius + spec.laneWidth / 2, kEps);
        CHECK_APPROX(f.tightestRadius, halfBox + spec.laneWidth / 2, kEps);
        // A quarter arc drawn in straight pieces: a little shorter than the arc.
        CHECK(n.length < n.tightestRadius * 3.14159265 / 2 && n.length > n.tightestRadius * 1.5);
        CHECK(n.junction == t.junction(1, 1) && n.from == t.laneArriving(1, 1, Heading::East, 1));
    }
    // The kerb-most lane of a road lies nearer the kerb than the offside-most: further from the road's middle line,
    // on the side the near turn goes.
    const engine::Vec2 p0 = t.graph.pointAt(t.laneLeaving(1, 1, Heading::East, 0), 50);
    const engine::Vec2 p1 = t.graph.pointAt(t.laneLeaving(1, 1, Heading::East, 1), 50);
    CHECK(engine::dot(p1 - p0, driving_test::headingVector(near)) > 0);

    // A straight road routes through its sections with no junction turns.
    const TestRoads road = driving_test::straightRoad(3, spec);
    const Route r = route(road, at(road.straightLane(0, true, 1), 10), at(road.straightLane(2, true, 1), 50));
    CHECK(r.found && r.legs.size() == 3u && r.turns.size() == 2u);
    if (r.found && r.turns.size() == 2u) {
        CHECK(r.turns[0].turn == engine::Turn::None && !r.turns[0].connector.valid());
        CHECK_APPROX(r.metres, 90 + 100 + 50, kEps);
    }
}

// ---- 2. routes on a grid --------------------------------------------------------------------------------------------

TEST_CASE(route_grid_far_corners_fewest_junctions_same_every_run) {
    const TestRoads t = driving_test::gridRoads(4, 4, specWith(1, DrivesOn::Left));
    const PathPoint from = at(t.laneLeaving(0, 0, Heading::East, 0), 10);
    const PathPoint to = at(t.laneArriving(3, 3, Heading::East, 0), 90);
    const Route r = route(t, from, to);
    CHECK(r.found);
    if (!r.found) return;
    // From (0, 1), where the first road ends, to (3, 2), where the last begins: four roads, five junctions.
    CHECK(r.turns.size() == 5u);
    CHECK(r.legs.front().section == t.sectionLeaving(0, 0, Heading::East));
    CHECK(r.legs.back().section == t.sectionArriving(3, 3, Heading::East));
    CHECK_APPROX(r.legs.front().enterFraction, 0.1, kEps);
    CHECK_APPROX(r.legs.back().leaveFraction, 0.9, kEps);
    checkJoinsUp(t.graph, r);
    // The same question, the same route.
    const Route again = route(t, from, to);
    CHECK(routeHash(r) == routeHash(again));
    CHECK(again.legs.size() == r.legs.size());
    // remainingFrom the start is the whole route.
    const Remaining all = remainingFrom(r, t.graph, from);
    CHECK(all.onRoute);
    CHECK_APPROX(all.metres, r.metres, kEps);
    CHECK_APPROX(all.seconds, r.seconds, kEps);
    CHECK_APPROX(r.seconds, r.metres / t.spec.speedLimit, kEps);
}

TEST_CASE(route_starting_away_goes_round_never_reverses) {
    const TestRoads t = driving_test::gridRoads(4, 4, specWith(1, DrivesOn::Left));
    // Facing west, toward the corner, with the destination far to the north-east.
    const PathPoint from = at(t.laneLeaving(0, 1, Heading::West, 0), 10);
    const Route r = route(t, from, at(t.laneArriving(3, 3, Heading::East, 0), 90));
    CHECK(r.found);
    if (!r.found) return;
    CHECK(r.legs.front().section == t.sectionLeaving(0, 1, Heading::West));
    CHECK(r.legs.size() > 1u && r.legs[1].section != t.sectionLeaving(0, 0, Heading::East));   // no swing round
    checkJoinsUp(t.graph, r);

    // A destination behind the car on its own lane: round the block and back along it.
    const engine::PathId lane = t.laneLeaving(1, 1, Heading::East, 0);
    const Route back = route(t, at(lane, 60), at(lane, 20));
    CHECK(back.found);
    if (!back.found) return;
    CHECK(back.legs.size() == 5u);
    CHECK(back.legs.front().section == back.legs.back().section);
    checkJoinsUp(t.graph, back);
    for (const RouteTurn& turn : back.turns) CHECK(turn.turn == engine::Turn::NearTurn);   // the smallest block
    // The same lane is two places on the route; between them it is on neither.
    CHECK(placeOnRoute(back, t.graph, at(lane, 70)).part == RoutePart::Leg);
    CHECK(placeOnRoute(back, t.graph, at(lane, 70)).index == 0u);
    CHECK(placeOnRoute(back, t.graph, at(lane, 10)).index == 4u);
    CHECK(!placeOnRoute(back, t.graph, at(lane, 40)).onRoute());
    CHECK(placeOnRoute(back, t.graph, at(back.turns[1].connector, 1)).part == RoutePart::Turn);
    CHECK(placeOnRoute(back, t.graph, at(back.turns[1].connector, 1)).index == 1u);
    // From the last visit on, the first is no longer looked at.
    CHECK(placeOnRoute(back, t.graph, at(lane, 70), 4).part == RoutePart::Off);
}

// ---- 3. which lanes lead on --------------------------------------------------------------------------------------

TEST_CASE(route_lane_changes_table_for_a_far_turn) {
    for (int side = 0; side < 2; ++side) {
        const TestRoadSpec spec = specWith(2, side == 0 ? DrivesOn::Left : DrivesOn::Right);
        TestRoads t = driving_test::gridRoads(4, 4, spec);
        const Heading far = driving_test::farTurnFrom(Heading::East, spec.drivesOn);
        // Starting in the kerb-side lane, straight on through (1, 1), then the far turn at (1, 2).
        const engine::PathId start = t.laneLeaving(1, 0, Heading::East, 1);
        const PathPoint to = at(t.laneLeaving(1, 2, far, 0), 50);
        const Route r = route(t, at(start, 10), to);
        CHECK(r.found && r.legs.size() == 3u);
        if (!r.found || r.legs.size() != 3u) continue;
        CHECK(r.turns[0].turn == engine::Turn::Straight && r.turns[1].turn == engine::Turn::FarTurn);
        // Lane 0 is the offside lane, lane 1 the kerb-side lane.
        for (std::size_t leg = 0; leg < 3; ++leg) {
            CHECK(laneIn(r, leg, 0).changes == 0u);
            CHECK(laneIn(r, leg, 1).changes == 1u);
        }
        CHECK(laneIn(r, 0, 1).changesIfStaying == 1u);            // it goes straight on, and changes later
        CHECK(laneIn(r, 1, 1).changesIfStaying == kCannotFinish); // the last section before the turn: it must leave
        CHECK(laneIn(r, 2, 1).changesIfStaying == kCannotFinish); // the destination is not on it

        // Three points.
        const engine::PathId kerb0 = t.laneLeaving(1, 0, Heading::East, 1);
        const engine::PathId kerb1 = t.laneLeaving(1, 1, Heading::East, 1);
        const engine::PathId off1 = t.laneLeaving(1, 1, Heading::East, 0);
        const engine::PathId kerb2 = t.laneLeaving(1, 2, far, 1);
        LanesToCross x = lanesToCross(r, kerb0);
        CHECK(x.onRoute && x.lanes == 1u && x.toward == Across::TowardOffside);
        x = lanesToCross(r, kerb1);
        CHECK(x.onRoute && x.lanes == 1u && x.toward == Across::TowardOffside);
        x = lanesToCross(r, off1);
        CHECK(x.onRoute && x.lanes == 0u && x.toward == Across::None);
        CHECK(!lanesToCross(r, t.laneLeaving(2, 2, Heading::East, 0)).onRoute);
        LastChance last = lastChance(r, kerb0, 30);
        CHECK(!last.needed);
        last = lastChance(r, kerb1, 30);
        CHECK(last.needed);
        CHECK_APPROX(last.metres, 70, kEps);
        last = lastChance(r, kerb2, 20);
        CHECK(last.needed);
        CHECK_APPROX(last.metres, 30, kEps);   // the destination is 50 m along
        CHECK(!lastChance(r, off1, 30).needed);

        // A line that may not be crossed from the kerb side in the last section before the turn: there the kerb-side
        // lane cannot finish; before it, it can still change.
        const engine::Section& sec = t.graph.sections[t.sectionLeaving(1, 1, Heading::East).index];
        t.graph.laneLines[sec.lines.first].crossFromKerbside = false;
        checkValid(t.graph, "one solid line");
        const Route solid = route(t, at(start, 10), to);
        CHECK(solid.found && solid.legs.size() == 3u);
        if (!solid.found || solid.legs.size() != 3u) continue;
        CHECK(laneIn(solid, 1, 1).changes == kCannotFinish);
        CHECK(laneIn(solid, 1, 0).changes == 0u);
        CHECK(laneIn(solid, 0, 1).changes == 1u);
        CHECK(laneIn(solid, 0, 1).changesIfStaying == kCannotFinish);   // now it must change before the solid line
        CHECK(lastChance(solid, kerb0, 30).needed);
    }
}

// ---- 4. the way ahead -----------------------------------------------------------------------------------------------

TEST_CASE(route_way_ahead_through_junctions_lane_end_destination) {
    const TestRoadSpec spec = specWith(2, DrivesOn::Left);
    const TestRoads t = driving_test::gridRoads(4, 4, spec);
    const Heading far = driving_test::farTurnFrom(Heading::East, spec.drivesOn);
    const engine::PathId off0 = t.laneLeaving(1, 0, Heading::East, 0);
    const PathPoint to = at(t.laneLeaving(1, 3, far, 0), 60);
    const Route r = route(t, at(off0, 50), to);
    CHECK(r.found && r.legs.size() == 4u);
    if (!r.found) return;
    const Real across = t.graph.paths[t.connector(1, 1, Heading::East, 0, Heading::East, 0).index].length;

    // From a lane that leads on: through two junctions, stopping once the look-ahead is covered.
    WayAhead w = wayAhead(r, t.graph, off0, 50, 50 + across + 100 + across + 10);
    CHECK(w.paths.size() == 5u);
    if (w.paths.size() == 5u) {
        CHECK(w.paths[0] == off0);
        CHECK(w.paths[1] == t.connector(1, 1, Heading::East, 0, Heading::East, 0));
        CHECK(w.paths[2] == t.laneLeaving(1, 1, Heading::East, 0));
        CHECK(w.paths[3] == t.connector(1, 2, Heading::East, 0, Heading::East, 0));
        CHECK(w.paths[4] == t.laneLeaving(1, 2, Heading::East, 0));
    }
    // A short look-ahead: the car's own path alone.
    CHECK(wayAhead(r, t.graph, off0, 50, 40).paths.size() == 1u);

    // From the kerb-side lane: on straight while it leads on, ending with the lane before the far turn.
    const engine::PathId kerb0 = t.laneLeaving(1, 0, Heading::East, 1);
    w = wayAhead(r, t.graph, kerb0, 50, 1000);
    CHECK(w.paths.size() == 5u);
    if (w.paths.size() == 5u) CHECK(w.paths[4] == t.laneLeaving(1, 2, Heading::East, 1));
    w = wayAhead(r, t.graph, t.laneLeaving(1, 2, Heading::East, 1), 10, 1000);
    CHECK(w.paths.size() == 1u);

    // To the destination, and no further.
    w = wayAhead(r, t.graph, t.laneLeaving(1, 2, Heading::East, 0), 10, 1000);
    CHECK(w.paths.size() == 3u);
    if (w.paths.size() == 3u) {
        CHECK(w.paths[1] == t.connector(1, 3, Heading::East, 0, far, 0));
        CHECK(w.paths[2] == to.path);
    }
    CHECK(wayAhead(r, t.graph, to.path, 30, 1000).paths.size() == 1u);
    // Off the route: its own path alone.
    CHECK(wayAhead(r, t.graph, t.laneLeaving(2, 2, Heading::East, 0), 10, 1000).paths.size() == 1u);

    // What remains from part way, along the route's own lanes.
    const Remaining rest = remainingFrom(r, t.graph, at(t.laneLeaving(1, 2, Heading::East, 0), 10));
    const Real farLength = t.graph.paths[t.connector(1, 3, Heading::East, 0, far, 0).index].length;
    CHECK(rest.onRoute);
    CHECK_APPROX(rest.metres, 90 + farLength + 60, kEps);
    CHECK(!remainingFrom(r, t.graph, at(t.laneLeaving(2, 2, Heading::East, 0), 10)).onRoute);
    CHECK(!remainingFrom(r, t.graph, at(to.path, 70)).onRoute);   // past the destination
}

// ---- 5. what a class may use ---------------------------------------------------------------------------------------

TEST_CASE(route_class_tags_and_turning_radius) {
    TestRoads t = driving_test::gridRoads(4, 4, specWith(1, DrivesOn::Left));
    const PathPoint from = at(t.laneLeaving(0, 0, Heading::East, 0), 10);
    const PathPoint to = at(t.laneArriving(3, 3, Heading::East, 0), 90);
    const Route before = route(t, from, to);
    CHECK(before.found && before.legs.size() == 6u);
    if (!before.found || before.legs.size() != 6u) return;

    // A street open only to another tag: the test car goes round it.
    const engine::SectionId closed = before.legs[2].section;
    t.graph.vehicleTagNames.push_back("test_other");
    const engine::Section& sec = t.graph.sections[closed.index];
    for (uint32_t k = 0; k < sec.lanes.count; ++k) t.graph.paths[t.graph.sectionLanes[sec.lanes.first + k].index].vehicleTags = 2;
    checkValid(t.graph, "one street closed");
    const Route round = route(t, from, to);
    CHECK(round.found);
    CHECK(!routeUses(round, closed));
    checkJoinsUp(t.graph, round);
    CHECK(round.turns.size() == 5u);   // still the fewest junctions: another street as short

    // A class with only that tag may use nothing but that street: the destination is closed to it.
    std::vector<std::string> notes;
    const RouteClass other = bindRouteClass(t.graph, driving_test::testCarClass({"test_other"}), &notes);
    CHECK(notes.empty());
    CHECK(other.tags == 2u);
    Route none = findRoute(t.graph, studyRoads(t.graph), other, from, to);
    CHECK(!none.found && none.why == NoRouteWhy::DestinationClosed);
    // ... and from that street, nothing leads anywhere it may go.
    const engine::PathId closedLane = t.graph.sectionLanes[sec.lanes.first];
    none = findRoute(t.graph, studyRoads(t.graph), other, at(closedLane, 10), at(closedLane, 5));
    CHECK(!none.found && none.why == NoRouteWhy::NothingLeadsThere);
    // A tag the road does not know is a plain note, and matches nothing.
    const RouteClass unknown = bindRouteClass(t.graph, driving_test::testCarClass({"no_such_tag"}), &notes);
    CHECK(unknown.tags == 0u);
    CHECK(notes.size() == 1u && says(notes[0], "no_such_tag"));
    CHECK(!mayUse(t.graph, unknown, from.path));
    // An empty mask is open to nobody.
    t.graph.paths[from.path.index].vehicleTags = 0;
    CHECK(!mayUse(t.graph, carOn(t.graph), from.path));

    // A class that cannot turn as tightly as a near turn: round the block by far turns only.
    const TestRoads g = driving_test::gridRoads(4, 4, specWith(1, DrivesOn::Left));
    RouteClass wide = carOn(g.graph);
    wide.tightestTurnRadius = g.spec.cornerRadius + g.spec.laneWidth;   // wider than the near turn, inside the far
    const engine::PathId lane = g.laneLeaving(1, 1, Heading::East, 0);
    const engine::PathId nearTurn =
        g.connector(1, 2, Heading::East, 0, driving_test::nearTurnFrom(Heading::East, DrivesOn::Left), 0);
    CHECK(nearTurn.valid() && !mayUse(g.graph, wide, nearTurn));
    const Route wideBack = findRoute(g.graph, studyRoads(g.graph), wide, at(lane, 60), at(lane, 20));
    CHECK(wideBack.found);
    for (const RouteTurn& turn : wideBack.turns) {
        CHECK(turn.turn != engine::Turn::NearTurn);
        CHECK(g.graph.paths[turn.connector.index].tightestRadius >= wide.tightestTurnRadius);
    }
}

// ---- 6. learned section times ----------------------------------------------------------------------------------------

TEST_CASE(route_learned_times_move_the_route) {
    const TestRoads t = driving_test::gridRoads(4, 4, specWith(1, DrivesOn::Left));
    const PathPoint from = at(t.laneLeaving(0, 0, Heading::East, 0), 10);
    const PathPoint to = at(t.laneArriving(3, 3, Heading::East, 0), 90);
    const Route quick = route(t, from, to);
    CHECK(quick.found && quick.legs.size() == 6u);
    if (!quick.found || quick.legs.size() != 6u) return;
    const engine::SectionId jammed = quick.legs[2].section;
    SectionSeconds learned(t.graph.sections.size(), 0);
    learned[jammed.index] = 600;   // ten minutes to drive 100 m
    const Route slow = route(t, from, to, &learned);
    CHECK(slow.found);
    CHECK(!routeUses(slow, jammed));
    CHECK(slow.turns.size() == 5u);
    // An empty table changes nothing.
    const SectionSeconds nothing;
    CHECK(routeHash(route(t, from, to, &nothing)) == routeHash(quick));
    // A learned time faster than the limit is still taken (and the lower bound allows for it).
    learned[jammed.index] = 1;
    const Route fast = route(t, from, to, &learned);
    CHECK(fast.found && routeUses(fast, jammed));
}

// ---- 7. no route -----------------------------------------------------------------------------------------------------

TEST_CASE(route_no_route_plain_reason_and_in_driving) {
    TestRoads t = driving_test::gridRoads(2, 2, specWith(1, DrivesOn::Left));
    const engine::PathId island = addIsolatedLane(t.graph);
    checkValid(t.graph, "with an isolated lane");
    const PathPoint from = at(t.laneLeaving(0, 0, Heading::East, 0), 50);
    const Route r = route(t, from, at(island, 50));
    CHECK(!r.found && r.why == NoRouteWhy::NothingLeadsThere);
    CHECK(says(r.whyWords, "nothing") && says(r.whyWords, "leads"));
    std::printf("    no route: %s\n", r.whyWords.c_str());
    const Route off = route(t, from, at(island, 101));
    CHECK(!off.found && off.why == NoRouteWhy::DestinationNotOnRoad);

    Driving d(&t.graph);
    VehicleClasses classes;
    classes.add(driving_test::testCarClass());
    d.setVehicleClasses(classes);
    CHECK(d.routeNotes().empty());
    CHECK(d.placeCar(CarId(1), 0, from));
    CHECK(!d.beginTrip(CarId(1), 7, at(island, 50)).valid());
    CHECK(!d.car(CarId(1))->driven);
    std::vector<Event> ev = d.drainEvents();
    CHECK(ev.size() == 1u);
    if (ev.size() == 1u) {
        CHECK(ev[0].kind == EventKind::NoRoute && ev[0].car == CarId(1) && ev[0].person == 7u);
        CHECK(!ev[0].trip.valid());
    }

    const PathPoint home = at(t.laneLeaving(1, 1, Heading::West, 0), 40);
    const TripId id = d.beginTrip(CarId(1), 7, home);
    CHECK(id.valid());
    const Route* kept = d.route(id);
    CHECK(kept && kept->found && kept->to == home);
    if (!kept) return;
    const uint64_t keptHash = routeHash(*kept);
    const uint64_t stateBefore = d.stateHash();
    CHECK(!d.changeDestination(id, at(island, 50)));
    CHECK(d.trip(id)->destination == home);
    CHECK(routeHash(*d.route(id)) == keptHash);
    ev = d.drainEvents();
    CHECK(ev.size() == 1u);
    if (ev.size() == 1u) CHECK(ev[0].kind == EventKind::NoRoute && ev[0].trip == id);
    CHECK(d.stateHash() == stateBefore);   // nothing changed but the event, now drained
    CHECK(d.reroute(id));
    CHECK(routeHash(*d.route(id)) == keptHash);   // the car has not moved: the same route

    // The state hash covers the route: the same trip on a road where one street is slow hashes differently.
    SectionSeconds learned(t.graph.sections.size(), 0);
    const Route other = route(t, from, home, &learned);
    CHECK(routeHash(other) == keptHash);
    TestRoads slowed = t;
    const engine::SectionId first = kept->legs.size() > 1 ? kept->legs[1].section : kept->legs[0].section;
    const engine::Section& sec = slowed.graph.sections[first.index];
    slowed.graph.paths[slowed.graph.sectionLanes[sec.lanes.first].index].speedLimit = 1;
    Driving d2(&slowed.graph);
    d2.setVehicleClasses(classes);
    CHECK(d2.placeCar(CarId(1), 0, from));
    const TripId id2 = d2.beginTrip(CarId(1), 7, home);
    CHECK(id2.valid() && d2.stateHash() != d.stateHash());
}

// ---- 8. speed --------------------------------------------------------------------------------------------------------

TEST_CASE(route_timing_1000_routes_on_a_40_by_40_grid) {
    const TestRoads t = driving_test::gridRoads(40, 40, specWith(2, DrivesOn::Left));
    const engine::LaneGraph& g = t.graph;
    std::vector<engine::PathId> lanes;
    for (uint32_t i = 0; i < g.paths.size(); ++i)
        if (g.paths[i].kind == engine::PathKind::Lane) lanes.push_back(engine::PathId(i));
    const RouteBounds bounds = studyRoads(g);
    const RouteClass car = carOn(g);
    constexpr uint32_t kRoutes = 1000;
    constexpr uint32_t kSeed = 254;
    std::vector<std::pair<PathPoint, PathPoint>> trips;
    for (uint32_t i = 0; i < kRoutes; ++i) {
        PathPoint ends[2];
        for (uint32_t e = 0; e < 2; ++e) {
            const uint32_t h = engine::hashPair(kSeed, i * 2 + e);
            const engine::PathId lane = lanes[h % lanes.size()];
            ends[e] = at(lane, engine::unit24(engine::hashPair(h, 1)) * g.paths[lane.index].length);
        }
        trips.emplace_back(ends[0], ends[1]);
    }
    // Pass 0 with the lower bound (A*), pass 1 without (plain Dijkstra): the bound never over-estimates, so both find
    // routes of the same cost.
    std::vector<Real> costs(kRoutes, 0);
    for (int pass = 0; pass < 2; ++pass) {
        const RouteBounds used = pass == 0 ? bounds : RouteBounds();
        uint32_t found = 0;
        double metres = 0;
        std::vector<Real> passCosts;
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        for (const std::pair<PathPoint, PathPoint>& trip : trips) {
            const Route r = findRoute(g, used, car, trip.first, trip.second);
            if (r.found) ++found;
            metres += r.metres;
            passCosts.push_back(r.cost);
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        if (pass == 0) costs = passCosts;
        uint32_t sameCost = 0;
        for (uint32_t i = 0; i < kRoutes; ++i)
            if (std::fabs(passCosts[i] - costs[i]) <= kEps) ++sameCost;
        CHECK(sameCost == kRoutes);
        std::printf("    %u routes on a 40x40 grid, two lanes each way (%zu paths), %s: %.1f ms (%.0f us each); "
                    "%u found, %.0f km\n",
                    kRoutes, g.paths.size(), pass == 0 ? "A*" : "plain Dijkstra", ms, ms * 1000.0 / kRoutes, found,
                    metres / 1000.0);
        CHECK(found == kRoutes);
    }
}

// WAYS ON CLOSED TO A CLASS (issue #284): the tree of closed chains closes the turn that completes a chain, never the
// first turn alone; a chain under way that turns elsewhere is over; and it tells "never" from "too seldom".
TEST_CASE(route_closed_chains_close_the_chain_not_the_turn) {
    using engine::PathId;
    // Closed: 10 then 20 (never), and 10 then 30 then 40 (too seldom). Listed out of order, one twice.
    const ClosedChains c = buildClosedChains({{PathId(10), PathId(30), PathId(40)}, {PathId(10), PathId(20)},
                                              {PathId(10), PathId(20)}, {PathId(5)}},
                                             {0, 1, 0, 1});
    CHECK(c.chains == 2u);   // the one-turn "chain" is not a chain; the repeat counted once
    const uint32_t after10 = c.step(0, PathId(10));
    CHECK(after10 != 0u && after10 != kChainClosed);    // the first turn stays open
    CHECK(c.step(after10, PathId(20)) == kChainClosed); // ... the turn completing a closed chain does not
    const uint32_t after30 = c.step(after10, PathId(30));
    CHECK(after30 != 0u && after30 != kChainClosed);
    CHECK(c.step(after30, PathId(40)) == kChainClosed);
    CHECK(c.step(after30, PathId(41)) == 0u);           // another way on from 30: the chain is over
    CHECK(c.step(after10, PathId(50)) == 0u);
    CHECK(c.step(0, PathId(20)) == 0u);                 // 20 alone is fine
    const PathId twoTurns[] = {PathId(10), PathId(20)};
    const PathId threeTurns[] = {PathId(10), PathId(30), PathId(40)};
    bool never = false;
    CHECK(c.closedChain(twoTurns, 2, &never) && never);
    CHECK(c.closedChain(threeTurns, 3, &never) && !never);
    CHECK(!c.closedChain(threeTurns, 2, &never));
    CHECK(buildClosedChains({}, {}).empty());
}

#ifdef RT_ROADS_LANES
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/lanes.h"

#include <memory>

namespace {

// A real junction's lane graph, as the road builder makes it for these vehicle classes (driving::tagSizes); with
// `withLength` false, the same sizes swept as points (each body's length left out, as before #284 item 9).
engine::roads::lanes::LaneGraphBuild builtFor(const engine::roads::lanes::Result& roads, const VehicleClasses& classes,
                                             bool withLength) {
    engine::roads::lanes::LaneGraphOptions o;
    for (const TagSize& t : tagSizes(classes)) {
        engine::roads::lanes::VehicleSize v;
        v.tag = t.tag;
        v.turningRadius = t.turningRadius;
        v.width = t.width;
        v.length = withLength ? t.length : 0;
        o.vehicles.push_back(v);
    }
    return engine::roads::lanes::buildLaneGraph(roads, o);
}

}  // namespace

TEST_CASE(route_a_long_body_is_never_sent_round_a_turn_its_swing_takes_off_the_road) {
    // ITEM 9 OF REVIEW #284 (ADR-0412). The road builder sweeps each turn at each vehicle's length: a 12 m bus whose turning
    // circle fits a turn can still swing its middle over the kerb there, and that turn is closed to it (its tag) while a
    // car's stays open. The router reads only the tags, so it never sends the bus that way. On the lab's hill junction.
    const std::unique_ptr<engine::roads::lanes::Result> roads = engine::roads::lanes::build(
        engine::roads::lanes::RoadLabGraph::load(std::string(RT_SOURCE_DIR) + "/assets/lanelab/hill_junction.json"));
    VehicleClasses classes;
    classes.add(driving_test::testCarClass({"test_car_tag"}));
    VehicleClass bus = deriveVehicleClass("test_bus", 12.0, 2.55, 5.7, 38);
    bus.tags = {"test_bus_tag"};
    classes.add(bus);
    const engine::roads::lanes::LaneGraphBuild asPoints = builtFor(*roads, classes, false);
    const engine::roads::lanes::LaneGraphBuild swept = builtFor(*roads, classes, true);
    const engine::LaneGraph& g = swept.graph;
    const RouteClass car = bindRouteClass(g, classes.at(0)), longBody = bindRouteClass(g, classes.at(1));
    // The connectors open to the bus swept as a point and closed to it swept at its length, open to the car either way.
    uint32_t closedForLength = 0, routed = 0;
    for (uint32_t c = 0; c < g.paths.size(); ++c) {
        const engine::Path& p = g.paths[c];
        if (p.kind != engine::PathKind::Connector || mayUse(g, longBody, engine::PathId(c))) continue;
        bool openAsPoint = false;
        for (uint32_t k = 0; k < asPoints.graph.paths.size(); ++k) {
            const engine::Path& q = asPoints.graph.paths[k];
            if (q.kind == engine::PathKind::Connector && q.from == p.from && asPoints.graph.nextPaths[q.next.first] ==
                                                                                   g.nextPaths[p.next.first])
                openAsPoint = mayUse(asPoints.graph, bindRouteClass(asPoints.graph, classes.at(1)), engine::PathId(k));
        }
        if (!openAsPoint || !mayUse(g, car, engine::PathId(c))) continue;
        ++closedForLength;
        CHECK(p.tightestRadius >= classes.at(1).tightestTurnRadius);   // closed for its swing, not its turning circle
        // A trip from the lane before the turn to the lane after it: the car takes the turn; the bus never does.
        const engine::PathId from = p.from, to = g.nextPaths[p.next.first];
        const PathPoint start = at(from, g.paths[from.index].length * 0.5);
        const PathPoint end = at(to, std::min<Real>(10, g.paths[to.index].length * 0.5));
        const Route carRoute = findRoute(g, studyRoads(g), car, start, end);
        const Route busRoute = findRoute(g, studyRoads(g), longBody, start, end);
        bool carTurns = false, busTurns = false;
        for (const RouteTurn& t : carRoute.turns) carTurns = carTurns || t.connector == engine::PathId(c);
        for (const RouteTurn& t : busRoute.turns) busTurns = busTurns || t.connector == engine::PathId(c);
        CHECK(carRoute.found && carTurns);
        CHECK(!busTurns);
        routed += busRoute.found ? 1 : 0;
    }
    const engine::NameIndex busTag = engine::LaneGraph::nameIndex(g.vehicleTagNames, "test_bus_tag");
    std::printf("    hill junction: %u connectors closed to the bus only for its length (%u of them with another way round "
                "for it); closed to it: %u swept as a point, %u at its length\n", closedForLength, routed,
                busTag < asPoints.closedTo.size() ? asPoints.closedTo[busTag] : 0u,
                busTag < swept.closedTo.size() ? swept.closedTo[busTag] : 0u);
    CHECK(closedForLength > 0u);
}
#endif  // RT_ROADS_LANES
