// THE REAL DRIVE SKILL (ADR-0440, brain/brain_driving.h): the brain's DriveSkill over the driving library's front door,
// call for call, on two straight lanes laid by hand; and the driving's events handed to the machine under the data's
// names. No rules are handed to the driving here, so no car moves: what is held is that each call reaches the driving
// and each event reaches the right person. (A whole trip driven by the real driving is the next step, in the lab.)

#include "test_framework.h"

#include "brain_fakes.h"

#include "apps/citysim/brain/brain_driving.h"

#include <string>
#include <vector>

using namespace brain_test;

namespace {

constexpr Real kLaneLength = 100;
constexpr const char* kTag = "test_tag";

// Two straight lanes, 100 m each, side by side in one section (the driving's own front-door test road).
engine::RoadModel twoLanes() {
    engine::RoadModel m;
    engine::LaneGraph& g = m.laneLevel;
    g.vehicleTagNames = {kTag};
    engine::Section s;
    s.lanes.count = 2;
    s.lines.count = 1;
    for (uint32_t i = 0; i < 2; ++i) {
        engine::Path p;
        p.curve = g.addCurve({engine::Vec2(0, i * 4.0), engine::Vec2(kLaneLength, i * 4.0)}, {0, 0});
        p.length = kLaneLength;
        p.vehicleTags = 1;
        p.section = engine::SectionId(0);
        p.indexInSection = i;
        g.paths.push_back(p);
        g.sectionLanes.push_back(engine::PathId(i));
    }
    engine::LaneLine line;
    line.crossFromOffside = line.crossFromKerbside = true;
    g.laneLines.push_back(line);
    g.sections.push_back(s);
    return m;
}

// Places 0, 1 and 3 are 10 m, 60 m and 90 m along lane 0; place 2 is off the road.
class TwoPlaces : public KerbLanes {
public:
    bool kerbLane(PlaceRef place, driving::PathPoint& out) const override {
        if (place.id > 3) return false;
        out.path = engine::PathId(place.id == 2 ? 9u : 0u);
        out.along = place.id == 0 ? 10 : place.id == 3 ? 90 : 60;
        return true;
    }
};

// A LANE WITH TWO WAYS INTO IT, laid by hand, for the room to pull out into (LaneRoom):
//   path 0  "the lane before"   100 m, leads into path 1
//   path 1  "the turn"           20 m, leads into path 2
//   path 2  "the kerb lane"     100 m: where a car leaving its space is put
//   path 3  "the other turn"     20 m, from the side, also leads into path 2
constexpr uint32_t kLaneBefore = 0, kTurn = 1, kKerbLane = 2, kOtherTurn = 3;
engine::RoadModel laneWithTwoWaysIn() {
    engine::RoadModel m;
    engine::LaneGraph& g = m.laneLevel;
    g.vehicleTagNames = {kTag};
    struct Laid {
        engine::Vec2 from, to;
        Real length;
        int next;   // the path it leads into (-1: none)
    };
    const Laid laid[] = {
        {engine::Vec2(0, 0), engine::Vec2(100, 0), 100, static_cast<int>(kTurn)},
        {engine::Vec2(100, 0), engine::Vec2(120, 0), 20, static_cast<int>(kKerbLane)},
        {engine::Vec2(120, 0), engine::Vec2(220, 0), 100, -1},
        {engine::Vec2(120, -20), engine::Vec2(120, 0), 20, static_cast<int>(kKerbLane)},
    };
    for (const Laid& l : laid) {
        engine::Path p;
        p.curve = g.addCurve({l.from, l.to}, {0, 0});
        p.length = l.length;
        p.vehicleTags = 1;
        if (l.next >= 0) {
            p.next.first = static_cast<uint32_t>(g.nextPaths.size());
            p.next.count = 1;
            g.nextPaths.push_back(engine::PathId(static_cast<uint32_t>(l.next)));
        }
        g.paths.push_back(p);
    }
    return m;
}

PlaceRef place(uint32_t id) {
    PlaceRef p;
    p.id = id;
    return p;
}
CarRef carRef(uint32_t id) {
    CarRef c;
    c.id = id;
    return c;
}

}  // namespace

TEST_CASE(brain_driving_skill_maps_onto_the_driving_call_for_call) {
    const engine::RoadModel road = twoLanes();
    driving::Driving d(&road);
    driving::VehicleClasses classes;
    driving::VehicleClass car = driving::deriveVehicleClass("test_car", 4.5, 1.8, 2.7, 30);
    car.tags = {kTag};
    classes.add(car);
    d.setVehicleClasses(classes);
    const TwoPlaces lanes;
    DrivingSkill skill(d, lanes, 0);

    Lab lab;
    CHECK(lab.ok);
    if (!lab.ok) return;
    std::string err;
    CHECK(skill.bindEvents(lab.table, &err));
    const AgentId who = lab.person(lab.door(0), 0, CarRef(), Detail::Near);
    const driving::CarId seven(7);

    // pullOut is placeCar: on the lane beside the space; asked again for a car already on the road, nothing changes
    CHECK(d.car(seven) == nullptr);
    CHECK(skill.pullOut(carRef(7), place(0)));
    CHECK(d.car(seven) != nullptr && d.car(seven)->path == engine::PathId(0) && d.car(seven)->along == 10);
    CHECK(skill.pullOut(carRef(7), place(1)) && d.carCount() == 1u && d.car(seven)->along == 10);
    CHECK(!skill.pullOut(carRef(8), place(2)));   // a place that is not on the road
    // beginTrip is beginTrip: this person, this car, to the lane beside that space
    CHECK(skill.beginTrip(carRef(7), who, place(0), place(1), 0));
    const driving::TripId trip = d.tripOf(seven);
    CHECK(trip.valid() && d.trip(trip)->person == who && d.trip(trip)->destination.along == 60);
    CHECK(!skill.beginTrip(carRef(7), who, place(0), place(1), 0));   // already on a trip
    // endTrip is endTrip, and the "ended early" the driving reports for it is not sent back to the brain, which asked
    skill.endTrip(carRef(7));
    CHECK(!d.tripOf(seven).valid() && d.car(seven) != nullptr);
    skill.report(*lab.machine);
    lab.tick();
    CHECK(lab.machine->counts().events == 0u);
    // ...but a trip the HOST ends is reported, to the person at the wheel, as the data's `ended_early`
    CHECK(skill.beginTrip(carRef(7), who, place(0), place(1), 0));
    CHECK(d.endTrip(d.tripOf(seven)));
    skill.report(*lab.machine);
    lab.tick();
    CHECK(lab.machine->counts().events == 1u);
    CHECK(lab.machine->counts().plans == 1u);   // (`ended_early` plans again, by the table)
    // a destination that is not on the road: no trip
    CHECK(!skill.beginTrip(carRef(7), who, place(0), place(2), 0));
    // changeDestination is changeDestination: the trip under way ends somewhere else, with the same driver
    CHECK(!skill.changeDestination(carRef(7), place(3)));   // on no trip
    CHECK(skill.beginTrip(carRef(7), who, place(0), place(1), 0));
    CHECK(skill.changeDestination(carRef(7), place(3)));
    CHECK(d.tripOf(seven).valid() && d.trip(d.tripOf(seven))->destination.along == 90 && d.trip(d.tripOf(seven))->person == who);
    CHECK(!skill.changeDestination(carRef(7), place(2)));   // a place that is not on the road: the trip is as it was
    CHECK(d.trip(d.tripOf(seven))->destination.along == 90);
    {
        // nothing of all that was reported to the person (the tick first lets the walk home that `ended_early` set off,
        // no distance at all, report its own arrival)
        lab.tick();
        const uint64_t before = lab.machine->counts().events;
        skill.report(*lab.machine);
        lab.tick();
        CHECK(lab.machine->counts().events == before);
    }
    skill.endTrip(carRef(7));
    skill.report(*lab.machine);
    // park is removeCar
    skill.park(carRef(7), place(1));
    CHECK(d.car(seven) == nullptr && d.carCount() == 0u);
    // a car not yet on the road is put there, beside the space it was planned from, when its trip begins
    CHECK(skill.beginTrip(carRef(9), who, place(0), place(1), 0));
    CHECK(d.car(driving::CarId(9)) != nullptr && d.car(driving::CarId(9))->along == 10 && d.carCount() == 1u);
    skill.endTrip(carRef(9));
    skill.report(*lab.machine);
    lab.tick();
    // a new destination there is no route to (behind the car, on a lane that leads nowhere): refused, the trip keeps
    // its old end, and the driving's own "no route" report is not passed on, since the answer already said so
    {
        CHECK(skill.pullOut(carRef(11), place(1)));
        CHECK(skill.beginTrip(carRef(11), who, place(1), place(3), 0));
        const uint64_t before = lab.machine->counts().events;
        CHECK(!skill.changeDestination(carRef(11), place(0)));
        const driving::TripId trip = d.tripOf(driving::CarId(11));
        CHECK(trip.valid() && d.trip(trip)->destination.along == 90);
        skill.report(*lab.machine);
        lab.tick();
        CHECK(lab.machine->counts().events == before);
        skill.endTrip(carRef(11));
        skill.report(*lab.machine);
    }

    // the data must name every event the driving reports
    BrainTable lacking = lab.table;
    for (std::string& name : lacking.events)
        if (name == "cannot_park") name = "somewhere_else";
    DrivingSkill other(d, lanes, 0);
    CHECK(!other.bindEvents(lacking, &err));
    CHECK(err == "brain.lua: events has no `cannot_park`, which the driving reports");
}

// ROOM TO PULL OUT INTO (ADR-0442, brain_driving.h LaneRoom): a car 4.5 m long is to be put in the kerb lane with its
// front 10 m along it, so its rear is at 5.5 m. The lane must be empty from 30 m behind the rear to 3 m ahead of the
// front. Only 5.5 m of that 30 m is on the kerb lane itself: the other 24.5 m reach back over the whole of each 20 m
// turn that leads in, and 4.5 m into the lane before the first turn (from 95.5 m to its end at 100 m).
TEST_CASE(brain_lane_room_looks_back_along_every_path_that_leads_in) {
    const engine::RoadModel road = laneWithTwoWaysIn();
    constexpr Real kLength = 4.5, kBehind = 30, kAhead = 3;
    driving::PathPoint front;
    front.path = engine::PathId(kKerbLane);
    front.along = 10;
    // One other car, its front `along` metres along `path`: is there room?
    const auto roomWith = [&](uint32_t path, Real along) {
        driving::Driving d(&road);
        driving::VehicleClasses classes;
        driving::VehicleClass car = driving::deriveVehicleClass("test_car", kLength, 1.8, 2.7, 30);
        car.tags = {kTag};
        classes.add(car);
        d.setVehicleClasses(classes);
        driving::PathPoint at;
        at.path = engine::PathId(path);
        at.along = along;
        std::string why;
        const bool placed = d.placeCar(driving::CarId(1), 0, at, {}, &why);
        if (!placed) std::printf("    the test's car was not placed: %s\n", why.c_str());
        CHECK(placed);
        const LaneRoom room(d);
        return room.clear(front, kLength, kBehind, kAhead);
    };
    {
        // what leads into what: the turn and the other turn into the kerb lane; the lane before into the turn
        driving::Driving d(&road);
        const LaneRoom room(d);
        CHECK(room.pathsInto(engine::PathId(kKerbLane)) == (std::vector<engine::PathId>{engine::PathId(kTurn), engine::PathId(kOtherTurn)}));
        CHECK(room.pathsInto(engine::PathId(kTurn)) == std::vector<engine::PathId>{engine::PathId(kLaneBefore)});
        CHECK(room.pathsInto(engine::PathId(kLaneBefore)).empty());
        CHECK(room.clear(front, kLength, kBehind, kAhead));   // an empty road
    }
    // ON THE KERB LANE: a car whose rear is within 3 m of where the front will be (its rear at 12.5 m, under 13 m) is in
    // the way; one whose rear is at 15.5 m is not
    CHECK(!roomWith(kKerbLane, 17));
    CHECK(roomWith(kKerbLane, 20));
    // ON THE TURN THAT LEADS IN: anywhere on it is within the 30 m
    CHECK(!roomWith(kTurn, 10));
    CHECK(!roomWith(kTurn, 20));
    // ON THE OTHER TURN, from the side: the same
    CHECK(!roomWith(kOtherTurn, 10));
    // ON THE LANE BEFORE THE TURN: its last 4.5 m count. A car whose front is at 97 m is in the way; one whose front is
    // at 95 m (0.5 m short of the stretch) is not
    CHECK(!roomWith(kLaneBefore, 97));
    CHECK(roomWith(kLaneBefore, 95));
}
