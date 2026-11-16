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

// Places 0 and 1 are 10 m and 60 m along lane 0; place 2 is off the road.
class TwoPlaces : public KerbLanes {
public:
    bool kerbLane(PlaceRef place, driving::PathPoint& out) const override {
        if (place.id > 2) return false;
        out.path = engine::PathId(place.id == 2 ? 9u : 0u);
        out.along = place.id == 0 ? 10 : 60;
        return true;
    }
};

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
    // park is removeCar
    skill.park(carRef(7), place(1));
    CHECK(d.car(seven) == nullptr && d.carCount() == 0u);
    // a car not yet on the road is put there, beside the space it was planned from, when its trip begins
    CHECK(skill.beginTrip(carRef(9), who, place(0), place(1), 0));
    CHECK(d.car(driving::CarId(9)) != nullptr && d.car(driving::CarId(9))->along == 10 && d.carCount() == 1u);
    skill.endTrip(carRef(9));

    // the data must name every event the driving reports
    BrainTable lacking = lab.table;
    for (std::string& name : lacking.events)
        if (name == "cannot_park") name = "somewhere_else";
    DrivingSkill other(d, lanes, 0);
    CHECK(!other.bindEvents(lacking, &err));
    CHECK(err == "brain.lua: events has no `cannot_park`, which the driving reports");
}
