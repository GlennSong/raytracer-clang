// THE TRIP PLANNER (ADR-0415, src/apps/citysim/trip_plan.h): every way of travelling priced as a whole trip, leg by
// leg, and the fewest weighted minutes chosen. On a straight 2 km street with a node every 100 m, so every number
// below can be worked out by hand. The shipped travel.lua is read in test_agent_goals.cpp (it needs the scripting
// build); these build their table here.
#include "test_framework.h"

#include "../src/apps/citysim/trip_plan.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cmath>
#include <cstdio>

using namespace citysim;
using engine::Vec2;

namespace {

// A street along x from 0 to 2000 m, a node every 100 m: walkable and drivable alike.
engine::NavGraph longStreet() {
    engine::RoadGraph g;
    for (int i = 0; i <= 20; ++i) g.nodes.push_back({Vec2(100.0 * i, 0)});
    for (int i = 0; i < 20; ++i) g.edges.push_back(engine::RoadEdge{i, i + 1, 8, engine::RoadClass::Local, 0});
    return engine::buildNavGraph(g);
}

// travel.lua's two rows, as numbers: walk 5 km/h; car at the router's road-class speeds, 1 min to set off, 1.5 to
// park, reached on foot. Streets, freeway and ramps told apart as travel.lua does.
TravelCatalog walkAndCar() {
    TravelCatalog c;
    TravelMode walk;
    walk.name = "walk";
    walk.speed = 5.0 / 3.6;
    TravelMode car;
    car.name = "car";
    car.roads = true;
    car.speed = 0;
    car.startMinutes = 1.0;
    car.endMinutes = 1.5;
    car.ownVehicle = true;
    car.access = 0;
    c.modes = {walk, car};
    RoadKind streets, freeway, ramps;
    streets.name = "streets";
    streets.classes = {engine::RoadClass::Arterial, engine::RoadClass::Collector, engine::RoadClass::Local};
    freeway.name = "freeway";
    freeway.classes = {engine::RoadClass::Freeway};
    ramps.name = "ramps";
    ramps.one = "ramp";
    ramps.count = true;
    ramps.classes = {engine::RoadClass::Ramp};
    c.roadKinds = {streets, freeway, ramps};
    return c;
}

// The person at the street's start, their car parked 20 m along it, left 20 m short of the door.
TripRequest goTo(Real x) {
    TripRequest r;
    r.from = Vec2(0, 6);
    r.to = Vec2(x, 6);
    r.vehicles.push_back(OwnedVehicle{1, Vec2(20, 3), Vec2(x - 20, 3)});
    return r;
}

bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }

}  // namespace

TEST_CASE(trip_plan_walks_a_short_trip_and_drives_a_long_one) {
    const engine::NavGraph nav = longStreet();
    const TravelCatalog cat = walkAndCar();
    // 200 m (6 m off the street at each end: 212 m on foot): walking is 2.5 min; the car's fixed 2.5 minutes alone lose
    const TripPlan shortTrip = planTrip(nav, cat, goTo(200));
    CHECK(shortTrip.valid());
    CHECK(shortTrip.options.size() == 2);
    CHECK(shortTrip.choice()->mode == 0);
    CHECK(near(shortTrip.options[0].minutes, 212.0 / (5.0 / 3.6) / 60.0, 1e-6));
    // 1500 m: walking 1512 m is 18.1 min; the car two short walks, a drive between the nodes nearest the car (0 m)
    // and the space (1500 m) -- 1540.4 m with the bits off the street, on a local street at the router's 8 m/s (no
    // junctions on the way: every node is a street's bend), 3.2 min -- and its 2.5 fixed minutes
    const TripPlan longTrip = planTrip(nav, cat, goTo(1500));
    CHECK(longTrip.choice()->mode == 1);
    const TripOption& car = longTrip.options[1];
    CHECK(car.possible && car.legs.size() == 3);
    CHECK(car.legs[0].mode == 0 && car.legs[1].mode == 1 && car.legs[2].mode == 0);
    const Real off = std::sqrt(20.0 * 20.0 + 3.0 * 3.0);
    CHECK(near(car.legs[1].metres, 1500 + 2 * off, 1e-6));
    CHECK(near(car.legs[1].minutes, (1500 + 2 * off) / engine::classSpeed(engine::RoadClass::Local) / 60.0 + 2.5, 1e-6));
    CHECK(describeRoads(cat, car.legs[1]) == "1,540 m streets");
    CHECK(describeRoads(cat, car.legs[0]).empty());   // (a walk has no roads to tell apart)
    CHECK(near(car.minutes, car.legs[0].minutes + car.legs[1].minutes + car.legs[2].minutes, 1e-9));
    CHECK(car.minutes < longTrip.options[0].minutes);
    std::printf("    %s\n    %s\n", shortTrip.reason.c_str(), longTrip.reason.c_str());
    // the reason names both, the legs of the car, and the choice last
    CHECK(longTrip.reason.find("walk 18.1 min, car ") == 0);
    CHECK(longTrip.reason.find(" walk + ") != std::string::npos && longTrip.reason.find(" car + ") != std::string::npos);
    CHECK(longTrip.reason.size() > 5 && longTrip.reason.compare(longTrip.reason.size() - 5, 5, ": car") == 0);
}

TEST_CASE(trip_plan_legs_follow_the_way_s_network_end_to_end) {
    const engine::NavGraph nav = longStreet();
    const TravelCatalog cat = walkAndCar();
    const TripPlan p = planTrip(nav, cat, goTo(1500));
    const TripOption& car = p.options[1];
    // each leg starts where the last one ended: the person -> the car -> where it is left -> the door
    CHECK(near((car.legs[0].from - p.from).length(), 0, 1e-9));
    CHECK(near((car.legs[0].to - car.legs[1].from).length(), 0, 1e-9));
    CHECK(near((car.legs[1].to - car.legs[2].from).length(), 0, 1e-9));
    CHECK(near((car.legs[2].to - p.to).length(), 0, 1e-9));
    // the drive is routed: its links run from the node by the car to the node by the space
    CHECK(car.legs[1].route.valid());
    CHECK(nav.links[static_cast<std::size_t>(car.legs[1].route.links.front())].from == car.legs[1].fromNode);
    CHECK(nav.links[static_cast<std::size_t>(car.legs[1].route.links.back())].to == car.legs[1].toNode);
    // a leg's metres are its route plus the bits off the network at either end
    const TripLeg& w = p.options[0].legs[0];
    CHECK(near(w.metres, w.route.length(nav) + (w.from - nav.nodes[static_cast<std::size_t>(w.fromNode)]).length() +
                             (nav.nodes[static_cast<std::size_t>(w.toNode)] - w.to).length(), 1e-9));
}

TEST_CASE(trip_plan_without_a_car_walks_and_says_why) {
    const engine::NavGraph nav = longStreet();
    const TravelCatalog cat = walkAndCar();
    TripRequest r = goTo(1500);
    r.vehicles.clear();
    const TripPlan p = planTrip(nav, cat, r);
    CHECK(p.choice()->mode == 0);
    CHECK(!p.options[1].possible && p.options[1].legs.empty());
    CHECK(p.reason == "walk 18.1 min, car -- no car: walk");
}

TEST_CASE(trip_plan_weight_counts_a_minute_the_person_minds) {
    const engine::NavGraph nav = longStreet();
    TravelCatalog cat = walkAndCar();
    cat.modes[1].weight = 4.0;   // someone who hates driving: the car's ~6 min count as ~24
    const TripPlan p = planTrip(nav, cat, goTo(1500));
    CHECK(p.choice()->mode == 0);
    CHECK(p.options[1].weighted > p.options[0].weighted);
    CHECK(near(p.options[1].weighted, p.options[1].legs[0].minutes + 4.0 * p.options[1].legs[1].minutes +
                                          p.options[1].legs[2].minutes, 1e-9));
}

TEST_CASE(trip_plan_ties_go_to_the_earlier_row) {
    const engine::NavGraph nav = longStreet();
    TravelCatalog cat = walkAndCar();
    TravelMode stroll = cat.modes[0];
    stroll.name = "stroll";
    cat.modes.insert(cat.modes.begin(), stroll);   // the same way twice, "stroll" first
    cat.modes[2].access = 1;
    const TripPlan p = planTrip(nav, cat, goTo(200));
    CHECK(p.options[0].weighted == p.options[1].weighted);
    CHECK(p.chosen == 0);
}

TEST_CASE(trip_plan_a_way_ruled_out_is_not_taken) {
    // city_trip re-plans without a way whose leg could not be done
    const engine::NavGraph nav = longStreet();
    const TravelCatalog cat = walkAndCar();
    TripRequest r = goTo(1500);
    r.ruledOut = {1};
    const TripPlan p = planTrip(nav, cat, r);
    CHECK(p.choice()->mode == 0);
    CHECK(!p.options[1].possible);
}

TEST_CASE(trip_plan_no_route_on_its_network_is_no_option) {
    // two streets with nothing between them: a destination on the other one cannot be reached by either way
    // (ten nodes each, more than the planner tries at either end, so it never reaches across for the other street's)
    engine::RoadGraph g;
    for (int s = 0; s < 2; ++s) {
        const int base = static_cast<int>(g.nodes.size());
        for (int i = 0; i < 10; ++i) g.nodes.push_back({Vec2(5000.0 * s + 15.0 * i, 0)});
        for (int i = 0; i < 9; ++i) g.edges.push_back(engine::RoadEdge{base + i, base + i + 1, 8, engine::RoadClass::Local, 0});
    }
    engine::NavBuildParams bp;
    bp.junctionMergeRadius = 0;
    const engine::NavGraph nav = engine::buildNavGraph(g, bp);
    const TravelCatalog cat = walkAndCar();
    TripRequest r;
    r.from = Vec2(0, 5);
    r.to = Vec2(5100, 5);
    r.vehicles.push_back(OwnedVehicle{1, Vec2(10, 3), Vec2(5090, 3)});
    const TripPlan p = planTrip(nav, cat, r);
    CHECK(!p.valid());
    CHECK(p.options[0].why == "no route on the footpaths");
    CHECK(p.options[1].why == "no route by car");
    CHECK(p.reason.find(": no way there") != std::string::npos);
}

// A DRIVE'S MINUTES ARE THE ROUTER'S (ADR-0416): the planner prices a road leg by the time findRoute chose its route
// by -- each road class at its speed, a few seconds a street junction -- not by one flat speed. Two ways from one end
// of a town to the other: straight along a 2 km local street with a cross-street every 200 m, or up a ramp onto a
// freeway beside it and down another. The freeway is longer and wins on time; the plan says so, and its minutes,
// metres and kinds of road are the route's.
TEST_CASE(trip_plan_drive_minutes_are_the_router_s_road_class_times) {
    engine::RoadGraph g;
    // the street: x 0..2000 at z 0, a node every 100 m; a stub north at every 200 m, so those nodes are junctions
    for (int i = 0; i <= 20; ++i) g.nodes.push_back({Vec2(100.0 * i, 0)});
    for (int i = 0; i < 20; ++i) g.edges.push_back(engine::RoadEdge{i, i + 1, 8, engine::RoadClass::Local, 0});
    for (int i = 2; i < 20; i += 2) {
        g.nodes.push_back({Vec2(100.0 * i, -60)});
        g.edges.push_back(engine::RoadEdge{i, static_cast<int>(g.nodes.size()) - 1, 8, engine::RoadClass::Local, 0});
    }
    // the freeway 150 m north, x 100..1900, a ramp down to the street at each end
    const int f0 = static_cast<int>(g.nodes.size());
    for (int i = 0; i <= 18; ++i) g.nodes.push_back({Vec2(100.0 + 100.0 * i, 150)});
    for (int i = 0; i < 18; ++i) g.edges.push_back(engine::RoadEdge{f0 + i, f0 + i + 1, 12, engine::RoadClass::Freeway, 0});
    g.edges.push_back(engine::RoadEdge{0, f0, 8, engine::RoadClass::Ramp, 0});
    g.edges.push_back(engine::RoadEdge{f0 + 18, 20, 8, engine::RoadClass::Ramp, 0});
    engine::NavBuildParams bp;
    bp.junctionMergeRadius = 0;
    const engine::NavGraph nav = engine::buildNavGraph(g, bp);
    const TravelCatalog cat = walkAndCar();
    TripLeg drive;
    CHECK(planLeg(nav, cat, 1, Vec2(0, 0), Vec2(2000, 0), drive));
    CHECK(drive.route.valid());
    // the router took the freeway: 1,800 m of it and two ramps (each ~180 m), against 2,000 m of street and nine
    // junctions at 6 s each
    Real fw = 0, street = 0;
    for (int li : drive.route.links) {
        const engine::NavLink& L = nav.links[static_cast<std::size_t>(li)];
        (L.klass == engine::RoadClass::Freeway ? fw : street) += L.klass == engine::RoadClass::Ramp ? 0 : L.length;
    }
    std::printf("    drive: %.0f m, %.2f min: %s\n", drive.metres, drive.minutes, describeRoads(cat, drive).c_str());
    CHECK(near(fw, 1800, 1.0) && street < 1.0);
    // its minutes are the router's time for that route, plus the car's fixed 2.5
    CHECK(near(drive.minutes, engine::routeSeconds(nav, drive.route) / 60.0 + 2.5, 1e-9));
    CHECK(near(engine::routeSeconds(nav, drive.route),
               1800.0 / engine::classSpeed(engine::RoadClass::Freeway) +
                   (drive.route.length(nav) - 1800.0) / engine::classSpeed(engine::RoadClass::Ramp), 1e-6));
    // ...not one flat speed over its metres: the same metres at the old 30 km/h would be 4.2 min against ~1.6
    CHECK(drive.minutes - 2.5 < drive.metres / (30.0 / 3.6) / 60.0 * 0.6);
    // and its kinds of road, as the plan says them
    CHECK(near(kindMetres(cat, drive, "freeway"), 1800, 1.0));
    CHECK(near(kindMetres(cat, drive, "streets"), 0, 1e-9));
    CHECK(describeRoads(cat, drive) == "1,800 m freeway, 2 ramps");
    // the street's own time, for the record: 2,000 m at 8 m/s and 9 junctions -- longer than the freeway's
    const Real streetSeconds = 2000.0 / engine::classSpeed(engine::RoadClass::Local) + 9 * 6.0;
    CHECK(engine::routeSeconds(nav, drive.route) < streetSeconds);
}

