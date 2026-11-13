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

// travel.lua's two rows, as numbers: walk 5 km/h; car 30 km/h, 1 min to set off, 1.5 to park, reached on foot.
TravelCatalog walkAndCar() {
    TravelCatalog c;
    TravelMode walk;
    walk.name = "walk";
    walk.speed = 5.0 / 3.6;
    TravelMode car;
    car.name = "car";
    car.roads = true;
    car.speed = 30.0 / 3.6;
    car.startMinutes = 1.0;
    car.endMinutes = 1.5;
    car.ownVehicle = true;
    car.access = 0;
    c.modes = {walk, car};
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
    // and the space (1500 m) -- 1540.4 m with the bits off the street, 3.1 min -- and its 2.5 fixed minutes
    const TripPlan longTrip = planTrip(nav, cat, goTo(1500));
    CHECK(longTrip.choice()->mode == 1);
    const TripOption& car = longTrip.options[1];
    CHECK(car.possible && car.legs.size() == 3);
    CHECK(car.legs[0].mode == 0 && car.legs[1].mode == 1 && car.legs[2].mode == 0);
    const Real off = std::sqrt(20.0 * 20.0 + 3.0 * 3.0);
    CHECK(near(car.legs[1].metres, 1500 + 2 * off, 1e-6));
    CHECK(near(car.legs[1].minutes, (1500 + 2 * off) / (30.0 / 3.6) / 60.0 + 2.5, 1e-6));
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
