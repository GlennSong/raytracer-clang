// THE TRIP PLANNER (ADR-0415, src/apps/citysim/trip_plan.h): every way of travelling priced as a whole trip, leg by
// leg, and the fewest weighted minutes chosen. On a straight 2 km street (a road file built as a level's roads are,
// model_scene.h), every leg routed over the road model's walks by the one router (#306: the street map went). A leg's
// metres are its route on the walks plus the steps from each end to the walk; the numbers below are read off those, not
// worked out by hand from a hand-made map. The shipped travel.lua is read in test_agent_goals.cpp (it needs the
// scripting build); these build their table here.
#include "test_framework.h"

#include "model_scene.h"
#include "trip_driving.h"
#include "../src/apps/citysim/trip_plan.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/engine/procgen/city/roads/route.h"

#include <cmath>
#include <cstdio>

using namespace citysim;
using engine::Vec2;

namespace {

// The street, and the walker on its walks, the network every leg is routed on.
struct Street {
    const citytest::ModelScene* scene = nullptr;
    engine::roads::RouteWalker walker;
    std::shared_ptr<driving::Driving> driving;   // its router prices the drives
    TripNetwork net() const { return TripNetwork{scene->model.get(), &walker, driving.get()}; }
};
Street street(const citytest::ModelScene& s) {
    Street out;
    out.scene = &s;
    out.walker = engine::roads::bindWalker(*s.model, "walker", 1.4);
    out.driving = citytest::tripDriving(*s.model);
    return out;
}
const Street& longStreet() {
    static const Street s = street(citytest::straightScene(2000.0));
    return s;
}

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

TripRequest goTo(Real x) {
    TripRequest r;
    r.from = Vec2(20, -6);
    r.to = Vec2(x, -6);
    r.vehicles.push_back(OwnedVehicle{1, Vec2(40, -3), Vec2(x - 20, -3)});
    return r;
}

bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }

}  // namespace

TEST_CASE(trip_plan_walks_a_short_trip_and_drives_a_long_one) {
    const TripNetwork nav = longStreet().net();
    const TravelCatalog cat = walkAndCar();
    // 200 m: walking is about 2.5 min; the car's fixed 2.5 minutes alone lose
    const TripPlan shortTrip = planTrip(nav, cat, goTo(220));
    CHECK(shortTrip.valid());
    CHECK(shortTrip.options.size() == 2);
    CHECK(shortTrip.choice()->mode == 0);
    const TripLeg& walkShort = shortTrip.options[0].legs[0];
    CHECK(near(shortTrip.options[0].minutes, walkShort.metres / (5.0 / 3.6) / 60.0, 1e-9));
    CHECK(walkShort.metres >= 200.0 && walkShort.metres < 230.0);   // the sidewalk, and the steps to it at each end
    // 1500 m: the car two short walks and a drive along the street at a local street's speed, and its 2.5 fixed minutes
    const TripPlan longTrip = planTrip(nav, cat, goTo(1520));
    CHECK(longTrip.choice()->mode == 1);
    const TripOption& car = longTrip.options[1];
    CHECK(car.possible && car.legs.size() == 3);
    if (car.legs.size() != 3) return;
    CHECK(car.legs[0].mode == 0 && car.legs[1].mode == 1 && car.legs[2].mode == 0);
    CHECK(car.legs[1].metres >= 1460.0 && car.legs[1].metres < 1500.0);
    CHECK(near(car.legs[1].minutes, car.legs[1].metres / engine::classSpeed(engine::RoadClass::Local) / 60.0 + 2.5, 1e-6));
    CHECK(near(kindMetres(cat, car.legs[1], "streets"), car.legs[1].metres, 1e-6));
    CHECK(describeRoads(cat, car.legs[0]).empty());   // (a walk has no roads to tell apart)
    CHECK(near(car.minutes, car.legs[0].minutes + car.legs[1].minutes + car.legs[2].minutes, 1e-9));
    CHECK(car.minutes < longTrip.options[0].minutes);
    std::printf("    %s\n    %s\n", shortTrip.reason.c_str(), longTrip.reason.c_str());
    // the reason names both, the legs of the car, and the choice last
    CHECK(longTrip.reason.find("walk ") == 0 && longTrip.reason.find(", car ") != std::string::npos);
    CHECK(longTrip.reason.find(" walk + ") != std::string::npos && longTrip.reason.find(" car + ") != std::string::npos);
    CHECK(longTrip.reason.size() > 5 && longTrip.reason.compare(longTrip.reason.size() - 5, 5, ": car") == 0);
}

TEST_CASE(trip_plan_legs_follow_the_way_s_network_end_to_end) {
    const TripNetwork nav = longStreet().net();
    const TravelCatalog cat = walkAndCar();
    const TripPlan p = planTrip(nav, cat, goTo(1520));
    const TripOption& car = p.options[1];
    CHECK(car.legs.size() == 3);
    if (car.legs.size() != 3) return;
    // each leg starts where the last one ended: the person -> the car -> where it is left -> the door
    CHECK(near((car.legs[0].from - p.from).length(), 0, 1e-9));
    CHECK(near((car.legs[0].to - car.legs[1].from).length(), 0, 1e-9));
    CHECK(near((car.legs[1].to - car.legs[2].from).length(), 0, 1e-9));
    CHECK(near((car.legs[2].to - p.to).length(), 0, 1e-9));
    // the drive is routed by the driving's router (the one its cars drive by), kerb lane to kerb lane, not over the
    // walks: its metres and minutes are that route's, every metre of it on the street kind of road
    const TripLeg& drive = car.legs[1];
    CHECK(!drive.route.found);
    const engine::KerbLanePlace a = longStreet().scene->model->kerbLaneBeside(longStreet().scene->model->frontageAt(drive.from));
    const engine::KerbLanePlace b = longStreet().scene->model->kerbLaneBeside(longStreet().scene->model->frontageAt(drive.to));
    CHECK(a.valid() && b.valid());
    if (!a.valid() || !b.valid()) return;
    engine::PathPoint pa, pb;
    pa.path = a.path;
    pa.along = a.along;
    pb.path = b.path;
    pb.along = b.along;
    const engine::roads::Route r = longStreet().driving->routeBetween(0, pa, pb);
    std::printf("    drive %.1f m in %.2f min (the router: %.1f m in %.1f s)\n", drive.metres, drive.minutes, r.metres, r.seconds);
    CHECK(r.found && near(drive.metres, r.metres, 1e-9) && near(r.metres, 1460.0, 1.0));
    CHECK(near(drive.minutes, r.seconds / 60.0 + cat.modes[1].startMinutes + cat.modes[1].endMinutes, 1e-9));
    CHECK(drive.kindMetres.size() == 3 && near(drive.kindMetres[0], drive.metres, 1e-6) && drive.kindStretches[0] == 1);
    const TripLeg& w = p.options[0].legs[0];
    CHECK(w.route.found && w.metres >= w.route.metres - 1e-9 && w.metres < w.route.metres + 20.0);
}

TEST_CASE(trip_plan_without_a_car_walks_and_says_why) {
    const TripNetwork nav = longStreet().net();
    const TravelCatalog cat = walkAndCar();
    TripRequest r = goTo(1520);
    r.vehicles.clear();
    const TripPlan p = planTrip(nav, cat, r);
    CHECK(p.choice()->mode == 0);
    CHECK(!p.options[1].possible && p.options[1].legs.empty());
    CHECK(p.reason.find("walk ") == 0);
    CHECK(p.reason.find(", car -- no car: walk") != std::string::npos);
}

TEST_CASE(trip_plan_weight_counts_a_minute_the_person_minds) {
    const TripNetwork nav = longStreet().net();
    TravelCatalog cat = walkAndCar();
    cat.modes[1].weight = 4.0;   // someone who hates driving: the car's ~6 min count as ~24
    const TripPlan p = planTrip(nav, cat, goTo(1520));
    CHECK(p.choice()->mode == 0);
    CHECK(p.options[1].weighted > p.options[0].weighted);
    CHECK(near(p.options[1].weighted, p.options[1].legs[0].minutes + 4.0 * p.options[1].legs[1].minutes +
                                          p.options[1].legs[2].minutes, 1e-9));
}

TEST_CASE(trip_plan_ties_go_to_the_earlier_row) {
    const TripNetwork nav = longStreet().net();
    TravelCatalog cat = walkAndCar();
    TravelMode stroll = cat.modes[0];
    stroll.name = "stroll";
    cat.modes.insert(cat.modes.begin(), stroll);   // the same way twice, "stroll" first
    cat.modes[2].access = 1;
    const TripPlan p = planTrip(nav, cat, goTo(220));
    CHECK(p.options[0].weighted == p.options[1].weighted);
    CHECK(p.chosen == 0);
}

TEST_CASE(trip_plan_a_way_ruled_out_is_not_taken) {
    // a caller re-plans without a way whose leg could not be done
    const TripNetwork nav = longStreet().net();
    const TravelCatalog cat = walkAndCar();
    TripRequest r = goTo(1520);
    r.ruledOut = {1};
    const TripPlan p = planTrip(nav, cat, r);
    CHECK(p.choice()->mode == 0);
    CHECK(!p.options[1].possible);
}

TEST_CASE(trip_plan_no_route_on_its_network_is_no_option) {
    // two streets with nothing between them (each a T, as a road file with a junction gets a lane level): a
    // destination on the other one cannot be reached by either way
    engine::RoadGraph g;
    for (int s = 0; s < 2; ++s) {
        const int base = static_cast<int>(g.nodes.size());
        const Real x0 = 5000.0 * s;
        g.nodes.push_back({Vec2(x0, 0)});
        g.nodes.push_back({Vec2(x0 + 200, 0)});
        g.nodes.push_back({Vec2(x0 + 200, -30)});
        g.nodes.push_back({Vec2(x0 + 200, 30)});
        for (int k = 1; k <= 3; ++k)
            g.edges.push_back(engine::RoadEdge{k == 1 ? base : base + 1, base + k, 8, engine::RoadClass::Local, 0});
    }
    static const Street two = street(citytest::graphScene("trip_two_streets", g));
    const TripNetwork nav = two.net();
    const TravelCatalog cat = walkAndCar();
    TripRequest r;
    r.from = Vec2(20, -6);
    r.to = Vec2(5100, -6);
    r.vehicles.push_back(OwnedVehicle{1, Vec2(30, -3), Vec2(5090, -3)});
    const TripPlan p = planTrip(nav, cat, r);
    CHECK(!p.valid());
    CHECK(p.options[0].why == "no route on the footpaths");
    CHECK(p.options[1].why == "no route by car");
    CHECK(p.reason.find(": no way there") != std::string::npos);
}

// (trip_plan_drive_minutes_are_the_router_s_road_class_times went with the street map, #306: it priced a drive up a
// ramp and along a freeway over the street map's links. A drive is now priced over the road model's walks, each walk
// at its street's class speed (trip_plan.h), so a freeway, which has no sidewalk, is not priced: on the known-bugs list
// until a drive is priced by the driving's own lane router.)

// WHAT THE AGENT BRAIN NEEDED OF THE PLANNER (ADR-0440; agent-brain-design.md section 12): a way priced only for people
// who have what it requires and only for a leg it suits; a vehicle reached and left by the cheapest way on foot the
// person has, unless the table fixes one; and a household's car not priced while someone else has it.
TEST_CASE(trip_plan_traits_longest_leg_cheapest_way_to_the_car_and_a_car_in_use) {
    const TripNetwork nav = longStreet().net();
    TravelCatalog cat = walkAndCar();
    cat.modes[1].access = -1;   // the car fixes no way to it
    cat.traits = {"has_board"};
    TravelMode board;
    board.name = "skateboard";
    board.speed = 10.0 / 3.6;
    board.needs = "has_board";
    board.needsTrait = 0;
    board.longestLegMinutes = 6.0;   // 1000 m
    board.weight = 1.2;
    cat.modes.push_back(board);
    auto ask = [](Real x, uint32_t traits) {
        TripRequest r;
        r.from = Vec2(20, -6);
        r.to = Vec2(x, -6);
        r.traits = traits;
        r.vehicles.push_back(OwnedVehicle{1, Vec2(120, -3), Vec2(x - 200, -3)});
        return r;
    };
    // NO BOARD: the row is not theirs. It is not priced, and the reason does not mention it.
    const TripPlan without = planTrip(nav, cat, ask(520, 0));
    CHECK(without.options.size() == 3 && !without.options[2].offered && !without.options[2].possible);
    CHECK(without.options[2].why == "needs has_board");
    CHECK(without.reason.find("skateboard") == std::string::npos);
    CHECK(without.choice()->mode == 0);
    // WITH ONE, ~500 m: about 3 minutes on the board, minded as 1.2 times that, against about 6 on foot
    const TripPlan with = planTrip(nav, cat, ask(520, 1));
    CHECK(with.choice()->mode == 2 && with.choice()->legs.size() == 1);
    CHECK(near(with.options[2].weighted, 1.2 * with.options[2].minutes, 1e-9));
    CHECK(with.options[2].minutes > 2.9 && with.options[2].minutes < 3.3);
    // ITS LONGEST LEG: ~900 m is allowed (under 6 minutes); ~1500 m is not
    CHECK(planTrip(nav, cat, ask(920, 1)).options[2].possible);
    TripRequest far = ask(1520, 1);
    far.vehicles.clear();
    const TripPlan tooFar = planTrip(nav, cat, far);
    CHECK(!tooFar.options[2].possible && tooFar.options[2].offered);
    CHECK(tooFar.options[2].why.find("too far for skateboard (") == 0 &&
          tooFar.options[2].why.find(" min, at most 6.0)") != std::string::npos);
    CHECK(tooFar.choice()->mode == 0);
    // THE CAR IS REACHED AND LEFT BY THE CHEAPEST WAY ON FOOT THEY HAVE: on the board for someone with one, on foot
    // for someone without
    const TripPlan drives = planTrip(nav, cat, ask(1520, 1));
    CHECK(drives.choice()->mode == 1);
    const TripOption& car = drives.options[1];
    CHECK(car.legs.size() == 3 && car.legs[0].mode == 2 && car.legs[1].mode == 1 && car.legs[2].mode == 2);
    if (car.legs.size() == 3) {
        CHECK(car.legs[0].end == LegEnd::Vehicle && car.legs[1].end == LegEnd::VehicleLeft && car.legs[2].end == LegEnd::Door);
        CHECK(near(car.legs[0].minutes, car.legs[0].metres / (10.0 / 3.6) / 60.0, 1e-9));
    }
    const TripPlan drivesOnFoot = planTrip(nav, cat, ask(1520, 0));
    CHECK(drivesOnFoot.options[1].legs.size() == 3 && drivesOnFoot.options[1].legs[0].mode == 0 &&
          drivesOnFoot.options[1].legs[2].mode == 0);
    // ...unless the table fixes the way (`access`): then that way, board or no board
    TravelCatalog fixed = cat;
    fixed.modes[1].access = 0;
    const TripPlan fixedWay = planTrip(nav, fixed, ask(1520, 1));
    CHECK(fixedWay.options[1].legs.size() == 3 && fixedWay.options[1].legs[0].mode == 0 && fixedWay.options[1].legs[2].mode == 0);
    // ...and a way on foot that is ruled out is not how the car is reached
    TripRequest boardBroken = ask(1520, 1);
    boardBroken.ruledOut = {2};
    const TripPlan broken = planTrip(nav, cat, boardBroken);
    CHECK(!broken.options[1].legs.empty() && broken.options[1].legs[0].mode == 0);
    // THE HOUSEHOLD'S CAR IN USE: said by the caller at plan time; the car is not priced
    TripRequest inUse = ask(1520, 0);
    inUse.vehicles[0].free = false;
    const TripPlan walks = planTrip(nav, cat, inUse);
    CHECK(!walks.options[1].possible && walks.options[1].why == "the car is in use");
    CHECK(walks.choice()->mode == 0);
    CHECK(walks.reason.find(", car -- the car is in use: walk") != std::string::npos);
}
