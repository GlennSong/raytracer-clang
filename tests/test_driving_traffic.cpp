// Each car's state and each path's cars in order (#249): placing, asking for and removing cars, and every refusal;
// a car's pose read off the curve (straight, through a bend, offset toward the kerb under either side of the road, on
// a slope); a long vehicle listed on three short paths; a car changing lanes listed in both; the car ahead across many
// tiny paths and at the look-ahead limit; the car behind and who is on a stretch; and the same lists, answers and hash
// whatever order the cars are stored in. Every graph is a few paths built by hand.

#include "test_framework.h"

#include "apps/driving/driving.h"
#include "apps/driving/traffic_state.h"
#include "engine/procgen/city/polyline.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace driving;

namespace {

constexpr Real kEps = 1e-9;
constexpr Real kPi = 3.14159265358979323846;

// The vehicle classes every test here uses, by index.
constexpr uint32_t kCar = 0;     // 4.5 m
constexpr uint32_t kLong = 1;    // 12 m
constexpr uint32_t kShort = 2;   // 0.4 m: shorter than the 0.5 m paths
constexpr Real kCarLength = 4.5, kLongLength = 12, kShortLength = 0.4;

VehicleClasses testClasses() {
    VehicleClasses c;
    c.add(deriveVehicleClass("test_car", kCarLength, 1.8, 2.7, 30));
    c.add(deriveVehicleClass("test_long", kLongLength, 2.5, 7.0, 35));
    c.add(deriveVehicleClass("test_short", kShortLength, 0.3, 0.25, 30));
    return c;
}

engine::PathId pid(uint32_t i) { return engine::PathId(i); }

PathPoint at(uint32_t path, Real along) {
    PathPoint p;
    p.path = pid(path);
    p.along = along;
    return p;
}

// Straight paths laid end to end along +x, flat, each leading into the next.
engine::LaneGraph chain(const std::vector<Real>& lengths) {
    engine::LaneGraph g;
    Real x = 0;
    for (Real len : lengths) {
        engine::Path p;
        p.curve = g.addCurve({engine::Vec2(x, 0), engine::Vec2(x + len, 0)}, {0, 0});
        p.length = len;
        g.paths.push_back(p);
        x += len;
    }
    for (std::size_t i = 0; i + 1 < g.paths.size(); ++i) {
        g.paths[i].next.first = static_cast<uint32_t>(g.nextPaths.size());
        g.paths[i].next.count = 1;
        g.nextPaths.push_back(pid(static_cast<uint32_t>(i + 1)));
    }
    return g;
}

// Two straight lanes side by side, 100 m each, 4 m apart.
constexpr Real kLaneLength = 100;
engine::LaneGraph twoLanes() {
    engine::LaneGraph g;
    for (int i = 0; i < 2; ++i) {
        engine::Path p;
        p.curve = g.addCurve({engine::Vec2(0, i * 4.0), engine::Vec2(kLaneLength, i * 4.0)}, {0, 0});
        p.length = kLaneLength;
        g.paths.push_back(p);
    }
    return g;
}

// One path from a polyline (heights optional), its length from the curve.
engine::LaneGraph onePath(const std::vector<engine::Vec2>& pts, std::vector<Real> heights = {}) {
    engine::LaneGraph g;
    heights.resize(pts.size(), 0);
    engine::Path p;
    p.curve = g.addCurve(pts, heights);
    p.length = g.along[p.curve.end() - 1];
    g.paths.push_back(p);
    return g;
}

Car record(uint32_t id, uint32_t cls, uint32_t path, Real along, Real speed = 0) {
    Car c;
    c.id = CarId(id);
    c.vehicleClass = cls;
    c.path = pid(path);
    c.along = along;
    c.speed = speed;
    return c;
}

bool says(const std::string& why, const std::string& words) { return why.find(words) != std::string::npos; }

Real angleBetween(const engine::Vec3& a, const engine::Vec3& b) {
    return std::acos(std::clamp(engine::dot(a, b), Real(-1), Real(1)));
}

const PathEntry* entryOf(const PathList& list, uint32_t car) {
    for (const PathEntry& e : list)
        if (e.car == CarId(car)) return &e;
    return nullptr;
}

}  // namespace

// ---- 1. place, query, remove; each refusal ---------------------------------------------------------------------

TEST_CASE(driving_car_place_query_remove) {
    const engine::LaneGraph g = chain({20, 30});
    Driving d(&g);
    d.setVehicleClasses(testClasses());
    std::string why;

    CHECK(d.placeCar(CarId(1), kCar, at(1, 10), {}, &why));
    const Car* c = d.car(CarId(1));
    CHECK(c != nullptr);
    if (!c) return;
    CHECK(c->path == pid(1));
    CHECK_APPROX(c->along, 10, kEps);
    CHECK(c->speed == 0 && c->offset == 0 && !c->driven);
    CHECK(c->trailCount == 0);
    CHECK(d.carCount() == 1u);

    CHECK(!d.placeCar(CarId(1), kCar, at(1, 20), {}, &why));
    CHECK(says(why, "already placed"));
    CHECK(!d.placeCar(CarId(2), 9, at(1, 20), {}, &why));
    CHECK(says(why, "no vehicle class 9"));
    CHECK(!d.placeCar(CarId(2), kCar, at(7, 20), {}, &why));
    CHECK(says(why, "no path 7"));
    CHECK(!d.placeCar(CarId(2), kCar, at(1, 31), {}, &why));
    CHECK(says(why, "is off path 1"));
    CHECK(!d.placeCar(CarId(2), kCar, at(1, -1), {}, &why));
    CHECK(says(why, "is off path 1"));
    CHECK(!d.placeCar(CarId(2), kCar, at(1, 2), {}, &why));                 // 2.5 m of body behind the path's start
    CHECK(says(why, "does not fit"));
    CHECK(!d.placeCar(CarId(2), kCar, at(0, 10), {pid(1)}, &why));         // path 1 leads nowhere, least of all to 0
    CHECK(says(why, "does not lead into path 0"));
    CHECK(!d.placeCar(CarId(2), kCar, at(1, 2), {pid(9)}, &why));
    CHECK(says(why, "path 9, which is not on the road"));
    CHECK(!d.placeCar(CarId(2), kCar, at(1, 2), std::vector<engine::PathId>(kMaxTrail + 1, pid(0)), &why));
    CHECK(says(why, "more than the"));
    CHECK(!d.placeCar(CarId(), kCar, at(1, 2), {}, &why));
    CHECK(d.carCount() == 1u);

    // A body reaching back onto the path before; a trail longer than the body needs is cut to what it covers.
    CHECK(d.placeCar(CarId(2), kCar, at(1, 2), {pid(0)}, &why));
    CHECK(d.car(CarId(2))->trailCount == 1 && d.car(CarId(2))->trail[0] == pid(0));
    CHECK(d.placeCar(CarId(3), kCar, at(1, 25), {pid(0)}, &why));
    CHECK(d.car(CarId(3))->trailCount == 0);

    // A car whose body needs a longer trail than any car may have.
    const engine::LaneGraph tiny = chain(std::vector<Real>(kMaxTrail + 2, 0.5));
    Driving t(&tiny);
    t.setVehicleClasses(testClasses());
    std::vector<engine::PathId> fullTrail;
    for (uint32_t i = 0; i < kMaxTrail; ++i) fullTrail.push_back(pid(static_cast<uint32_t>(kMaxTrail - i)));
    CHECK(!t.placeCar(CarId(1), kLong, at(static_cast<uint32_t>(kMaxTrail + 1), 0.5), fullTrail, &why));
    CHECK(says(why, "reaches back over more than"));
    CHECK(t.placeCar(CarId(1), kShort, at(static_cast<uint32_t>(kMaxTrail + 1), 0.5), fullTrail, &why));

    // Remove: not while on a trip; then gone, from the records and the lists.
    const TripId trip = d.beginTrip(CarId(2), 77, at(1, 28));
    CHECK(trip.valid());
    CHECK(!d.removeCar(CarId(2), &why));
    CHECK(says(why, "on a trip"));
    CHECK(d.endTrip(trip));
    CHECK(d.pathLists().on(pid(0)).size() == 1u);   // car 2's tail
    CHECK(d.removeCar(CarId(2), &why));
    CHECK(d.car(CarId(2)) == nullptr);
    CHECK(d.carCount() == 2u);
    CHECK(d.pathLists().on(pid(0)).empty());
    CHECK(!d.removeCar(CarId(2), &why));
    CHECK(says(why, "not placed"));
    CHECK(!d.beginTrip(CarId(2), 77, at(1, 28)).valid());   // a trip needs a placed car
}

// ---- 2. the pose ------------------------------------------------------------------------------------------------

TEST_CASE(driving_car_pose_straight_and_sideways) {
    const engine::LaneGraph g = chain({100});
    const VehicleClasses classes = testClasses();
    RuleSet rules;
    Car c = record(1, kCar, 0, 30);
    CarPose p = carPose(g, classes, rules, c);
    CHECK_APPROX(p.front.x, 30, kEps);
    CHECK_APPROX(p.front.y, 0, kEps);
    CHECK_APPROX(p.front.z, 0, kEps);
    CHECK_APPROX(p.rear.x, 30 - kCarLength, kEps);
    CHECK_APPROX(p.travel.x, 1, kEps);
    CHECK_APPROX(p.body.x, 1, kEps);

    // Heading +x in plan (x, z), height up. perp turns it a quarter turn clockwise seen from above, to +z. Keeping to
    // the side DrivesOn::Right names, that is the kerb side; keeping to the other, the kerb is at -z. A positive offset
    // moves toward the kerb under both; a negative one toward the offside.
    const Real kOffset = 1.25;
    c.offset = kOffset;
    rules.drivesOn = DrivesOn::Right;
    p = carPose(g, classes, rules, c);
    CHECK_APPROX(p.front.z, kOffset, kEps);
    CHECK_APPROX(p.rear.z, kOffset, kEps);
    CHECK_APPROX(p.front.x, 30, kEps);
    rules.drivesOn = DrivesOn::Left;
    p = carPose(g, classes, rules, c);
    CHECK_APPROX(p.front.z, -kOffset, kEps);
    c.offset = -kOffset;
    p = carPose(g, classes, rules, c);
    CHECK_APPROX(p.front.z, kOffset, kEps);
    rules.drivesOn = DrivesOn::Right;
    p = carPose(g, classes, rules, c);
    CHECK_APPROX(p.front.z, -kOffset, kEps);
    // Driving the other way down a path laid toward -x, the kerb turns with the car.
    const engine::LaneGraph back = onePath({engine::Vec2(100, 0), engine::Vec2(0, 0)});
    c = record(1, kCar, 0, 30);
    c.offset = kOffset;
    p = carPose(back, classes, rules, c);
    CHECK_APPROX(p.front.x, 70, kEps);
    CHECK_APPROX(p.front.z, -kOffset, kEps);
    CHECK(kerbwardPerpSign(DrivesOn::Right) == -kerbwardPerpSign(DrivesOn::Left));
}

TEST_CASE(driving_car_pose_through_a_bend) {
    // A quarter circle of radius 20 in 16 pieces, from (0, 0) heading +x round to (20, 20) heading +z.
    constexpr Real kRadius = 20;
    constexpr int kPieces = 16;
    std::vector<engine::Vec2> arc;
    for (int i = 0; i <= kPieces; ++i) {
        const Real phi = (kPi / 2) * i / kPieces;
        arc.push_back(engine::Vec2(kRadius * std::sin(phi), kRadius - kRadius * std::cos(phi)));
    }
    const engine::LaneGraph g = onePath(arc);
    const VehicleClasses classes = testClasses();
    const RuleSet rules;
    const Real length = g.paths[0].length;

    constexpr Real kStep = 0.05;
    constexpr Real kMostTurnPerStep = 3 * kPi / 180;   // "a few degrees" between two close distances
    Real worstTurn = 0;
    engine::Vec3 lastTravel;
    bool first = true;
    for (Real s = kCarLength; s <= length; s += kStep) {
        const CarPose p = carPose(g, classes, rules, record(1, kShort, 0, s));
        // On the curve.
        CHECK(engine::distToPolyline(engine::Vec2(p.front.x, p.front.z), arc) < kEps);
        CHECK_APPROX(p.travel.length(), 1, kEps);
        if (!first) worstTurn = std::max(worstTurn, angleBetween(lastTravel, p.travel));
        lastTravel = p.travel;
        first = false;
    }
    CHECK(worstTurn < kMostTurnPerStep);
    // Half way round, it heads along the circle's tangent there (45 degrees), to well under a degree.
    const CarPose mid = carPose(g, classes, rules, record(1, kCar, 0, length / 2));
    CHECK(angleBetween(mid.travel, engine::normalize(engine::Vec3(1, 0, 1))) < kPi / 180);
    // Its body, rear to front, is the chord of the arc it covers: turned less than its front.
    CHECK(angleBetween(mid.body, engine::Vec3(1, 0, 0)) < angleBetween(mid.travel, engine::Vec3(1, 0, 0)));

    // A sharp corner (two straight pieces at 90 degrees): the direction still turns, never jumps.
    const engine::LaneGraph corner = onePath({engine::Vec2(0, 0), engine::Vec2(10, 0), engine::Vec2(10, 10)});
    constexpr Real kFineStep = 0.01;
    worstTurn = 0;
    first = true;
    for (Real s = 9; s <= 11; s += kFineStep) {
        const CarPose p = carPose(corner, classes, rules, record(1, kShort, 0, s));
        if (!first) worstTurn = std::max(worstTurn, angleBetween(lastTravel, p.travel));
        lastTravel = p.travel;
        first = false;
    }
    CHECK(worstTurn < kMostTurnPerStep);
    CHECK(angleBetween(lastTravel, engine::Vec3(0, 0, 1)) < kEps);   // all the way round by 11 m
}

TEST_CASE(driving_car_pose_on_a_slope) {
    // 100 m climbing 10 m.
    const engine::LaneGraph g = onePath({engine::Vec2(0, 0), engine::Vec2(100, 0)}, {0, 10});
    const VehicleClasses classes = testClasses();
    const RuleSet rules;
    const CarPose p = carPose(g, classes, rules, record(1, kCar, 0, 50));
    CHECK_APPROX(p.front.y, 5, kEps);
    CHECK_APPROX(p.rear.y, (50 - kCarLength) / 10, kEps);
    CHECK_APPROX(p.travel.y, 0.1 / std::sqrt(1.01), kEps);
    CHECK_APPROX(p.travel.x, 1 / std::sqrt(1.01), kEps);
    CHECK_APPROX(g.gradeAt(pid(0), 50), 0.1, kEps);
    CHECK_APPROX(g.heightAt(pid(0), 25), 2.5, kEps);
}

// ---- 3. a long vehicle over three short paths -------------------------------------------------------------------

TEST_CASE(driving_long_vehicle_listed_on_three_paths) {
    // A feeder lane Z of 20 m, then A 3 m, B 4 m, C 6 m.
    const engine::LaneGraph g = chain({20, 3, 4, 6});
    const uint32_t Z = 0, A = 1, B = 2, C = 3;
    Driving d(&g);
    d.setVehicleClasses(testClasses());
    std::string why;
    // 12 m long, its front at the end of C: C 0..6, B 0..4, A 1..3.
    CHECK(d.placeCar(CarId(9), kLong, at(C, 6), {pid(B), pid(A)}, &why));
    const PathLists& lists = d.pathLists();
    const PathEntry* onC = entryOf(lists.on(pid(C)), 9);
    const PathEntry* onB = entryOf(lists.on(pid(B)), 9);
    const PathEntry* onA = entryOf(lists.on(pid(A)), 9);
    CHECK(onC && onB && onA);
    if (!onC || !onB || !onA) return;
    CHECK(onC->why == OnPath::Front);
    CHECK_APPROX(onC->front, 6, kEps);
    CHECK_APPROX(onC->rear, 0, kEps);
    CHECK(onB->why == OnPath::Tail);
    CHECK_APPROX(onB->front, 4, kEps);
    CHECK_APPROX(onB->rear, 0, kEps);
    CHECK(onA->why == OnPath::Tail);
    CHECK_APPROX(onA->front, 3, kEps);
    CHECK_APPROX(onA->rear, 1, kEps);
    CHECK(lists.on(pid(Z)).empty());

    // A follower on A, its front 0.25 m in (its own tail back on Z): the long vehicle is ahead, 0.75 m to its rear.
    CHECK(d.placeCar(CarId(1), kCar, at(A, 0.25), {pid(Z)}, &why));
    const std::vector<engine::PathId> fromA = {pid(B), pid(C)};
    Neighbour n = carAhead(g, d.cars(), d.pathLists(), CarId(1), fromA, 50);
    CHECK(n.car == CarId(9));
    CHECK_APPROX(n.gap, 0.75, kEps);
    // A car on Z behind it finds the follower by the follower's tail on Z ...
    CHECK(d.placeCar(CarId(2), kCar, at(Z, 15), {}, &why));
    const std::vector<engine::PathId> fromZ = {pid(A), pid(B), pid(C)};
    n = carAhead(g, d.cars(), d.pathLists(), CarId(2), fromZ, 50);
    CHECK(n.car == CarId(1));
    CHECK_APPROX(n.gap, 20 - 4.25 - 15, kEps);
    // ... and, with the follower gone, the long vehicle by its tail on A, a path before the one its front is on.
    CHECK(d.removeCar(CarId(1), &why));
    n = carAhead(g, d.cars(), d.pathLists(), CarId(2), fromZ, 50);
    CHECK(n.car == CarId(9));
    CHECK_APPROX(n.gap, 5 + 1, kEps);
}

// ---- 4. a lane change -------------------------------------------------------------------------------------------

TEST_CASE(driving_lane_change_listed_in_both_lanes) {
    const engine::LaneGraph g = twoLanes();
    const VehicleClasses classes = testClasses();
    std::vector<Car> cars;
    Car changer = record(5, kCar, 0, 50, 12);
    changer.besidePath = pid(1);
    changer.offset = -2;   // half way toward lane 1
    cars.push_back(changer);
    cars.push_back(record(1, kCar, 0, 30, 10));
    cars.push_back(record(2, kCar, 1, 30, 11));
    PathLists lists;
    rebuildPathLists(g, classes, cars, lists);

    const PathEntry* in0 = entryOf(lists.on(pid(0)), 5);
    const PathEntry* in1 = entryOf(lists.on(pid(1)), 5);
    CHECK(in0 && in1);
    if (!in0 || !in1) return;
    CHECK(in0->why == OnPath::Front);
    CHECK(in1->why == OnPath::Beside);
    CHECK_APPROX(in1->front, 50, kEps);
    CHECK_APPROX(in1->rear, 50 - kCarLength, kEps);
    CHECK(lists.on(pid(1)).size() == 2u);
    CHECK(lists.on(pid(1))[0].car == CarId(2));   // in order along

    for (uint32_t follower : {1u, 2u}) {
        const Neighbour n = carAhead(g, cars, lists, CarId(follower), {}, 50);
        CHECK(n.car == CarId(5));
        CHECK_APPROX(n.gap, 50 - kCarLength - 30, kEps);
        CHECK_APPROX(n.speed, 12, kEps);
    }
    // The changer itself sees nobody ahead (never itself).
    CHECK(!carAhead(g, cars, lists, CarId(5), {}, 50).found());
}

// ---- 5. the car ahead over many tiny paths, and the look-ahead limit --------------------------------------------

TEST_CASE(driving_car_ahead_across_tiny_paths) {
    // A 20 m feeder, ten paths of 0.5 m, a 20 m lane after.
    std::vector<Real> lengths = {20};
    for (int i = 0; i < 10; ++i) lengths.push_back(0.5);
    lengths.push_back(20);
    const engine::LaneGraph g = chain(lengths);
    Driving d(&g);
    d.setVehicleClasses(testClasses());
    std::string why;
    // Me, 0.4 m long, front 0.1 m into the first tiny path (my tail on the feeder). The leader, front 0.45 m into
    // the tenth, its rear 0.05 m in.
    CHECK(d.placeCar(CarId(1), kShort, at(1, 0.1), {pid(0)}, &why));
    CHECK(d.placeCar(CarId(2), kShort, at(10, 0.45), {}, &why));
    std::vector<engine::PathId> ahead;
    for (uint32_t i = 2; i < g.paths.size(); ++i) ahead.push_back(pid(i));

    const Neighbour far = carAhead(g, d.cars(), d.pathLists(), CarId(1), ahead, 100);
    CHECK(far.car == CarId(2));
    CHECK_APPROX(far.gap, 0.4 + 8 * 0.5 + 0.05, kEps);
    // Short of it: nobody.
    const Neighbour shortOf = carAhead(g, d.cars(), d.pathLists(), CarId(1), ahead, 4.0);
    CHECK(!shortOf.found());
    CHECK(shortOf.car == CarId());
    // Exactly at the limit: found; a hair short of it: not.
    CHECK(carAhead(g, d.cars(), d.pathLists(), CarId(1), ahead, far.gap).car == CarId(2));
    CHECK(!carAhead(g, d.cars(), d.pathLists(), CarId(1), ahead, std::nextafter(far.gap, Real(0))).found());
    // Nobody on the road ahead at all; and an unplaced car asking.
    CHECK(d.removeCar(CarId(2), &why));
    CHECK(!carAhead(g, d.cars(), d.pathLists(), CarId(1), ahead, 1000).found());
    CHECK(!carAhead(g, d.cars(), d.pathLists(), CarId(7), ahead, 1000).found());
}

// ---- 6. the car behind, and who is on a stretch ------------------------------------------------------------------

TEST_CASE(driving_car_behind_and_cars_on_stretch) {
    const engine::LaneGraph g = twoLanes();
    Driving d(&g);
    d.setVehicleClasses(testClasses());
    for (uint32_t i = 1; i <= 3; ++i) CHECK(d.placeCar(CarId(i), kCar, at(0, 20.0 * i)));   // fronts at 20, 40, 60
    const std::vector<Car>& cars = d.cars();
    const PathLists& lists = d.pathLists();

    Neighbour b = carBehind(cars, lists, pid(0), 50);
    CHECK(b.car == CarId(2));
    CHECK_APPROX(b.gap, 10, kEps);
    CHECK(!carBehind(cars, lists, pid(0), 10).found());
    CHECK(carBehind(cars, lists, pid(0), 40).car == CarId(2));   // a front exactly at the point: gap 0
    b = carBehind(cars, lists, pid(0), 40, CarId(2));             // passing over one's own car
    CHECK(b.car == CarId(1));
    CHECK_APPROX(b.gap, 20, kEps);
    CHECK(!carBehind(cars, lists, pid(1), 50).found());

    // 38..56: car 2 (35.5..40) and car 3, only partly (55.5..60); not car 1 (15.5..20).
    std::vector<PathEntry> on = carsOnStretch(lists, pid(0), 38, 56);
    CHECK(on.size() == 2u);
    if (on.size() == 2u) {
        CHECK(on[0].car == CarId(2));
        CHECK(on[1].car == CarId(3));
    }
    CHECK(carsOnStretch(lists, pid(0), 0, 15).empty());
    on = carsOnStretch(lists, pid(0), 16, 17);   // a stretch inside one car's body
    CHECK(on.size() == 1u && on[0].car == CarId(1));
}

// ---- 7. storage order changes nothing ---------------------------------------------------------------------------

TEST_CASE(driving_traffic_same_whatever_the_storage_order) {
    const engine::LaneGraph g = chain({20, 3, 4, 6, 30});
    const VehicleClasses classes = testClasses();
    std::vector<Car> cars;
    Car bus = record(9, kLong, 3, 6, 7);
    bus.trail[0] = pid(2);
    bus.trail[1] = pid(1);
    bus.trailCount = 2;
    cars.push_back(bus);
    Car follower = record(4, kCar, 1, 0.25, 3);
    follower.trail[0] = pid(0);
    follower.trailCount = 1;
    cars.push_back(follower);
    cars.push_back(record(6, kCar, 0, 10, 5));
    cars.push_back(record(2, kCar, 0, 10, 6));   // level with car 6: the tie goes by car id
    cars.push_back(record(7, kShort, 4, 12, 8));
    cars.push_back(record(3, kCar, 4, 25, 9));

    struct Answers {
        PathLists lists;
        std::vector<Neighbour> ahead;
        std::vector<Neighbour> behind;
        std::vector<std::vector<PathEntry>> stretches;
    };
    const auto answer = [&](const std::vector<Car>& order) {
        Answers a;
        rebuildPathLists(g, classes, order, a.lists);
        const std::vector<engine::PathId> route = {pid(1), pid(2), pid(3), pid(4)};
        for (uint32_t id : {1u, 2u, 3u, 4u, 6u, 7u, 9u}) {
            std::vector<engine::PathId> ahead;
            const Car* c = findCar(order, a.lists, CarId(id));
            if (c)
                for (const engine::PathId p : route)
                    if (p.index > c->path.index) ahead.push_back(p);
            a.ahead.push_back(carAhead(g, order, a.lists, CarId(id), ahead, 40));
        }
        for (uint32_t p = 0; p < g.paths.size(); ++p) {
            a.behind.push_back(carBehind(order, a.lists, pid(p), g.paths[p].length));
            a.stretches.push_back(carsOnStretch(a.lists, pid(p), 0, g.paths[p].length));
        }
        return a;
    };
    const Answers base = answer(cars);
    // The tie: car 6 and car 2 are level, so car 2 lists first.
    const PathList z = base.lists.on(pid(0));
    CHECK(z.size() == 3u);   // car 2, car 6, the follower's tail
    if (z.size() == 3u) CHECK(z[0].car == CarId(2) && z[1].car == CarId(6) && z[2].car == CarId(4));

    std::vector<std::vector<Car>> orders;
    std::vector<Car> reversed = cars;
    std::reverse(reversed.begin(), reversed.end());
    orders.push_back(reversed);
    for (std::size_t k = 1; k < cars.size(); ++k) {
        std::vector<Car> rotated = cars;
        std::rotate(rotated.begin(), rotated.begin() + k, rotated.end());
        orders.push_back(rotated);
    }
    for (const std::vector<Car>& order : orders) {
        const Answers other = answer(order);
        CHECK(other.lists == base.lists);
        CHECK(other.ahead.size() == base.ahead.size());
        for (std::size_t i = 0; i < base.ahead.size() && i < other.ahead.size(); ++i) {
            CHECK(other.ahead[i].car == base.ahead[i].car);
            CHECK(other.ahead[i].gap == base.ahead[i].gap);
            CHECK(other.ahead[i].speed == base.ahead[i].speed);
        }
        for (std::size_t i = 0; i < base.behind.size(); ++i) {
            CHECK(other.behind[i].car == base.behind[i].car);
            CHECK(other.behind[i].gap == base.behind[i].gap);
            CHECK(other.stretches[i] == base.stretches[i]);
        }
    }

    // The state hash: the same cars placed in another order hash the same.
    const auto placeAll = [&](const std::vector<Car>& order) {
        Driving d(&g);
        d.setVehicleClasses(classes);
        for (const Car& c : order) {
            std::vector<engine::PathId> trail(c.trail.begin(), c.trail.begin() + c.trailCount);
            CHECK(d.placeCar(c.id, c.vehicleClass, at(c.path.index, c.along), trail));
        }
        return d.stateHash();
    };
    const uint64_t hash = placeAll(cars);
    for (const std::vector<Car>& order : orders) CHECK(placeAll(order) == hash);
    std::vector<Car> fewer(cars.begin() + 1, cars.end());
    CHECK(placeAll(fewer) != hash);
}
