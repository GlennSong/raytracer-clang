// THE NEAR TIER'S TEST BED (ADR-0447; docs/two-drivers-meeting-design.md section 7): a physics car carrying out the new
// driving's intent, against a lane car, before any car of the crowd switches tiers.
//
// The traffic lab's roads (assets/levels/brain_lab.json, its residents' days off, so its roads are empty) through the
// real LevelLoader, CityRenderSystem, PhysicsSystem, VehicleSystem and CityNearSystem, stepped as the viewer steps
// them, 60 times a second. One route: from 60 m before a junction with traffic lights, round its tightest turn, to 40 m
// along the lane out (city_near.h labRoutes). Each car sets out the moment its light turns red, so it stops at the line,
// waits, and goes round.
//
//   RT_TEST_FILTER=near_tier ./build-viewer/physics_tests
//
// Reported (each case prints its numbers): how far off the route's centre line each car's middle strays, on the way and
// in the turn; where each stops against the stop line; how long each takes; in a queue of a lane car and a physics car
// (both orders) the closest gap; switching mid-route each way, the jump in where the car is drawn, its vertical speed
// and acceleration, and the speed kept; what a physics car costs a step.

#include "test_framework.h"

#include "../src/apps/citysim/city_brain.h"
#include "../src/apps/citysim/city_car.h"
#include "../src/apps/citysim/city_near.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/control/control_commands.h"
#include "../src/engine/frame_stats.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/physics/physics_world.h"
#include "../src/engine/system.h"
#include "../src/engine/systems/camera_system.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/systems/vehicle_system.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace engine;
using citysim::CityNearSystem;
using citysim::LabRoute;

namespace {

constexpr Real kStep = 1.0 / 60.0;
constexpr Real kRunUp = citysim::kLabRunUp;   // (the viewer's `near route|queue` set out the same way)
constexpr Real kRunOn = citysim::kLabRunOn;
constexpr Real kQueueBehind = citysim::kLabQueueBehind;
constexpr Real kRunSeconds = 150.0;    // the most a run is stepped (a test's bound: the checks say whether it arrived)
constexpr Real kSwitchSpeed = 8.0;     // m/s: a car is switched into physics once this fast
constexpr Real kSettleSeconds = 1.0;   // after a switch into physics, how long its vertical motion is watched

// THE TARGETS (design section 7)
constexpr Real kStopMatch = 0.30;      // m: a physics car stops within this of where a lane car stops
constexpr Real kLaneHold = 0.75;       // m: its middle strays at most this much further off the line than a lane car's
constexpr Real kTimeMatch = 0.15;      // share: it takes at most this much longer or shorter than a lane car
constexpr Real kJumpMost = 0.05;       // m: a switch moves the drawn car this much at most beyond its own travel
constexpr Real kDropMost = 0.25;       // m/s: the vertical speed of the chassis after a switch into physics
constexpr Real kSpeedKept = 0.5;       // m/s: the speed kept through a switch, within this

// Everything a frame needs (as tests/test_player_trace.cpp's rig, FrameOwners building the context as the app does),
// with the city, its near tier and the real VehicleSystem.
struct Rig {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader{*renderer};
    AssetManager assets{uploader};
    World world;
    RenderView view;
    SimClock clock;
    Settings settings;
    JobSystem jobs{0};
    EventBus events;
    DebugDraw debug;
    AudioEngine audio;
    FrameStats stats;
    InputState input;
    InputMap actions;
    PlayerInputs players;
    XrState xr;
    bool quit = false;
    StateTransition transition;
    ControlCommands commands;
    PhysicsSystem physics;
    CameraSystem cameras;
    VehicleSystem vehicles{physics, cameras};
    citysim::CityRenderSystem city;
    CityNearSystem near{city, physics};
    bool ok = false;
    double frameMs = 0, frameWorstMs = 0;
    uint64_t frames = 0;

    FrameContext ctx() {
        return FrameOwners(world, *renderer, assets, view, clock, settings, jobs, events, debug, audio, stats, input, actions,
                           players, xr, quit, transition, &commands)
            .context(1280, 720, 1280, 720, clock.fixedStep(), 1.0, false);
    }
    Rig() {
        if (!LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/brain_lab.json", world, *renderer, view, assets, false))
            return;
        if (!city.build(world, &assets, nullptr)) return;
        if (citysim::CityBrain* b = city.simMutable().brain()) b->setDay(false);
        physics.initialize(nullptr);
        physics.createBodies(world);
        FrameContext c = ctx();
        near.onStart(c);
        vehicles.onStart(c);
        ok = city.sim().drivingV2() != nullptr;
    }
    driving::Driving& d() { return *city.simMutable().drivingV2Mutable(); }
    // one fixed step, in the app's order: physics, the city (the driving's tick, with the poses written first), the near
    // tier's controller, the cars' pedals
    void frame() {
        FrameContext c = ctx();
        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        physics.fixedUpdate(c);
        city.step(world, kStep);
        near.fixedUpdate(c);
        vehicles.fixedUpdate(c);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        frameMs += ms;
        frameWorstMs = std::max(frameWorstMs, ms);
        ++frames;
    }
    // the fleet slot and vehicle class of the level's first household car (what the lab cars are built as)
    void householdBody(int& slot, uint32_t& cls) {
        world.each<citysim::CityCarBody>([&](Entity, citysim::CityCarBody& b) {
            if (slot < 0) {
                slot = b.slot;
                cls = b.vehicleClass;
            }
        });
    }
    // step until the route's light turns `lamp` from another
    bool waitForLamp(const LabRoute& r, driving::Lamp lamp) {
        bool other = false;
        for (int i = 0; i < static_cast<int>(120 / kStep); ++i) {
            const driving::Lamp now = d().lamp(r.turn);
            if (now != lamp) other = true;
            else if (other) return true;
            frame();
        }
        return false;
    }
};

// WHERE A CAR IS DRAWN, its middle: a physics car's chassis, a lane car's body as the driving lays it
Vec2 middleOf(Rig& rig, Entity e) {
    if (const Vehicle* v = rig.world.get<Vehicle>(e))
        if (v->vehicleId != PhysicsWorld::INVALID_VEHICLE) {
            const Vec3 p = rig.physics.physicsWorld().vehiclePosition(v->vehicleId);
            return Vec2(p.x, p.z);
        }
    const citysim::CityCarBody* b = rig.world.get<citysim::CityCarBody>(e);
    const driving::Car* c = b ? rig.d().car(driving::CarId(b->id)) : nullptr;
    if (!c) return Vec2();
    const driving::CarPose pose = driving::carPose(*rig.d().roads(), rig.d().vehicleClasses(), rig.d().ruleSet(), *c);
    const Vec3 m = (pose.front + pose.rear) * 0.5;
    return Vec2(m.x, m.z);
}

// WHERE A CAR'S FRONT REALLY IS: a physics car's chassis a half driving-class length ahead, a lane car's front bumper
// as the driving lays it
Vec2 frontOf(Rig& rig, Entity e, Real halfLength) {
    if (const Vehicle* v = rig.world.get<Vehicle>(e))
        if (v->vehicleId != PhysicsWorld::INVALID_VEHICLE) {
            const PhysicsWorld& pw = rig.physics.physicsWorld();
            const Vec3 p = pw.vehiclePosition(v->vehicleId), f = pw.vehicleOrientation(v->vehicleId).rotate(Vec3(0, 0, 1));
            return Vec2(p.x, p.z) + normalize(Vec2(f.x, f.z)) * halfLength;
        }
    const citysim::CityCarBody* b = rig.world.get<citysim::CityCarBody>(e);
    const driving::Car* c = b ? rig.d().car(driving::CarId(b->id)) : nullptr;
    if (!c) return Vec2();
    const driving::CarPose pose = driving::carPose(*rig.d().roads(), rig.d().vehicleClasses(), rig.d().ruleSet(), *c);
    return Vec2(pose.front.x, pose.front.z);
}

// A point `along` metres along a path's centre line, and the line's direction there
void pointOn(const LaneGraph& roads, PathId path, Real along, Vec2& at, Vec2& dir) {
    const Path& P = roads.paths[path.index];
    at = roads.points[P.curve.first];
    dir = Vec2(1, 0);
    for (uint32_t k = P.curve.first; k + 1 < P.curve.end(); ++k) {
        const Real a = roads.along[k], b = roads.along[k + 1];
        dir = normalize(roads.points[k + 1] - roads.points[k]);
        at = roads.points[k] + dir * std::clamp(along - a, Real(0), b - a);
        if (along <= b) return;
    }
}

// THE ROUTE'S CENTRE LINE (lane in, connector, lane out), the same for every car measured
LaneFollower routeLine(Rig& rig, const LabRoute& r) {
    std::vector<Vec2> pts;
    for (const PathId p : {r.approach, r.turn, r.exit}) {
        const Path& P = rig.d().roads()->paths[p.index];
        for (uint32_t k = P.curve.first; k < P.curve.end(); ++k)
            if (pts.empty() || (rig.d().roads()->points[k] - pts.back()).length() > kSamePoint) pts.push_back(rig.d().roads()->points[k]);
    }
    LaneFollower lf;
    lf.setPath(pts);
    return lf;
}

// ONE CAR ROUND THE ROUTE
struct RouteRun {
    bool placed = false, arrived = false;
    Real mostOff = 0, meanOff = 0, mostOffInTurn = 0;
    Real stopShort = 0;       // m: where its front came to rest short of the stop it held at the red light (- past it)
    bool stopped = false;
    Real seconds = 0;
    Real mostFrontOffset = 0;   // the driving's record: its front's offset (a physics car's, as written)
    double controlMs = 0, frameMs = 0;
};

RouteRun runRoute(bool physicsCar) {
    RouteRun out;
    Rig rig;
    CHECK(rig.ok);
    if (!rig.ok) return out;
    int slot = -1;
    uint32_t cls = 0;
    rig.householdBody(slot, cls);
    const std::vector<LabRoute> routes = citysim::labRoutes(rig.d(), kRunUp, kRunOn);
    CHECK(!routes.empty());
    if (routes.empty()) {
        int lit = 0;
        Real longest = 0;
        for (uint32_t p = 0; p < rig.d().roads()->paths.size(); ++p)
            if (rig.d().roads()->paths[p].kind == PathKind::Connector && rig.d().lamp(PathId(p)) != driving::Lamp::Dark) {
                ++lit;
                const PathId from = rig.d().roads()->paths[p].from;
                if (from.valid()) longest = std::max(longest, rig.d().roads()->paths[from.index].length);
            }
        std::printf("    [near] no route: lights run %d, %d connectors lit, the longest lane into one %.1f m\n",
                    rig.d().lightsRun() ? 1 : 0, lit, longest);
        return out;
    }
    const LabRoute& r = routes.front();
    if (!rig.waitForLamp(r, driving::Lamp::Red)) return out;
    std::string why;
    const Entity e = rig.near.labCar(rig.world, 1000, slot, cls, r.start, r.to, &why);
    if (!e.valid()) std::printf("    [near] the lab car was refused: %s\n", why.c_str());
    if (!e.valid()) return out;
    out.placed = true;
    const std::chrono::steady_clock::time_point s0 = std::chrono::steady_clock::now();
    if (physicsCar && !rig.near.toPhysics(rig.world, rig.assets, e, kStep, &why)) {
        std::printf("    [near] into physics refused: %s\n", why.c_str());
        return out;
    }
    if (physicsCar)
        std::printf("    [near] the switch into physics took %.1f ms (its slot's Jolt car measured once: rest height, "
                    "launch, brakes; the spec built from vehicles.lua)\n",
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s0).count());
    std::printf("    [near] route: %u -> connector %u (tightest bend %.1f m) -> %u, from %.1f m, %s\n", r.approach.index,
                r.turn.index, r.turnRadius, r.exit.index, r.start.along, physicsCar ? "a physics car" : "a lane car");
    LaneFollower line = routeLine(rig, r);
    double offSum = 0;
    int offCount = 0;
    const driving::CarId id(1000);
    for (int i = 0; i < static_cast<int>(kRunSeconds / kStep); ++i) {
        rig.frame();
        const driving::Car* c = rig.d().car(id);
        if (!c) {
            std::printf("    [near] the driving lost the car after %.2f s\n", (i + 1) * kStep);
            break;
        }
        if (!c->driven) {
            out.arrived = true;
            out.seconds = (i + 1) * kStep;
            break;
        }
        const Real off = line.update(middleOf(rig, e));
        if (std::getenv("RT_NEAR_LOG") && i % 60 == 0) {
            const driving::CarIntent* want = rig.d().intent(id);
            const AgentDriver* ad = rig.world.get<AgentDriver>(e);
            const Vehicle* v = rig.world.get<Vehicle>(e);
            std::printf("    [near] t %.1f: path %u along %.2f speed %.2f offset %.2f | intent %s speed %.2f rest %d %.2f | "
                        "command %.2f ff %.2f | vehicle %d | written %llu off-road %llu | middle %.2f %.2f\n",
                        (i + 1) * kStep, c->path.index, c->along, c->speed, c->offset, want ? "yes" : "no",
                        want ? want->speed : 0.0, want && want->hasRest ? 1 : 0, want ? want->restIn : 0.0,
                        ad ? ad->command.desiredSpeed : -1.0, ad ? ad->command.feedForward : 0.0,
                        v ? static_cast<int>(v->vehicleId) : -1, static_cast<unsigned long long>(rig.near.counts().written),
                        static_cast<unsigned long long>(rig.near.counts().offRoad), middleOf(rig, e).x, middleOf(rig, e).y);
        }
        if (c->speed > 0.5) {
            out.mostOff = std::max(out.mostOff, off);
            offSum += off;
            ++offCount;
            if (c->path == r.turn) out.mostOffInTurn = std::max(out.mostOffInTurn, off);
            out.mostFrontOffset = std::max(out.mostFrontOffset, std::fabs(c->offset));
        }
        // at rest at the red light: where its front really is (not where the driving's record has it) against the place
        // of the stop it holds, along the lane
        if (!out.stopped && c->speed < 0.05 && c->held.hasStop && rig.d().lamp(r.turn) != driving::Lamp::Green &&
            c->path == r.approach) {
            out.stopped = true;
            Vec2 place, dir;
            pointOn(*rig.d().roads(), c->path, c->along + static_cast<Real>(c->held.stopAt - c->odometer), place, dir);
            const Vec2 front = frontOf(rig, e, rig.d().vehicleClasses().at(c->vehicleClass).length * 0.5);
            out.stopShort = (place - front).x * dir.x + (place - front).y * dir.y;
        }
    }
    out.meanOff = offCount > 0 ? static_cast<Real>(offSum / offCount) : 0;
    out.controlMs = rig.near.counts().controlMs / static_cast<double>(std::max<uint64_t>(1, rig.near.counts().controlSteps));
    out.frameMs = rig.frameMs / static_cast<double>(std::max<uint64_t>(1, rig.frames));
    std::printf("    [near] %s: arrived %d in %.2f s; off the line most %.2f m (in the turn %.2f m), mean %.2f m; front's "
                "offset most %.2f m; stopped at the red %d, %.3f m short of the stop; %.3f ms a step for the controller, "
                "%.3f ms a whole step (worst %.3f)\n",
                physicsCar ? "physics car" : "lane car", out.arrived ? 1 : 0, out.seconds, out.mostOff, out.mostOffInTurn,
                out.meanOff, out.mostFrontOffset, out.stopped ? 1 : 0, out.stopShort, out.controlMs, out.frameMs,
                rig.frameWorstMs);
    return out;
}

}  // namespace

TEST_CASE(near_tier_a_physics_car_drives_the_route_as_a_lane_car_does) {
    const RouteRun lane = runRoute(false);
    const RouteRun phys = runRoute(true);
    CHECK(lane.placed && phys.placed);
    CHECK(lane.arrived && phys.arrived);
    CHECK(lane.stopped && phys.stopped);
    // stops where a lane car stops, to about 30 cm
    CHECK(std::fabs(phys.stopShort - lane.stopShort) <= kStopMatch);
    // holds its lane through the turn
    CHECK(phys.mostOff <= lane.mostOff + kLaneHold);
    CHECK(phys.mostOffInTurn <= lane.mostOffInTurn + kLaneHold);
    // and takes about as long
    CHECK(std::fabs(phys.seconds - lane.seconds) <= kTimeMatch * lane.seconds);
    std::printf("    [near] against the lane car: stops %.3f m from where it stops; strays %.2f m further at most (%.2f in "
                "the turn); takes %.2f s %s\n",
                phys.stopShort - lane.stopShort, phys.mostOff - lane.mostOff, phys.mostOffInTurn - lane.mostOffInTurn,
                std::fabs(phys.seconds - lane.seconds), phys.seconds >= lane.seconds ? "longer" : "less");
}

namespace {

// TWO CARS QUEUED AT THE LIGHT, one of each kind: the closest the follower's front comes to the leader's rear
Real runQueue(bool physicsBehind, bool& bothArrived) {
    bothArrived = false;
    Rig rig;
    CHECK(rig.ok);
    if (!rig.ok) return -1;
    int slot = -1;
    uint32_t cls = 0;
    rig.householdBody(slot, cls);
    const std::vector<LabRoute> routes = citysim::labRoutes(rig.d(), kRunUp, kRunOn);
    if (routes.empty()) return -1;
    const LabRoute& r = routes.front();
    if (!rig.waitForLamp(r, driving::Lamp::Red)) return -1;
    // (the leader a queue's length ahead, and sent as far beyond: each its own place to park)
    driving::PathPoint ahead = r.start, followTo = r.to;
    ahead.along += kQueueBehind;
    followTo.along -= kQueueBehind;
    std::string why;
    const Entity lead = rig.near.labCar(rig.world, 1000, slot, cls, ahead, r.to, &why);
    const Entity follow = rig.near.labCar(rig.world, 1001, slot, cls, r.start, followTo, &why);
    CHECK(lead.valid() && follow.valid());
    if (!lead.valid() || !follow.valid()) {
        std::printf("    [near] queue refused: %s\n", why.c_str());
        return -1;
    }
    CHECK(rig.near.toPhysics(rig.world, rig.assets, physicsBehind ? follow : lead, kStep, &why));
    // THE GAP AS IT REALLY IS, not as the driving's records have it: each car's middle where it is drawn (a physics
    // car's chassis), measured along the route's centre line, less half of each body (the driving's class length) --
    // so a physics car whose record lagged its body could not hide a touch. (And the driving's own gap, for the report.)
    LaneFollower leadOn = routeLine(rig, r), followOn = routeLine(rig, r);
    const Real half = rig.d().vehicleClasses().at(cls).length * 0.5;
    Real closest = 1e9, closestRecorded = 1e9;
    bool arrivedLead = false, arrivedFollow = false;
    for (int i = 0; i < static_cast<int>(kRunSeconds / kStep) && !(arrivedLead && arrivedFollow); ++i) {
        rig.frame();
        const driving::Car* f = rig.d().car(driving::CarId(1001));
        const driving::Car* l = rig.d().car(driving::CarId(1000));
        arrivedLead = !l || !l->driven;
        arrivedFollow = !f || !f->driven;
        if (!f || !l || arrivedFollow) continue;
        leadOn.update(middleOf(rig, lead));
        followOn.update(middleOf(rig, follow));
        if (leadOn.remaining() > kSamePoint && followOn.remaining() > kSamePoint)
            closest = std::min(closest, (leadOn.station() - half) - (followOn.station() + half));
        std::vector<PathId> after = rig.d().wayFromFront(driving::CarId(1001));
        if (!after.empty()) after.erase(after.begin());
        const driving::Neighbour n =
            driving::carAhead(*rig.d().roads(), rig.d().cars(), rig.d().pathLists(), driving::CarId(1001), after, 200);
        if (n.found() && n.car == driving::CarId(1000)) closestRecorded = std::min(closestRecorded, n.gap);
    }
    bothArrived = arrivedLead && arrivedFollow;
    std::printf("    [near] queue, a %s behind a %s: the closest gap %.2f m (the driving's records: %.2f m); both arrived %d\n",
                physicsBehind ? "physics car" : "lane car", physicsBehind ? "lane car" : "physics car", closest,
                closestRecorded, bothArrived ? 1 : 0);
    return closest;
}

}  // namespace

TEST_CASE(near_tier_a_mixed_queue_at_a_light_never_touches) {
    bool a = false, b = false;
    const Real physicsBehind = runQueue(true, a);
    const Real laneBehind = runQueue(false, b);
    CHECK(a && b);
    // no contact, ever: the follower's front never reaches the leader's rear
    CHECK(physicsBehind > 0);
    CHECK(laneBehind > 0);
}

TEST_CASE(near_tier_a_car_switches_into_physics_and_back_mid_route) {
    Rig rig;
    CHECK(rig.ok);
    if (!rig.ok) return;
    int slot = -1;
    uint32_t cls = 0;
    rig.householdBody(slot, cls);
    const std::vector<LabRoute> routes = citysim::labRoutes(rig.d(), kRunUp, kRunOn);
    CHECK(!routes.empty());
    if (routes.empty()) return;
    const LabRoute& r = routes.front();
    // (it sets out as its light turns green: it goes through without stopping)
    CHECK(rig.waitForLamp(r, driving::Lamp::Green));
    std::string why;
    const Entity e = rig.near.labCar(rig.world, 1000, slot, cls, r.start, r.to, &why);
    CHECK(e.valid());
    if (!e.valid()) return;
    const driving::CarId id(1000);

    // INTO PHYSICS, at speed on the lane in
    Vec2 lastDrawn = middleOf(rig, e), prevDrawn = lastDrawn;
    Real lastSpeed = 0;
    bool switched = false;
    for (int i = 0; i < static_cast<int>(30 / kStep) && !switched; ++i) {
        rig.frame();
        const driving::Car* c = rig.d().car(id);
        if (!c) break;
        prevDrawn = lastDrawn;
        lastDrawn = middleOf(rig, e);
        lastSpeed = c->speed;
        if (c->speed >= kSwitchSpeed && c->path == r.approach)
            switched = rig.near.toPhysics(rig.world, rig.assets, e, kStep, &why);
    }
    CHECK(switched);
    if (!switched) {
        std::printf("    [near] into physics refused: %s\n", why.c_str());
        return;
    }
    // the next frames: how far the drawn car moves beyond its own travel, its vertical speed and acceleration, its speed
    Real jumpIn = 0, mostVy = 0, mostAy = 0, prevVy = 0, speedAfter = 0;
    bool haveVy = false;
    for (int i = 0; i < static_cast<int>(kSettleSeconds / kStep); ++i) {
        rig.frame();
        const Vehicle* v = rig.world.get<Vehicle>(e);
        if (!v || v->vehicleId == PhysicsWorld::INVALID_VEHICLE) continue;
        const PhysicsWorld& pw = rig.physics.physicsWorld();
        const Vec3 vel = pw.vehicleVelocity(v->vehicleId);
        const Vec2 now = middleOf(rig, e);
        const Vec3 fwd = pw.vehicleOrientation(v->vehicleId).rotate(Vec3(0, 0, 1));
        const Real fwdSpeed = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z;
        if (!haveVy) {
            // the first frame it is drawn by physics, against where the lane car would have been: a step of its speed
            // on from the last frame it was drawn by the driving
            const Vec2 expected = lastDrawn + normalize(lastDrawn - prevDrawn) * (lastSpeed * kStep);
            jumpIn = (now - expected).length();
            speedAfter = fwdSpeed;
        }
        mostVy = std::max(mostVy, std::fabs(vel.y));
        if (haveVy) mostAy = std::max(mostAy, std::fabs(vel.y - prevVy) / kStep);
        prevVy = vel.y;
        haveVy = true;
    }
    std::printf("    [near] into physics at %.2f m/s: the drawn car jumped %.3f m beyond its travel; its speed after %.2f "
                "m/s; the chassis' vertical speed at most %.3f m/s, its vertical acceleration at most %.2f m/s2\n",
                lastSpeed, jumpIn, speedAfter, mostVy, mostAy);
    CHECK(jumpIn <= kJumpMost);
    CHECK(mostVy <= kDropMost);
    CHECK(std::fabs(speedAfter - lastSpeed) <= kSpeedKept);

    // BACK TO THE LANE: on the lane out, the first moment it may
    bool back = false;
    Vec2 before, prevBefore;
    Real speedBefore = 0;
    for (int i = 0; i < static_cast<int>(kRunSeconds / kStep) && !back; ++i) {
        rig.frame();
        const driving::Car* c = rig.d().car(id);
        if (!c || !c->driven) {
            why = c ? "it arrived before it could be put back on its lane" : "the driving lost it";
            break;
        }
        prevBefore = before;
        before = middleOf(rig, e);
        speedBefore = c->speed;
        if (c->path == r.exit && c->speed > 1.0) back = rig.near.toLane(rig.world, e, &why);
    }
    CHECK(back);
    if (!back) {
        std::printf("    [near] back to the lane refused: %s\n", why.c_str());
        return;
    }
    rig.frame();
    const driving::Car* c = rig.d().car(id);
    CHECK(c != nullptr);
    if (!c) return;
    const Vec2 after = middleOf(rig, e);
    const Real jumpBack = (after - (before + normalize(before - prevBefore) * (speedBefore * kStep))).length();
    std::printf("    [near] back to the lane at %.2f m/s: the drawn car jumped %.3f m beyond its travel; its speed after "
                "%.2f m/s; a Vehicle left %d\n",
                speedBefore, jumpBack, c->speed, rig.world.has<Vehicle>(e) ? 1 : 0);
    CHECK(jumpBack <= kJumpMost);
    CHECK(std::fabs(c->speed - speedBefore) <= kSpeedKept);
    CHECK(!rig.world.has<Vehicle>(e));   // VehicleSystem took the Jolt car away
    CHECK(rig.world.has<citysim::CityCarOnLane>(e));
}
