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
#include "../src/apps/citysim/city_physics.h"
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
#include <map>
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
// the most a run is stepped (a test's bound: the checks say whether it arrived). 150 -> 180 s (ADR-0452): drivers now
// give way to people at crosswalks, and the mixed queue's follower came to rest 0.6 m short of its place at 150 s
constexpr Real kRunSeconds = 180.0;
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
    // THE VIEWER'S KINEMATIC BOXES (ADR-0449): CityPhysicsSystem keeps a box on every drawn car and walker, as the
    // viewer's arena does; the 3b crowd test ran without it, and its near cars never met a drawn car
    citysim::CityPhysicsSystem bridge{city, physics};
    bool withBridge = false;
    bool ok = false;
    double frameMs = 0, frameWorstMs = 0;
    double physicsMs = 0, physicsWorstMs = 0;   // PhysicsSystem's step alone
    uint64_t frames = 0;
    GamepadState pad;          // the stand-in player's pedals (the take-and-give-back case)
    bool withUpdate = false;   // VehicleSystem's per-frame update too (getting in and out)
    // FIXED STEPS IN ONE APP FRAME (RT_NEAR_STEPS_PER_FRAME; the viewer at `sim speed 16` runs 8, the clock's cap) and
    // the city stepped as the app steps it (RT_NEAR_CITY_AS_APP=1: CityRenderSystem::fixedUpdate, not step)
    int stepsPerFrame = std::getenv("RT_NEAR_STEPS_PER_FRAME") ? std::max(1, std::atoi(std::getenv("RT_NEAR_STEPS_PER_FRAME"))) : 1;
    bool cityAsApp = std::getenv("RT_NEAR_CITY_AS_APP") != nullptr;
    std::string pressNext;     // ...and an action pressed on the next frame

    FrameContext ctx() {
        return FrameOwners(world, *renderer, assets, view, clock, settings, jobs, events, debug, audio, stats, input, actions,
                           players, xr, quit, transition, &commands)
            .context(1280, 720, 1280, 720, clock.fixedStep(), 1.0, false);
    }
    explicit Rig(const std::string& level = "brain_lab", bool daysOff = true) {
        if (!LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/" + level + ".json", world, *renderer, view,
                               assets, false))
            return;
        if (!city.build(world, &assets, nullptr)) return;
        if (citysim::CityBrain* b = city.simMutable().brain(); b && daysOff) b->setDay(false);
        physics.initialize(nullptr);
        physics.createBodies(world);
        FrameContext c = ctx();
        near.onStart(c);
        // (as the arena: ADR-0449. RT_NEAR_BOX_LATE=1 leaves a switched car's box to the next sync, as before the fix:
        // the A/B that shows the crowd test catching it)
        if (!std::getenv("RT_NEAR_BOX_LATE")) near.useBoxes(&bridge);
        vehicles.onStart(c);
        ok = city.sim().drivingV2() != nullptr;
    }
    driving::Driving& d() { return *city.simMutable().drivingV2Mutable(); }
    // one fixed step, in the app's order: physics, the city (the driving's tick, with the poses written first), the near
    // tier's controller, the cars' pedals
    void frame() {
        if (withUpdate) {
            actions.beginFrame();
            actions.updateGamepad(pad);
            if (!pressNext.empty()) actions.injectPress(pressNext);   // (a key press for this frame)
            pressNext.clear();
        }
        FrameContext c = ctx();
        if (withUpdate) vehicles.update(c);
        const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        physics.fixedUpdate(c);
        const double pms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        physicsMs += pms;
        physicsWorstMs = std::max(physicsWorstMs, pms);
        if (stepsPerFrame > 1 || cityAsApp) {
            // (the app's own fixed step: on a frame of several, the pose bake only on its last two)
            c.fixedStepIndex = static_cast<int>(frames % static_cast<uint64_t>(stepsPerFrame));
            c.fixedStepCount = stepsPerFrame;
            city.fixedUpdate(c);
        } else {
            city.step(world, kStep);
        }
        if (withBridge) bridge.fixedUpdate(c);   // (the arena's order: the city, its boxes, then the near tier)
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
    for (const uint32_t id : {1000u, 1001u})
        if (rig.d().car(driving::CarId(id)) && rig.d().car(driving::CarId(id))->driven)
            std::printf("    [near] car %u did not arrive:\n%s", id, rig.d().describeCar(driving::CarId(id)).c_str());
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

// ---- Sitting 4: what is in a driver's way, whatever it is (ADR-0451) --------------------------------------------------

namespace {

constexpr Real kThingAhead = 30.0;        // m: the thing stands this far along the lane from where the car starts
constexpr Real kThingRunSeconds = 240.0;  // the most a run is stepped (a test's bound: the checks say what happened)
constexpr Real kStoodShortLeast = 1.0;    // m: stopped short, the car's front at least this far from the thing
constexpr Real kBarrierAlong = 8.0;       // m along the lane beyond the junction where the road block stands
constexpr Real kBarrierHalfAcross = 9.0;  // m: the road block reaches this far either side of the lane (a road's width)

enum class ThingKind { Player, Crate, RoadBlock };
const char* thingName(ThingKind k) {
    return k == ThingKind::Player ? "the stand-in player" : k == ThingKind::Crate ? "a crate" : "a road block";
}

// THE GAP BETWEEN A CAR'S BODY AND A THING: the car a box (middle, heading, half length and width) on the plan, the thing
// a circle; below zero they overlap
Real gapToCar(Vec2 middle, Vec2 forward, Real halfLength, Real halfWidth, Vec2 thing, Real radius) {
    const Vec2 d = thing - middle;
    const Real a = d.x * forward.x + d.y * forward.y, b = d.x * forward.y - d.y * forward.x;
    const Real dx = std::max(Real(0), std::fabs(a) - halfLength), dy = std::max(Real(0), std::fabs(b) - halfWidth);
    return std::sqrt(dx * dx + dy * dy) - radius;
}

struct ThingRun {
    bool placed = false, sawIt = false, stoppedShort = false, honked = false, swerved = false, passed = false,
         anotherWay = false, arrived = false;
    Real closest = 1e9;          // m: the least gap between the car's body and the thing (below zero: they touched)
    Real stoodShort = 0;         // m: its front from the thing when it first came to rest for it
    uint32_t chassisHits = 0;    // the car's chassis touching anything (Jolt)
    Real seconds = 0;
};

// A NEAR CAR ON THE LAB'S ROUTE meets a thing standing in its way: `kind` stands on the approach lane kThingAhead along
// (the player, a crate), or across the road beyond the junction (a road block: the car is sent past it, one junction on).
ThingRun runThing(ThingKind kind) {
    ThingRun out;
    Rig rig;
    CHECK(rig.ok);
    if (!rig.ok) return out;
    int slot = -1;
    uint32_t cls = 0;
    rig.householdBody(slot, cls);
    const std::vector<LabRoute> routes = citysim::labRoutes(rig.d(), kRunUp, kRunOn);
    CHECK(!routes.empty());
    if (routes.empty()) return out;
    const LabRoute& r = routes.front();
    const LaneGraph& roads = *rig.d().roads();
    PhysicsWorld& pw = rig.physics.physicsWorld();
    // where it stands, and where the car is sent
    driving::PathPoint to = r.to;
    PathId on = r.approach;
    Real along = r.start.along + kThingAhead;
    if (kind == ThingKind::RoadBlock) {
        on = r.exit;
        along = kBarrierAlong;
        // ...sent one junction past it: along the exit lane (and the lane it runs on into) to the next junction, and the
        // lane out of its first turn
        to.path = PathId();
        PathId p = r.exit;
        for (int step = 0; step < 16 && !to.path.valid(); ++step) {
            const Path& P = roads.paths[p.index];
            if (P.next.count == 0) break;
            const PathId q = roads.nextPaths[P.next.first];
            if (roads.paths[q.index].kind == PathKind::Connector) {
                if (roads.paths[q.index].next.count > 0) to.path = roads.nextPaths[roads.paths[q.index].next.first];
            } else {
                p = q;
            }
        }
        CHECK(to.path.valid());
        if (!to.path.valid()) return out;
        to.along = std::min(Real(15), roads.paths[to.path.index].length * Real(0.5));
    }
    Vec2 at, dir;
    pointOn(roads, on, along, at, dir);
    const Real ground = roads.heightAt(on, along);
    if (std::getenv("RT_THING_LOG")) {
        const Path& A = roads.paths[r.approach.index];
        std::printf("    [thing] route %u -> %u -> %u; the approach leads into %u paths:", r.approach.index, r.turn.index,
                    r.exit.index, A.next.count);
        for (uint32_t k = A.next.first; k < A.next.end(); ++k) std::printf(" %u", roads.nextPaths[k].index);
        std::printf("; lane changes %d\n", rig.d().settings().laneChanges ? 1 : 0);
    }
    Real radius = 0;
    if (kind == ThingKind::Player) {
        radius = 0.3;
        pw.addCharacter(0.6, radius, Vec3(at.x, ground + 0.95, at.y));
    } else if (kind == ThingKind::Crate) {
        radius = 0.5;
        pw.addBox(Vec3(0.5, 0.5, 0.5), Vec3(at.x, ground + 0.55, at.y), Quat(), BodyMotion::Dynamic, 0.0, 0.8);
    } else {
        radius = 0.5;
        const Quat turn = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(dir.x, dir.y));
        pw.addBox(Vec3(kBarrierHalfAcross, 0.6, 0.5), Vec3(at.x, ground + 0.6, at.y), turn, BodyMotion::Kinematic);
    }
    std::string why;
    const Entity e = rig.near.labCar(rig.world, 1000, slot, cls, r.start, to, &why);
    if (!e.valid()) std::printf("    [thing] the lab car was refused: %s\n", why.c_str());
    if (!e.valid()) return out;
    CHECK(rig.near.toPhysics(rig.world, rig.assets, e, kStep, &why));
    out.placed = true;
    const driving::CarId id(1000);
    const Real halfLength = rig.d().vehicleClasses().at(cls).length * 0.5;
    const uint32_t hitsBefore = rig.near.counts().otherHits;
    bool moved = false;   // (it stands at the start: stopping short counts once it has moved off)
    for (int i = 0; i < static_cast<int>(kThingRunSeconds / kStep); ++i) {
        rig.frame();
        const driving::Car* c = rig.d().car(id);
        if (!c) break;
        if (!c->driven) {
            out.arrived = true;
            out.seconds = (i + 1) * kStep;
            break;
        }
        const driving::ThingAhead* seen = rig.d().thingAhead(id);
        if (seen && seen->found) out.sawIt = true;
        if (c->speed > 1.0) moved = true;
        const Vehicle* v = rig.world.get<Vehicle>(e);
        if (v && v->vehicleId != PhysicsWorld::INVALID_VEHICLE && kind != ThingKind::RoadBlock) {
            const Vec3 m = pw.vehiclePosition(v->vehicleId), f = pw.vehicleOrientation(v->vehicleId).rotate(Vec3(0, 0, 1));
            const Vec2 fwd = normalize(Vec2(f.x, f.z));
            out.closest = std::min(out.closest, gapToCar(Vec2(m.x, m.z), fwd, halfLength, v->config.chassisHalfExtent.x, at, radius));
            const Vec2 front = Vec2(m.x, m.z) + fwd * halfLength;
            if ((front - at).x * dir.x + (front - at).y * dir.y > radius + halfLength * 2) out.passed = true;
            if (moved && !out.stoppedShort && c->speed < driving::kStandingBelow && seen && seen->found) {
                out.stoppedShort = true;
                out.stoodShort = (at - front).x * dir.x + (at - front).y * dir.y - radius;
            }
        }
        if (kind == ThingKind::RoadBlock && moved && !out.stoppedShort && c->speed < driving::kStandingBelow && seen &&
            seen->found) {
            out.stoppedShort = true;
            out.stoodShort = seen->distance;
            out.closest = seen->distance;
        }
        if (std::getenv("RT_THING_LOG") && i % 30 == 0)
            std::printf("    [thing] t %.1f: path %u along %.1f speed %.2f | seen %d %.2f m side %.2f room %d offset %.2f | "
                        "horns %llu swerves %llu ways %llu/%llu | pass offset %.2f\n",
                        (i + 1) * kStep, c->path.index, c->along, c->speed, seen && seen->found ? 1 : 0,
                        seen ? seen->distance : 0.0, seen ? seen->side : 0.0, seen && seen->roomToPass ? 1 : 0,
                        seen ? seen->passOffset : 0.0, static_cast<unsigned long long>(rig.d().thingCounts().horns),
                        static_cast<unsigned long long>(rig.d().thingCounts().swerves),
                        static_cast<unsigned long long>(rig.d().thingCounts().anotherWays),
                        static_cast<unsigned long long>(rig.d().thingCounts().anotherWaysWanted), rig.d().passOffset(id));
    }
    out.honked = rig.d().thingCounts().horns > 0;
    out.swerved = rig.d().thingCounts().swerves > 0;
    out.anotherWay = rig.d().thingCounts().anotherWays > 0;
    out.chassisHits = rig.near.counts().otherHits - hitsBefore;
    std::printf("    [thing] %s: saw it %d; stopped short %d, %.2f m from it; honked %d (near horns played %u); went round "
                "it %d, past it %d; another way %d; the least gap %.2f m; chassis hits %u; arrived %d in %.1f s\n",
                thingName(kind), out.sawIt ? 1 : 0, out.stoppedShort ? 1 : 0, out.stoodShort, out.honked ? 1 : 0,
                rig.near.counts().hornsPlayed, out.swerved ? 1 : 0, out.passed ? 1 : 0, out.anotherWay ? 1 : 0,
                out.closest, out.chassisHits, out.arrived ? 1 : 0, out.seconds);
    return out;
}

}  // namespace

// THE PLAYER STANDS IN THE LANE (ADR-0451; Glenn: "the car probably should have stopped or swerved out of the way. It
// would be funny to also get a horn sound"): the near car sees him (a character, swept like any body), stops short
// without touching, sounds its horn, and goes round him where there is room (the lab's day is off: the road is clear).
TEST_CASE(near_tier_a_near_car_stops_honks_and_goes_round_the_player) {
    const ThingRun run = runThing(ThingKind::Player);
    CHECK(run.placed && run.sawIt);
    CHECK(run.stoppedShort && run.stoodShort >= kStoodShortLeast);
    CHECK(run.closest > 0);
    CHECK(run.honked);
    CHECK(run.swerved && run.passed);
    CHECK(run.arrived);
}

// ...THE SAME WITH A CRATE: anything with a body is seen; nothing in the driver asks what it is.
TEST_CASE(near_tier_a_near_car_stops_honks_and_goes_round_a_crate) {
    const ThingRun run = runThing(ThingKind::Crate);
    CHECK(run.placed && run.sawIt);
    CHECK(run.stoppedShort && run.stoodShort >= kStoodShortLeast);
    CHECK(run.closest > 0);
    CHECK(run.chassisHits == 0u);
    CHECK(run.honked);
    CHECK(run.swerved && run.passed);
    CHECK(run.arrived);
}

// ...AND WITH NO ROOM: a road block across the road beyond a junction. The car stops at the junction's line (not in the
// box), finds no room to go round, waits as long as its driver waits (wait_before_another_way, its own), then finds
// another way from where it stands and arrives by it.
TEST_CASE(near_tier_a_near_car_with_no_way_past_finds_another_way) {
    const ThingRun run = runThing(ThingKind::RoadBlock);
    CHECK(run.placed && run.sawIt);
    CHECK(run.stoppedShort && run.stoodShort > 0);
    CHECK(!run.swerved);
    CHECK(run.anotherWay);
    CHECK(run.chassisHits == 0u);
    CHECK(run.arrived);
}

// A LANE CAR AND A PERSON CROSSING (the middle tier): a person steps into the road ahead of a lane car and walks across
// it. The car stops short of them, and goes on once they are clear.
constexpr Real kCrossFrom = 6.0;      // m to the kerb side of the lane's line, where the person starts
constexpr Real kCrossWhen = 20.0;     // m: they step out when the car's front is this far short of where they cross
constexpr Real kWalkerSpeed = 1.4;    // m/s
TEST_CASE(near_tier_a_lane_car_stops_for_a_person_crossing) {
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
    const LaneGraph& roads = *rig.d().roads();
    Vec2 cross, dir;
    pointOn(roads, r.approach, r.start.along + kThingAhead + 10, cross, dir);
    const Vec2 kerbward(dir.y, -dir.x);
    std::string why;
    const Entity e = rig.near.labCar(rig.world, 1000, slot, cls, r.start, r.to, &why);
    CHECK(e.valid());
    if (!e.valid()) return;
    const driving::CarId id(1000);
    const Real halfLength = rig.d().vehicleClasses().at(cls).length * 0.5;
    const Real halfWidth = rig.d().vehicleClasses().at(cls).width * 0.5;
    bool walking = false, stopped = false, wentOn = false, arrived = false;
    Real walked = 0, closest = 1e9, speedAtStep = 0;
    for (int i = 0; i < static_cast<int>(kThingRunSeconds / kStep); ++i) {
        const driving::Car* c = rig.d().car(id);
        if (!c) break;
        if (!c->driven) {
            arrived = true;
            break;
        }
        const Vec2 front = frontOf(rig, e, halfLength);
        const Real toCross = (cross - front).x * dir.x + (cross - front).y * dir.y;
        if (!walking && toCross <= kCrossWhen) {
            walking = true;
            speedAtStep = c->speed;
        }
        Vec2 person = cross + kerbward * kCrossFrom;
        if (walking) {
            walked = std::min(walked + kWalkerSpeed * kStep, 2 * kCrossFrom);
            person = cross + kerbward * (kCrossFrom - walked);
        }
        driving::RoadThing th;
        th.id = 77;
        th.at = person;
        th.halfAcross = citysim::CitySim::kWalkerHalfAcross;
        th.velocity = walking && walked < 2 * kCrossFrom ? kerbward * -kWalkerSpeed : Vec2();
        rig.city.simMutable().setExternalThings({th});
        rig.frame();
        const Vec2 mid = middleOf(rig, e);
        closest = std::min(closest, gapToCar(mid, dir, halfLength, halfWidth, person, th.halfAcross));
        if (walking && c->speed < driving::kStandingBelow) stopped = true;
        if (stopped && walked >= 2 * kCrossFrom && c->speed > 1.0) wentOn = true;
    }
    std::printf("    [thing] a person crossing ahead of a lane car (it was at %.1f m/s when they stepped out): stopped %d, "
                "the least gap %.2f m, went on once they were clear %d, arrived %d; horns %llu\n",
                speedAtStep, stopped ? 1 : 0, closest, wentOn ? 1 : 0, arrived ? 1 : 0,
                static_cast<unsigned long long>(rig.d().thingCounts().horns));
    CHECK(walking && stopped);
    CHECK(closest > 0);
    CHECK(wentOn && arrived);
}

// ---- Sitting 3b: the near tier in the crowd (ADR-0448) ---------------------------------------------------------------

namespace {

constexpr Real kWalkSpeed = 1.4;      // m/s: the stand-in player walking
constexpr Real kDriveSpeed = 12.0;    // m/s: ...and then at a driving speed
constexpr int kWalkMinutes = 6;
constexpr int kDriveMinutes = 6;
constexpr Real kRouteLong = 2500.0;   // m of streets the stand-in follows (back along them when they run out)
constexpr Real kOffLane = 0.5;              // m: a near car's front this far off its lane's centre is off its lane
constexpr Real kStandingBelowSpeed = 0.1;   // m/s: ...and standing, slower than this
constexpr int kProbeStepsPerFrame = 8;      // the clock's cap (clock.h MAX_STEPS_PER_ADVANCE), what `sim speed 16` runs
// WHAT IS LEFT (ADR-0451): since drivers see what is in their way, whatever it is, no near car touches a parked car, a
// person on foot or another near car (each checked: 0). The chassis hits left are the ground: car 83 scraping the road
// under hard braking (the ground is a static body, which a driver's look never sees). A near car pushed off its line
// steers back within kSteerBack.
// STATE AFTER THE #306 RE-BAKE (ADR-0478, ADR-0479): the S7 bake put the near cars across sub-millimetre sidewalk slivers
// standing as 10 cm kerb faces in traffic_lab's j7 and j19 (hits on other things 0 -> 65, the longest off its lane 9.5 ->
// 220.7 s, car 148 resting against j19's). ADR-0479 opens the layers by 1 cm and the slivers are gone: hits on other
// things 65 -> 3 (car 179 on s260's real kerb at 0.69 m/s, and two on a walker), the longest off its lane 220.7 -> 4.3 s.
// STILL FAILING, left on purpose: hits on walkers 2, car 220 touching a walker on s15 at j11 (-259.8, 47.7) at 1.6 m/s,
// then 0.15 m/s. The pins below are not retaken until it passes (now: 14 at minute 9, in 76, out 64,
// 0x79168141d1414b43).
constexpr uint32_t kChassisHitsKnown = 8;
constexpr Real kOffLaneKnownWorst = 5;      // s: kSteerBack, the time a car pushed off its line takes to steer back


// THE STAND-IN PLAYER'S ROUTE: lane centre lines end to end from the lane nearest `from`, each the first lane on from
// the last (a busy part of the lab: its signalled junction at -270, 0 that the crowd's cars cross at the busy minute)
std::vector<Vec2> playerRoute(Rig& rig, Vec2 from) {
    const LaneGraph& roads = *rig.d().roads();
    uint32_t best = 0;
    Real bestD = 1e30;
    for (uint32_t p = 0; p < roads.paths.size(); ++p) {
        if (roads.paths[p].kind != PathKind::Lane) continue;
        const Real away = (roads.points[roads.paths[p].curve.first] - from).length();
        if (away < bestD) {
            bestD = away;
            best = p;
        }
    }
    std::vector<Vec2> pts;
    Real length = 0;
    uint32_t at = best;
    for (int k = 0; k < 400 && length < kRouteLong; ++k) {
        const Path& P = roads.paths[at];
        for (uint32_t i = P.curve.first; i < P.curve.end(); ++i) {
            if (!pts.empty()) length += (roads.points[i] - pts.back()).length();
            if (pts.empty() || (roads.points[i] - pts.back()).length() > kSamePoint) pts.push_back(roads.points[i]);
        }
        if (P.next.count == 0) break;
        at = roads.nextPaths[P.next.first].index;
    }
    return pts;
}

// `s` metres along the route, there and back again
Vec2 onRoute(const std::vector<Vec2>& pts, Real s) {
    Real total = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) total += (pts[i] - pts[i - 1]).length();
    if (!(total > 0)) return pts.empty() ? Vec2() : pts.front();
    s = std::fmod(s, 2 * total);
    if (s > total) s = 2 * total - s;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const Real seg = (pts[i] - pts[i - 1]).length();
        if (s <= seg) return pts[i - 1] + (pts[i] - pts[i - 1]) * (seg > 0 ? s / seg : 0);
        s -= seg;
    }
    return pts.back();
}

Entity standInPlayer(Rig& rig, Vec2 at) {
    // (the level's own player, standing at its spawn, is not the one followed: the stand-in is the only player)
    std::vector<Entity> players;
    rig.world.each<ControlledBy>([&](Entity e, ControlledBy&) { players.push_back(e); });
    for (const Entity e : players) rig.world.remove<ControlledBy>(e);
    const Entity p = rig.world.create();
    Transform t;
    t.position = Vec3(at.x, 1.0, at.y);
    rig.world.add<Transform>(p, t);
    rig.world.add<ControlledBy>(p, ControlledBy{});
    return p;
}

}  // namespace

// THE KERB LANE KEEPS CLEAR OF PARKED CARS (ADR-0450, #302): on brain_crowd, along every lane path beside a kerb
// parking band, sampled every kClearanceStep, the side gap from a car on the path to a car in the nearest kerb space
// beside it, and from the car to its street's centre line (oncoming traffic beyond), each for the WIDEST car of the
// fleet on both sides. Before #302 the line ran through the band's inner part and the gap to a parked van was below 0.
constexpr Real kClearanceStep = 2.0;     // m along a path between samples
constexpr Real kBesideSpace = 4.0;       // m: a space whose middle is within this of the sample, along the road, is beside it
constexpr Real kLeastSideGap = 0.3;      // m: a car passing a parked car leaves at least this
TEST_CASE(near_tier_the_kerb_lane_keeps_clear_of_parked_cars) {
    Rig rig("brain_crowd", false);
    CHECK(rig.ok);
    if (!rig.ok) return;
    const RoadModel& model = *rig.d().model();
    const LaneGraph& roads = *rig.d().roads();
    Real widest = 0;
    for (int s = 0; s < rig.city.sim().fleetSize(); ++s) widest = std::max(widest, rig.city.sim().fleetBody(s).width);
    Real leastToParked = 1e9, leastToMiddle = 1e9, mostToParked = 0, widthDriven = 0;
    int paths = 0, samples = 0;
    for (uint32_t pi = 0; pi < roads.paths.size(); ++pi) {
        const Path& P = roads.paths[pi];
        if (P.kind != PathKind::Lane || P.street >= model.streets.size()) continue;
        const ModelSection& sec = model.streets[P.street].section;
        if (!(sec.parking > 0)) continue;
        const std::vector<KerbSpace> spaces = model.kerbSpacesOf(ModelStreetId(P.street));
        bool any = false;
        for (Real at = kClearanceStep; at < P.length - kClearanceStep; at += kClearanceStep) {
            const Vec2 q = roads.pointAt(PathId(pi), at), dir = roads.directionAt(PathId(pi), at);
            const RoadFrontage f = model.frontageAt(q);
            if (f.valid()) leastToMiddle = std::min(leastToMiddle, f.distance);
            for (const KerbSpace& sp : spaces) {
                KerbSpacePose pose;
                if (!model.kerbSpacePose(sp, pose)) continue;
                const Vec2 d = pose.at - q;
                if (std::fabs(d.x * dir.x + d.y * dir.y) > kBesideSpace) continue;
                const Real side = std::fabs(d.x * dir.y - d.y * dir.x);
                if (side > sec.parking + P.width) continue;   // (the far kerb's spaces)
                leastToParked = std::min(leastToParked, side);
                mostToParked = std::max(mostToParked, side);
                any = true;
                ++samples;
            }
        }
        if (any) {
            ++paths;
            widthDriven = P.width;
        }
    }
    const Real gapToParked = leastToParked - widest, gapToMiddle = leastToMiddle - widest * 0.5;
    std::printf("    [kerb-lane] %d lane paths beside parked spaces, %d samples; driven width %.2f m; a car's middle from a "
                "parked car's middle %.2f-%.2f m; from the centre line at least %.2f m; the widest car %.2f m: side gap to a "
                "parked car of that width %.2f m, to the centre line %.2f m (%.2f m between two passing)\n",
                paths, samples, widthDriven, leastToParked, mostToParked, leastToMiddle, widest, gapToParked, gapToMiddle,
                2 * gapToMiddle);
    CHECK(paths > 0);
    CHECK(gapToParked >= kLeastSideGap);
    CHECK(gapToMiddle >= kLeastSideGap * 0.5);
}

TEST_CASE(near_tier_in_the_crowd_follows_the_player) {
    Rig rig("brain_crowd", false);
    CHECK(rig.ok);
    if (!rig.ok) return;
    rig.withBridge = std::getenv("RT_NEAR_NO_BRIDGE") == nullptr;   // (the viewer's kinematic boxes: ADR-0449)
    // (the city stepped as the app steps it, eight fixed steps a frame as the viewer runs at `sim speed 16`, the probe's:
    // ADR-0450 found the boxes went stale on such frames. RT_NEAR_STEPS_PER_FRAME sets another number.)
    rig.cityAsApp = true;
    if (!std::getenv("RT_NEAR_STEPS_PER_FRAME")) rig.stepsPerFrame = kProbeStepsPerFrame;
    CitySimConfig::NearTier rules;
    rig.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) { rules = c.near; });
    CHECK(rules.cap == 100);   // (the level's data: brain_crowd.json citysim.near)
    // A NEAR CAR STANDING OFF ITS LANE: each one's seconds at rest with its front more than kOffLane from its lane's
    // centre, and the worst
    std::map<uint32_t, Real> offFor;
    Real worstOffFor = 0, worstOff = 0;
    uint32_t worstOffCar = 0;
    const std::vector<Vec2> route = playerRoute(rig, Vec2(-270, 0));
    CHECK(route.size() >= 2);
    const Entity player = standInPlayer(rig, route.front());
    const int ticksPerMinute = static_cast<int>(60.0 / kStep + 0.5);
    Real travelled = 0;
    int ringsSeen = 0;
    citysim::NearCounts last = rig.near.counts();
    double lastTier = 0, lastPhys = 0;
    uint64_t lastTierSteps = 0, lastFrames = 0;
    int mostNear = 0, mostNearAt = 0;
    for (int minute = 1; minute <= kWalkMinutes + kDriveMinutes; ++minute) {
        const Real speed = minute <= kWalkMinutes ? kWalkSpeed : kDriveSpeed;
        int nearMost = 0;
        for (int t = 0; t < ticksPerMinute; ++t) {
            travelled += speed * kStep;
            const Vec2 at = onRoute(route, travelled);
            rig.world.get<Transform>(player)->position = Vec3(at.x, 1.0, at.y);
            rig.frame();
            nearMost = std::max(nearMost, rig.near.counts().nearNow);
            // RT_NEAR_TRACE=<car>: that near car's intent, controller output, real pose and the driving's record,
            // four times a second
            static const int traced = std::getenv("RT_NEAR_TRACE") ? std::atoi(std::getenv("RT_NEAR_TRACE")) : -1;
            // RT_NEAR_TRACE_CAR=<car>: what the brain says of another car, every five seconds
            static const int other = std::getenv("RT_NEAR_TRACE_CAR") ? std::atoi(std::getenv("RT_NEAR_TRACE_CAR")) : -1;
            if (other >= 0 && t % 300 == 0)
                if (citysim::CityBrain* b = rig.city.simMutable().brain())
                    std::printf("    [near-trace] %d:%05.2f car %d: %s\n", minute, t * kStep, other, b->describeCar(other).c_str());
            if (traced >= 0 && t % 15 == 0)
                rig.world.each<citysim::CityNearCar, Vehicle, AgentDriver>(
                    [&](Entity, citysim::CityNearCar& n, Vehicle& v, AgentDriver& ad) {
                        if (static_cast<int>(n.car.index) != traced) return;
                        const driving::Car* car = rig.d().car(n.car);
                        const driving::CarIntent* want = rig.d().intent(n.car);
                        const PhysicsWorld& pw = rig.physics.physicsWorld();
                        const Vec3 p = pw.vehiclePosition(v.vehicleId), f = pw.vehicleOrientation(v.vehicleId).rotate(Vec3(0, 0, 1));
                        std::printf("    [near-trace] %d:%05.2f car %d at %.2f %.2f heading %.2f %.2f %.2f m/s | record path %u along "
                                    "%.2f offset %.2f | want %.2f m/s, rest %d %.2f | command %.2f ff %.2f | steer %.2f throttle %.2f "
                                    "brake %.2f\n",
                                    minute, t * kStep, traced, p.x, p.z, f.x, f.z, pw.vehicleVelocity(v.vehicleId).length(),
                                    car ? car->path.index : 0u, car ? car->along : 0.0, car ? car->offset : 0.0,
                                    want ? want->speed : -1.0, want && want->hasRest ? 1 : 0, want ? want->restIn : 0.0,
                                    ad.command.desiredSpeed, ad.command.feedForward, v.steerApplied, v.throttle, v.brake);
                    });
            rig.world.each<citysim::CityNearCar>([&](Entity, citysim::CityNearCar& n) {
                const driving::Car* car = rig.d().car(n.car);
                const Real offLine = car ? car->offset : 0;
                if (!car || !car->driven || car->speed > kStandingBelowSpeed || std::fabs(offLine) < kOffLane) {
                    offFor.erase(n.car.index);
                    return;
                }
                const Real s = (offFor[n.car.index] += kStep);
                if (s > worstOffFor) {
                    worstOffFor = s;
                    worstOff = offLine;
                    worstOffCar = n.car.index;
                }
            });
        }
        const citysim::NearCounts& c = rig.near.counts();
        const driving::WaitReport wr = rig.d().waitReport();
        ringsSeen += static_cast<int>(wr.loops.size());
        if (nearMost > mostNear) {
            mostNear = nearMost;
            mostNearAt = minute;
        }
        std::string refusals;
        for (const std::pair<const std::string, uint32_t>& r : c.refusedCarsBy)
            refusals += " " + r.first + " " + std::to_string(r.second) + " cars (" + std::to_string(c.refusedBy.at(r.first)) + " tries);";
        int lanesDriven = 0;
        rig.world.each<citysim::CityCarOnLane>([&](Entity, citysim::CityCarOnLane& l) { lanesDriven += l.driven ? 1 : 0; });
        std::printf("    [near-crowd] %2d min (%s): near %d now, %d at most | %d lane cars driven | in %u, out %u | the most "
                    "in one step %d | over the cap at most %d | contacts %u | %zu rings | the tier %.3f ms a step, physics "
                    "%.3f ms a step | looking ahead %.4f ms a step (%.4f a look) | refused:%s\n",
                    minute, minute <= kWalkMinutes ? "walking" : "driving", c.nearNow, nearMost, lanesDriven,
                    c.intoPhysics - last.intoPhysics, c.intoLane - last.intoLane, c.mostThisStep, c.mostOverCap,
                    c.contacts, wr.loops.size(), (c.tierMs - lastTier) / static_cast<double>(std::max<uint64_t>(1, c.tierSteps - lastTierSteps)),
                    (rig.physicsMs - lastPhys) / static_cast<double>(std::max<uint64_t>(1, rig.frames - lastFrames)),
                    (c.sweepMs - last.sweepMs) / static_cast<double>(std::max<uint64_t>(1, rig.frames - lastFrames)),
                    (c.sweepMs - last.sweepMs) / static_cast<double>(std::max<uint64_t>(1, c.sweeps - last.sweeps)), refusals.c_str());
        last = c;
        lastTier = c.tierMs;
        lastTierSteps = c.tierSteps;
        lastPhys = rig.physicsMs;
        lastFrames = rig.frames;
    }
    const citysim::NearCounts& c = rig.near.counts();
    for (const citysim::NearContact& k : c.contactList)
        std::printf("    [near-crowd] contact: car %u and car %u at %.1f %.1f %.1f, closing at %.2f m/s\n", k.a, k.b, k.at.x,
                    k.at.y, k.at.z, k.speed);
    for (const citysim::NearContact& k : c.otherHitList)
        std::printf("    [near-crowd] a near car's chassis hit something else: car %u at %.1f %.1f %.1f, closing at %.2f m/s: "
                    "%s %lld, at %.2f %.2f %.2f, half %.2f x %.2f, entity %s%s\n",
                    k.a, k.at.x, k.at.y, k.at.z, k.speed, k.carBox ? "a drawn car's box, key" : "not a car", k.boxKey,
                    k.boxAt.x, k.boxAt.y, k.boxAt.z, k.boxHalf.x, k.boxHalf.z, rig.world.alive(k.otherEntity) ? "yes" : "no",
                    rig.world.get<Vehicle>(k.otherEntity) ? " (a vehicle)" : (rig.world.get<ControlledBy>(k.otherEntity) ? " (the player)" : ""));
    // (where each hit was across its road: from the centre line, and where the carriageway ends)
    for (const citysim::NearContact& k : c.otherHitList) {
        const RoadFrontage f = rig.d().model()->frontageAt(Vec2(k.at.x, k.at.z));
        if (f.valid())
            std::printf("    [near-crowd]   car %u's hit: %.2f m from %s's centre line (%s), its carriageway %.2f m either side\n", k.a,
                        f.distance, f.street.valid() ? rig.d().model()->streets[f.street.index].id.c_str() : "a junction road", f.junction.valid() ? "in a junction" : "on the street",
                        f.section->carriagewayHalfWidth());
    }
    std::printf("    [near-crowd] chassis hits: on parked cars %u, on driven cars' boxes %u, on walkers %u\n", c.parkedHits,
                c.drivenBoxHits, c.pedestrianHits);
    const driving::Driving::ThingCounts& tc = rig.d().thingCounts();
    std::printf("    [near-crowd] what was in the way (ADR-0451): decisions with something in the way %llu; horns %llu (near "
                "cars' played %u); swerves %llu; other ways found %llu of %llu wanted; looks ahead %llu, box sweeps %llu, "
                "%.4f ms a look, %.3f ms a step in all\n",
                static_cast<unsigned long long>(tc.seen), static_cast<unsigned long long>(tc.horns), c.hornsPlayed,
                static_cast<unsigned long long>(tc.swerves), static_cast<unsigned long long>(tc.anotherWays),
                static_cast<unsigned long long>(tc.anotherWaysWanted), static_cast<unsigned long long>(c.sweeps),
                static_cast<unsigned long long>(c.sweepCasts), c.sweeps ? c.sweepMs / static_cast<double>(c.sweeps) : 0.0,
                rig.frames ? c.sweepMs / static_cast<double>(rig.frames) : 0.0);
    std::printf("    [near-crowd] chassis hits on other things %u; the longest a near car stood off its lane %.1f s (car %u, "
                "%.2f m off)\n",
                c.otherHits, worstOffFor, worstOffCar, worstOff);
    const uint64_t hash = rig.d().stateHash();
    for (const std::pair<const std::string, uint32_t>& r : c.refusedCarsBy)
        std::printf("    [near-crowd] refused, %s: %u times a car (%u tries)\n", r.first.c_str(), r.second, c.refusedBy.at(r.first));
    std::printf("    [near-crowd] at the end: the most near %d (minute %d); in %u, out %u, refused %u (%u tries); the most in one step %d; "
                "over the cap at most %d; contacts %u; taken %u, given back %u; %d slots measured at load in %.1f ms; worst "
                "physics step %.3f ms; the driving's state hash %016llx\n",
                mostNear, mostNearAt, c.intoPhysics, c.intoLane, c.refusedCars, c.refused, c.mostThisStep, c.mostOverCap, c.contacts,
                c.taken, c.givenBack, c.slotsMeasured, c.measureMs, rig.physicsWorstMs, static_cast<unsigned long long>(hash));
    CHECK(ringsSeen == 0);                       // no ring of waits, ever
    CHECK(c.contacts == 0);                      // two of the city's Jolt cars never touch
    CHECK(c.parkedHits == 0);                    // no near car touches a parked car (ADR-0450; 3c: most of 59)
    CHECK(c.pedestrianHits == 0);                // ...nor a person on foot (ADR-0451; 3d: 11)
    CHECK(c.drivenBoxHits == 0);                 // ...nor a car sliding into its kerb space (#303; 3d: 1)
    CHECK(rig.d().thingCounts().horns > 0);      // drivers sounded their horns at what stood in their way
    CHECK(c.otherHits <= kChassisHitsKnown);     // (the known gaps, above)
    CHECK(worstOffFor <= kOffLaneKnownWorst);
    CHECK(c.mostThisStep <= rules.perStep);      // a few switches a step, never a burst
    CHECK(c.slotsMeasured > 0);                  // the slots measured as the level loads...
    CHECK(c.intoPhysics > 0 && c.intoLane > 0);  // ...and cars came and went
    // PINNED from the run looked at (2026-11-17): brain_crowd never fills the cap of 100 within 150 m. Re-pinned the
    // same day (ADR-0449) from 24 at minute 12, in 86, out 60, 0x7e9a72ab10d087d5: the drawn cars' kinematic boxes are
    // now in the run, as in the viewer (they were missing, so the near cars met nothing the viewer's meet), and a car
    // switched in loses its box at once
    // Re-pinned again (ADR-0450) from 34 at minute 12, in 77, out 46, 0x317e4d66f3504130: the kerb lanes are driven
    // clear of the parking band, so every car's line, and every car's day, moved; and the city is stepped as the app
    // steps it, eight steps a frame. And again (ADR-0451) from 16 at minute 11, in 82, out 66, 0x30470fc2f83cc7c7:
    // drivers stop for what is in their way. And again (ADR-0452) from 14 at minute 11, in 91, out 76,
    // 0x9699075db5940e90: people wait at crosswalks' kerb ends and cross on WALK or in a gap, and drivers give way to them.
    // And again (ADR-0454) from 0xb80206dda672d5d5, every count the same: a car is switched into physics where its body
    // is this step (CityCarPose: its middle and heading as fitted on its wheels) instead of at its plan place and heading
    // (with the old switch-in pose, and nothing else changed, the run gives 0xb80206dda672d5d5 again). And again (ADR-0454,
    // the pose moved to the sim, city_poses.h) from 0x032b5fd2c2071980, every count the same: a car's four-wheel fit
    // memory (its tilt low-pass) now lasts while the car is posed, not until the drawing's cache had gone 600 bakes
    // without it (with that sweep put back, and nothing else changed, the run gives 0x032b5fd2c2071980 again). And again
    // (ADR-0456) from 12 at minute 7, 0xdf46fdec96641d00, in and out the same: the old signal clock went, so a walker at a
    // junction the model's crosswalks do not rule judges the gap instead of waiting for that clock's WALK.
    // RE-PINNED (#306 piece 6 W3, ADR-0480) from 13 at minute 12, in 83, out 63, 0x59168988decd4c99 to 11 at minute 8,
    // in 81, out 71, 0xb182a398296f4017: the people walk the road model's walks by the one router (their own file across
    // the sidewalk, crossing where their routes cross, waiting at the crosswalk's kerb end), so every walker's line, and
    // every driver's stops for them, moved. STILL FAILING, not loosened: one touch on a walker (and one on another
    // thing) -- car 220 stands 1.47 m off its lane on s20, 9.63 m from the centre line, so the car is off its
    // carriageway when it meets the walker on the pavement.
    // And again (ADR-0480 W3b) from 11 at minute 8, in 81, out 71, 0xb182a398296f4017 to 11 at minute 6, in 52, out 48,
    // 0x0fb18ec83c665a00: the crowd maker puts places where the sim's walks and the brain's kerb spaces serve them (see
    // test_brain_crowd), so brain_crowd's households, cars and days are other ones (cars 229 -> 209, the most on the road
    // 80 -> 57). The touches above went with that day: on walkers 1 -> 0, on other things 1 -> 0.
    // And again (#306, the street map's deletion) from 11 at minute 6, in 52, out 48, 0x0fb18ec83c665a00 to 20 at
    // minute 12, in 120, out 90, 0xed4281eb889d8e9b: the sim's own walkers rest at and go to walk points on the road
    // model's walks (city_walk_points.h), not street-map nodes, so every walker's day, and every driver's stops for
    // them, moved. STILL FAILING, not loosened, on the known-bugs list: car 10 stands 0.56 m off its lane for 14.8 s
    // (kOffLaneKnownWorst).
    CHECK(mostNear == 20);
    CHECK(mostNearAt == 12);
    CHECK(c.intoPhysics == 120u);
    CHECK(c.intoLane == 90u);
    CHECK(c.mostOverCap == 0);
    CHECK(hash == 0xed4281eb889d8e9bull);
}

// THE DRAWN CARS' BOXES FOLLOW THEIR CARS EVERY FIXED STEP (ADR-0454): eight fixed steps a frame, as the viewer runs at
// `sim speed 16`, on traffic_lab (300 cars, no near tier), with the viewer's kinematic boxes. Each step, every lane
// car's box moves by what the car moved (the bridge sets the box's target in one step and the next physics step
// carries it there, so the box's move this step is the car's move the step before). Before ADR-0454 the boxes were
// read from the drawing's bake, which ran only on a frame's last two steps: they stood still for six steps and then
// jumped six steps' worth.
TEST_CASE(near_tier_boxes_follow_their_cars_every_fixed_step) {
    Rig rig("traffic_lab", false);
    CHECK(rig.ok);
    if (!rig.ok) return;
    rig.withBridge = true;
    rig.cityAsApp = true;
    rig.stepsPerFrame = kProbeStepsPerFrame;
    constexpr Real kBoxFollows = 0.02;   // m: a box's step off its car's step (the kinematic move is exact to the step)
    constexpr Real kPutDown = 5.0;       // m: a car moved further in one step was put down again, not driven
    constexpr int kSteps = 6 * 60;       // six seconds
    struct Seen { Vec2 box, car, carBefore; int steps = 0; };
    std::map<uint32_t, Seen> seen;
    uint64_t looks = 0, off = 0;
    Real worst = 0;
    uint32_t worstCar = 0;
    int worstStep = 0;
    for (int i = 0; i < kSteps; ++i) {
        rig.frame();
        rig.world.each<citysim::CityCarBody, citysim::CityCarOnLane>(
            [&](Entity e, citysim::CityCarBody& b, citysim::CityCarOnLane&) {
                const PhysicsBodyId box = rig.bridge.entityCarBox(rig.world, e);
                if (box == INVALID_PHYSICS_BODY) {
                    seen.erase(b.id);
                    return;
                }
                const Vec3 bp = rig.physics.physicsWorld().bodyPosition(box);
                const Vec2 boxAt(bp.x, bp.z), carAt = middleOf(rig, e);
                std::map<uint32_t, Seen>::iterator s = seen.find(b.id);
                if (s == seen.end()) {
                    seen[b.id] = Seen{boxAt, carAt, carAt, 1};
                    return;
                }
                Seen& was = s->second;
                if (was.steps >= 2) {
                    const Vec2 carMove = was.car - was.carBefore;
                    if (carMove.length() < kPutDown) {
                        const Real miss = ((boxAt - was.box) - carMove).length();
                        ++looks;
                        if (miss > kBoxFollows) ++off;
                        if (miss > worst) {
                            worst = miss;
                            worstCar = b.id;
                            worstStep = i;
                        }
                    }
                }
                was.carBefore = was.car;
                was.car = carAt;
                was.box = boxAt;
                ++was.steps;
            });
    }
    std::printf("    [boxes] %llu looks at a lane car's box over %d fixed steps at %d a frame: %llu off their car's step by "
                "more than %.2f m; the worst %.3f m (car %u, step %d)\n",
                static_cast<unsigned long long>(looks), kSteps, rig.stepsPerFrame, static_cast<unsigned long long>(off),
                kBoxFollows, worst, worstCar, worstStep);
    CHECK(looks > 10000u);
    CHECK(off == 0u);
}

// THE CAP HELD (ADR-0448): the same walk and drive with the cap at 4 (the level's data changed before the first step),
// so it fills: never more near cars than the cap, but for any that could not go back (counted over the cap).
TEST_CASE(near_tier_holds_its_cap) {
    constexpr int kSmallCap = 4;
    Rig rig("brain_crowd", false);
    CHECK(rig.ok);
    if (!rig.ok) return;
    rig.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) { c.near.cap = kSmallCap; });
    const std::vector<Vec2> route = playerRoute(rig, Vec2(-270, 0));
    const Entity player = standInPlayer(rig, route.front());
    Real travelled = 0;
    int mostNear = 0;
    for (int t = 0; t < static_cast<int>(10 * 60 / kStep); ++t) {
        travelled += (t * kStep < 6 * 60 ? kWalkSpeed : kDriveSpeed) * kStep;
        const Vec2 at = onRoute(route, travelled);
        rig.world.get<Transform>(player)->position = Vec3(at.x, 1.0, at.y);
        rig.frame();
        const citysim::NearCounts& c = rig.near.counts();
        mostNear = std::max(mostNear, c.nearNow);
    }
    const citysim::NearCounts& c = rig.near.counts();
    std::printf("    [near-cap] cap %d: the most near %d, over the cap at most %d, in %u, out %u, refused %u\n", kSmallCap,
                mostNear, c.mostOverCap, c.intoPhysics, c.intoLane, c.refused);
    CHECK(mostNear >= kSmallCap);   // it fills
    // and never more: a car joins only below the cap, so the only way over it is a car that could not go back when the
    // cap was lowered under it, which does not happen here (over the cap at most, reported: 0)
    CHECK(mostNear <= kSmallCap);
    CHECK(c.mostOverCap == 0);
}

// THE PLAYER TAKES A NEAR CAR AND GIVES IT BACK (#293; ADR-0448), on brain_lab: ana drives her household's car (car
// 0); it becomes a physics car; the stand-in player gets in with the real key (VehicleSystem's enter, which only changes
// who is at the wheel), drives it a little, stops and gets out; the household has it back where it stands, and ana uses
// it again.
TEST_CASE(near_tier_the_player_takes_a_near_car_and_gives_it_back) {
    Rig rig;
    CHECK(rig.ok);
    citysim::CityBrain* brain = rig.city.simMutable().brain();
    CHECK(brain != nullptr);
    if (!rig.ok || !brain) return;
    rig.withUpdate = true;
    rig.pad.connected = true;
    rig.actions.bindAxis("drive_throttle", GamepadAxis::RightTrigger, 1.0);
    rig.actions.bindButton("drive_brake", GamepadAxis::LeftTrigger);
    const Entity car = brain->carEntity(0);
    CHECK(car.valid());
    if (!car.valid()) return;
    std::string why;
    CHECK(brain->goTo(0, "far", &why));
    // her car on the road at speed: into physics
    bool near = false;
    for (int i = 0; i < static_cast<int>(240 / kStep) && !near; ++i) {
        rig.frame();
        const citysim::CityCarOnLane* lane = rig.world.get<citysim::CityCarOnLane>(car);
        // (on a lane with 40 m of it left: the player stops it on that street, not in a junction, where a car left is
        // off the road for the brain, ADR-0443)
        const driving::Car* dc = lane ? rig.d().car(lane->car) : nullptr;
        const bool midStreet = dc && rig.d().roads()->paths[dc->path.index].kind == PathKind::Lane &&
                               rig.d().roads()->paths[dc->path.index].length - dc->along > 40.0;
        if (lane && lane->driven && lane->speed > 8.0 && midStreet)
            near = rig.near.toPhysics(rig.world, rig.assets, car, kStep, &why);
    }
    CHECK(near);
    if (!near) {
        std::printf("    [near-take] her car never became a physics car: %s\n", why.c_str());
        return;
    }
    // THE KEY: the stand-in player beside it, E
    const Entity player = standInPlayer(rig, Vec2());
    bool inIt = false;
    for (int i = 0; i < 120 && !inIt; ++i) {
        const Transform* ct = rig.world.get<Transform>(car);
        rig.world.get<Transform>(player)->position = ct->position + Vec3(0, 0.5, 0);
        if (i % 10 == 0) rig.pressNext = "enter_vehicle";
        rig.frame();
        const Vehicle* v = rig.world.get<Vehicle>(car);
        inIt = v && v->driver == player;
    }
    CHECK(inIt);
    rig.frame();   // (the near tier sees who is at the wheel)
    const citysim::CityCarLink* link = rig.world.get<citysim::CityCarLink>(car);
    std::printf("    [near-take] in it: taken %u; the household's link away %d; ana is %s; the car a Vehicle %d, near %d\n",
                rig.near.counts().taken, link && link->away ? 1 : 0, brain->stateName(0).c_str(),
                rig.world.has<Vehicle>(car) ? 1 : 0, rig.world.has<citysim::CityNearCar>(car) ? 1 : 0);
    CHECK(rig.near.counts().taken == 1u);
    CHECK(link && link->away);
    CHECK(!rig.world.has<citysim::CityNearCar>(car));
    // DRIVE IT a little, then stop
    // (straight on, the wheel let go: a short run, so it is still on the road when it stops)
    rig.pad.axes[static_cast<std::size_t>(GamepadAxis::RightTrigger)] = 0.4f;
    for (int i = 0; i < static_cast<int>(1 / kStep); ++i) rig.frame();
    rig.pad.axes[static_cast<std::size_t>(GamepadAxis::RightTrigger)] = 0;
    rig.pad.axes[static_cast<std::size_t>(GamepadAxis::LeftTrigger)] = 1.0f;
    const Vehicle* v = rig.world.get<Vehicle>(car);
    Real moved = 0;
    for (int i = 0; i < static_cast<int>(8 / kStep) && v; ++i) {
        rig.frame();
        v = rig.world.get<Vehicle>(car);
        if (v) moved = std::max(moved, rig.physics.physicsWorld().vehicleVelocity(v->vehicleId).length());
        if (v && std::getenv("RT_NEAR_LOG") && i % 30 == 0)
            std::printf("    [near-take] braking: %.2f m/s, brake %.2f, throttle %.2f\n",
                        rig.physics.physicsWorld().vehicleVelocity(v->vehicleId).length(), v->brake, v->throttle);
    }
    rig.pad.axes[static_cast<std::size_t>(GamepadAxis::LeftTrigger)] = 0;
    // OUT, and given back
    rig.pressNext = "enter_vehicle";
    rig.frame();
    for (int i = 0; i < static_cast<int>(10 / kStep) && rig.near.counts().givenBack == 0; ++i) rig.frame();
    const Transform* at = rig.world.get<Transform>(car);
    std::printf("    [near-take] drove it at up to %.2f m/s; given back %u, at %.1f %.1f; a Vehicle %d; the household's link away "
                "%d\n",
                moved, rig.near.counts().givenBack, at->position.x, at->position.z, rig.world.has<Vehicle>(car) ? 1 : 0,
                link && link->away ? 1 : 0);
    CHECK(moved > 2.0);
    CHECK(rig.near.counts().givenBack == 1u);
    CHECK(!rig.world.has<Vehicle>(car));
    CHECK(link && !link->away);
    // THE HOUSEHOLD USES IT AGAIN: ana sent on, by car
    CHECK(brain->goTo(0, "work", &why));
    bool usedAgain = false;
    for (int i = 0; i < static_cast<int>(300 / kStep) && !usedAgain; ++i) {
        rig.frame();
        const citysim::CityCarOnLane* lane = rig.world.get<citysim::CityCarOnLane>(car);
        usedAgain = lane && lane->driven;
    }
    std::printf("    [near-take] ana's plan: %s | %s; her car on a trip again %d\n", brain->planText(0).c_str(),
                brain->planReason(0).c_str(), usedAgain ? 1 : 0);
    CHECK(usedAgain);
}
