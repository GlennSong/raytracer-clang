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
    double physicsMs = 0, physicsWorstMs = 0;   // PhysicsSystem's step alone
    uint64_t frames = 0;
    GamepadState pad;          // the stand-in player's pedals (the take-and-give-back case)
    bool withUpdate = false;   // VehicleSystem's per-frame update too (getting in and out)
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

// ---- Sitting 3b: the near tier in the crowd (ADR-0448) ---------------------------------------------------------------

namespace {

constexpr Real kWalkSpeed = 1.4;      // m/s: the stand-in player walking
constexpr Real kDriveSpeed = 12.0;    // m/s: ...and then at a driving speed
constexpr int kWalkMinutes = 6;
constexpr int kDriveMinutes = 6;
constexpr Real kRouteLong = 2500.0;   // m of streets the stand-in follows (back along them when they run out)

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

TEST_CASE(near_tier_in_the_crowd_follows_the_player) {
    Rig rig("brain_crowd", false);
    CHECK(rig.ok);
    if (!rig.ok) return;
    CitySimConfig::NearTier rules;
    rig.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) { rules = c.near; });
    CHECK(rules.cap == 100);   // (the level's data: brain_crowd.json citysim.near)
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
        }
        const citysim::NearCounts& c = rig.near.counts();
        const driving::WaitReport wr = rig.d().waitReport();
        ringsSeen += static_cast<int>(wr.loops.size());
        if (nearMost > mostNear) {
            mostNear = nearMost;
            mostNearAt = minute;
        }
        std::string refusals;
        for (const std::pair<const std::string, uint32_t>& r : c.refusedBy)
            refusals += " " + r.first + " " + std::to_string(r.second) + ";";
        int lanesDriven = 0;
        rig.world.each<citysim::CityCarOnLane>([&](Entity, citysim::CityCarOnLane& l) { lanesDriven += l.driven ? 1 : 0; });
        std::printf("    [near-crowd] %2d min (%s): near %d now, %d at most | %d lane cars driven | in %u, out %u | the most "
                    "in one step %d | over the cap at most %d | contacts %u | %zu rings | the tier %.3f ms a step, physics "
                    "%.3f ms a step | refused:%s\n",
                    minute, minute <= kWalkMinutes ? "walking" : "driving", c.nearNow, nearMost, lanesDriven,
                    c.intoPhysics - last.intoPhysics, c.intoLane - last.intoLane, c.mostThisStep, c.mostOverCap,
                    c.contacts, wr.loops.size(), (c.tierMs - lastTier) / static_cast<double>(std::max<uint64_t>(1, c.tierSteps - lastTierSteps)),
                    (rig.physicsMs - lastPhys) / static_cast<double>(std::max<uint64_t>(1, rig.frames - lastFrames)), refusals.c_str());
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
    const uint64_t hash = rig.d().stateHash();
    std::printf("    [near-crowd] at the end: the most near %d (minute %d); in %u, out %u, refused %u; the most in one step %d; "
                "over the cap at most %d; contacts %u; taken %u, given back %u; %d slots measured at load in %.1f ms; worst "
                "physics step %.3f ms; the driving's state hash %016llx\n",
                mostNear, mostNearAt, c.intoPhysics, c.intoLane, c.refused, c.mostThisStep, c.mostOverCap, c.contacts,
                c.taken, c.givenBack, c.slotsMeasured, c.measureMs, rig.physicsWorstMs, static_cast<unsigned long long>(hash));
    CHECK(ringsSeen == 0);                       // no ring of waits, ever
    CHECK(c.contacts == 0);                      // two of the city's Jolt cars never touch
    CHECK(c.mostThisStep <= rules.perStep);      // a few switches a step, never a burst
    CHECK(c.slotsMeasured > 0);                  // the slots measured as the level loads...
    CHECK(c.intoPhysics > 0 && c.intoLane > 0);  // ...and cars came and went
    // PINNED from the run looked at (2026-11-17): brain_crowd never fills the cap of 100 within 150 m
    CHECK(mostNear == 24);
    CHECK(mostNearAt == 12);
    CHECK(c.intoPhysics == 86u);
    CHECK(c.intoLane == 60u);
    CHECK(c.mostOverCap == 0);
    CHECK(hash == 0x7e9a72ab10d087d5ull);
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
