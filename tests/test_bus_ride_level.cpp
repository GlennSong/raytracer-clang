// THE PLAYER RIDES A BUS ON RIVER_TOWN (ADR-0486; the level test ADR-0459 deleted with the old buses, back on the new).
// The real level through LevelLoader on the NullRenderer, Jolt, the real PlayerSystem walking the player, the city
// stepped as the app steps it (its kinematic boxes, the near tier and the VehicleSystem in the arena's order), and the
// BusRideSystem. A script on the pad: wait for a bus to stand at a stop, walk to its doorway from the pavement and in
// when its doors are open, sit,
// stand, try the door while it drives (it stays shut), and walk out at the next stop.
//
// What it holds the ride to:
//   - ON by walking: the player walks into the doorway's mouth while the bus stands, and is aboard that bus;
//   - HELD: standing still, aboard a standing bus, the player's place in its frame does not move; driving, it never
//     jumps (no step moves it more than a few centimetres in the bus) -- the carrier is the bus's own pose;
//   - never physical: the bus carrying the player is never made a physics car by the near tier (its box would push
//     the player out);
//   - E sits you on an offered seat and E stands you up;
//   - the doors (city_driving.h busDoorsOpen, ADR-0487) are shut while it drives: walking at the doorway does not get
//     you off; on and off only while they are fully open;
//   - OFF at the next stop through the doorway, onto the pavement beside the bus, standing on the ground.
//
//   RT_BUS_RIDE_LOG=1   a line a second: the bus, its stop, where you are in it

#include "test_framework.h"

#include "../src/apps/citysim/city_bus_ride.h"
#include "../src/apps/citysim/city_car.h"
#include "../src/apps/citysim/city_driving.h"
#include "../src/apps/citysim/city_near.h"
#include "../src/apps/citysim/city_physics.h"
#include "../src/apps/citysim/city_render.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/control/control_commands.h"
#include "../src/engine/frame_stats.h"
#include "../src/engine/interact_broker.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/physics/physics_world.h"
#include "../src/engine/riding.h"
#include "../src/engine/system.h"
#include "../src/engine/systems/camera_system.h"
#include "../src/engine/systems/physics_system.h"
#include "../src/engine/systems/player_system.h"
#include "../src/engine/systems/vehicle_system.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>

using namespace engine;
using citysim::BusCabin;

namespace {

constexpr Real kStillMost = 0.01;    // m: standing still aboard a standing bus, your place in it moves this much at most
constexpr Real kStepMost = 0.05;     // m: driving, one step moves you this much at most in the bus (no jumps)
constexpr Real kLeanMost = 0.25;     // m: ...and seated, this far from where you sat at most (the capsule hangs straight
                                     //    down from the eye on a slope: engine/riding.h's riderCentre)
constexpr Real kRodeLeast = 40.0;    // m: the bus took you at least this far

// Everything a frame needs, as tests/test_near_tier.cpp's rig (FrameOwners building the context as the app does), with
// the player and the bus ride added in the arena's order.
struct Rig {
    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader{*renderer};
    AssetManager assets{uploader};
    World world;
    RenderView view;
    SimClock clock;
    Settings settings;
    JobSystem jobs{0};   // synchronous: the same steps every run
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
    PlayerSystem player{cameras.flyController(), physics};
    citysim::CityRenderSystem city;
    citysim::CityPhysicsSystem bridge{city, physics};
    citysim::BusRideSystem bus{city, physics, cameras.flyController()};
    citysim::CityNearSystem near{city, physics};
    VehicleSystem vehicles{physics, cameras};
    GamepadState pad;
    double time = 0;
    bool ok = false;

    FrameContext ctx() {
        return FrameOwners(world, *renderer, assets, view, clock, settings, jobs, events, debug, audio, stats, input, actions,
                           players, xr, quit, transition, &commands)
            .context(1280, 720, 1280, 720, clock.fixedStep(), 1.0, false);
    }
    FlyCameraController& fly() { return cameras.flyController(); }

    Rig() {
        if (!LevelLoader::load(std::string(RT_SOURCE_DIR) + "/assets/levels/river_town.json", world, *renderer, view, assets,
                               false))
            return;
        if (!city.build(world, &assets, nullptr)) return;
        pad.connected = true;
        actions.bindAxis("cam_forward", GamepadAxis::LeftY, -1.0);   // (CameraSystem's movement axes, as the trace test)
        actions.bindAxis("cam_right", GamepadAxis::LeftX, 1.0);
        physics.initialize(nullptr);
        physics.createBodies(world);
        FrameContext c = ctx();
        player.onStart(c);
        bus.onStart(c);
        near.onStart(c);
        near.useBoxes(&bridge);
        vehicles.onStart(c);
        fly().positionLocked = true;
        ok = city.sim().drivingV2() != nullptr && body().valid();
    }

    Entity body() {
        Entity out;
        world.each<Transform, CharacterController, ControlledBy>(
            [&](Entity e, Transform&, CharacterController&, ControlledBy&) { out = e; });
        return out;
    }
    Vec3 centre() { return world.get<Transform>(body())->position; }
    Real centreUp() {
        const CharacterController* cc = world.get<CharacterController>(body());
        return cc->halfHeight + cc->radius;
    }
    void place(const Vec3& feet) {
        const Entity e = body();
        const Vec3 at = feet + Vec3(0, centreUp() + 0.02, 0);
        physics.physicsWorld().setCharacterPosition(world.get<CharacterController>(e)->characterId, at);
        world.get<Transform>(e)->position = at;
        if (PrevTransform* p = world.get<PrevTransform>(e)) p->value.position = at;
    }
    // face (level) toward a world direction, and push the stick forward (or let it go)
    void walk(const Vec3& dir, bool go) {
        if (dir.x * dir.x + dir.z * dir.z > 1e-8) fly().yaw = std::atan2(dir.x, -dir.z) * Real(180) / Real(3.14159265358979);
        fly().pitch = 0;
        pad.axes[static_cast<std::size_t>(GamepadAxis::LeftY)] = go ? -1.0f : 0.0f;
        pad.axes[static_cast<std::size_t>(GamepadAxis::LeftX)] = 0.0f;
    }
    // E on the bus's offer (sit on the nearest seat offered, or stand up)
    bool pressE() {
        const Entity e = body();
        const InteractOffers* o = world.get<InteractOffers>(e);
        if (!o) return false;
        const InteractOffer* best = nullptr;
        Real bestD = 1e9;
        for (const InteractOffer& offer : o->list) {
            if (offer.provider != "bus") continue;
            const Real d = (offer.anchor - centre()).length();
            if (d < bestD) bestD = d, best = &offer;
        }
        if (!best) return false;
        InteractCommand c;
        c.provider = best->provider;
        c.entity = best->entity;
        c.key = best->key;
        world.add<InteractCommand>(e, c);
        return true;
    }

    // One frame the way the arena runs one: input; the per-frame updates; one fixed step in the arena's order (physics,
    // the player, the city, its boxes, the bus ride, the near tier, the vehicles)
    void frame() {
        actions.beginFrame();
        actions.updateGamepad(pad);
        FrameContext c = ctx();
        if (InteractOffers* o = world.get<InteractOffers>(body())) o->list.clear();   // (the broker clears them)
        player.update(c);
        bus.update(c);
        vehicles.update(c);
        c.fixedStepIndex = 0;
        c.fixedStepCount = 1;
        physics.fixedUpdate(c);
        player.fixedUpdate(c);
        city.fixedUpdate(c);
        bridge.fixedUpdate(c);
        bus.fixedUpdate(c);
        near.fixedUpdate(c);
        vehicles.fixedUpdate(c);
        time += clock.fixedStep();
    }

    // where the player's capsule centre is in the bus's frame (false: the bus is not one with a saloon this step)
    bool inBus(Entity b, Vec3& local, BusCabin* cabinOut = nullptr, Mat4* frameOut = nullptr) {
        BusCabin cabin;
        Mat4 m;
        if (!city.busCabinOf(world, b, cabin, &m)) return false;
        local = m.inverse().transformPoint(centre());
        if (cabinOut) *cabinOut = cabin;
        if (frameOut) *frameOut = m;
        return true;
    }
};

const citysim::CityBusRuns* runsOf(Rig& r, Entity b) { return r.world.get<citysim::CityBusRuns>(b); }

}  // namespace

TEST_CASE(river_town_the_player_walks_onto_a_bus_rides_it_and_walks_off_at_the_next_stop) {
    Rig r;
    CHECK(r.ok);
    if (!r.ok) return;
    const bool log = std::getenv("RT_BUS_RIDE_LOG") != nullptr;
    const int perSecond = static_cast<int>(std::lround(1.0 / r.clock.fixedStep()));

    // 1. A BUS JUST ARRIVED AT A STOP: the first bus seen to go from driving to standing (its whole dwell ahead)
    Entity b;
    std::map<uint32_t, bool> wasStanding;
    for (int i = 0; i < 180 * perSecond && !b.valid(); ++i) {
        r.frame();
        r.world.each<citysim::CityBusRuns, citysim::CityCarBody>([&](Entity e, citysim::CityBusRuns& runs, citysim::CityCarBody& body) {
            const bool before = wasStanding.count(body.id) ? wasStanding[body.id] : true;
            wasStanding[body.id] = runs.standing;
            BusCabin cabin;
            if (!b.valid() && runs.standing && !before && r.city.busCabinOf(r.world, e, cabin, nullptr) && !cabin.doors.empty())
                b = e;
        });
    }
    CHECK(b.valid());
    if (!b.valid()) return;
    const uint32_t busId = r.world.get<citysim::CityCarBody>(b)->id;
    const uint32_t firstStop = runsOf(r, b)->stop;

    // 2. ON: put down on the pavement a step out from its middle door, and walk in
    BusCabin cabin;
    Mat4 at;
    Vec3 local;
    CHECK(r.inBus(b, local, &cabin, &at));
    const int door = cabin.doors.size() > 1 ? 1 : 0;
    Vec3 outside = citysim::busOutside(cabin, door);
    outside.x += 0.8;   // (a step further out than where you land getting off)
    r.place(at.transformPoint(outside) + Vec3(0, 0.3, 0));
    for (int i = 0; i < 4 * perSecond && !r.bus.riding().valid() && runsOf(r, b)->standing; ++i) {
        Mat4 now;
        if (!r.inBus(b, local, nullptr, &now)) break;
        // (to the doorway, and wait there till the doors are open, as a person does: pushing at a shut bus climbs it)
        if (citysim::busDoorsOpen(*runsOf(r, b))) {
            r.walk(now.transformPoint(citysim::busInside(cabin, door)) - r.centre(), true);
        } else {
            Vec3 to = now.transformPoint(citysim::busOutside(cabin, door)) - r.centre();
            to.y = 0;
            r.walk(to, to.length() > 0.15);
        }
        r.frame();
        if (log && i % (perSecond / 4) == 0 && r.inBus(b, local))
            std::printf("    [bus ride] walking in t %.2f s: doors %.2f open, feet at %.3f %.3f %.3f in the bus (door %d at %.2f %.2f %.2f)\n",
                        r.time, citysim::busDoorsOpenness(*runsOf(r, b)), local.x, local.y - r.centreUp(), local.z, door,
                        cabin.doors[static_cast<std::size_t>(door)].x, cabin.doors[static_cast<std::size_t>(door)].y,
                        cabin.doors[static_cast<std::size_t>(door)].z);
    }
    std::printf("    [bus ride] bus %u at stop %u, door %d: aboard %d after %.1f s\n", busId, firstStop, door,
                r.bus.riding() == b ? 1 : 0, r.time);
    CHECK(r.bus.riding() == b);
    if (r.bus.riding() != b) return;
    CHECK(citysim::busDoorsOpen(*runsOf(r, b)));   // (on through an open door: ADR-0487)

    // 3. HELD STILL in a standing bus (the stick let go)
    r.walk(Vec3(0, 0, 0), false);
    for (int i = 0; i < 10; ++i) r.frame();   // (the step taken in settles)
    Vec3 still;
    CHECK(r.inBus(b, still));
    Real stillWorst = 0;
    for (int i = 0; i < perSecond && runsOf(r, b)->standing; ++i) {
        r.frame();
        if (r.inBus(b, local)) stillWorst = std::max(stillWorst, (local - still).length());
    }
    std::printf("    [bus ride] standing still aboard: moved %.4f m in the bus\n", stillWorst);
    CHECK(stillWorst <= kStillMost);

    // 4. SIT (E on the nearest seat offered); seated, your place in the bus is the seat's
    bool pressed = false;
    for (int i = 0; i < 5 && !pressed; ++i) {
        r.frame();
        pressed = r.pressE();
    }
    CHECK(pressed);
    for (int i = 0; i < 3; ++i) r.frame();
    CHECK(r.bus.seat() >= 0);
    const int seat = r.bus.seat();

    // 5. RIDE: seated until it has set off and gone a way, never physical, never a jump in the bus
    Vec3 seated;
    CHECK(r.inBus(b, seated));
    const Vec3 setOffAt = r.world.get<citysim::CityCarPose>(b) ? Vec3(r.world.get<citysim::CityCarPose>(b)->body.m[0][3], 0,
                                                                     r.world.get<citysim::CityCarPose>(b)->body.m[2][3])
                                                               : Vec3(0, 0, 0);
    Real rode = 0, leanWorst = 0, jumpWorst = 0;
    bool everPhysical = false, leftStop = false;
    Vec3 prevLocal = seated, prevBus = setOffAt;
    auto rideStep = [&]() {
        r.frame();
        everPhysical = everPhysical || r.world.has<Vehicle>(b);
        const citysim::CityCarPose* pose = r.world.get<citysim::CityCarPose>(b);
        if (pose) {
            const Vec3 p(pose->body.m[0][3], 0, pose->body.m[2][3]);
            rode += (p - prevBus).length();
            prevBus = p;
        }
        if (r.bus.riding() == b && r.inBus(b, local)) {
            jumpWorst = std::max(jumpWorst, (local - prevLocal).length());
            prevLocal = local;
        }
        if (!runsOf(r, b)->standing) leftStop = true;
    };
    for (int i = 0; i < 240 * perSecond && r.bus.riding() == b && (!leftStop || rode < kRodeLeast); ++i) {
        rideStep();
        if (r.bus.seat() >= 0 && r.inBus(b, local)) leanWorst = std::max(leanWorst, (local - seated).length());
        if (log && i % perSecond == 0)
            std::printf("    [bus ride] t %.0f s: stop %u standing %d, rode %.1f m, you at %.3f %.3f %.3f in it (seat %d)\n", r.time,
                        runsOf(r, b)->stop, runsOf(r, b)->standing ? 1 : 0, rode, local.x, local.y, local.z, r.bus.seat());
    }
    std::printf("    [bus ride] seated on seat %d: rode %.1f m, %.3f m off the seat at worst, worst step in the bus %.4f m, "
                "ever physical %d\n", seat, rode, leanWorst, jumpWorst, everPhysical ? 1 : 0);
    CHECK(r.bus.riding() == b);
    CHECK(leftStop && rode >= kRodeLeast);
    CHECK(leanWorst <= kLeanMost);

    // 6. STAND UP (E), and walk to the doorway and on out while it drives: the doors are shut
    const Real jumpSeated = jumpWorst;
    pressed = false;
    for (int i = 0; i < 5 && !pressed; ++i) {
        rideStep();
        pressed = r.pressE();
    }
    CHECK(pressed);
    for (int i = 0; i < 3; ++i) rideStep();
    CHECK(r.bus.seat() < 0);
    if (r.inBus(b, local)) prevLocal = local;   // (standing up moves you from the seat to the aisle: not a jump)
    jumpWorst = jumpSeated;
    bool offWhileDriving = false;
    Vec3 beyond = citysim::busOutside(cabin, door);
    beyond.x += 2.0;
    int pushedAtShutDoor = 0;
    for (int i = 0; i < 240 * perSecond && r.bus.riding() == b && !runsOf(r, b)->standing; ++i) {
        Mat4 now;
        if (r.inBus(b, local, nullptr, &now)) r.walk(now.transformPoint(beyond) - r.centre(), true);
        const bool wasAboard = r.bus.riding() == b;
        const Vec3 before = local;
        rideStep();
        if (wasAboard && r.bus.riding() != b && !citysim::busDoorsOpen(*runsOf(r, b))) offWhileDriving = true;
        ++pushedAtShutDoor;
        if (log && (i % perSecond == 0 || (wasAboard && r.bus.riding() != b) || (local - before).length() > kStepMost))
            std::printf("    [bus ride] door push t %.2f s: aboard %d, stop %u standing %d, you at %.3f %.3f %.3f in it, centre %.2f %.2f %.2f\n",
                        r.time, r.bus.riding() == b ? 1 : 0, runsOf(r, b)->stop, runsOf(r, b)->standing ? 1 : 0, local.x, local.y,
                        local.z, r.centre().x, r.centre().y, r.centre().z);
        if (log && i % (20 * perSecond) == 10 * perSecond)
            if (const citysim::CityCarOnLane* ol = r.world.get<citysim::CityCarOnLane>(b))
                if (ol->speed < 0.1)
                    std::printf("    [bus ride] the bus at rest, not at a stop; its driver card:\n%s\n",
                                r.city.sim().drivingV2()->describeCar(ol->car).c_str());
    }
    std::printf("    [bus ride] pushed at the shut door for %.1f s; off while it drove %d; at stop %u now, standing %d\n",
                pushedAtShutDoor / static_cast<double>(perSecond), offWhileDriving ? 1 : 0, runsOf(r, b)->stop,
                runsOf(r, b)->standing ? 1 : 0);
    CHECK(!offWhileDriving);   // (not off through a shut door)
    CHECK(runsOf(r, b)->standing && runsOf(r, b)->stop != firstStop);
    CHECK(jumpWorst <= kStepMost);
    CHECK(!everPhysical);

    // 7. OFF at the next stop: the same push takes you out through the open doorway
    bool offShut = false;
    for (int i = 0; i < 4 * perSecond && r.bus.riding() == b; ++i) {
        Mat4 now;
        if (r.inBus(b, local, nullptr, &now)) r.walk(now.transformPoint(beyond) - r.centre(), true);
        r.frame();
        if (r.bus.riding() != b && !citysim::busDoorsOpen(*runsOf(r, b))) offShut = true;
    }
    CHECK(!r.bus.riding().valid());
    CHECK(!offShut);   // (out through an open door)
    // (a little further out onto the pavement, then the stick let go: not taken back on)
    for (int i = 0; i < perSecond / 2; ++i) {
        Mat4 now;
        if (r.inBus(b, local, nullptr, &now)) r.walk(now.transformPoint(beyond) - r.centre(), true);
        r.frame();
    }
    r.walk(Vec3(0, 0, 0), false);
    CHECK(!r.bus.riding().valid());
    CHECK(!r.world.has<Passenger>(r.body()));
    CHECK(!r.world.has<citysim::CityCarCarriesPlayer>(b));
    // ...beside the bus on the kerb side, standing on the ground (not falling through it)
    for (int i = 0; i < perSecond; ++i) r.frame();
    const Real y0 = r.centre().y;
    for (int i = 0; i < perSecond; ++i) r.frame();
    Mat4 now;
    CHECK(r.inBus(b, local, nullptr, &now));
    const Real floorY = now.transformPoint(Vec3(0, citysim::busFloor(cabin), 0)).y;
    const Real feetY = r.centre().y - r.centreUp();
    std::printf("    [bus ride] off: %.2f m out from the bus's middle (half width %.2f); feet %.2f m below its floor; "
                "settled %.3f m in the last second\n", local.x, cabin.halfWidth, floorY - feetY, std::fabs(r.centre().y - y0));
    CHECK(local.x > cabin.halfWidth);
    CHECK(std::fabs(r.centre().y - y0) < 0.05);
    CHECK(floorY - feetY > -0.2 && floorY - feetY < 1.0);
}
