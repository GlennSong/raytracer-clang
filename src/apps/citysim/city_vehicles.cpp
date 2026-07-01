#include "city_vehicles.h"

#include "../../engine/components.h"       // Transform, Vehicle, AgentDriver, Renderable
#include "../../engine/asset_manager.h"
#include "../../engine/world.h"
#include "city_sim.h"                       // VehicleBody, vehicleFleetBody

#include <cmath>

namespace citysim {

using engine::Vec2;
using engine::Vec3;
using engine::Quat;
using engine::Real;
using engine::Entity;
using engine::World;
using engine::Transform;
using engine::PrevTransform;
using engine::Renderable;
using engine::Vehicle;
using engine::AgentDriver;
using engine::PhysicsWorld;

namespace {

// A Jolt vehicle config sized to a fleet body — a 4WD arcade car like the player's
// sedan, scaled up for a van/box-truck. Front wheels steer; all four are driven
// (so heavy bodies still pull away), rear wheels take the handbrake.
PhysicsWorld::VehicleConfig configFromBody(const VehicleBody& b) {
    PhysicsWorld::VehicleConfig c;
    c.chassisHalfExtent = Vec3(b.width * 0.5, b.height * 0.5, b.length * 0.5);
    c.mass = 900.0 + b.length * b.width * 180.0;   // ~1500 kg sedan, more for a truck
    c.maxSteerDegrees = 32.0;
    c.engineTorque = 650.0;
    c.maxRPM = 6000.0;
    c.brakeTorque = 1600.0;
    c.handBrakeTorque = 4200.0;
    c.friction = 1.0;
    c.comOffsetY = -0.4;                            // low CoM: resist tipping
    Real hw = b.width * 0.5 - 0.10;
    Real hl = b.length * 0.34;
    Real wr = std::max(Real(0.30), b.height * 0.28);
    auto wheel = [&](Real x, Real z, bool front) {
        PhysicsWorld::VehicleWheel w;
        w.position = Vec3(x, -b.height * 0.5 + wr, z);
        w.radius = wr;
        w.width = 0.24;
        w.steered = front;
        w.driven = true;
        w.handBrake = !front;
        return w;
    };
    c.wheels = { wheel(hw, hl, true), wheel(-hw, hl, true),
                 wheel(hw, -hl, false), wheel(-hw, -hl, false) };
    return c;
}

Vec2 normalized(Vec2 v, Vec2 fallback) {
    Real l = std::sqrt(v.x * v.x + v.y * v.y);
    return l > 1e-6 ? Vec2(v.x / l, v.y / l) : fallback;
}

}  // namespace

void CityVehicleSystem::spawnCars(engine::FrameContext& ctx) {
    if (spawned_ || !city_.built()) return;
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    int slots = vehicleFleetSize();
    bodyMesh_.assign(slots, engine::MeshHandle{});

    const auto& agents = sim.agents();
    for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
        const Agent& a = agents[i];
        if (a.mode != Agent::Mode::Driver || a.vehicle < 0) continue;
        int slot = a.vehicle % slots;
        const VehicleBody& body = vehicleFleetBody(a.vehicle);

        // Shared body mesh per fleet slot (same as the instanced renderer used).
        if (!bodyMesh_[slot].valid())
            bodyMesh_[slot] = ctx.assets.acquireMesh(fleetCarMesh(a.vehicle),
                                                     "cityveh:" + std::to_string(slot));

        Entity e = world.create();
        // Start at the ghost's pose, a little above ground so it drops onto its
        // wheels; physics settles it. Heading yaw matches the render convention
        // (atan2(heading.x, heading.y) about +Y).
        Vec3 pos(a.pos.x, body.height * 0.5 + 0.35, a.pos.y);
        Real yaw = std::atan2(a.heading.x, a.heading.y);
        Transform t;
        t.position = pos;
        t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
        t.scale = Vec3(1, 1, 1);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});

        Renderable r;
        r.mesh = bodyMesh_[slot];
        r.material.albedo = Vec3(1, 1, 1);   // hue baked into the mesh vertex colour
        r.material.metallic = 0.0f;
        r.material.roughness = 0.55f;
        r.material.opacity = 1.0f;
        world.add<Renderable>(e, r);

        Vehicle v;
        v.config = configFromBody(body);
        world.add<Vehicle>(e, v);

        AgentDriver ad;
        ad.agentId = i;
        ad.command.desiredHeading = a.heading;
        ad.command.desiredSpeed = 0;
        // Personality (ADR-0061): deterministic per-agent hash varies the
        // controller gains and following buffer, so no two drivers handle their
        // (identical) car identically — the fleet stops moving in lockstep.
        uint32_t h = static_cast<uint32_t>(i) * 2654435761u + 0x9e3779b9u;
        auto unit = [&h]() {
            h ^= h << 13; h ^= h >> 17; h ^= h << 5;
            return (h >> 8) * (1.0 / 16777216.0);
        };
        ad.tuning.steerGain = 1.45 + 0.35 * unit();      // wheel eagerness
        ad.tuning.turnSlowdown = 0.65 + 0.20 * unit();   // corner caution
        world.add<AgentDriver>(e, ad);

        NpcCar car;
        car.entity = e;
        car.agentId = i;
        car.bumperGap = 0.6 + 0.7 * unit();              // tailgater .. cautious
        cars_.push_back(car);
    }
    spawned_ = true;
}

namespace {
constexpr Real kStandoff = 1.5;      // park this short of the ghost's spot (m)
constexpr Real kCatchupGain = 0.5;   // station error (m) -> extra speed (m/s)
constexpr Real kCatchupMax = 4.0;    // max speed over the plan while catching up
constexpr Real kTetherLead = 12.0;   // ghost may lead its car by at most this (m)
constexpr Real kSenseRange = 22.0;   // forward traffic cone (m)
constexpr Real kSenseHalfAngle = 0.5;
constexpr Real kFlipRecover = 2.0;   // rolled this long -> right the car (s)
}  // namespace

void CityVehicleSystem::driveCars(engine::FrameContext& ctx) {
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    PhysicsWorld& pw = physics_.physicsWorld();
    Real dt = ctx.clock.fixedStep();

    // Everything on the road at its REAL pose, sensed once for all cars: every
    // physics vehicle (NPC and the player's alike, driven or parked), the on-foot
    // player, and the sim's pedestrians. Pedestrians report speed 0 so they are
    // ALWAYS an obstacle in the lane — a car yields to a walker whichever way the
    // walker faces (the planner's ped rule, applied to real poses).
    std::vector<engine::SensedBody> bodies;
    std::vector<Entity> bodyOwner;
    world.each<Transform, engine::Vehicle>([&](Entity e, Transform& t, engine::Vehicle& v) {
        engine::SensedBody b;
        b.pos = Vec2(t.position.x, t.position.z);
        Vec3 f3 = t.orientation.rotate(Vec3(0, 0, 1));
        b.heading = normalized(Vec2(f3.x, f3.z), Vec2(1, 0));
        if (v.vehicleId != PhysicsWorld::INVALID_VEHICLE) {
            Vec3 vel = pw.vehicleVelocity(v.vehicleId);
            b.speed = std::fabs(vel.x * f3.x + vel.y * f3.y + vel.z * f3.z);
        }
        b.halfLength = v.config.chassisHalfExtent.z;
        bodies.push_back(b);
        bodyOwner.push_back(e);
    });
    world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform& t, engine::CharacterController&, engine::ControlledBy&) {
            engine::SensedBody b;
            b.pos = Vec2(t.position.x, t.position.z);
            b.speed = 0;
            b.halfLength = 0.4;
            bodies.push_back(b);
            bodyOwner.push_back(e);
        });
    for (const Agent& a : sim.agents()) {
        if (a.mode != Agent::Mode::Pedestrian) continue;
        engine::SensedBody b;
        b.pos = a.pos;
        b.heading = a.heading;
        b.speed = 0;              // a walker is an obstacle regardless of direction
        b.halfLength = 0.3;
        bodies.push_back(b);
        bodyOwner.push_back(Entity{});
    }

    // Cannot mutate the sim (release) inside a loop that also reads it — collect.
    std::vector<int> toRelease;
    for (NpcCar& car : cars_) {
        if (car.released) continue;
        if (!world.alive(car.entity)) { car.released = true; continue; }
        // The player commandeered it: VehicleSystem removed the AgentDriver on
        // entry. Free the ghost so the planner stops fighting the physical car.
        AgentDriver* ad = world.get<AgentDriver>(car.entity);
        if (!ad) { car.released = true; toRelease.push_back(car.agentId); continue; }
        if (car.agentId < 0 || car.agentId >= static_cast<int>(sim.agents().size())) continue;
        const Agent& g = sim.agents()[car.agentId];
        Transform* t = world.get<Transform>(car.entity);
        engine::Vehicle* v = world.get<engine::Vehicle>(car.entity);
        if (!t || !v) continue;

        Vec2 carXZ(t->position.x, t->position.z);
        Vec3 f3 = t->orientation.rotate(Vec3(0, 0, 1));
        Vec2 carFwd = normalized(Vec2(f3.x, f3.z), g.heading);
        Real realSpeed = 0, speedMag = 0;
        if (v->vehicleId != PhysicsWorld::INVALID_VEHICLE) {
            Vec3 vel = pw.vehicleVelocity(v->vehicleId);
            realSpeed = vel.x * f3.x + vel.y * f3.y + vel.z * f3.z;
            speedMag = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        }

        // A new trip = a new route: rebuild the pursuit path from the ghost's lane.
        if (car.trip != g.trips) {
            car.follower.setPath(sim.lanePath(car.agentId));
            car.trip = g.trips;
        }

        // SPEED, layer 1 — the PLAN: proportional station control toward the
        // ghost. `aheadGap` is how far the ghost sits ahead of the car (along the
        // car's own forward): drive a little faster than the plan to close it,
        // ease off (to a stop) when at or past it. A ghost held at a red (speed 0)
        // parks the car at the line; an arrived ghost parks it at the kerb.
        Real aheadGap = (g.pos.x - carXZ.x) * carFwd.x + (g.pos.y - carXZ.y) * carFwd.y;
        Real planned = g.moving ? g.speed : 0.0;
        Real target = planned + kCatchupGain * (aheadGap - kStandoff);
        Real cap = planned + kCatchupMax;
        if (target < 0) target = 0;
        if (target > cap) target = cap;

        // SPEED, layer 2 — the EYES: cap off the nearest real body ahead in the
        // lane (another physics car, the player, a pedestrian). Plans can't keep
        // physics-driven cars apart; sensing real poses can.
        int selfIdx = -1;
        for (std::size_t k = 0; k < bodyOwner.size(); ++k)
            if (bodyOwner[k] == car.entity) { selfIdx = static_cast<int>(k); break; }
        engine::VisionCone cone;
        cone.origin = carXZ;
        cone.forward = carFwd;
        cone.range = kSenseRange;
        cone.halfAngleRad = kSenseHalfAngle;
        engine::LeaderSense lead = engine::senseLeader(cone, bodies,
                                                       /*lateralMax=*/2.2,
                                                       /*stationarySpeed=*/1.0, selfIdx);
        target = engine::followSpeed(target, lead, v->config.chassisHalfExtent.z,
                                     car.bumperGap);

        // HEADING: pure pursuit over the lane path from the car's REAL position
        // (with the built-in ease-to-a-stop at the path end). No path (parked /
        // fresh spawn): hold the ghost's heading.
        if (car.follower.valid()) {
            car.follower.update(carXZ);
            ad->command = engine::pursuitCommand(
                car.follower, carXZ, target,
                engine::pursuitLookahead(std::max(realSpeed, Real(3.0))));
        } else {
            ad->command.desiredHeading = g.heading;
            ad->command.desiredSpeed = target;
        }

        // RECOVERY: rolled past ~60 degrees for a while, or wanting speed but
        // getting none (beached, wedged) -> right/unstick the car in place.
        Real upY = t->orientation.rotate(Vec3(0, 1, 0)).y;
        car.flipTimer = (upY < 0.5) ? car.flipTimer + dt : 0.0;
        bool recover = car.stuck.update(ad->command.desiredSpeed, speedMag, dt) ||
                       car.flipTimer > kFlipRecover;
        if (recover && v->vehicleId != PhysicsWorld::INVALID_VEHICLE) {
            pw.resetVehicleUpright(v->vehicleId);
            car.flipTimer = 0;
        }

        // TETHER: feed the sim the car's real position so the ghost waits rather
        // than outrunning the physics (collision, hill, slow start).
        city_.simMutable().setAgentTether(car.agentId, carXZ, kTetherLead);
    }
    for (int id : toRelease) city_.simMutable().releaseDriver(id);
}

void CityVehicleSystem::fixedUpdate(engine::FrameContext& ctx) {
    spawnCars(ctx);
    driveCars(ctx);
}

void CityVehicleSystem::onStop(engine::FrameContext&) {
    // Just drop our tracking; the world/physics teardown reclaims the entities and
    // Jolt vehicles (VehicleSystem has no per-entity removal hook yet — the same
    // documented vehicle-destroy leak, harmless at level teardown).
    cars_.clear();
    spawned_ = false;
}

}  // namespace citysim
