#include "city_vehicles.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"       // Transform, Vehicle, Renderable, InVehicle
#include "../../engine/world.h"
#include "city_sim.h"                       // VehicleBody, vehicleFleetBody

#include <cmath>
#include <string>

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
using engine::PhysicsWorld;

namespace {
constexpr Real kCommandeerRadius = 4.0;   // same reach as VehicleSystem's enter

// A Jolt vehicle config sized to a fleet body — a 4WD arcade car like the
// player's sedan, scaled up for a van/box-truck. Front wheels steer; all four
// are driven (heavy bodies still pull away), rear wheels take the handbrake.
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
}  // namespace

void CityVehicleSystem::update(engine::FrameContext& ctx) {
    // Commandeering: the same button as VehicleSystem's enter. This system is
    // registered FIRST, so on the frame the player presses enter next to an
    // AMBIENT car, we promote it here and VehicleSystem (later this frame) finds
    // a real, unoccupied Vehicle within reach and seats the player as usual.
    if (!ctx.actions.pressed("enter_vehicle")) return;
    if (!city_.built()) return;
    World& world = ctx.world;

    // The (first) player entity, on foot.
    Entity player;
    Vec3 playerPos;
    world.each<Transform, engine::ControlledBy>(
        [&](Entity e, Transform& t, engine::ControlledBy&) {
            if (!player.valid()) { player = e; playerPos = t.position; }
        });
    if (!player.valid() || world.has<engine::InVehicle>(player)) return;
    Vec2 pXZ(playerPos.x, playerPos.z);

    // If a REAL vehicle (the player's own, or an earlier promotion) is already
    // within reach, let VehicleSystem take it — don't promote a second car.
    Real bestRealD2 = kCommandeerRadius * kCommandeerRadius;
    bool realNearby = false;
    world.each<Transform, Vehicle>([&](Entity, Transform& t, Vehicle&) {
        Real dx = t.position.x - playerPos.x, dz = t.position.z - playerPos.z;
        if (dx * dx + dz * dz <= bestRealD2) realNearby = true;
    });
    if (realNearby) return;

    // Nearest ambient (planner-owned) car within reach.
    const CitySim& sim = city_.sim();
    int bestAgent = -1;
    Real bestD2 = kCommandeerRadius * kCommandeerRadius;
    const auto& agents = sim.agents();
    for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
        const Agent& a = agents[i];
        if (a.mode != Agent::Mode::Driver || a.vehicle < 0) continue;
        if (a.released || a.playerControlled) continue;
        Real dx = a.pos.x - pXZ.x, dz = a.pos.y - pXZ.y;
        Real d2 = dx * dx + dz * dz;
        if (d2 <= bestD2) { bestD2 = d2; bestAgent = i; }
    }
    if (bestAgent < 0) return;

    // PROMOTE: release the agent (its instanced car + widget vanish this tick)
    // and spawn a real Vehicle with the same fleet body at the same pose.
    const Agent& a = agents[bestAgent];
    const VehicleBody& body = vehicleFleetBody(a.vehicle);

    Entity e = world.create();
    Transform t;
    t.position = Vec3(a.pos.x, body.height * 0.5 + 0.25, a.pos.y);
    t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0),
                                        std::atan2(a.heading.x, a.heading.y));
    t.scale = Vec3(1, 1, 1);
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});

    Renderable r;
    r.mesh = ctx.assets.acquireMesh(fleetCarMesh(a.vehicle, /*withWheels=*/false),
                                    "cityveh:promoted" + std::to_string(a.vehicle));
    r.material.albedo = Vec3(1, 1, 1);   // hue baked in the mesh vertex colour
    r.material.metallic = 0.4f;
    r.material.roughness = 0.5f;
    r.material.opacity = 1.0f;
    world.add<Renderable>(e, r);

    Vehicle v;
    v.config = configFromBody(body);
    world.add<Vehicle>(e, v);   // VehicleSystem builds the Jolt car + wheels + lamps

    city_.simMutable().releaseDriver(bestAgent);
    promoted_.push_back(PromotedCar{e, bestAgent});
}

void CityVehicleSystem::onStop(engine::FrameContext&) {
    // Tracking only; world/physics teardown reclaims the entities (the same
    // no-removal-hook note as VehicleSystem — fine at level teardown).
    promoted_.clear();
}

}  // namespace citysim
