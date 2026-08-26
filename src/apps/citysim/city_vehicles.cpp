#include "city_vehicles.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"       // Transform, Vehicle, Renderable, InVehicle
#include "../../engine/physics/physics_world.h"   // VehicleConfig (was via the header)
#include "../../engine/world.h"
#include "../../log.h"
#include "city_sim.h"                       // VehicleBody

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

// SUSPENSION, and the trap in it. PhysicsWorld::VehicleWheel::position is the
// suspension ATTACHMENT point — Jolt hangs the wheel below it — so feeding it a
// wheel's drawn RESTING centre parks the whole chassis one suspension-drop too
// high, on wheels that dangle below the arches. With the frequency-based spring
// the resting drop is clamp(max - kStaticDeflection, min, max) and is
// MASS-INDEPENDENT: a 1100 kg hatchback and a 2600 kg truck settle identically
// (measured across travel and mass in tests/test_driving_lab.cpp, which pins
// both this constant and the rule).
//
// The stock 0.20/0.45 travel drops a wheel 0.375 m. That is what jacked every
// commandeered car into the air and made the tall ones feel top-heavy — the CoG
// rises with the body. Real cars move a fraction of that.
// (The rule itself lives in PhysicsWorld so the drivable spec shares it.)
constexpr Real kSuspensionMin = PhysicsWorld::kStreetSuspensionMin;
constexpr Real kSuspensionMax = PhysicsWorld::kStreetSuspensionMax;
constexpr Real kSuspensionRestDrop = PhysicsWorld::kStreetSuspensionRestDrop;

// A Jolt vehicle config sized to a fleet body — a 4WD arcade car like the
// player's sedan, scaled up for a van/box-truck. Front wheels steer; all four
// are driven (heavy bodies still pull away), rear wheels take the handbrake.
//
// `layout` is the fleet recipe's own wheelset (vehicles.lua `wheel_layout`),
// cut from the same fitted geometry as the drawn car. Prefer it: the derived
// fallback below guesses an axle offset from the body length and a radius from
// its height, and those guesses miss the drawn arches by several centimetres —
// the same class of mismatch as the drivable spec's old hardcoded 0.34 radius
// against a 0.62 m wheel. The fallback survives only for a recipe that
// publishes no layout at all.
PhysicsWorld::VehicleConfig configFromBody(
    const VehicleBody& b, const std::vector<CityRenderSystem::CarWheel>& layout) {
    PhysicsWorld::VehicleConfig c;
    c.chassisHalfExtent = Vec3(b.width * 0.5, b.height * 0.5, b.length * 0.5);
    c.mass = 900.0 + b.length * b.width * 180.0;   // ~1500 kg sedan, more for a truck
    c.maxSteerDegrees = 32.0;
    c.engineTorque = 650.0;
    c.maxRPM = 6000.0;
    c.brakeTorque = 1600.0;
    c.handBrakeTorque = 4200.0;
    c.friction = 1.0;
    // Centre of gravity, derived rather than fixed. A flat -0.4 sits low under a
    // sedan but high under a 2.8 m box truck, which is the other half of "top
    // heavy". Same anchor the drivable spec uses (vehicles.lua), so a sedan lands
    // on the value it was hand-tuned to and taller bodies drop further.
    c.comOffsetY = -(0.23 + 0.22 * (b.height / 1.45));

    // Lift each attachment point by the drop so the wheel RESTS where the car is
    // drawn (see kSuspensionRestDrop above).
    auto attach = [](PhysicsWorld::VehicleWheel& w, Vec3 restCentre) {
        w.position = restCentre + Vec3(0, kSuspensionRestDrop, 0);
        w.suspensionMin = kSuspensionMin;
        w.suspensionMax = kSuspensionMax;
    };

    if (!layout.empty()) {
        for (const CityRenderSystem::CarWheel& s : layout) {
            PhysicsWorld::VehicleWheel w;
            attach(w, s.pos);
            w.radius = s.radius;
            w.width = s.width;
            w.steered = s.steered;
            w.driven = s.driven;
            w.handBrake = s.handBrake;
            c.wheels.push_back(w);
        }
        return c;
    }

    Real hw = b.width * 0.5 - 0.10;
    Real hl = b.length * 0.34;
    Real wr = std::max(Real(0.30), b.height * 0.28);
    auto wheel = [&](Real x, Real z, bool front) {
        PhysicsWorld::VehicleWheel w;
        attach(w, Vec3(x, -b.height * 0.5 + wr, z));
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
    // The ADOPTED fleet's body, so the promoted car's chassis, mass and CoG come
    // from the same catalogue entry its drawn shell was built from.
    const VehicleBody& body = sim.fleetBody(a.vehicle);

    // WHERE THE CAR ACTUALLY IS. agentWorldPose resolves the ghost's (x,z) onto
    // the terrain and adds its elevation (a bridge deck, a freeway ramp); the
    // spawn used to be a bare `height/2 + 0.25`, an ABSOLUTE y that silently
    // assumed the whole city sat at sea level. On flat test ground that reads
    // fine, which is why it survived. On piedmont, whose downtown is tens of
    // metres up, every commandeered car was created that far UNDER the road —
    // it fell out of the world, and VehicleSystem then found no vehicle within
    // reach to seat the player in, so pressing enter simply appeared to do
    // nothing while the ambient car vanished.
    Vec3 groundPos;
    Vec2 groundHeading;
    if (!city_.agentWorldPose(bestAgent, groundPos, groundHeading)) return;

    Entity e = world.create();
    Transform t;
    t.position = Vec3(groundPos.x, groundPos.y + body.height * 0.5 + 0.25,
                      groundPos.z);
    t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0),
                                        std::atan2(groundHeading.x, groundHeading.y));
    t.scale = Vec3(1, 1, 1);
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, PrevTransform{t});

    Renderable r;
    // THE SAME CAR the player just walked up to. The level's Lua fleet body,
    // minus its baked wheels (VehicleSystem grows real ones). This used to be
    // fleetCarMesh unconditionally — so the whole city was mesh.car and the one
    // car in the player's hands turned into the retired box the moment they
    // pressed enter. Only a level with no fleet recipes falls back now.
    r.mesh = city_.carChassisMesh(a.vehicle);
    if (!r.mesh.valid()) {
        // No wheel-less body for this slot means the level shipped no vehicle
        // catalogue — in which case there was no ambient car to walk up to
        // either, so this is unreachable in a real city. There is deliberately
        // nothing to substitute: a C++ box car is what this whole round removed.
        LOG_WARN << "[cityveh] slot " << a.vehicle
                 << " has no body to promote — not commandeering";
        world.destroy(e);
        return;
    }
    // RETAIN: this handle is the render bridge's, acquired once at build. Taking
    // a second reference is what keeps it alive if the bridge ever drops its own
    // (a city rebuild must not free a mesh the player is driving).
    ctx.assets.retain(r.mesh);
    r.material.albedo = Vec3(1, 1, 1);   // hue baked in the mesh vertex colour
    r.material.metallic = 0.4f;
    r.material.roughness = 0.5f;
    r.material.opacity = 1.0f;
    world.add<Renderable>(e, r);

    Vehicle v;
    v.config = configFromBody(body, city_.carWheels(a.vehicle));

    // LAMPS from the recipe's markers, for the same reason the wheels come from
    // its layout: without them VehicleSystem places four lenses at guessed
    // chassis corners. Those corners nearly coincided with the retired box car's
    // baked lamp boxes, so the guess was invisible — against a real body, whose
    // housings are inset from the extremes, it lit a second pair beyond the
    // bodywork and every commandeered car drove around with doubled taillights.
    Vec3 seat(0, 0, 0);
    bool hasSeat = false;
    for (const PromotedLamp& l :
         promotedLamps(city_.carLamps(a.vehicle), &seat, &hasSeat))
        v.lamps.push_back(Vehicle::Lamp{Entity{}, l.local, l.front, l.left});
    if (hasSeat) {
        v.driverSeat = seat;
        v.hasDriverSeat = true;
    }
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
