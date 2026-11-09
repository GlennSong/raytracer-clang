#include "city_vehicles.h"
#include "../../engine/interact_broker.h"
#include "../../engine/interaction.h"   // engine::Seated

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"       // Transform, Vehicle, Renderable, InVehicle
#include "../../engine/physics/physics_world.h"   // VehicleConfig (was via the header)
#include "../../engine/world.h"
#include "../../engine/script_assets.h"   // loadScriptCode, makeModuleSource
#ifdef RT_ENABLE_SCRIPTING
#include "../../engine/scripting/procgen_bindings.h"
#include "../../engine/scripting/script_modules.h"
#include "../../engine/scripting/script_vm.h"
#include "../../engine/scripting/vehicle_spec.h"
#endif
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

}  // namespace

void CityVehicleSystem::update(engine::FrameContext& ctx) {
    // Commandeering: the same button as VehicleSystem's enter. This system is
    // registered FIRST, so on the frame the player presses enter next to an
    // AMBIENT car, we promote it here and VehicleSystem (later this frame) finds
    // a real, unoccupied Vehicle within reach and seats the player as usual.
    // THE OFFER (the interaction broker, E): the nearest ambient car in reach, four times a second -- a real
    // Vehicle in reach is VehicleSystem's offer. The broker's command comes back to VehicleSystem, which presses
    // enter_vehicle; that press is what promotes the car below.
    // A PARKED CAR's HOOD OR TRUNK, opened on foot (ADR-0356; Glenn: "the cars in the world are not the same as the one
    // I am in ... I can't open the hatch"): the car becomes the real one where it stands, and the panel swings open
    if (city_.built()) {
        Entity pl;
        ctx.world.each<Transform, engine::ControlledBy>([&](Entity e, Transform&, engine::ControlledBy&) {
            if (!pl.valid()) pl = e;
        });
        engine::InteractCommand cmd;
        if (pl.valid() && engine::takeInteractCommand(ctx.world, pl, "city_panel", cmd) && ambientAgent_ >= 0) {
            const Entity car = promote(ctx, ambientAgent_);
            if (Vehicle* v = car.valid() ? ctx.world.get<Vehicle>(car) : nullptr) {
                for (Vehicle::Panel& p : v->panels) {
                    const bool back = p.name == "trunk" || p.name == "hatch" || p.name == "tailgate";
                    if ((cmd.key == 0 && p.name == "hood") || (cmd.key == 1 && back)) { p.target = p.open; p.hold = 0.0f; }
                }
                LOG_INFO << "[cityveh] opened a parked car's " << (cmd.key == 0 ? "hood" : "trunk");
            }
            ambientAgent_ = -1;
            haveAmbient_ = false;
        }
    }
    if (city_.built() && !ctx.actions.pressed("enter_vehicle")) {
        scanT_ -= ctx.frameDelta;
        if (scanT_ <= 0) {
            scanT_ = 0.25;
            ambientAgent_ = -1;
            Entity pl;
            Vec3 pp;
            ctx.world.each<Transform, engine::ControlledBy>([&](Entity e, Transform& t, engine::ControlledBy&) {
                if (!pl.valid()) { pl = e; pp = t.position; }
            });
            if (pl.valid() && !ctx.world.has<engine::InVehicle>(pl)) {
                bool real = false;
                ctx.world.each<Transform, Vehicle>([&](Entity, Transform& t, Vehicle&) {
                    const Real dx = t.position.x - pp.x, dz = t.position.z - pp.z;
                    if (dx * dx + dz * dz <= kCommandeerRadius * kCommandeerRadius) real = true;
                });
                const auto& ag = city_.sim().agents();
                Real bestD2 = kCommandeerRadius * kCommandeerRadius;
                int best = -1;
                for (int i = 0; i < static_cast<int>(ag.size()) && !real; ++i) {
                    const Agent& a = ag[static_cast<std::size_t>(i)];
                    if (a.mode != Agent::Mode::Driver || a.vehicle < 0 || a.released || a.playerControlled) continue;
                    const Real dx = a.pos.x - pp.x, dz = a.pos.y - pp.z;
                    if (dx * dx + dz * dz <= bestD2) { bestD2 = dx * dx + dz * dz; best = i; }
                }
                ambientAgent_ = best;
            }
        }
        // THE DRIVER'S SEAT, where the car is this frame (Glenn: "we'd have to be pointing at the seat that we
        // want"): the offer sits on it, beside CityPlayerTransitSystem's passenger seats, so looking at the driver's
        // side takes the car and looking at another seat rides along
        haveAmbient_ = false;
        if (ambientAgent_ >= 0) {
            engine::Mat4 pose;
            std::vector<Vec3> seats;
            Vec3 gp;
            Vec2 gh;
            if (city_.carSeatsOf(ambientAgent_, &pose, &seats)) {
                ambientAt_ = pose.transformPoint(seats[0]) + Vec3(0, 0.35, 0);
                haveAmbient_ = true;
            } else if (city_.agentWorldPose(ambientAgent_, gp, gh)) {
                ambientAt_ = gp + Vec3(0, 0.9, 0);
                haveAmbient_ = true;
            }
        }
        if (haveAmbient_) {
            Entity pl;
            ctx.world.each<Transform, engine::ControlledBy>([&](Entity e, Transform&, engine::ControlledBy&) {
                if (!pl.valid()) pl = e;
            });
            if (pl.valid() && !ctx.world.has<engine::InVehicle>(pl) && !ctx.world.has<engine::Seated>(pl)) {
                engine::InteractOffer o;
                o.provider = "vehicle";
                o.name = "driver's seat";
                o.tap = "take the car";
                o.anchor = ambientAt_;
                o.reach = kCommandeerRadius;
                o.needsSight = false;
                engine::offerInteraction(ctx.world, pl, o);
                // ...and a PARKED one's hood and trunk, at its nose and its tail (ADR-0356)
                const Agent& amb = city_.sim().agents()[static_cast<std::size_t>(ambientAgent_)];
                Vec3 gp;
                Vec2 gh;
                const int slot = city_.carSlotOf(ambientAgent_);
                std::string recipe;
                Vec3 paint;
                if (!amb.moving && slot >= 0 && city_.carDrivable(slot, recipe, paint) && city_.agentWorldPose(ambientAgent_, gp, gh)) {
                    const Real hl = city_.sim().fleetBody(slot).length * 0.5;
                    const Real gl = gh.length();
                    const Vec2 f = gl > 1e-6 ? gh * (1.0 / gl) : Vec2(0, 1);
                    for (int end = 0; end < 2; ++end) {
                        const Real s = end == 0 ? hl - 0.6 : -(hl - 0.4);
                        engine::InteractOffer po;
                        po.provider = "city_panel";
                        po.key = static_cast<uint64_t>(end);
                        po.name = end == 0 ? "hood" : "trunk";
                        po.tap = "open";
                        po.anchor = Vec3(gp.x + f.x * s, gp.y + 0.9, gp.z + f.y * s);
                        po.reach = 2.2;
                        po.needsSight = false;
                        engine::offerInteraction(ctx.world, pl, po);
                    }
                }
            }
        }
        return;
    }
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

    promote(ctx, bestAgent);
}

// PROMOTE an ambient car: release its agent (its instanced car vanishes this tick) and spawn a real Vehicle of the
// same car at the same pose -- for the player to get into, or to open a panel of (ADR-0356). The new entity, or none.
Entity CityVehicleSystem::promote(engine::FrameContext& ctx, int bestAgent) {
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    const auto& agents = sim.agents();
    // PROMOTE: release the agent (its instanced car + widget vanish this tick)
    // and spawn a real Vehicle with the same fleet body at the same pose.
    const Agent& a = agents[bestAgent];
    // THE MODEL IT IS DRAWN AS (#115): every part below by the draw slot, not the vehicle index -- they differ
    // whenever the fleet has service vehicles, and the car in the player's hands turned into another model.
    const int slot = city_.carSlotOf(bestAgent);
    if (slot < 0) return Entity{};
    // The ADOPTED fleet's body, so the promoted car's chassis, mass and CoG come
    // from the same catalogue entry its drawn shell was built from.
    const VehicleBody& body = sim.fleetBody(slot);
    // ...and what it drives with: the slot's handling (item 23, #117), the same config its kit or class gives the
    // car the player drops from the menu -- one handling builder, so a commandeered sedan drives as that sedan.
    const PhysicsWorld::VehicleConfig* own = city_.carConfig(slot);
    if (!own) {
        LOG_WARN << "[cityveh] slot " << slot << " publishes no handling -- not commandeering";
        return Entity{};
    }

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
    if (!city_.agentWorldPose(bestAgent, groundPos, groundHeading)) return Entity{};

    // THE SAME CAR THE MENU DROPS (ADR-0355; Glenn: "Why is an SUV in the world different than mine? I can't open
    // the hatch"): the slot's drivable recipe in its paint -- panels on hinges, a body to dent, its damage tuning --
    // built fresh (a rare action, as N's drop is). A slot without one keeps the traffic body below.
#ifdef RT_ENABLE_SCRIPTING
    {
        std::string recipe;
        Vec3 paint;
        if (city_.carDrivable(slot, recipe, paint)) {
            const std::string lib = engine::loadScriptCode("vehicles.lua", "");
            engine::ScriptVM vm;
            engine::openProcgenLibrary(vm);
            engine::openModuleLoader(vm, engine::makeModuleSource(""));
            std::string err;
            engine::VehicleSpec spec;
            char chunk[256];
            std::snprintf(chunk, sizeof chunk, "return vehicle.%s(seed, { color = { %.4f, %.4f, %.4f } })", recipe.c_str(),
                          paint.x, paint.y, paint.z);
            if (!lib.empty() && vm.doString(lib, &err) && engine::loadVehicleSpec(vm, chunk, 1u, spec, &err)) {
                const Vec3 at(groundPos.x, groundPos.y + spec.config.chassisHalfExtent.y + 0.35, groundPos.z);
                const Real yawDeg = std::atan2(groundHeading.x, groundHeading.y) * 57.29577951308232;
                const Entity car = engine::spawnVehicle(world, ctx.assets, spec, at, yawDeg);
                LOG_INFO << "[cityveh] took agent " << bestAgent << "'s car as the drivable " << recipe << ": vehicle "
                         << a.vehicle << ", model slot " << slot;
                if (!city_.simMutable().ejectDriver(bestAgent)) city_.simMutable().releaseDriver(bestAgent);
                promoted_.push_back(PromotedCar{car, bestAgent});
                return car;
            }
            LOG_WARN << "[cityveh] slot " << slot << "'s drivable " << recipe << ": " << err << " -- the traffic body instead";
        }
    }
#endif

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
    r.mesh = city_.carChassisMesh(slot);
    if (!r.mesh.valid()) {
        // No wheel-less body for this slot means the level shipped no vehicle
        // catalogue — in which case there was no ambient car to walk up to
        // either, so this is unreachable in a real city. There is deliberately
        // nothing to substitute: a C++ box car is what this whole round removed.
        LOG_WARN << "[cityveh] slot " << slot
                 << " has no body to promote — not commandeering";
        world.destroy(e);
        return Entity{};
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
    v.config = *own;

    // GLASS AND CABIN as body parts pinned to the chassis (as vehicle_spec does for a spawned car). The ambient
    // car drew them as instance groups keyed to the agent, which stop drawing the moment it is released -- Glenn
    // got into a jeep and looked straight through the empty shell at the road.
    auto addPart = [&](engine::MeshHandle mh, float opacity, float roughness, float metallic) {
        if (!mh.valid()) return;
        Entity pe = world.create();
        world.add<Transform>(pe, t);
        world.add<PrevTransform>(pe, PrevTransform{t});
        Renderable pr;
        pr.mesh = mh;
        ctx.assets.retain(mh);
        pr.material.albedo = Vec3(1, 1, 1);
        pr.material.metallic = metallic;
        pr.material.roughness = roughness;
        pr.material.opacity = opacity;
        world.add<Renderable>(pe, pr);
        v.bodyParts.push_back(pe);
    };
    addPart(city_.carGlassMesh(slot), city_.carGlassOpacity(slot), 0.06f, 0.0f);
    addPart(city_.carCabinMesh(slot), 1.0f, 0.9f, 0.0f);

    // LAMPS from the recipe's markers, for the same reason the wheels come from
    // its layout: without them VehicleSystem places four lenses at guessed
    // chassis corners. Those corners nearly coincided with the retired box car's
    // baked lamp boxes, so the guess was invisible — against a real body, whose
    // housings are inset from the extremes, it lit a second pair beyond the
    // bodywork and every commandeered car drove around with doubled taillights.
    Vec3 seat(0, 0, 0);
    bool hasSeat = false;
    for (const PromotedLamp& l :
         promotedLamps(city_.carLamps(slot), &seat, &hasSeat))
        v.lamps.push_back(Vehicle::Lamp{l.local, l.front, l.left});
    // THE DRIVER'S SEAT: a marker if the recipe placed one, else the fleet recipe's own `driver_seat` (the Lua fleet
    // publishes the seat apart from its lamps, so the marker was never there and VehicleSystem's guess -- the LEFT
    // seat -- put the player in the passenger's place of a right-hand-drive car; Glenn: "when I'm driving a car I'm
    // in the wrong seat")
    if (!hasSeat) hasSeat = city_.carDriverSeat(slot, seat);
    if (hasSeat) {
        v.driverSeat = seat;
        v.hasDriverSeat = true;
    }
    v.openSeat = city_.carOpenSeat(slot);
    world.add<Vehicle>(e, v);   // VehicleSystem builds the Jolt car + wheels + lamps

    LOG_INFO << "[cityveh] took agent " << bestAgent << "'s car: vehicle " << a.vehicle << ", model slot " << slot;
    // the driver gets out onto the pavement and stands there (no longer vanishing with the car)
    if (!city_.simMutable().ejectDriver(bestAgent)) city_.simMutable().releaseDriver(bestAgent);
    promoted_.push_back(PromotedCar{e, bestAgent});
    return e;
}

void CityVehicleSystem::onStop(engine::FrameContext&) {
    // Tracking only; world/physics teardown reclaims the entities (the same
    // no-removal-hook note as VehicleSystem — fine at level teardown).
    promoted_.clear();
}

}  // namespace citysim
