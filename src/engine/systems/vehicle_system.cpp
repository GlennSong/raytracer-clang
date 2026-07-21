#include "vehicle_system.h"

#include "../vehicle_lamps.h"
#include "../procgen/vehicle/occupant.h"

#include "../components.h"
#include "../asset_manager.h"
#include "../mesh_builder.h"
#include "physics_system.h"
#include "camera_system.h"
#include "../../log.h"

#include <vector>

#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/script_vm.h"
#include "../scripting/procgen_bindings.h"
#include "../scripting/vehicle_spec.h"
#include "../script_assets.h"
#include "../scripting/script_modules.h"
#include <fstream>
#include <sstream>
#endif

namespace engine {

void VehicleSystem::onStart(FrameContext& ctx) {
    // Driving actions (global; single-player drive for now). Throttle/steer are
    // axes, brake/handbrake/enter are buttons. Gamepad face-button names may differ
    // per backend — adjust on a real build.
    ctx.actions.bindAxis("drive_throttle", KeyCode::W, 1.0);
    ctx.actions.bindAxis("drive_throttle", KeyCode::S, -1.0);
    ctx.actions.bindAxis("drive_throttle", GamepadAxis::RightTrigger, 1.0);
    ctx.actions.bindAxis("drive_throttle", GamepadAxis::LeftTrigger, -1.0);

    ctx.actions.bindAxis("drive_steer", KeyCode::D, 1.0);
    ctx.actions.bindAxis("drive_steer", KeyCode::A, -1.0);
    ctx.actions.bindAxis("drive_steer", GamepadAxis::LeftX, 1.0);

    ctx.actions.bindButton("drive_brake", KeyCode::Space);
    ctx.actions.bindButton("drive_handbrake", KeyCode::LeftControl);

    ctx.actions.bindButton("enter_vehicle", KeyCode::G);
    ctx.actions.bindButton("enter_vehicle", GamepadButton::DpadUp);

    // Recover a rolled car (keep heading, level it, lift it).
    ctx.actions.bindButton("vehicle_flip", KeyCode::T);
    // Toggle the head/taillights.
    ctx.actions.bindButton("vehicle_lights", KeyCode::L);
    // Debug: spawn a fresh car in front of the player (always on solid ground,
    // since the player is standing on a collider).
    ctx.actions.bindButton("spawn_vehicle", KeyCode::N);
}

void VehicleSystem::spawnInFront(FrameContext& ctx) {
#ifdef RT_ENABLE_SCRIPTING
    // The player's stance + look direction.
    Entity player;
    Vec3 pos;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (!player.valid()) { player = e; pos = t.position; }
    });
    if (!player.valid()) return;

    Vec3 fwd = ctx.view.camera.target - ctx.view.camera.position;
    fwd.y = 0;
    Real fl = fwd.length();
    fwd = fl > 1e-3 ? fwd * (1.0 / fl) : Vec3(0, 0, -1);
    Vec3 spawn = pos + fwd * 6.0 + Vec3(0, 1.5, 0);   // a few metres ahead, dropped in
    // Face the car the way the player looks (Y-rotation of its local +Z front).
    Real yawDeg = radiansToDegrees(std::atan2(fwd.x, fwd.z));

    // Read vehicles.lua and build a sedan (rare debug action — rebuild the VM each time).
    const std::string lib = loadScriptCode("vehicles.lua", "");
    if (lib.empty()) { LOG_WARN << "spawn_vehicle: vehicles.lua not found"; return; }

    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(""));
    std::string err;
    if (!vm.doString(lib, &err)) { LOG_WARN << "vehicles.lua: " << err; return; }
    VehicleSpec spec;
    if (loadVehicleSpec(vm, "return vehicle.sedan(seed, {})",
                        static_cast<uint32_t>(++spawnCount_), spec, &err))
        spawnVehicle(ctx.world, ctx.assets, spec, spawn, yawDeg);
    else
        LOG_WARN << "spawn_vehicle: " << err;
#else
    (void)ctx;
#endif
}

void VehicleSystem::createVehicles(FrameContext& ctx) {
    // Collect vehicles awaiting a Jolt body first — spawning wheel entities is a
    // structural mutation, forbidden inside World::each (ADR-0006).
    std::vector<Entity> pending;
    ctx.world.each<Transform, Vehicle>([&](Entity e, Transform&, Vehicle& v) {
        if (v.vehicleId == PhysicsWorld::INVALID_VEHICLE) pending.push_back(e);
    });
    if (pending.empty()) return;

    PhysicsWorld& pw = physicsSys.physicsWorld();
    for (Entity e : pending) {
        Vehicle* v = ctx.world.get<Vehicle>(e);
        Transform* t = ctx.world.get<Transform>(e);
        if (!v || !t) continue;
        v->vehicleId = pw.addVehicle(v->config, t->position, t->orientation);
        if (v->vehicleId == PhysicsWorld::INVALID_VEHICLE) continue;
        if (!ctx.world.has<PrevTransform>(e)) ctx.world.add<PrevTransform>(e, {*t});

        // Lazily build a shared UNIT wheel mesh: a unit cylinder (diameter 1 along
        // Y, width 1) rotated so its axle is local X. The actual per-wheel size is
        // applied as a Transform scale in writeBack — so the wheel is correct
        // regardless of whether Jolt bakes wheel size into GetWheelWorldTransform
        // (which scaled a pre-sized mesh down to a stub — the "tiny wheels" bug).
        if (!wheelMesh.valid()) {
            RenderMesh wheel = MeshBuilder::cylinder(0.5f, 1.0f);
            MeshBuilder::transform(
                wheel, Mat4::trs(Vec3(0, 0, 0),
                                 Quat::fromAxisAngle(Vec3(0, 0, 1), PI * 0.5),
                                 Vec3(1, 1, 1)));
            wheelMesh = ctx.assets.acquireMesh(wheel, "vehicle:wheel");
        }
        for (size_t i = 0; i < v->config.wheels.size(); ++i) {
            Entity we = ctx.world.create();
            Transform wt;
            ctx.world.add<Transform>(we, wt);
            ctx.world.add<PrevTransform>(we, {wt});
            Renderable r;
            r.mesh = wheelMesh;
            r.material.albedo = Vec3(0.05, 0.05, 0.06);
            r.material.metallic = 0.1f;
            r.material.roughness = 0.8f;
            r.material.opacity = 1.0f;
            ctx.world.add<Renderable>(we, r);
            v->wheelEntities.push_back(we);
        }

        // Head/taillight lens boxes (emission toggled in writeBack) + a driver
        // capsule (stowed until someone's aboard). Shared meshes, built once.
        if (!lensMesh.valid())
            lensMesh = ctx.assets.acquireMesh(MeshBuilder::box(Vec3(0.34, 0.18, 0.10)),
                                              "vehicle:lens");
        // A SEATED occupant, not a capsule. The capsule proved a car was
        // occupied; this shows who is driving it, posed from the same SAE manikin
        // the interior is packaged around so it fits the seat it sits in.
        if (!driverMesh.valid())
            driverMesh = ctx.assets.acquireMesh(buildSeatedOccupant(), "vehicle:driver");
        auto makeLens = [&](Vec3 albedo) {
            Entity le = ctx.world.create();
            Transform lt;
            ctx.world.add<Transform>(le, lt);
            ctx.world.add<PrevTransform>(le, {lt});
            Renderable r;
            r.mesh = lensMesh;
            r.material.albedo = albedo;     // always a coloured lens (front/back cue)
            r.material.roughness = 0.4f;
            r.material.opacity = 1.0f;
            ctx.world.add<Renderable>(le, r);
            return le;
        };
        if (!v->lamps.empty()) {
            // Recipe-driven: one lens per marker, wherever the body generator put
            // it. This is what lets a generated car's lamps sit in its actual
            // housings instead of at guessed chassis corners.
            for (Vehicle::Lamp& lamp : v->lamps)
                lamp.entity = makeLens(lamp.front ? Vec3(1.0, 0.97, 0.82)
                                                  : Vec3(0.85, 0.06, 0.05));
        } else {
            // No marker data (hand-built or legacy body): fall back to corners so
            // every car still has lamps.
            const Vec3& hx = v->config.chassisHalfExtent;
            const Real lx = hx.x - 0.30, ly = -hx.y * 0.15;
            for (int i = 0; i < 4; ++i) {
                const bool front = i < 2;
                const bool left = (i % 2) == 1;
                Vehicle::Lamp lamp;
                lamp.front = front;
                lamp.left = left;
                lamp.local = Vec3(left ? -lx : lx, ly,
                                  front ? hx.z + 0.05 : -hx.z - 0.05);
                lamp.entity = makeLens(front ? Vec3(1.0, 0.97, 0.82)
                                             : Vec3(0.85, 0.06, 0.05));
                v->lamps.push_back(lamp);
            }
        }
        for (const Vehicle::Lamp& lamp : v->lamps)
            (lamp.front ? v->headlights : v->taillights).push_back(lamp.entity);

        v->driverModel = ctx.world.create();
        {
            Transform dt;
            ctx.world.add<Transform>(v->driverModel, dt);
            ctx.world.add<PrevTransform>(v->driverModel, {dt});
            Renderable r;
            r.mesh = driverMesh;
            r.material.albedo = Vec3(0.30, 0.34, 0.48);
            r.material.roughness = 0.7f;
            r.material.opacity = 1.0f;
            ctx.world.add<Renderable>(v->driverModel, r);
        }
    }
}

void VehicleSystem::driveVehicles(FrameContext& ctx) {
    Real throttle = ctx.actions.axis("drive_throttle");
    Real steer = ctx.actions.axis("drive_steer");
    Real brake = ctx.actions.held("drive_brake") ? 1.0 : 0.0;
    Real hand = ctx.actions.held("drive_handbrake") ? 1.0 : 0.0;

    PhysicsWorld& pw = physicsSys.physicsWorld();
    // One control path for EVERY car (ADR-0062). Whoever drives — the seated player
    // or an AI brain — produces the same DriverInput, fed to the same Jolt vehicle.
    ctx.world.each<Vehicle>([&](Entity e, Vehicle& v) {
        if (v.vehicleId == PhysicsWorld::INVALID_VEHICLE) return;
        DriverInput in;
        if (v.driver.valid()) {
            // The player is seated: host input drives it.
            in.throttle = throttle;
            in.steer = steer;
            in.brake = brake;
            in.handBrake = hand;
        } else if (AgentDriver* ad = ctx.world.get<AgentDriver>(e)) {
            // An AI brain drives it: turn what the brain WANTS (heading + speed)
            // into the same pedal/wheel values through the shared controller, so
            // this NPC car obeys identical physics to the player's.
            DriverState s;
            Quat q = pw.vehicleOrientation(v.vehicleId);
            Vec3 fwd = q.rotate(Vec3(0, 0, 1));       // chassis forward in world
            s.forward = Vec2(fwd.x, fwd.z);
            Vec3 vel = pw.vehicleVelocity(v.vehicleId);
            s.speed = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z;   // forward m/s
            in = computeDriverInput(s, ad->command, ad->tuning);
        } else {
            // Nobody at the wheel: hold the brake so it doesn't creep on a slope.
            in.brake = 1.0;
        }
        v.throttle = in.throttle;
        v.steer = in.steer;
        v.brake = in.brake;
        v.handBrake = in.handBrake;
        pw.setVehicleInput(v.vehicleId, in.throttle, in.steer, in.brake, in.handBrake);
    });
}

void VehicleSystem::writeBack(FrameContext& ctx) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    ctx.world.each<Transform, PrevTransform, Vehicle>(
        [&](Entity e, Transform& t, PrevTransform& prev, Vehicle& v) {
            if (v.vehicleId == PhysicsWorld::INVALID_VEHICLE) return;
            prev.value = t;
            t.position = pw.vehiclePosition(v.vehicleId);
            t.orientation = pw.vehicleOrientation(v.vehicleId);

            for (size_t i = 0; i < v.wheelEntities.size(); ++i) {
                Entity we = v.wheelEntities[i];
                if (!ctx.world.alive(we)) continue;
                Transform* wt = ctx.world.get<Transform>(we);
                if (!wt) continue;
                if (PrevTransform* wp = ctx.world.get<PrevTransform>(we))
                    wp->value = *wt;
                // Pose from Jolt (position + orientation, scale-normalized), then
                // force the wheel's true size onto the unit mesh: axle (X) = width,
                // the disc (Y,Z) = diameter. Ignoring the matrix's own scale is what
                // fixes the tiny wheels.
                Transform pose = transformFromMatrix(
                    pw.wheelTransform(v.vehicleId, static_cast<int>(i)));
                Real radius = (i < v.config.wheels.size()) ? v.config.wheels[i].radius : 0.34;
                Real width = (i < v.config.wheels.size()) ? v.config.wheels[i].width : 0.24;
                pose.scale = Vec3(width, 2.0 * radius, 2.0 * radius);
                *wt = pose;
            }

            // Place a child entity at a chassis-local offset (rigidly attached).
            const Vec3& hx = v.config.chassisHalfExtent;
            auto place = [&](Entity child, const Vec3& local) {
                if (!child.valid() || !ctx.world.alive(child)) return;
                Transform* ct = ctx.world.get<Transform>(child);
                if (!ct) return;
                if (PrevTransform* cp = ctx.world.get<PrevTransform>(child))
                    cp->value = *ct;
                ct->position = t.position + t.orientation.rotate(local);
                ct->orientation = t.orientation;
            };

            // Body parts (glass, interior) share the chassis frame exactly — they
            // were authored in the same space as the body mesh — so they ride at
            // local origin.
            for (Entity part : v.bodyParts) place(part, Vec3(0, 0, 0));
            // --- lamps -----------------------------------------------------
            // The player's car now runs the SAME predicate as city traffic
            // (engine::vehicleLampState): headlights at dusk, brake lights on
            // deceleration, indicators while turning. Previously this was a
            // manual on/off toggle with no brakes and no indicators at all.
            v.prevSpeed = v.speed;
            v.speed = pw.vehicleVelocity(v.vehicleId).length();

            // Indicators come from STEERING, the player's equivalent of the
            // route bend citysim reads. `signalHold` keeps them on briefly after
            // the wheel straightens, otherwise they strobe on every correction.
            constexpr Real kSignalSteer = 0.35;
            constexpr Real kSignalHold = 0.6;
            if (std::fabs(v.steer) > kSignalSteer) {
                v.turnSignal = v.steer > 0 ? +1 : -1;
                v.signalHold = kSignalHold;
            } else if (v.signalHold > 0) {
                v.signalHold -= ctx.clock.fixedStep();
            } else {
                v.turnSignal = 0;
            }

            const bool braking = v.brake > 0.15 || v.handBrake > 0.15;
            const LampMotion motion =
                braking ? LampMotion::Holding
                        : (v.turnSignal != 0 ? LampMotion::Turning : LampMotion::Rolling);
            const VehicleLamps lit = vehicleLampState(
                motion, v.speed, v.prevSpeed, lampsDark(ctx.view.lighting.sun.direction),
                v.turnSignal, v.lightsOn);
            const bool blink = lampBlinkOn(ctx.clock.simulatedTime());

            for (const Vehicle::Lamp& lamp : v.lamps) {
                place(lamp.entity, lamp.local);
                Renderable* r = ctx.world.get<Renderable>(lamp.entity);
                if (!r) continue;
                const bool signalling = (lamp.left ? lit.left : lit.right) && blink;
                Vec3 e(0, 0, 0);
                if (signalling)          e = Vec3(2.4, 1.05, 0.07);   // amber wins
                else if (lamp.front)     e = lit.head ? Vec3(3.0, 2.8, 2.2) : Vec3(0, 0, 0);
                else if (lit.brake)      e = Vec3(2.6, 0.10, 0.08);   // hard red
                else if (lit.head)       e = Vec3(0.7, 0.05, 0.04);   // dim tail glow
                r->material.emission = e;
            }

            // Driver capsule: in the seat when occupied — by a seated PLAYER or by
            // an AI brain (AgentDriver) — stowed far below otherwise. Seeing the
            // capsule is the proof that an agent is IN the car, not that the car
            // moves on its own (ADR-0062).
            bool occupied = v.driver.valid() || ctx.world.has<AgentDriver>(e);
            if (v.driverModel.valid() && ctx.world.alive(v.driverModel)) {
                if (Transform* dt = ctx.world.get<Transform>(v.driverModel)) {
                    if (PrevTransform* dp = ctx.world.get<PrevTransform>(v.driverModel))
                        dp->value = *dt;
                    // The recipe's own hip point when it has one — the occupant
                    // mesh's origin IS its hip, so this lines the two up exactly
                    // instead of guessing an offset off the chassis box.
                    const Vec3 seat =
                        v.hasDriverSeat
                            ? v.driverSeat
                            : Vec3(-hx.x * 0.35, hx.y * 0.15, -hx.z * 0.05);
                    if (occupied)
                        dt->position = t.position + t.orientation.rotate(seat);
                    else
                        dt->position = Vec3(0, -100000, 0);
                    dt->orientation = t.orientation;
                }
            }

            // Keep the seated driver glued to the chassis so they ride along and
            // are sensibly placed when they get out.
            if (v.driver.valid() && ctx.world.alive(v.driver)) {
                if (Transform* pt = ctx.world.get<Transform>(v.driver))
                    pt->position = t.position;
            }
        });
}

void VehicleSystem::handleEnterExit(FrameContext& ctx) {
    if (!ctx.actions.pressed("enter_vehicle")) return;

    // The (first) player entity.
    Entity player;
    Vec3 playerPos;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (!player.valid()) { player = e; playerPos = t.position; }
    });
    if (!player.valid()) return;

    if (ctx.world.has<InVehicle>(player)) {
        // --- get out ---
        Entity car = ctx.world.get<InVehicle>(player)->vehicle;
        if (car.valid() && ctx.world.alive(car) && ctx.world.has<Vehicle>(car))
            ctx.world.get<Vehicle>(car)->driver = Entity{};
        if (car.valid() && ctx.world.has<Transform>(car)) {
            const Transform& ct = *ctx.world.get<Transform>(car);
            // Step out well CLEAR of the car: up (world up, so a flipped car still
            // ejects you upward, never trapping you under it) plus a flattened
            // sideways nudge. You drop onto your feet beside the car.
            Vec3 side = ct.orientation.rotate(Vec3(1, 0, 0));
            side.y = 0;
            Real sl = side.length();
            side = sl > 1e-3 ? side * (1.0 / sl) : Vec3(1, 0, 0);
            Vec3 out = ct.position + Vec3(0, 2.5, 0) + side * 2.0;
            if (Transform* pt = ctx.world.get<Transform>(player)) pt->position = out;
            if (CharacterController* cc = ctx.world.get<CharacterController>(player))
                physicsSys.physicsWorld().setCharacterPosition(cc->characterId, out);
        }
        cameras.clearFollowTarget();
        ctx.world.remove<InVehicle>(player);   // structural — after the each() above
    } else {
        // --- get in: the nearest car within reach, occupied by an AI or not ---
        // The player can commandeer ANY car. An AI-driven car counts (we eject its
        // agent below); only a car already holding another *player* is skipped.
        Entity best;
        Real bestD2 = enterRadius * enterRadius;
        ctx.world.each<Transform, Vehicle>([&](Entity e, Transform& t, Vehicle& v) {
            if (v.driver.valid()) return;   // a (player) driver already has it
            Real d2 = (t.position - playerPos).lengthSquared();
            if (d2 <= bestD2) { bestD2 = d2; best = e; }
        });
        if (best.valid()) {
            // Kick out the AI: removing AgentDriver stops the brain controlling the
            // car (the CitySim bridge sees the component gone and frees the agent),
            // so there's ever only ONE driver — now the player.
            if (ctx.world.has<AgentDriver>(best)) ctx.world.remove<AgentDriver>(best);
            ctx.world.get<Vehicle>(best)->driver = player;
            cameras.setFollowTarget(best);
            ctx.world.add<InVehicle>(player, InVehicle{best});   // structural
        }
    }
}

void VehicleSystem::update(FrameContext& ctx) {
    handleEnterExit(ctx);

    // Flip the car the player is currently driving back upright.
    if (ctx.actions.pressed("vehicle_flip")) {
        ctx.world.each<ControlledBy, InVehicle>(
            [&](Entity, ControlledBy&, InVehicle& iv) {
                if (!iv.vehicle.valid() || !ctx.world.has<Vehicle>(iv.vehicle)) return;
                Vehicle* v = ctx.world.get<Vehicle>(iv.vehicle);
                if (v->vehicleId != PhysicsWorld::INVALID_VEHICLE)
                    physicsSys.physicsWorld().resetVehicleUpright(v->vehicleId);
            });
    }

    // Toggle the driven car's lights.
    if (ctx.actions.pressed("vehicle_lights")) {
        ctx.world.each<ControlledBy, InVehicle>(
            [&](Entity, ControlledBy&, InVehicle& iv) {
                if (iv.vehicle.valid() && ctx.world.has<Vehicle>(iv.vehicle))
                    ctx.world.get<Vehicle>(iv.vehicle)->lightsOn =
                        !ctx.world.get<Vehicle>(iv.vehicle)->lightsOn;
            });
    }

    // Debug: drop a fresh car in front of the player.
    if (ctx.actions.pressed("spawn_vehicle")) spawnInFront(ctx);
}

void VehicleSystem::fixedUpdate(FrameContext& ctx) {
    createVehicles(ctx);
    driveVehicles(ctx);
    writeBack(ctx);
}

}  // namespace engine
