#include "vehicle_system.h"

#include "../components.h"
#include "../asset_manager.h"
#include "../mesh_builder.h"
#include "physics_system.h"
#include "camera_system.h"

#include <vector>

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
    }
}

void VehicleSystem::driveVehicles(FrameContext& ctx) {
    Real throttle = ctx.actions.axis("drive_throttle");
    Real steer = ctx.actions.axis("drive_steer");
    Real brake = ctx.actions.held("drive_brake") ? 1.0 : 0.0;
    Real hand = ctx.actions.held("drive_handbrake") ? 1.0 : 0.0;

    PhysicsWorld& pw = physicsSys.physicsWorld();
    ctx.world.each<Vehicle>([&](Entity, Vehicle& v) {
        if (v.vehicleId == PhysicsWorld::INVALID_VEHICLE) return;
        if (v.driver.valid()) {
            v.throttle = throttle;
            v.steer = steer;
            v.brake = brake;
            v.handBrake = hand;
        } else {
            // Parked: hold the brake so it doesn't creep on a slope.
            v.throttle = 0;
            v.steer = 0;
            v.brake = 1.0;
            v.handBrake = 0;
        }
        pw.setVehicleInput(v.vehicleId, v.throttle, v.steer, v.brake, v.handBrake);
    });
}

void VehicleSystem::writeBack(FrameContext& ctx) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    ctx.world.each<Transform, PrevTransform, Vehicle>(
        [&](Entity, Transform& t, PrevTransform& prev, Vehicle& v) {
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
        // --- get in: nearest unoccupied car within reach ---
        Entity best;
        Real bestD2 = enterRadius * enterRadius;
        ctx.world.each<Transform, Vehicle>([&](Entity e, Transform& t, Vehicle& v) {
            if (v.driver.valid()) return;
            Real d2 = (t.position - playerPos).lengthSquared();
            if (d2 <= bestD2) { bestD2 = d2; best = e; }
        });
        if (best.valid()) {
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
}

void VehicleSystem::fixedUpdate(FrameContext& ctx) {
    createVehicles(ctx);
    driveVehicles(ctx);
    writeBack(ctx);
}

}  // namespace engine
