#include "vehicle_system.h"
#include "../interact_broker.h"
#include "../interaction.h"   // Seated
#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif
#include "../vehicle_steering.h"

#include "../vehicle_lamps.h"
#include "../vehicle_audio.h"
#include "../audio/sfx.h"
#include "../particles.h"
#include "../procgen/vehicle/occupant.h"
#include "../procgen/person.h"

#include "../components.h"
#include "../player_pose.h"
#include "../asset_manager.h"
#include "../mesh_builder.h"
#include "physics_system.h"
#include "camera_system.h"
#include "player_system.h"   // LimpRequest: a rider thrown off
#include "../../log.h"

#include <algorithm>
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
    onSpawn_ = ctx.events.listen<VehicleSpawnRequest>([this](const VehicleSpawnRequest& r) {
        spawnPending_ = true;
        spawnRecipe_ = r.recipe;
    });
    answers_.bind(ctx.commands, "VehicleSystem", "vehicle?");
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

    // THE brake pedal (S stays plain reverse throttle — Glenn's call). B is
    // the conventional pad brake, like Y for get-in/out below.
    ctx.actions.bindButton("drive_brake", KeyCode::Space);
    ctx.actions.bindButton("drive_brake", GamepadButton::B);
    ctx.actions.setActionContext("drive_brake", engine::InputContext::InVehicle);
    ctx.actions.bindButton("drive_handbrake", KeyCode::LeftControl);
    // RB on the pad: the drift's handbrake had no button there (LB / RB cycle the tools only on foot)
    ctx.actions.bindButton("drive_handbrake", GamepadButton::RightBumper);
    ctx.actions.setActionContext("drive_handbrake", engine::InputContext::InVehicle);

    // GETTING IN is the interaction broker's E like everything else (Glenn, 2026-10-02: "move getting into the
    // car to 'e' like everything else"): this system OFFERS the car in reach and, on the broker's command, presses
    // enter_vehicle (injected, so CityVehicleSystem promotes an ambient car the same frame this one seats you).
    // GETTING OUT is E / pad X again, read here while in the car (the broker stands down in a vehicle).
    ctx.actions.bindButton("vehicle_exit", KeyCode::E);
    ctx.actions.bindButton("vehicle_exit", GamepadButton::X);
    ctx.actions.setActionContext("vehicle_exit", engine::InputContext::InVehicle);

    // Recover a rolled car (keep heading, level it, lift it).
    ctx.actions.bindButton("vehicle_flip", KeyCode::T);
    ctx.actions.bindButton("vehicle_flip", GamepadButton::LeftBumper);
    ctx.actions.setActionContext("vehicle_flip", engine::InputContext::InVehicle);
    // DRIFT BOOST (ADR-0236): spend what the slides banked.
    ctx.actions.bindButton("vehicle_boost", KeyCode::LeftShift);
    ctx.actions.bindButton("vehicle_boost", GamepadButton::Y);
    ctx.actions.setActionContext("vehicle_boost", engine::InputContext::InVehicle);
    // Toggle the head/taillights.
    ctx.actions.bindButton("vehicle_lights", KeyCode::L);
    ctx.actions.setActionContext("vehicle_lights", engine::InputContext::InVehicle);
    // The horn: held, like a real one (L3 is the pad convention).
    ctx.actions.bindButton("vehicle_horn", KeyCode::H);
    ctx.actions.bindButton("vehicle_horn", GamepadButton::LeftThumb);
    ctx.actions.setActionContext("vehicle_horn", engine::InputContext::InVehicle);
    // The car picker: - and = cycle the drivable catalogue; N drops the pick.
    // NOT , and . -- those are DevControlSystem's sim slower/faster, and picking
    // a car three places along ran the world at 8x ("super speed mode") -- nor
    // [ and ], the spectator's. The boot log names any clash ([input] key clash).
    ctx.actions.bindButton("vehicle_pick_prev", KeyCode::Minus);
    ctx.actions.bindButton("vehicle_pick_next", KeyCode::Equal);
    // A part-time 4x4's transfer case: 2WD (rear) <-> 4WD (#41).
    ctx.actions.bindButton("drive_4wd", KeyCode::Z);
    ctx.actions.bindButton("drive_4wd", GamepadButton::DpadDown);
    ctx.actions.setActionContext("drive_4wd", engine::InputContext::InVehicle);
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
    loadCatalogue();
    for (std::size_t i = 0; i < catalogue_.size() && !wantRecipe_.empty(); ++i)
        if (catalogue_[i].recipe == wantRecipe_) pick_ = static_cast<int>(i);
    wantRecipe_.clear();
    std::string recipe = "sedan", label = "Sedan", drive = "awd";
    if (!catalogue_.empty()) {
        const Pick& p = catalogue_[static_cast<std::size_t>(pick_)];
        recipe = p.recipe;
        label = p.label;
        drive = p.drive;
    }
    VehicleSpec spec;
    if (loadVehicleSpec(vm, "return vehicle." + recipe + "(seed, {})",
                        static_cast<uint32_t>(++spawnCount_), spec, &err)) {
        spawnVehicle(ctx.world, ctx.assets, spec, spawn, yawDeg);
        panelText_ = "Dropped: " + label + "  -  " + driveLabel(drive);
        panelUntil_ = clock_ + 3.0;
        LOG_INFO << "[vehicle] " << panelText_;
    } else {
        LOG_WARN << "spawn_vehicle: " << err;
    }
#else
    (void)ctx;
#endif
}

void VehicleSystem::loadCatalogue() {
#ifdef RT_ENABLE_SCRIPTING
    if (catalogueTried_) return;
    catalogueTried_ = true;
    const std::string lib = loadScriptCode("vehicles.lua", "");
    if (lib.empty()) return;
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(""));
    std::string err;
    if (!vm.doString(lib, &err)) { LOG_WARN << "vehicles.lua: " << err; return; }
    std::vector<DrivableEntry> entries;
    if (!loadDrivableCatalogue(vm, entries, &err)) { LOG_WARN << "car picker: " << err; return; }
    for (const DrivableEntry& e : entries) catalogue_.push_back({e.recipe, e.label, e.drive});
    LOG_INFO << "[vehicle] car picker: " << catalogue_.size() << " drivable vehicles (, . to pick, N to drop)";
#endif
}

void VehicleSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (clock_ > panelUntil_ || panelText_.empty()) return;
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(ctx.windowWidth) * 0.5f, static_cast<float>(ctx.windowHeight) - 24.0f),
                            ImGuiCond_Always, ImVec2(0.5f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.6f);
    ImGui::Begin("##carpicker", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    ImGui::TextUnformatted(panelText_.c_str());
    ImGui::TextDisabled("-  =  pick      N  drop      Z  2WD/4WD");
    ImGui::End();
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
            // every car still has lamps. The car's LEFT is +X (forward +Z, up +Y,
            // right-handed — the lab-measured convention car_mesh.cpp documents).
            const Vec3& hx = v->config.chassisHalfExtent;
            const Real lx = hx.x - 0.30, ly = -hx.y * 0.15;
            for (int i = 0; i < 4; ++i) {
                const bool front = i < 2;
                const bool left = (i % 2) == 1;
                Vehicle::Lamp lamp;
                lamp.front = front;
                lamp.left = left;
                lamp.local = Vec3(left ? lx : -lx, ly,
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
            // The player is seated: host input drives it. S is plain reverse
            // throttle (Glenn's call — "S can stay reverse for now"); the
            // BRAKE is its own pedal: Space (Ctrl = handbrake). The wheel
            // goes through the keyboard assist (vehicle_steering.h): less
            // lock the faster the car goes, wound on at a finite rate — a
            // key is a square wave, and full lock at 90 km/h is a spin.
            const Quat q = pw.vehicleOrientation(v.vehicleId);
            const Vec3 fwd = q.rotate(Vec3(0, 0, 1));
            const Vec3 vel = pw.vehicleVelocity(v.vehicleId);
            const Real fwdSpeed = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z;
            in.throttle = throttle;
            in.steer = shapeSteer(v.steerApplied, steer, fwdSpeed, ctx.clock.fixedStep());
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
        v.steer = v.driver.valid() ? steer : in.steer;   // the driver's wheel, for the indicators
        v.steerApplied = in.steer;
        v.brake = in.brake;
        v.handBrake = in.handBrake;
        pw.setVehicleInput(v.vehicleId, in.throttle, in.steer, in.brake, in.handBrake);
    });
}

void VehicleSystem::writeBack(FrameContext& ctx) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    // Headlight cones (WS3): collect candidate SpotLight pairs from every
    // vehicle whose headlights are lit this step, then keep the nearest few
    // under the shared 32-light budget (player-driven car always first).
    // Staged into lighting.vehicleSpots; RenderSystem merges on a copy.
    struct ConeCand {
        Real priority;   // 0 = player (wins), else camera distance
        SpotLight left, right;
    };
    std::vector<ConeCand> cones;
    const Vec3 camPos = ctx.view.camera.position;
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
                motion, v.speed, v.prevSpeed,
                lampsDark(ctx.view.lighting.solarElevation),
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

            // Headlight CONES (WS3, the ADR-0065 deferred row): lit headlights
            // cast real SpotLights on the road, not just glowing lenses. Beam
            // from each front lamp housing, pitched down ~8 deg so the pool
            // lands ahead of the bumper. Unshadowed (sun-only shadow maps) —
            // cones leak through walls; accepted v1, noted in the debt row.
            if (lit.head) {
                ConeCand cand;
                cand.priority = v.driver.valid()
                                    ? Real(0)
                                    : (t.position - camPos).length();
                const Vec3 fwd = t.orientation.rotate(Vec3(0, 0, 1));
                const Vec3 beamDir = normalize(fwd + Vec3(0, -0.14, 0));
                const Vec3 warm(1.0, 0.95, 0.82);
                int frontSeen = 0;
                for (const Vehicle::Lamp& lamp : v.lamps) {
                    if (!lamp.front) continue;
                    const Vec3 at =
                        t.position + t.orientation.rotate(lamp.local) +
                        fwd * 0.25;
                    SpotLight beam(at, beamDir, warm, /*intensity=*/22.0f,
                                   /*inner=*/0.32f, /*outer=*/0.55f);
                    beam.range = 45.0f;
                    (frontSeen == 0 ? cand.left : cand.right) = beam;
                    if (++frontSeen == 2) break;
                }
                if (frontSeen == 2) cones.push_back(std::move(cand));
            }

            // Driver capsule: in the seat when occupied — by a seated PLAYER or by
            // an AI brain (AgentDriver) — stowed far below otherwise. Seeing the
            // capsule is the proof that an agent is IN the car, not that the car
            // moves on its own (ADR-0062).
            // (the PLAYER at the wheel is their own jointed body, posed in the seat: PlayerBodySystem, ADR-0252)
            bool occupied = !v.driver.valid() && ctx.world.has<AgentDriver>(e);
            if (v.driverModel.valid() && ctx.world.alive(v.driverModel)) {
                if (Transform* dt = ctx.world.get<Transform>(v.driverModel)) {
                    if (PrevTransform* dp = ctx.world.get<PrevTransform>(v.driverModel))
                        dp->value = *dt;
                    // The recipe's own hip point when it has one — the occupant
                    // mesh's origin IS its hip, so this lines the two up exactly
                    // instead of guessing an offset off the chassis box.
                    const Vec3 seat = v.hipPoint();
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

    // Publish the nearest headlight cones (player first: priority 0). Three
    // vehicles' worth = 6 spot slots — with the sun/moon, 14 street lamps and
    // authored lights that stays comfortably inside the 32-light array.
    auto& spots = ctx.view.lighting.vehicleSpots;
    spots.clear();
    constexpr size_t kMaxConeVehicles = 3;
    if (cones.size() > kMaxConeVehicles) {
        std::nth_element(cones.begin(), cones.begin() + kMaxConeVehicles,
                         cones.end(), [](const ConeCand& a, const ConeCand& b) {
                             return a.priority < b.priority;
                         });
        cones.resize(kMaxConeVehicles);
    }
    for (const ConeCand& c : cones) {
        spots.push_back(c.left);
        spots.push_back(c.right);
    }
}

void VehicleSystem::handleEnterExit(FrameContext& ctx) {
    // The broker's command: a car was focused and E pressed -> press enter_vehicle for next frame.
    {
        Entity p;
        ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform&, ControlledBy&) { if (!p.valid()) p = e; });
        InteractCommand cmd;
        if (p.valid() && takeInteractCommand(ctx.world, p, "vehicle", cmd)) {
            ctx.actions.injectPress("enter_vehicle");
            LOG_INFO << "[vehicle] E: getting in";
        }
        // The OFFER: the nearest unoccupied car in reach (an ambient city car is CityVehicleSystem's offer).
        if (p.valid() && !ctx.world.has<InVehicle>(p) && !ctx.world.has<Seated>(p)) {
            const Vec3 pp = ctx.world.get<Transform>(p)->position;
            Entity best;
            Vec3 at;
            Real bestD2 = enterRadius * enterRadius;
            ctx.world.each<Transform, Vehicle>([&](Entity e, Transform& t, Vehicle& v) {
                if (v.driver.valid()) return;
                const Real dx = t.position.x - pp.x, dz = t.position.z - pp.z;
                if (dx * dx + dz * dz <= bestD2) { bestD2 = dx * dx + dz * dz; best = e; at = t.position; }
            });
            if (best.valid()) {
                InteractOffer o;
                o.provider = "vehicle";
                o.entity = best;
                o.name = "car";
                o.tap = "get in";
                o.anchor = at;   // the chassis centre: within a floor of the feet (the broker drops other floors)
                o.reach = enterRadius;
                o.needsSight = false;
                offerInteraction(ctx.world, p, o);
            }
        }
    }
    if (!ctx.actions.pressed("enter_vehicle") && !ctx.actions.pressed("vehicle_exit")) return;

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
            Vehicle& v = *ctx.world.get<Vehicle>(best);
            // A BIKE LYING ON ITS SIDE is picked up as you get on (Glenn, 2026-10-24: "if you ramp off the bike and
            // it falls over and you try and get onto it you just ragdoll"): it read as going down the moment you sat.
            const Transform& bt = *ctx.world.get<Transform>(best);
            if (v.config.twoWheeler && v.vehicleId != PhysicsWorld::INVALID_VEHICLE &&
                bt.orientation.rotate(Vec3(0, 1, 0)).y < std::cos(v.config.maxLeanDegrees * 0.0174533))
                physicsSys.physicsWorld().resetVehicleUpright(v.vehicleId, 0.3);
            bikeUpright_ = false;
            jolt_.reset();
            sway_.reset();
            haveSeatVel_ = false;
            v.driver = player;
            cameras.setFollowTarget(best);
            // a bike's chase sits closer than a car's (the car's 8 m left a motorcycle a speck)
            if (ctx.world.get<Vehicle>(best)->config.twoWheeler) cameras.setFollowFraming(4.0, 1.0);
            else cameras.setFollowFraming(8.0, 1.2);
            ctx.world.add<InVehicle>(player, InVehicle{best});   // structural
        }
    }
}

// THE PLAYER'S SEAT, each fixed step (ADR-0252). Its JOLT -- how far its velocity changed over a moment -- against what
// the seat can take: an open one (a bike's, a moped's, a convertible's) past it throws the rider off, limp at the speed
// the vehicle had; a closed car keeps them in. A two-wheeler rolled past its lean (it is going down) puts its rider off
// too. Whatever stays in the seat sways in it: the torso and the head thrown against the change and pulled back, drawn
// by PlayerBodySystem from the pose written here. The ragdoll is PlayerSystem's: this hands it the velocity in a
// LimpRequest.
void VehicleSystem::stepSeat(FrameContext& ctx) {
    Entity player, ride;
    ctx.world.each<ControlledBy, InVehicle>([&](Entity e, ControlledBy&, InVehicle& iv) {
        if (!player.valid()) { player = e; ride = iv.vehicle; }
    });
    Vehicle* v = player.valid() && ride.valid() && ctx.world.alive(ride) ? ctx.world.get<Vehicle>(ride) : nullptr;
    const Transform* rt = v ? ctx.world.get<Transform>(ride) : nullptr;
    if (!v || !rt || v->vehicleId == PhysicsWorld::INVALID_VEHICLE) return;
    const Real dt = ctx.clock.fixedStep();
    const Vec3 vel = physicsSys.physicsWorld().vehicleVelocity(v->vehicleId);
    Vec3 before;
    const Real jolt = jolt_.step(vel, dt, &before);
    // the sway: thrown against this step's change, in the vehicle's own frame
    const Vec3 dv = haveSeatVel_ ? rt->orientation.conjugate().rotate(vel - seatVel_) : Vec3(0, 0, 0);
    seatVel_ = vel;
    haveSeatVel_ = true;
    sway_.step(dv, dt);
    // a bicycle's cranks go round while it is pedalled (the wheel's turn, geared down)
    if (v->seat.pedals && v->throttle > 0.05) crank_ += v->speed * dt / (6.2832 * 0.34 * 2.6);
    if (PlayerPose* pose = ctx.world.get<PlayerPose>(player)) {
        pose->mode = PlayerPose::Mode::Drive;
        pose->handsFree = 0;
        pose->seatLean = sway_.lean;
        pose->seatNod = sway_.nod;
        pose->steer = v->steerApplied;
        pose->crank = crank_;
        pose->velocity = vel;
    }

    const Real tilt = std::acos(std::clamp(rt->orientation.rotate(Vec3(0, 1, 0)).y, Real(-1), Real(1))) * 57.29578;
    bool down = false;
    if (v->config.twoWheeler) {
        if (tilt < v->config.maxLeanDegrees) bikeUpright_ = true;
        down = bikeUpright_ && tilt > v->config.maxLeanDegrees + 10.0 && v->speed < 25.0;
    }
    const bool hit = jolt > ejectTolerance(v->config.twoWheeler, v->openSeat, ctx.settings.getBool("vehicle.windscreen", true));
    if (!down && !hit) return;
    const Vec3 thrown = hit ? ejectThrow(before, true) : ejectThrow(vel, false);
    jolt_.reset();
    v->driver = Entity{};
    if (CharacterController* cc = ctx.world.get<CharacterController>(player)) {
        // off the seat and clear of the body: the capsule's feet just over its top (a convertible's doors are a
        // box round the seat)
        Vec3 at = rt->position + rt->orientation.rotate(v->hipPoint());
        at.y = std::max(at.y + 0.9, rt->position.y + v->config.chassisHalfExtent.y + 0.95);
        physicsSys.physicsWorld().setCharacterPosition(cc->characterId, at);
        if (Transform* pt = ctx.world.get<Transform>(player)) pt->position = at;
    }
    cameras.clearFollowTarget();
    ctx.world.remove<InVehicle>(player);
    LimpRequest limp;
    limp.thrown = true;
    limp.velocity = thrown;
    ctx.events.publish(limp);
    LOG_INFO << "[vehicle] thrown off: " << (hit ? "a crash" : "went down") << ", jolt " << jolt << " m/s from "
             << before.length() * 3.6 << " km/h";
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

    // Shift a part-time 4x4 in or out of four-wheel drive.
    if (ctx.actions.pressed("drive_4wd")) {
        ctx.world.each<ControlledBy, InVehicle>(
            [&](Entity, ControlledBy&, InVehicle& iv) {
                Vehicle* v = iv.vehicle.valid() ? ctx.world.get<Vehicle>(iv.vehicle) : nullptr;
                if (!v || !v->partTime4wd || v->vehicleId == PhysicsWorld::INVALID_VEHICLE) return;
                v->fourWheel = !v->fourWheel;
                physicsSys.physicsWorld().setVehicleFrontDriveShare(v->vehicleId, v->fourWheel ? 0.5 : 0.0);
                LOG_INFO << "[vehicle] " << (v->fourWheel ? "4WD engaged" : "2WD (rear)");
                panelText_ = v->fourWheel ? "4WD ENGAGED" : "2WD (rear)";
                panelUntil_ = clock_ + 2.5;
            });
    }

    // The car picker.
    clock_ += ctx.frameDelta;
    const int step = ctx.actions.pressed("vehicle_pick_next") ? 1 : ctx.actions.pressed("vehicle_pick_prev") ? -1 : 0;
    if (step != 0) {
        loadCatalogue();
        if (!catalogue_.empty()) {
            const int n = static_cast<int>(catalogue_.size());
            pick_ = ((pick_ + step) % n + n) % n;
            const Pick& p = catalogue_[static_cast<std::size_t>(pick_)];
            panelText_ = std::to_string(pick_ + 1) + "/" + std::to_string(n) + "  " + p.label + "  -  " + driveLabel(p.drive);
            panelUntil_ = clock_ + 4.0;
            LOG_INFO << "[vehicle] pick " << panelText_;
        }
    }
    // Drop the picked car in front of the player.
    if (ctx.actions.pressed("spawn_vehicle") || spawnPending_) {
        // `vehicle_spawn <recipe>`: pick that one first (the catalogue's recipe names: motorcycle, moped, ...)
        wantRecipe_ = spawnPending_ ? spawnRecipe_ : std::string();
        spawnPending_ = false;
        spawnInFront(ctx);
    }

    // THE DRIFT is the player's car's (ADR-0236): on while driven by the player, off for everyone else; Shift
    // spends the boost a slide banked.
    {
        PhysicsWorld& pw = physicsSys.physicsWorld();
        PhysicsWorld::VehicleId mine = PhysicsWorld::INVALID_VEHICLE;
        ctx.world.each<ControlledBy, InVehicle>([&](Entity, ControlledBy&, InVehicle& iv) {
            if (const Vehicle* v = iv.vehicle.valid() ? ctx.world.get<Vehicle>(iv.vehicle) : nullptr)
                if (!v->config.twoWheeler) mine = v->vehicleId;
        });
        if (mine != driftCar_) {
            if (driftCar_ != PhysicsWorld::INVALID_VEHICLE) pw.setVehicleDriftEnabled(driftCar_, false);
            if (mine != PhysicsWorld::INVALID_VEHICLE) pw.setVehicleDriftEnabled(mine, true);
            driftCar_ = mine;
        }
        if (mine != PhysicsWorld::INVALID_VEHICLE) {
            if (ctx.actions.pressed("vehicle_boost")) pw.boostVehicle(mine);
            const PhysicsWorld::DriftState d = pw.vehicleDrift(mine);
            if (d.drifting && !wasDrifting_) LOG_INFO << "[vehicle] drift";
            if (!d.drifting && wasDrifting_) LOG_INFO << "[vehicle] drift held " << lastDriftTime_ << " s";
            wasDrifting_ = d.drifting;
            if (d.drifting) lastDriftTime_ = d.time;
        }
    }
    // The player's car for the control channel's `vehicle?`.
    std::string telem = "none";
    ctx.world.each<ControlledBy, InVehicle>([&](Entity, ControlledBy&, InVehicle& iv) {
        const Vehicle* v = iv.vehicle.valid() ? ctx.world.get<Vehicle>(iv.vehicle) : nullptr;
        const Transform* t = iv.vehicle.valid() ? ctx.world.get<Transform>(iv.vehicle) : nullptr;
        if (!v || !t || v->vehicleId == PhysicsWorld::INVALID_VEHICLE) return;
        const auto tel = physicsSys.physicsWorld().vehicleTelemetry(v->vehicleId);
        const Vec3 right = t->orientation.rotate(Vec3(1, 0, 0)), fwd = t->orientation.rotate(Vec3(0, 0, 1));
        const Real roll = std::asin(std::clamp(right.y, Real(-1), Real(1))) * 57.2958;
        const Real pitch = std::asin(std::clamp(fwd.y, Real(-1), Real(1))) * 57.2958;
        const Real yaw = std::atan2(fwd.x, fwd.z) * 57.2958;
        char b[256];
        std::snprintf(b, sizeof(b), "kmh %.1f rpm %.0f gear %d drive %s pos %.2f %.2f %.2f roll %.1f pitch %.1f yaw %.1f",
                      tel.speed * 3.6, tel.rpm, tel.gear, v->partTime4wd ? (v->fourWheel ? "4wd" : "2wd") : "fixed",
                      t->position.x, t->position.y, t->position.z, roll, pitch, yaw);
        telem = b;
        const PhysicsWorld::DriftState d = physicsSys.physicsWorld().vehicleDrift(v->vehicleId);
        char dr[96];
        std::snprintf(dr, sizeof(dr), " drift %d slip %.0f boost %.2f%s", d.drifting ? 1 : 0, d.slipDeg, d.boost,
                      d.boosting > 0 ? " BOOSTING" : "");
        telem += dr;
    });
    answers_.set("vehicle?", telem + " timescale " + std::to_string(ctx.clock.timeScale()));

    updateHorn(ctx);
    updateEngines(ctx);
    measureSlide(ctx);
    updateTyres(ctx);
}

void VehicleSystem::updateEngines(FrameContext& ctx) {
    if (!ctx.audio.ready()) return;

    // Every RUNNING physical car — the player's and any the AI drives — near
    // enough to hear. A driverless car is silent (engineSoundFor's contract),
    // so a parked street stays parked.
    struct Candidate {
        Entity car;
        Vec3 pos;
        EngineSound sound;
        Real rev;      // for the per-car rev hunt (wobble depth)
        Real dist2;
    };
    std::vector<Candidate> heard;
    const Vec3 ear = ctx.view.camera.position;
    ctx.world.each<Transform, Vehicle>([&](Entity e, Transform& t, Vehicle& v) {
        const bool running = v.driver.valid() || ctx.world.has<AgentDriver>(e);
        if (!running || v.pedalled) return;   // a bicycle has legs, not an engine
        const Real d2 = (t.position - ear).lengthSquared();
        if (d2 > kEngineAudibleRange * kEngineAudibleRange) return;
        heard.push_back({e, t.position,
                         engineSoundFor(v.speed, v.throttle, true),
                         engineRevFraction(v.speed), d2});
    });
    // Nearest first: when there are more running cars than voices, the ones
    // you can actually pick out win the slots.
    std::sort(heard.begin(), heard.end(),
              [](const Candidate& a, const Candidate& b) {
                  return a.dist2 < b.dist2;
              });
    if (heard.size() > static_cast<std::size_t>(kMaxEngineVoices))
        heard.resize(static_cast<std::size_t>(kMaxEngineVoices));

    for (EngineVoice& ev : engineVoices) ev.wanted = false;

    for (const Candidate& c : heard) {
        EngineVoice* slot = nullptr;
        for (EngineVoice& ev : engineVoices)
            if (ev.car == c.car) { slot = &ev; break; }
        if (!slot) {
            if (!engineClip.valid()) {
                // Procedural, synthesized once (ADR-0071): one loop-clean
                // period, pitch-shifted per car — no sound file, no per-car
                // sample bank.
                std::vector<float> pcm = sfx::engine(ctx.audio.sampleRate(), 1);
                engineClip = ctx.audio.createClip(pcm.data(), pcm.size(), 1,
                                                  ctx.audio.sampleRate());
            }
            if (!engineClip.valid()) return;
            AudioPlayParams params;
            params.loop = true;
            params.volume = 0.0f;   // ramped in below — no click on start
            params.pitch = c.sound.pitch;
            EngineVoice ev;
            ev.car = c.car;
            ev.voice = ctx.audio.playAt(engineClip, c.pos, kEngineAudibleRange,
                                        params);
            if (!ev.voice.valid()) continue;
            engineVoices.push_back(ev);
            slot = &engineVoices.back();
        }
        slot->wanted = true;
        // Ease the gain (a car coming into earshot fades up rather than
        // snapping on); pitch tracks directly, so a gear change reads as the
        // shift it is instead of a smear.
        slot->gain += (c.sound.gain - slot->gain) *
                      std::min(1.0f, 8.0f * static_cast<float>(ctx.frameDelta));
        // Hunt the revs a touch (per car, so two cars never wobble in step) —
        // a perfectly steady playback rate is what makes a loop read as a hum.
        const float wobble =
            engineWobble(ctx.clock.simulatedTime(),
                         static_cast<Real>(c.car.index % 17) * Real(0.37), c.rev);
        ctx.audio.setVoiceVolume(slot->voice, slot->gain);
        ctx.audio.setVoicePitch(slot->voice, c.sound.pitch * wobble);
        ctx.audio.setVoicePosition(slot->voice, c.pos);
    }

    // Fade out and reclaim: cars that drove away, lost their driver, or lost
    // their slot to a nearer one. A voice the engine already dropped (level
    // swap -> stopAll) is forgotten so the car can be voiced again.
    for (std::size_t i = engineVoices.size(); i-- > 0;) {
        EngineVoice& ev = engineVoices[i];
        if (!ctx.audio.isPlaying(ev.voice)) {
            engineVoices.erase(engineVoices.begin() +
                               static_cast<std::ptrdiff_t>(i));
            continue;
        }
        if (ev.wanted) continue;
        ev.gain += (0.0f - ev.gain) *
                   std::min(1.0f, 6.0f * static_cast<float>(ctx.frameDelta));
        if (ev.gain < 0.01f) {
            ctx.audio.stop(ev.voice);
            engineVoices.erase(engineVoices.begin() +
                               static_cast<std::ptrdiff_t>(i));
        } else {
            ctx.audio.setVoiceVolume(ev.voice, ev.gain);
        }
    }
}

// HOW HARD THE PLAYER'S TYRES SLIDE, 0..1, onto Vehicle::slide: sideways (the slip angle -- a drift) or spinning /
// locked along (a burnout, a stop). The squeal's loudness and ParticleSystem's tyre smoke both read it.
void VehicleSystem::measureSlide(FrameContext& ctx) {
    for (Entity& s : smoke_)
        if (!s.valid() || !ctx.world.alive(s)) {
            s = ctx.world.create();
            ctx.world.add<Transform>(s, Transform{});
            ParticleEmitter pe;
            pe.effect = "smoke";
            pe.halfExtent = Vec3(0.15, 0.05, 0.15);
            ctx.world.add<ParticleEmitter>(s, pe);
        }
    auto smokeOff = [&]() {
        for (Entity s : smoke_)
            if (ParticleEmitter* pe = ctx.world.get<ParticleEmitter>(s)) pe->rate = 0;
    };
    Entity car;
    ctx.world.each<ControlledBy, InVehicle>([&](Entity, ControlledBy&, InVehicle& iv) {
        if (!car.valid()) car = iv.vehicle;
    });
    Vehicle* v = car.valid() && ctx.world.alive(car) ? ctx.world.get<Vehicle>(car) : nullptr;
    if (!v || v->vehicleId == PhysicsWorld::INVALID_VEHICLE) return smokeOff();
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const PhysicsWorld::VehicleTelemetry tel = pw.vehicleTelemetry(v->vehicleId);
    auto smooth = [](Real a, Real b, Real x) {
        const Real u = std::clamp((x - a) / (b - a), Real(0), Real(1));
        return u * u * (3 - 2 * u);
    };
    Real along = 0;
    for (std::size_t i = 0; i < tel.wheelSlip.size(); ++i)
        if (i < tel.wheelContact.size() && tel.wheelContact[i]) along = std::max(along, std::fabs(tel.wheelSlip[i]));
    // ONLY A DRIFT OR A HARD STOP (Glenn, 2026-10-24: "still kinda loud. We should use it when we brake hard or
    // drift"): a slide the drift has hold of, by its slip angle; or the brake pressed hard at speed, more as the wheels
    // lock. A brisk corner, a wheelspin pull-away, a bump: silent.
    const PhysicsWorld::DriftState drift = pw.vehicleDrift(v->vehicleId);
    const Real speed = std::fabs(v->speed);
    const Real drifting = drift.drifting ? smooth(6.0, 26.0, std::fabs(drift.slipDeg)) : Real(0);
    const Real braking = smooth(0.55, 0.95, std::max(v->brake, v->handBrake)) * (0.6 + 0.4 * smooth(0.2, 0.7, along));
    const Real fast = smooth(4.0, 11.0, speed);
    v->slide = v->pedalled ? Real(0) : std::max(drifting, braking) * fast;
    // the smoke: a drift's in full, a hard stop's a wisp (Glenn: "The braking smoke is a little heavy")
    const Real smoky = v->pedalled ? Real(0) : std::max(drifting, braking * Real(0.3)) * fast;
    // SMOKE off the rear tyres while they slide (Glenn: "dust kick up when you ... drift around a corner")
    const Transform* ct = ctx.world.get<Transform>(car);
    if (!ct) return smokeOff();
    const Vec3 ahead = ct->orientation.rotate(Vec3(0, 0, 1));
    int k = 0;
    for (const PhysicsWorld::VehicleWheel& w : v->config.wheels) {
        if (w.position.z > 0 || k >= 2) continue;   // the rear ones
        if (Transform* st = ctx.world.get<Transform>(smoke_[k]))
            st->position = ct->position + ct->orientation.rotate(w.position) - Vec3(0, w.radius * 0.8, 0);
        if (ParticleEmitter* pe = ctx.world.get<ParticleEmitter>(smoke_[k])) {
            pe->rate = smoky > 0.05 ? 40.0 * smoky : 0.0;
            pe->velocity = ahead * (v->speed * 0.2);
        }
        ++k;
    }
    for (; k < 2; ++k)
        if (ParticleEmitter* pe = ctx.world.get<ParticleEmitter>(smoke_[k])) pe->rate = 0;
}

void VehicleSystem::updateTyres(FrameContext& ctx) {
    if (!ctx.audio.ready()) return;
    if (!tyreClips_) {
        const uint32_t r = ctx.audio.sampleRate();
        auto make = [&](const std::vector<float>& pcm) { return ctx.audio.createClip(pcm.data(), pcm.size(), 1, r); };
        squeal_.clip = make(sfx::tyreSqueal(r, 1));
        freewheel_.clip = make(sfx::freewheel(r, 1));
        roll_.clip = make(sfx::tyreRoll(r, 1));
        tyreClips_ = true;
    }
    const float dt = static_cast<float>(std::min(ctx.frameDelta, 0.1));
    auto drive = [&](TyreLoop& l, float target, float pitch, const Vec3& at, float rate) {
        l.gain += (target - l.gain) * std::min(1.0f, rate * dt);
        if (!l.voice.valid() || !ctx.audio.isPlaying(l.voice)) {
            l.voice = AudioVoiceHandle{};
            if (l.gain < 0.005f || !l.clip.valid()) return;
            AudioPlayParams p;
            p.loop = true;
            p.volume = 0.0f;
            p.pitch = pitch;
            l.voice = ctx.audio.playAt(l.clip, at, 70.0, p);
            if (!l.voice.valid()) return;
        }
        if (l.gain < 0.003f && target <= 0.0f) {
            ctx.audio.stop(l.voice);
            l.voice = AudioVoiceHandle{};
            l.gain = 0.0f;
            return;
        }
        ctx.audio.setVoiceVolume(l.voice, l.gain);
        ctx.audio.setVoicePitch(l.voice, std::max(0.05f, pitch));
        ctx.audio.setVoicePosition(l.voice, at);
    };
    // the player's ride, if any
    Entity car;
    ctx.world.each<ControlledBy, InVehicle>([&](Entity, ControlledBy&, InVehicle& iv) {
        if (!car.valid()) car = iv.vehicle;
    });
    const Vehicle* v = car.valid() && ctx.world.alive(car) ? ctx.world.get<Vehicle>(car) : nullptr;
    const Transform* t = v ? ctx.world.get<Transform>(car) : nullptr;
    if (!v || !t || v->vehicleId == PhysicsWorld::INVALID_VEHICLE) {
        const Vec3 ear = ctx.view.camera.position;
        drive(squeal_, 0, 1, ear, 10);
        drive(freewheel_, 0, 1, ear, 10);
        drive(roll_, 0, 1, ear, 10);
        return;
    }
    const Real speed = std::fabs(v->speed);
    const Real slide = v->slide;   // (measureSlide)
    auto smooth = [](Real a, Real b, Real x) {
        const Real u = std::clamp((x - a) / (b - a), Real(0), Real(1));
        return u * u * (3 - 2 * u);
    };
    drive(squeal_, static_cast<float>(0.3 * slide), static_cast<float>(0.9 + 0.15 * slide + speed / 150.0),
          t->position, 9);
    // A BICYCLE: the freewheel's pawls ticking while it coasts (as fast as the back wheel turns), the tyres' hum
    const bool bike = v->pedalled;
    const Real r = v->config.wheels.empty() ? 0.34 : v->config.wheels.back().radius;
    const Real clicks = 24.0 * speed / (6.283185307179586 * r);   // 24 pawl engagements a turn
    const bool coasting = bike && speed > 0.8 && std::fabs(v->throttle) < 0.05;
    drive(freewheel_, coasting ? static_cast<float>(0.3 * smooth(0.8, 3.0, speed)) : 0.0f,
          static_cast<float>(std::clamp(clicks / 40.0, 0.25, 3.0)), t->position, 12);
    drive(roll_, bike ? static_cast<float>(0.45 * std::pow(std::clamp(speed / 9.0, 0.0, 1.0), 0.8)) : 0.0f,
          static_cast<float>(0.7 + speed / 14.0), t->position, 8);
}

void VehicleSystem::updateHorn(FrameContext& ctx) {
    if (!ctx.audio.ready()) return;

    // The car the player is driving (the horn is the seated player's voice —
    // parked cars and AI traffic don't honk from this path).
    Entity car;
    ctx.world.each<ControlledBy, InVehicle>(
        [&](Entity, ControlledBy&, InVehicle& iv) {
            if (!car.valid()) car = iv.vehicle;
        });
    const bool driving = car.valid() && ctx.world.alive(car) &&
                         ctx.world.has<Transform>(car);
    const bool wantHorn = driving && ctx.actions.held("vehicle_horn");

    if (wantHorn && !hornVoice.valid()) {
        if (!hornClip.valid()) {
            // Procedural, synthesized once (ADR-0071 — no sound file): a
            // loop-clean period sfx::horn documents, held by a looping voice.
            std::vector<float> pcm = sfx::horn(ctx.audio.sampleRate(), 1);
            hornClip = ctx.audio.createClip(pcm.data(), pcm.size(), 1,
                                            ctx.audio.sampleRate());
        }
        if (hornClip.valid()) {
            AudioPlayParams params;
            params.loop = true;
            params.volume = 0.0f;   // ramped in below — no click on press
            hornVoice = ctx.audio.playAt(
                hornClip, ctx.world.get<Transform>(car)->position, 60.0, params);
        }
    }
    if (!hornVoice.valid()) return;

    // Attack ~30 ms, release ~90 ms: an exponential ramp toward the target, so
    // both edges of the honk are clickless; the voice stops once inaudible.
    const float target = wantHorn ? 0.9f : 0.0f;
    const float rate = wantHorn ? 33.0f : 11.0f;
    hornGain += (target - hornGain) *
                std::min(1.0f, rate * static_cast<float>(ctx.frameDelta));
    if (!wantHorn && hornGain < 0.01f) {
        ctx.audio.stop(hornVoice);
        hornVoice = AudioVoiceHandle{};
        hornGain = 0.0f;
        return;
    }
    ctx.audio.setVoiceVolume(hornVoice, hornGain);
    if (driving)   // honk rides along with the car
        ctx.audio.setVoicePosition(hornVoice, ctx.world.get<Transform>(car)->position);
}

void VehicleSystem::fixedUpdate(FrameContext& ctx) {
    createVehicles(ctx);
    driveVehicles(ctx);
    writeBack(ctx);
    stepSeat(ctx);
}

}  // namespace engine
