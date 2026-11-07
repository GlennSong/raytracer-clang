#include "vehicle_system.h"
#include "../interact_broker.h"
#include "../vehicle_damage.h"
#include "../particles.h"
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
#include "../riding.h"   // rideStep: the driver rides the car like any rider
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

namespace {
// The driver's eye over the hip (Passenger::eyeAbove, sitting), as a seated passenger's.
constexpr Real kDriverEyeAboveHip = 0.74;
}  // namespace

void VehicleSystem::onStart(FrameContext& ctx) {
    onSpawn_ = ctx.events.listen<VehicleSpawnRequest>([this](const VehicleSpawnRequest& r) {
        spawnPending_ = true;
        spawnRecipe_ = r.recipe;
    });
    answers_.bind(ctx.commands, "VehicleSystem", "vehicle?");
    // THE TOUCHES (ADR-0353): every contact a drivable car's chassis makes, kept for the next step's damage pass
    onCollision_ = ctx.events.listen<Collision>([this](const Collision& c) {
        Hit h;
        h.at = c.position;
        h.normal = c.normal;
        h.speed = c.speed;
        h.slide = c.slide;
        h.persisted = c.persisted;
        h.broke = c.broke;
        h.car = c.a;
        h.other = c.b;
        hits_.push_back(h);
        if (c.b.valid()) {   // (either side may be the car; the pass sorts it out)
            Hit g = h;
            g.car = c.b;
            g.other = c.a;
            g.normal = c.normal * -1.0;
            hits_.push_back(g);
        }
    });
    if (ctx.commands != nullptr)
        panelsCmd_ = ctx.commands->bind("panels", "VehicleSystem", [this](const ControlCommand& cmd) -> std::string {
            if (cmd.args.empty() || (cmd.args[0] != "open" && cmd.args[0] != "shut" && cmd.args[0] != "knock"))
                return "err panels open|shut|knock [name]";
            panelsStaged_.push_back(cmd.args[0] + (cmd.args.size() >= 2 ? " " + cmd.args[1] : std::string()));
            return "ok staged " + panelsStaged_.back();
        });
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
    // NOT , and . -- they were DevControlSystem's sim slower/faster, and picking
    // a car three places along ran the world at 8x ("super speed mode"); sim speed
    // is unbound by default now, but the keys stay clear -- nor [ and ], the spectator's. The boot log names any clash ([input] key clash).
    ctx.actions.bindButton("vehicle_pick_prev", KeyCode::Minus);
    ctx.actions.bindButton("vehicle_pick_next", KeyCode::Equal);
    // A part-time 4x4's transfer case: 2WD (rear) <-> 4WD (#41).
    ctx.actions.bindButton("drive_4wd", KeyCode::Z);
    ctx.actions.bindButton("drive_4wd", GamepadButton::DpadDown);
    ctx.actions.setActionContext("drive_4wd", engine::InputContext::InVehicle);
    // Debug: spawn a fresh car in front of the player (always on solid ground,
    // since the player is standing on a collider).
    ctx.actions.bindButton("spawn_vehicle", KeyCode::N);
    // ...and M drops the catalogue's QUICK vehicle (the entry marked `quick`: the motorcycle) without cycling to it
    ctx.actions.bindButton("vehicle_quick", KeyCode::M);
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
    for (const DrivableEntry& e : entries) catalogue_.push_back({e.recipe, e.label, e.drive, e.quick});
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
    std::string keys = "-  =  pick      N  drop";
    for (const Pick& p : catalogue_)
        if (p.quick) { keys += "      M  " + p.label; break; }   // (the quick vehicle's own label, from the data)
    ImGui::TextDisabled("%s      Z  2WD/4WD", keys.c_str());
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

        // A driver (stowed until someone's aboard). The LAMPS are the body's own lenses, lit by the mask writeBack puts
        // on the body (ADR-0342); `lamps` keeps where they are, for the headlight cones.
        // A SEATED occupant, not a capsule. The capsule proved a car was
        // occupied; this shows who is driving it, posed from the same SAE manikin
        // the interior is packaged around so it fits the seat it sits in.
        if (!driverMesh.valid())
            driverMesh = ctx.assets.acquireMesh(buildSeatedOccupant(), "vehicle:driver");
        if (v->lamps.empty()) {
            // No marker data (hand-built or legacy body): the cones fall back to the corners. The car's LEFT is +X
            // (forward +Z, up +Y, right-handed — the lab-measured convention car_mesh.cpp documents).
            const Vec3& hx = v->config.chassisHalfExtent;
            const Real lx = hx.x - 0.30, ly = -hx.y * 0.15;
            for (int i = 0; i < 4; ++i) {
                Vehicle::Lamp lamp;
                lamp.front = i < 2;
                lamp.left = (i % 2) == 1;
                lamp.local = Vec3(lamp.left ? lx : -lx, ly, lamp.front ? hx.z + 0.05 : -hx.z - 0.05);
                v->lamps.push_back(lamp);
            }
        }

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
    starterCooldown_ = std::max(Real(0), starterCooldown_ - ctx.clock.fixedStep());
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
            // An AI brain drives it: turn what the brain WANTS (a lane frame + speed)
            // into the same pedal/wheel values through the one controller, so
            // this NPC car obeys identical physics to the player's.
            DriverState s;
            const Vec3 at = pw.vehiclePosition(v.vehicleId);
            s.pos = Vec2(at.x, at.z);
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
        if (v.engineDead) {   // (ADR-0353: the nose spent -- it steers and brakes, it will not drive)
            // the driver tries it: the starter turns it over and it will not catch (ADR-0355)
            if (v.driver.valid() && in.throttle > 0.3 && starterCooldown_ <= 0) {
                if (const Transform* ct = ctx.world.get<Transform>(e)) panelSound(ctx, ct->position, 4, 0.7f);
                starterCooldown_ = 1.8;
            }
            in.throttle = 0.0;
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
            // ...a part on a moving panel (ADR-0352) turns about its hinge: x -> q (x - pivot) + pivot
            for (std::size_t k = 0; k < v.bodyParts.size(); ++k) {
                const int pn = k < v.bodyPartPanel.size() ? v.bodyPartPanel[k] : -1;
                if (pn < 0 || pn >= static_cast<int>(v.panels.size()) || v.panels[static_cast<std::size_t>(pn)].angle == 0.0f) {
                    place(v.bodyParts[k], Vec3(0, 0, 0));
                    continue;
                }
                const Vehicle::Panel& panel = v.panels[static_cast<std::size_t>(pn)];
                const Quat q = panel.turn();
                place(v.bodyParts[k], panel.pivot - q.rotate(panel.pivot));
                if (Transform* ct = ctx.world.get<Transform>(v.bodyParts[k])) ct->orientation = t.orientation * q;
            }
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

            // THE BODY'S OWN LENSES light (ADR-0342): the mask rides its model's bottom row (Renderable::instanceData)
            if (Renderable* body = ctx.world.get<Renderable>(e)) {
                body->material.flags |= RenderMaterial::FLAG_LAMP_STATE;
                body->instanceData = static_cast<float>(lampMask(lit, blink));
            }
            // ...and a lens that rides a panel -- the SUV's tail lamps on its hatch (Glenn: "the lights should go with
            // it") -- lights from the panel's part the same way
            for (Entity part : v.bodyParts)
                if (Renderable* pr = ctx.world.get<Renderable>(part)) {
                    pr->material.flags |= RenderMaterial::FLAG_LAMP_STATE;
                    pr->instanceData = static_cast<float>(lampMask(lit, blink));
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

            // Keep a driver with no seat (no Passenger) glued to the chassis so they ride along; a seated one is
            // the riding component's (stepSeat -> rideStep puts them at the seat).
            if (v.driver.valid() && ctx.world.alive(v.driver) && !ctx.world.has<Passenger>(v.driver)) {
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
        if (p.valid() && takeInteractCommand(ctx.world, p, "vehicle_panel", cmd) && ctx.world.alive(cmd.entity)) {
            if (Vehicle* v = ctx.world.get<Vehicle>(cmd.entity); v && cmd.key < v->panels.size()) {
                Vehicle::Panel& panel = v->panels[static_cast<std::size_t>(cmd.key)];
                panel.target = panel.target > 0.0f ? 0.0f : panel.open;
                panel.hold = 0.0f;
                LOG_INFO << "[vehicle] E: " << (panel.target > 0.0f ? "opening " : "shutting ") << panel.name;
            }
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
            // THE HOOD AND THE TRUNK (ADR-0352): open or shut from beside them, on any parked car in reach
            ctx.world.each<Transform, Vehicle>([&](Entity e, Transform& t, Vehicle& v) {
                if (v.driver.valid()) return;
                for (std::size_t i = 0; i < v.panels.size(); ++i) {
                    const Vehicle::Panel& panel = v.panels[i];
                    if (panel.gone || panel.name.rfind("door_", 0) == 0) continue;
                    const Vec3 at = t.position + t.orientation.rotate(panel.centre);
                    const Real dx = at.x - pp.x, dz = at.z - pp.z;
                    if (dx * dx + dz * dz > 2.2 * 2.2) continue;
                    InteractOffer o;
                    o.provider = "vehicle_panel";
                    o.entity = e;
                    o.key = i;
                    o.name = panel.name;
                    o.tap = panel.target > 0.0f ? "close" : "open";
                    o.anchor = at;
                    o.reach = 2.2;
                    o.needsSight = false;
                    offerInteraction(ctx.world, p, o);
                }
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
        if (car.valid() && ctx.world.alive(car) && ctx.world.has<Vehicle>(car)) {
            Vehicle& out = *ctx.world.get<Vehicle>(car);
            out.driver = Entity{};
            out.swingOpen(out.driverDoor(), 0.9f);
        }
        if (car.valid() && ctx.world.has<Transform>(car)) {
            const Transform& ct = *ctx.world.get<Transform>(car);
            // Step out well CLEAR of the car: up (world up, so a flipped car still
            // ejects you upward, never trapping you under it) plus a flattened
            // sideways nudge. You drop onto your feet beside the car.
            // OUT OF THE DRIVER'S DOOR (Glenn: "when you get out of the car, it has to be on the side of the car you
            // entered to match the door"): beside the door, level with the seat, standing -- the driver sits on the
            // right (-X), the door that swings open is theirs. A car on its side or roof: up and out, as before.
            const Vehicle* vv = ctx.world.get<Vehicle>(car);
            const Vec3 hip = vv ? vv->hipPoint() : Vec3(-0.5, 0, 0);
            const Real sx = hip.x < 0 ? -1.0 : 1.0;
            Vec3 side = ct.orientation.rotate(Vec3(sx, 0, 0));
            side.y = 0;
            Real sl = side.length();
            side = sl > 1e-3 ? side * (1.0 / sl) : Vec3(sx, 0, 0);
            const bool upright = ct.orientation.rotate(Vec3(0, 1, 0)).y > 0.7;
            const Real halfW = vv ? vv->config.chassisHalfExtent.x : 0.9;
            Vec3 out = ct.position + Vec3(0, 2.5, 0) + side * 2.0;
            if (upright) {
                Vec3 along = ct.orientation.rotate(Vec3(0, 0, hip.z));
                along.y = 0;
                out = ct.position + along + side * (halfW + 0.65) + Vec3(0, 0.6, 0);
            }
            if (Transform* pt = ctx.world.get<Transform>(player)) pt->position = out;
            if (CharacterController* cc = ctx.world.get<CharacterController>(player))
                physicsSys.physicsWorld().setCharacterPosition(cc->characterId, out);
        }
        cameras.clearFollowTarget();
        ctx.world.remove<InVehicle>(player);   // structural — after the each() above
        if (ctx.world.has<Passenger>(player)) ctx.world.remove<Passenger>(player);   // (out of the seat)
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
            bikeDown_ = 0;
            v.driver = player;
            v.swingOpen(v.driverDoor(), 0.9f);   // the door opens for you and shuts behind you
            cameras.setFollowTarget(best);
            // a bike's chase sits closer than a car's (the car's 8 m left a motorcycle a speck)
            if (ctx.world.get<Vehicle>(best)->config.twoWheeler) cameras.setFollowFraming(4.0, 1.0);
            else cameras.setFollowFraming(8.0, 1.2);
            ctx.world.add<InVehicle>(player, InVehicle{best});   // structural
            // IN THE SEAT: a rider of the car (engine/riding.h), sat at its hip point; the chase rig frames the car,
            // so the view does not go round with it. stepSeat sets the carrier each step.
            Passenger seated;
            seated.sitting = true;
            seated.carSeat = true;
            seated.eyeAbove = kDriverEyeAboveHip;
            seated.hipBelow = kDriverEyeAboveHip - kEyeAboveCentre;
            seated.turnsView = false;
            if (Passenger* was = ctx.world.get<Passenger>(player)) *was = seated;
            else ctx.world.add<Passenger>(player, seated);
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
    Passenger* ps = ctx.world.get<Passenger>(player);
    if (!ps) return;
    const Real dt = ctx.clock.fixedStep();
    const Vec3 vel = physicsSys.physicsWorld().vehicleVelocity(v->vehicleId);
    // the driver rides the car: the carrier this step, the hip point in it -- the pin, and the sway and jolt the seat
    // throws the body into, are the riding component's (rideStep)
    ps->carrier = rt->matrix();
    ps->local = v->hipPoint();
    rideStep(ctx.world, &physicsSys.physicsWorld(), player, nullptr, dt);
    const Real jolt = ps->joltNow;
    const Vec3 before = ps->joltBefore;
    // a bicycle's cranks go round while it is pedalled (the wheel's turn, geared down)
    if (v->seat.pedals && v->throttle > 0.05) crank_ += v->speed * dt / (6.2832 * 0.34 * 2.6);
    if (PlayerPose* pose = ctx.world.get<PlayerPose>(player)) {
        pose->mode = PlayerPose::Mode::Drive;
        pose->handsFree = 0;
        pose->seatLean = ps->sway.lean;
        pose->seatNod = ps->sway.nod;
        pose->steer = v->steerApplied;
        pose->crank = crank_;
        pose->velocity = vel;
    }

    const Real tilt = std::acos(std::clamp(rt->orientation.rotate(Vec3(0, 1, 0)).y, Real(-1), Real(1))) * 57.29578;
    // a bike that has GONE DOWN puts you off: on its side (past 75 degrees) for 0.4 s, not leant hard into a turn past
    // its limit (that threw the rider out of a fast bend)
    bool down = false;
    if (v->config.twoWheeler) {
        if (tilt < v->config.maxLeanDegrees) bikeUpright_ = true;
        bikeDown_ = bikeUpright_ && tilt > 75.0 && v->speed < 25.0 ? bikeDown_ + dt : 0.0;
        down = bikeDown_ > 0.4;
    }
    EjectRule rule;
    rule.bike = ctx.settings.getDouble("vehicle.ejectBike", rule.bike);
    rule.open = ctx.settings.getDouble("vehicle.ejectOpen", rule.open);
    rule.car = ctx.settings.getDouble("vehicle.ejectCar", rule.car);
    const bool hit = jolt > ejectTolerance(v->config.twoWheeler, v->openSeat, ctx.settings.getBool("vehicle.windscreen", true), rule);
    if (!down && !hit) return;
    const Vec3 thrown = hit ? ejectThrow(before, true) : ejectThrow(vel, false);
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
    ctx.world.remove<Passenger>(player);   // (thrown out of the seat)
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
            [&](Entity e, ControlledBy&, InVehicle& iv) {
                if (!iv.vehicle.valid() || !ctx.world.has<Vehicle>(iv.vehicle)) return;
                Vehicle* v = ctx.world.get<Vehicle>(iv.vehicle);
                if (v->vehicleId == PhysicsWorld::INVALID_VEHICLE) return;
                physicsSys.physicsWorld().resetVehicleUpright(v->vehicleId);
                // the car was PLACED, not driven: a 1.5 m lift in one step read as a 90 m/s jolt and threw the driver
                // through the windscreen. A new frame for the seat, as on boarding (rideStep's jump).
                if (Passenger* p = ctx.world.get<Passenger>(e)) p->haveLast = false;
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
    const bool quick = ctx.actions.pressed("vehicle_quick");
    if (ctx.actions.pressed("spawn_vehicle") || spawnPending_ || quick) {
        // `vehicle_spawn <recipe>`: pick that one first (the catalogue's recipe names: motorcycle, moped, ...)
        wantRecipe_ = spawnPending_ ? spawnRecipe_ : std::string();
        if (quick) {
            loadCatalogue();
            for (const Pick& p : catalogue_)
                if (p.quick) { wantRecipe_ = p.recipe; break; }
        }
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
        // ...and its damage (ADR-0353): each zone's health, the panels still on (weakest), the engine
        char dm[192];
        float weakest = 1.0f;
        int on = 0;
        for (const Vehicle::Panel& p : v->panels) if (!p.gone) { ++on; weakest = std::min(weakest, p.health); }
        std::snprintf(dm, sizeof(dm), " damage f %.2f r %.2f l %.2f r %.2f roof %.2f under %.2f panels %d/%zu weakest %.2f%s%s",
                      v->zoneHealth[0], v->zoneHealth[1], v->zoneHealth[2], v->zoneHealth[3], v->zoneHealth[4],
                      v->zoneHealth[5], on, v->panels.size(), weakest, v->engineDead ? " ENGINE DEAD" : "",
                      v->bodyDented ? " dented" : "");
        telem += dm;
    });
    answers_.set("vehicle?", telem + " timescale " + std::to_string(ctx.clock.timeScale()));

    updateHorn(ctx);
    updateEngines(ctx);
    measureSlide(ctx);
    updateTyres(ctx);
}

void VehicleSystem::updateEngines(FrameContext& ctx) {
    if (!ctx.audio.ready()) return;
    // Every RUNNING physical car -- the player's and any the AI drives -- near enough to hear. A driverless car is
    // silent (engineSoundFor's contract), so a parked street stays parked.
    std::vector<EngineVoicePool::Engine> heard;
    const Vec3 ear = ctx.view.camera.position;
    const Real range = engines_.range();
    ctx.world.each<Transform, Vehicle>([&](Entity e, Transform& t, Vehicle& v) {
        const bool running = v.driver.valid() || ctx.world.has<AgentDriver>(e);
        if (!running || v.pedalled || v.engineDead) return;   // a bicycle has legs, not an engine; a dead one is silent
        const Real d2 = (t.position - ear).lengthSquared();
        if (d2 > range * range) return;
        EngineVoicePool::Engine en{(static_cast<uint64_t>(e.generation) << 32) | e.index, t.position, v.speed, v.throttle,
                                   static_cast<Real>(e.index % 17) * Real(0.37), d2};
        en.health = v.zoneHealth[static_cast<int>(DamageZone::Front)];
        en.roughBelow = v.damage.roughBelow;
        en.roughWobble = v.damage.roughWobble;
        en.roughMisfire = v.damage.roughMisfire;
        heard.push_back(en);
    });
    engines_.update(ctx.audio, heard, ctx.clock.simulatedTime(), ctx.frameDelta);
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
    Real along = 0;
    for (std::size_t i = 0; i < tel.wheelSlip.size(); ++i)
        if (i < tel.wheelContact.size() && tel.wheelContact[i]) along = std::max(along, std::fabs(tel.wheelSlip[i]));
    // ONLY A DRIFT OR A HARD STOP (Glenn, 2026-10-24: "still kinda loud. We should use it when we brake hard or
    // drift"): a slide the drift has hold of, by its slip angle; or the brake pressed hard at speed, more as the wheels
    // lock. A brisk corner, a wheelspin pull-away, a bump: silent.
    const PhysicsWorld::DriftState drift = pw.vehicleDrift(v->vehicleId);
    const Real speed = std::fabs(v->speed);
    const Real drifting = drift.drifting ? smoothstep(6.0, 26.0, std::fabs(drift.slipDeg)) : Real(0);
    const Real braking = smoothstep(0.55, 0.95, std::max(v->brake, v->handBrake)) * (0.6 + 0.4 * smoothstep(0.2, 0.7, along));
    const Real fast = smoothstep(4.0, 11.0, speed);
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
    drive(squeal_, static_cast<float>(0.3 * slide), static_cast<float>(0.9 + 0.15 * slide + speed / 150.0),
          t->position, 9);
    // A BICYCLE: the freewheel's pawls ticking while it coasts (as fast as the back wheel turns), the tyres' hum
    const bool bike = v->pedalled;
    const Real r = v->config.wheels.empty() ? 0.34 : v->config.wheels.back().radius;
    const Real clicks = 24.0 * speed / (6.283185307179586 * r);   // 24 pawl engagements a turn
    const bool coasting = bike && speed > 0.8 && std::fabs(v->throttle) < 0.05;
    drive(freewheel_, coasting ? static_cast<float>(0.3 * smoothstep(0.8, 3.0, speed)) : 0.0f,
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

// THE PANELS SWING (ADR-0352): each toward its target, a door in about half a second, the hood and trunk slower; a
// door opened for the driver shuts again by itself
void VehicleSystem::swingPanels(FrameContext& ctx) {
    const float dt = static_cast<float>(ctx.clock.fixedStep());
    // the control channel's `panels open|shut|knock [name]`, in the order they came
    std::vector<std::string> queue;
    queue.swap(panelsStaged_);
    for (const std::string& staged : queue) {
        const bool knock = staged.rfind("knock", 0) == 0;
        const bool open = staged.rfind("open", 0) == 0;
        const std::size_t cut = staged.find(' ');
        const std::string only = cut == std::string::npos ? std::string() : staged.substr(cut + 1);
        auto named = [&](const Vehicle::Panel& p) { return only.empty() || p.name.find(only) != std::string::npos; };
        if (knock) {   // the named panel off every car, as a spent one comes off (ADR-0353)
            std::vector<std::pair<Entity, int>> off;
            ctx.world.each<Vehicle>([&](Entity e, Vehicle& v) {
                for (std::size_t i = 0; i < v.panels.size(); ++i)
                    if (!v.panels[i].gone && named(v.panels[i])) off.emplace_back(e, static_cast<int>(i));
            });
            for (const auto& [e, i] : off)
                if (Vehicle* v = ctx.world.get<Vehicle>(e)) detachPanel(ctx, e, *v, i, Vec3(0, 1.0, 0));
            continue;
        }
        ctx.world.each<Vehicle>([&](Entity, Vehicle& v) {
            for (Vehicle::Panel& p : v.panels)
                if (named(p)) { p.target = open ? p.open : 0.0f; p.hold = 0.0f; }
        });
    }
    struct Heard { Vec3 at; int kind; float volume; };
    std::vector<Heard> heard;
    ctx.world.each<Vehicle>([&](Entity e, Vehicle& v) {
        const Transform* ct = ctx.world.get<Transform>(e);
        for (Vehicle::Panel& p : v.panels) {
            if (p.gone) continue;
            if (p.hold > 0.0f) {
                p.hold -= dt;
                if (p.hold <= 0.0f) { p.hold = 0.0f; p.target = 0.0f; }
            }
            const bool door = p.name.rfind("door_", 0) == 0;
            const float rate = p.open / (door ? 0.45f : 0.85f) * dt;
            const float was = p.angle;
            if (p.angle < p.target) p.angle = std::min(p.target, p.angle + rate);
            else if (p.angle > p.target) p.angle = std::max(p.target, p.angle - rate);
            // heard (ADR-0355): the latch letting go as it opens, the panel slamming home as it shuts
            if (!ct) continue;
            const Vec3 at = ct->position + ct->orientation.rotate(p.centre);
            if (was == 0.0f && p.angle > 0.0f) heard.push_back({at, 3, 0.5f});
            if (was > 0.0f && p.angle == 0.0f)
                heard.push_back({at, door ? 0 : p.name == "hood" ? 1 : 2, door ? 0.9f : 0.8f});
        }
    });
    for (const Heard& h : heard) panelSound(ctx, h.at, h.kind, h.volume);
}

void VehicleSystem::panelSound(FrameContext& ctx, const Vec3& at, int kind, float volume) {
    if (!ctx.audio.ready()) return;
    AudioClipHandle* clip = kind < 3 ? &shutClip_[kind] : kind == 3 ? &openClip_ : &starterClip_;
    if (!clip->valid()) {
        const uint32_t r = ctx.audio.sampleRate();
        const std::vector<float> pcm = kind < 3 ? sfx::panelShut(static_cast<sfx::PanelKind>(kind), r, 1)
                                     : kind == 3 ? sfx::panelOpen(r, 1) : sfx::starterNoCatch(r, 1);
        *clip = ctx.audio.createClip(pcm.data(), pcm.size(), 1, r);
    }
    if (!clip->valid()) return;
    AudioPlayParams params;
    params.volume = volume;
    ctx.audio.playAt(*clip, at, 30.0, params);
}

// THE DAMAGE PASS (ADR-0353): each touch on a drivable car -- a knock by its closing speed, a scrape by its slide --
// costs the zone it lands on and the panel nearest it, dents the meshes round the point, and sparks; a panel spent
// comes off, the nose spent kills the engine. The dented meshes go up to the GPU a few times a second at most.
void VehicleSystem::applyDamage(FrameContext& ctx) {
    const float dt = static_cast<float>(ctx.clock.fixedStep());
    PhysicsWorld& pw = physicsSys.physicsWorld();
    // the loose parts' first push (their bodies exist one step after they are made)
    for (std::size_t i = 0; i < looseKick_.size();) {
        const RigidBody* rb = ctx.world.alive(looseKick_[i].e) ? ctx.world.get<RigidBody>(looseKick_[i].e) : nullptr;
        if (rb && rb->bodyId != INVALID_PHYSICS_BODY) {
            // its weight and the air on it (Glenn: "maybe they should be given more weight"), then its push
            pw.setBodyMass(rb->bodyId, looseKick_[i].mass, 0.3, 0.6);
            pw.setLinearVelocity(rb->bodyId, looseKick_[i].velocity);
            looseKick_.erase(looseKick_.begin() + static_cast<std::ptrdiff_t>(i));
        } else if (!rb) {
            looseKick_.erase(looseKick_.begin() + static_cast<std::ptrdiff_t>(i));
        } else {
            ++i;
        }
    }
    std::vector<Hit> hits;
    hits.swap(hits_);
    for (const Hit& h : hits) {
        if (!h.car.valid() || !ctx.world.alive(h.car)) continue;
        Vehicle* v = ctx.world.get<Vehicle>(h.car);
        const Transform* t = ctx.world.get<Transform>(h.car);
        if (!v || !t || v->vehicleId == PhysicsWorld::INVALID_VEHICLE) continue;
        // THE GROUND IS NOT A CRASH (Glenn: "cars seem to lose power and then eventually become undrivable"): the
        // chassis box's bottom corners brush the road as the car pitches over bumps and kerbs, and every such brush was
        // a scrape on the nose or the tail until the engine died. A touch whose normal stands up is the ground: a
        // scrape along it costs nothing (it sparks only bottoming out fast), a knock from below -- a hard landing --
        // is the underside's, at a third.
        const bool ground = std::fabs(h.normal.y) > 0.7;
        if (ground && h.persisted) {
            if (h.slide > 8.0 && v->scrapeSparkClock <= 0.0f) {
                requestParticles(ctx.world, "spark", h.at, Vec3(0, 1, 0), 3, 0.8);
                v->scrapeSparkClock = 0.15f;
            }
            continue;
        }
        // (a scrape's sparks are a trickle: a few a tenth of a second, not every contact every step)
        const DamageRule& rule = v->damage;
        const int sparks = touchSparks(rule, h.persisted, h.speed, h.slide);
        if (sparks > 0 && (!h.persisted || v->scrapeSparkClock <= 0.0f)) {
            const Vec3 away = h.normal * -1.0;   // (the normal points from the car to what it hit)
            requestParticles(ctx.world, "spark", h.at, away, sparks, std::min(Real(2), std::max(h.speed, h.slide) / 8.0));
            if (h.persisted) v->scrapeSparkClock = 0.08f;
        }
        // (a piece knocked loose -- a hydrant, a bin -- weighs a few dozen kilos: a tenth of a wall's knock)
        const Real harm = touchHarm(rule, h.persisted, h.speed, h.slide, dt, v->config.mass) * (h.broke ? rule.breakShare : 1.0) *
                          (ground ? rule.groundShare : 1.0);
        if (harm <= 0) continue;
        // NOT A CRASH: a car's first half second (made, or taken over from the city, on top of whatever it overlaps --
        // a car taken from the traffic came with its engine already "wrecked"), or a knock faster than anything drives
        // (a body gone bad, the physics sanitizer's to report)
        if (v->age < 0.5f || h.speed > 60.0) continue;
        const Quat inv = t->orientation.conjugate();
        const Vec3 local = inv.rotate(h.at - t->position);
        const Vec3 inward = inv.rotate(h.normal);   // into the car, chassis frame
        const DamageZone zone = ground ? DamageZone::Under : damageZone(local, v->config.chassisHalfExtent);
        float& zh = v->zoneHealth[static_cast<int>(zone)];
        // (a knock within a third of a second of the last on this zone is the same crash: it adds only what it
        // exceeds the worst so far by; a scrape is its own, slow, cost)
        Real cost = harm;
        if (!h.persisted) {
            const int zi = static_cast<int>(zone);
            if (v->crashClock[zi] > 0.0f) cost = std::max(Real(0), harm - v->crashHarm[zi]);
            v->crashHarm[zi] = v->crashClock[zi] > 0.0f ? std::max(v->crashHarm[zi], static_cast<float>(harm)) : static_cast<float>(harm);
            v->crashClock[zi] = static_cast<float>(rule.crashWindow);
        }
        zh = std::max(0.0f, zh - static_cast<float>(cost * rule.zoneScale));
        if (zone == DamageZone::Front && zh <= 0.0f && !v->engineDead) {
            v->engineDead = true;
            LOG_INFO << "[vehicle] the engine is wrecked: undrivable";
            // a crumpled hood springs its catch and stands ajar (Glenn: "this car still looks drivable")
            const int hood = v->panelIndex("hood");
            if (hood >= 0 && !v->panels[static_cast<std::size_t>(hood)].gone) {
                Vehicle::Panel& p = v->panels[static_cast<std::size_t>(hood)];
                p.target = p.open * 0.3f;
                p.hold = 0.0f;
            }
        }
        // the panel the touch lands on (still on): the nearest by its box, within 0.35 m of it -- a bumper knock is
        // the hood's, a door's edge its own
        int near = -1;
        Real nearD = 0.35;
        for (std::size_t i = 0; i < v->panels.size(); ++i) {
            const Vehicle::Panel& p = v->panels[i];
            if (p.gone) continue;
            const Vec3 c(std::clamp(local.x, p.lo.x, p.hi.x), std::clamp(local.y, p.lo.y, p.hi.y),
                         std::clamp(local.z, p.lo.z, p.hi.z));
            const Real d = (c - local).length();
            if (d < nearD) { nearD = d; near = static_cast<int>(i); }
        }
        if (near >= 0) v->panels[static_cast<std::size_t>(near)].health -= static_cast<float>(cost * rule.panelScale);
        // THE DENT: a knock pushes the skin in round the point (a scrape only scuffs: no dent)
        if (!h.persisted && harm > rule.dentFrom) {
            // deep enough to read (Glenn: "show the visual damage"): a wide shallow ding for a nudge, a crumple of a
            // quarter metre for a wreck, the paint scraped toward the bottom of it
            const Real radius = rule.dentRadiusLo + (rule.dentRadiusHi - rule.dentRadiusLo) * std::min(Real(1), harm);
            const Real depth = std::min(rule.dentDepthMax, rule.dentDepthLo + rule.dentDepthPer * harm);
            const Real scuff = std::min(Real(1), rule.scuffLo + rule.scuffPer * harm);
            if (v->bodyMesh) {
                if (!v->bodyDented) { v->bodyMesh = std::make_shared<RenderMesh>(*v->bodyMesh); v->bodyDented = true; }
                v->meshDirty |= dentMesh(*v->bodyMesh, local, inward, radius, depth, scuff);
            }
            for (std::size_t k = 0; k < v->partMeshes.size(); ++k) {
                if (!v->partMeshes[k]) continue;
                // a panel's part: the point in its own (hinged) frame
                Vec3 at = local, dir = inward;
                const int pn = k < v->bodyPartPanel.size() ? v->bodyPartPanel[k] : -1;
                if (pn >= 0 && pn < static_cast<int>(v->panels.size())) {
                    const Vehicle::Panel& p = v->panels[static_cast<std::size_t>(pn)];
                    const Quat qi = p.turn().conjugate();
                    at = qi.rotate(local - p.pivot) + p.pivot;
                    dir = qi.rotate(inward);
                }
                if (!v->partDented[k]) {
                    // copy only when the dent reaches it
                    bool reaches = false;
                    for (const Vertex& vx : v->partMeshes[k]->vertices)
                        if ((vx.position - at).lengthSquared() < radius * radius) { reaches = true; break; }
                    if (!reaches) continue;
                    v->partMeshes[k] = std::make_shared<RenderMesh>(*v->partMeshes[k]);
                    v->partDented[k] = 1;
                }
                v->meshDirty |= dentMesh(*v->partMeshes[k], at, dir, radius, depth, scuff);
            }
        }
        if (near >= 0 && v->panels[static_cast<std::size_t>(near)].health <= 0.0f) {
            // it leaves slower than the car (torn, it drags) and only just lifts
            const Vec3 vel = pw.vehicleVelocity(v->vehicleId);
            detachPanel(ctx, h.car, *v, near, vel * rule.kickCar + h.normal * (-rule.kickOut) + Vec3(0, rule.kickUp, 0));
        }
    }
    // the dented meshes up to the GPU, at most every 0.15 s per car
    ctx.world.each<Vehicle>([&](Entity e, Vehicle& v) {
        v.age += dt;
        v.scrapeSparkClock = std::max(0.0f, v.scrapeSparkClock - dt);
        for (float& c : v.crashClock) c = std::max(0.0f, c - dt);
        // THE HOOD SMOKES (Glenn: "maybe even show smoke and sparks and look damaged"): white steam from the radiator
        // once the nose is hurt, thickening as it goes, then dark smoke near the end and once the engine is dead;
        // trailing the car as it drives
        const float nose = v.zoneHealth[static_cast<int>(DamageZone::Front)];
        if (nose < v.damage.steamBelow || v.engineDead) {
            const Transform* ct = ctx.world.get<Transform>(e);
            if (ct && !(v.engineSmoke.valid() && ctx.world.alive(v.engineSmoke))) {
                v.engineSmoke = ctx.world.create();
                ctx.world.add<Transform>(v.engineSmoke, *ct);
                ParticleEmitter em;
                em.halfExtent = Vec3(0.25, 0.05, 0.2);
                ctx.world.add<ParticleEmitter>(v.engineSmoke, em);
            }
            Transform* st = ctx.world.get<Transform>(v.engineSmoke);
            ParticleEmitter* pe = ctx.world.get<ParticleEmitter>(v.engineSmoke);
            if (ct && st && pe) {
                const Vec3& hx = v.config.chassisHalfExtent;
                st->position = ct->position + ct->orientation.rotate(Vec3(0, hx.y * 0.8, hx.z * 0.65));
                const bool black = v.engineDead || nose < v.damage.blackBelow;
                pe->effect = black ? "engine_smoke" : "steam";
                pe->rate = black ? 28.0 : 4.0 + 30.0 * (v.damage.steamBelow - nose);
                const Vec3 vel = v.vehicleId != PhysicsWorld::INVALID_VEHICLE ? pw.vehicleVelocity(v.vehicleId) : Vec3();
                pe->velocity = vel * 0.6;
            }
        }
        v.sinceUpload += dt;
        if (!v.meshDirty || v.sinceUpload < 0.15f) return;
        v.meshDirty = false;
        v.sinceUpload = 0.0f;
        if (v.bodyDented && v.bodyMesh)
            if (Renderable* r = ctx.world.get<Renderable>(e)) {
                const MeshHandle old = r->mesh;
                r->mesh = ctx.assets.acquireMesh(*v.bodyMesh);
                if (old.valid()) ctx.assets.releaseMesh(old);
            }
        for (std::size_t k = 0; k < v.bodyParts.size() && k < v.partMeshes.size(); ++k) {
            if (!v.partDented[k] || !v.partMeshes[k]) continue;
            if (Renderable* r = ctx.world.get<Renderable>(v.bodyParts[k])) {
                const MeshHandle old = r->mesh;
                r->mesh = ctx.assets.acquireMesh(*v.partMeshes[k]);
                if (old.valid()) ctx.assets.releaseMesh(old);
            }
        }
    });
}

// A SPENT PANEL COMES OFF (ADR-0353): its painted skin becomes a loose body tumbling from where it was, at the car's
// speed and a knock's push; its glass and card go with it (the glass is gone -- shattered). The car keeps driving
// with the hole. At most 24 loose parts lie about; the oldest goes when another falls.
void VehicleSystem::detachPanel(FrameContext& ctx, Entity car, Vehicle& v, int pn, const Vec3& kick) {
    if (pn < 0 || pn >= static_cast<int>(v.panels.size())) return;
    Vehicle::Panel& panel = v.panels[static_cast<std::size_t>(pn)];
    if (panel.gone) return;
    panel.gone = true;
    const Transform* ct = ctx.world.get<Transform>(car);
    if (!ct) return;
    const Quat q = panel.turn();
    const Vec3 centreWorld = ct->position + ct->orientation.rotate(q.rotate(panel.centre - panel.pivot) + panel.pivot);
    bool freed = false;
    std::vector<Entity> keep;
    std::vector<int> keepPanel;
    std::vector<std::shared_ptr<RenderMesh>> keepMesh;
    std::vector<char> keepDent;
    for (std::size_t k = 0; k < v.bodyParts.size(); ++k) {
        const int kp = k < v.bodyPartPanel.size() ? v.bodyPartPanel[k] : -1;
        if (kp != pn) {
            keep.push_back(v.bodyParts[k]);
            keepPanel.push_back(kp);
            keepMesh.push_back(k < v.partMeshes.size() ? v.partMeshes[k] : nullptr);
            keepDent.push_back(k < v.partDented.size() ? v.partDented[k] : 0);
            continue;
        }
        const Entity pe = v.bodyParts[k];
        Renderable* r = ctx.world.get<Renderable>(pe);
        const bool shell = r && r->material.opacity >= 1.0f && r->material.metallic > 0.05f;
        if (freed || !shell || k >= v.partMeshes.size() || !v.partMeshes[k]) {
            ctx.world.destroy(pe);   // the glass, the card
            continue;
        }
        // the skin, re-centred on the panel's middle so the body tumbles about it
        RenderMesh m = *v.partMeshes[k];
        Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
        for (Vertex& vx : m.vertices) {
            vx.position = vx.position - panel.centre;
            lo = Vec3(std::min(lo.x, vx.position.x), std::min(lo.y, vx.position.y), std::min(lo.z, vx.position.z));
            hi = Vec3(std::max(hi.x, vx.position.x), std::max(hi.y, vx.position.y), std::max(hi.z, vx.position.z));
        }
        const MeshHandle old = r->mesh;
        r->mesh = ctx.assets.acquireMesh(m);
        if (old.valid()) ctx.assets.releaseMesh(old);
        if (Transform* pt = ctx.world.get<Transform>(pe)) {
            pt->position = centreWorld;
            pt->orientation = ct->orientation * q;
            if (PrevTransform* pp = ctx.world.get<PrevTransform>(pe)) pp->value = *pt;
        }
        Collider col;
        col.shape = ColliderShape::Box;
        col.halfExtent = Vec3(std::max(Real(0.03), (hi.x - lo.x) * 0.5), std::max(Real(0.03), (hi.y - lo.y) * 0.5),
                              std::max(Real(0.03), (hi.z - lo.z) * 0.5));
        col.friction = 0.6;
        ctx.world.add<Collider>(pe, col);
        RigidBody rb;
        rb.motion = BodyMotion::Dynamic;
        rb.ignore = physicsSys.physicsWorld().vehicleBody(v.vehicleId);   // (born inside the car's box: never pushed off it)
        rb.ignoreGrace = 60;
        ctx.world.add<RigidBody>(pe, rb);
        looseKick_.push_back({pe, kick, panel.mass});
        loose_.push_back(pe);
        freed = true;
    }
    v.bodyParts = keep;
    v.bodyPartPanel = keepPanel;
    v.partMeshes = keepMesh;
    v.partDented = keepDent;
    requestParticles(ctx.world, "spark", centreWorld, kick, 24, 1.2);
    LOG_INFO << "[vehicle] " << panel.name << " came off";
    while (loose_.size() > 24) {
        if (ctx.world.alive(loose_.front())) ctx.world.destroy(loose_.front());
        loose_.erase(loose_.begin());
    }
}

void VehicleSystem::fixedUpdate(FrameContext& ctx) {
    createVehicles(ctx);
    applyDamage(ctx);
    driveVehicles(ctx);
    swingPanels(ctx);
    writeBack(ctx);
    stepSeat(ctx);
}

}  // namespace engine
