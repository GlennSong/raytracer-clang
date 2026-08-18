#include "city_possess.h"

#include "city_render.h"
#include "city_sim.h"

#include "../../engine/components.h"
#include "../../engine/ai/pathfind.h"
#include "../../engine/systems/physics_system.h"
#include "../../log.h"

#ifdef RT_ENABLE_SCRIPTING
#include "../../engine/scripting/script_vm.h"
#include "../../engine/scripting/procgen_bindings.h"
#include "../../engine/scripting/script_modules.h"
#include "../../engine/scripting/vehicle_spec.h"
#include "../../engine/script_assets.h"
#endif

#include <algorithm>
#include <cmath>

namespace citysim {

using engine::Entity;
using engine::Vec2;
using engine::Vec3;

namespace {

// The spectate rig's presets — a car reads best a little farther and higher
// than a pedestrian.
void configureFollow(engine::FollowCameraController& f, bool car) {
    f.distance = car ? 8.0 : 5.0;
    f.minDistance = car ? 3.0 : 2.0;
    f.maxDistance = car ? 22.0 : 12.0;
    f.targetHeight = car ? 1.5 : 1.2;
    f.orbitPitch = car ? -14.0 : -12.0;
    f.orbitYaw = 0.0;
    f.lateralOffset = 0.0;
}

}  // namespace

void CityPossessSystem::update(engine::FrameContext& ctx) {
    // Consume the channel's one-shot (the citysim.* idiom). Commands are
    // consumed even when the city isn't built so the sender gets a status
    // explaining why nothing happened, not silence.
    const std::string staged = ctx.settings.getString("possess.cmd", "");
    if (!staged.empty()) {
        ctx.settings.setString("possess.cmd", "");
        handleCommand(ctx, parsePossessCmd(staged));
    }

    // The player grabbing the possessed car (G) is a release, not a fight:
    // VehicleSystem's driver branch outranks AgentDriver, so just let go.
    if (car_.valid() && ctx.world.alive(car_)) {
        if (const engine::Vehicle* v = ctx.world.get<engine::Vehicle>(car_))
            if (v->driver.valid()) releasePossession(ctx);
    } else if (car_.valid()) {
        releasePossession(ctx);   // car destroyed under us (reload etc.)
    }

    updateCamera(ctx);
    publishStatus(ctx);
}

void CityPossessSystem::handleCommand(engine::FrameContext& ctx,
                                      const PossessCmd& cmd) {
    error_.clear();
    switch (cmd.kind) {
        case PossessCmd::Kind::None:
            return;
        case PossessCmd::Kind::Invalid:
            error_ = cmd.error;
            return;
        case PossessCmd::Kind::Car:
            possessCar(ctx, cmd);
            return;
        case PossessCmd::Kind::Walker:
            possessWalker(ctx, cmd);
            return;
        case PossessCmd::Kind::DriveTo:
            if (car_.valid()) driveTo(ctx, cmd.x, cmd.z);
            else if (walkerAgent_ >= 0) walkTo(ctx, cmd.x, cmd.z);
            else error_ = "nothing possessed — `possess car` first";
            return;
        case PossessCmd::Kind::WalkTo:
            if (walkerAgent_ >= 0) walkTo(ctx, cmd.x, cmd.z);
            else if (car_.valid()) driveTo(ctx, cmd.x, cmd.z);
            else error_ = "nothing possessed — `possess walker` first";
            return;
        case PossessCmd::Kind::Stop:
            hasDest_ = false;
            follower_ = engine::LaneFollower{};
            pathSpeeds_.clear();
            if (car_.valid())
                if (auto* ad = ctx.world.get<engine::AgentDriver>(car_))
                    ad->command = engine::DriverCommand{};   // speed 0 = brake hold
            state_ = (car_.valid() || walkerAgent_ >= 0) ? PossessState::Idle
                                                         : PossessState::None;
            return;
        case PossessCmd::Kind::Release:
            releasePossession(ctx);
            return;
    }
}

void CityPossessSystem::possessCar(engine::FrameContext& ctx,
                                   const PossessCmd& cmd) {
    if (!city_.built()) {
        error_ = "no city here — possession needs play mode on a city level";
        return;
    }
    releasePossession(ctx);   // one avatar at a time

#ifdef RT_ENABLE_SCRIPTING
    // Spawn point: given, or 12 m ahead of the current view. Aligned with the
    // nearest road link so the first drive command doesn't start with a U-turn.
    Vec2 at(cmd.x, cmd.z);
    if (!cmd.hasPos) {
        const auto& cam = ctx.view.camera;
        Vec3 fwd = cam.target - cam.position;
        fwd.y = 0;
        const engine::Real fl = fwd.length();
        fwd = fl > 1e-3 ? fwd * (1.0 / fl) : Vec3(0, 0, -1);
        at = Vec2(cam.position.x + fwd.x * 12.0, cam.position.z + fwd.z * 12.0);
    }
    const engine::NavGraph& nav = city_.nav();
    const int li = nav.nearestLink(at);
    engine::Real yawDeg = 0;
    if (li >= 0) {
        const Vec2 d = nav.direction(li);
        yawDeg = engine::radiansToDegrees(std::atan2(d.x, d.y));
        // Spawn ON the lane centre (the sim's spacing — kerb parking narrows
        // the carriageway), so the first pursuit point is already underfoot.
        at = nav.laneCenter(li, 0, 0.5, city_.sim().laneSpacingFor(li));
    }
    const Vec3 spawn(at.x, city_.groundHeightAt(at.x, at.y) + 1.2, at.y);

    // The spawnInFront recipe (vehicle_system.cpp): rare action, fresh VM.
    const std::string lib = engine::loadScriptCode("vehicles.lua", "");
    if (lib.empty()) {
        error_ = "vehicles.lua not found";
        return;
    }
    engine::ScriptVM vm;
    engine::openProcgenLibrary(vm);
    engine::openModuleLoader(vm, engine::makeModuleSource(""));
    std::string err;
    if (!vm.doString(lib, &err)) {
        error_ = "vehicles.lua: " + err;
        return;
    }
    engine::VehicleSpec spec;
    // A signal-red sedan: the avatar should be findable in a screenshot.
    if (!engine::loadVehicleSpec(vm,
                                 "return vehicle.sedan(seed, {color={0.82,0.12,0.10}})",
                                 static_cast<uint32_t>(1000 + ++spawnSeed_),
                                 spec, &err)) {
        error_ = "vehicle spec: " + err;
        return;
    }
    car_ = engine::spawnVehicle(ctx.world, ctx.assets, spec, spawn, yawDeg);
    ctx.world.add<engine::AgentDriver>(car_, engine::AgentDriver{});
    configureFollow(follow_, /*car=*/true);
    state_ = PossessState::Idle;
    hasDest_ = false;
    LOG_INFO << "possess: car spawned at " << spawn.x << ", " << spawn.z;
#else
    (void)cmd;
    error_ = "car possession needs the scripting build (vehicles.lua)";
#endif
}

void CityPossessSystem::possessWalker(engine::FrameContext& ctx,
                                      const PossessCmd& cmd) {
    if (!city_.built()) {
        error_ = "no city here — possession needs play mode on a city level";
        return;
    }
    releasePossession(ctx);

    // Commandeer the nearest live pedestrian: they already have a routed life,
    // a body when near, and a walk cycle — possession just redirects them.
    Vec2 ref(cmd.x, cmd.z);
    if (!cmd.hasPos)
        ref = Vec2(ctx.view.camera.position.x, ctx.view.camera.position.z);
    const auto& agents = city_.sim().agents();
    int best = -1;
    engine::Real bestD2 = 1e30;
    for (std::size_t i = 0; i < agents.size(); ++i) {
        const Agent& a = agents[i];
        if (a.mode != Agent::Mode::Pedestrian || a.released) continue;
        const engine::Real dx = a.pos.x - ref.x, dz = a.pos.y - ref.y;
        const engine::Real d2 = dx * dx + dz * dz;
        if (d2 < bestD2) { bestD2 = d2; best = static_cast<int>(i); }
    }
    if (best < 0) {
        error_ = "no pedestrians to possess";
        return;
    }
    walkerAgent_ = best;
    configureFollow(follow_, /*car=*/false);
    state_ = PossessState::Idle;
    hasDest_ = false;
}

void CityPossessSystem::driveTo(engine::FrameContext& ctx, engine::Real x,
                                engine::Real z) {
    const engine::Transform* t = ctx.world.get<engine::Transform>(car_);
    if (!t) { error_ = "possessed car has no transform"; return; }
    const engine::NavGraph& nav = city_.nav();
    const Vec2 pos2(t->position.x, t->position.z);

    // Enter the network along the car's OWN link, not via the globally nearest
    // node: mid-block, the nearest node can belong to the street on the far
    // side of the buildings, and pursuit then aims straight through a wall
    // (measured on the first live drive — the car wedged nose-first into a
    // facade). So: partial samples from the car's spot to its link's forward
    // end, then the A* route from that end.
    // Sample lanes with the SIM'S per-link spacing, not the default width:
    // kerb-parking links narrow the carriageway (laneSpacingFor subtracts the
    // strips), and the default-width offset landed the second live drive on
    // the sidewalk, where it kerb-climbed and beached on a terrace step. The
    // possessed car drives exactly the line ambient traffic does.
    auto sampleLane = [&](int l, engine::Real tt) {
        return nav.laneCenter(l, 0, tt, city_.sim().laneSpacingFor(l));
    };
    // nearestLink ties between the two directed siblings of a two-way street
    // (same geometry; lower index wins) — and the wrong one puts the polyline
    // in the ONCOMING lane, one lane-pair away (the fourth live drive spawned
    // with lat=2.7 and got dragged across the street). Pick the direction that
    // agrees with the car's heading.
    const Vec3 head3 = t->orientation.rotate(Vec3(0, 0, 1));
    const Vec2 heading(head3.x, head3.z);
    int li = nav.nearestLink(pos2);
    if (li >= 0) {
        const Vec2 d = nav.direction(li);
        if (d.x * heading.x + d.y * heading.y < 0) {
            const engine::NavLink& fwdLink = nav.links[li];
            for (int cand : nav.outLinks[fwdLink.to]) {
                if (nav.links[cand].to == fwdLink.from) { li = cand; break; }
            }
        }
    }
    engine::Route route;
    std::vector<Vec2> path;
    pathSpeeds_.clear();
    if (li >= 0) {
        const engine::NavLink& link = nav.links[li];
        const Vec2 a = nav.nodes[link.from], b = nav.nodes[link.to];
        const Vec2 ab(b.x - a.x, b.y - a.y);
        const engine::Real len2 = ab.x * ab.x + ab.y * ab.y;
        engine::Real tOn = 0;
        if (len2 > 1e-9)
            tOn = std::clamp(((pos2.x - a.x) * ab.x + (pos2.y - a.y) * ab.y) /
                                 len2,
                             engine::Real(0), engine::Real(1));
        route = engine::findRoute(nav, link.to, nav.nearestNode(Vec2(x, z)));
        const engine::Real speed = engine::classSpeed(link.klass);
        const int segs = std::max(
            1, static_cast<int>(std::ceil(link.length * (1.0 - tOn) / 3.0)));
        for (int s = 0; s <= segs; ++s) {
            const engine::Real tt = tOn + (1.0 - tOn) * s / segs;
            path.push_back(sampleLane(li, tt));
            pathSpeeds_.push_back(speed);
        }
    } else {
        route = engine::findRouteBetween(nav, pos2, Vec2(x, z));
    }
    if (!route.valid()) {
        // Dead-end entry: the heading-matched link can point into a stub (the
        // corniche's seaward end did — the car drove 8 m and declared
        // "arrived"). Fall back to nearest-node routing; the follower's
        // windowed snap tolerates a start slightly behind the car. If THAT
        // fails too, the destination is genuinely unroutable — say so rather
        // than fake a mini-drive to the stub's end.
        route = engine::findRouteBetween(nav, pos2, Vec2(x, z));
        if (!route.valid()) {
            state_ = PossessState::NoRoute;
            return;
        }
        path.clear();
        pathSpeeds_.clear();
    }
    if (route.valid()) {
        for (std::size_t leg = 0; leg < route.links.size(); ++leg) {
            const int l2 = route.links[leg];
            const engine::NavLink& lk = nav.links[l2];
            const engine::Real speed = engine::classSpeed(lk.klass);
            const int segs =
                std::max(1, static_cast<int>(std::ceil(lk.length / 3.0)));
            // Skip t=0 when a previous stretch already ends at this junction.
            const int first = path.empty() ? 0 : 1;
            for (int s = first; s <= segs; ++s) {
                path.push_back(sampleLane(l2, static_cast<engine::Real>(s) / segs));
                pathSpeeds_.push_back(speed);
            }
        }
    }
    follower_.setPath(std::move(path));
    stuck_ = engine::StuckDetector{};
    dest_ = Vec2(x, z);
    hasDest_ = true;
    state_ = PossessState::Driving;
}

void CityPossessSystem::walkTo(engine::FrameContext& ctx, engine::Real x,
                               engine::Real z) {
    (void)ctx;
    if (!city_.sim().graph()) { error_ = "no nav graph"; return; }
    if (!city_.simMutable().sendAgentTo(walkerAgent_, Vec2(x, z))) {
        state_ = PossessState::NoRoute;
        return;
    }
    dest_ = Vec2(x, z);
    hasDest_ = true;
    state_ = PossessState::Walking;
}

void CityPossessSystem::releasePossession(engine::FrameContext& ctx) {
    // The car stays in the world (parked: no driver, no brain command means
    // VehicleSystem holds the brake) — a transient prop, never saved.
    if (car_.valid() && ctx.world.alive(car_) &&
        ctx.world.has<engine::AgentDriver>(car_))
        ctx.world.remove<engine::AgentDriver>(car_);
    car_ = Entity{};
    walkerAgent_ = -1;
    hasDest_ = false;
    follower_ = engine::LaneFollower{};
    pathSpeeds_.clear();
    state_ = PossessState::None;
}

void CityPossessSystem::fixedUpdate(engine::FrameContext& ctx) {
    // Stuck is a REPORT, not a mode: the brain keeps running (the first live
    // build gated driveCar on Driving only, so the first stuck fire froze the
    // follower and left the car driving blind on its last stale command —
    // frozen `remaining`, 8 m/s into nowhere).
    if (car_.valid() &&
        (state_ == PossessState::Driving || state_ == PossessState::Stuck))
        driveCar(ctx);

    // Walker arrival is judged from the ghost — the sim drives the walking.
    if (walkerAgent_ >= 0 && state_ == PossessState::Walking && hasDest_) {
        const auto& agents = city_.sim().agents();
        if (walkerAgent_ < static_cast<int>(agents.size())) {
            const Agent& a = agents[walkerAgent_];
            const engine::Real dx = a.pos.x - dest_.x, dz = a.pos.y - dest_.y;
            if (dx * dx + dz * dz < 5.0 * 5.0) state_ = PossessState::Arrived;
        }
    }
}

void CityPossessSystem::driveCar(engine::FrameContext& ctx) {
    if (!ctx.world.alive(car_)) return;
    engine::Vehicle* v = ctx.world.get<engine::Vehicle>(car_);
    engine::AgentDriver* ad = ctx.world.get<engine::AgentDriver>(car_);
    if (!v || !ad || v->vehicleId == engine::PhysicsWorld::INVALID_VEHICLE)
        return;   // Jolt body arrives on VehicleSystem's next createVehicles

    engine::PhysicsWorld& pw = physicsSys_.physicsWorld();
    const Vec3 pos = pw.vehiclePosition(v->vehicleId);
    const engine::Quat q = pw.vehicleOrientation(v->vehicleId);
    const Vec3 fwd = q.rotate(Vec3(0, 0, 1));
    const Vec3 vel = pw.vehicleVelocity(v->vehicleId);
    const engine::Real speed =
        vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z;
    const Vec2 pos2(pos.x, pos.z);

    lastLateral_ = follower_.update(pos2);

    // Don't rear-end the city: sense along the route against every nearby sim
    // driver plus any player-driven physical car, and yield to the leader.
    std::vector<engine::SensedBody> bodies;
    for (const Agent& a : city_.sim().agents()) {
        if (a.mode != Agent::Mode::Driver || a.released) continue;
        const engine::Real dx = a.pos.x - pos.x, dz = a.pos.y - pos.z;
        if (dx * dx + dz * dz > 50.0 * 50.0) continue;
        engine::SensedBody b;
        b.pos = a.pos;
        b.heading = a.heading;
        b.speed = a.speed;
        bodies.push_back(b);
    }
    ctx.world.each<engine::Transform, engine::Vehicle>(
        [&](Entity e, engine::Transform& vt, engine::Vehicle& other) {
            if (e == car_ || !other.driver.valid()) return;
            const Vec3 of = vt.orientation.rotate(Vec3(0, 0, 1));
            engine::SensedBody b;
            b.pos = Vec2(vt.position.x, vt.position.z);
            b.heading = Vec2(of.x, of.z);
            b.speed = other.speed;
            bodies.push_back(b);
        });
    const engine::LeaderSense lead =
        engine::senseAlongPath(follower_, 30.0, 2.2, bodies);

    engine::Real desired =
        speedAtStation(follower_.arcs(), pathSpeeds_, follower_.station());
    desired = engine::followSpeed(desired, lead, /*ownHalfLength=*/2.1);

    ad->command = engine::pursuitCommand(follower_, pos2, desired,
                                         engine::pursuitLookahead(speed));
    lastLeadGap_ = lead.found ? lead.gap : -1.0;

    if (follower_.remaining() < kPossessArriveDist && std::fabs(speed) < 0.6) {
        ad->command = engine::DriverCommand{};   // hold the brake at the kerb
        state_ = PossessState::Arrived;
        hasDest_ = false;
        return;
    }

    // Stuck recovery loop: on fire, hold the brake briefly (never drive blind
    // into whatever wedged us), then resume pursuit. A truly wedged car
    // cycles stuck/driving in the status line — the director's cue to replan.
    if (state_ == PossessState::Stuck) {
        stuckHold_ -= ctx.clock.fixedStep();
        ad->command = engine::DriverCommand{};
        if (stuckHold_ <= 0) state_ = PossessState::Driving;
        return;
    }
    if (stuck_.update(desired, speed, ctx.clock.fixedStep())) {
        state_ = PossessState::Stuck;
        stuckHold_ = 1.5;
        ad->command = engine::DriverCommand{};
    }
}

bool CityPossessSystem::walkerPose(Vec3& outPos, Vec2& outHeading) const {
    // Physical body pose when the walker has one; the sim GHOST otherwise —
    // agentWorldPose only reports external bodies, and a possessed pedestrian
    // beyond the body radius read as pos 0,0,0 (frozen camera, frozen
    // remaining) on the first live walk.
    if (city_.agentWorldPose(walkerAgent_, outPos, outHeading)) return true;
    const auto& agents = city_.sim().agents();
    if (walkerAgent_ < 0 || walkerAgent_ >= static_cast<int>(agents.size()))
        return false;
    const Agent& a = agents[walkerAgent_];
    outPos = Vec3(a.pos.x, city_.groundHeightAt(a.pos.x, a.pos.y) + 0.9,
                  a.pos.y);
    outHeading = a.heading;
    return true;
}

void CityPossessSystem::updateCamera(engine::FrameContext& ctx) {
    Vec3 worldPos;
    engine::Real yawDeg = 0;
    if (car_.valid() && ctx.world.alive(car_)) {
        const engine::Transform* t = ctx.world.get<engine::Transform>(car_);
        if (!t) return;
        worldPos = t->position;
        const Vec3 f = t->orientation.rotate(Vec3(0, 0, 1));
        yawDeg = engine::radiansToDegrees(std::atan2(f.x, -f.z));
    } else if (walkerAgent_ >= 0) {
        Vec2 heading;
        if (!walkerPose(worldPos, heading)) return;
        yawDeg = engine::radiansToDegrees(std::atan2(heading.x, -heading.y));
    } else {
        return;
    }

    // Zoom-only input, heading owns the yaw (the spectate idiom — a full
    // update() would double-apply look input over PlayerSystem's).
    follow_.setTarget(worldPos, yawDeg);
    engine::CameraInput zoom;
    zoom.zoomDelta = ctx.input.scrollDelta;
    follow_.update(zoom, ctx.frameDelta);
    const float aspect =
        (ctx.framebufferHeight > 0)
            ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
            : 1.0f;
    ctx.view.camera = follow_.cameraState(aspect);
    ctx.view.activeCameraEntity = Entity{};
}

void CityPossessSystem::publishStatus(engine::FrameContext& ctx) {
    std::string line;
    if (!error_.empty()) {
        line = "err " + error_;
    } else if (car_.valid() && ctx.world.alive(car_)) {
        const engine::Transform* t = ctx.world.get<engine::Transform>(car_);
        const engine::Vehicle* v = ctx.world.get<engine::Vehicle>(car_);
        line = formatPossessStatus("car", state_,
                                   t ? t->position : Vec3(0, 0, 0),
                                   v ? v->speed : 0.0,
                                   hasDest_ ? follower_.remaining() : 0.0);
        // Follower telemetry: how far off the lane line, and whether we're
        // held behind someone. The blind-drive bug hid for a whole run
        // because none of this was visible.
        if (hasDest_) {
            char extra[64];
            std::snprintf(extra, sizeof(extra), " lat=%.1f lead=%.0f",
                          lastLateral_, lastLeadGap_);
            line += extra;
        }
    } else if (walkerAgent_ >= 0) {
        Vec3 wp(0, 0, 0);
        Vec2 heading;
        walkerPose(wp, heading);
        engine::Real remaining = 0;
        if (hasDest_) {
            const engine::Real dx = wp.x - dest_.x, dz = wp.z - dest_.y;
            remaining = std::sqrt(dx * dx + dz * dz);
        }
        const auto& agents = city_.sim().agents();
        const engine::Real spd =
            walkerAgent_ < static_cast<int>(agents.size())
                ? agents[walkerAgent_].speed
                : 0.0;
        line = formatPossessStatus("walker", state_, wp, spd, remaining);
    } else {
        line = city_.built() ? "none" : "none (city not built — play mode only)";
    }
    ctx.settings.setString("possess.status", line);
}

}  // namespace citysim
