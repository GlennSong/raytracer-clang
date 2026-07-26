#include "city_walkers.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/world.h"
#include "city_sim.h"

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
using engine::CharacterController;
using engine::PhysicsWorld;

namespace {
constexpr Real kCapsuleRadius = 0.25;   // walker capsule (0.5 wide, 1.8 tall)
constexpr Real kCapsuleHalf = 0.65;     // cylinder half-height
constexpr Real kWalkGain = 1.2;         // metres behind the ghost -> m/s toward it
constexpr Real kWalkStandoff = 0.4;     // settle this short of the ghost's spot
constexpr Real kWalkMax = 2.4;          // catch-up ceiling (a brisk jog)
constexpr Real kTetherLead = 5.0;       // ghost may lead its walker by at most this
constexpr Real kKnockRadius = 1.5;      // a vehicle centre this close...
constexpr Real kKnockSpeed = 2.5;       // ...moving this fast -> knockdown
constexpr Real kFaceSpeed = 0.3;        // turn to face travel above this speed
constexpr Real kPersonalSpace = 1.3;    // 360-degree separation radius from people
constexpr Real kSeparationGain = 1.8;   // push strength inside that radius
constexpr Real kBackoffTime = 0.8;      // blocked -> stand still this long, re-think
}  // namespace

// The walk cycle (kWalkPoses / walkPoseSwing / walkPoseIndex, city_meshes.h) is
// SHARED with the third-person player body — one set of constants, one phase
// rule, so the player and the crowd stride identically.
engine::MeshHandle CityWalkerSystem::poseMesh(engine::AssetManager& assets,
                                              int outfit, int pose) {
    int key = outfit * kWalkPoses + pose;
    auto it = poseMeshes_.find(key);
    if (it != poseMeshes_.end()) return it->second;
    engine::MeshHandle h = assets.acquireMesh(
        buildPersonMesh(walkPoseSwing(pose), outfit),
        "citywalk:person" + std::to_string(outfit) + ":" + std::to_string(pose));
    poseMeshes_[key] = h;
    return h;
}

void CityWalkerSystem::spawnWalkers(engine::FrameContext& ctx) {
    // TIERED (8km-city plan P6, the 20fps fix): a physical walker body — a
    // Jolt capsule stepped every tick plus 360-degree separation against
    // every other body — exists ONLY for K-tier pedestrians (the bubble the
    // player can see; ~200 downtown). Piedmont's 1800 all-city capsules were
    // 10.1 ms of the 13 ms fixed step. V-tier peds keep full sim identity
    // and re-grow a body the moment the bubble reaches them. With tiering
    // off (small levels, tests) every ped is K and behavior is unchanged —
    // except that this reconcile now also runs per step, which is what lets
    // bodies come and go at all.
    if (!city_.built()) return;
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    PhysicsWorld& pw = physics_.physicsWorld();

    const auto& agents = sim.agents();
    // Drop walkers whose agents left the K tier (explicit character removal —
    // physics has no entity-destroy hook; see the onStop note).
    for (std::size_t wi = 0; wi < walkers_.size();) {
        Walker& w = walkers_[wi];
        const bool stale =
            w.agentId < 0 || w.agentId >= static_cast<int>(agents.size()) ||
            agents[w.agentId].tier != Agent::Tier::K ||
            agents[w.agentId].mode != Agent::Mode::Pedestrian;
        if (!stale) { ++wi; continue; }
        if (world.alive(w.entity)) {
            if (CharacterController* cc = world.get<CharacterController>(w.entity))
                if (cc->characterId != engine::INVALID_CHARACTER)
                    pw.removeCharacter(cc->characterId);
            world.destroy(w.entity);
        }
        walkers_[wi] = walkers_.back();
        walkers_.pop_back();
    }
    // Membership of the CURRENT walker set, then grow the missing K peds.
    haveWalker_.assign(agents.size(), 0);
    for (const Walker& w : walkers_)
        if (w.agentId >= 0 && w.agentId < static_cast<int>(agents.size()))
            haveWalker_[w.agentId] = 1;
    for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
        const Agent& a = agents[i];
        if (a.mode != Agent::Mode::Pedestrian) continue;
        if (a.tier != Agent::Tier::K) continue;
        if (haveWalker_[i]) continue;

        Entity e = world.create();
        // Spawn a little above the ghost's spot; the character settles under
        // gravity like the player does. Transform.position = capsule CENTRE.
        Transform t;
        t.position = Vec3(a.pos.x, kCapsuleHalf + kCapsuleRadius + 0.6, a.pos.y);
        t.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0),
                                            std::atan2(a.heading.x, a.heading.y));
        t.scale = Vec3(1, 1, 1);
        world.add<Transform>(e, t);
        world.add<PrevTransform>(e, PrevTransform{t});

        // A simple articulated person (device ask): head, torso, swinging limbs,
        // outfit picked deterministically per walker. Hue lives in the mesh's
        // vertex colours, so the material stays white.
        int outfit = static_cast<int>((static_cast<uint32_t>(i) * 2654435761u >> 8) %
                                      static_cast<uint32_t>(personOutfitCount()));
        Renderable r;
        r.mesh = poseMesh(ctx.assets, outfit, 0);
        r.material.albedo = Vec3(1, 1, 1);
        r.material.metallic = 0.0f;
        r.material.roughness = 0.9f;
        r.material.opacity = 1.0f;
        world.add<Renderable>(e, r);

        CharacterController cc;
        cc.radius = kCapsuleRadius;
        cc.halfHeight = kCapsuleHalf;
        cc.stepHeight = 0.4;                 // kerbs and sidewalk lips
        world.add<CharacterController>(e, cc);   // PhysicsSystem creates the capsule

        Walker w;
        w.entity = e;
        w.agentId = i;
        w.outfit = outfit;
        w.facing = a.heading;
        w.blocked.stallSeconds = 1.2;        // walker-scale stall watchdog
        w.blocked.minCommand = 0.5;
        w.blocked.minMotion = 0.15;
        walkers_.push_back(w);
    }
}

void CityWalkerSystem::driveWalkers(engine::FrameContext& ctx) {
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    PhysicsWorld& pw = physics_.physicsWorld();
    Real dt = ctx.clock.fixedStep();

    // Vehicle poses + speeds once for all walkers (the knockdown trigger): the
    // REAL vehicles (player's / promoted), plus the ambient planner cars — with
    // one motion authority their drawn pose IS the sim pose.
    std::vector<Vec2> carPos;
    std::vector<Real> carSpeed;
    world.each<Transform, engine::Vehicle>([&](Entity, Transform& t, engine::Vehicle& v) {
        carPos.push_back(Vec2(t.position.x, t.position.z));
        Real s = 0;
        if (v.vehicleId != PhysicsWorld::INVALID_VEHICLE) {
            Vec3 vel = pw.vehicleVelocity(v.vehicleId);
            s = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
        }
        carSpeed.push_back(s);
    });
    for (const Agent& a : sim.agents()) {
        if (a.mode != Agent::Mode::Driver || a.released || !a.moving) continue;
        carPos.push_back(a.pos);
        carSpeed.push_back(a.speed);
    }
    // Every person (other walkers + the on-foot player) for 360° SEPARATION —
    // spatial awareness beyond the planner's forward cone, so a body squeezes
    // AROUND a neighbour instead of clumping against it or brushing the player.
    std::vector<Vec2> people;
    std::vector<Entity> peopleOwner;
    world.each<Transform, CharacterController>([&](Entity e, Transform& t, CharacterController&) {
        people.push_back(Vec2(t.position.x, t.position.z));
        peopleOwner.push_back(e);
    });

    std::vector<CityRenderSystem::ExternalAgentPose> widgetPoses;
    widgetPoses.reserve(walkers_.size());

    for (Walker& w : walkers_) {
        if (!world.alive(w.entity)) continue;
        CharacterController* cc = world.get<CharacterController>(w.entity);
        Transform* t = world.get<Transform>(w.entity);
        if (!cc || !t || cc->characterId == engine::INVALID_CHARACTER) continue;
        if (w.agentId < 0 || w.agentId >= static_cast<int>(sim.agents().size())) continue;
        const Agent& g = sim.agents()[w.agentId];

        Vec3 pos = pw.characterPosition(cc->characterId);
        Vec2 posXZ(pos.x, pos.z);

        // Knockdown: a vehicle moving through the walker's body floors it.
        for (std::size_t c = 0; c < carPos.size(); ++c) {
            Vec2 d = carPos[c] - posXZ;
            if (d.length() < kKnockRadius && carSpeed[c] > kKnockSpeed) {
                w.knock.knock();
                break;
            }
        }
        bool down = w.knock.update(dt);

        // Body speed BEFORE this tick's move (the blocked watchdog's input).
        Vec3 velBefore = pw.characterVelocity(cc->characterId);
        Real hBefore = std::sqrt(velBefore.x * velBefore.x + velBefore.z * velBefore.z);

        // Walk toward the ghost's planned spot (station control, walker-simple):
        // speed grows with the distance behind the plan, capped at a brisk jog;
        // settles at a standoff. Plus 360° SEPARATION from nearby people — the
        // spatial awareness a forward cone lacks — so a walker squeezes around a
        // neighbour or the player instead of pressing into them.
        // Down (knocked): no intent. Backing off (blocked): stand still and let
        // the think cadence + tethered plan re-route before trying again.
        Vec3 desired;
        bool backingOff = w.backoff > 0;
        if (backingOff) w.backoff -= dt;
        if (!down && !backingOff) {
            Vec2 steer(0, 0);
            Vec2 to(g.pos.x - posXZ.x, g.pos.y - posXZ.y);
            Real d = to.length();
            if (d > kWalkStandoff) {
                Real speed = std::min(kWalkMax, kWalkGain * (d - kWalkStandoff));
                steer = Vec2(to.x / d * speed, to.y / d * speed);
            }
            for (std::size_t p = 0; p < people.size(); ++p) {
                if (peopleOwner[p] == w.entity) continue;
                Vec2 away = posXZ - people[p];
                Real ad = away.length();
                if (ad > 1e-4 && ad < kPersonalSpace) {
                    Real push = kSeparationGain * (kPersonalSpace - ad) / kPersonalSpace;
                    steer = steer + away * (push / ad);
                }
            }
            Real sl = steer.length();
            if (sl > kWalkMax) steer = steer * (kWalkMax / sl);
            desired = Vec3(steer.x, 0, steer.y);
            // Wants to walk but the body is pinned (crowd, parked car, wall):
            // stop trying for a beat — a standing person reads as "waiting", a
            // pinned person shoving reads as broken.
            Real commanded = std::sqrt(desired.x * desired.x + desired.z * desired.z);
            if (w.blocked.update(commanded, hBefore, dt)) {
                w.backoff = kBackoffTime;
                desired = Vec3();
            }
        }
        pw.moveCharacter(cc->characterId, desired, dt);   // zero intent still settles
        pos = pw.characterPosition(cc->characterId);
        posXZ = Vec2(pos.x, pos.z);

        // Face the actual travel direction once really moving.
        Vec3 vel = pw.characterVelocity(cc->characterId);
        Real hSpeed = std::sqrt(vel.x * vel.x + vel.z * vel.z);
        if (hSpeed > kFaceSpeed) {
            w.facing = Vec2(vel.x / hSpeed, vel.z / hSpeed);
        }

        // WALK CYCLE: the stride phase advances with the body's REAL speed, so
        // the legs move exactly as fast as the ground goes by — a held body
        // stands (pose 0), a knocked-down one keeps whatever it fell in.
        if (!down) {
            int pose = walkPoseIndex(w.stride, hSpeed, dt);
            if (Renderable* r = world.get<Renderable>(w.entity))
                r->mesh = poseMesh(ctx.assets, w.outfit, pose);
        }

        // Pose write-back. Standing: the box rides the capsule centre. Down: lay
        // the box flat at shin height, pitched forward over its facing (the
        // capsule itself stays standing — a v1 visual; ragdoll is future work).
        if (PrevTransform* pt = world.get<PrevTransform>(w.entity)) pt->value = *t;
        Real yaw = std::atan2(w.facing.x, w.facing.y);
        Quat yawQ = Quat::fromAxisAngle(Vec3(0, 1, 0), yaw);
        if (down) {
            t->position = Vec3(pos.x, pos.y - kCapsuleHalf, pos.z);
            t->orientation = yawQ * Quat::fromAxisAngle(Vec3(1, 0, 0), engine::PI * 0.5);
        } else {
            t->position = pos;
            t->orientation = yawQ;
        }

        // The plan waits for the body (never outruns a blocked/downed walker),
        // and the debug widgets ring the REAL walker.
        city_.simMutable().setAgentTether(w.agentId, posXZ, kTetherLead);
        // Body truth for the debug ring: a downed or blocked walker shows RED
        // whatever its planner ghost believes.
        int stateOverride = (down || w.backoff > 0)
                                ? static_cast<int>(Agent::State::Waiting) : -1;
        widgetPoses.push_back(CityRenderSystem::ExternalAgentPose{
            w.agentId, posXZ, w.facing, g.pos, stateOverride});
    }
    city_.setExternalPedPoses(std::move(widgetPoses));
}

void CityWalkerSystem::fixedUpdate(engine::FrameContext& ctx) {
    spawnWalkers(ctx);
    driveWalkers(ctx);
}

void CityWalkerSystem::onStop(engine::FrameContext&) {
    // Tracking only; world/physics teardown reclaims entities + characters (the
    // same no-removal-hook note as the vehicle bridge — fine at level teardown).
    walkers_.clear();
    poseMeshes_.clear();
}

}  // namespace citysim
