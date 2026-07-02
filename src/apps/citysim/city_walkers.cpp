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

// A small clothing palette, picked per walker by a deterministic hash.
const Vec3 kClothes[] = {
    Vec3(0.62, 0.35, 0.25), Vec3(0.30, 0.42, 0.58), Vec3(0.38, 0.50, 0.32),
    Vec3(0.55, 0.48, 0.30), Vec3(0.45, 0.32, 0.48), Vec3(0.60, 0.58, 0.55),
};
constexpr int kNumClothes = static_cast<int>(sizeof(kClothes) / sizeof(kClothes[0]));
}  // namespace

void CityWalkerSystem::spawnWalkers(engine::FrameContext& ctx) {
    if (spawned_ || !city_.built()) return;
    World& world = ctx.world;
    const CitySim& sim = city_.sim();

    if (!bodyMesh_.valid())
        bodyMesh_ = ctx.assets.acquireMesh(
            engine::MeshBuilder::box(Vec3(0.5, 1.8, 0.5)), "citywalk:ped");

    const auto& agents = sim.agents();
    for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
        const Agent& a = agents[i];
        if (a.mode != Agent::Mode::Pedestrian) continue;

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

        Renderable r;
        r.mesh = bodyMesh_;
        r.material.albedo = kClothes[(static_cast<uint32_t>(i) * 2654435761u >> 8) %
                                     kNumClothes];
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
        w.facing = a.heading;
        walkers_.push_back(w);
    }
    spawned_ = true;
}

void CityWalkerSystem::driveWalkers(engine::FrameContext& ctx) {
    World& world = ctx.world;
    const CitySim& sim = city_.sim();
    PhysicsWorld& pw = physics_.physicsWorld();
    Real dt = ctx.clock.fixedStep();

    // Vehicle poses + speeds once for all walkers (the knockdown trigger).
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

        // Walk toward the ghost's planned spot (station control, walker-simple):
        // speed grows with the distance behind the plan, capped at a brisk jog;
        // settles at a standoff. Down: no intent — the body just lies there.
        Vec3 desired;
        if (!down) {
            Vec2 to(g.pos.x - posXZ.x, g.pos.y - posXZ.y);
            Real d = to.length();
            if (d > kWalkStandoff) {
                Real speed = std::min(kWalkMax, kWalkGain * (d - kWalkStandoff));
                desired = Vec3(to.x / d, 0, to.y / d) * speed;
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
        widgetPoses.push_back(CityRenderSystem::ExternalAgentPose{
            w.agentId, posXZ, w.facing, g.pos});   // goal = the plan's spot
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
    spawned_ = false;
}

}  // namespace citysim
