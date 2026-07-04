#include "city_player_body.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/world.h"
#include "city_meshes.h"

#include <cmath>
#include <string>

namespace citysim {

using engine::Entity;
using engine::MeshHandle;
using engine::Quat;
using engine::Real;
using engine::Renderable;
using engine::Transform;
using engine::PrevTransform;
using engine::Vec2;
using engine::Vec3;
using engine::World;

namespace {
constexpr Real kFaceSpeed = 0.3;        // turn to face travel above this (walkers')
constexpr Real kPersonHalfHeight = 0.9; // buildPersonMesh spans y in [-0.9, 0.9]
constexpr Real kTeleportJump = 3.0;     // a horizontal step past this in ONE tick is a
                                        // teleport (respawn/fall net), not locomotion —
                                        // ~30x a sprint tick; re-anchor, don't animate it
}  // namespace

engine::MeshHandle CityPlayerBodySystem::poseMesh(engine::FrameContext& ctx,
                                                  int pose, Real scale) {
    if (poseMeshes_.empty()) poseMeshes_.assign(kWalkPoses, MeshHandle{});
    if (poseMeshes_[pose].valid()) return poseMeshes_[pose];
    // The walker person mesh in the player's own outfit, scaled so its 1.8 m
    // envelope matches the player capsule (the Transform stays unit-scale —
    // nothing else on the player is resized). Its own dedup key: the player
    // outfit is reserved (never dealt to a walker), so no sharing is lost.
    engine::RenderMesh m = buildPersonMesh(walkPoseSwing(pose), playerOutfit());
    for (engine::Vertex& v : m.vertices) v.position = v.position * scale;
    poseMeshes_[pose] =
        ctx.assets.acquireMesh(m, "citysim:playerbody:" + std::to_string(pose));
    return poseMeshes_[pose];
}

void CityPlayerBodySystem::fixedUpdate(engine::FrameContext& ctx) {
    World& world = ctx.world;

    // The (first) player: the entity PlayerSystem drives. Collected before any
    // structural change (World::each contract, ADR-0006).
    Entity player;
    world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform&, engine::CharacterController&,
            engine::ControlledBy&) {
            if (!player.valid()) player = e;
        });

    // The body draws only over the third-person shoulder camera and on foot:
    // first person must not put a head in the camera, and a seated driver is
    // already drawn by the car's driver model.
    bool visible = player.valid() &&
                   ctx.settings.getBool("playerThirdPerson", false) &&
                   !world.has<engine::InVehicle>(player);
    if (!visible) {
        if (bodyOn_ && player.valid() && world.has<Renderable>(player))
            world.remove<Renderable>(player);
        bodyOn_ = false;   // re-anchor speed/pose sampling on re-entry
        return;
    }

    Transform* t = world.get<Transform>(player);
    auto* cc = world.get<engine::CharacterController>(player);
    if (!t || !cc) return;
    Real dt = ctx.clock.fixedStep();
    if (dt <= 0) return;

    // Transform.position is the capsule CENTRE; the person mesh is centred the
    // same way, so matching half-heights puts the feet on the ground.
    Real scale = (cc->halfHeight + cc->radius) / kPersonHalfHeight;

    if (!bodyOn_) {
        Renderable r;
        r.mesh = poseMesh(ctx, 0, scale);
        r.material.albedo = Vec3(1, 1, 1);   // hue in vertex colours, like walkers
        r.material.metallic = 0.0f;
        r.material.roughness = 0.9f;
        r.material.opacity = 1.0f;
        if (!world.has<Renderable>(player)) world.add<Renderable>(player, r);
        bodyOn_ = true;
        lastPos_ = t->position;
        prevPose_ = *t;
        stride_ = 0;
    }

    // WALK CYCLE — exactly the walker rule (shared walkPoseIndex): the stride
    // phase advances with the body's REAL horizontal speed, here measured from
    // the capsule's motion this tick (PlayerSystem already moved it), so a held
    // body stands and the legs match the ground speed.
    Vec3 d = t->position - lastPos_;
    lastPos_ = t->position;
    Real hDist = std::sqrt(d.x * d.x + d.z * d.z);

    // Teleport re-anchor (respawn / fall-safety net move the capsule in one
    // tick, not the player walking): snap the speed sampling AND the render
    // interpolation to the new spot, so the body doesn't spin its legs up or
    // streak across the map for a frame. Facing is held; no next-tick lag since
    // lastPos_/prevPose_ both become the new position here.
    if (hDist > kTeleportJump) {
        if (Renderable* r = world.get<Renderable>(player))
            r->mesh = poseMesh(ctx, 0, scale);   // stand
        if (PrevTransform* pt = world.get<PrevTransform>(player)) pt->value = *t;
        prevPose_ = *t;
        stride_ = 0;
        return;
    }

    Real hSpeed = hDist / dt;
    int pose = walkPoseIndex(stride_, hSpeed, dt);
    if (Renderable* r = world.get<Renderable>(player))
        r->mesh = poseMesh(ctx, pose, scale);

    // Face the actual travel direction once really moving (walker rule); the
    // camera heading is free to look around a standing body.
    if (hSpeed > kFaceSpeed && hDist > 1e-6) {
        facing_ = Vec2(d.x / hDist, d.z / hDist);
    }

    // Pose write-back with render interpolation: prev gets LAST tick's drawn
    // pose (PlayerSystem updates the position before us, so *t is already this
    // tick's). Orientation rides the player Transform — physics ignores it
    // (the capsule doesn't rotate) and nothing else writes it.
    if (PrevTransform* pt = world.get<PrevTransform>(player)) pt->value = prevPose_;
    t->orientation =
        Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(facing_.x, facing_.y));
    prevPose_ = *t;
}

void CityPlayerBodySystem::onStop(engine::FrameContext&) {
    // Tracking only; world/assets teardown reclaims the entity + meshes (the
    // same no-removal-hook note as the walker bridge — fine at level teardown).
    poseMeshes_.clear();
    bodyOn_ = false;
    stride_ = 0;
    facing_ = Vec2(0, 1);
}

}  // namespace citysim
