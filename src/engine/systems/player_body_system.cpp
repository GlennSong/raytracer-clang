#include "player_body_system.h"

#include "../asset_manager.h"
#include "../components.h"
#include "../interaction.h"   // Seated
#include "../mesh_builder.h"
#include "../player_pose.h"
#include "../procgen/person.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

constexpr Real kTau = 6.283185307179586;
constexpr Real kDeg = 3.14159265358979 / 180.0;

Vec3 flat(const Vec3& v) {
    const Real l = std::sqrt(v.x * v.x + v.z * v.z);
    return l > 1e-6 ? Vec3(v.x / l, 0, v.z / l) : Vec3(0, 0, 0);
}

Quat yawTo(const Vec3& face) { return Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(face.x, face.z)); }

void addBlock(RenderMesh& m, const Vec3& centre, const Vec3& size, const Vec3& colour) {
    MeshBuilder::emitBox(m, MeshFrame::centred(centre, size), colour);
}

// A SKATEBOARD along +z, its wheels on y = 0: a deck, kicked-up tail and nose, four wheels
RenderMesh buildSkateboard() {
    RenderMesh m;
    const Vec3 deck(0.16, 0.1, 0.12), wheel(0.92, 0.9, 0.82), truck(0.6, 0.6, 0.62);
    addBlock(m, Vec3(0, 0.1, 0), Vec3(0.21, 0.025, 0.62), deck);
    addBlock(m, Vec3(0, 0.125, 0.36), Vec3(0.21, 0.025, 0.14), deck);    // nose
    addBlock(m, Vec3(0, 0.125, -0.36), Vec3(0.21, 0.025, 0.14), deck);   // tail
    for (Real z : {-0.24, 0.24}) {
        addBlock(m, Vec3(0, 0.065, z), Vec3(0.16, 0.04, 0.04), truck);
        for (Real x : {-0.085, 0.085}) addBlock(m, Vec3(x, 0.03, z), Vec3(0.035, 0.06, 0.06), wheel);
    }
    return m;
}

// A KICK SCOOTER along +z: a low deck, the stem up from the front, the bar across
RenderMesh buildScooter() {
    RenderMesh m;
    const Vec3 deck(0.15, 0.16, 0.18), metal(0.7, 0.72, 0.75), grip(0.08, 0.08, 0.09), wheel(0.2, 0.2, 0.22);
    addBlock(m, Vec3(0, 0.07, 0), Vec3(0.13, 0.035, 0.6), deck);
    addBlock(m, Vec3(0, 0.55, 0.34), Vec3(0.04, 0.95, 0.04), metal);    // the stem
    addBlock(m, Vec3(0, 1.02, 0.34), Vec3(0.5, 0.035, 0.035), metal);   // the bar
    for (Real x : {-0.22, 0.22}) addBlock(m, Vec3(x, 1.02, 0.34), Vec3(0.09, 0.045, 0.045), grip);
    for (Real z : {-0.3, 0.34}) addBlock(m, Vec3(0, 0.05, z), Vec3(0.03, 0.1, 0.1), wheel);
    return m;
}

}  // namespace

void PlayerBodySystem::drawBoards(FrameContext& ctx, const PlayerPose* p, const Transform* body, Real lift) {
    World& world = ctx.world;
    const bool on = p && body && p->mode == PlayerPose::Mode::Board;
    if (!boardsMade_) {
        if (!on) return;
        const MeshHandle meshes[2] = {ctx.assets.acquireMesh(buildSkateboard(), "player:skateboard"),
                                      ctx.assets.acquireMesh(buildScooter(), "player:scooter")};
        for (int i = 0; i < 2; ++i) {
            board_[i] = world.create();
            Transform away;
            away.position = Vec3(0, -100000, 0);
            world.add<Transform>(board_[i], away);
            world.add<PrevTransform>(board_[i], PrevTransform{away});
            Renderable r;
            r.mesh = meshes[i];
            r.material.albedo = Vec3(1, 1, 1);   // (colour in the vertices)
            r.material.roughness = 0.6f;
            r.material.metallic = 0.0f;
            r.material.opacity = 1.0f;
            world.add<Renderable>(board_[i], r);
        }
        boardsMade_ = true;
    }
    for (int i = 0; i < 2; ++i) {
        Transform* t = world.get<Transform>(board_[i]);
        PrevTransform* pt = world.get<PrevTransform>(board_[i]);
        if (!t) continue;
        const bool here = on && p->board == i;
        const bool wasAway = t->position.y < -1000;
        if (pt) pt->value = *t;
        if (here) {
            t->position = body->position - Vec3(0, lift, 0);   // wheels on the ground under the feet
            t->orientation = yawTo(p->boardHeading);
        } else {
            t->position = Vec3(0, -100000, 0);
        }
        if (pt && (wasAway || !here)) pt->value = *t;
    }
}

void PlayerBodySystem::fixedUpdate(FrameContext& ctx) {
    World& world = ctx.world;
    Entity player;
    world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform&, CharacterController&, ControlledBy&) {
            if (!player.valid()) player = e;
        });
    const PlayerPose* p = player.valid() ? world.get<PlayerPose>(player) : nullptr;
    const Transform* body = player.valid() ? world.get<Transform>(player) : nullptr;
    const CharacterController* cc = player.valid() ? world.get<CharacterController>(player) : nullptr;
    const bool ragdoll = p && p->mode == PlayerPose::Mode::Ragdoll;
    drawBoards(ctx, p, body, cc ? cc->halfHeight + cc->radius : 0.7);
    const bool visible = p && body && cc && drawnAsBlocks(p->mode) && !world.has<InVehicle>(player) &&
                         !world.has<Seated>(player) &&
                         (ragdoll || ctx.settings.getBool("playerThirdPerson", false));
    if (!made_) {
        if (!visible) return;
        const MeshHandle box = ctx.assets.acquireMesh(MeshBuilder::box(Vec3(1, 1, 1)), "player:arm_block");
        const PersonOutfit& o = personOutfit(personOutfitTotal() - 1);
        for (int i = 0; i < BlockBody::kParts; ++i) {
            parts_[i] = world.create();
            Transform away;
            away.position = Vec3(0, -100000, 0);
            world.add<Transform>(parts_[i], away);
            world.add<PrevTransform>(parts_[i], PrevTransform{away});
            Renderable r;
            r.mesh = box;
            r.material.albedo = i == BlockBody::Head ? o.skin : i >= BlockBody::LegL ? o.pants : o.shirt;
            r.material.roughness = 0.9f;
            r.material.metallic = 0.0f;
            r.material.opacity = 1.0f;
            world.add<Renderable>(parts_[i], r);
        }
        made_ = true;
    }
    auto place = [&](int i, const Vec3& at, const Quat& q, const Vec3& size) {
        Transform* t = world.get<Transform>(parts_[i]);
        PrevTransform* pt = world.get<PrevTransform>(parts_[i]);
        if (!t) return;
        if (pt) pt->value = *t;   // (rendered between last update's pose and this one)
        t->position = at;
        t->orientation = q;
        t->scale = size;
    };
    if (!visible) {
        if (shown_)
            for (int i = 0; i < BlockBody::kParts; ++i) {
                place(i, Vec3(0, -100000, 0), Quat::identity(), Vec3(1, 1, 1));
                if (PrevTransform* pt = world.get<PrevTransform>(parts_[i])) pt->value = *world.get<Transform>(parts_[i]);
            }
        shown_ = false;
        return;
    }
    const bool firstShow = !shown_;
    shown_ = true;
    clock_ += ctx.clock.fixedStep();

    std::array<PartTransform, BlockBody::kParts> blocks;
    if (ragdoll) {
        const Real s = (cc->halfHeight + cc->radius) / BlockBody::kHalfHeight;
        for (int i = 0; i < BlockBody::kParts; ++i) {
            blocks[static_cast<std::size_t>(i)].position = p->ragdollPos[i];
            blocks[static_cast<std::size_t>(i)].orientation = p->ragdollRot[i];
            blocks[static_cast<std::size_t>(i)].size = BlockBody::parts()[static_cast<std::size_t>(i)].size * s;
        }
    } else {
        BodyPose pose;
        pose.position = body->position;
        pose.scale = (cc->halfHeight + cc->radius) / BlockBody::kHalfHeight;
        // which way the body faces: the wall it holds, else the way it moves, else as it was
        Vec3 face = facing_;
        const Vec3 moving = flat(p->velocity);
        const Real hspeed = std::sqrt(p->velocity.x * p->velocity.x + p->velocity.z * p->velocity.z);
        switch (p->mode) {
            case PlayerPose::Mode::Mantle:
            case PlayerPose::Mode::Climb:
            case PlayerPose::Mode::Ladder:
                if (flat(p->wallNormal).lengthSquared() > 0.5) face = flat(p->wallNormal) * -1.0;
                break;
            case PlayerPose::Mode::Parachute:
                if (flat(p->canopyHeading).lengthSquared() > 0.5) face = flat(p->canopyHeading);
                break;
            case PlayerPose::Mode::Board:
                face = flat(p->boardHeading);
                break;
            case PlayerPose::Mode::PlaySwing:
            case PlayerPose::Mode::Bars:
                if (flat(p->wallNormal).lengthSquared() > 0.5) face = flat(p->wallNormal);
                break;
            default:
                if (hspeed > 0.5) face = moving;
                break;
        }
        facing_ = face;
        Quat lean = Quat::identity();   // about the body's x: +ve tips the head forward
        const Real cyc = std::sin(p->climbCycle * 3.14159265358979);
        switch (p->mode) {
            case PlayerPose::Mode::Mantle:
            case PlayerPose::Mode::Climb:
            case PlayerPose::Mode::Ladder: {
                lean = Quat::fromAxisAngle(Vec3(1, 0, 0), 8 * kDeg);
                pose.orientation = yawTo(face) * lean;
                pose.armL = reachToward(pose, BlockBody::ArmL, p->handL);
                pose.armR = reachToward(pose, BlockBody::ArmR, p->handR);
                if (p->mode == PlayerPose::Mode::Mantle) {
                    const Vec3 hang = normalize(Vec3(0, -1, 0.15)), knee = normalize(Vec3(0, -0.35, 1));
                    const Real e = std::clamp((p->progress - 0.55) / 0.3, Real(0), Real(1));
                    pose.legL = normalize(hang + (knee - hang) * e);
                    pose.legR = hang;
                } else {
                    // a foot up, a foot down, in turn as the hands go
                    pose.legL = normalize(Vec3(0, -1, 0.3 + 0.35 * std::max(Real(0), cyc)));
                    pose.legR = normalize(Vec3(0, -1, 0.3 + 0.35 * std::max(Real(0), -cyc)));
                }
                break;
            }
            case PlayerPose::Mode::Swim: {
                // prone, the crawl's arms round over the back, the feet fluttering; lifted so the back is at the
                // waterline (the capsule floats with its centre under the surface, the eye just over it)
                pose.orientation = yawTo(face) * Quat::fromAxisAngle(Vec3(1, 0, 0), 78 * kDeg);
                pose.position = pose.position + Vec3(0, 0.38 * pose.scale / 0.78, 0);
                for (int k = 0; k < 2; ++k) {
                    const Real ph = p->swimPhase + (k ? 0.5 : 0.0);
                    const Real a = kTau * (ph - std::floor(ph));
                    const Vec3 d(k ? -0.12 : 0.12, std::cos(a), std::sin(a));
                    (k ? pose.armR : pose.armL) = normalize(d);
                }
                const Real kick = std::sin(clock_ * 9.0);
                pose.legL = normalize(Vec3(0.04, -1, 0.15 * kick));
                pose.legR = normalize(Vec3(-0.04, -1, -0.15 * kick));
                break;
            }
            case PlayerPose::Mode::Parachute:
                pose.orientation = yawTo(face);
                pose.armL = normalize(Vec3(0.3, 1, 0.25));   // up on the toggles
                pose.armR = normalize(Vec3(-0.3, 1, 0.25));
                pose.legL = normalize(Vec3(0.05, -1, 0.25));
                pose.legR = normalize(Vec3(-0.05, -1, 0.15));
                break;
            case PlayerPose::Mode::Wingsuit: {
                // flat to the air along the way it flies: head first and down as it dives
                const Real speed = p->velocity.length();
                const Real dive = speed > 1 ? std::asin(std::clamp(p->velocity.y / speed, Real(-1), Real(1))) : 0;
                const Real tilt = std::clamp(90 * kDeg - dive, 60 * kDeg, 150 * kDeg);
                pose.orientation = yawTo(face) * Quat::fromAxisAngle(Vec3(1, 0, 0), tilt);
                pose.armL = normalize(Vec3(1, 0.2, 0));
                pose.armR = normalize(Vec3(-1, 0.2, 0));
                pose.legL = normalize(Vec3(0.35, -1, 0));
                pose.legR = normalize(Vec3(-0.35, -1, 0));
                break;
            }
            case PlayerPose::Mode::PlaySwing: {
                // sat on the seat, hands up on the chains, the legs out on the swing forward and tucked on the back
                const Real th = p->progress;
                pose.orientation = yawTo(face) * Quat::fromAxisAngle(Vec3(1, 0, 0), -0.6 * th - 0.1);
                pose.armL = reachToward(pose, BlockBody::ArmL, p->handL);
                pose.armR = reachToward(pose, BlockBody::ArmR, p->handR);
                const Vec3 legs = normalize(Vec3(0, -0.45 + 0.4 * std::max(Real(0), th), 1.0 - 0.6 * std::max(Real(0), -th)));
                pose.legL = normalize(legs + Vec3(0.05, 0, 0));
                pose.legR = normalize(legs - Vec3(0.05, 0, 0));
                break;
            }
            case PlayerPose::Mode::Bars: {
                // hanging off the bar, the legs swinging a little with each reach
                pose.orientation = yawTo(face);
                pose.armL = reachToward(pose, BlockBody::ArmL, p->handL);
                pose.armR = reachToward(pose, BlockBody::ArmR, p->handR);
                const Real sw = 0.25 * std::sin(clock_ * 3.0);
                pose.legL = normalize(Vec3(0.04, -1, sw));
                pose.legR = normalize(Vec3(-0.04, -1, sw));
                break;
            }
            case PlayerPose::Mode::Board: {
                if (p->board == 1) {
                    // the scooter: facing ahead, hands on the bar, a foot kicking back off the ground
                    pose.orientation = yawTo(face) * Quat::fromAxisAngle(Vec3(1, 0, 0), 6 * kDeg);
                    pose.armL = reachToward(pose, BlockBody::ArmL, p->handL);
                    pose.armR = reachToward(pose, BlockBody::ArmR, p->handR);
                    const Real k = std::sin(p->kick * 3.14159265358979);
                    pose.legL = normalize(Vec3(0.05, -1, 0.05));
                    pose.legR = normalize(Vec3(-0.08, -1, -0.6 * k));
                } else {
                    // the skateboard: side-on to the way it rolls, knees soft, arms out; the back foot kicks
                    const Vec3 side(face.z, 0, -face.x);   // the stance: the body's front faces off the board's left
                    pose.orientation = yawTo(side * -1.0);
                    const Real k = std::sin(p->kick * 3.14159265358979);
                    pose.legL = normalize(Vec3(0.28, -1, 0));                 // the front foot (toward the nose)
                    pose.legR = normalize(Vec3(-0.28 - 0.4 * k, -1, 0.25 * k));   // the back foot, kicking
                    pose.armL = normalize(Vec3(1, -0.4, 0));
                    pose.armR = normalize(Vec3(-1, -0.4, 0));
                }
                break;
            }
            case PlayerPose::Mode::Grapple: {
                // hanging off the right hand, the body swung along the rope a little, legs trailing
                const Vec3 rope = p->anchor - body->position;
                const Real swingLean = rope.length() > 1e-3 ? std::asin(std::clamp(dot(normalize(rope), face) * 0.6,
                                                                                    Real(-0.6), Real(0.6)))
                                                            : 0;
                pose.orientation = yawTo(face) * Quat::fromAxisAngle(Vec3(1, 0, 0), swingLean);
                pose.armR = reachToward(pose, BlockBody::ArmR, p->anchor);
                pose.armL = normalize(Vec3(0.8, -0.4, 0.1));
                pose.legL = normalize(Vec3(0.1, -1, -0.3));
                pose.legR = normalize(Vec3(-0.1, -1, -0.15));
                break;
            }
            default:
                pose.orientation = yawTo(face);
                break;
        }
        blocks = poseBlocks(pose);
    }
    for (int i = 0; i < BlockBody::kParts; ++i) {
        const PartTransform& b = blocks[static_cast<std::size_t>(i)];
        place(i, b.position, b.orientation, b.size);
        if (firstShow)
            if (PrevTransform* pt = world.get<PrevTransform>(parts_[i])) pt->value = *world.get<Transform>(parts_[i]);
    }
}

}  // namespace engine
