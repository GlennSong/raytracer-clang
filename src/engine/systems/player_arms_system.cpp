#include "player_arms_system.h"

#include "../asset_manager.h"
#include "../components.h"
#include "../interaction.h"   // Seated
#include "../mesh_builder.h"
#include "../player_pose.h"
#include "../procgen/person.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace engine {

namespace {

constexpr Real kTau = 6.283185307179586;
constexpr Real kSleeve = 0.075;   // the arm's block, square
constexpr Real kOut = -100000.0;  // stowed: far below the world, like the gun viewmodel

// local +Z along `dir`, +Y toward `up`
Quat facing(const Vec3& dir, const Vec3& up) {
    const Vec3 f = normalize(dir);
    Vec3 r = cross(up, f);
    r = r.length() < 1e-4 ? Vec3(1, 0, 0) : normalize(r);
    const Vec3 u = cross(f, r);
    Mat4 m;
    m.m[0][0] = r.x; m.m[1][0] = r.y; m.m[2][0] = r.z;
    m.m[0][1] = u.x; m.m[1][1] = u.y; m.m[2][1] = u.z;
    m.m[0][2] = f.x; m.m[1][2] = f.y; m.m[2][2] = f.z;
    return Quat::fromRotationMatrix(m);
}

// One arm's front crawl at phase 0..1, camera space with +x outward: keyframes eased between.
Vec3 crawl(Real phase) {
    struct Key { Real at; Vec3 p; };
    static const Key keys[] = {
        {0.00, Vec3(0.17, -0.04, 0.48)},   // the hand goes in, ahead
        {0.30, Vec3(0.13, -0.08, 0.72)},   // stretched out straight in front
        {0.45, Vec3(0.17, -0.24, 0.58)},   // the catch
        {0.62, Vec3(0.24, -0.55, 0.12)},   // the pull, down and back past the hip: out of sight
        {0.82, Vec3(0.42, 0.02, 0.18)},    // over the top, out to the side
        {1.00, Vec3(0.17, -0.04, 0.48)},
    };
    for (int i = 0; i + 1 < 6; ++i)
        if (phase <= keys[i + 1].at) {
            Real t = (phase - keys[i].at) / (keys[i + 1].at - keys[i].at);
            t = t * t * (3 - 2 * t);
            return keys[i].p + (keys[i + 1].p - keys[i].p) * t;
        }
    return keys[0].p;
}

}  // namespace

void PlayerArmsSystem::update(FrameContext& ctx) {
    World& world = ctx.world;
    Entity player;
    world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform&, CharacterController&, ControlledBy&) {
            if (!player.valid()) player = e;
        });
    const PlayerPose* pose = player.valid() ? world.get<PlayerPose>(player) : nullptr;
    // ...and only while the view IS the player's eye: a spectate camera, a cutscene, the freecam are elsewhere
    const Transform* body = player.valid() ? world.get<Transform>(player) : nullptr;
    const bool atEye = body && (ctx.view.camera.position - body->position).length() < 1.2;
    const bool visible = pose && atEye && pose->mode != PlayerPose::Mode::Hidden && !ctx.xr.active &&
                         !ctx.settings.getBool("playerThirdPerson", false) && !world.has<InVehicle>(player) &&
                         !world.has<Seated>(player) && !ctx.view.activeCameraEntity.valid();

    if (!made_) {
        if (!visible) return;
        const MeshHandle box = ctx.assets.acquireMesh(MeshBuilder::box(Vec3(1, 1, 1)), "player:arm_block");
        const PersonOutfit& outfit = personOutfit(personOutfitTotal() - 1);   // the player's red jacket
        for (int i = 0; i < 4; ++i) {
            parts_[i] = world.create();
            world.add<Transform>(parts_[i], Transform{});
            world.add<PrevTransform>(parts_[i], PrevTransform{Transform{}});
            Renderable r;
            r.mesh = box;
            r.material.albedo = i < 2 ? outfit.shirt : outfit.skin;
            r.material.roughness = 0.85f;
            r.material.metallic = 0.0f;
            r.material.opacity = 1.0f;
            world.add<Renderable>(parts_[i], r);
        }
        made_ = true;
    }
    auto place = [&](int i, const Vec3& at, const Quat& q, const Vec3& scale) {
        Transform* t = world.get<Transform>(parts_[i]);
        PrevTransform* p = world.get<PrevTransform>(parts_[i]);
        if (!t) return;
        t->position = at;
        t->orientation = q;
        t->scale = scale;
        if (p) p->value = *t;   // glued to the view: no interpolation smear
    };
    if (!visible) {
        if (shown_)
            for (int i = 0; i < 4; ++i) place(i, Vec3(0, kOut, 0), Quat::identity(), Vec3(1, 1, 1));
        shown_ = false;
        haveLocal_ = false;
        return;
    }
    shown_ = true;

    for (int k = 1; k <= 4; ++k)   // (only reached in first person, where the gun script takes the keys too)
        if (ctx.actions.pressed("slot_" + std::to_string(k))) slot_ = k;

    const CameraState& cam = ctx.view.camera;
    const Vec3 eye = cam.position;
    const Vec3 fwd = normalize(cam.target - cam.position);
    Vec3 right = cross(fwd, Vec3(0, 1, 0));
    right = right.length() < 1e-4 ? Vec3(1, 0, 0) : normalize(right);
    const Vec3 up = cross(right, fwd);
    auto toLocal = [&](const Vec3& w) {
        const Vec3 d = w - eye;
        return Vec3(dot(d, right), dot(d, up), dot(d, fwd));
    };
    auto toWorld = [&](const Vec3& l) { return eye + right * l.x + up * l.y + fwd * l.z; };

    const Real dt = std::clamp(static_cast<Real>(ctx.frameDelta), Real(0), Real(0.1));
    const bool held = pose->mode == PlayerPose::Mode::Mantle || pose->mode == PlayerPose::Mode::Climb ||
                      pose->mode == PlayerPose::Mode::Ladder;
    // THE GUN, where gun.lua's viewmodel puts it: 0.35 ahead of the eye, 0.18 right, 0.18 down, barrel along
    // the view. The hands go on it: the right on the grip, the left under the barrel.
    const bool gun = slot_ == 2 && (pose->mode == PlayerPose::Mode::Walk || pose->mode == PlayerPose::Mode::Air);
    Vec3 gunGrip[2];
    if (gun) {
        const Vec3 at = eye + fwd * 0.35 + right * 0.18 + Vec3(0, -0.18, 0);
        const Vec3 gf = fwd;
        Vec3 gr = cross(Vec3(0, 1, 0), gf);
        gr = gr.length() < 1e-4 ? Vec3(1, 0, 0) : normalize(gr);
        const Vec3 gu = cross(gf, gr);
        gunGrip[0] = toLocal(at + gu * -0.03 + gf * 0.3 + gr * -0.02);    // left: cupping the barrel, past the receiver
        gunGrip[1] = toLocal(at + gu * -0.15 + gf * -0.11);                 // right: on the grip
    }
    for (int k = 0; k < 2; ++k) {
        const Real s = k == 0 ? -1.0 : 1.0;   // left, right
        Vec3 want;
        switch (pose->mode) {
            case PlayerPose::Mode::Mantle:
            case PlayerPose::Mode::Climb:
            case PlayerPose::Mode::Ladder: {
                Vec3 grip = toLocal(k == 0 ? pose->handL : pose->handR);
                if (pose->mode == PlayerPose::Mode::Mantle && pose->progress > 0.75) {
                    // over the top: the hands push down and come away
                    const Real e = (pose->progress - 0.75) / 0.25;
                    grip = grip + (Vec3(s * 0.22, -0.5, 0.35) - grip) * e;
                }
                want = grip;
                break;
            }
            case PlayerPose::Mode::Swim: {
                // THE CRAWL, half a stroke apart (Glenn: "those arms make it look like you're going backwards ...
                // the arm should extend out"): what you see is the arm coming over from the side, going in ahead
                // and stretching out straight in front; the pull that drives you happens below the view.
                const Real p = pose->swimPhase + (k == 0 ? 0.0 : 0.5);
                want = crawl(p - std::floor(p));
                want.x *= s;
                break;
            }
            case PlayerPose::Mode::Air:
                want = Vec3(s * 0.3, -0.16, 0.45);   // out for balance
                break;
            default: {
                // walking: the hands swing opposite the legs, low at the edge of the view; running pumps them
                const Real amp = std::clamp(pose->speed / 6.0, Real(0), Real(1));
                const Real swing = std::sin(kTau * pose->stride + (k == 0 ? 0.0 : kTau * 0.5));
                // (the view's lower edge is ~0.58 x the distance ahead below the eye: a hand at 0.45 ahead shows
                // above -0.26)
                if (pose->run && pose->speed > 3.0)
                    want = Vec3(s * 0.2, -0.22 + 0.04 * std::fabs(swing), 0.46 + 0.16 * swing);
                else
                    want = Vec3(s * 0.24, -0.38 + 0.15 * amp + 0.03 * amp * swing, 0.42 + 0.13 * amp * swing);
                want.y -= 0.1 * pose->landed;   // a landing's jolt
                break;
            }
        }
        if (gun) want = gunGrip[k];
        if (!haveLocal_) local_[k] = want;
        // planted hands follow the grip fast (they hold the wall while the view moves); free ones ease
        const Real rate = held || gun ? 30.0 : 14.0;
        local_[k] = local_[k] + (want - local_[k]) * (1.0 - std::exp(-dt * rate));

        const Vec3 hand = toWorld(local_[k]);
        const Vec3 shoulder = toWorld(Vec3(s * 0.21, -0.26, -0.04));
        Vec3 along = hand - shoulder;
        const Real len = std::max(along.length(), Real(0.05));
        along = along * (1.0 / len);
        const Quat q = facing(along, up);
        const Real sleeve = std::max(len - 0.07, Real(0.02));
        place(k, shoulder + along * (sleeve * 0.5), q, Vec3(kSleeve, kSleeve, sleeve));
        place(2 + k, hand, q, Vec3(0.07, 0.05, 0.1));
    }
    haveLocal_ = true;
}

}  // namespace engine
