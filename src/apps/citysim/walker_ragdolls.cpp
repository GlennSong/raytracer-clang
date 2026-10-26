#include "walker_ragdolls.h"

#include "../../engine/asset_manager.h"
#include "../../engine/components.h"
#include "../../engine/mesh_builder.h"
#include "../../engine/procgen/person.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace citysim {

using engine::Entity;
using engine::PhysicsWorld;
using engine::Quat;
using engine::Real;
using engine::Vec2;
using engine::Vec3;
using engine::World;

namespace {
constexpr Real kStillFor = 1.0;    // s lying still before it gets up
constexpr Real kMaxDown = 6.0;     // s at most, still or not
constexpr Real kOut = -100000.0;   // a free slot's blocks: far below the world
}  // namespace

int WalkerRagdolls::knock(World& world, engine::AssetManager& assets, PhysicsWorld& pw, int owner, const Vec3& centre,
                          Vec2 facing, Real scale, int outfit, const Vec3& velocity) {
    // a free slot, else the one down longest
    int pick = -1;
    Real oldest = -1;
    for (int i = 0; i < kSlots; ++i) {
        if (slots_[static_cast<std::size_t>(i)].owner == owner) { pick = i; break; }   // hit again: re-thrown
        if (slots_[static_cast<std::size_t>(i)].owner < 0) { pick = i; oldest = 1e9; break; }
        if (slots_[static_cast<std::size_t>(i)].age > oldest) {
            oldest = slots_[static_cast<std::size_t>(i)].age;
            pick = i;
        }
    }
    if (pick < 0) return -1;
    Slot& s = slots_[static_cast<std::size_t>(pick)];
    if (s.body != PhysicsWorld::INVALID_RAGDOLL) pw.removeRagdoll(s.body);
    assets_ = &assets;
    engine::BodyPose pose;
    pose.position = centre;
    pose.scale = scale;
    pose.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(facing.x, facing.y));
    s.body = pw.addRagdoll(engine::blockRagdoll(pose), velocity);
    if (s.body == PhysicsWorld::INVALID_RAGDOLL) return -1;
    if (!s.made) {
        const engine::MeshHandle box = assets.acquireMesh(engine::MeshBuilder::box(Vec3(1, 1, 1)), "player:arm_block");
        for (Entity& e : s.parts) {
            e = world.create();
            engine::Transform away;
            away.position = Vec3(0, kOut, 0);
            world.add<engine::Transform>(e, away);
            world.add<engine::PrevTransform>(e, engine::PrevTransform{away});
            engine::Renderable r;
            r.mesh = box;
            r.material.roughness = 0.9f;
            r.material.metallic = 0.0f;
            r.material.opacity = 1.0f;
            world.add<engine::Renderable>(e, r);
        }
        s.made = true;
    }
    const engine::PersonOutfit& o = engine::personOutfit(outfit);
    for (int i = 0; i < engine::BlockBody::kParts; ++i)
        if (engine::Renderable* r = world.get<engine::Renderable>(s.parts[static_cast<std::size_t>(i)]))
            r->material.albedo = i == engine::BlockBody::Head ? o.skin : engine::BlockBody::isLeg(i) ? o.pants : o.shirt;
    s.owner = owner;
    s.age = 0;
    s.still = 0;
    s.scale = scale;
    return pick;
}

std::vector<WalkerRagdolls::Up> WalkerRagdolls::step(World& world, PhysicsWorld& pw, Real dt) {
    std::vector<Up> up;
    for (Slot& s : slots_) {
        if (s.owner < 0) continue;
        s.age += dt;
        s.still = pw.ragdollSpeed(s.body) < 0.4 ? s.still + dt : 0;
        // the parts where physics has them, drawn as the player's are: blocks and skinned limbs (body_draw.h)
        std::array<engine::PartTransform, engine::BlockBody::kParts> blocks;
        bool all = true;
        for (int i = 0; i < engine::BlockBody::kParts; ++i) {
            engine::PartTransform& b = blocks[static_cast<std::size_t>(i)];
            all = pw.ragdollPart(s.body, i, b.position, b.orientation) && all;
            b.size = engine::BlockBody::parts()[static_cast<std::size_t>(i)].size * s.scale;
        }
        if (all && assets_) {
            const std::array<engine::BodyPiece, engine::BlockBody::kParts> pieces = meshes_.pieces(*assets_, blocks, s.scale);
            for (int i = 0; i < engine::BlockBody::kParts; ++i) {
                const engine::BodyPiece& b = pieces[static_cast<std::size_t>(i)];
                engine::Transform* t = world.get<engine::Transform>(s.parts[static_cast<std::size_t>(i)]);
                engine::PrevTransform* pt = world.get<engine::PrevTransform>(s.parts[static_cast<std::size_t>(i)]);
                if (!t) continue;
                const bool first = t->position.y < kOut * 0.5;
                if (pt) pt->value = *t;
                if (!b.drawn) {
                    t->position = Vec3(0, kOut, 0);
                    if (pt) pt->value = *t;
                    continue;
                }
                if (engine::Renderable* r = world.get<engine::Renderable>(s.parts[static_cast<std::size_t>(i)]))
                    r->mesh = b.mesh;
                t->position = b.position;
                t->orientation = b.orientation;
                t->scale = b.scale;
                if (pt && first) pt->value = *t;   // no streak up from below the world
            }
        }
        if (s.still > kStillFor || s.age > kMaxDown) {
            Vec3 torso;
            Quat q;
            pw.ragdollPart(s.body, engine::BlockBody::Torso, torso, q);
            up.push_back({s.owner, torso});
            pw.removeRagdoll(s.body);
            s.body = PhysicsWorld::INVALID_RAGDOLL;
            s.owner = -1;
            stow(world, s);
        }
    }
    return up;
}

std::string WalkerRagdolls::describe(PhysicsWorld& pw) const {
    std::string out;
    int n = 0;
    for (const Slot& s : slots_) {
        if (s.owner < 0) continue;
        ++n;
        Vec3 p;
        Quat q;
        pw.ragdollPart(s.body, engine::BlockBody::Torso, p, q);
        char b[128];
        std::snprintf(b, sizeof b, "; agent %d torso (%.2f %.2f %.2f) speed %.2f down %.1f s", s.owner, p.x, p.y, p.z,
                      pw.ragdollSpeed(s.body), s.age);
        out += b;
    }
    return std::to_string(n) + " down" + out;
}

bool WalkerRagdolls::down(int owner) const {
    for (const Slot& s : slots_)
        if (s.owner == owner) return true;
    return false;
}

void WalkerRagdolls::stow(World& world, Slot& s) {
    for (Entity e : s.parts) {
        engine::Transform* t = world.get<engine::Transform>(e);
        engine::PrevTransform* pt = world.get<engine::PrevTransform>(e);
        if (!t) continue;
        t->position = Vec3(0, kOut, 0);
        if (pt) pt->value = *t;
    }
}

void WalkerRagdolls::clear(World& world, PhysicsWorld& pw) {
    for (Slot& s : slots_) {
        if (s.body != PhysicsWorld::INVALID_RAGDOLL) pw.removeRagdoll(s.body);
        s.body = PhysicsWorld::INVALID_RAGDOLL;
        s.owner = -1;
        if (s.made) stow(world, s);
    }
}

}  // namespace citysim
