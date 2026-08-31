#include "door_system.h"

#include "../components.h"
#include "../world.h"
#include "../procgen/city/shape_grammar.h"   // materialFor(PartId::Door)
#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {

Real doorSwingSign(const Vec2& moverPos, const Vec2& moverVel,
                   const Vec2& foot, const Vec2& normal) {
    const Real vn = dot(moverVel, normal);
    if (vn < -0.05) return -1.0;   // heading inward: the leaf leads them in
    if (vn > 0.05) return 1.0;     // heading outward: it leads them out
    // Standing still at the threshold: swing to the side they are NOT on.
    return dot(moverPos - foot, normal) >= 0 ? -1.0 : 1.0;
}

namespace {
// 2-D rotation about +Y by `a` (right-handed, matching the walker facing
// convention yaw = atan2(dir.x, dir.z)); v is (x, z).
Vec2 rotY(const Vec2& v, Real a) {
    const Real c = std::cos(a), s = std::sin(a);
    return Vec2(v.x * c + v.y * s, -v.x * s + v.y * c);
}
}  // namespace

void DoorSystem::fixedUpdate(FrameContext& ctx) {
    World& world = ctx.world;
    const CityBuildings* cb = nullptr;
    world.each<CityBuildings>(
        [&](Entity, CityBuildings& c) { if (!cb) cb = &c; });
    if (!cb || cb->records.empty()) return;

    // The player (the only mover for now; walkers later).
    Vec3 pos, prev;
    bool found = false;
    world.each<Transform, ControlledBy>(
        [&](Entity e, Transform& t, ControlledBy&) {
            if (found) return;
            pos = prev = t.position;
            if (const PrevTransform* pt = world.get<PrevTransform>(e))
                prev = pt->value.position;
            found = true;
        });
    if (!found) return;
    const double dt = std::max(1e-6, ctx.clock.fixedStep());
    const Vec2 xz(pos.x, pos.z);
    const Vec2 vel((pos.x - prev.x) / dt, (pos.z - prev.z) / dt);

    // Desired leaves: every door of an enterable record near the player.
    std::vector<const BuildingRecord*> nearby;
    cb->near(xz, ACTIVE_M, nearby);
    std::vector<int64_t> desired;
    for (const BuildingRecord* r : nearby) {
        if (!r->enterable) continue;
        const int64_t ri = r - cb->records.data();
        for (std::size_t d = 0; d < r->doors.size() && d < 255; ++d) {
            const int64_t key = (ri << 8) | static_cast<int64_t>(d);
            desired.push_back(key);
            if (leaves_.count(key)) continue;
            const DoorSpec& ds = r->doors[d];
            Leaf lf;
            lf.foot = ds.foot;
            lf.normal = ds.normal;
            lf.width = ds.width > 0.5 ? ds.width : Real(2.0);
            lf.height = ds.height > 1.0 ? ds.height : Real(2.7);
            lf.footY = r->baseY;
            lf.mesh = ctx.assets.acquirePrimitive(
                "box", Vec3(lf.width, lf.height, 0.06));
            Entity e = world.create();
            Transform t;
            t.position = Vec3(lf.foot.x, lf.footY + lf.height * 0.5, lf.foot.y);
            world.add<Transform>(e, t);
            world.add<PrevTransform>(e, PrevTransform{t});
            Renderable rd;
            rd.mesh = lf.mesh;
            rd.material = materialFor(PartId::Door, Vec3(1, 1, 1));
            rd.drawClass = DrawClass::Structure;
            rd.drawDistance = 120.0;
            world.add<Renderable>(e, rd);
            lf.entity = e;
            leaves_[key] = lf;
        }
    }

    // Drop leaves whose building left the bubble.
    for (auto it = leaves_.begin(); it != leaves_.end();) {
        if (std::find(desired.begin(), desired.end(), it->first) !=
            desired.end()) {
            ++it;
            continue;
        }
        if (world.alive(it->second.entity)) world.destroy(it->second.entity);
        ctx.assets.releaseMesh(it->second.mesh);
        it = leaves_.erase(it);
    }

    // Swing state + pose. The hinge is the door's left jamb; the box pivots
    // around it by composing position and yaw from the SAME rotated tangent,
    // so the leaf stays seated on the hinge at every angle.
    for (auto& kv : leaves_) {
        Leaf& lf = kv.second;
        const Real dist = (xz - lf.foot).length();
        if (dist < TRIGGER_M) {
            // doorSwingSign's contract is semantic (-1 = the inside face,
            // +1 = the outside); the WORLD side a positive rotY angle lands
            // on was only ever inferred from a dark screenshot — and the
            // device played it: "the door opens into the user not away".
            // The measured mapping is the negation.
            lf.target =
                -doorSwingSign(xz, vel, lf.foot, lf.normal) * SWING_RAD;
            lf.clearFor = 0;
        } else {
            lf.clearFor += dt;
            if (lf.clearFor > CLOSE_S) lf.target = 0;
        }
        const Real maxStep = static_cast<Real>(RATE_RAD_S * dt);
        lf.angle += std::max(-maxStep, std::min(maxStep, lf.target - lf.angle));

        const Vec2 tangent(-lf.normal.y, lf.normal.x);
        const Vec2 hinge = lf.foot - tangent * (lf.width * 0.5);
        const Vec2 rt = rotY(tangent, lf.angle);
        const Vec2 rn = rotY(lf.normal, lf.angle);
        const Vec2 centre = hinge + rt * (lf.width * 0.5);
        if (Transform* t = world.get<Transform>(lf.entity)) {
            if (PrevTransform* pt = world.get<PrevTransform>(lf.entity))
                pt->value = *t;
            t->position =
                Vec3(centre.x, lf.footY + lf.height * 0.5, centre.y);
            t->orientation =
                Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(rn.x, rn.y));
        }
    }
}

void DoorSystem::onStop(FrameContext& ctx) {
    for (auto& kv : leaves_) {
        if (ctx.world.alive(kv.second.entity))
            ctx.world.destroy(kv.second.entity);
        ctx.assets.releaseMesh(kv.second.mesh);
    }
    leaves_.clear();
}

}  // namespace engine
