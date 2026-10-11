#include "train_system.h"

#include "../../engine/asset_manager.h"
#include "../../engine/camera/fly_camera_controller.h"
#include "../../engine/input/input_map.h"
#include "../../engine/systems/physics_system.h"
#include "../../engine/world.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace citysim {

using engine::Entity;
using engine::InstanceGroup;
using engine::Mat4;
using engine::Real;
using engine::Transform;
using engine::Vec3;

namespace {

constexpr Real kWalk = 1.8;               // m/s along a carriage
constexpr Real kStandingEye = 1.62;       // eye above the floor
constexpr Real kEyeAboveTransform = 0.7;  // PlayerSystem's eye above the capsule centre
constexpr Real kWallClear = 0.28;         // how near a wall a standing body comes

Vec3 col(const Mat4& m, int c) { return Vec3(m.m[0][c], m.m[1][c], m.m[2][c]); }
Vec3 toLocal(const Mat4& m, const Vec3& p) {
    const Vec3 d = p - col(m, 3);
    return Vec3(engine::dot(d, col(m, 0)), engine::dot(d, col(m, 1)), engine::dot(d, col(m, 2)));
}
Real yawOf(const Mat4& m) {
    const Vec3 f = col(m, 2);
    return std::atan2(f.x, -f.z) * Real(57.29577951308232);
}

Entity makeGroup(engine::FrameContext& ctx, const engine::RenderMesh& mesh, const std::string& name, bool glass) {
    if (mesh.vertices.empty()) return Entity{};
    const Entity e = ctx.world.create();
    InstanceGroup g;
    g.mesh = ctx.assets.acquireMesh(mesh, name);
    engine::RenderMaterial m;
    m.albedo = Vec3(1, 1, 1);
    if (glass) { m.roughness = 0.06f; m.opacity = 0.30f; m.metallic = 0.0f; }
    else { m.roughness = 0.45f; m.metallic = 0.15f; }
    g.material = m;
    g.renderLayer = engine::LayerSim;
    g.drawClass = engine::DrawClass::SimBody;
    ctx.world.add<InstanceGroup>(e, g);
    return e;
}

}  // namespace

void TrainSystem::build(engine::FrameContext& ctx) {
    tried_ = true;
    std::vector<engine::RailTrack> tracks;
    ctx.world.each<engine::RailTracks>([&](Entity, engine::RailTracks& rt) {
        tracks.insert(tracks.end(), rt.tracks.begin(), rt.tracks.end());
    });
    if (tracks.empty()) return;
    // the stock: by the track's class (a metro's subway, a tram line's light rail), else intercity; RT_TRAIN_KIND
    // overrides for a look at the others on the same rails
    std::string kindName = tracks.front().cls == "metro" ? "subway" : tracks.front().cls == "light_rail" ? "light_rail" : "intercity";
    if (const char* k = std::getenv("RT_TRAIN_KIND")) kindName = k;
    const TrainKind kind = trainKindNamed(kindName);
    int perLine = 2;
    if (const char* n = std::getenv("RT_TRAINS_PER_LINE")) perLine = std::max(1, std::atoi(n));
    sim_.build(tracks, kind, perLine);
    engine::TrainBodySpec spec;
    spec.kind = kind.name;
    spec.width = kind.width;
    spec.floor = kind.floor;
    spec.gap = kind.gap;
    spec.length = kind.locoLength;
    loco_ = engine::buildTrainLocomotive(spec);
    spec.length = kind.carLength;
    coach_ = engine::buildTrainCarriage(spec);
    locoBody_ = makeGroup(ctx, loco_.body, "train:loco", false);
    locoGlass_ = makeGroup(ctx, loco_.glass, "train:locoglass", true);
    coachBody_ = makeGroup(ctx, coach_.body, "train:coach", false);
    coachGlass_ = makeGroup(ctx, coach_.glass, "train:coachglass", true);
    leaves_ = makeGroup(ctx, coach_.doorLeaf, "train:door", false);
    LOG_INFO << "[trains] " << kind.name << ": " << sim_.describe() << "; coach " << coach_.body.indices.size() / 3
             << " tris, " << coach_.seats.size() << " seats; locomotive " << loco_.body.indices.size() / 3 << " tris";
}

// Can a body stand at `local` in car `car`? The aisle; full width in a vestibule or a doorway; the gangway's width at
// the ends. `doorSide`: the side whose doors are open (a doorway reaches out to the sill there), 0 none.
bool TrainSystem::walkableAt(int car, const Vec3& local, int doorSide) const {
    const engine::TrainBodyMesh& M = model(car);
    if (!M.walkable) return false;
    const Real hl = static_cast<Real>(M.length) / 2, hw = static_cast<Real>(M.width) / 2;
    const Real ax = std::fabs(local.x);
    if (std::fabs(local.z) > hl - 0.15) return ax <= static_cast<Real>(M.gangwayHalf) - 0.2;   // the gangway
    for (double dz : M.doorZ)
        if (std::fabs(local.z - static_cast<Real>(dz)) < static_cast<Real>(M.doorHalf) - 0.15) {
            if (doorSide != 0 && local.x * doorSide > 0) return ax <= hw + 0.6;   // out through the open door
            return ax <= hw - kWallClear;
        }
    // an intercity car's vestibules (between its end and the first seat bay) are full width too
    if (!M.seats.empty()) {
        Real zMin = 1e9, zMax = -1e9;
        for (const Vec3& s : M.seats) { zMin = std::min(zMin, s.z); zMax = std::max(zMax, s.z); }
        if (local.z < zMin - 0.6 || local.z > zMax + 0.6) return ax <= hw - kWallClear;
    }
    return ax <= static_cast<Real>(M.aisleHalf);
}

void TrainSystem::fixedUpdate(engine::FrameContext& ctx) {
    if (!tried_) build(ctx);
    if (sim_.trains().empty()) return;
    const Real dt = ctx.clock.fixedStep();
    sim_.step(dt);
    // `train_board <train> <car>`: put the player aboard (a probe's way in; on foot you walk in through a door)
    const std::string req = ctx.settings.getString("train.board", "");
    if (!req.empty()) {
        ctx.settings.setString("train.board", "");
        int t = 0, c = 1;
        if (std::sscanf(req.c_str(), "%d %d", &t, &c) >= 1 && t >= 0 && t < static_cast<int>(sim_.trains().size())) {
            train_ = t;
            car_ = std::clamp(c, 1, sim_.carCount(t) - 1);
            local_ = Vec3(0, static_cast<Real>(coach_.floor), 0);
            haveYaw_ = false;
            LOG_INFO << "[trains] aboard train " << train_ << " car " << car_ << " (by request)";
        }
    }
    ride(ctx, dt);
    if (++tick_ % 30 == 0) {
        std::string aboard = "on foot";
        if (train_ >= 0) {
            char b[160];
            std::snprintf(b, sizeof(b), "aboard train %d car %d at %.2f %.2f (speed %.1f)", train_, car_, local_.x, local_.z,
                          sim_.trains()[static_cast<std::size_t>(train_)].v);
            aboard = b;
        }
        ctx.settings.setString("trains.telemetry", sim_.describe() + "; " + aboard);
    }
}

void TrainSystem::ride(engine::FrameContext& ctx, Real dt) {
    Entity player;
    ctx.world.each<Transform, engine::CharacterController, engine::ControlledBy>(
        [&](Entity e, Transform&, engine::CharacterController&, engine::ControlledBy&) { if (!player.valid()) player = e; });
    if (!player.valid()) { train_ = -1; return; }
    Transform* pt = ctx.world.get<Transform>(player);
    auto* cc = ctx.world.get<engine::CharacterController>(player);
    if (!pt || !cc) return;
    auto pin = [&](const Vec3& pos) {
        pt->position = pos;
        if (auto* prev = ctx.world.get<engine::PrevTransform>(player)) prev->value = *pt;
        if (cc->characterId != engine::INVALID_CHARACTER) physics_.physicsWorld().setCharacterPosition(cc->characterId, pos);
        pinned_ = pos;
    };
    auto leave = [&](const Vec3& where) {
        pin(where);
        if (ctx.world.has<engine::Passenger>(player)) ctx.world.remove<engine::Passenger>(player);
        LOG_INFO << "[trains] you stepped off train " << train_ << " at (" << where.x << ", " << where.z << ")";
        train_ = car_ = -1;
    };

    if (train_ >= 0) {
        // moved by something else (fast travel, teleport): that is getting off
        if ((pt->position - pinned_).length() > 3.0 && pinned_.lengthSquared() > 0) {
            if (ctx.world.has<engine::Passenger>(player)) ctx.world.remove<engine::Passenger>(player);
            train_ = car_ = -1;
            return;
        }
    } else {
        // ON FOOT: walking into an open doorway from the platform is boarding
        const Vec3 feet = pt->position - Vec3(0, kEyeAboveTransform, 0);
        for (int ti = 0; ti < static_cast<int>(sim_.trains().size()) && train_ < 0; ++ti) {
            const Train& T = sim_.trains()[static_cast<std::size_t>(ti)];
            if (T.doorSide == 0) continue;
            for (int c = 1; c < sim_.carCount(ti); ++c) {
                const CarPose P = sim_.carPose(ti, c, 1.0);
                const Vec3 l = toLocal(P.m, feet);
                if (std::fabs(l.z) > P.length / 2 + 1 || l.x * T.doorSide < 0) continue;
                if (std::fabs(l.y - coach_.floor) > 0.6) continue;   // at platform height, not down on the ballast
                bool door = false;
                for (double dz : coach_.doorZ) door = door || std::fabs(l.z - static_cast<Real>(dz)) < coach_.doorHalf - 0.15;
                if (!door || std::fabs(l.x) > coach_.width / 2 + 0.35) continue;
                train_ = ti;
                car_ = c;
                local_ = Vec3(std::clamp(l.x, Real(-(coach_.width / 2 - kWallClear)), Real(coach_.width / 2 - kWallClear)),
                              static_cast<Real>(coach_.floor), l.z);
                haveYaw_ = false;
                if (!ctx.world.has<engine::Passenger>(player)) ctx.world.add<engine::Passenger>(player);
                LOG_INFO << "[trains] you boarded train " << ti << ", car " << c;
                break;
            }
        }
        if (train_ < 0) return;
    }
    if (!ctx.world.has<engine::Passenger>(player)) ctx.world.add<engine::Passenger>(player);

    // WALK the carriage in its own frame
    const Train& T = sim_.trains()[static_cast<std::size_t>(train_)];
    CarPose P = sim_.carPose(train_, car_, 1.0);
    const Vec3 fwdAxis = col(P.m, 2), leftAxis = col(P.m, 0);
    Vec3 move = fly_.forward() * static_cast<Real>(ctx.actions.axis("cam_forward")) + fly_.right() * static_cast<Real>(ctx.actions.axis("cam_right"));
    move.y = 0;
    const Real ml = move.length();
    if (ml > 1) move = move * (1 / ml);
    Vec3 next = local_;
    next.x += engine::dot(move, leftAxis) * kWalk * dt;
    next.z += engine::dot(move, fwdAxis) * kWalk * dt;
    // try the step, then each axis alone (slide along a seat row or a wall)
    if (walkableAt(car_, next, T.doorSide)) local_ = next;
    else if (walkableAt(car_, Vec3(next.x, local_.y, local_.z), T.doorSide)) local_.x = next.x;
    else if (walkableAt(car_, Vec3(local_.x, local_.y, next.z), T.doorSide)) local_.z = next.z;
    local_.y = static_cast<Real>(coach_.floor);
    // THROUGH THE GANGWAY: past the coupling's middle, the point belongs to the next car
    const Real hl = static_cast<Real>(coach_.length) / 2, half = static_cast<Real>(T.kind.gap) / 2;
    if (local_.z > hl + half - 0.05 || local_.z < -hl - half + 0.05) {
        const int to = local_.z > 0 ? car_ - 1 : car_ + 1;
        if (to >= 1 && to < sim_.carCount(train_)) {
            const Vec3 w = P.m.transformPoint(local_);
            const CarPose Q = sim_.carPose(train_, to, 1.0);
            Vec3 l = toLocal(Q.m, w);
            l.y = static_cast<Real>(coach_.floor);
            l.x = std::clamp(l.x, Real(-(coach_.gangwayHalf - 0.2)), Real(coach_.gangwayHalf - 0.2));
            car_ = to;
            local_ = l;
            P = Q;
        } else {
            local_.z = std::clamp(local_.z, -hl - half + 0.06, hl + half - 0.06);   // the train's last end: a closed door
            local_.z = std::clamp(local_.z, -hl + 0.2, hl - 0.2);
        }
    }
    // OUT OF THE DOOR: past the body's side where it is open
    if (T.doorSide != 0 && local_.x * T.doorSide > coach_.width / 2 + 0.3) {
        Vec3 out = P.m.transformPoint(Vec3(T.doorSide * static_cast<Real>(coach_.width / 2 + 0.5), static_cast<Real>(coach_.floor), local_.z));
        out.y += kEyeAboveTransform + 0.05;
        leave(out);
        return;
    }
    const Vec3 w = P.m.transformPoint(local_);
    pin(Vec3(w.x, w.y + kStandingEye - kEyeAboveTransform, w.z));
}

void TrainSystem::render(engine::FrameContext& ctx) {
    if (sim_.trains().empty()) return;
    const double alpha = std::clamp(ctx.interpolation, 0.0, 1.0);
    InstanceGroup* lb = locoBody_.valid() ? ctx.world.get<InstanceGroup>(locoBody_) : nullptr;
    InstanceGroup* lg = locoGlass_.valid() ? ctx.world.get<InstanceGroup>(locoGlass_) : nullptr;
    InstanceGroup* cb = coachBody_.valid() ? ctx.world.get<InstanceGroup>(coachBody_) : nullptr;
    InstanceGroup* cg = coachGlass_.valid() ? ctx.world.get<InstanceGroup>(coachGlass_) : nullptr;
    InstanceGroup* dl = leaves_.valid() ? ctx.world.get<InstanceGroup>(leaves_) : nullptr;
    for (InstanceGroup* g : {lb, lg, cb, cg, dl}) if (g) g->transforms.clear();
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    const Real hw = static_cast<Real>(coach_.width / 2), dh = static_cast<Real>(coach_.doorHalf);
    for (int ti = 0; ti < static_cast<int>(sim_.trains().size()); ++ti) {
        const Train& T = sim_.trains()[static_cast<std::size_t>(ti)];
        for (const CarPose& P : sim_.carPoses(ti, alpha)) {
            const Vec3 c = col(P.m, 3);
            lo = Vec3(std::min(lo.x, c.x), std::min(lo.y, c.y), std::min(lo.z, c.z));
            hi = Vec3(std::max(hi.x, c.x), std::max(hi.y, c.y), std::max(hi.z, c.z));
            if (P.loco) {
                if (lb) lb->transforms.push_back(P.m);
                if (lg) lg->transforms.push_back(P.m);
                continue;
            }
            if (cb) cb->transforms.push_back(P.m);
            if (cg) cg->transforms.push_back(P.m);
            if (!dl) continue;
            for (double dz : coach_.doorZ)
                for (int s = -1; s <= 1; s += 2) {
                    const Real open = T.doorSide == s ? dh : 0;   // plug doors slide along the skin
                    for (int leaf = -1; leaf <= 1; leaf += 2) {
                        const Real z = static_cast<Real>(dz) + leaf * (dh / 2 + open);
                        dl->transforms.push_back(P.m * Mat4::translate(s * (hw + 0.03), static_cast<Real>(coach_.floor + coach_.doorHeight / 2), z));
                    }
                }
        }
    }
    const Vec3 centre = (lo + hi) * 0.5;
    const Real radius = (hi - lo).length() * 0.5 + 40;
    for (InstanceGroup* g : {lb, lg, cb, cg, dl})
        if (g) { g->boundsCenter = centre; g->boundsRadius = radius; }

    // THE EYE IN THE CARRIAGE AS DRAWN, and the view turning with it
    if (train_ >= 0 && car_ >= 1) {
        const CarPose P = sim_.carPose(train_, car_, alpha);
        const Real yaw = yawOf(P.m);
        if (haveYaw_) {
            Real d = yaw - lastYaw_;
            while (d > 180) d -= 360;
            while (d < -180) d += 360;
            fly_.yaw += d;
        }
        lastYaw_ = yaw;
        haveYaw_ = true;
        const Vec3 w = P.m.transformPoint(local_);
        const Vec3 eye(w.x, w.y + kStandingEye, w.z);
        engine::CameraState& cam = ctx.view.camera;
        if ((eye - cam.position).length() < 3.0) {   // only the first-person eye
            cam.position = eye;
            cam.target = eye + fly_.forward();
        }
    }
}

}  // namespace citysim
