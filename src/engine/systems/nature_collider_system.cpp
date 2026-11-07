#include "nature_collider_system.h"

#include "physics_system.h"
#include "../components.h"
#include "../world.h"
#include "../../log.h"
#include "../particles.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace engine {

void NatureColliderSystem::onStart(FrameContext& ctx) {
    // the knocks that broke a piece loose (ADR-0354), kept for the next step
    onCollision_ = ctx.events.listen<Collision>([this](const Collision& c) {
        if (c.broke) knocks_.push_back({c.bodyA, c.bodyB, c.normal, c.speed});
    });
    if (ctx.commands != nullptr)
        knockCmd_ = ctx.commands->bind("pieces", "NatureColliderSystem", [this](const ControlCommand& cmd) -> std::string {
            if (cmd.args.empty() || cmd.args[0] != "knock") return "err pieces knock";
            knockNearest_ = true;
            return "ok staged";
        });
}

// A PIECE KNOCKED LOOSE (ADR-0354): its static body goes, its drawn instance goes, and a loose body of it is dropped
// in its place, pushed on along the knock; a hydrant's stub sprays for a minute. At most 40 loose pieces lie about.
void NatureColliderSystem::breakPieces(FrameContext& ctx, NatureColliders& nc) {
    PhysicsWorld& pw = physics_.physicsWorld();
    for (std::size_t i = 0; i < looseKick_.size();) {
        const RigidBody* rb = ctx.world.alive(looseKick_[i].e) ? ctx.world.get<RigidBody>(looseKick_[i].e) : nullptr;
        if (rb && rb->bodyId != INVALID_PHYSICS_BODY) {
            pw.setBodyMass(rb->bodyId, looseKick_[i].mass, 0.3, 0.6);
            pw.setLinearVelocity(rb->bodyId, looseKick_[i].velocity);
        }
        if (!rb || rb->bodyId != INVALID_PHYSICS_BODY) looseKick_.erase(looseKick_.begin() + static_cast<std::ptrdiff_t>(i));
        else ++i;
    }
    const Real dt = ctx.clock.fixedStep();
    for (std::size_t i = 0; i < sprays_.size();) {
        sprays_[i].left -= dt;
        if (sprays_[i].left > 0) { ++i; continue; }
        if (ctx.world.alive(sprays_[i].e)) ctx.world.destroy(sprays_[i].e);
        sprays_.erase(sprays_.begin() + static_cast<std::ptrdiff_t>(i));
    }
    std::vector<Knock> knocks;
    knocks.swap(knocks_);
    for (const Knock& k : knocks) {
        // which side is the piece (the normal points from body A to body B: the piece is pushed away from the car)
        PhysicsBodyId body = INVALID_PHYSICS_BODY, car = INVALID_PHYSICS_BODY;
        Vec3 away;
        if (shapeOf_.count(k.a)) { body = k.a; car = k.b; away = k.normal * -1.0; }
        else if (shapeOf_.count(k.b)) { body = k.b; car = k.a; away = k.normal; }
        if (body == INVALID_PHYSICS_BODY) continue;
        const int si = shapeOf_[body];
        NatureCollider& shape = nc.shapes[static_cast<std::size_t>(si)];
        if (shape.piece < 0) continue;
        NatureColliders::Piece& piece = nc.pieces[static_cast<std::size_t>(shape.piece)];
        if (piece.broken) continue;
        piece.broken = true;
        pw.setBreakable(body, 0);
        pw.removeBody(body);
        shapeOf_.erase(body);
        for (auto& [cell, ids] : live_) ids.erase(std::remove(ids.begin(), ids.end(), body), ids.end());
        // the drawn one, hidden (scaled to nothing where it stood)
        if (InstanceGroup* g = ctx.world.alive(piece.group) ? ctx.world.get<InstanceGroup>(piece.group) : nullptr)
            if (piece.instance < g->transforms.size())
                g->transforms[piece.instance] = Mat4::translate(shape.centre.x, shape.centre.y, shape.centre.z) * Mat4::scale(0, 0, 0);
        const NatureColliders::PieceKind& kind = nc.pieceKinds[static_cast<std::size_t>(piece.kind)];
        // the loose one
        const Entity e = ctx.world.create();
        Transform t;
        t.position = shape.centre;
        t.orientation = shape.orientation;
        ctx.world.add<Transform>(e, t);
        ctx.world.add<PrevTransform>(e, {t});
        Renderable r;
        r.mesh = kind.mesh;
        r.material = kind.material;
        ctx.world.add<Renderable>(e, r);
        Collider col;
        col.shape = ColliderShape::Box;
        col.halfExtent = shape.half;
        col.friction = 0.6;
        ctx.world.add<Collider>(e, col);
        RigidBody rb;
        rb.ignore = car;   // (born where the car is passing through it: not shoved out of it)
        ctx.world.add<RigidBody>(e, rb);
        Vec3 push = away;
        push.y = 0;
        const Real pl = push.length();
        push = pl > 1e-6 ? push * (1.0 / pl) : Vec3(0, 0, 0);
        looseKick_.push_back({e, push * (k.speed * nc.knockPush) + Vec3(0, nc.knockUp + nc.knockUpPerSpeed * k.speed, 0), kind.mass});
        loose_.push_back(e);
        // loose, it can be picked up (ADR-0358; CarrySystem leaves what is over its weight limit)
        ctx.world.add<Carryable>(e, Carryable{kind.name.empty() ? std::string("piece") : kind.name,
                                             static_cast<float>(kind.mass), shape.half});
        while (loose_.size() > 40) {   // (one carried or stowed is the player's now: it stays)
            const Entity old = loose_.front();
            if (ctx.world.alive(old) && !ctx.world.has<CarriedBy>(old) && !ctx.world.has<Stowed>(old)) ctx.world.destroy(old);
            loose_.erase(loose_.begin());
        }
        requestParticles(ctx.world, "spark", shape.centre, push, 10, 0.8);
        // the stub sprays
        if (!kind.spray.empty() && kind.sprayRate > 0) {
            const Entity se = ctx.world.create();
            Transform st;
            st.position = Vec3(shape.centre.x, shape.centre.y - shape.half.y + 0.12, shape.centre.z);
            ctx.world.add<Transform>(se, st);
            ParticleEmitter em;
            em.effect = kind.spray;
            em.rate = kind.sprayRate;
            em.velocity = Vec3(0, kind.sprayUp, 0);
            em.halfExtent = Vec3(0.05, 0.02, 0.05);
            ctx.world.add<ParticleEmitter>(se, em);
            sprays_.push_back({se, 60.0});
        }
        LOG_INFO << "[street] a piece knocked loose at " << shape.centre.x << " " << shape.centre.z
                 << (kind.spray.empty() ? "" : " (spraying)");
    }
}

void NatureColliderSystem::fixedUpdate(FrameContext& ctx) {
    NatureColliders* nc = nullptr;
    ctx.world.each<NatureColliders>([&](Entity, NatureColliders& c) { if (!nc) nc = &c; });
    if (!nc || nc->shapes.empty()) return;
    // `pieces knock`: the live piece nearest the player knocked as a 6 m/s car would, from the player's side
    if (knockNearest_) {
        knockNearest_ = false;
        Vec3 me;
        ctx.world.each<Transform, ControlledBy>([&](Entity, Transform& t, ControlledBy&) { me = t.position; });
        PhysicsBodyId best = INVALID_PHYSICS_BODY;
        Real bestD = 1e30;
        for (const auto& [id, si] : shapeOf_) {
            const Real d = (nc->shapes[static_cast<std::size_t>(si)].centre - me).length();
            if (d < bestD) { bestD = d; best = id; }
        }
        if (best != INVALID_PHYSICS_BODY) {
            Vec3 n = nc->shapes[static_cast<std::size_t>(shapeOf_[best])].centre - me;
            n.y = 0;
            const Real l = n.length();
            knocks_.push_back({INVALID_PHYSICS_BODY, best, l > 1e-6 ? n * (1.0 / l) : Vec3(1, 0, 0), 6.0});
        }
    }
    breakPieces(ctx, *nc);
    // the player, or the car it drives (as the terrain collider window does)
    Vec3 at, prev;
    bool found = false;
    ctx.world.each<Transform, ControlledBy>([&](Entity e, Transform& t, ControlledBy&) {
        if (found) return;
        Entity tracked = e;
        if (const InVehicle* iv = ctx.world.get<InVehicle>(e))
            if (iv->vehicle.valid() && ctx.world.alive(iv->vehicle) && ctx.world.has<Transform>(iv->vehicle)) tracked = iv->vehicle;
        const Transform* tt = tracked == e ? &t : ctx.world.get<Transform>(tracked);
        at = tt->position;
        prev = at;
        if (const PrevTransform* pt = ctx.world.get<PrevTransform>(tracked)) prev = pt->value.position;
        found = true;
    });
    if (!found) return;
    const Vec3 vel = (at - prev) * (1.0 / std::max(1e-6, ctx.clock.fixedStep()));
    const Vec3 ahead = at + vel * 1.5;
    constexpr double kReach = 60.0;
    const double C = NatureColliders::kCell;
    // the cells wanted: round the body and round where it will be
    std::unordered_map<int64_t, double> want;   // cell -> distance from the body
    for (const Vec3& c : {at, ahead}) {
        const int i0 = static_cast<int>(std::floor((c.x - kReach) / C)), i1 = static_cast<int>(std::floor((c.x + kReach) / C));
        const int j0 = static_cast<int>(std::floor((c.z - kReach) / C)), j1 = static_cast<int>(std::floor((c.z + kReach) / C));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) {
                const int64_t k = NatureColliders::key(i, j);
                if (!nc->cells.count(k)) continue;
                const double cx = (i + 0.5) * C, cz = (j + 0.5) * C;
                const double d = std::hypot(cx - at.x, cz - at.z);
                auto it = want.find(k);
                if (it == want.end() || d < it->second) want[k] = d;
            }
    }
    PhysicsWorld& pw = physics_.physicsWorld();
    // drop the cells left behind
    for (auto it = live_.begin(); it != live_.end();) {
        if (want.count(it->first)) { ++it; continue; }
        for (PhysicsBodyId id : it->second) {
            if (shapeOf_.count(id)) { pw.setBreakable(id, 0); shapeOf_.erase(id); }
            pw.removeBody(id);
        }
        it = live_.erase(it);
    }
    // add the missing, nearest first, a budget a step (the cell under the body always)
    std::vector<std::pair<double, int64_t>> missing;
    for (const auto& [k, d] : want) if (!live_.count(k)) missing.push_back({d, k});
    std::sort(missing.begin(), missing.end());
    int budget = 600;
    for (const auto& [d, k] : missing) {
        const std::vector<int>& idx = nc->cells.at(k);
        if (budget <= 0 && d > C) break;
        std::vector<PhysicsBodyId>& ids = live_[k];
        for (int s : idx) {
            const NatureCollider& c = nc->shapes[static_cast<std::size_t>(s)];
            if (c.piece >= 0 && nc->pieces[static_cast<std::size_t>(c.piece)].broken) continue;   // (knocked off)
            const PhysicsBodyId id = c.capsule
                ? pw.addCapsule(c.half.y, c.half.x, c.centre, c.orientation, BodyMotion::Static, 0.0, 0.8)
                : pw.addBox(c.half, c.centre, c.orientation, BodyMotion::Static, 0.0, 0.8);
            if (id == INVALID_PHYSICS_BODY) continue;
            ids.push_back(id);
            if (c.piece >= 0) {
                shapeOf_[id] = s;
                pw.setBreakable(id, nc->pieceKinds[static_cast<std::size_t>(nc->pieces[static_cast<std::size_t>(c.piece)].kind)].breaksAt);
            }
        }
        budget -= static_cast<int>(idx.size());
    }
}

void NatureColliderSystem::onStop(FrameContext&) {
    PhysicsWorld& pw = physics_.physicsWorld();
    for (auto& [k, ids] : live_)
        for (PhysicsBodyId id : ids) {
            if (shapeOf_.count(id)) pw.setBreakable(id, 0);
            pw.removeBody(id);
        }
    live_.clear();
    shapeOf_.clear();
}

}  // namespace engine
