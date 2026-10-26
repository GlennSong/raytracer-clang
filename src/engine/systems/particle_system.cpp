#include "particle_system.h"

#include "../asset_manager.h"
#include "../components.h"
#include "../mesh_builder.h"
#include "../../log.h"
#ifdef RT_ENABLE_SCRIPTING
#include "../scripting/particles_lua.h"
#endif

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr int kTex = 64;

// The sprites' shapes, alpha only (RGB white).
std::vector<unsigned char> bake(ParticleShape s) {
    std::vector<unsigned char> px(static_cast<std::size_t>(kTex) * kTex * 4, 255);
    for (int y = 0; y < kTex; ++y)
        for (int x = 0; x < kTex; ++x) {
            const double u = (x + 0.5) / kTex - 0.5, v = (y + 0.5) / kTex - 0.5;
            const double r = std::sqrt(u * u + v * v) / 0.5;   // 0 centre .. 1 edge
            double a = 0;
            if (r < 1.0) {
                switch (s) {
                    case ParticleShape::Puff: {
                        // soft, a little lumpy so overlapping puffs don't read as discs
                        const double lump = 1.0 - 0.18 * r * r * (0.5 + 0.5 * std::sin(std::atan2(v, u) * 5.0));   // the edge only
                        a = std::min(1.0, 1.6 * std::pow(1.0 - r, 1.1)) * lump;   // a fuller middle: it reads
                        break;
                    }
                    case ParticleShape::Ring:   // the rim, a faint inside, a glint
                        a = std::exp(-std::pow((r - 0.78) / 0.12, 2.0)) * 0.9 + 0.12 * (1.0 - r) +
                            (std::hypot(u + 0.15, v - 0.15) < 0.08 ? 0.8 : 0.0);
                        break;
                    case ParticleShape::Glow:
                        a = r < 0.25 ? 1.0 : std::pow((1.0 - r) / 0.75, 2.0);
                        break;
                }
            }
            px[(static_cast<std::size_t>(y) * kTex + x) * 4 + 3] =
                static_cast<unsigned char>(std::clamp(a, 0.0, 1.0) * 255.0);
        }
    return px;
}
}  // namespace

void ParticleSystem::onStart(FrameContext&) {
#ifdef RT_ENABLE_SCRIPTING
    ensureParticleBookLoaded();
#endif
}

std::array<Entity, ParticleSystem::kOpacitySteps>& ParticleSystem::groupsFor(FrameContext& ctx, int effect) {
    if (groups_.size() < pool_.book().size()) groups_.resize(pool_.book().size());
    std::array<Entity, kOpacitySteps>& row = groups_[static_cast<std::size_t>(effect)];
    if (row[0].valid()) return row;
    if (!haveQuad_) {
        // a unit quad in the XY plane, centred: the sprite. Its normal leans to the sprite's up (the view's up) more
        // than toward the camera: a flat card facing the camera was lit only when the camera faced the sun, and a
        // puff of steam came out black looking the other way -- lit from above, it reads as a soft volume both ways
        RenderMesh q;
        const Vec3 n = normalize(Vec3(0, 0.85, 0.5));
        auto vtx = [&](Real x, Real y, float u, float v) {
            Vertex vt(Vec3(x, y, 0), n, Vec3(1, 0, 0), u, v);
            vt.color = Vec3(1, 1, 1);
            q.vertices.push_back(vt);
        };
        vtx(-0.5, -0.5, 0, 0); vtx(0.5, -0.5, 1, 0); vtx(0.5, 0.5, 1, 1); vtx(-0.5, 0.5, 0, 1);
        MeshBuilder::quadFacing(q, 0, 1, 2, 3, Vec3(0, 0, 1));   // (winding by the face, not the lit normal)
        quad_ = ctx.assets.acquireMesh(q, "particles:sprite");
        haveQuad_ = true;
    }
    const ParticleEffect& E = pool_.book().at(effect);
    TextureHandle& tex = tex_[static_cast<std::size_t>(E.shape)];
    if (!tex.valid()) {
        const std::vector<unsigned char> px = bake(E.shape);
        tex = ctx.renderer.uploadTexture(kTex, kTex, 4, px.data());
    }
    for (int b = 0; b < kOpacitySteps; ++b) {
        InstanceGroup g;
        g.mesh = quad_;
        g.material.albedo = E.colour;
        g.material.metallic = 0.0f;
        g.material.roughness = 1.0f;
        g.material.opacity = kOpacity[b];
        g.material.albedoMap = tex;
        g.material.flags |= RenderMaterial::FLAG_ALPHA_FROM_MAP | RenderMaterial::FLAG_TWO_SIDED;
        g.material.emission = E.glow;
        g.drawClass = DrawClass::Effect;
        g.drawDistance = 400.0;
        row[static_cast<std::size_t>(b)] = ctx.world.create();
        ctx.world.add<InstanceGroup>(row[static_cast<std::size_t>(b)], std::move(g));
    }
    return row;
}

void ParticleSystem::onStop(FrameContext& ctx) {
    for (auto& row : groups_)
        for (Entity& e : row)
            if (e.valid() && ctx.world.alive(e)) ctx.world.destroy(e);
    groups_.clear();
    if (haveQuad_) ctx.assets.releaseMesh(quad_);
    haveQuad_ = false;
    pool_.clear();
}

void ParticleSystem::spend(FrameContext& ctx, Real dt) {
    const ParticleBook& book = pool_.book();
    auto lookup = [&](const std::string& name) {
        const int i = book.find(name);
        if (i < 0 && std::find(warned_.begin(), warned_.end(), name) == warned_.end()) {
            warned_.push_back(name);
            LOG_WARN << "[particles] no effect called \"" << name << "\" (assets/scripts/particles.lua)";
        }
        return i;
    };
    // the bursts asked for since last frame
    ctx.world.each<ParticleRequests>([&](Entity, ParticleRequests& q) {
        for (const ParticleRequest& r : q.queue)
            pool_.burst(lookup(r.effect), r.at, r.dir, r.count, r.scale, r.ceiling, r.ref);
        q.queue.clear();
    });
    // the steady flows
    std::uniform_real_distribution<Real> u(-1.0, 1.0);
    ctx.world.each<ParticleEmitter, Transform>([&](Entity, ParticleEmitter& em, Transform& t) {
        if (em.rate <= 0) {
            em.due = 0;
            return;
        }
        const int fx = lookup(em.effect);
        if (fx < 0) return;
        em.due += em.rate * dt;
        const Vec3 origin = t.position + t.orientation.rotate(em.offset) + em.entityVelocity * em.ahead;
        while (em.due >= 1.0) {
            em.due -= 1.0;
            const Vec3 at = origin + Vec3(u(pool_.rng()) * em.halfExtent.x, u(pool_.rng()) * em.halfExtent.y,
                                          u(pool_.rng()) * em.halfExtent.z);
            const ParticleEffect& E = book.at(fx);
            Vec3 ball;   // the effect's own up and spread, besides the emitter's velocity
            do ball = Vec3(u(pool_.rng()), u(pool_.rng()), u(pool_.rng())); while (ball.lengthSquared() > 1.0);
            pool_.emit(fx, at, em.velocity + Vec3(0, E.up, 0) + ball * E.spread, 1.0 + 0.25 * u(pool_.rng()),
                       em.ceiling, em.relative ? em.entityVelocity : Vec3());
        }
    });
}

void ParticleSystem::draw(FrameContext& ctx) {
    for (auto& row : groups_)
        for (Entity e : row)
            if (InstanceGroup* g = e.valid() ? ctx.world.get<InstanceGroup>(e) : nullptr) {
                g->transforms.clear();
                g->boundsCenter = ctx.view.camera.position;
                g->boundsRadius = 400.0;
            }
    const std::vector<Particle>& ps = pool_.particles();
    if (ps.empty()) return;
    const auto& cam = ctx.view.camera;
    Vec3 fwd = cam.target - cam.position;
    fwd = fwd.lengthSquared() > 1e-9 ? normalize(fwd) : Vec3(0, 0, 1);
    Vec3 right = cross(fwd, cam.up);
    right = right.lengthSquared() > 1e-9 ? normalize(right) : Vec3(1, 0, 0);
    const Vec3 up = normalize(cross(right, fwd));
    // back to front, then into the groups in that order (each group stays sorted)
    order_.clear();
    for (std::size_t i = 0; i < ps.size(); ++i) order_.push_back({(ps[i].pos - cam.position).lengthSquared(), i});
    std::sort(order_.begin(), order_.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    for (const auto& [d2, i] : order_) {
        const Particle& p = ps[i];
        const Real a = pool_.drawnAlpha(p);
        if (a < 0.04) continue;
        int b = 0;
        for (int s = 1; s < kOpacitySteps; ++s)
            if (std::fabs(kOpacity[s] - a) < std::fabs(kOpacity[b] - a)) b = s;
        InstanceGroup* grp = ctx.world.get<InstanceGroup>(groupsFor(ctx, p.effect)[static_cast<std::size_t>(b)]);
        if (!grp) continue;
        const ParticleEffect& E = pool_.book().at(p.effect);
        const Real size = pool_.drawnSize(p);
        Vec3 ax = right * size, ay = up * size;
        if (E.stretch > 0) {   // a line along its flight across the view
            const Vec3 rel = p.vel - p.ref;
            const Vec3 across = rel - fwd * dot(rel, fwd);
            const Real len = across.length();
            if (len > 1e-3) {
                const Vec3 dir = across * (1.0 / len);
                ax = dir * (size + len * E.stretch);
                ay = normalize(cross(fwd, dir)) * size;
            }
        }
        Mat4 m;
        for (int r = 0; r < 3; ++r) {
            m.m[r][0] = r == 0 ? ax.x : r == 1 ? ax.y : ax.z;
            m.m[r][1] = r == 0 ? ay.x : r == 1 ? ay.y : ay.z;
            m.m[r][2] = -(r == 0 ? fwd.x : r == 1 ? fwd.y : fwd.z) * size;
        }
        m.m[0][3] = p.pos.x; m.m[1][3] = p.pos.y; m.m[2][3] = p.pos.z;
        grp->transforms.push_back(m);
    }
}

void ParticleSystem::update(FrameContext& ctx) {
    const Real dt = std::clamp(ctx.frameDelta, 0.0, 0.1);
    spend(ctx, dt);
    pool_.step(dt);
    draw(ctx);
}

}  // namespace engine
