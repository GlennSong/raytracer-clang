#include "particles.h"
#include "../proc_rng.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
// THE BUILT-INS: what assets/scripts/particles.lua ships with (keep the two alike; the Lua wins once loaded).
std::vector<ParticleEffect> builtins() {
    std::vector<ParticleEffect> out;
    auto add = [&](const char* name) -> ParticleEffect& {
        out.emplace_back();
        out.back().name = name;
        return out.back();
    };
    {   // kicked off the ground: tan, low, spreading and settling, gone in about a second
        ParticleEffect& e = add("dust");
        e.gravity = 1.0; e.drag = 3.0; e.lifeMin = 0.7; e.lifeMax = 1.3;
        e.colour = Vec3(0.62, 0.55, 0.45); e.sizeStart = 0.18; e.sizeEnd = 0.75; e.alpha = 0.7;
        e.fadeIn = 0.1; e.fadeOut = 0.6;
        e.count = 8; e.speed = 1.0; e.up = 0.4; e.spread = 0.9; e.jitter = 0.15;
    }
    {   // tyre smoke: big soft billows that hang, drift up a little and grow
        ParticleEffect& e = add("smoke");
        e.gravity = -0.4; e.drag = 1.6; e.lifeMin = 1.4; e.lifeMax = 2.4;
        e.colour = Vec3(0.82, 0.82, 0.84); e.glow = Vec3(0.12, 0.12, 0.13); e.sizeStart = 0.5; e.sizeEnd = 2.6;
        e.alpha = 0.75;
        e.fadeIn = 0.12; e.fadeOut = 0.65;
        e.count = 1; e.speed = 1.0; e.up = 0.5; e.spread = 0.7; e.jitter = 0.25;
    }
    {   // under water: small rings rising (buoyant, held to ~0.7 m/s by the water), wobbling
        ParticleEffect& e = add("bubble");
        e.gravity = -6.0; e.drag = 9.0; e.wobble = 0.15; e.lifeMin = 1.5; e.lifeMax = 3.0;
        e.shape = ParticleShape::Ring; e.colour = Vec3(0.85, 0.95, 1.0); e.glow = Vec3(0.05, 0.07, 0.08);
        e.sizeStart = 0.035; e.sizeEnd = 0.07; e.alpha = 0.8; e.fadeIn = 0.05; e.fadeOut = 0.15;
        e.count = 1; e.speed = 0.3; e.spread = 0.15; e.jitter = 0.03;
    }
    {   // a breath let out under water: a gulp of bubbles
        ParticleEffect e = out.back();
        e.name = "breath";
        e.count = 14; e.speed = 0.5; e.spread = 0.35; e.jitter = 0.06;
        out.push_back(e);
    }
    {   // the air going past under a canopy / in a wingsuit: a faint line where it was, stretched against the body
        ParticleEffect& e = add("streak");
        e.lifeMin = 0.5; e.lifeMax = 0.9; e.shape = ParticleShape::Glow; e.colour = Vec3(1, 1, 1);
        e.glow = Vec3(0.45, 0.45, 0.48); e.sizeStart = 0.05; e.sizeEnd = 0.05; e.alpha = 0.38;
        e.fadeIn = 0.3; e.fadeOut = 0.4; e.stretch = 0.16;
        e.count = 1; e.speed = 0; e.spread = 0;
    }
    {   // a hook biting, a truck grinding: hot, quick, falling, a line along its flight
        ParticleEffect& e = add("spark");
        e.gravity = 9.81; e.drag = 1.0; e.lifeMin = 0.18; e.lifeMax = 0.45; e.shape = ParticleShape::Glow;
        e.colour = Vec3(0, 0, 0); e.glow = Vec3(6.0, 3.0, 0.9); e.sizeStart = 0.03; e.sizeEnd = 0.015;
        e.alpha = 1.0; e.fadeIn = 0.0; e.fadeOut = 0.5; e.stretch = 0.025;
        e.count = 12; e.speed = 3.0; e.up = 1.5; e.spread = 4.0; e.jitter = 0.03;
    }
    {   // into the water: white drops thrown up and falling back
        ParticleEffect& e = add("splash");
        e.gravity = 9.81; e.drag = 0.6; e.lifeMin = 0.5; e.lifeMax = 0.9; e.shape = ParticleShape::Glow;
        e.colour = Vec3(0.95, 0.97, 1.0);
        e.glow = Vec3(0.35, 0.36, 0.38); e.sizeStart = 0.09; e.sizeEnd = 0.05; e.alpha = 0.85;
        e.fadeIn = 0.0; e.fadeOut = 0.3; e.stretch = 0.02;
        e.count = 16; e.speed = 0; e.up = 2.5; e.spread = 1.4; e.jitter = 0.4;
    }
    {   // a vent, a kettle, a manhole on a cold night: white wisps rising and spreading
        ParticleEffect& e = add("steam");
        e.gravity = -1.2; e.drag = 1.2; e.lifeMin = 1.5; e.lifeMax = 3.0; e.colour = Vec3(0.92, 0.93, 0.95);
        e.glow = Vec3(0.18, 0.18, 0.19); e.sizeStart = 0.25; e.sizeEnd = 1.8; e.alpha = 0.6; e.fadeIn = 0.15; e.fadeOut = 0.7;
        e.count = 1; e.speed = 0; e.up = 0.8; e.spread = 0.3; e.jitter = 0.15;
    }
    return out;
}
}  // namespace

ParticleBook::ParticleBook() {
    for (const ParticleEffect& e : builtins()) set(e);
}

ParticleBook& ParticleBook::global() {
    static ParticleBook book;
    return book;
}

void ParticleBook::set(const ParticleEffect& e) {
    const auto it = byName_.find(e.name);
    if (it != byName_.end()) {
        effects_[static_cast<std::size_t>(it->second)] = e;
        return;
    }
    byName_[e.name] = static_cast<int>(effects_.size());
    effects_.push_back(e);
}

int ParticleBook::find(const std::string& name) const {
    const auto it = byName_.find(name);
    return it == byName_.end() ? -1 : it->second;
}

void ParticlePool::emit(int effect, const Vec3& pos, const Vec3& vel, Real size, Real ceiling, const Vec3& ref) {
    if (effect < 0 || static_cast<std::size_t>(effect) >= book_->size()) return;
    if (p_.size() >= kMax) p_.erase(p_.begin(), p_.begin() + static_cast<std::ptrdiff_t>(kMax / 8));   // the oldest
    const ParticleEffect& E = book_->at(effect);
    ProcUniform<float> u(0.0f, 1.0f);
    Particle p;
    p.pos = pos;
    p.vel = vel;
    p.ref = ref;
    p.effect = static_cast<uint16_t>(effect);
    p.life = static_cast<float>(E.lifeMin + (E.lifeMax - E.lifeMin) * u(rng_));
    p.size = static_cast<float>(size);
    p.ceiling = static_cast<float>(ceiling);
    p.phase = u(rng_) * 6.2831853f;
    p_.push_back(p);
}

void ParticlePool::burst(int effect, const Vec3& pos, const Vec3& dir, int count, Real scale, Real ceiling,
                         const Vec3& ref) {
    if (effect < 0 || static_cast<std::size_t>(effect) >= book_->size()) return;
    const ParticleEffect& E = book_->at(effect);
    ProcUniform<Real> u(-1.0, 1.0);
    auto ball = [&]() {
        for (;;) {
            const Vec3 v(u(rng_), u(rng_), u(rng_));
            if (v.lengthSquared() <= 1.0) return v;
        }
    };
    const Vec3 base = dir * (E.speed * scale) + Vec3(0, E.up * scale, 0);
    const int n = count < 0 ? E.count : count;
    for (int i = 0; i < n; ++i)
        emit(effect, pos + ball() * E.jitter, base + ball() * (E.spread * scale), 1.0 + 0.25 * u(rng_), ceiling, ref);
}

void ParticlePool::step(Real dt) {
    if (dt <= 0) return;
    for (Particle& p : p_) {
        const ParticleEffect& E = book_->at(p.effect);
        p.age += static_cast<float>(dt);
        p.vel = p.vel * std::exp(-E.drag * dt) + Vec3(0, -E.gravity * dt, 0);
        Vec3 move = p.vel * dt;
        if (E.wobble > 0) {
            const Real w = p.age * 9.0 + p.phase;
            move = move + Vec3(std::cos(w), 0, std::sin(w * 1.3)) * (E.wobble * dt);
        }
        p.pos = p.pos + move;
        if (p.pos.y > p.ceiling) p.age = p.life;   // a bubble at the surface: popped
    }
    p_.erase(std::remove_if(p_.begin(), p_.end(), [](const Particle& p) { return p.age >= p.life; }), p_.end());
}

Real ParticlePool::drawnSize(const Particle& p) const {
    const ParticleEffect& E = book_->at(p.effect);
    const Real t = std::clamp(static_cast<Real>(p.age / std::max(p.life, 1e-3f)), Real(0), Real(1));
    const Real e = 1.0 - (1.0 - t) * (1.0 - t);   // grows fast, then slows
    return (E.sizeStart + (E.sizeEnd - E.sizeStart) * e) * p.size;
}

Real ParticlePool::drawnAlpha(const Particle& p) const {
    const ParticleEffect& E = book_->at(p.effect);
    const Real t = std::clamp(static_cast<Real>(p.age / std::max(p.life, 1e-3f)), Real(0), Real(1));
    Real a = 1;
    if (E.fadeIn > 0 && t < E.fadeIn) a = t / E.fadeIn;
    if (E.fadeOut > 0 && t > 1.0 - E.fadeOut) a = std::min(a, (1.0 - t) / E.fadeOut);
    return E.alpha * a;
}

void requestParticles(World& world, const ParticleRequest& r) {
    ParticleRequests* q = nullptr;
    world.each<ParticleRequests>([&](Entity, ParticleRequests& pr) { if (!q) q = &pr; });
    if (!q) {
        const Entity e = world.create();
        world.add<ParticleRequests>(e, ParticleRequests{});
        q = world.get<ParticleRequests>(e);
    }
    if (q && q->queue.size() < 512) q->queue.push_back(r);   // (nobody spending them: don't grow without end)
}

}  // namespace engine
