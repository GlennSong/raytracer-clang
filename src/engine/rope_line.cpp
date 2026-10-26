#include "rope_line.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
constexpr int kPasses = 10;        // constraint passes per step
constexpr Real kDrag = 0.6;        // 1/s: the air taking out the rope's own motion
constexpr Real kRadius = 0.03;     // m: how far a point is held off a surface it lies on
constexpr Real kFriction = 0.6;    // of the slide along a surface a point keeps per step
}  // namespace

void RopeLine::reset(const Vec3& at) {
    p_.assign(kPoints, at);
    prev_ = p_;
}

void RopeLine::step(const Vec3& start, const Vec3& end, bool endFree, Real restLength, Real dt, const Sweep& sweep) {
    if (p_.empty()) reset(start);
    const int n = kPoints;
    const int last = n - 1;
    const Real seg = std::max(restLength, Real(0)) / last;
    const Real keep = std::exp(-kDrag * dt);
    const Vec3 g(0, -9.81 * dt * dt, 0);
    // VERLET: each free point carries on as it was going, under gravity
    std::vector<Vec3> before = p_;
    for (int i = 1; i < n; ++i) {
        if (i == last && !endFree) continue;
        const Vec3 moved = (p_[i] - prev_[i]) * keep;
        prev_[i] = p_[i];
        p_[i] = p_[i] + moved + g;
    }
    prev_[0] = p_[0];
    p_[0] = start;
    if (!endFree) {
        prev_[last] = p_[last];
        p_[last] = end;
    }
    auto collide = [&](int i, const Vec3& from) {
        if (!sweep) return;
        Vec3 hit, normal;
        if (!sweep(from, p_[i], hit, normal)) return;
        // held off the surface; what was left of the move slides along it, with friction
        const Vec3 rest = hit + normal * kRadius;
        const Vec3 over = p_[i] - rest;
        const Vec3 along = over - normal * dot(over, normal);
        const Vec3 vel = p_[i] - prev_[i];
        p_[i] = rest + along * kFriction;
        prev_[i] = p_[i] - (vel - normal * dot(vel, normal)) * kFriction;   // the bounce off it gone
    };
    for (int i = 1; i < last; ++i) collide(i, before[i]);
    if (endFree) collide(last, before[last]);
    // THE LINKS: pull, never push; then the long-range limits from each pinned end
    const std::vector<Vec3> swept = p_;
    for (int pass = 0; pass < kPasses; ++pass) {
        for (int i = 0; i < last; ++i) {
            Vec3 d = p_[i + 1] - p_[i];
            const Real len = d.length();
            if (len <= seg || len < 1e-9) continue;
            d = d * ((len - seg) / len);
            const bool pinA = i == 0, pinB = i + 1 == last && !endFree;
            if (pinA && pinB) continue;
            if (pinA) p_[i + 1] = p_[i + 1] - d;
            else if (pinB) p_[i] = p_[i] + d;
            else {
                p_[i] = p_[i] + d * 0.5;
                p_[i + 1] = p_[i + 1] - d * 0.5;
            }
        }
        // no point further from the hand than the rope between them (nor from the hook, when it holds)
        for (int i = 1; i < n; ++i) {
            if (i == last && !endFree) continue;
            const Vec3 d = p_[i] - p_[0];
            const Real len = d.length(), most = seg * i;
            if (len > most && len > 1e-9) p_[i] = p_[0] + d * (most / len);
        }
        if (!endFree)
            for (int i = 1; i < last; ++i) {
                const Vec3 d = p_[i] - p_[last];
                const Real len = d.length(), most = seg * (last - i);
                if (len > most && len > 1e-9) p_[i] = p_[last] + d * (most / len);
            }
    }
    // TAUT: the ends as far apart as the rope is long -- it runs straight. The passes alone leave a bow (gravity
    // against links pulling at a shallow angle converges slowly), so near taut the points are drawn onto the line.
    if (!endFree && restLength > 1e-6) {
        const Real span = (p_[last] - p_[0]).length();
        const Real taut = std::clamp((span / restLength - 0.97) / 0.03, Real(0), Real(1));
        if (taut > 0)
            for (int i = 1; i < last; ++i) {
                const Vec3 on = p_[0] + (p_[last] - p_[0]) * (static_cast<Real>(i) / last);
                const Vec3 pull = (on - p_[i]) * taut;
                p_[i] = p_[i] + pull;
                prev_[i] = prev_[i] + pull;   // carried, not flung: the straightening adds no speed
            }
    }
    // the pull may have dragged a point into what it was lying on: swept again from where it stood
    for (int i = 1; i < last; ++i) collide(i, swept[i]);
    if (endFree) collide(last, swept[last]);
}

}  // namespace engine
