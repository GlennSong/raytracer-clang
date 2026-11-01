#include "seat_motion.h"

#include <cmath>

namespace engine {

namespace {

// a spring toward zero, stepped (semi-implicit Euler)
void spring(Vec3& x, Vec3& rate, Real omega, Real zeta, Real dt) {
    rate = rate - (x * (omega * omega) + rate * (2 * zeta * omega)) * dt;
    x = x + rate * dt;
}

// stopped at `most` from the seat, the way a belt stops you: what was going outward comes back off it a little
void stopAt(Vec3& x, Vec3& rate, Real most) {
    const Real l = std::sqrt(x.x * x.x + x.z * x.z);
    if (l <= most) return;
    const Vec3 out(x.x / l, 0, x.z / l);
    x = Vec3(out.x * most, x.y, out.z * most);
    const Real r = rate.x * out.x + rate.z * out.z;
    if (r > 0) rate = rate - out * (r * 1.2);
}

}  // namespace

Real RideJolt::step(const Vec3& velocity, Real dt, Vec3* before) {
    clock += dt;
    samples.push_back({clock, velocity});
    while (!samples.empty() && samples.front().t < clock - kWindow) samples.erase(samples.begin());
    Real most = 0;
    Vec3 was = velocity;
    for (const Sample& s : samples) {
        const Vec3 d = velocity - s.v;
        const Real j = std::sqrt(d.x * d.x + 0.25 * d.y * d.y + d.z * d.z);
        if (j > most) { most = j; was = s.v; }
    }
    if (before) *before = was;
    return most;
}

void SeatSway::step(const Vec3& dv, Real dt) {
    if (dt <= 0) return;
    // thrown against the change: the vehicle stops, you go on (a forward lean); it is struck from the left, you go left
    const Vec3 push(-dv.x, 0, -dv.z);
    leanRate = leanRate + push * 0.5;
    nodRate = nodRate + push * 0.75;
    spring(lean, leanRate, 8.0, 0.55, dt);   // the torso: ~0.8 s and it is back
    spring(nod, nodRate, 13.0, 0.35, dt);    // the head on the neck: quicker, and it snaps back past
    stopAt(lean, leanRate, kLeanMax);
    stopAt(nod, nodRate, kNodMax);
}

}  // namespace engine
