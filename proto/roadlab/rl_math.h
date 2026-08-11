#ifndef ROADLAB_RL_MATH_H
#define ROADLAB_RL_MATH_H

// Self-contained math for the roadlab prototype. Deliberately does NOT include
// the engine's rt_math.h: this app is a clean-room prototype of the road system
// (proto/roadlab/README.md) and must stay buildable with nothing but the STL.

#include <cmath>
#include <cstdint>
#include <algorithm>

namespace roadlab {

// World is Y-up. Plan view is the XZ plane; headings are measured in XZ from
// +X toward +Z. Everything lateral lives in that plane, which is what makes the
// cross-section a genuinely 2D problem no matter how the road climbs.
struct Vec2 {
    double x = 0, y = 0;
    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}
};

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
inline Vec2 operator*(double s, Vec2 a) { return {a.x * s, a.y * s}; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline double lengthSq(Vec2 a) { return dot(a, a); }
inline Vec2 normalize(Vec2 a) {
    double l = length(a);
    return l > 1e-12 ? Vec2{a.x / l, a.y / l} : Vec2{1, 0};
}
inline Vec2 perpLeft(Vec2 a) { return {-a.y, a.x}; }

inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(double s, Vec3 a) { return {a.x * s, a.y * s, a.z * s}; }
inline Vec3 operator*(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Vec3& operator+=(Vec3& a, Vec3 b) { a = a + b; return a; }
inline Vec3& operator*=(Vec3& a, double s) { a = a * s; return a; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline double lengthSq(Vec3 a) { return dot(a, a); }
inline Vec3 normalize(Vec3 a) {
    double l = length(a);
    return l > 1e-12 ? Vec3{a.x / l, a.y / l, a.z / l} : Vec3{0, 1, 0};
}
inline Vec2 planOf(Vec3 a) { return {a.x, a.z}; }
inline Vec3 worldOf(Vec2 p, double y) { return {p.x, y, p.y}; }

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;

inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double lerp(double a, double b, double t) { return a + (b - a) * t; }
inline Vec3 lerp(Vec3 a, Vec3 b, double t) { return a + (b - a) * t; }
inline double saturate(double v) { return clampd(v, 0.0, 1.0); }

// Hermite smoothstep. Used both for shading edges and for width blends in the
// transition solver, which is why it lives here rather than in either.
inline double smoothstepd(double e0, double e1, double x) {
    if (e1 - e0 < 1e-12) return x < e0 ? 0.0 : 1.0;
    double t = saturate((x - e0) / (e1 - e0));
    return t * t * (3.0 - 2.0 * t);
}

// Wrap to (-pi, pi].
inline double wrapPi(double a) {
    while (a > kPi) a -= kTwoPi;
    while (a <= -kPi) a += kTwoPi;
    return a;
}

// Signed shortest angular difference a - b.
inline double angleDiff(double a, double b) { return wrapPi(a - b); }

inline Vec2 dirOf(double heading) { return {std::cos(heading), std::sin(heading)}; }
inline double headingOf(Vec2 d) { return std::atan2(d.y, d.x); }

// A cubic in a local parameter: v(ds) = a + b*ds + c*ds^2 + d*ds^3. Every
// continuously-varying quantity in the road model (lane width, elevation,
// superelevation, lateral offset) is one of these over an s-range, which is what
// makes tapers, vertical curves and banking the same machinery.
struct Poly3 {
    double a = 0, b = 0, c = 0, d = 0;

    double eval(double ds) const { return a + ds * (b + ds * (c + ds * d)); }
    double deriv(double ds) const { return b + ds * (2.0 * c + ds * 3.0 * d); }

    // The same cubic measured from a new origin `c` further along: q(u) = p(u+c).
    // Needed whenever a piecewise function has to be re-based — clipping a road
    // to a window, or exporting a section that starts partway through a piece.
    Poly3 shifted(double c) const {
        return {a + c * (b + c * (this->c + c * d)),
                b + c * (2.0 * this->c + 3.0 * d * c),
                this->c + 3.0 * d * c,
                d};
    }

    static Poly3 constant(double v) { return {v, 0, 0, 0}; }
    static Poly3 linear(double v0, double v1, double len) {
        if (len <= 1e-9) return constant(v1);
        return {v0, (v1 - v0) / len, 0, 0};
    }
    // Cubic with zero slope at both ends — the shape a lane taper, a vertical
    // curve and a superelevation runoff all want.
    static Poly3 smooth(double v0, double v1, double len) {
        if (len <= 1e-9) return constant(v1);
        double dv = v1 - v0;
        return {v0, 0, 3.0 * dv / (len * len), -2.0 * dv / (len * len * len)};
    }
    // General Hermite with explicit end slopes (used to keep grades continuous
    // across elevation-profile joints).
    static Poly3 hermite(double v0, double m0, double v1, double m1, double len) {
        if (len <= 1e-9) return constant(v1);
        double L = len, L2 = L * L, L3 = L2 * L;
        return {v0, m0,
                (3.0 * (v1 - v0) - (2.0 * m0 + m1) * L) / L2,
                (2.0 * (v0 - v1) + (m0 + m1) * L) / L3};
    }
};

// Deterministic PRNG. Procgen must be reproducible from a seed or none of the
// generated scenes are testable.
struct Rng {
    uint64_t state;
    explicit Rng(uint64_t seed = 0x9E3779B97F4A7C15ull) : state(seed ? seed : 1) {}
    uint64_t nextU64() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    }
    double next01() { return double(nextU64() >> 11) * (1.0 / 9007199254740992.0); }
    double range(double lo, double hi) { return lo + (hi - lo) * next01(); }
    int rangeI(int lo, int hi) {  // inclusive
        if (hi <= lo) return lo;
        return lo + int(nextU64() % uint64_t(hi - lo + 1));
    }
    bool chance(double p) { return next01() < p; }
};

// Value noise + fbm, for procedural asphalt aggregate and wear. Hash-based so it
// is stateless and can be evaluated per-fragment in any order.
double valueNoise(double x, double y);
double fbm(double x, double y, int octaves, double lacunarity = 2.0, double gain = 0.5);
// Worley/cellular F1 distance — the aggregate look in asphalt needs cells, not
// just smooth noise.
double worley(double x, double y);

// 8-point Gauss-Legendre integration of f over [0, h]. The clothoid position
// integral has no closed form; this gives ~1e-12 over the sub-intervals the
// spine splits into.
template <typename F>
double gauss8(F f, double h) {
    static const double kX[4] = {0.1834346424956498, 0.5255324099163290,
                                 0.7966664774136267, 0.9602898564975363};
    static const double kW[4] = {0.3626837833783620, 0.3137066458778873,
                                 0.2223810344533745, 0.1012285362903763};
    double half = 0.5 * h, sum = 0.0;
    for (int i = 0; i < 4; ++i) {
        sum += kW[i] * (f(half * (1.0 - kX[i])) + f(half * (1.0 + kX[i])));
    }
    return sum * half;
}

}  // namespace roadlab

#endif
