#ifndef RAYTRACER_CURVE_H
#define RAYTRACER_CURVE_H

#include "rt_math.h"
#include <vector>
#include <cmath>

// A piecewise-cubic Hermite spline kernel (ADR-0031). One template serves every
// curve consumer: procgen branch centerlines (Spline<Vec3>), animation F-curves
// (Spline<float>), and 2D/SVG paths later. Knots store value + in/out tangent —
// the canonical form Catmull-Rom (here), cubic Bezier, and keyframes all lower
// to. Per-consumer services (arc-length, rotation-minimizing frames) sit on top.
namespace engine {

// Norm of a curve value, for arc-length. Overloaded per supported value type.
inline double curveNorm(double v) { return std::abs(v); }
inline double curveNorm(float v) { return std::abs(static_cast<double>(v)); }
inline double curveNorm(const Vec3& v) { return v.length(); }

template <typename T>
class Spline {
public:
    struct Knot {
        T value{};
        T inTangent{};
        T outTangent{};
    };
    std::vector<Knot> knots;

    int segments() const {
        return knots.size() < 2 ? 0 : static_cast<int>(knots.size()) - 1;
    }
    bool valid() const { return knots.size() >= 2; }

    // Catmull-Rom through `points` (uniform): the tangent at point i is
    // (p[i+1]-p[i-1]) * 0.5 * (1-tension), one-sided at the ends. tension in
    // [0,1] (0 = standard Catmull-Rom, 1 = straight/zero tangents).
    static Spline catmullRom(const std::vector<T>& points, double tension = 0.0);

    // Evaluate at u in [0, segments()] (integer part = segment, fraction =
    // position within it); clamped outside the range.
    T eval(double u) const;
    T derivative(double u) const;   // d/du (an unnormalized tangent)

    // Total arc length (polyline approximation at `subdiv` samples/segment).
    double length(int subdiv = 16) const;

    // `count` parameter values in [0, segments()] spaced at equal arc length,
    // endpoints included — for placing evenly-spaced rings along the curve.
    std::vector<double> uniformArcLengthParams(int count, int subdiv = 16) const;

private:
    void localOf(double u, int& seg, double& t) const;
};

// A frame carried along a 3D curve: the tangent plus two orthonormal
// perpendiculars (right, up) for orienting a swept cross-section.
struct CurveFrame {
    Vec3 tangent;
    Vec3 right;
    Vec3 up;
};

// Rotation-minimizing frames along a polyline (Wang et al. double-reflection):
// the perpendiculars are transported with minimal twist, so swept tubes don't
// spiral. `tangents` are the (unnormalized) curve derivatives at each point.
std::vector<CurveFrame> rotationMinimizingFrames(const std::vector<Vec3>& points,
                                                 const std::vector<Vec3>& tangents);

// ---- template implementation ---------------------------------------------

template <typename T>
void Spline<T>::localOf(double u, int& seg, double& t) const {
    int segs = segments();
    if (segs <= 0) { seg = 0; t = 0.0; return; }
    if (u <= 0.0) { seg = 0; t = 0.0; return; }
    if (u >= segs) { seg = segs - 1; t = 1.0; return; }
    double f = std::floor(u);
    seg = static_cast<int>(f);
    t = u - f;
}

template <typename T>
Spline<T> Spline<T>::catmullRom(const std::vector<T>& p, double tension) {
    Spline s;
    const int n = static_cast<int>(p.size());
    if (n == 0) return s;
    if (n == 1) { s.knots.push_back({p[0], T{}, T{}}); return s; }
    const double c = 0.5 * (1.0 - tension);
    s.knots.resize(n);
    for (int i = 0; i < n; i++) {
        const T& prev = p[i > 0 ? i - 1 : 0];
        const T& next = p[i < n - 1 ? i + 1 : n - 1];
        T tan = (next - prev) * c;
        s.knots[i].value = p[i];
        s.knots[i].inTangent = tan;
        s.knots[i].outTangent = tan;
    }
    return s;
}

template <typename T>
T Spline<T>::eval(double u) const {
    if (knots.empty()) return T{};
    if (knots.size() == 1) return knots[0].value;
    int i; double t;
    localOf(u, i, t);
    const T& p0 = knots[i].value;
    const T& p1 = knots[i + 1].value;
    const T& m0 = knots[i].outTangent;
    const T& m1 = knots[i + 1].inTangent;
    double t2 = t * t, t3 = t2 * t;
    double h00 = 2 * t3 - 3 * t2 + 1;
    double h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2;
    double h11 = t3 - t2;
    return p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11;
}

template <typename T>
T Spline<T>::derivative(double u) const {
    if (knots.size() < 2) return T{};
    int i; double t;
    localOf(u, i, t);
    const T& p0 = knots[i].value;
    const T& p1 = knots[i + 1].value;
    const T& m0 = knots[i].outTangent;
    const T& m1 = knots[i + 1].inTangent;
    double t2 = t * t;
    double d00 = 6 * t2 - 6 * t;
    double d10 = 3 * t2 - 4 * t + 1;
    double d01 = -6 * t2 + 6 * t;
    double d11 = 3 * t2 - 2 * t;
    return p0 * d00 + m0 * d10 + p1 * d01 + m1 * d11;
}

template <typename T>
double Spline<T>::length(int subdiv) const {
    if (!valid()) return 0.0;
    int steps = segments() * (subdiv < 1 ? 1 : subdiv);
    double total = 0.0;
    T prev = eval(0.0);
    for (int k = 1; k <= steps; k++) {
        T cur = eval(static_cast<double>(k) / steps * segments());
        total += curveNorm(cur - prev);
        prev = cur;
    }
    return total;
}

template <typename T>
std::vector<double> Spline<T>::uniformArcLengthParams(int count, int subdiv) const {
    std::vector<double> out;
    if (count < 1) return out;
    if (count == 1 || !valid()) { out.assign(count, 0.0); return out; }

    int steps = segments() * (subdiv < 1 ? 1 : subdiv);
    std::vector<double> us(steps + 1), cum(steps + 1, 0.0);
    T prev = eval(0.0);
    us[0] = 0.0;
    for (int k = 1; k <= steps; k++) {
        double u = static_cast<double>(k) / steps * segments();
        T cur = eval(u);
        cum[k] = cum[k - 1] + curveNorm(cur - prev);
        us[k] = u;
        prev = cur;
    }

    out.resize(count);
    double total = cum[steps];
    if (total <= 1e-12) { out.assign(count, 0.0); return out; }

    int j = 0;
    for (int i = 0; i < count; i++) {
        double target = total * i / (count - 1);
        while (j < steps && cum[j + 1] < target) j++;
        if (j >= steps) j = steps - 1;  // fp rounding can push target past cum[steps];
                                        // keep j+1 within [0, steps] (last sample is set below)
        double segLen = cum[j + 1] - cum[j];
        double f = segLen > 1e-12 ? (target - cum[j]) / segLen : 0.0;
        out[i] = us[j] + (us[j + 1] - us[j]) * f;
    }
    out.front() = 0.0;
    out.back() = segments();
    return out;
}

}  // namespace engine

#endif
