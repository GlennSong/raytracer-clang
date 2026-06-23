#include "editable_curve.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
double vlen(const Vec3& v) { return std::sqrt(v.lengthSquared()); }
}  // namespace

int EditableCurve::segmentCount() const {
    int n = static_cast<int>(knots.size());
    if (n < 2) return 0;
    return closed ? n : n - 1;
}

// The handle a knot actually uses: the stored one, or — for an Auto knot — a
// Catmull-Rom tangent derived from its neighbours (a third of the tangent, the
// Catmull-Rom -> Bezier conversion).
static Vec3 autoTangent(const EditableCurve& c, int i) {
    int n = static_cast<int>(c.knots.size());
    int p = c.closed ? (i - 1 + n) % n : (i > 0 ? i - 1 : i);
    int q = c.closed ? (i + 1) % n : (i < n - 1 ? i + 1 : i);
    Vec3 d;
    if (p == i)      d = c.knots[q].position - c.knots[i].position;          // start cap
    else if (q == i) d = c.knots[i].position - c.knots[p].position;          // end cap
    else             d = (c.knots[q].position - c.knots[p].position) * 0.5;  // interior
    return d * (1.0 / 3.0);
}

static Vec3 effOut(const EditableCurve& c, int i) {
    return c.knots[i].mode == KnotMode::Auto ? autoTangent(c, i) : c.knots[i].outHandle;
}
static Vec3 effIn(const EditableCurve& c, int i) {
    return c.knots[i].mode == KnotMode::Auto ? autoTangent(c, i) * -1.0 : c.knots[i].inHandle;
}

void EditableCurve::segmentControls(int s, Vec3& p0, Vec3& p1, Vec3& p2, Vec3& p3) const {
    int n = static_cast<int>(knots.size());
    int a = s, b = closed ? (s + 1) % n : s + 1;
    p0 = knots[a].position;
    p1 = p0 + effOut(*this, a);
    p3 = knots[b].position;
    p2 = p3 + effIn(*this, b);
}

static Vec3 bezier(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, double u) {
    double v = 1.0 - u;
    return p0 * (v * v * v) + p1 * (3 * v * v * u) + p2 * (3 * v * u * u) + p3 * (u * u * u);
}
static Vec3 bezierDeriv(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, double u) {
    double v = 1.0 - u;
    return (p1 - p0) * (3 * v * v) + (p2 - p1) * (6 * v * u) + (p3 - p2) * (3 * u * u);
}

// Map t in [0,1] to a segment index + local parameter.
static void locate(int nseg, double t, int& seg, double& u) {
    t = std::max(0.0, std::min(1.0, t));
    double ft = t * nseg;
    seg = std::min(static_cast<int>(ft), nseg - 1);
    u = ft - seg;
}

Vec3 EditableCurve::position(double t) const {
    int nseg = segmentCount();
    if (nseg <= 0) return knots.empty() ? Vec3(0, 0, 0) : knots[0].position;
    int seg; double u; locate(nseg, t, seg, u);
    Vec3 p0, p1, p2, p3; segmentControls(seg, p0, p1, p2, p3);
    return bezier(p0, p1, p2, p3, u);
}

Vec3 EditableCurve::tangent(double t) const {
    int nseg = segmentCount();
    if (nseg <= 0) return Vec3(0, 0, 0);
    int seg; double u; locate(nseg, t, seg, u);
    Vec3 p0, p1, p2, p3; segmentControls(seg, p0, p1, p2, p3);
    return bezierDeriv(p0, p1, p2, p3, u);
}

std::vector<Vec3> EditableCurve::sample(int perSegment) const {
    std::vector<Vec3> out;
    int nseg = segmentCount();
    if (nseg <= 0) {
        if (!knots.empty()) out.push_back(knots[0].position);
        return out;
    }
    perSegment = std::max(1, perSegment);
    out.reserve(static_cast<std::size_t>(nseg) * perSegment + 1);
    for (int s = 0; s < nseg; ++s) {
        Vec3 p0, p1, p2, p3; segmentControls(s, p0, p1, p2, p3);
        int start = (s == 0) ? 0 : 1;                  // share the joint vertex
        for (int k = start; k <= perSegment; ++k)
            out.push_back(bezier(p0, p1, p2, p3, static_cast<double>(k) / perSegment));
    }
    return out;
}

double EditableCurve::length(int perSeg) const {
    std::vector<Vec3> pts = sample(perSeg);
    double L = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) L += vlen(pts[i] - pts[i - 1]);
    return L;
}

Vec3 EditableCurve::atDistance(double dist, int perSeg) const {
    std::vector<Vec3> pts = sample(perSeg);
    if (pts.empty()) return Vec3(0, 0, 0);
    if (pts.size() == 1 || dist <= 0) return pts.front();
    double acc = 0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        double seg = vlen(pts[i] - pts[i - 1]);
        if (acc + seg >= dist) {
            double f = seg > 1e-9 ? (dist - acc) / seg : 0.0;
            return pts[i - 1] + (pts[i] - pts[i - 1]) * f;
        }
        acc += seg;
    }
    return pts.back();
}

Vec3 EditableCurve::inHandleWorld(int i) const {
    return knots[i].position + effIn(*this, i);
}
Vec3 EditableCurve::outHandleWorld(int i) const {
    return knots[i].position + effOut(*this, i);
}

void EditableCurve::moveKnot(int i, const Vec3& p) {
    if (i < 0 || i >= static_cast<int>(knots.size())) return;
    knots[i].position = p;                              // handles are offsets -> ride along
}

void EditableCurve::setMode(int i, KnotMode mode) {
    if (i < 0 || i >= static_cast<int>(knots.size())) return;
    if (knots[i].mode == KnotMode::Auto && mode != KnotMode::Auto) {
        knots[i].outHandle = autoTangent(*this, i);    // bake the auto handles before editing
        knots[i].inHandle = autoTangent(*this, i) * -1.0;
    }
    knots[i].mode = mode;
}

void EditableCurve::setOutHandle(int i, const Vec3& worldPos) {
    if (i < 0 || i >= static_cast<int>(knots.size())) return;
    if (knots[i].mode == KnotMode::Auto) setMode(i, KnotMode::Aligned);
    Vec3 offset = worldPos - knots[i].position;
    knots[i].outHandle = offset;
    if (knots[i].mode == KnotMode::Aligned) {          // keep in/out collinear (smooth)
        double inLen = vlen(knots[i].inHandle);
        double ol = vlen(offset);
        if (inLen < 1e-9) inLen = ol;
        if (ol > 1e-9) knots[i].inHandle = offset * (-inLen / ol);
    }
}

void EditableCurve::setInHandle(int i, const Vec3& worldPos) {
    if (i < 0 || i >= static_cast<int>(knots.size())) return;
    if (knots[i].mode == KnotMode::Auto) setMode(i, KnotMode::Aligned);
    Vec3 offset = worldPos - knots[i].position;
    knots[i].inHandle = offset;
    if (knots[i].mode == KnotMode::Aligned) {
        double outLen = vlen(knots[i].outHandle);
        double ol = vlen(offset);
        if (outLen < 1e-9) outLen = ol;
        if (ol > 1e-9) knots[i].outHandle = offset * (-outLen / ol);
    }
}

int EditableCurve::addKnot(const Vec3& position) {
    knots.push_back(CurveKnot{position, Vec3(0, 0, 0), Vec3(0, 0, 0), KnotMode::Auto});
    return static_cast<int>(knots.size()) - 1;
}

bool EditableCurve::removeKnot(int i) {
    if (i < 0 || i >= static_cast<int>(knots.size())) return false;
    knots.erase(knots.begin() + i);
    return true;
}

}  // namespace engine
