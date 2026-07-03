#include "street_kit.h"

#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>

namespace engine {
namespace {

// Two-sided quad/tri push matching the city's winding convention (geometric
// normal opposite the shading normal; ADR engine winding). Flat markings only.
void pushTri(RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& c,
             const Vec3& nrm, const Vec3& col) {
    Vec3 geo = cross(b - a, c - a);
    uint32_t base = static_cast<uint32_t>(m.vertices.size());
    auto v = [&](const Vec3& p) { Vertex vt(p, nrm, Vec3(1, 0, 0), 0, 0); vt.color = col; return vt; };
    if (dot(geo, nrm) <= 0) {
        m.vertices.push_back(v(a)); m.vertices.push_back(v(b)); m.vertices.push_back(v(c));
    } else {
        m.vertices.push_back(v(a)); m.vertices.push_back(v(c)); m.vertices.push_back(v(b));
    }
    m.indices.push_back(base); m.indices.push_back(base + 1); m.indices.push_back(base + 2);
}
void pushQuad(RenderMesh& m, const Vec3& a, const Vec3& b, const Vec3& c,
              const Vec3& d, const Vec3& nrm, const Vec3& col) {
    pushTri(m, a, b, c, nrm, col);
    pushTri(m, a, c, d, nrm, col);
}

// Colour `m`, place it at `pos` rotated `yaw` about +Y, and merge into `out`.
void place(RenderMesh& out, RenderMesh m, const Vec3& col, const Vec3& pos, Real yaw) {
    for (Vertex& v : m.vertices) v.color = col;
    MeshBuilder::transform(m, Mat4::trs(pos, Quat::fromAxisAngle(Vec3(0, 1, 0), yaw),
                                        Vec3(1, 1, 1)));
    MeshBuilder::append(out, m);
}

}  // namespace

Poly2 roundPolygonCorners(const Poly2& poly, Real radius, int segs) {
    const std::size_t n = poly.size();
    if (n < 3 || radius <= 0) return poly;
    Poly2 out;
    out.reserve(n * (static_cast<std::size_t>(std::max(1, segs)) + 1));
    const bool ccw = isCCW(poly);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2& prev = poly[(i + n - 1) % n];
        const Vec2& cur  = poly[i];
        const Vec2& next = poly[(i + 1) % n];
        Vec2 toPrev = prev - cur, toNext = next - cur;
        Real lp = toPrev.length(), ln = toNext.length();
        if (lp < 1e-4 || ln < 1e-4) { out.push_back(cur); continue; }
        Vec2 e0 = toPrev / lp, e1 = toNext / ln;
        // Interior turn: only fillet convex corners (a CCW ring turns left, so the
        // signed edge cross is positive at a convex vertex; flip for CW rings).
        Real turn = cross(next - cur, cur - prev);   // >0 convex for CCW
        if (!ccw) turn = -turn;
        Real cosPhi = std::max(Real(-1), std::min(Real(1), dot(e0, e1)));
        Real phi = std::acos(cosPhi);                // interior angle at the corner
        if (turn <= 0 || phi < 0.4 || phi > 3.0) { out.push_back(cur); continue; }
        Real half = phi * 0.5;
        Real tanLen = radius / std::tan(half);
        Real maxLen = std::min(lp, ln) * 0.5;
        if (tanLen > maxLen) tanLen = maxLen;
        Real effR = tanLen * std::tan(half);
        Vec2 t0 = cur + e0 * tanLen, t1 = cur + e1 * tanLen;
        Vec2 bis = normalize(e0 + e1);
        Vec2 center = cur + bis * (effR / std::sin(half));
        Real a0 = std::atan2(t0.y - center.y, t0.x - center.x);
        Real a1 = std::atan2(t1.y - center.y, t1.x - center.x);
        // Sweep the short way from t0 to t1.
        Real d = a1 - a0;
        while (d >  PI) d -= 2 * PI;
        while (d < -PI) d += 2 * PI;
        int sc = std::max(1, segs);
        for (int k = 0; k <= sc; ++k) {
            Real a = a0 + d * (static_cast<Real>(k) / sc);
            out.push_back(Vec2(center.x + std::cos(a) * effR,
                               center.y + std::sin(a) * effR));
        }
    }
    return out;
}

void emitStopBar(RenderMesh& out, const Vec2& center, const Vec2& dir,
                 Real roadW, Real y, const Vec3& col) {
    Vec2 d = normalize(dir);
    if (d.lengthSquared() < 1e-8) return;
    Vec2 across = perp(d);                       // left normal of travel
    // Drive-on-the-right: approaching traffic is to the right of the centreline,
    // i.e. the -across side. Bar spans centreline -> right curb, 0.6 m deep.
    Real bar = 0.6, half = roadW * 0.5 - 0.3;
    Vec2 right = across * -1;
    Vec2 inner = center;                         // centreline
    Vec2 outer = center + right * half;          // right curb
    Vec2 back = d * (-bar * 0.5), front = d * (bar * 0.5);
    Vec3 nrm(0, 1, 0);
    auto P = [&](const Vec2& p, const Vec2& o) {
        Vec2 q = p + o; return Vec3(q.x, y + 0.03, q.y);
    };
    pushQuad(out, P(inner, back), P(outer, back), P(outer, front), P(inner, front),
             nrm, col);
}

void emitTrafficSignal(RenderMesh& out, const Vec3& base, const Vec2& faceDir,
                       const SignalParams& sp) {
    Vec2 f = normalize(faceDir);
    if (f.lengthSquared() < 1e-8) f = Vec2(0, 1);
    Real yaw = std::atan2(f.x, f.y);
    Vec3 fwd(f.x, 0, f.y);

    const Vec3 poleCol = sp.poleColor;
    const Vec3 housing = sp.housingColor;
    const Vec3 red(0.85, 0.12, 0.10), amber(0.85, 0.62, 0.10), green(0.12, 0.72, 0.28);
    const Vec3 silver(0.62, 0.64, 0.66);
    const Vec3 walkSig(0.90, 0.78, 0.30);

    const Real poleH = sp.poleHeight, armLen = sp.armLength, armY = sp.armHeight;

    // Pole + a small base collar.
    place(out, MeshBuilder::cylinder(0.12f, static_cast<float>(poleH), 8), poleCol,
          base + Vec3(0, poleH * 0.5, 0), yaw);
    place(out, MeshBuilder::cylinder(0.20f, 0.5f, 8), housing,
          base + Vec3(0, 0.25, 0), yaw);

    // Horizontal mast arm reaching SIDEWAYS over the carriageway (perpendicular to
    // the facing direction), so the head hangs out over the lane rather than
    // sticking out toward the driver. `side` is the in-plane perpendicular; with a
    // near-right-corner placement it points toward the road centre.
    Vec3 side(f.y, 0, -f.x);
    Real armYaw = std::atan2(side.x, side.z);
    Vec3 armMid = base + Vec3(0, armY, 0) + side * (armLen * 0.5);
    place(out, MeshBuilder::box(Vec3(0.12, 0.12, armLen)), poleCol, armMid, armYaw);

    // Three-lamp signal head hung from the arm end; the head faces the driver
    // (lenses on the faceDir side) even though the arm runs perpendicular.
    Vec3 headTop = base + Vec3(0, armY - 0.1, 0) + side * (armLen - 0.2);
    Vec3 headCenter = headTop + Vec3(0, -0.55, 0);
    place(out, MeshBuilder::box(Vec3(0.42, 1.30, 0.40)), housing, headCenter, yaw);
    Vec3 lensFace = fwd * 0.22;
    place(out, MeshBuilder::box(Vec3(0.26, 0.26, 0.06)), red,
          headCenter + Vec3(0, 0.42, 0) + lensFace, yaw);
    place(out, MeshBuilder::box(Vec3(0.26, 0.26, 0.06)), amber,
          headCenter + lensFace, yaw);
    place(out, MeshBuilder::box(Vec3(0.26, 0.26, 0.06)), green,
          headCenter + Vec3(0, -0.42, 0) + lensFace, yaw);

    // Pedestrian signal head + push-button box on the pole, facing across the
    // approach (toward the waiting pedestrian, i.e. back along the pole side).
    Vec3 pedFace = fwd * 0.16;
    place(out, MeshBuilder::box(Vec3(0.34, 0.40, 0.18)), housing,
          base + Vec3(0, 2.6, 0) + fwd * 0.18, yaw);
    place(out, MeshBuilder::box(Vec3(0.22, 0.26, 0.05)), walkSig,
          base + Vec3(0, 2.6, 0) + fwd * 0.18 + pedFace, yaw);
    place(out, MeshBuilder::box(Vec3(0.16, 0.22, 0.12)), housing,
          base + Vec3(0, 1.05, 0) + fwd * 0.16, yaw);
    place(out, MeshBuilder::box(Vec3(0.07, 0.07, 0.04)), silver,
          base + Vec3(0, 1.05, 0) + fwd * 0.22, yaw);
}

RenderMesh trafficSignalProto(const SignalParams& p) {
    RenderMesh out;
    emitTrafficSignal(out, Vec3(0, 0, 0), Vec2(0, 1), p);   // origin, facing +Z
    return out;
}

void emitStreetLamp(RenderMesh& out, const Vec3& base, const LampParams& p) {
    Real h = p.height;
    auto place = [&](RenderMesh m, const Vec3& col, Real y) {
        for (Vertex& v : m.vertices) v.color = col;
        MeshBuilder::transform(m, Mat4::translate(base.x, base.y + y, base.z));
        MeshBuilder::append(out, m);
    };
    place(MeshBuilder::cylinder(static_cast<float>(p.poleRadius), static_cast<float>(h), 6),
          p.poleColor, h * 0.5);
    place(MeshBuilder::box(p.headSize), p.headColor, h);
}

RenderMesh streetLamp(const LampParams& p) {
    RenderMesh out;
    emitStreetLamp(out, Vec3(0, 0, 0), p);
    return out;
}

}  // namespace engine
