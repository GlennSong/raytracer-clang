#include "vehicle_mesh.h"

#include "../mesh_builder.h"
#include "../../rt_math.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace engine {

namespace {

struct Key { Real z, v; };   // z in [-0.5 rear .. +0.5 nose], v a 0..1 fraction

// Piecewise-smoothstep through ordered keys (front first): soft transitions, so
// the windshield rake and roof edges come out CURVED after normal smoothing.
Real keyEval(const std::vector<Key>& keys, Real z) {
    if (z >= keys.front().z) return keys.front().v;
    if (z <= keys.back().z) return keys.back().v;
    for (std::size_t i = 0; i + 1 < keys.size(); ++i)
        if (z <= keys[i].z && z >= keys[i + 1].z) {
            Real t = (keys[i].z - z) / (keys[i].z - keys[i + 1].z);
            t = t * t * (3.0 - 2.0 * t);
            return keys[i].v + (keys[i + 1].v - keys[i].v) * t;
        }
    return keys.back().v;
}

Real sgn(Real x) { return x < 0 ? -1.0 : 1.0; }

// The style sheet: roofline + plan-taper keys, belt line, glass bands.
struct StyleSpec {
    std::vector<Key> roof;    // roofline as a fraction of overall height
    std::vector<Key> width;   // plan-view half-width scale
    Real belt;                // glass sill (fraction of height)
    Real glassMinZ, glassMaxZ;             // greenhouse band along the body
    std::vector<Key> pillars;              // z centres of body-colour pillars
    Real boxyBelowZ = -1.0;   // rear of this: squarer sections (bed / cargo box)
};

StyleSpec styleSpec(CarShellStyle s) {
    switch (s) {
        case CarShellStyle::Hatchback:
            return { { {0.50, 0.28}, {0.42, 0.40}, {0.08, 0.46}, {-0.06, 0.92},
                       {-0.30, 0.88}, {-0.44, 0.60}, {-0.50, 0.52} },
                     { {0.50, 0.72}, {0.34, 0.94}, {0.08, 1.0}, {-0.30, 1.0}, {-0.50, 0.86} },
                     0.56, -0.44, 0.10, { {-0.12, 0.0} } };
        case CarShellStyle::SUV:
            return { { {0.50, 0.34}, {0.40, 0.50}, {0.10, 0.55}, {0.00, 0.97},
                       {-0.38, 0.95}, {-0.50, 0.62} },
                     { {0.50, 0.74}, {0.34, 0.96}, {0.05, 1.0}, {-0.34, 1.0}, {-0.50, 0.88} },
                     0.58, -0.40, 0.12, { {-0.08, 0.0}, {-0.26, 0.0} } };
        case CarShellStyle::Pickup:
            return { { {0.50, 0.30}, {0.42, 0.44}, {0.16, 0.50}, {0.06, 0.95},
                       {-0.08, 0.95}, {-0.18, 0.52}, {-0.50, 0.52} },
                     { {0.50, 0.74}, {0.34, 0.96}, {0.06, 1.0}, {-0.50, 0.98} },
                     0.58, -0.12, 0.14, {}, /*boxyBelowZ=*/-0.16 };
        case CarShellStyle::Van:
            // A panel van: glass on the cab only, tall slab sides, short nose.
            return { { {0.50, 0.40}, {0.44, 0.55}, {0.30, 0.97}, {-0.42, 0.97},
                       {-0.50, 0.80} },
                     { {0.50, 0.80}, {0.40, 0.98}, {-0.42, 0.98}, {-0.50, 0.92} },
                     0.50, 0.20, 0.50, {} };
        case CarShellStyle::BoxTruck:
            // Rounded cab + a genuinely boxy cargo box (that one may stay a box).
            return { { {0.50, 0.34}, {0.44, 0.62}, {0.36, 0.72}, {0.31, 1.0},
                       {-0.50, 1.0} },
                     { {0.50, 0.82}, {0.42, 1.0}, {-0.50, 1.0} },
                     0.48, 0.33, 0.50, {}, /*boxyBelowZ=*/0.30 };
        default:   // Sedan — the reference silhouette (three-box, raked glass)
            return { { {0.50, 0.26}, {0.44, 0.36}, {0.34, 0.40}, {0.10, 0.44},
                       {-0.04, 0.92}, {-0.24, 0.90}, {-0.36, 0.52}, {-0.50, 0.44} },
                     { {0.50, 0.70}, {0.34, 0.92}, {0.10, 1.0}, {-0.30, 1.0}, {-0.50, 0.82} },
                     0.58, -0.34, 0.08, { {-0.12, 0.0} } };
    }
}

}  // namespace

RenderMesh buildCarShell(CarShellStyle style, const Vec3& paint, const Vec3& size,
                         bool withWheels) {
    const Real W = size.x, H = size.y, L = size.z;
    const StyleSpec spec = styleSpec(style);
    const Vec3 glass(0.05, 0.07, 0.10);
    const Vec3 tyre(0.05, 0.05, 0.06);
    const Real floor01 = 0.06;             // shell bottom (fraction of height)

    RenderMesh m;
    const int S = 26;   // lofting stations, nose to tail
    const int K = 18;   // points around each cross-section ring

    // --- the lofted shell -----------------------------------------------------
    // Station rings: a superellipse (rounded-rectangle) section whose centre,
    // height, width, and squareness all follow the style keys.
    for (int si = 0; si < S; ++si) {
        Real z01 = 0.5 - static_cast<Real>(si) / (S - 1);
        Real zw = z01 * L;
        Real roof01 = keyEval(spec.roof, z01);
        Real rx = 0.5 * W * keyEval(spec.width, z01);
        Real cy01 = (floor01 + roof01) * 0.5;
        Real ry01 = (roof01 - floor01) * 0.5;
        // Squareness: soft automotive curves (n=3) except in a bed/cargo zone.
        Real n = 3.0;
        if (spec.boxyBelowZ > -0.9 && z01 < spec.boxyBelowZ) n = 7.0;
        bool inGlassBand = z01 > spec.glassMinZ && z01 < spec.glassMaxZ;
        bool nearPillar = false;
        for (const Key& p : spec.pillars)
            if (std::fabs(z01 - p.z) < 0.022) nearPillar = true;
        for (int j = 0; j < K; ++j) {
            Real th = 6.28318530718 * j / K;
            Real c = std::cos(th), s = std::sin(th);
            Real x = rx * sgn(c) * std::pow(std::fabs(c), 2.0 / n);
            Real y01 = cy01 + ry01 * sgn(s) * std::pow(std::fabs(s), 2.0 / n);
            Vertex v;
            v.position = Vec3(x, (y01 - 0.5) * H, zw);
            v.normal = Vec3(0, 1, 0);   // recomputed below
            // Greenhouse paint: dark glass between the belt line and the roof
            // SKIN (the top 5% stays painted, so the roof panel reads), only in
            // the cabin band, never on a pillar. Everything else is body paint.
            bool isGlass = inGlassBand && !nearPillar &&
                           y01 > spec.belt && y01 < roof01 - 0.05 &&
                           roof01 > spec.belt + 0.10;
            v.color = isGlass ? glass : paint;
            m.vertices.push_back(v);
        }
    }
    // Quads between adjacent rings.
    for (int si = 0; si + 1 < S; ++si)
        for (int j = 0; j < K; ++j) {
            uint32_t a = si * K + j, b = si * K + (j + 1) % K;
            uint32_t c = (si + 1) * K + (j + 1) % K, d = (si + 1) * K + j;
            m.indices.push_back(a); m.indices.push_back(b); m.indices.push_back(c);
            m.indices.push_back(a); m.indices.push_back(c); m.indices.push_back(d);
        }
    // End caps: fans to a centre point at each end ring.
    auto cap = [&](int si, Real zNudge) {
        Real z01 = 0.5 - static_cast<Real>(si) / (S - 1);
        Real roof01 = keyEval(spec.roof, z01);
        Vertex ctr;
        ctr.position = Vec3(0, ((floor01 + roof01) * 0.5 - 0.5) * H, z01 * L + zNudge);
        ctr.normal = Vec3(0, 1, 0);
        ctr.color = paint;
        uint32_t ci = static_cast<uint32_t>(m.vertices.size());
        m.vertices.push_back(ctr);
        for (int j = 0; j < K; ++j) {
            m.indices.push_back(ci);
            m.indices.push_back(si * K + j);
            m.indices.push_back(si * K + (j + 1) % K);
        }
    };
    cap(0, 0.01);
    cap(S - 1, -0.01);

    // Fix winding OUTWARD (the engine treats cross(c-a, b-a) as the front-face
    // normal): flip any triangle whose geometric normal points into the body.
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        Vec3 centroid = (a + b + c) / 3.0;
        Vec3 axis(0, centroid.y * 0.2, centroid.z);       // point on the body axis
        Vec3 outward = centroid - axis;
        if (outward.lengthSquared() < 1e-9) outward = Vec3(0, 1, 0);
        Vec3 gn = cross(c - a, b - a);
        if (dot(gn, outward) < 0) std::swap(m.indices[t + 1], m.indices[t + 2]);
    }
    MeshBuilder::recomputeNormals(m);   // shared loft verts -> SMOOTH shading

    // --- trim: lights + wheels -------------------------------------------------
    auto lamp = [&](Real zEnd, Vec3 color) {
        Real z01 = zEnd > 0 ? 0.47 : -0.47;
        Real lx = 0.5 * W * keyEval(spec.width, z01) - 0.10;
        Real ly = ((floor01 + keyEval(spec.roof, z01)) * 0.4 - 0.5) * H;
        RenderMesh lens = MeshBuilder::box(Vec3(0.30, 0.12, 0.08));
        for (Vertex& v : lens.vertices) v.color = color;
        for (Real sx : { Real(1), Real(-1) })
            MeshBuilder::appendTransformed(
                m, lens, Mat4::trs(Vec3(sx * lx, ly, zEnd), Quat(), Vec3(1, 1, 1)));
    };
    lamp(L * 0.5 - 0.02, Vec3(1.0, 0.97, 0.82));    // headlights, pale warm
    lamp(-L * 0.5 + 0.02, Vec3(0.85, 0.06, 0.05));  // taillights, red

    if (withWheels) {
        Real wr = std::max(Real(0.26), std::min(Real(0.42), H * 0.28));
        Real ww = std::max(Real(0.16), W * 0.13);
        RenderMesh wheel = MeshBuilder::cylinder(static_cast<float>(wr),
                                                 static_cast<float>(ww), 20);
        for (Vertex& v : wheel.vertices) v.color = tyre;
        Real axleY = -H * 0.5 + wr * 0.9;
        Real wx = W * 0.5 - 0.03, wz = L * 0.32;
        Quat axleX = Quat::fromAxisAngle(Vec3(0, 0, 1), PI * 0.5);
        for (Real sx : { Real(1), Real(-1) })
            for (Real sz : { Real(1), Real(-1) })
                MeshBuilder::appendTransformed(
                    m, wheel,
                    Mat4::trs(Vec3(sx * wx, axleY, sz * wz), axleX, Vec3(1, 1, 1)));
    }
    return m;
}

}  // namespace engine
