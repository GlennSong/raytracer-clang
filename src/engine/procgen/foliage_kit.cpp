#include "foliage_kit.h"

#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace engine {

namespace {

constexpr double kPi = 3.14159265358979323846;

// One SHOOT from `p0` along `dir`, `len` long: `fins` strand planes through its axis, each a strip of `segs` quads
// bending toward `down` (the spray's -Z) as it goes. `tile` is the strand it wears.
void addShoot(RenderMesh& m, const SpraySpec& s, const Vec3& p0, Vec3 dir, double len, const Vec3& down, double roll,
              int tiles, int tile) {
    constexpr int segs = 2;
    const double w = 0.5 * len * s.aspect;
    std::vector<Vec3> pts{p0}, dirs{dir};
    for (int i = 0; i < segs; ++i) {
        dir = normalize(dir + down * (s.droop / segs));
        pts.push_back(pts.back() + dir * (len / segs));
        dirs.push_back(dir);
    }
    const double ts = 1.0 / std::max(1, tiles);
    const double u0 = (tile % std::max(1, tiles)) * ts, v0 = (tile / std::max(1, tiles)) * ts;
    for (int f = 0; f < s.fins; ++f) {
        const double th = roll + f * kPi / s.fins;
        const uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (int i = 0; i <= segs; ++i) {
            const Vec3 d = dirs[i];
            const Vec3 ref = std::fabs(d.z) < 0.9 ? Vec3(0, 0, 1) : Vec3(1, 0, 0);
            const Vec3 a = normalize(cross(d, ref)), b = normalize(cross(d, a));
            const Vec3 side = a * std::cos(th) + b * std::sin(th);
            const Vec3 n = normalize(cross(d, side));
            const double v = v0 + ts * (static_cast<double>(i) / segs);
            m.vertices.emplace_back(pts[i] - side * w, n, side, static_cast<float>(u0), static_cast<float>(v));
            m.vertices.emplace_back(pts[i] + side * w, n, side, static_cast<float>(u0 + ts), static_cast<float>(v));
        }
        for (int i = 0; i < segs; ++i) {
            const uint32_t q = base + 2 * i;
            const Vec3 n = m.vertices[q].normal;
            MeshBuilder::quadFacing(m, q, q + 1, q + 3, q + 2, n);   // (one winding: the material is two-sided)
        }
    }
}

}  // namespace

std::vector<RenderMesh> buildSprays(const SpraySpec& s, int tiles, uint32_t seed) {
    std::vector<RenderMesh> out;
    const int nt = std::max(1, tiles) * std::max(1, tiles);
    for (int k = 0; k < std::max(1, s.variants); ++k) {
        std::mt19937 rng(seed * 2654435761u + static_cast<uint32_t>(k) * 97u + 1u);
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        std::uniform_int_distribution<int> pickTile(0, nt - 1);
        RenderMesh m;
        const Vec3 down(0, 0, -1);
        // the main shoot, in three strides (a long strand stretched would blur its needles)
        const int n = std::max(0, s.sideShoots);
        const double mainBend = s.droop * (0.6 + 0.8 * u01(rng));
        constexpr int strides = 3;
        std::vector<Vec3> axis{Vec3(0, 0, 0)}, axisDir{Vec3(0, 1, 0)};
        Vec3 d(0, 1, 0);
        for (int i = 0; i < strides; ++i) {
            d = normalize(d + down * (mainBend / strides) + Vec3(1, 0, 0) * ((u01(rng) - 0.5) * 0.12));
            axis.push_back(axis.back() + d * (1.0 / strides));
            axisDir.push_back(d);
        }
        for (int i = 0; i < strides; ++i)
            addShoot(m, s, axis[i], axisDir[i], 1.04 / strides, down, u01(rng) * kPi, tiles, pickTile(rng));
        auto along = [&](double t, Vec3& at, Vec3& dir) {   // a point and heading on the main shoot, t in [0, 1]
            const double x = std::clamp(t, 0.0, 1.0) * strides;
            const int i = std::min(strides - 1, static_cast<int>(x));
            at = axis[i] + (axis[i + 1] - axis[i]) * (x - i);
            dir = axisDir[i + 1];
        };
        // the side shoots: alternate sides, the lowest longest, a few lifted out of the fan
        double len = s.sideLength;
        for (int j = 0; j < n; ++j) {
            const double sgn = (j % 2 == 0) ? 1.0 : -1.0;
            const double ang = (s.sideAngle + (u01(rng) - 0.5) * 14.0) * kPi / 180.0;
            const double lift = (u01(rng) * 2.0 - 1.0) * s.lift * kPi / 180.0;
            Vec3 at, fwd;
            along((j + 0.6) / (n + 0.6), at, fwd);
            const Vec3 across = normalize(cross(fwd, Vec3(0, 0, 1))) * -sgn;
            Vec3 sd = normalize(fwd * std::cos(ang) + across * std::sin(ang));
            sd = normalize(sd * std::cos(lift) + Vec3(0, 0, 1) * std::sin(lift));
            addShoot(m, s, at, sd, len * (0.9 + 0.2 * u01(rng)), down, u01(rng) * kPi, tiles, pickTile(rng));
            len *= s.sideFalloff;
        }
        out.push_back(std::move(m));
    }
    return out;
}

// (the element atlas's square tiles: a leaf drawn up the middle, stalk at the foot, filling 6% .. 94% of the height;
// a flower centred, its petals' tips 0.4375 of the tile from its centre)
namespace {
constexpr double kLeafFoot = 0.06, kLeafSpan = 0.88, kFlowerR = 0.4375;
}

void addKitLeaf(RenderMesh& m, const Vec3& base, const Vec3& dir, const Vec3& normal, double len, double fold,
                double curl, int tiles, int tile, const Vec3& color) {
    const int n = std::max(1, tiles);
    const double ts = 1.0 / n, u0 = (tile % n) * ts, v0 = (tile / n) * ts;
    const double Q = len / kLeafSpan;   // the tile's side, in metres
    const Vec3 f = normalize(dir), nrm = normalize(normal - f * dot(normal, f)), side = normalize(cross(nrm, f));
    // three rows up the blade (foot, middle, tip), the tip bent down by `curl`; three columns, the halves raised
    const Vec3 r0 = base - f * (kLeafFoot * Q);
    const Vec3 r1 = r0 + f * (0.5 * Q);
    const Vec3 fTip = normalize(f * std::cos(curl) - nrm * std::sin(curl));
    const Vec3 r2 = r1 + fTip * (0.5 * Q);
    const Vec3 rows[3] = {r0, r1, r2};
    const Vec3 rowF[3] = {f, normalize(f + fTip), fTip};
    const double fr = fold * 3.14159265358979323846 / 180.0;
    const uint32_t b = static_cast<uint32_t>(m.vertices.size());
    for (int r = 0; r < 3; ++r) {
        const Vec3 rn = normalize(cross(side, rowF[r]) * -1.0);   // the blade's face here (toward `normal`)
        const Vec3 up = dot(rn, nrm) < 0 ? rn * -1.0 : rn;
        for (int c = 0; c < 3; ++c) {
            const double sgn = c - 1.0;   // -1 left half, 0 midrib, 1 right half
            const Vec3 across = side * (std::cos(fr) * sgn) + up * (std::sin(fr) * std::fabs(sgn));
            const Vec3 p = rows[r] + across * (0.5 * Q);
            const Vec3 vn = normalize(up - side * (sgn * std::sin(fr)));
            Vertex v(p, vn, side, static_cast<float>(u0 + ts * (0.5 + 0.5 * sgn)), static_cast<float>(v0 + ts * (0.5 * r)));
            v.color = color;
            m.vertices.push_back(v);
        }
    }
    for (int r = 0; r < 2; ++r)
        for (int c = 0; c < 2; ++c) {
            const uint32_t a = b + r * 3 + c;
            MeshBuilder::quadFacing(m, a, a + 1, a + 4, a + 3, m.vertices[a].normal + m.vertices[a + 4].normal);
        }
}

void addKitBlossom(RenderMesh& m, const Vec3& centre, const Vec3& facing, double size, double cup, double roll,
                   int tiles, int tile, const Vec3& color) {
    const int n = std::max(1, tiles);
    const double ts = 1.0 / n, uc = (tile % n + 0.5) * ts, vc = (tile / n + 0.5) * ts;
    const Vec3 a = normalize(facing);
    const Vec3 ref = std::fabs(a.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    const Vec3 e1 = normalize(cross(a, ref)), e2 = cross(a, e1);
    const double R = 0.5 * size;
    constexpr int kRim = 7;
    const uint32_t b = static_cast<uint32_t>(m.vertices.size());
    Vertex c(centre, a, e1, static_cast<float>(uc), static_cast<float>(vc));
    c.color = color;
    m.vertices.push_back(c);
    for (int k = 0; k < kRim; ++k) {
        const double th = roll + 2.0 * 3.14159265358979323846 * k / kRim;
        const Vec3 radial = e1 * std::cos(th) + e2 * std::sin(th);
        const Vec3 p = centre + radial * R + a * (cup * R);   // the rim raised: a cup opening toward `facing`
        const Vec3 nrm = normalize(a - radial * (cup * 0.8));
        const double ang = th - roll;
        Vertex v(p, nrm, e1, static_cast<float>(uc + ts * kFlowerR * std::cos(ang)), static_cast<float>(vc + ts * kFlowerR * std::sin(ang)));
        v.color = color;
        m.vertices.push_back(v);
    }
    for (int k = 0; k < kRim; ++k)
        MeshBuilder::triFacing(m, b, b + 1 + k, b + 1 + (k + 1) % kRim, a);
}

void addBlade(RenderMesh& m, const Vec3& base, const Vec3& dir, const Vec3& across, const BladeShape& b,
              const Vec3& color, const Vec3& edgeColor) {
    const int n = std::max(2, b.segments);
    const double step = b.length / n, fr = b.fold * 3.14159265358979323846 / 180.0;
    Vec3 p = base, d = normalize(dir);
    Vec3 side = normalize(across - d * dot(across, d));
    const uint32_t first = static_cast<uint32_t>(m.vertices.size());
    for (int i = 0; i <= n; ++i) {
        const double s = static_cast<double>(i) / n;
        // the width: narrower at the very base, full to `taper`, then to a point
        const double w = 0.5 * b.width * (s < b.taper ? (0.65 + 0.35 * std::min(1.0, s / 0.12)) : (1.0 - s) / (1.0 - b.taper));
        const Vec3 up = normalize(cross(side, d));   // the blade's face
        const double tw = b.twist * s;
        const Vec3 sideT = side * std::cos(tw) + up * std::sin(tw), upT = normalize(cross(sideT, d));
        for (int c = 0; c < 3; ++c) {
            const double sgn = c - 1.0;
            const Vec3 off = sideT * (std::cos(fr) * sgn * w) + upT * (std::sin(fr) * std::fabs(sgn) * w);
            const Vec3 nrm = normalize(upT - sideT * (sgn * std::sin(fr)));
            Vertex v(p + off, nrm, sideT, static_cast<float>(0.5 + 0.5 * sgn), static_cast<float>(s * b.length / 0.6));
            // (the margin's colour only half way out to the edge vertex: the gradient across the half-blade otherwise
            // dyes it all)
            v.color = (c == 1 || edgeColor.x < 0.0) ? color : (color + edgeColor) * 0.5;
            m.vertices.push_back(v);
        }
        // on up the blade: the arch grows along it (the bend toward the ground strongest near the tip)
        const Vec3 down(0, -1, 0);
        const Vec3 dn = normalize(down - d * dot(down, d));
        d = normalize(d + dn * (b.arch * 2.0 * s / n));
        side = normalize(side - d * dot(side, d));
        p = p + d * step;
    }
    for (int i = 0; i < n; ++i)
        for (int c = 0; c < 2; ++c) {
            const uint32_t a = first + static_cast<uint32_t>(i * 3 + c);
            MeshBuilder::quadFacing(m, a, a + 1, a + 4, a + 3, m.vertices[a].normal + m.vertices[a + 4].normal);
        }
}

void buildBladePlant(const BladeSpec& sp, uint32_t seed, RenderMesh& leaves, RenderMesh& crown) {
    std::mt19937 rng(seed * 2246822519u + 11u);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const double pi = 3.14159265358979323846;
    const double fanAxis = u01(rng) * pi;   // the fan's own bearing
    for (int i = 0; i < sp.count; ++i) {
        const double rank = static_cast<double>(i) / std::max(1, sp.count - 1);   // 0 the outermost .. 1 the innermost
        // the bearing: round the clump by the golden angle, or (fan) gathered to the fan's two sides
        double az = i * 2.39996323 + u01(rng) * 0.4;
        if (sp.fan > 0.0) {
            const double fanAz = fanAxis + (i % 2 ? pi : 0.0) + (u01(rng) - 0.5) * pi * (1.0 - sp.fan);
            az = az * (1.0 - sp.fan) + fanAz * sp.fan;
        }
        const Vec3 out(std::cos(az), 0.0, std::sin(az));
        const double lean = (sp.rise + sp.spread * (1.0 - rank) * (0.7 + 0.6 * u01(rng))) * pi / 180.0;
        const Vec3 dir = normalize(Vec3(0, 1, 0) * std::cos(lean) + out * std::sin(lean));
        BladeShape b;
        // longest in the middle of the clump, the outer ones shorter, each its own
        b.length = sp.length * (0.75 + 0.35 * std::sin(pi * (0.25 + 0.6 * rank))) * (1.0 + sp.lengthJitter * (u01(rng) * 2.0 - 1.0));
        b.width = sp.width * (0.85 + 0.3 * u01(rng));
        b.arch = sp.arch * (0.6 + 0.8 * u01(rng)) * (0.5 + (1.0 - rank));
        b.fold = sp.fold;
        b.twist = sp.twist * (u01(rng) * 2.0 - 1.0);
        b.taper = sp.taper;
        const Vec3 across = normalize(cross(Vec3(0, 1, 0), out)) * (sp.fan > 0.5 ? 1.0 : 1.0);
        const Vec3 base = out * (sp.crown * (0.3 + 0.7 * (1.0 - rank)) * u01(rng)) + Vec3(0, -0.02, 0);
        const Vec3 col = sp.color + (sp.colorHi - sp.color) * u01(rng);
        addBlade(leaves, base, dir, across, b, col, sp.edgeColor.x < 0.0 ? col : sp.edgeColor);
    }
    // the crown: a low dark mound, so the blades rise out of something
    const RenderMesh mound = MeshBuilder::sphere(static_cast<float>(sp.crown * 1.4), 6, 10);
    const uint32_t base = static_cast<uint32_t>(crown.vertices.size());
    for (Vertex v : mound.vertices) {
        v.position = Vec3(v.position.x, std::max(-0.02, v.position.y * 0.45), v.position.z);
        v.color = sp.color * 0.45;
        crown.vertices.push_back(v);
    }
    for (uint32_t ix : mound.indices) crown.indices.push_back(base + ix);
}

}  // namespace engine
