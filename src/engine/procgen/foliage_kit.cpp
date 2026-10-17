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

}  // namespace engine
