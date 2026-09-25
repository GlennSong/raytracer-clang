#include "stylized_rock.h"
#include "noise.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <vector>

namespace engine {

bool rockFamilyFromName(const std::string& n, RockFamily& out) {
    if (n == "boulder") out = RockFamily::Boulder;
    else if (n == "slab") out = RockFamily::Slab;
    else if (n == "pebbles") out = RockFamily::Pebbles;
    else if (n == "outcrop") out = RockFamily::Outcrop;
    else return false;
    return true;
}
bool rockMaterialFromName(const std::string& n, RockMaterial& out) {
    if (n == "granite") out = RockMaterial::Granite;
    else if (n == "sandstone") out = RockMaterial::Sandstone;
    else if (n == "basalt") out = RockMaterial::Basalt;
    else if (n == "mossy") out = RockMaterial::Mossy;
    else return false;
    return true;
}

namespace {

struct Rng {
    uint64_t s;
    explicit Rng(uint32_t seed) : s(0xA0761D6478BD642Full ^ (static_cast<uint64_t>(seed) * 0xE7037ED1A0B428DBull)) {}
    double next() {
        s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
        return static_cast<double>((s * 0x2545F4914F6CDD1Dull) >> 11) * (1.0 / 9007199254740992.0);
    }
    double in(double a, double b) { return a + (b - a) * next(); }
};

// The unit icosphere at `subdiv` levels (0: 12 verts / 20 faces, 1: 42 / 80).
void icosphere(int subdiv, std::vector<Vec3>& v, std::vector<std::array<int, 3>>& f) {
    const double t = (1.0 + std::sqrt(5.0)) / 2.0;
    const double raw[12][3] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                               {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
    v.clear();
    for (const auto& r : raw) v.push_back(normalize(Vec3(r[0], r[1], r[2])));
    f = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
         {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
    for (int k = 0; k < subdiv; ++k) {
        std::map<std::pair<int, int>, int> mid;
        auto midpoint = [&](int a, int b) {
            const auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it = mid.find(key);
            if (it != mid.end()) return it->second;
            v.push_back(normalize((v[static_cast<std::size_t>(a)] + v[static_cast<std::size_t>(b)]) * 0.5));
            return mid[key] = static_cast<int>(v.size() - 1);
        };
        std::vector<std::array<int, 3>> nf;
        for (const auto& t3 : f) {
            const int a = midpoint(t3[0], t3[1]), b = midpoint(t3[1], t3[2]), c = midpoint(t3[2], t3[0]);
            nf.push_back({t3[0], a, c}); nf.push_back({t3[1], b, a}); nf.push_back({t3[2], c, b}); nf.push_back({a, b, c});
        }
        f = std::move(nf);
    }
}

struct Palette { Vec3 base, dark; double moss; };
Palette paletteFor(RockMaterial m) {
    switch (m) {
        // linear albedo, warm-neutral: stylized stone reads best a little lighter than real rock
        case RockMaterial::Sandstone: return {{0.30, 0.215, 0.135}, {0.17, 0.115, 0.07}, 0.15};
        case RockMaterial::Basalt:    return {{0.085, 0.080, 0.080}, {0.045, 0.043, 0.043}, 0.25};
        case RockMaterial::Mossy:     return {{0.19, 0.185, 0.165}, {0.10, 0.097, 0.088}, 0.9};
        default:                      return {{0.21, 0.200, 0.180}, {0.11, 0.105, 0.095}, 0.25};
    }
}

// One stone into `m`: a displaced icosphere shaped by `scale`, cut by `cuts` planes (facets),
// flat-ish shaded, coloured by facet with moss on top and a dark foot.
void stone(RenderMesh& m, const Vec3& at, const Vec3& scale, int subdiv, int cuts, double lumps, Rng& rng,
           const Noise& noise, const Palette& pal, double moss, double footY, bool flatTop = false) {
    std::vector<Vec3> v; std::vector<std::array<int, 3>> f;
    icosphere(subdiv, v, f);
    const double off = rng.in(0, 100);
    std::vector<std::pair<Vec3, double>> planes;   // unit normal, offset: keep dot(n, p) <= d
    for (int c = 0; c < cuts; ++c) {
        const double a = rng.in(0, 6.2831853), el = rng.in(-0.3, 1.2);
        planes.push_back({Vec3(std::cos(a) * std::cos(el), std::sin(el), std::sin(a) * std::cos(el)), rng.in(0.55, 0.85)});
    }
    if (flatTop)   // a weathered, near-level top instead of a spire
        planes.push_back({normalize(Vec3(rng.in(-0.25, 0.25), 1.0, rng.in(-0.25, 0.25))), rng.in(0.45, 0.7)});
    std::vector<Vec3> P(v.size());
    for (std::size_t i = 0; i < v.size(); ++i) {
        Vec3 q = v[i] * (1.0 + lumps * noise.noise3(v[i].x * 1.7 + off, v[i].y * 1.7, v[i].z * 1.7 - off));
        for (const auto& [n, d] : planes) {   // flatten onto each cut plane: crisp breaks
            const double k = dot(q, n) - d;
            if (k > 0) q = q - n * k;
        }
        if (q.y < -0.35) q.y = -0.35 + (q.y + 0.35) * 0.3;   // a flattened foot
        P[i] = at + Vec3(q.x * scale.x, (q.y + 0.35) * scale.y, q.z * scale.z);
    }
    const Vec3 c0 = at + Vec3(0, 0.5 * scale.y, 0);
    for (const auto& t3 : f) {
        const Vec3 A = P[static_cast<std::size_t>(t3[0])], B = P[static_cast<std::size_t>(t3[1])], C = P[static_cast<std::size_t>(t3[2])];
        Vec3 fn = cross(C - A, B - A);
        if (fn.lengthSquared() < 1e-16) continue;
        fn = normalize(fn);
        const Vec3 fc = (A + B + C) * (1.0 / 3.0);
        if (dot(fn, fc - c0) < 0) fn = fn * -1.0;
        // facet colour: base varied per facet, moss on faces that look up, a dark foot
        const double shade = 0.85 + 0.3 * rng.next();
        Vec3 col = pal.base * shade;
        const double up = std::clamp((fn.y - 0.25) / 0.4, 0.0, 1.0) * moss;
        const double mossNoise = 0.5 + 0.5 * noise.noise3(fc.x * 1.3, fc.y * 1.3, fc.z * 1.3);
        col = col + (Vec3(0.045, 0.10, 0.02) - col) * std::clamp(up * (0.7 + 0.9 * mossNoise), 0.0, 1.0);
        const double foot = std::clamp((fc.y - footY) / std::max(0.05, 0.25 * scale.y), 0.0, 1.0);
        col = pal.dark + (col - pal.dark) * (0.45 + 0.55 * foot);
        const uint32_t base = static_cast<uint32_t>(m.vertices.size());
        for (const Vec3* p3 : {&A, &B, &C}) {
            const Vec3 soft = normalize(*p3 - c0);
            Vertex vx(*p3, normalize(fn * 0.75 + soft * 0.25), Vec3(1, 0, 0), 0.0f, 0.0f);   // semi-faceted
            vx.color = col;
            m.vertices.push_back(vx);
        }
        // front face as MeshBuilder::emitTri winds it
        if (dot(cross(C - A, B - A), fn) >= 0) m.indices.insert(m.indices.end(), {base, base + 1, base + 2});
        else m.indices.insert(m.indices.end(), {base, base + 2, base + 1});
    }
}

}  // namespace

RenderMesh stylizedRock(uint32_t seed, const StylizedRockParams& p) {
    RenderMesh m;
    Rng rng(seed);
    const Noise noise(seed * 31u + 7u);
    const Palette pal = paletteFor(p.material);
    const double moss = p.moss >= 0 ? p.moss : pal.moss;
    const double s = p.size * rng.in(0.8, 1.2);
    switch (p.family) {
        case RockFamily::Slab:
            stone(m, Vec3(0, 0, 0), Vec3(s * 0.8 * rng.in(0.9, 1.3), s * 0.28 * rng.in(0.8, 1.2), s * 0.6 * rng.in(0.9, 1.2)), 1,
                  3, 0.18, rng, noise, pal, moss * 1.2, 0.0);
            break;
        case RockFamily::Pebbles: {
            const int n = 5 + static_cast<int>(rng.next() * 5);
            for (int i = 0; i < n; ++i) {
                const double a = rng.in(0, 6.2831853), r = s * 0.5 * std::sqrt(rng.next());
                const double ps = s * rng.in(0.12, 0.24);
                stone(m, Vec3(std::cos(a) * r, 0, std::sin(a) * r), Vec3(ps, ps * rng.in(0.5, 0.8), ps * rng.in(0.8, 1.1)), 0, 1,
                      0.15, rng, noise, pal, moss * 0.5, 0.0);
            }
            break;
        }
        case RockFamily::Outcrop: {
            stone(m, Vec3(0, 0, 0), Vec3(s * 0.5, s * 0.85 * rng.in(0.85, 1.2), s * 0.45), 1, 5, 0.22, rng, noise, pal, moss, 0.0, true);
            if (rng.next() < 0.7)   // a leaning second shard
                stone(m, Vec3(s * rng.in(0.25, 0.4), 0, s * rng.in(-0.2, 0.2)), Vec3(s * 0.34, s * 0.6 * rng.in(0.8, 1.2), s * 0.32), 1, 4,
                      0.2, rng, noise, pal, moss, 0.0, true);
            break;
        }
        default:
            stone(m, Vec3(0, 0, 0), Vec3(s * 0.55 * rng.in(0.9, 1.2), s * 0.42 * rng.in(0.8, 1.2), s * 0.5 * rng.in(0.9, 1.2)), 1,
                  2, 0.25, rng, noise, pal, moss, 0.0);
            break;
    }
    // THE SEAM: a skirt of small stones around a boulder's or an outcrop's foot, so where the
    // rock meets the ground reads as rubble settling against it, not a mesh cut by a plane.
    if (p.family == RockFamily::Boulder || p.family == RockFamily::Outcrop) {
        const double reach = p.family == RockFamily::Outcrop ? 0.5 : 0.62;
        const int n = 3 + static_cast<int>(rng.next() * 4);
        for (int i = 0; i < n; ++i) {
            const double a = rng.in(0, 6.2831853), r = s * reach * rng.in(0.85, 1.25);
            const double ps = s * rng.in(0.07, 0.14);
            stone(m, Vec3(std::cos(a) * r, -ps * 0.15, std::sin(a) * r), Vec3(ps, ps * rng.in(0.5, 0.8), ps * rng.in(0.8, 1.1)), 0, 1,
                  0.15, rng, noise, pal, moss * 0.4, 0.0);
        }
    }
    return m;
}

}  // namespace engine
