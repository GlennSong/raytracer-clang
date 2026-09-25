#include "stylized_tree.h"
#include "noise.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace engine {

bool stylizedShapeFromName(const std::string& n, StylizedShape& out) {
    static const std::pair<const char*, StylizedShape> kNames[] = {
        {"round", StylizedShape::Round}, {"spreading", StylizedShape::Spreading},
        {"columnar", StylizedShape::Columnar}, {"flowering", StylizedShape::Flowering},
        {"pine", StylizedShape::Pine}, {"palm", StylizedShape::Palm}};
    for (const auto& [name, s] : kNames)
        if (n == name) { out = s; return true; }
    return false;
}

namespace {

constexpr double kPi = 3.14159265358979;

struct Rng {
    uint64_t s;
    explicit Rng(uint32_t seed) : s(0xD1B54A32D192ED03ull ^ (static_cast<uint64_t>(seed) * 0x9E3779B97F4A7C15ull)) {}
    double next() {
        s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
        return static_cast<double>((s * 0x2545F4914F6CDD1Dull) >> 11) * (1.0 / 9007199254740992.0);
    }
    double in(double a, double b) { return a + (b - a) * next(); }
};

Vec3 lerp3(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }

uint32_t vert(RenderMesh& m, const Vec3& p, const Vec3& n, const Vec3& c) {
    Vertex v(p, normalize(n), Vec3(1, 0, 0), 0.0f, 0.0f);
    v.color = c;
    m.vertices.push_back(v);
    return static_cast<uint32_t>(m.vertices.size() - 1);
}
// One triangle facing `out` (the renderer's front face, as MeshBuilder::emitTri winds it).
void tri(RenderMesh& m, uint32_t a, uint32_t b, uint32_t c, const Vec3& out) {
    const Vec3& A = m.vertices[a].position; const Vec3& B = m.vertices[b].position; const Vec3& C = m.vertices[c].position;
    if (dot(cross(C - A, B - A), out) >= 0) m.indices.insert(m.indices.end(), {a, b, c});
    else m.indices.insert(m.indices.end(), {a, c, b});
}

// A tapered tube through `pts`: `sides`-gon rings, radial normals, colour by ring.
void tube(RenderMesh& m, const std::vector<Vec3>& pts, const std::vector<double>& radii, int sides,
          const std::vector<Vec3>& colours) {
    const std::size_t n = pts.size();
    if (n < 2) return;
    std::vector<uint32_t> base(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec3 d = normalize(i + 1 < n ? pts[i + 1] - pts[i] : pts[i] - pts[i - 1]);
        const Vec3 a = std::fabs(d.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        const Vec3 u = normalize(cross(d, a)), w = cross(d, u);
        base[i] = static_cast<uint32_t>(m.vertices.size());
        for (int k = 0; k < sides; ++k) {
            const double t = 2 * kPi * k / sides;
            const Vec3 r = u * std::cos(t) + w * std::sin(t);
            vert(m, pts[i] + r * radii[i], r, colours[i]);
        }
    }
    for (std::size_t i = 0; i + 1 < n; ++i)
        for (int k = 0; k < sides; ++k) {
            const uint32_t a = base[i] + k, b = base[i] + (k + 1) % sides, c = base[i + 1] + k, d = base[i + 1] + (k + 1) % sides;
            const Vec3 out = m.vertices[a].normal;
            tri(m, a, b, d, out);
            tri(m, a, d, c, out);
        }
}

// The unit icosphere, subdivided once: 42 vertices, 80 faces.
struct Ico { std::vector<Vec3> v; std::vector<std::array<int, 3>> f; };
const Ico& icosphere() {
    static const Ico ico = [] {
        Ico s;
        const double t = (1.0 + std::sqrt(5.0)) / 2.0;
        const double raw[12][3] = {{-1, t, 0}, {1, t, 0}, {-1, -t, 0}, {1, -t, 0}, {0, -1, t}, {0, 1, t},
                                   {0, -1, -t}, {0, 1, -t}, {t, 0, -1}, {t, 0, 1}, {-t, 0, -1}, {-t, 0, 1}};
        for (const auto& r : raw) s.v.push_back(normalize(Vec3(r[0], r[1], r[2])));
        std::vector<std::array<int, 3>> f = {{0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
                                             {11, 10, 2}, {10, 7, 6}, {7, 1, 8}, {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8},
                                             {3, 8, 9}, {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}};
        std::map<std::pair<int, int>, int> mid;
        auto midpoint = [&](int a, int b) {
            const auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it = mid.find(key);
            if (it != mid.end()) return it->second;
            s.v.push_back(normalize((s.v[static_cast<std::size_t>(a)] + s.v[static_cast<std::size_t>(b)]) * 0.5));
            return mid[key] = static_cast<int>(s.v.size() - 1);
        };
        for (const auto& t3 : f) {
            const int a = midpoint(t3[0], t3[1]), b = midpoint(t3[1], t3[2]), c = midpoint(t3[2], t3[0]);
            s.f.push_back({t3[0], a, c}); s.f.push_back({t3[1], b, a}); s.f.push_back({t3[2], c, b}); s.f.push_back({a, b, c});
        }
        return s;
    }();
    return ico;
}

// One puffy clump of the crown: a noise-lumped icosphere, flattened a little underneath. Its
// normals lean from the clump's own toward the crown's (centre C, radii E), so the crown
// lights as one volume; colour darkens down and in, lightens up and out.
void clump(RenderMesh& m, const Vec3& centre, double r, const Vec3& C, const Vec3& E, const Noise& noise,
           double noiseOff, const Vec3& dark, const Vec3& light) {
    const Ico& ico = icosphere();
    const uint32_t base = static_cast<uint32_t>(m.vertices.size());
    for (const Vec3& q : ico.v) {
        const double bump = 1.0 + 0.22 * noise.noise3(q.x * 1.6 + noiseOff, q.y * 1.6, q.z * 1.6 - noiseOff);
        Vec3 off = q * (r * bump);
        if (off.y < 0) off.y *= 0.72;
        const Vec3 P = centre + off;
        const Vec3 rel((P.x - C.x) / E.x, (P.y - C.y) / E.y, (P.z - C.z) / E.z);
        const Vec3 nCrown = normalize(Vec3(rel.x / E.x, rel.y / E.y, rel.z / E.z));
        const Vec3 n = normalize(lerp3(q, nCrown, 0.7));
        const double up = std::clamp((P.y - (C.y - E.y)) / (2.0 * E.y), 0.0, 1.0);
        const double outward = std::clamp(rel.length() / 1.25, 0.0, 1.0);
        const double t = std::clamp(0.1 + 0.55 * up + 0.4 * outward * outward, 0.0, 1.0);
        vert(m, P, n, lerp3(dark, light, t));
    }
    for (const auto& f : ico.f) {
        const uint32_t a = base + static_cast<uint32_t>(f[0]), b = base + static_cast<uint32_t>(f[1]), c = base + static_cast<uint32_t>(f[2]);
        const Vec3 fc = (m.vertices[a].position + m.vertices[b].position + m.vertices[c].position) * (1.0 / 3.0);
        tri(m, a, b, c, fc - centre);
    }
}

// Broadleaf crowns (round, spreading, columnar, flowering): trunk, a few limbs into the
// crown, clumps over an ellipsoid.
StylizedTree broadleaf(uint32_t seed, const StylizedTreeParams& p, Rng& rng) {
    StylizedTree t;
    const double H = p.height * rng.in(0.85, 1.15);
    double R = 0, cy = 0; Vec3 Escale; int nClumps = 11; double clumpR = 0.46;
    switch (p.shape) {
        case StylizedShape::Spreading: R = H * 0.52; Escale = Vec3(0.78, 0.30, 0.78); cy = H - R * 0.45; nClumps = 15; clumpR = 0.36; break;
        case StylizedShape::Columnar:  R = H * 0.19; Escale = Vec3(0.55, 1.75, 0.55); cy = H * 0.55; nClumps = 11; clumpR = 0.62; break;
        case StylizedShape::Flowering: R = H * 0.40; Escale = Vec3(0.66, 0.46, 0.66); cy = H - R * 0.85; nClumps = 12; clumpR = 0.44; break;
        default:                       R = H * 0.36; Escale = Vec3(0.62, 0.50, 0.62); cy = H - R * 0.95; nClumps = 11; clumpR = 0.46; break;
    }
    if (p.crownRadius > 0) R = p.crownRadius;
    if (p.clumps > 0) nClumps = p.clumps;
    const Vec3 C(rng.in(-0.1, 0.1) * R, cy, rng.in(-0.1, 0.1) * R);
    const Vec3 E(R * Escale.x, R * Escale.y, R * Escale.z);
    Vec3 dark = p.leafDark, light = p.leafLight;
    const StylizedTreeParams defaults;
    if (p.shape == StylizedShape::Flowering && (dark - defaults.leafDark).length() < 1e-9 && (light - defaults.leafLight).length() < 1e-9) {
        dark = Vec3(0.20, 0.035, 0.07); light = Vec3(0.80, 0.30, 0.42);   // cherry pink
    }
    dark = dark * p.leafTint; light = light * p.leafTint;
    const Noise noise(seed * 7u + 3u);
    // clump centres: a Fibonacci spread over the crown ellipsoid, plus a cap on top
    std::vector<std::pair<Vec3, double>> clumps;
    const double phase = rng.in(0, 2 * kPi);
    for (int i = 0; i < nClumps; ++i) {
        const double y = 1.0 - 2.0 * (i + 0.5) / nClumps;
        const double rr = std::sqrt(std::max(0.0, 1 - y * y));
        const double a = phase + i * 2.39996323 + rng.in(-0.3, 0.3);
        const Vec3 dir(rr * std::cos(a), y, rr * std::sin(a));
        const double reach = rng.in(0.7, 1.0);
        const Vec3 pos = C + Vec3(dir.x * E.x, dir.y * E.y, dir.z * E.z) * reach;
        const double r = R * clumpR * rng.in(0.85, 1.15) * (0.82 + 0.18 * (dir.y + 1) * 0.5);
        clumps.push_back({pos, r});
    }
    clumps.push_back({C + Vec3(0, E.y * 0.55, 0), R * clumpR * 1.05});
    for (std::size_t i = 0; i < clumps.size(); ++i)
        clump(t.canopy, clumps[i].first, clumps[i].second, C, E, noise, 3.7 * static_cast<double>(i), dark, light);
    // the skeleton: a trunk up into the crown, and limbs to the lower clumps
    const double tr = std::max(0.08, H * 0.028);
    const Vec3 lean(rng.in(-0.06, 0.06) * H, 0, rng.in(-0.06, 0.06) * H);
    const Vec3 top = Vec3(C.x, C.y - E.y * 0.1, C.z);
    std::vector<Vec3> pts, cols; std::vector<double> rad;
    for (int i = 0; i <= 4; ++i) {
        const double s = i / 4.0;
        pts.push_back(lerp3(Vec3(0, 0, 0), top, s) + lean * (s * (1 - s)));
        rad.push_back(tr * (1.25 - 0.6 * s) * (i == 0 ? 1.2 : 1.0));
        cols.push_back(p.barkColor * (0.75 + 0.35 * s));
    }
    tube(t.bark, pts, rad, 6, cols);
    const double fork = C.y - E.y * 0.75;
    int limbs = 0;
    for (std::size_t i = clumps.size() / 2; i < clumps.size() - 1 && limbs < 4; ++i, ++limbs) {
        const Vec3 from(pts[0].x + (top.x - pts[0].x) * (fork / std::max(top.y, 1e-6)), fork, pts[0].z + (top.z - pts[0].z) * (fork / std::max(top.y, 1e-6)));
        const Vec3 to = lerp3(from, clumps[i].first, 0.75);
        tube(t.bark, {from, lerp3(from, to, 0.5) + Vec3(0, 0.15 * R, 0), to}, {tr * 0.7, tr * 0.5, tr * 0.32}, 5,
             {p.barkColor * 0.9, p.barkColor, p.barkColor * 1.1});
    }
    return t;
}

// A conifer: a straight trunk under stacked cone tiers with jagged, drooping rims.
StylizedTree pine(uint32_t seed, const StylizedTreeParams& p, Rng& rng) {
    StylizedTree t;
    (void)seed;
    const double H = p.height * rng.in(0.85, 1.2);
    const double Rb = (p.crownRadius > 0 ? p.crownRadius : H * 0.25) * rng.in(0.9, 1.1);
    const int tiers = p.clumps > 0 ? p.clumps : 6 + static_cast<int>(rng.next() * 2.0);
    Vec3 dark = p.leafDark * 0.8 * p.leafTint, light = p.leafLight * Vec3(0.75, 0.85, 1.1) * p.leafTint;   // cooler, bluer
    const double tr = std::max(0.08, H * 0.022);
    tube(t.bark, {Vec3(0, 0, 0), Vec3(0, H * 0.5, 0), Vec3(0, H * 0.95, 0)}, {tr * 1.3, tr, tr * 0.4}, 6,
         {p.barkColor * 0.8, p.barkColor, p.barkColor});
    const int ringN = 14;
    for (int i = 0; i < tiers; ++i) {
        const double s = static_cast<double>(i) / std::max(1, tiers - 1);
        const double r = Rb * (1.0 - 0.78 * s) * rng.in(0.92, 1.08);
        const double y0 = H * (0.18 + 0.66 * s);
        const double apexY = std::min(H, y0 + r * 1.25 + H * 0.06);
        const Vec3 apex(0, apexY, 0), under(0, y0 + r * 0.22, 0);
        const double twist = rng.in(0, 2 * kPi);
        std::vector<Vec3> rim(ringN);
        for (int k = 0; k < ringN; ++k) {
            const double a = twist + 2 * kPi * k / ringN;
            const double rr = r * (k % 2 == 0 ? 1.0 : 0.72) * rng.in(0.92, 1.08);
            rim[static_cast<std::size_t>(k)] = Vec3(rr * std::cos(a), y0 - (k % 2 == 0 ? 0.22 * r : 0.05 * r), rr * std::sin(a));
        }
        // the top surface: apex to the rim, normals out and up (a soft cone)
        const uint32_t ia = vert(t.canopy, apex, Vec3(0, 1, 0), lerp3(dark, light, 0.75));
        std::vector<uint32_t> ir(ringN), iu(ringN);
        for (int k = 0; k < ringN; ++k) {
            const Vec3& q = rim[static_cast<std::size_t>(k)];
            const Vec3 n = normalize(Vec3(q.x, r * 0.9, q.z));
            ir[static_cast<std::size_t>(k)] = vert(t.canopy, q, n, lerp3(dark, light, k % 2 == 0 ? 0.95 : 0.55));
            iu[static_cast<std::size_t>(k)] = vert(t.canopy, q, Vec3(q.x * 0.3, -1, q.z * 0.3), dark * 0.7);
        }
        const uint32_t iuc = vert(t.canopy, under, Vec3(0, -1, 0), dark * 0.5);
        for (int k = 0; k < ringN; ++k) {
            const std::size_t k0 = static_cast<std::size_t>(k), k1 = static_cast<std::size_t>((k + 1) % ringN);
            const Vec3 mid = (rim[k0] + rim[k1]) * 0.5;
            tri(t.canopy, ia, ir[k0], ir[k1], Vec3(mid.x, r, mid.z));
            tri(t.canopy, iuc, iu[k0], iu[k1], Vec3(0, -1, 0));   // the underside, dark
        }
    }
    return t;
}

// A palm: a curved, ringed trunk and a crown of drooping fronds with sawtooth leaflet edges.
StylizedTree palm(uint32_t seed, const StylizedTreeParams& p, Rng& rng) {
    StylizedTree t;
    (void)seed;
    const double H = p.height * rng.in(0.85, 1.2);
    const double bend = rng.in(0.10, 0.24) * H, yaw = rng.in(0, 2 * kPi);
    const Vec3 bendDir(std::cos(yaw), 0, std::sin(yaw));
    std::vector<Vec3> pts, cols; std::vector<double> rad;
    const int segs = 12;
    for (int i = 0; i <= segs; ++i) {
        const double s = static_cast<double>(i) / segs;
        pts.push_back(Vec3(0, H * s, 0) + bendDir * (bend * s * s));
        rad.push_back(std::max(0.07, H * 0.028) * (1.25 - 0.45 * s) * (i % 2 == 0 ? 1.0 : 0.9));   // the rings
        cols.push_back(Vec3(0.20, 0.15, 0.09) * (i % 2 == 0 ? 1.0 : 0.72));
    }
    tube(t.bark, pts, rad, 6, cols);
    const Vec3 top = pts.back();
    const Vec3 dark = p.leafDark * Vec3(1.1, 1.0, 0.8) * p.leafTint, light = p.leafLight * Vec3(1.15, 1.05, 0.7) * p.leafTint;   // warmer
    const int fronds = p.clumps > 0 ? p.clumps : 11;
    const double L = H * rng.in(0.36, 0.44);
    for (int f = 0; f < fronds; ++f) {
        const double a = 2 * kPi * f / fronds + rng.in(-0.2, 0.2);
        const Vec3 out(std::cos(a), 0, std::sin(a)), side(-out.z, 0, out.x);
        const double lift = rng.in(0.25, 0.7), droop = rng.in(1.2, 1.8);
        const int n = 10;
        std::vector<uint32_t> spine, left, right;
        for (int k = 0; k <= n; ++k) {
            const double s = static_cast<double>(k) / n;
            const Vec3 c = top + out * (L * s) + Vec3(0, L * (lift * s - droop * s * s * 0.5), 0);
            const double w = L * 0.24 * std::sin(kPi * std::min(1.0, s * 1.05)) * (k % 2 == 0 ? 1.0 : 0.5);   // sawtooth
            const Vec3 col = lerp3(dark, light, 0.3 + 0.6 * s);
            const Vec3 up(0, 1, 0);
            spine.push_back(vert(t.canopy, c + Vec3(0, w * 0.12, 0), up, col));
            left.push_back(vert(t.canopy, c + side * w - Vec3(0, w * 0.3, 0), normalize(up + side * 0.35), col * 0.9));
            right.push_back(vert(t.canopy, c - side * w - Vec3(0, w * 0.3, 0), normalize(up - side * 0.35), col * 0.9));
        }
        for (int k = 0; k < n; ++k) {
            const std::size_t k0 = static_cast<std::size_t>(k), k1 = k0 + 1;
            tri(t.canopy, spine[k0], spine[k1], left[k1], Vec3(0, 1, 0));
            tri(t.canopy, spine[k0], left[k1], left[k0], Vec3(0, 1, 0));
            tri(t.canopy, spine[k0], right[k1], spine[k1], Vec3(0, 1, 0));
            tri(t.canopy, spine[k0], right[k0], right[k1], Vec3(0, 1, 0));
        }
    }
    return t;
}

}  // namespace

StylizedTree stylizedTree(uint32_t seed, const StylizedTreeParams& p) {
    Rng rng(seed);
    switch (p.shape) {
        case StylizedShape::Pine: return pine(seed, p, rng);
        case StylizedShape::Palm: return palm(seed, p, rng);
        default: return broadleaf(seed, p, rng);
    }
}

}  // namespace engine
