#include "stylized_tree.h"
#include "noise.h"
#include "proc_rng.h"
#include "../mesh_builder.h"

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
        {"pine", StylizedShape::Pine}, {"palm", StylizedShape::Palm}, {"oak", StylizedShape::Oak},
        {"maple", StylizedShape::Maple}, {"birch", StylizedShape::Birch}, {"shrub", StylizedShape::Shrub},
        {"flowering_shrub", StylizedShape::FloweringShrub}};
    for (const auto& [name, s] : kNames)
        if (n == name) { out = s; return true; }
    return false;
}

namespace {

constexpr double kPi = 3.14159265358979;

Vec3 lerp3(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }

// The tree recipes build on the shape kit (MeshBuilder): icosphere, displaceNoise, deform,
// leanNormals, colorBy, tube, vertex / triFacing.
uint32_t vert(RenderMesh& m, const Vec3& p, const Vec3& n, const Vec3& c) { return MeshBuilder::vertex(m, p, n, c); }
void tri(RenderMesh& m, uint32_t a, uint32_t b, uint32_t c, const Vec3& out) { MeshBuilder::triFacing(m, a, b, c, out); }
void tube(RenderMesh& m, const std::vector<Vec3>& pts, const std::vector<double>& radii, int sides, const std::vector<Vec3>& colours) {
    MeshBuilder::append(m, MeshBuilder::tube(pts, radii, sides, colours));
}

// One puffy clump of the crown: a noise-lumped icosphere, flattened a little underneath, its
// normals leaned toward the crown's (centre C, radii E) so the crown lights as one volume; colour
// darkens down and in, lightens up and out.
void clump(RenderMesh& m, const Vec3& centre, double r, const Vec3& C, const Vec3& E, uint32_t seed,
           const Vec3& dark, const Vec3& light) {
    RenderMesh cl = MeshBuilder::icosphere(1);
    MeshBuilder::displaceNoise(cl, Vec3(0, 0, 0), 0.22, 1.6, seed);
    MeshBuilder::deform(cl, [&](const Vec3& q) { return centre + Vec3(q.x * r, q.y * r * (q.y < 0 ? 0.72 : 1.0), q.z * r); });
    MeshBuilder::leanNormals(cl, C, E, 0.7);
    MeshBuilder::colorBy(cl, [&](const Vertex& v) {
        const Vec3& P = v.position;
        const Vec3 rel((P.x - C.x) / E.x, (P.y - C.y) / E.y, (P.z - C.z) / E.z);
        const double up = std::clamp((P.y - (C.y - E.y)) / (2.0 * E.y), 0.0, 1.0);
        const double outward = std::clamp(rel.length() / 1.25, 0.0, 1.0);
        return lerp3(dark, light, std::clamp(0.1 + 0.55 * up + 0.4 * outward * outward, 0.0, 1.0));
    });
    MeshBuilder::append(m, cl);
}

// A tiny coarse blob (the 20-face icosahedron): a blossom on a flowering shrub.
void bud(RenderMesh& m, const Vec3& centre, double r, const Vec3& colour, const Vec3& C, const Vec3& E) {
    RenderMesh b = MeshBuilder::icosphere(0);
    MeshBuilder::colorBy(b, [&](const Vertex& v) { return colour * (0.85 + 0.15 * v.position.y); });
    MeshBuilder::deform(b, [&](const Vec3& q) { return centre + q * r; });
    MeshBuilder::leanNormals(b, C, E, 0.6);
    MeshBuilder::append(m, b);
}

// Broadleaf crowns and shrubs: a trunk (or none), a few limbs into the crown, clumps over an
// ellipsoid. Each shape is a FORM -- crown proportions, trunk weight, limbs, stems, bark.
StylizedTree broadleaf(uint32_t seed, const StylizedTreeParams& p, ProcRng& rng) {
    StylizedTree t;
    const double H = p.height * rng.in(0.85, 1.15);
    struct Form {
        double R, cyFromTop;          // crown radius (x H) and the crown centre below the top (x R)
        Vec3 E;                       // crown ellipsoid (x R)
        int clumps; double clumpR;    // clump count, clump radius (x R)
        double trunk, flare;          // trunk radius (x H) and its widening at the ground
        int limbs; double limb;       // limb count and thickness (x trunk)
        double limbSpread;            // how far out (0) vs up (1) limbs head first
        double twin; int stemsMax;    // chance of a forked second stem; up to this many stems
        bool noTrunk = false, banded = false;
    };
    Form F;
    switch (p.shape) {
        case StylizedShape::Spreading: F = {0.52, 0.45, {0.78, 0.30, 0.78}, 15, 0.36, 0.040, 1.35, 4, 0.55, 0.5, 0.3, 1}; break;
        case StylizedShape::Columnar:  F = {0.19, 0.0,  {0.55, 1.75, 0.55}, 11, 0.62, 0.028, 1.2, 2, 0.5, 0.8, 0.0, 1}; break;
        case StylizedShape::Flowering: F = {0.40, 0.85, {0.66, 0.46, 0.66}, 12, 0.44, 0.034, 1.3, 4, 0.55, 0.6, 0.15, 1}; break;
        case StylizedShape::Oak:       F = {0.52, 0.80, {0.86, 0.46, 0.86}, 16, 0.40, 0.060, 1.7, 5, 0.62, 0.25, 0.1, 1}; break;
        case StylizedShape::Maple:     F = {0.42, 0.95, {0.64, 0.62, 0.64}, 14, 0.44, 0.036, 1.35, 4, 0.55, 0.6, 0.2, 1}; break;
        case StylizedShape::Birch:     F = {0.28, 1.1,  {0.46, 0.95, 0.46}, 13, 0.34, 0.017, 1.15, 3, 0.45, 0.8, 0.0, 3}; break;
        case StylizedShape::Shrub:
        case StylizedShape::FloweringShrub:
                                       F = {0.75, 0.0,  {0.95, 0.58, 0.95}, 6, 0.52, 0.0, 1.0, 0, 0.0, 0.0, 0.0, 1}; break;
        default:                       F = {0.36, 0.95, {0.62, 0.50, 0.62}, 11, 0.46, 0.036, 1.3, 4, 0.55, 0.6, 0.3, 1}; break;
    }
    const bool shrub = p.shape == StylizedShape::Shrub || p.shape == StylizedShape::FloweringShrub;
    F.noTrunk = shrub;
    F.banded = p.shape == StylizedShape::Birch;
    double R = (p.crownRadius > 0 ? p.crownRadius : H * F.R);
    // VARIETY per seed: an oval or squat or tall crown, more or fewer clumps, a crown sitting
    // higher or lower and off-centre (lopsided), sometimes a forked twin trunk.
    const Vec3 Escale(F.E.x * rng.in(0.85, 1.2), F.E.y * rng.in(0.85, 1.2), F.E.z * rng.in(0.85, 1.2));
    int nClumps = F.clumps + static_cast<int>(std::floor(rng.in(-2.0, 4.0)));
    if (p.clumps > 0) nClumps = p.clumps;
    const Vec3 E(R * Escale.x, R * Escale.y, R * Escale.z);
    double cy = p.shape == StylizedShape::Columnar ? H * 0.55 : shrub ? E.y * 0.95 : H - R * F.cyFromTop;
    cy *= rng.in(0.92, 1.06);
    const Vec3 C(rng.in(-0.2, 0.2) * R, cy, rng.in(-0.2, 0.2) * R);
    // palettes: the params' greens, shifted by shape where the caller left the defaults
    Vec3 dark = p.leafDark, light = p.leafLight, bark = p.barkColor;
    const StylizedTreeParams defaults;
    const bool defaultLeaves = (dark - defaults.leafDark).length() < 1e-9 && (light - defaults.leafLight).length() < 1e-9;
    if (defaultLeaves) {
        if (p.shape == StylizedShape::Flowering) { dark = Vec3(0.20, 0.035, 0.07); light = Vec3(0.80, 0.30, 0.42); }   // cherry pink
        if (p.shape == StylizedShape::Birch) { dark = Vec3(0.03, 0.07, 0.012); light = Vec3(0.16, 0.27, 0.04); }          // light, yellow-green
        if (p.shape == StylizedShape::Maple) { dark = Vec3(0.018, 0.055, 0.010); light = Vec3(0.10, 0.23, 0.035); }
        if (p.shape == StylizedShape::Oak) { dark = Vec3(0.012, 0.038, 0.009); light = Vec3(0.065, 0.16, 0.028); }       // deep
    }
    if ((bark - defaults.barkColor).length() < 1e-9 && p.shape == StylizedShape::Birch) bark = Vec3(0.55, 0.53, 0.48);
    dark = dark * p.leafTint; light = light * p.leafTint;
    // clump centres: a Fibonacci spread over the crown ellipsoid, plus a cap on top
    std::vector<std::pair<Vec3, double>> clumps;
    const double phase = rng.in(0, 2 * kPi);
    for (int i = 0; i < nClumps; ++i) {
        const double y = 1.0 - 2.0 * (i + 0.5) / nClumps;
        const double rr = std::sqrt(std::max(0.0, 1 - y * y));
        const double a = phase + i * 2.39996323 + rng.in(-0.3, 0.3);
        const Vec3 dir(rr * std::cos(a), y, rr * std::sin(a));
        const double reach = rng.in(0.7, 1.0);
        Vec3 pos = C + Vec3(dir.x * E.x, dir.y * E.y, dir.z * E.z) * reach;
        const double r = R * F.clumpR * rng.in(0.85, 1.15) * (0.82 + 0.18 * (dir.y + 1) * 0.5);
        if (shrub) pos.y = std::max(pos.y, r * 0.8);   // a shrub sits on the ground, not in it
        clumps.push_back({pos, r});
    }
    clumps.push_back({C + Vec3(0, E.y * 0.55, 0), R * F.clumpR * 1.05});
    for (std::size_t i = 0; i < clumps.size(); ++i)
        clump(t.canopy, clumps[i].first, clumps[i].second, C, E, seed * 7u + 3u + static_cast<uint32_t>(i) * 131u, dark, light);
    if (p.shape == StylizedShape::FloweringShrub) {   // blossom dotted over the top of the bush
        const Vec3 petal = defaultLeaves ? (rng.next() < 0.5 ? Vec3(0.85, 0.25, 0.40) : rng.next() < 0.5 ? Vec3(0.9, 0.85, 0.8) : Vec3(0.9, 0.65, 0.08))
                                         : light * 3.0;
        for (int i = 0; i < 16; ++i) {
            const double a = rng.in(0, 2 * kPi), el = rng.in(0.1, 1.2);
            const Vec3 dir(std::cos(a) * std::cos(el), std::sin(el), std::sin(a) * std::cos(el));
            const Vec3 at = C + Vec3(dir.x * E.x, dir.y * E.y, dir.z * E.z) * 1.02;
            bud(t.canopy, at, R * 0.07 * rng.in(0.8, 1.2), petal * p.leafTint, C, E);
        }
    }
    if (F.noTrunk) return t;
    // the skeleton: stems up into the crown, limbs out to the lower clumps
    const double tr = std::max(0.06, H * F.trunk) * rng.in(0.9, 1.15);
    const Vec3 top = Vec3(C.x, C.y - E.y * 0.1, C.z);
    const int stems = F.stemsMax > 1 && rng.next() < 0.45 ? 2 + static_cast<int>(rng.next() * (F.stemsMax - 1)) : 1;
    std::vector<Vec3> mainPts;
    for (int st = 0; st < stems; ++st) {
        const double a = rng.in(0, 2 * kPi);
        const Vec3 foot = stems > 1 ? Vec3(std::cos(a), 0, std::sin(a)) * (tr * 1.4) : Vec3(0, 0, 0);
        const Vec3 head = stems > 1 ? top + Vec3(std::cos(a) * E.x * 0.35, -E.y * 0.1 * st, std::sin(a) * E.z * 0.35) : top;
        const Vec3 lean(rng.in(-0.06, 0.06) * H, 0, rng.in(-0.06, 0.06) * H);
        const int rings = F.banded ? 12 : 5;
        std::vector<Vec3> pts, cols; std::vector<double> rad;
        for (int i = 0; i < rings; ++i) {
            const double s = static_cast<double>(i) / (rings - 1);
            pts.push_back(lerp3(foot, head, s) + lean * (s * (1 - s)));
            const double flare = 1.0 + (F.flare - 1.0) * std::pow(1.0 - s, 6.0);   // the root flare, then straight
            rad.push_back(tr * (stems > 1 ? 0.75 : 1.0) * (1.1 - 0.5 * s) * flare);
            Vec3 c = bark * (0.75 + 0.35 * s);
            if (F.banded && i > 0 && rng.next() < 0.35) c = Vec3(0.04, 0.035, 0.03);   // a birch's dark marks
            cols.push_back(c);
        }
        tube(t.bark, pts, rad, p.shape == StylizedShape::Oak ? 8 : 6, cols);
        if (st == 0) mainPts = pts;
    }
    if (stems == 1 && rng.next() < F.twin) {   // a second stem forking low off the first, out to the crown's side
        const double a = rng.in(0, 2 * kPi);
        const Vec3 from = lerp3(mainPts.front(), mainPts.back(), 0.28);
        const Vec3 to = C + Vec3(std::cos(a) * E.x * 0.45, -E.y * 0.15, std::sin(a) * E.z * 0.45);
        tube(t.bark, {from, lerp3(from, to, 0.5) + Vec3(0, 0.1 * R, 0), to}, {tr * 0.85, tr * 0.7, tr * 0.5}, 6,
             {bark * 0.8, bark * 0.95, bark * 1.1});
    }
    const double fork = C.y - E.y * 0.75;
    int limbs = 0;
    const double forkS = std::clamp(fork / std::max(mainPts.back().y, 1e-6), 0.0, 1.0);
    const Vec3 from = lerp3(mainPts.front(), mainPts.back(), forkS);
    for (std::size_t i = clumps.size() / 3; i < clumps.size() - 1 && limbs < F.limbs; i += 2, ++limbs) {
        const Vec3 to = lerp3(from, clumps[i].first, 0.8);
        // out first, then up (an oak's limbs run nearly level before they climb)
        const Vec3 mid = Vec3(lerp3(from, to, 0.55).x, from.y + (to.y - from.y) * F.limbSpread + 0.1 * R, lerp3(from, to, 0.55).z);
        tube(t.bark, {from, mid, to}, {tr * F.limb, tr * F.limb * 0.7, tr * F.limb * 0.42}, p.shape == StylizedShape::Oak ? 6 : 5,
             {bark * 0.9, bark, bark * 1.1});
    }
    return t;
}

// A conifer: a straight trunk under stacked cone tiers with jagged, drooping rims.
StylizedTree pine(uint32_t seed, const StylizedTreeParams& p, ProcRng& rng) {
    StylizedTree t;
    (void)seed;
    const double H = p.height * rng.in(0.85, 1.2);
    const double Rb = (p.crownRadius > 0 ? p.crownRadius : H * 0.25) * rng.in(0.8, 1.25);
    const int tiers = p.clumps > 0 ? p.clumps : 5 + static_cast<int>(rng.next() * 5.0);   // 5-9
    const double droop = rng.in(0.1, 0.35), start = rng.in(0.12, 0.3);
    Vec3 dark = p.leafDark * 0.8 * p.leafTint, light = p.leafLight * Vec3(0.75, 0.85, 1.1) * p.leafTint;   // cooler, bluer
    const double tr = std::max(0.08, H * 0.022);
    tube(t.bark, {Vec3(0, 0, 0), Vec3(0, H * 0.5, 0), Vec3(0, H * 0.95, 0)}, {tr * 1.3, tr, tr * 0.4}, 6,
         {p.barkColor * 0.8, p.barkColor, p.barkColor});
    const int ringN = 14;
    for (int i = 0; i < tiers; ++i) {
        const double s = static_cast<double>(i) / std::max(1, tiers - 1);
        const double r = Rb * (1.0 - 0.78 * s) * rng.in(0.92, 1.08);
        const double y0 = H * (start + (0.84 - start) * s);
        const double apexY = std::min(H, y0 + r * 1.25 + H * 0.06);
        const Vec3 apex(0, apexY, 0), under(0, y0 + r * 0.22, 0);
        const double twist = rng.in(0, 2 * kPi);
        std::vector<Vec3> rim(ringN);
        for (int k = 0; k < ringN; ++k) {
            const double a = twist + 2 * kPi * k / ringN;
            const double rr = r * (k % 2 == 0 ? 1.0 : 0.72) * rng.in(0.92, 1.08);
            rim[static_cast<std::size_t>(k)] = Vec3(rr * std::cos(a), y0 - (k % 2 == 0 ? droop * r : 0.05 * r), rr * std::sin(a));
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
StylizedTree palm(uint32_t seed, const StylizedTreeParams& p, ProcRng& rng) {
    StylizedTree t;
    (void)seed;
    const double H = p.height * rng.in(0.85, 1.2);
    const double bend = rng.in(0.05, 0.3) * H, yaw = rng.in(0, 2 * kPi);
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
    const int fronds = p.clumps > 0 ? p.clumps : 8 + static_cast<int>(rng.next() * 6.0);   // 8-13
    const double L = H * rng.in(0.32, 0.48);
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
    ProcRng rng(seed);
    switch (p.shape) {
        case StylizedShape::Pine: return pine(seed, p, rng);
        case StylizedShape::Palm: return palm(seed, p, rng);
        default: return broadleaf(seed, p, rng);
    }
}

}  // namespace engine
