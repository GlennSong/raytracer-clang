#include "real_tree.h"

#include "../mesh_builder.h"
#include "proc_rng.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <vector>

namespace engine {

namespace {
constexpr double kPi = 3.14159265358979;

// ---- names -----------------------------------------------------------------------------------
const char* const kNames[] = {"spruce", "fir", "pine", "oak", "beech", "birch", "maple", "aspen", "willow", "alder", "shrub",
                              "royal_palm", "coconut_palm", "monkeypod", "poinciana", "jacaranda", "plumeria"};

double smooth01(double a, double b, double x) {
    const double t = std::clamp((x - a) / (b - a), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

Vec3 perpTo(const Vec3& d) {
    const Vec3 a = std::abs(d.y) < 0.9 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
    return normalize(cross(d, a));
}
// rotate v about unit axis k by angle a (Rodrigues)
Vec3 rotateAbout(const Vec3& v, const Vec3& k, double a) {
    const double c = std::cos(a), s = std::sin(a);
    return v * c + cross(k, v) * s + k * (dot(k, v) * (1.0 - c));
}

// the foliage atlas: 2 x 2 sprays, a card picks one
constexpr int kTiles = 2;

// One alpha-cut card: its base at `base`, running `len` along `along`, `wid` wide across `side`.
// v runs 0 (base) .. 1 (tip) along, u across the tile. Normal `n` on all four corners (bent).
void card(RenderMesh& m, const Vec3& base, const Vec3& along, const Vec3& side, double len, double wid, const Vec3& n,
          const Vec3& col, int tile) {
    const double u0 = (tile % kTiles) / double(kTiles), v0 = (tile / kTiles) / double(kTiles), ts = 1.0 / kTiles;
    const Vec3 h = side * (0.5 * wid), t = along * len;
    const Vec3 p[4] = {base - h, base + h, base + h + t, base - h + t};
    const double uv[4][2] = {{u0 + 0.004, v0 + 0.004}, {u0 + ts - 0.004, v0 + 0.004}, {u0 + ts - 0.004, v0 + ts - 0.004}, {u0 + 0.004, v0 + ts - 0.004}};
    const uint32_t b = static_cast<uint32_t>(m.vertices.size());
    const Vec3 tan = normalize(side);
    for (int k = 0; k < 4; ++k) {
        Vertex v(p[k], n, tan, static_cast<float>(uv[k][0]), static_cast<float>(uv[k][1]));
        v.color = col;
        m.vertices.push_back(v);
    }
    for (uint32_t i : {0u, 1u, 2u, 0u, 2u, 3u}) m.indices.push_back(b + i);
}

void tubeInto(RenderMesh& dst, const std::vector<Vec3>& pts, const std::vector<double>& radii, int sides, const std::vector<Vec3>& cols) {
    if (pts.size() < 2) return;
    MeshBuilder::append(dst, MeshBuilder::tube(pts, radii, sides, cols));
}

struct Species {
    double hLo, hHi;
    Vec3 bark, barkHigh, tint;
    // conifer whorls
    double crownBaseLo, crownBaseHi, crownRFrac, pitchTop, pitchBottom, sag, upturn;
    int whorlLo, whorlHi;
    // shell crowns
    double forkFrac, centreFrac, rFrac, ryFrac, elevLo, elevHi, cardSize;
    int limbs, targets, cardsPer;
    bool leader;       // a limb continues straight up
    double hang;       // cards lean down (birch)
    // IN FLOWER (the street trees): this share of the leaf cards wears the flower tile (the atlas' last) in this
    // colour. 0 for the forest species, which then draw exactly the random numbers they always did.
    Vec3 bloom;
    double bloomShare;
};

Species speciesOf(RealSpecies s) {
    Species p{};
    switch (s) {
        case RealSpecies::Spruce:
            p = {16, 26, {0.10, 0.075, 0.06}, {0.12, 0.085, 0.065}, {0.030, 0.058, 0.036},
                 0.12, 0.32, 0.19, 22, -12, 0.55, 0.35, 5, 7,
                 0, 0, 0, 0, 0, 0, 1.2, 0, 0, 0, false, 0};
            break;
        case RealSpecies::Fir:
            p = {14, 24, {0.13, 0.12, 0.11}, {0.15, 0.14, 0.12}, {0.024, 0.052, 0.030},
                 0.10, 0.28, 0.16, 25, 2, 0.25, 0.5, 4, 6,
                 0, 0, 0, 0, 0, 0, 1.1, 0, 0, 0, false, 0};
            break;
        case RealSpecies::Pine:
            p = {16, 26, {0.15, 0.10, 0.075}, {0.30, 0.17, 0.09}, {0.045, 0.068, 0.030},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.60, 0.80, 0.24, 0.16, 12, 38, 2.0, 5, 110, 8, false, 0};
            break;
        case RealSpecies::Oak:
            p = {12, 19, {0.12, 0.105, 0.09}, {0.13, 0.11, 0.095}, {0.050, 0.080, 0.022},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.32, 0.62, 0.45, 0.36, 28, 55, 1.6, 5, 150, 6, false, 0};
            break;
        case RealSpecies::Beech:
            p = {16, 25, {0.24, 0.24, 0.22}, {0.26, 0.26, 0.24}, {0.058, 0.095, 0.022},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.38, 0.64, 0.34, 0.35, 42, 66, 1.5, 4, 140, 6, true, 0};
            break;
        case RealSpecies::Maple:
            p = {12, 20, {0.14, 0.12, 0.10}, {0.15, 0.13, 0.11}, {0.055, 0.092, 0.022},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.28, 0.60, 0.44, 0.38, 30, 58, 1.5, 5, 150, 6, false, 0};
            break;
        case RealSpecies::Aspen:
            p = {14, 22, {0.52, 0.56, 0.48}, {0.58, 0.61, 0.53}, {0.075, 0.118, 0.035},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.50, 0.72, 0.18, 0.26, 55, 75, 1.0, 6, 90, 5, true, 0.1};
            break;
        case RealSpecies::Willow:
            p = {9, 15, {0.14, 0.12, 0.09}, {0.15, 0.13, 0.10}, {0.080, 0.110, 0.040},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.25, 0.56, 0.55, 0.42, 35, 60, 1.9, 6, 130, 7, false, 1.3};
            break;
        case RealSpecies::Alder:
            p = {12, 20, {0.18, 0.17, 0.15}, {0.19, 0.18, 0.16}, {0.034, 0.064, 0.020},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.35, 0.62, 0.28, 0.36, 40, 65, 1.3, 5, 120, 6, true, 0};
            break;
        case RealSpecies::Shrub:
            p = {2.2, 4.5, {0.13, 0.11, 0.09}, {0.14, 0.12, 0.10}, {0.050, 0.085, 0.025},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.03, 0.55, 0.55, 0.45, 45, 75, 0.9, 5, 50, 5, false, 0};
            break;
        // street trees. Heights and spreads from the botanical references in CREDITS.md: a royal palm 15-25 m on a
        // ~60 cm column, fronds ~3-4 m; a coconut 15-25 m (street ones 10-18), fronds 4-6 m; a monkeypod 15-25 m
        // and wider than tall; a poinciana 5-12 m, wider than tall; a jacaranda 8-15 m; a plumeria 3-8 m
        case RealSpecies::RoyalPalm:   // (palm(): bark = column, tint = fronds, bloom = crownshaft)
            p = {14, 21, {0.16, 0.16, 0.15}, {0.20, 0.20, 0.19}, {0.040, 0.075, 0.022},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0, {0.06, 0.13, 0.04}, 0};
            break;
        case RealSpecies::CoconutPalm:
            p = {10, 17, {0.10, 0.085, 0.065}, {0.15, 0.13, 0.10}, {0.060, 0.090, 0.026},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, false, 0, {0.14, 0.10, 0.03}, 0};
            break;
        case RealSpecies::Monkeypod:
            p = {12, 18, {0.11, 0.09, 0.07}, {0.13, 0.11, 0.085}, {0.042, 0.080, 0.020},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.16, 0.66, 0.78, 0.22, 10, 26, 1.7, 7, 190, 6, false, 0, {0, 0, 0}, 0};
            break;
        case RealSpecies::Poinciana:
            p = {7, 11, {0.12, 0.11, 0.095}, {0.14, 0.13, 0.11}, {0.050, 0.085, 0.022},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.24, 0.70, 0.72, 0.22, 12, 30, 1.3, 6, 150, 6, false, 0, {0.34, 0.045, 0.012}, 0.55};
            break;
        case RealSpecies::Jacaranda:
            p = {9, 14, {0.11, 0.10, 0.085}, {0.13, 0.12, 0.10}, {0.048, 0.085, 0.026},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.30, 0.62, 0.46, 0.32, 35, 58, 1.3, 5, 140, 6, false, 0, {0.13, 0.085, 0.27}, 0.7};
            break;
        case RealSpecies::Plumeria:
            p = {4, 7, {0.15, 0.15, 0.13}, {0.18, 0.18, 0.16}, {0.040, 0.078, 0.022},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.14, 0.66, 0.52, 0.34, 48, 70, 0.95, 9, 46, 4, false, 0, {0.36, 0.33, 0.20}, 0.32};
            break;
        case RealSpecies::Birch:
        default:
            p = {13, 20, {0.60, 0.58, 0.54}, {0.64, 0.62, 0.58}, {0.070, 0.110, 0.030},
                 0, 0, 0, 0, 0, 0, 0, 0, 0,
                 0.42, 0.66, 0.21, 0.32, 50, 72, 1.1, 6, 100, 5, true, 0.5};
            break;
    }
    return p;
}

// the trunk: a tapering tube with a root flare and a little wander, bark colour by height
void trunk(RenderMesh& bark, ProcRng& r, double h, double top, double r0, const Species& sp, bool birch, std::vector<Vec3>* axis) {
    std::vector<Vec3> pts;
    std::vector<double> rad;
    std::vector<Vec3> col;
    const int n = 12;
    Vec3 off(0, 0, 0);
    for (int i = 0; i <= n; ++i) {
        const double t = double(i) / n, y = top * t;
        if (i > 0) off = off + Vec3(r.in(-1, 1), 0, r.in(-1, 1)) * (0.02 * h / n);
        const double flare = 1.0 + 0.45 * std::exp(-y / 0.7);
        pts.push_back(Vec3(off.x, y, off.z));
        rad.push_back(std::max(0.03, r0 * flare * std::pow(1.0 - 0.85 * y / h, 1.1)));
        Vec3 c = sp.bark + (sp.barkHigh - sp.bark) * smooth01(0.35 * h, 0.7 * h, y);
        if (birch) c = c * (r.next() < 0.3 ? 0.35 : 1.0);   // the black bands
        col.push_back(c * r.in(0.85, 1.1));
    }
    tubeInto(bark, pts, rad, 8, col);
    if (axis) *axis = pts;
}

Vec3 axisAt(const std::vector<Vec3>& axis, double y) {
    if (axis.empty()) return Vec3(0, y, 0);
    for (std::size_t i = 1; i < axis.size(); ++i)
        if (axis[i].y >= y) {
            const double t = (y - axis[i - 1].y) / std::max(1e-6, axis[i].y - axis[i - 1].y);
            return axis[i - 1] + (axis[i] - axis[i - 1]) * t;
        }
    return Vec3(axis.back().x, y, axis.back().z);
}

// ---- conifers: whorls ----------------------------------------------------------------------------
void conifer(RealTree& t, RealSpecies s, ProcRng& r, double h) {
    const Species sp = speciesOf(s);
    const double cb = h * r.in(sp.crownBaseLo, sp.crownBaseHi);
    const double R = h * sp.crownRFrac * r.in(0.9, 1.1);
    const double r0 = h * 0.017;
    std::vector<Vec3> axis;
    trunk(t.bark, r, h, h, r0, sp, false, &axis);
    t.height = h; t.crownRadius = R; t.crownBase = cb; t.trunkRadius = r0 * 1.45;
    const Vec3 up(0, 1, 0);
    const double golden = 137.508 * kPi / 180.0;
    const double dz = r.in(0.42, 0.55) * std::sqrt(h / 20.0);
    int bi = 0;
    for (double y = cb; y < h - 0.35; y += dz * r.in(0.85, 1.15)) {
        const double tc = (y - cb) / (h - cb);                       // 0 crown base .. 1 top
        const double reach = R * std::pow(1.0 - tc, 0.85);
        const int k = sp.whorlLo + r.below(sp.whorlHi - sp.whorlLo + 1);
        const double pitch = (sp.pitchBottom + (sp.pitchTop - sp.pitchBottom) * tc) * kPi / 180.0;
        const Vec3 a = axisAt(axis, y);
        const double trunkR = std::max(0.03, r0 * std::pow(1.0 - 0.85 * y / h, 1.1));
        for (int j = 0; j < k; ++j, ++bi) {
            const double az = bi * golden + r.in(-0.3, 0.3);
            const Vec3 d(std::cos(az), 0.0, std::sin(az));
            const double L = std::max(0.35, reach * r.in(0.8, 1.15));
            // the branch: out and up at `pitch`, sagging through its middle, tips turned up
            std::vector<Vec3> pts;
            std::vector<double> rad;
            std::vector<Vec3> col;
            const int nseg = 4;
            for (int i = 0; i <= nseg; ++i) {
                const double f = double(i) / nseg, sLen = f * L;
                const double yOff = sLen * std::sin(pitch) - sp.sag * L * 0.3 * f * f + sp.upturn * L * 0.18 * std::pow(f, 4.0);
                pts.push_back(a + d * (trunkR * 0.8 + sLen * std::cos(pitch)) + up * yOff);
                rad.push_back((0.012 + 0.018 * L) * (1.0 - 0.8 * f));
                col.push_back(sp.bark * 0.9);
            }
            tubeInto(t.bark, pts, rad, 4, col);
            // foliage: overlapping sprays along the outer 85 %, each a near-flat card and a hanging one
            const double Lc = std::min(1.4, 0.55 + 0.4 * L);
            for (double sPos = 0.15 * L; sPos < L - 0.15 * Lc; sPos += Lc * 0.6) {
                const double f = sPos / L;
                const int si = std::min(nseg - 1, static_cast<int>(f * nseg));
                const double ff = f * nseg - si;
                const Vec3 p = pts[si] + (pts[si + 1] - pts[si]) * ff;
                const Vec3 along = normalize(pts[si + 1] - pts[si]);
                const Vec3 side0 = normalize(cross(up, along));
                const double len = std::min(Lc, L - sPos + 0.2 * Lc) * r.in(0.9, 1.1);
                const Vec3 rel = p - axisAt(axis, p.y);
                const double crownHere = std::max(0.3, R * std::pow(std::max(0.0, 1.0 - (p.y - cb) / (h - cb)), 0.85));
                const double rr = std::clamp(std::sqrt(rel.x * rel.x + rel.z * rel.z) / crownHere, 0.0, 1.2);
                const Vec3 n = normalize(Vec3(rel.x, 0, rel.z) / crownHere + up * 0.55);
                double ao = (0.42 + 0.58 * smooth01(0.1, 1.0, rr)) * (0.72 + 0.28 * tc);
                const Vec3 c = sp.tint * (ao * r.in(0.85, 1.12));
                const Vec3 sideA = rotateAbout(side0, along, r.in(-0.35, 0.35));
                card(t.foliage, p, rotateAbout(along, up, r.in(-0.25, 0.25)), sideA, len, len * 0.85, n, c, r.below(kTiles * kTiles));
                const Vec3 sideB = rotateAbout(side0, along, kPi * 0.5 + r.in(-0.3, 0.3));
                card(t.foliage, p - up * (0.18 * len), along, sideB, len * 0.9, len * 0.7, n, c * 0.92, r.below(kTiles * kTiles));
                // a side spray at the outer half, turned off the branch's line
                if (f > 0.4 && r.next() < 0.6) {
                    const Vec3 dirS = rotateAbout(along, up, (r.next() < 0.5 ? -1 : 1) * r.in(0.5, 0.9));
                    card(t.foliage, p, dirS, normalize(cross(up, dirS)), len * 0.7, len * 0.6, n, c * 1.04, r.below(kTiles * kTiles));
                }
            }
        }
    }
    // the leader: a few upright sprays on the top metre
    const Vec3 topP = axisAt(axis, h - 0.9);
    for (int i = 0; i < 3; ++i) {
        const double az = i * kPi / 3.0;
        card(t.foliage, topP, up, Vec3(std::cos(az), 0, std::sin(az)), 1.1, 0.5, up, sp.tint * 1.05, r.below(kTiles * kTiles));
    }
}

// ---- shell crowns (broadleaf, pine) ----------------------------------------------------------------
void shellCrown(RealTree& t, RealSpecies s, ProcRng& r, double h) {
    const Species sp = speciesOf(s);
    const double forkY = h * sp.forkFrac * r.in(0.9, 1.1);
    const Vec3 C(r.in(-0.3, 0.3), h * sp.centreFrac, r.in(-0.3, 0.3));
    const double R = h * sp.rFrac * r.in(0.88, 1.12), Ry = h * sp.ryFrac * r.in(0.9, 1.1);
    const double r0 = h * (s == RealSpecies::Birch || s == RealSpecies::Aspen ? 0.013 : (s == RealSpecies::Shrub ? 0.018 : 0.02)) *
                      (s == RealSpecies::Plumeria ? 1.6 : s == RealSpecies::Monkeypod ? 1.45 : 1.0);   // stout, for their height
    std::vector<Vec3> axis;
    // the trunk runs on up into the crown (a stem that forks once at one point reads as a candelabra)
    const bool bush = s == RealSpecies::Shrub;   // many stems from the ground, no trunk
    trunk(t.bark, r, h, bush ? 0.12 * h : (sp.leader ? C.y + 0.6 * Ry : C.y + 0.3 * Ry), r0, sp, s == RealSpecies::Birch, &axis);
    t.height = std::max(h, C.y + Ry); t.crownRadius = R; t.crownBase = C.y - Ry; t.trunkRadius = r0 * 1.45;
    const Vec3 up(0, 1, 0);
    const Vec3 F = axisAt(axis, forkY);
    // crown shell targets: a jittered Fibonacci sphere, the lower third mostly left out
    std::vector<Vec3> tg;
    const int N = sp.targets;
    for (int i = 0; i < N * 2 && static_cast<int>(tg.size()) < N; ++i) {
        const double yy = 1.0 - 2.0 * (i + 0.5) / (N * 2);
        const double rad = std::sqrt(std::max(0.0, 1.0 - yy * yy)), th = i * 2.39996 + r.in(-0.3, 0.3);
        if (yy < -0.55 && r.next() < 0.8) continue;
        const double shell = r.in(0.72, 1.0);
        Vec3 p(rad * std::cos(th) * R * shell, yy * Ry * shell, rad * std::sin(th) * R * shell);
        tg.push_back(C + p);
    }
    // main limbs
    const int M = sp.limbs;
    std::vector<Vec3> limbDir(M), limbEnd(M), limbBase(M);
    const double az0 = r.in(0, 2 * kPi);
    for (int m = 0; m < M; ++m) {
        const bool lead = sp.leader && m == 0;
        // STAGGERED up the stem, lowest limbs first, spreading widest
        const double f = M > 1 ? double(m) / (M - 1) : 0.0;
        const Vec3 Fm = lead || bush ? F + (bush ? Vec3(r.in(-0.15, 0.15), 0.0, r.in(-0.15, 0.15)) : Vec3(0, 0, 0))
                                     : axisAt(axis, forkY + (C.y - forkY) * (0.05 + 0.75 * f) + r.in(-0.4, 0.4));
        limbBase[m] = Fm;
        const double el = (lead ? r.in(78, 88) : r.in(sp.elevLo, sp.elevHi) + 18.0 * f) * kPi / 180.0;
        const double az = az0 + m * 2 * kPi / M + r.in(-0.35, 0.35);
        limbDir[m] = normalize(Vec3(std::cos(az) * std::cos(el), std::sin(el), std::sin(az) * std::cos(el)));
        // to the envelope along the direction, stopping inside it
        const Vec3 q = Fm - C;
        const Vec3 d(limbDir[m].x / R, limbDir[m].y / Ry, limbDir[m].z / R), o(q.x / R, q.y / Ry, q.z / R);
        const double A = dot(d, d), B = 2 * dot(o, d), Cc = dot(o, o) - 1.0;
        const double disc = std::max(0.0, B * B - 4 * A * Cc);
        const double tHit = (-B + std::sqrt(disc)) / (2 * A);
        limbEnd[m] = Fm + limbDir[m] * (tHit * r.in(0.55, 0.75));
        std::vector<Vec3> pts{Fm, Fm + (limbEnd[m] - Fm) * 0.5 + Vec3(r.in(-0.4, 0.4), r.in(-0.2, 0.4), r.in(-0.4, 0.4)), limbEnd[m]};
        const double lr = r0 * (lead ? 0.7 : 0.55 - 0.2 * f);
        tubeInto(t.bark, pts, {lr, lr * 0.7, lr * 0.35}, 6, {sp.barkHigh, sp.barkHigh, sp.barkHigh});
    }
    // each target hangs off its nearest limb by a twig, and carries a cluster of leaf cards
    for (const Vec3& p : tg) {
        int best = 0;
        double bd = -2;
        for (int m = 0; m < M; ++m) {
            const double c = dot(normalize(p - limbBase[m]), limbDir[m]);
            if (c > bd) { bd = c; best = m; }
        }
        const Vec3 a = limbBase[best], b = limbEnd[best];
        // attached along the limb (spread, so the twigs do not fan from one point like spokes), and
        // ending INSIDE the leaf cluster: a twig that reaches its target shows as a bare pole
        const double u = std::clamp(dot(p - a, b - a) / std::max(1e-6, dot(b - a, b - a)) + r.in(-0.25, 0.25),
                                    s == RealSpecies::Pine ? 0.55 : 0.3, 1.0);
        const Vec3 att = a + (b - a) * u;
        const Vec3 tip = att + (p - att) * 0.72;
        const Vec3 mid = att + (tip - att) * 0.5 + up * (0.12 * (tip - att).length() * (s == RealSpecies::Birch ? -1.0 : 1.0));
        const double tr = r0 * (s == RealSpecies::Pine ? 0.08 : 0.12) * (1.0 - 0.4 * u);
        tubeInto(t.bark, {att, mid, tip}, {tr, tr * 0.6, tr * 0.25}, 4, {sp.barkHigh * 0.9, sp.barkHigh * 0.9, sp.barkHigh * 0.9});
        const Vec3 e = p - C;
        const Vec3 en(e.x / R, e.y / Ry, e.z / R);
        const double rr = std::clamp(en.length(), 0.0, 1.2);
        const Vec3 n = normalize(Vec3(en.x, en.y * 0.6, en.z) + up * 0.35);
        const double low = std::clamp((p.y - (C.y - Ry)) / (2 * Ry), 0.0, 1.0);
        const Vec3 outDir = normalize(p - att);
        for (int k = 0; k < sp.cardsPer; ++k) {
            const Vec3 jitter(r.in(-1, 1), r.in(-0.7, 0.7), r.in(-1, 1));
            const Vec3 base = p + jitter * (0.35 * sp.cardSize) - outDir * (0.3 * sp.cardSize);
            Vec3 along = normalize(outDir + Vec3(r.in(-0.6, 0.6), r.in(-0.4, 0.5) - sp.hang, r.in(-0.6, 0.6)));
            const Vec3 side = rotateAbout(perpTo(along), along, r.in(0, kPi));
            const double sz = sp.cardSize * r.in(0.8, 1.2);
            const double ao = (0.40 + 0.60 * smooth01(0.35, 1.0, rr)) * (0.72 + 0.28 * low);
            if (sp.bloomShare <= 0) {
                card(t.foliage, base, along, side, sz, sz * 0.9, n, sp.tint * (ao * r.in(0.82, 1.15)), r.below(kTiles * kTiles));
                continue;
            }
            // in flower: the flower tile (the atlas' last) in the bloom colour, flowers carried on the OUTSIDE of the
            // crown (the share rises toward the shell and the top); the leaves on the other three tiles
            const double lift = r.in(0.82, 1.15);
            const bool flower = r.next() < sp.bloomShare * (0.55 + 0.6 * smooth01(0.5, 1.0, rr)) * (0.75 + 0.5 * low);
            const Vec3 tint = flower ? sp.bloom * (0.55 + 0.45 * ao) * lift : sp.tint * (ao * lift);
            const int tile = flower ? kTiles * kTiles - 1 : r.below(kTiles * kTiles - 1);
            card(t.foliage, base, along, side, flower ? sz * 0.85 : sz, (flower ? sz * 0.85 : sz) * 0.9, n, tint, tile);
        }
    }
}

// ---- palms: a column and a head of fronds ------------------------------------------------------------
// A frond is a chain of cards along a drooping arc -- each card a stretch of feather leaf (the frond tile: a
// midrib, leaflets both sides), two per stretch in a shallow V, narrowing to the tip -- so it arches as a real
// one does instead of standing out as a flat board. Young fronds stand up out of the head, old ones hang.
void palm(RealTree& t, RealSpecies s, ProcRng& r, double h) {
    const Species sp = speciesOf(s);
    const bool royal = s == RealSpecies::RoyalPalm;
    const Vec3 up(0, 1, 0);
    // the column: royal straight, swollen at the foot and a little mid-height, smooth pale grey with faint rings;
    // coconut leaning and bowing back up (it grows toward the light), slimmer, ringed dark at every frond scar
    const double topY = royal ? h * 0.80 : h * 0.90;   // where the fronds leave (royal: atop the crownshaft)
    const double shaft = royal ? h * 0.13 : 0.0;
    const double r0 = royal ? 0.30 * (h / 18.0 + 0.4) / 1.4 : 0.17 + 0.012 * h;
    const double leanAz = r.in(0, 2 * kPi), lean = royal ? r.in(0.0, 0.015) : r.in(0.16, 0.32);
    const Vec3 leanDir(std::cos(leanAz), 0, std::sin(leanAz));
    std::vector<Vec3> pts, cols;
    std::vector<double> rad;
    const int n = 18;
    const double colTop = topY - shaft;
    for (int i = 0; i <= n; ++i) {
        const double f = double(i) / n, y = colTop * f;
        // the coconut's bow: out along its lean, curving back up over the top third
        const double sway = royal ? lean * y : lean * colTop * (f - 0.35 * f * f * f);
        pts.push_back(leanDir * sway + Vec3(0, y, 0));
        double rr = r0 * (1.0 + 0.55 * std::exp(-y / 0.8));                       // the root flare
        if (royal) rr *= 1.0 + 0.14 * std::exp(-std::pow((f - 0.42) / 0.2, 2.0)) - 0.12 * f;   // the mid swelling
        else rr *= 1.0 - 0.25 * f;
        rad.push_back(rr);
        Vec3 c = sp.bark + (sp.barkHigh - sp.bark) * f;
        if (!royal && i % 2) c = c * 0.62;                                        // the frond-scar rings
        cols.push_back(c * r.in(0.92, 1.06));
    }
    tubeInto(t.bark, pts, rad, 9, cols);
    const Vec3 head = pts.back();
    Vec3 crown = head;
    if (royal) {   // the crownshaft: glossy green, a touch wider than the column top, tapering into the fronds
        std::vector<Vec3> sp2, sc;
        std::vector<double> sr;
        for (int i = 0; i <= 5; ++i) {
            const double f = i / 5.0;
            sp2.push_back(head + up * (shaft * f));
            sr.push_back(rad.back() * (1.12 - 0.35 * f * f));
            sc.push_back(sp.bloom * (0.9 + 0.2 * f));
        }
        tubeInto(t.bark, sp2, sr, 9, sc);
        crown = head + up * shaft;
    }
    // the nuts, a bunch under the coconut's head
    if (!royal) {
        const int nuts = 4 + r.below(5);
        for (int k = 0; k < nuts; ++k) {
            const double a = r.in(0, 2 * kPi), rr = 0.13;
            const Vec3 c = crown + Vec3(std::cos(a) * 0.28, r.in(-0.55, -0.25), std::sin(a) * 0.28);
            tubeInto(t.bark, {c - up * rr, c - up * (rr * 0.5), c + up * (rr * 0.5), c + up * rr}, {0.01, rr * 0.9, rr * 0.9, 0.01}, 6,
                     {sp.bloom, sp.bloom, sp.bloom * 0.8, sp.bloom * 0.8});
        }
    }
    // the fronds
    const int fronds = royal ? 13 + r.below(4) : 16 + r.below(5);
    const double golden = 137.508 * kPi / 180.0;
    double reach = 0, topMost = crown.y;
    for (int k = 0; k < fronds; ++k) {
        const double age = (k + r.next()) / fronds;                  // 0 the youngest (upright) .. 1 the oldest
        const double az = k * golden + r.in(-0.2, 0.2);
        const Vec3 d(std::cos(az), 0, std::sin(az));
        const double L = (royal ? r.in(3.4, 4.3) : r.in(4.2, 5.4)) * (0.8 + 0.2 * std::sqrt(h / 16.0));
        const double el0 = (royal ? 72.0 - 88.0 * age : 55.0 - 75.0 * age) * kPi / 180.0;   // leaving the head
        const double droop = royal ? 0.55 : 0.85;                    // how far it bends over along its length (rad)
        const int seg = 4;
        Vec3 p = crown + d * (royal ? 0.18 : 0.25) + up * r.in(-0.25, 0.15);
        const Vec3 rib0 = p;
        for (int j = 0; j < seg; ++j) {
            const double f0 = double(j) / seg, f1 = double(j + 1) / seg;
            const double el = el0 - droop * std::pow(0.5 * (f0 + f1), 1.4) * (0.6 + 0.6 * age);
            const Vec3 along = normalize(d * std::cos(el) + up * std::sin(el));
            const double len = L / seg * 1.08;
            const Vec3 side = normalize(cross(up, d));
            // leaflets spread widest past the frond's middle, closing to the tip; the coconut's hang down in a V
            const double wid = (royal ? 2.0 : 1.8) * (0.55 + 0.6 * std::sin(kPi * std::min(1.0, 0.25 + f0 * 0.9))) * (j == seg - 1 ? 0.6 : 1.0);
            const double vee = royal ? 0.55 : 0.6;                   // each half tilted down off flat
            const Vec3 nrm = normalize(d * 0.7 + up * 0.8);
            const Vec3 c = sp.tint * (r.in(0.85, 1.1) * (1.0 - 0.25 * age) * (j == 0 ? 0.85 : 1.0));
            for (int half : {-1, 1}) {
                // each half: a card from the rib outward, so the V hinges on the midrib
                const Vec3 sh = rotateAbout(side * double(half), along, -half * vee);
                card(t.foliage, p + sh * (0.25 * wid), along, sh, len, 0.5 * wid, nrm, c, r.below(kTiles * kTiles));
            }
            p = p + along * len;
            reach = std::max(reach, std::sqrt((p.x - head.x) * (p.x - head.x) + (p.z - head.z) * (p.z - head.z)));
            topMost = std::max(topMost, p.y);
        }
        // the rib's first metre, bare (the petiole), as bark
        const double rl = royal ? 0.07 : 0.06;
        tubeInto(t.bark, {crown, rib0 + normalize(d * std::cos(el0) + up * std::sin(el0)) * 0.6}, {rl, rl * 0.6}, 4,
                 {sp.bloom * 0.8 + sp.tint * 0.6, sp.tint * 1.4});
    }
    t.height = std::max(h, topMost);
    t.crownRadius = reach + std::sqrt(head.x * head.x + head.z * head.z);
    t.crownBase = crown.y - 2.5;
    t.trunkRadius = r0 * 1.3;
}

// ---- the foliage textures ---------------------------------------------------------------------------
struct Canvas {
    int n;
    std::vector<float> a, l;   // coverage, brightness (premultiplied by coverage)
    explicit Canvas(int size) : n(size), a(size * size, 0.0f), l(size * size, 0.0f) {}
    void put(int x, int y, float cov, float lum) {
        if (x < 0 || y < 0 || x >= n || y >= n || cov <= 0.0f) return;
        const std::size_t i = static_cast<std::size_t>(y) * n + x;
        const float k = std::min(cov, 1.0f);
        l[i] = l[i] * (1.0f - k) + lum * k;   // "over": what's drawn later sits on top
        a[i] = a[i] + (1.0f - a[i]) * k;
    }
    // a stroke (capsule) from p to q in [0,1]^2 of the tile at (ox, oy, s), width w
    void stroke(double px, double py, double qx, double qy, double w, float lum) {
        const double x0 = std::min(px, qx) - w, x1 = std::max(px, qx) + w, y0 = std::min(py, qy) - w, y1 = std::max(py, qy) + w;
        const double dx = qx - px, dy = qy - py, L2 = std::max(1e-12, dx * dx + dy * dy);
        for (int y = std::max(0, int(y0 * n)); y <= std::min(n - 1, int(y1 * n) + 1); ++y)
            for (int x = std::max(0, int(x0 * n)); x <= std::min(n - 1, int(x1 * n) + 1); ++x) {
                const double cx = (x + 0.5) / n, cy = (y + 0.5) / n;
                const double t = std::clamp(((cx - px) * dx + (cy - py) * dy) / L2, 0.0, 1.0);
                const double ex = cx - (px + dx * t), ey = cy - (py + dy * t);
                const double d = std::sqrt(ex * ex + ey * ey);
                const double cov = std::clamp((w * 0.5 - d) * n + 0.5, 0.0, 1.0);
                put(x, y, static_cast<float>(cov), lum);
            }
    }
    // a leaf: centre (cx, cy), pointing along angle `ang`, length `len`, width `wid`; `lobes` > 0 for oak,
    // `serr` a toothed edge; shaded lighter toward one side and darker along the midrib
    void leaf(double cx, double cy, double ang, double len, double wid, int lobes, double serr, float lum) {
        const double ca = std::cos(ang), sa = std::sin(ang), R = 0.5 * std::max(len, wid) + 0.01;
        for (int y = std::max(0, int((cy - R) * n)); y <= std::min(n - 1, int((cy + R) * n) + 1); ++y)
            for (int x = std::max(0, int((cx - R) * n)); x <= std::min(n - 1, int((cx + R) * n) + 1); ++x) {
                const double px = (x + 0.5) / n - cx, py = (y + 0.5) / n - cy;
                const double u = (px * ca + py * sa) / (0.5 * len), v = (-px * sa + py * ca) / (0.5 * wid);   // u along -1..1
                if (u < -1 || u > 1) continue;
                double half = std::pow(std::max(0.0, 1.0 - u * u), 0.55) * (1.0 - 0.25 * u);   // broad base, pointed tip
                if (lobes > 0) half *= 0.72 + 0.28 * std::abs(std::sin((u + 1) * 0.5 * kPi * lobes));
                if (serr > 0) half *= 1.0 - serr * (0.5 + 0.5 * std::sin((u + 1) * 40.0));
                const double edge = (half - std::abs(v)) * 0.5 * wid * n;
                const double cov = std::clamp(edge + 0.5, 0.0, 1.0);
                if (cov <= 0) continue;
                float sh = lum * static_cast<float>(0.88 + 0.12 * v - 0.1 * std::exp(-v * v * 60.0) + 0.06 * u);
                put(x, y, static_cast<float>(cov), sh);
            }
    }
};

void paintTile(Canvas& cv, RealSpecies s, ProcRng& r, int tx, int ty) {
    // draw in tile space then offset: a small lambda maps tile [0,1]^2 to the canvas
    const double s0 = 1.0 / kTiles, ox = tx * s0, oy = ty * s0;
    auto X = [&](double u) { return ox + u * s0; };
    auto Y = [&](double v) { return oy + v * s0; };
    const double pw = 1.0 / cv.n;   // a pixel, in canvas units
    const bool conifer = s == RealSpecies::Spruce || s == RealSpecies::Fir;
    if (conifer || s == RealSpecies::Pine) {
        // the twig runs up the tile's middle (v 0 at the base); side twigs off it; needles all over
        std::vector<std::array<double, 4>> twigs;
        const double bend = r.in(-0.06, 0.06);
        twigs.push_back({0.5, 0.02, 0.5 + bend, 0.97});
        if (s == RealSpecies::Pine)   // a few long side shoots, each ending in a tuft
            for (double v = 0.25; v < 0.8; v += r.in(0.16, 0.22))
                for (int sgn : {-1, 1}) {
                    const double len = r.in(0.22, 0.32), ang = r.in(30.0, 48.0) * kPi / 180.0, bx = 0.5 + bend * v;
                    twigs.push_back({bx, v, bx + sgn * len * std::sin(ang), v + len * std::cos(ang)});
                }
        else
            for (double v = 0.12; v < 0.85; v += r.in(0.09, 0.13))
                for (int sgn : {-1, 1}) {
                    const double len = (0.34 - 0.28 * v) * r.in(0.8, 1.15);
                    const double ang = (s == RealSpecies::Fir ? 70.0 : 55.0) * kPi / 180.0;
                    const double bx = 0.5 + bend * v;
                    twigs.push_back({bx, v, bx + sgn * len * std::sin(ang), v + len * std::cos(ang)});
                }
        for (const auto& tw : twigs) cv.stroke(X(tw[0]), Y(tw[1]), X(tw[2]), Y(tw[3]), 2.2 * pw, 0.55f);
        for (const auto& tw : twigs) {
            const double dx = tw[2] - tw[0], dy = tw[3] - tw[1], L = std::sqrt(dx * dx + dy * dy);
            const double step = s == RealSpecies::Pine ? 0.012 : 0.009;
            for (double f = 0.0; f <= 1.0; f += step / std::max(L, 1e-3)) {
                if (s == RealSpecies::Pine && f < 0.45) continue;   // tufts at the ends
                const double px = tw[0] + dx * f, py = tw[1] + dy * f;
                const double base = std::atan2(dy, dx);
                const int per = s == RealSpecies::Pine ? 4 : 3;
                for (int k = 0; k < per; ++k) {
                    const double spread = s == RealSpecies::Pine ? r.in(-0.55, 0.55) : (s == RealSpecies::Fir ? (k % 2 ? 1 : -1) * r.in(1.2, 1.5) : r.in(-1.3, 1.3));
                    const double a = base + spread;
                    const double nl = s == RealSpecies::Pine ? r.in(0.11, 0.17) : r.in(0.035, 0.055);
                    const float lum = static_cast<float>(r.in(0.72, 1.0) * (0.85 + 0.25 * f));   // new growth lighter
                    cv.stroke(X(px), Y(py), X(px + nl * std::cos(a)), Y(py + nl * std::sin(a)), 1.4 * pw, lum);
                }
            }
        }
        return;
    }
    if (realSpeciesIsPalm(s)) {
        // a stretch of FEATHER FROND: the midrib up the middle (v 0 at the frond's base end), narrow leaflets off both
        // sides angled toward the tip -- the royal's stiffer and more even, the coconut's longer and looser
        const bool royal = s == RealSpecies::RoyalPalm;
        cv.stroke(X(0.5), Y(0.0), X(0.5), Y(1.0), 3.0 * pw, 0.6f);
        for (double v = 0.0; v < 1.02; v += royal ? 0.034 : 0.04) {
            for (int sgn : {-1, 1}) {
                const double ang = (sgn > 0 ? 0.0 : kPi) + sgn * r.in(0.5, 0.75) * (sgn > 0 ? 1.0 : -1.0);
                const double len = (royal ? r.in(0.40, 0.48) : r.in(0.44, 0.52));
                const double bx = 0.5, by = v + r.in(-0.008, 0.008);
                // a leaflet: a long narrow leaf from the rib, its tip toward the frond's tip
                const double dx = std::cos(ang) * len, dy = std::abs(std::sin(ang)) * len;
                cv.leaf(X(bx + 0.5 * dx), Y(by + 0.5 * dy), std::atan2(dy, dx), len * s0, (royal ? 0.07 : 0.06) * s0, 0, 0.0,
                        static_cast<float>(r.in(0.72, 1.0)));
            }
        }
        return;
    }
    const bool flowering = s == RealSpecies::Poinciana || s == RealSpecies::Jacaranda || s == RealSpecies::Plumeria;
    if (flowering && tx == kTiles - 1 && ty == kTiles - 1) {
        // the FLOWER tile (the atlas' last; a bloom card wears it in its species' colour): clusters of five-petalled
        // flowers -- the poinciana's broad and crowded, the jacaranda's small trumpets in loose panicles, the
        // plumeria's pinwheels in a few heads
        const int heads = s == RealSpecies::Plumeria ? 3 : s == RealSpecies::Jacaranda ? 7 : 6;
        const double fr = s == RealSpecies::Plumeria ? 0.075 : s == RealSpecies::Jacaranda ? 0.035 : 0.05;
        const int per = s == RealSpecies::Plumeria ? 6 : s == RealSpecies::Jacaranda ? 14 : 9;
        for (int hd = 0; hd < heads; ++hd) {
            const double hx = r.in(0.22, 0.78), hy = r.in(0.25, 0.8);
            cv.stroke(X(0.5), Y(0.02), X(hx), Y(hy), 1.4 * pw, 0.45f);
            for (int k = 0; k < per; ++k) {
                const double cx = hx + r.in(-1, 1) * 0.13, cy = hy + r.in(-1, 1) * 0.11;
                const double rot = r.in(0, 2 * kPi);
                const float lum = static_cast<float>(r.in(0.78, 1.0));
                for (int pt = 0; pt < 5; ++pt) {
                    const double a = rot + pt * 2.0 * kPi / 5.0 + (s == RealSpecies::Plumeria ? 0.35 : 0.0);
                    cv.leaf(X(cx + 0.5 * fr * std::cos(a)), Y(cy + 0.5 * fr * std::sin(a)), a, fr * s0,
                            fr * (s == RealSpecies::Jacaranda ? 0.75 : 0.8) * s0, 0, 0.0, lum * static_cast<float>(0.9 + 0.1 * pt / 4.0));
                }
            }
        }
        return;
    }
    // broadleaf: a twig forking into three or four, leaves along them and clustered at the ends
    const int forks = 3 + r.below(2);
    cv.stroke(X(0.5), Y(0.02), X(0.5), Y(0.35), 2.5 * pw, 0.5f);
    struct Leaf { double x, y, ang, len, wid; float lum; };
    std::vector<Leaf> leaves;
    double sizeK = s == RealSpecies::Birch || s == RealSpecies::Aspen ? 0.75 : (s == RealSpecies::Willow ? 1.25 : (s == RealSpecies::Oak || s == RealSpecies::Maple ? 1.0 : 0.95));
    // the rain tree's and the flame tree's fine pinnate leaflets; the frangipani's long paddles
    if (s == RealSpecies::Monkeypod || s == RealSpecies::Poinciana || s == RealSpecies::Jacaranda) sizeK = 0.5;
    if (s == RealSpecies::Plumeria) sizeK = 1.7;
    const bool hanging = s == RealSpecies::Birch || s == RealSpecies::Willow;
    for (int f = 0; f < forks; ++f) {
        const double ang = kPi * 0.5 + (f - (forks - 1) * 0.5) * r.in(0.35, 0.55);
        const double len = r.in(0.4, 0.58);
        const double ex = 0.5 + len * std::cos(ang), ey = 0.35 + len * std::sin(ang) * (hanging ? 0.8 : 1.0);
        cv.stroke(X(0.5), Y(0.35), X(ex), Y(ey), 1.6 * pw, 0.5f);
        const bool fine = s == RealSpecies::Monkeypod || s == RealSpecies::Poinciana || s == RealSpecies::Jacaranda;
        const int nl = static_cast<int>((s == RealSpecies::Willow ? 11 : (hanging ? 7 : 6)) + r.below(4)) * (fine ? 3 : 1);
        for (int k = 0; k < nl; ++k) {
            const double f2 = 0.2 + 0.8 * (k + r.next()) / nl;
            const double px = 0.5 + (ex - 0.5) * f2, py = 0.35 + (ey - 0.35) * f2;
            const double la = ang + (k % 2 ? 1 : -1) * r.in(0.5, 1.1) + (hanging ? -0.4 : 0.0);
            const double L = r.in(0.13, 0.19) * sizeK;
            double ratio = 0.58;   // width / length of the leaf
            switch (s) {
                case RealSpecies::Oak: ratio = 0.55; break;
                case RealSpecies::Birch: ratio = 0.7; break;
                case RealSpecies::Maple: ratio = 0.95; break;
                case RealSpecies::Aspen: ratio = 0.9; break;
                case RealSpecies::Willow: ratio = 0.16; break;
                case RealSpecies::Alder: ratio = 0.8; break;
                case RealSpecies::Shrub: ratio = 0.78; break;
                case RealSpecies::Monkeypod: case RealSpecies::Poinciana: case RealSpecies::Jacaranda: ratio = 0.5; break;
                case RealSpecies::Plumeria: ratio = 0.32; break;
                default: break;
            }
            leaves.push_back({px + 0.5 * L * std::cos(la), py + 0.5 * L * std::sin(la), la, L, L * ratio,
                              static_cast<float>(r.in(0.7, 1.0))});
        }
    }
    // back to front by brightness: darker leaves first (they read as the ones behind)
    std::sort(leaves.begin(), leaves.end(), [](const Leaf& a, const Leaf& b) { return a.lum < b.lum; });
    for (const Leaf& L : leaves)
        cv.leaf(X(L.x), Y(L.y), L.ang, L.len * s0, L.wid * s0, s == RealSpecies::Oak ? 3 : (s == RealSpecies::Maple ? 2 : 0),
                s == RealSpecies::Beech ? 0.05 : (s == RealSpecies::Birch || s == RealSpecies::Alder || s == RealSpecies::Shrub ? 0.08 : 0.0), L.lum);
}

// fill the RGB under transparent texels from covered neighbours (mips then average the right colour)
void dilateRGB(std::vector<uint8_t>& px, int w, int h, int stride, int passes) {
    for (int p = 0; p < passes; ++p) {
        std::vector<uint8_t> src(px);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                uint8_t* d = &px[static_cast<std::size_t>(y) * stride + x * 4];
                if (d[3] >= 128 || (p > 0 && src[static_cast<std::size_t>(y) * stride + x * 4 + 3] == 1)) continue;
                int sr = 0, sg = 0, sb = 0, c = 0;
                for (int k = 0; k < 4; ++k) {
                    const int nx = x + (k == 0) - (k == 1), ny = y + (k == 2) - (k == 3);
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    const uint8_t* s = &src[static_cast<std::size_t>(ny) * stride + nx * 4];
                    if (s[3] >= 128 || s[3] == 1) { sr += s[0]; sg += s[1]; sb += s[2]; ++c; }
                }
                if (c) { d[0] = uint8_t(sr / c); d[1] = uint8_t(sg / c); d[2] = uint8_t(sb / c); if (d[3] < 1) d[3] = 1; }
            }
    }
}
}  // namespace

bool realSpeciesFromName(const std::string& name, RealSpecies& out) {
    for (int i = 0; i < static_cast<int>(RealSpecies::Count); ++i)
        if (name == kNames[i]) { out = static_cast<RealSpecies>(i); return true; }
    return false;
}
const char* realSpeciesName(RealSpecies s) { return kNames[std::min<int>(static_cast<int>(s), static_cast<int>(RealSpecies::Count) - 1)]; }

RealTree realTree(RealSpecies species, uint32_t seed, double height) {
    RealTree t;
    ProcRng r(seed, 0x7EE5 + static_cast<uint64_t>(species));
    const Species sp = speciesOf(species);
    const double h = height > 0 ? height : r.in(sp.hLo, sp.hHi);
    if (species == RealSpecies::Spruce || species == RealSpecies::Fir) conifer(t, species, r, h);
    else if (realSpeciesIsPalm(species)) palm(t, species, r, h);
    else shellCrown(t, species, r, h);
    // the street trees measure what they grew (a frangipani's tip rosettes and a rain tree's outer sprays stand past
    // the shell's nominal size): the impostor frames the whole tree. (The forest species keep their numbers as cached.)
    if (species >= RealSpecies::RoyalPalm)
        for (const Vertex& v : t.foliage.vertices) {
            t.height = std::max(t.height, static_cast<double>(v.position.y));
            t.crownRadius = std::max(t.crownRadius, std::sqrt(static_cast<double>(v.position.x * v.position.x + v.position.z * v.position.z)));
        }
    t.foliage.materialIndex = 0;
    t.bark.materialIndex = 0;
    return t;
}

TextureData realFoliageTexture(RealSpecies species, int size, uint32_t seed) {
    const int ss = size * 2;   // painted at twice the size, box-filtered down
    Canvas cv(ss);
    ProcRng r(seed, 0xF011A6E + static_cast<uint64_t>(species));
    for (int ty = 0; ty < kTiles; ++ty)
        for (int tx = 0; tx < kTiles; ++tx) paintTile(cv, species, r, tx, ty);
    TextureData td;
    td.width = td.height = size;
    td.channels = 4;
    td.pixels.assign(static_cast<std::size_t>(size) * size * 4, 0);
    for (int y = 0; y < size; ++y)
        for (int x = 0; x < size; ++x) {
            float a = 0, l = 0;
            for (int k = 0; k < 4; ++k) {
                const std::size_t i = static_cast<std::size_t>(2 * y + k / 2) * ss + 2 * x + k % 2;
                a += cv.a[i];
                l += cv.l[i] * cv.a[i];
            }
            const float cov = a / 4.0f, lum = a > 1e-4f ? l / a : 0.0f;
            // image row 0 is v = 0 (the twig's base): the card's v runs base -> tip
            uint8_t* d = &td.pixels[(static_cast<std::size_t>(y) * size + x) * 4];
            const uint8_t g = static_cast<uint8_t>(std::clamp(lum * 255.0f, 0.0f, 255.0f));
            d[0] = static_cast<uint8_t>(g * 0.97f); d[1] = g; d[2] = static_cast<uint8_t>(g * 0.9f);
            d[3] = static_cast<uint8_t>(std::clamp(cov * 255.0f, 0.0f, 255.0f));
        }
    dilateRGB(td.pixels, size, size, size * 4, 8);
    for (std::size_t i = 3; i < td.pixels.size(); i += 4) if (td.pixels[i] == 1) td.pixels[i] = 0;
    return td;
}

void renderImpostor(const RealTree& tree, const TextureData& fol, bool top, int w, int h, double colourScale,
                    uint8_t* out, int stride, uint8_t* normalOut) {
    const int S = 2, W = w * S, H = h * S;   // supersampled
    std::vector<float> depth(static_cast<std::size_t>(W) * H, -1e30f);
    std::vector<float> rgb(static_cast<std::size_t>(W) * H * 3, 0.0f);
    std::vector<uint8_t> cov(static_cast<std::size_t>(W) * H, 0);
    std::vector<float> nrm(normalOut ? static_cast<std::size_t>(W) * H * 3 : 0, 0.0f);
    // the tree's normal in the card's tangent frame (see real_tree.h): side card T = +x, B = +y, N = +z;
    // top card T = +x, N = +y, B = N x T = -z
    auto toCard = [top](const Vec3& n) {
        const double tx = n.x, ty = top ? -n.z : n.y, along = top ? n.y : n.z;
        // tilted at most ~50 deg off the card: steeper, the one-pixel rims of a far crown catch the sun
        // at full strength and read as glitter, not volume
        return Vec3(tx, ty, 0.85 + 0.5 * std::fabs(along));
    };
    const double halfW = tree.crownRadius * 1.18 + 0.5;
    const double Ht = tree.height * 1.03;
    // world -> image (x right, y down), and depth (larger = nearer the viewer)
    auto proj = [&](const Vec3& p, double& ix, double& iy, double& dz) {
        if (!top) { ix = (p.x + halfW) / (2 * halfW) * W; iy = (1.0 - p.y / Ht) * H; dz = p.z; }
        else { ix = (p.x + halfW) / (2 * halfW) * W; iy = (p.z + halfW) / (2 * halfW) * H; dz = p.y; }
    };
    auto sampleFol = [&](double u, double v, float& a, Vec3& c) {
        const int x = std::clamp(static_cast<int>(u * fol.width), 0, fol.width - 1), y = std::clamp(static_cast<int>(v * fol.height), 0, fol.height - 1);
        const uint8_t* p = &fol.pixels[(static_cast<std::size_t>(y) * fol.width + x) * 4];
        a = p[3] / 255.0f;
        c = Vec3(p[0], p[1], p[2]) / 255.0;
    };
    auto raster = [&](const RenderMesh& m, bool leaves) {
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const Vertex* v[3] = {&m.vertices[m.indices[t]], &m.vertices[m.indices[t + 1]], &m.vertices[m.indices[t + 2]]};
            double x[3], y[3], z[3];
            for (int k = 0; k < 3; ++k) proj(v[k]->position, x[k], y[k], z[k]);
            const double area = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0]);
            if (std::abs(area) < 1e-9) continue;
            const int x0 = std::max(0, static_cast<int>(std::floor(std::min({x[0], x[1], x[2]})))), x1 = std::min(W - 1, static_cast<int>(std::ceil(std::max({x[0], x[1], x[2]}))));
            const int y0 = std::max(0, static_cast<int>(std::floor(std::min({y[0], y[1], y[2]})))), y1 = std::min(H - 1, static_cast<int>(std::ceil(std::max({y[0], y[1], y[2]}))));
            for (int py = y0; py <= y1; ++py)
                for (int px = x0; px <= x1; ++px) {
                    const double cx = px + 0.5, cy = py + 0.5;
                    const double w0 = ((x[1] - cx) * (y[2] - cy) - (x[2] - cx) * (y[1] - cy)) / area;
                    const double w1 = ((x[2] - cx) * (y[0] - cy) - (x[0] - cx) * (y[2] - cy)) / area;
                    const double w2 = 1.0 - w0 - w1;
                    if (w0 < 0 || w1 < 0 || w2 < 0) continue;
                    const double dz = w0 * z[0] + w1 * z[1] + w2 * z[2];
                    const std::size_t i = static_cast<std::size_t>(py) * W + px;
                    if (dz <= depth[i]) continue;
                    Vec3 col = v[0]->color * w0 + v[1]->color * w1 + v[2]->color * w2;
                    if (leaves) {
                        const double u = w0 * v[0]->u + w1 * v[1]->u + w2 * v[2]->u, vv = w0 * v[0]->v + w1 * v[1]->v + w2 * v[2]->v;
                        float a;
                        Vec3 tc;
                        sampleFol(u, vv, a, tc);
                        if (a < 0.5f) continue;
                        col = col * tc;
                    }
                    depth[i] = static_cast<float>(dz);
                    cov[i] = 1;
                    if (normalOut) {
                        const Vec3 cn = toCard(normalize(v[0]->normal * w0 + v[1]->normal * w1 + v[2]->normal * w2));
                        nrm[i * 3] = static_cast<float>(cn.x); nrm[i * 3 + 1] = static_cast<float>(cn.y); nrm[i * 3 + 2] = static_cast<float>(cn.z);
                    }
                    rgb[i * 3] = static_cast<float>(col.x); rgb[i * 3 + 1] = static_cast<float>(col.y); rgb[i * 3 + 2] = static_cast<float>(col.z);
                }
        }
    };
    raster(tree.bark, false);
    raster(tree.foliage, true);
    std::vector<uint8_t> img(static_cast<std::size_t>(w) * h * 4, 0);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            double r = 0, g = 0, b = 0;
            int c = 0;
            for (int k = 0; k < S * S; ++k) {
                const std::size_t i = static_cast<std::size_t>(y * S + k / S) * W + x * S + k % S;
                if (!cov[i]) continue;
                r += rgb[i * 3]; g += rgb[i * 3 + 1]; b += rgb[i * 3 + 2]; ++c;
            }
            uint8_t* d = &img[(static_cast<std::size_t>(y) * w + x) * 4];
            if (c) {
                d[0] = static_cast<uint8_t>(std::clamp(r / c * colourScale * 255.0, 0.0, 255.0));
                d[1] = static_cast<uint8_t>(std::clamp(g / c * colourScale * 255.0, 0.0, 255.0));
                d[2] = static_cast<uint8_t>(std::clamp(b / c * colourScale * 255.0, 0.0, 255.0));
            }
            d[3] = static_cast<uint8_t>(255 * c / (S * S));
        }
    dilateRGB(img, w, h, w * 4, 6);
    for (std::size_t i = 3; i < img.size(); i += 4) if (img[i] == 1) img[i] = 0;
    for (int y = 0; y < h; ++y) std::memcpy(out + static_cast<std::size_t>(y) * stride, &img[static_cast<std::size_t>(y) * w * 4], static_cast<std::size_t>(w) * 4);
    if (normalOut) {
        std::vector<uint8_t> nimg(static_cast<std::size_t>(w) * h * 4, 0);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                Vec3 acc(0, 0, 0);
                int c = 0;
                for (int k = 0; k < S * S; ++k) {
                    const std::size_t i = static_cast<std::size_t>(y * S + k / S) * W + x * S + k % S;
                    if (!cov[i]) continue;
                    acc = acc + Vec3(nrm[i * 3], nrm[i * 3 + 1], nrm[i * 3 + 2]);
                    ++c;
                }
                const Vec3 n = c ? normalize(acc) : Vec3(0, 0, 1);
                uint8_t* d = &nimg[(static_cast<std::size_t>(y) * w + x) * 4];
                d[0] = static_cast<uint8_t>(std::clamp((n.x * 0.5 + 0.5) * 255.0, 0.0, 255.0));
                d[1] = static_cast<uint8_t>(std::clamp((n.y * 0.5 + 0.5) * 255.0, 0.0, 255.0));
                d[2] = static_cast<uint8_t>(std::clamp((n.z * 0.5 + 0.5) * 255.0, 0.0, 255.0));
                d[3] = c ? 255 : 0;
            }
        dilateRGB(nimg, w, h, w * 4, 6);   // under the cut, so the mips at the silhouette keep crown normals
        for (std::size_t i = 3; i < nimg.size(); i += 4) nimg[i] = 255;
        for (int y = 0; y < h; ++y) std::memcpy(normalOut + static_cast<std::size_t>(y) * stride, &nimg[static_cast<std::size_t>(y) * w * 4], static_cast<std::size_t>(w) * 4);
    }
}

}  // namespace engine
