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
const char* const kNames[] = {"spruce", "fir", "pine", "oak", "beech", "birch"};

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
    const double r0 = h * (s == RealSpecies::Birch ? 0.013 : 0.02);
    std::vector<Vec3> axis;
    // the trunk runs on up into the crown (a stem that forks once at one point reads as a candelabra)
    trunk(t.bark, r, h, sp.leader ? C.y + 0.6 * Ry : C.y + 0.3 * Ry, r0, sp, s == RealSpecies::Birch, &axis);
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
        const Vec3 Fm = lead ? F : axisAt(axis, forkY + (C.y - forkY) * (0.05 + 0.75 * f) + r.in(-0.4, 0.4));
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
            card(t.foliage, base, along, side, sz, sz * 0.9, n, sp.tint * (ao * r.in(0.82, 1.15)), r.below(kTiles * kTiles));
        }
    }
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
    // broadleaf: a twig forking into three or four, leaves along them and clustered at the ends
    const int forks = 3 + r.below(2);
    cv.stroke(X(0.5), Y(0.02), X(0.5), Y(0.35), 2.5 * pw, 0.5f);
    struct Leaf { double x, y, ang, len, wid; float lum; };
    std::vector<Leaf> leaves;
    const double sizeK = s == RealSpecies::Birch ? 0.75 : (s == RealSpecies::Oak ? 1.0 : 0.95);
    for (int f = 0; f < forks; ++f) {
        const double ang = kPi * 0.5 + (f - (forks - 1) * 0.5) * r.in(0.35, 0.55);
        const double len = r.in(0.4, 0.58);
        const double ex = 0.5 + len * std::cos(ang), ey = 0.35 + len * std::sin(ang) * (s == RealSpecies::Birch ? 0.8 : 1.0);
        cv.stroke(X(0.5), Y(0.35), X(ex), Y(ey), 1.6 * pw, 0.5f);
        const int nl = static_cast<int>((s == RealSpecies::Birch ? 7 : 6) + r.below(4));
        for (int k = 0; k < nl; ++k) {
            const double f2 = 0.2 + 0.8 * (k + r.next()) / nl;
            const double px = 0.5 + (ex - 0.5) * f2, py = 0.35 + (ey - 0.35) * f2;
            const double la = ang + (k % 2 ? 1 : -1) * r.in(0.5, 1.1) + (s == RealSpecies::Birch ? -0.4 : 0.0);
            const double L = r.in(0.13, 0.19) * sizeK;
            leaves.push_back({px + 0.5 * L * std::cos(la), py + 0.5 * L * std::sin(la), la, L, L * (s == RealSpecies::Oak ? 0.55 : (s == RealSpecies::Birch ? 0.7 : 0.58)),
                              static_cast<float>(r.in(0.7, 1.0))});
        }
    }
    // back to front by brightness: darker leaves first (they read as the ones behind)
    std::sort(leaves.begin(), leaves.end(), [](const Leaf& a, const Leaf& b) { return a.lum < b.lum; });
    for (const Leaf& L : leaves)
        cv.leaf(X(L.x), Y(L.y), L.ang, L.len * s0, L.wid * s0, s == RealSpecies::Oak ? 3 : 0, s == RealSpecies::Beech ? 0.05 : (s == RealSpecies::Birch ? 0.08 : 0.0), L.lum);
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
const char* realSpeciesName(RealSpecies s) { return kNames[std::min<int>(static_cast<int>(s), 5)]; }
bool realSpeciesIsConifer(RealSpecies s) { return s == RealSpecies::Spruce || s == RealSpecies::Fir || s == RealSpecies::Pine; }

RealTree realTree(RealSpecies species, uint32_t seed, double height) {
    RealTree t;
    ProcRng r(seed, 0x7EE5 + static_cast<uint64_t>(species));
    const Species sp = speciesOf(species);
    const double h = height > 0 ? height : r.in(sp.hLo, sp.hHi);
    if (species == RealSpecies::Spruce || species == RealSpecies::Fir) conifer(t, species, r, h);
    else shellCrown(t, species, r, h);
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
                    uint8_t* out, int stride) {
    const int S = 2, W = w * S, H = h * S;   // supersampled
    std::vector<float> depth(static_cast<std::size_t>(W) * H, -1e30f);
    std::vector<float> rgb(static_cast<std::size_t>(W) * H * 3, 0.0f);
    std::vector<uint8_t> cov(static_cast<std::size_t>(W) * H, 0);
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
}

}  // namespace engine
