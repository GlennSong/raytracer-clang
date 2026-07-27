#include "city_footprint.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine {
namespace {

constexpr double kPi = 3.14159265358979323846;

// splitmix32: the gate phase comes from a hash, not a shared RNG stream, so
// gate placement can never desync another stage's draws.
std::uint32_t mix32(std::uint32_t x) {
    x += 0x9e3779b9u;
    x ^= x >> 16; x *= 0x21f0aaadu;
    x ^= x >> 15; x *= 0x735a2d97u;
    x ^= x >> 15;
    return x;
}
double hash01(std::uint32_t x) { return mix32(x) / 4294967296.0; }

// --- the flood-fill grid ---------------------------------------------------
// Integer cells + fixed-order BFS: every step of F0 is deterministic by
// construction. Cell (i, j) covers origin + [i, i+1)x[j, j+1) * cell.
struct Grid {
    int n = 0;
    double cell = 0;
    Vec2 origin;
    int idx(int i, int j) const { return j * n + i; }
    bool in(int i, int j) const { return i >= 0 && i < n && j >= 0 && j < n; }
    Vec2 center(int i, int j) const {
        return origin + Vec2((i + 0.5) * cell, (j + 0.5) * cell);
    }
    Vec2 corner(int i, int j) const {
        return origin + Vec2(i * cell, j * cell);
    }
};

const int kDx[4] = {0, 1, 0, -1};   // N, E, S, W — fixed neighbor order
const int kDy[4] = {1, 0, -1, 0};

// Multi-source 4-connected BFS distance (in cells) from every non-mask cell;
// mask cells on the grid border count as distance 1 (outside is non-mask).
std::vector<int> distanceToEdge(const Grid& g, const std::vector<char>& mask) {
    std::vector<int> d(g.n * g.n, -1);
    std::vector<int> queue;
    queue.reserve(g.n * g.n);
    // Seed distance-0 sources first, distance-1 (border mask, outside is
    // non-mask) second: the FIFO queue must pop distances monotonically or
    // an early d=1 source can mis-relax a cell an interior wave owns.
    for (int j = 0; j < g.n; ++j)
        for (int i = 0; i < g.n; ++i) {
            const int c = g.idx(i, j);
            if (!mask[c]) {
                d[c] = 0;
                queue.push_back(c);
            }
        }
    for (int j = 0; j < g.n; ++j)
        for (int i = 0; i < g.n; ++i) {
            const int c = g.idx(i, j);
            if (mask[c] && d[c] < 0 &&
                (i == 0 || j == 0 || i == g.n - 1 || j == g.n - 1)) {
                d[c] = 1;
                queue.push_back(c);
            }
        }
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const int c = queue[head];
        const int ci = c % g.n, cj = c / g.n;
        for (int k = 0; k < 4; ++k) {
            const int ni = ci + kDx[k], nj = cj + kDy[k];
            if (!g.in(ni, nj)) continue;
            const int nc = g.idx(ni, nj);
            if (mask[nc] && d[nc] < 0) {
                d[nc] = d[c] + 1;
                queue.push_back(nc);
            }
        }
    }
    return d;
}

// Flood the connected component of `from` over `allowed`, 4-connected.
std::vector<char> floodComponent(const Grid& g, const std::vector<char>& allowed,
                                 int from) {
    std::vector<char> out(g.n * g.n, 0);
    if (from < 0 || !allowed[from]) return out;
    std::vector<int> queue;
    queue.push_back(from);
    out[from] = 1;
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const int c = queue[head];
        const int ci = c % g.n, cj = c / g.n;
        for (int k = 0; k < 4; ++k) {
            const int ni = ci + kDx[k], nj = cj + kDy[k];
            if (!g.in(ni, nj)) continue;
            const int nc = g.idx(ni, nj);
            if (allowed[nc] && !out[nc]) {
                out[nc] = 1;
                queue.push_back(nc);
            }
        }
    }
    return out;
}

// Interior holes (enclosed unbuildable pockets) fold INTO the footprint: the
// polygon is the outer contour only; pockets stay respected later by the
// per-emission buildable() checks and the face-centroid gate.
void fillHoles(const Grid& g, std::vector<char>& mask) {
    std::vector<char> outside(g.n * g.n, 0);
    std::vector<int> queue;
    for (int j = 0; j < g.n; ++j)
        for (int i = 0; i < g.n; ++i) {
            if (i != 0 && j != 0 && i != g.n - 1 && j != g.n - 1) continue;
            const int c = g.idx(i, j);
            if (!mask[c] && !outside[c]) {
                outside[c] = 1;
                queue.push_back(c);
            }
        }
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const int c = queue[head];
        const int ci = c % g.n, cj = c / g.n;
        for (int k = 0; k < 4; ++k) {
            const int ni = ci + kDx[k], nj = cj + kDy[k];
            if (!g.in(ni, nj)) continue;
            const int nc = g.idx(ni, nj);
            if (!mask[nc] && !outside[nc]) {
                outside[nc] = 1;
                queue.push_back(nc);
            }
        }
    }
    for (int c = 0; c < g.n * g.n; ++c)
        if (!mask[c] && !outside[c]) mask[c] = 1;
}

// Morphological OPENING at radius minWidth/2, anchored to the seed's core:
// erode (distance transform threshold), keep the seed's core component,
// geodesically re-dilate through the mask. Provably removes every lobe or
// peninsula narrower than minWidth and never widens the footprint. Returns
// false when no core survives (the whole region is thinner than minWidth).
bool openMask(const Grid& g, std::vector<char>& mask, int seedCell,
              double minWidth) {
    const std::vector<int> d = distanceToEdge(g, mask);
    const double coreCells = minWidth * 0.5 / g.cell - 0.25;   // half-cell slack
    std::vector<char> core(g.n * g.n, 0);
    bool any = false;
    for (int c = 0; c < g.n * g.n; ++c)
        if (mask[c] && d[c] >= coreCells) {
            core[c] = 1;
            any = true;
        }
    if (!any) return false;

    // The seed's core component — if the seed sits off-core (thin ground under
    // the seed itself), walk through the mask to the nearest core cell first.
    int anchor = seedCell;
    if (!core[anchor]) {
        std::vector<char> seen(g.n * g.n, 0);
        std::vector<int> queue;
        queue.push_back(seedCell);
        seen[seedCell] = 1;
        anchor = -1;
        for (std::size_t head = 0; head < queue.size() && anchor < 0; ++head) {
            const int c = queue[head];
            if (core[c]) { anchor = c; break; }
            const int ci = c % g.n, cj = c / g.n;
            for (int k = 0; k < 4; ++k) {
                const int ni = ci + kDx[k], nj = cj + kDy[k];
                if (!g.in(ni, nj)) continue;
                const int nc = g.idx(ni, nj);
                if (mask[nc] && !seen[nc]) {
                    seen[nc] = 1;
                    queue.push_back(nc);
                }
            }
        }
        if (anchor < 0) return false;
    }
    const std::vector<char> keptCore = floodComponent(g, core, anchor);

    // Geodesic dilation: grow the kept core back out through the mask by the
    // same radius. BFS depth-limited.
    std::vector<int> depth(g.n * g.n, -1);
    std::vector<int> queue;
    for (int c = 0; c < g.n * g.n; ++c)
        if (keptCore[c]) {
            depth[c] = 0;
            queue.push_back(c);
        }
    const double growCells = minWidth * 0.5 / g.cell + 0.25;
    for (std::size_t head = 0; head < queue.size(); ++head) {
        const int c = queue[head];
        if (depth[c] + 1 > growCells) continue;
        const int ci = c % g.n, cj = c / g.n;
        for (int k = 0; k < 4; ++k) {
            const int ni = ci + kDx[k], nj = cj + kDy[k];
            if (!g.in(ni, nj)) continue;
            const int nc = g.idx(ni, nj);
            if (mask[nc] && depth[nc] < 0) {
                depth[nc] = depth[c] + 1;
                queue.push_back(nc);
            }
        }
    }
    for (int c = 0; c < g.n * g.n; ++c) mask[c] = depth[c] >= 0 ? 1 : 0;
    return true;
}

// --- contour: boundary-edge walk ------------------------------------------
// Directed cell-face segments with the mask on the LEFT chain into loops with
// no marching-squares saddle ambiguity; the outer loop (largest area) is the
// footprint, CCW by construction.
struct BoundarySeg {
    int fromKey, toKey;   // corner-lattice keys, key = j * (n+1) + i
    int dir;              // 0=N 1=E 2=S 3=W (direction of travel)
    bool used = false;
};

Poly2 traceContour(const Grid& g, const std::vector<char>& mask) {
    const int cn = g.n + 1;   // corners per side
    auto key = [&](int i, int j) { return j * cn + i; };
    std::vector<BoundarySeg> segs;
    for (int j = 0; j < g.n; ++j)
        for (int i = 0; i < g.n; ++i) {
            if (!mask[g.idx(i, j)]) continue;
            auto open = [&](int ni, int nj) {
                return !g.in(ni, nj) || !mask[g.idx(ni, nj)];
            };
            if (open(i, j - 1))   // south face, travel east
                segs.push_back({key(i, j), key(i + 1, j), 1});
            if (open(i + 1, j))   // east face, travel north
                segs.push_back({key(i + 1, j), key(i + 1, j + 1), 0});
            if (open(i, j + 1))   // north face, travel west
                segs.push_back({key(i + 1, j + 1), key(i, j + 1), 3});
            if (open(i - 1, j))   // west face, travel south
                segs.push_back({key(i, j + 1), key(i, j), 2});
        }

    // Outgoing map: each corner has at most two outgoing segments (a saddle).
    std::vector<int> first(cn * cn, -1), second(cn * cn, -1);
    for (int s = 0; s < static_cast<int>(segs.size()); ++s) {
        int& slot = first[segs[s].fromKey] < 0 ? first[segs[s].fromKey]
                                               : second[segs[s].fromKey];
        slot = s;
    }
    auto leftOf = [](int dir) { return (dir + 3) % 4; };   // hug the mask

    Poly2 best;
    double bestArea = 0;
    for (int s0 = 0; s0 < static_cast<int>(segs.size()); ++s0) {
        if (segs[s0].used) continue;
        std::vector<int> loop;
        int cur = s0;
        while (!segs[cur].used) {
            segs[cur].used = true;
            loop.push_back(cur);
            const int at = segs[cur].toKey;
            const int a = first[at], b = second[at];
            int next = -1;
            if (a >= 0 && !segs[a].used) next = a;
            if (b >= 0 && !segs[b].used) {
                // Saddle: prefer the LEFT turn relative to the incoming
                // direction — the trace hugs the mask, splitting the pinch
                // into two loops instead of crossing itself.
                if (next < 0 || segs[b].dir == leftOf(segs[cur].dir)) next = b;
            }
            if (next < 0) break;
            cur = next;
        }
        if (loop.size() < 4) continue;
        Poly2 poly;
        poly.reserve(loop.size());
        for (int s : loop) {
            const int i = segs[s].fromKey % cn, j = segs[s].fromKey / cn;
            poly.push_back(g.corner(i, j));
        }
        const double a = std::abs(signedArea(poly));
        if (a > bestArea) {
            bestArea = a;
            best = std::move(poly);
        }
    }
    ensureCCW(best);
    return best;
}

// --- simplify / smooth / resample ------------------------------------------
void dpChain(const Poly2& pts, int a, int b, double eps,
             std::vector<char>& keep) {
    if (b - a < 2) return;
    const Vec2 A = pts[a], B = pts[b];
    const Vec2 ab = B - A;
    const double len2 = ab.lengthSquared();
    double worst = 0;
    int at = -1;
    for (int i = a + 1; i < b; ++i) {
        double d;
        if (len2 < 1e-12) {
            d = (pts[i] - A).length();
        } else {
            const double t =
                std::clamp(dot(pts[i] - A, ab) / len2, 0.0, 1.0);
            d = (pts[i] - (A + ab * t)).length();
        }
        if (d > worst) {
            worst = d;
            at = i;
        }
    }
    if (worst > eps && at > 0) {
        keep[at] = 1;
        dpChain(pts, a, at, eps, keep);
        dpChain(pts, at, b, eps, keep);
    }
}

Poly2 simplifyClosed(const Poly2& poly, double eps) {
    const int n = static_cast<int>(poly.size());
    if (n < 8) return poly;
    // Split the ring at the two mutually farthest vertices so Douglas-Peucker
    // runs on open chains with real endpoints.
    int ia = 0, ib = n / 2;
    double far2 = 0;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            const double d2 = (poly[i] - poly[j]).lengthSquared();
            if (d2 > far2) {
                far2 = d2;
                ia = i;
                ib = j;
            }
        }
    Poly2 rot;
    rot.reserve(n + 1);
    for (int k = 0; k <= n; ++k) rot.push_back(poly[(ia + k) % n]);
    const int mid = ib >= ia ? ib - ia : ib + n - ia;
    std::vector<char> keep(n + 1, 0);
    keep[0] = keep[mid] = keep[n] = 1;
    dpChain(rot, 0, mid, eps, keep);
    dpChain(rot, mid, n, eps, keep);
    Poly2 out;
    for (int k = 0; k < n; ++k)
        if (keep[k]) out.push_back(rot[k]);
    return out.size() >= 3 ? out : poly;
}

Poly2 chaikinClosed(const Poly2& poly, int passes) {
    Poly2 cur = poly;
    for (int p = 0; p < passes; ++p) {
        Poly2 next;
        next.reserve(cur.size() * 2);
        for (std::size_t i = 0; i < cur.size(); ++i) {
            const Vec2& a = cur[i];
            const Vec2& b = cur[(i + 1) % cur.size()];
            next.push_back(a * 0.75 + b * 0.25);
            next.push_back(a * 0.25 + b * 0.75);
        }
        cur = std::move(next);
    }
    return cur;
}

Poly2 resampleClosed(const Poly2& poly, double spacing) {
    const std::size_t n = poly.size();
    if (n < 3) return poly;
    std::vector<double> cum(n + 1, 0);
    for (std::size_t i = 0; i < n; ++i)
        cum[i + 1] = cum[i] + (poly[(i + 1) % n] - poly[i]).length();
    const double P = cum[n];
    if (P < 1e-6) return poly;
    const int m = std::max<int>(12, static_cast<int>(std::llround(P / spacing)));
    Poly2 out;
    out.reserve(m);
    std::size_t edge = 0;
    for (int k = 0; k < m; ++k) {
        const double want = P * k / m;
        while (edge + 1 < n && cum[edge + 1] < want) ++edge;
        const double span = cum[edge + 1] - cum[edge];
        const double t = span > 1e-12 ? (want - cum[edge]) / span : 0.0;
        out.push_back(lerp(poly[edge], poly[(edge + 1) % n], t));
    }
    return out;
}

bool isSimple(const Poly2& poly) {
    const std::size_t n = poly.size();
    auto segsIntersect = [](const Vec2& a, const Vec2& b, const Vec2& c,
                            const Vec2& d) {
        const double d1 = cross(b - a, c - a), d2 = cross(b - a, d - a);
        const double d3 = cross(d - c, a - c), d4 = cross(d - c, b - c);
        return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
    };
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 2; j < n; ++j) {
            if (i == 0 && j == n - 1) continue;   // adjacent around the wrap
            if (segsIntersect(poly[i], poly[(i + 1) % n], poly[j],
                              poly[(j + 1) % n]))
                return false;
        }
    return true;
}

// Inscribed-disc fallback: hostile or too-thin terrain degrades to an honest
// 24-gon around the deepest raw-mask point rather than failing the build.
Footprint discFallback(const Vec2& center, double radius) {
    Footprint f;
    f.degraded = true;
    f.center = center;
    f.coreDepth = radius;
    f.polygon.reserve(24);
    for (int k = 0; k < 24; ++k) {
        const double a = 2.0 * kPi * k / 24;
        f.polygon.push_back(center +
                            Vec2(std::cos(a) * radius, std::sin(a) * radius));
    }
    return f;
}

}  // namespace

Footprint buildFootprint(const Vec2& seed, double maxRadius,
                         const HeightSampler& ground,
                         const BuildabilityConfig& build,
                         const FootprintParams& params) {
    const double cell = std::max(8.0, params.cell);
    Grid g;
    g.cell = cell;
    g.n = std::max(8, static_cast<int>(std::ceil(2.0 * maxRadius / cell)));
    g.origin = seed - Vec2(g.n * cell * 0.5, g.n * cell * 0.5);

    // Buildable field, clipped to the site disc.
    std::vector<char> buildableCell(g.n * g.n, 0);
    for (int j = 0; j < g.n; ++j)
        for (int i = 0; i < g.n; ++i) {
            const Vec2 q = g.center(i, j);
            if ((q - seed).length() > maxRadius) continue;
            if (isBuildable(build, ground, q.x, q.y))
                buildableCell[g.idx(i, j)] = 1;
        }

    // Seed cell — spiral out to the nearest buildable cell if the seed itself
    // is hostile (fixed ring/order scan, deterministic).
    const int si0 = std::clamp(
        static_cast<int>((seed.x - g.origin.x) / cell), 0, g.n - 1);
    const int sj0 = std::clamp(
        static_cast<int>((seed.y - g.origin.y) / cell), 0, g.n - 1);
    int seedCell = -1;
    for (int r = 0; r < g.n && seedCell < 0; ++r) {
        for (int dj = -r; dj <= r && seedCell < 0; ++dj)
            for (int di = -r; di <= r; ++di) {
                if (std::max(std::abs(di), std::abs(dj)) != r) continue;
                const int i = si0 + di, j = sj0 + dj;
                if (g.in(i, j) && buildableCell[g.idx(i, j)]) {
                    seedCell = g.idx(i, j);
                    break;
                }
            }
    }
    if (seedCell < 0)
        return discFallback(seed, 0.5 * maxRadius);   // nothing buildable at all

    // The seed's connected buildable region, holes folded in.
    std::vector<char> mask = floodComponent(g, buildableCell, seedCell);
    fillHoles(g, mask);
    const std::vector<int> rawDist = distanceToEdge(g, mask);

    // Deepest raw point — the fallback disc anchor (and its radius).
    int rawBest = seedCell;
    for (int c = 0; c < g.n * g.n; ++c)
        if (mask[c] && rawDist[c] > rawDist[rawBest]) rawBest = c;
    const Vec2 rawCenter = g.center(rawBest % g.n, rawBest / g.n);
    const double rawDepth = rawDist[rawBest] * cell;
    auto fallback = [&]() {
        return discFallback(
            rawCenter, std::clamp(rawDepth, 250.0, 0.9 * maxRadius));
    };

    // De-skinny: opening at minWidth/2 anchored to the seed's core.
    if (params.minWidth > 0 && !openMask(g, mask, seedCell, params.minWidth))
        return fallback();

    int cells = 0;
    for (int c = 0; c < g.n * g.n; ++c) cells += mask[c];
    if (cells < 8) return fallback();

    // Contour -> simplify -> smooth -> resample.
    Poly2 poly = traceContour(g, mask);
    if (poly.size() < 3) return fallback();
    const double eps = params.simplifyEps > 0 ? params.simplifyEps : 0.6 * cell;
    poly = simplifyClosed(poly, eps);
    poly = chaikinClosed(poly, std::max(0, params.smoothPasses));
    poly = resampleClosed(poly, std::max(cell * 0.25, params.resample));
    ensureCCW(poly);

    // Center = deepest point of the FINAL mask (ties: lowest j, then i).
    const std::vector<int> dist = distanceToEdge(g, mask);
    int best = -1;
    for (int c = 0; c < g.n * g.n; ++c)
        if (mask[c] && (best < 0 || dist[c] > dist[best])) best = c;
    if (best < 0) return fallback();

    Footprint f;
    f.polygon = std::move(poly);
    f.center = g.center(best % g.n, best / g.n);
    f.coreDepth = dist[best] * cell;

    // Validity gates — any failure degrades honestly instead of shipping a
    // polygon later stages can't trust.
    if (!isSimple(f.polygon)) return fallback();
    if (!pointInPolygon(f.polygon, f.center)) return fallback();
    if (area(f.polygon) < 0.15 * kPi * maxRadius * maxRadius) return fallback();
    // NOTE: no polygon-level min-width certificate here — the grid opening
    // above already guarantees it for DERIVED footprints (within ~a cell of
    // discretization), and insetRingArcs' medial eps (~2% of depth) is far
    // tighter than rasterized-contour roughness, so it false-positives on
    // perfectly healthy polygons. Hand-AUTHORED override polygons (P8-G) get
    // their own discretization-tolerant validation instead.
    return f;
}

void placeFootprintGates(Footprint& f, double gateSpacing, std::uint32_t seed) {
    f.gates.clear();
    const std::size_t n = f.polygon.size();
    if (n < 3 || gateSpacing <= 0) return;
    std::vector<double> cum(n + 1, 0);
    for (std::size_t i = 0; i < n; ++i)
        cum[i + 1] = cum[i] + (f.polygon[(i + 1) % n] - f.polygon[i]).length();
    const double P = cum[n];
    if (P < gateSpacing * 0.5) return;
    const int count = std::max<int>(3, static_cast<int>(std::llround(P / gateSpacing)));
    const double spacing = P / count;
    const double phase = hash01(seed) * spacing;

    std::size_t edge = 0;
    for (int k = 0; k < count; ++k) {
        const double s = std::fmod(phase + k * spacing, P);
        if (s < cum[edge]) edge = 0;   // wrapped
        while (edge + 1 < n && cum[edge + 1] < s) ++edge;
        const double span = cum[edge + 1] - cum[edge];
        const double t = span > 1e-12 ? (s - cum[edge]) / span : 0.0;
        FootprintGate gate;
        gate.pos = lerp(f.polygon[edge], f.polygon[(edge + 1) % n], t);
        gate.s = s;
        const Vec2 tangent =
            normalize(f.polygon[(edge + 1) % n] - f.polygon[edge]);
        // Probe rather than trust a winding convention: whichever normal's
        // probe point lies outside the polygon is outward.
        Vec2 nrm = perp(tangent);
        if (pointInPolygon(f.polygon, gate.pos + nrm * 5.0)) nrm = -nrm;
        gate.outward = nrm;
        f.gates.push_back(gate);
    }
}

void pairConnectorGates(std::vector<Footprint>& sites) {
    const int n = static_cast<int>(sites.size());
    if (n < 2) return;
    // Prim MST over site centers (mirrors the legacy hub-MST intent).
    std::vector<char> inTree(n, 0);
    inTree[0] = 1;
    std::vector<std::pair<int, int>> links;
    for (int added = 1; added < n; ++added) {
        double best = 1e30;
        int ba = -1, bb = -1;
        for (int a = 0; a < n; ++a) {
            if (!inTree[a]) continue;
            for (int b = 0; b < n; ++b) {
                if (inTree[b]) continue;
                const double d = (sites[a].center - sites[b].center).length();
                if (d < best) {
                    best = d;
                    ba = a;
                    bb = b;
                }
            }
        }
        if (bb < 0) break;
        inTree[bb] = 1;
        links.push_back({ba, bb});
    }

    auto assign = [&](int from, int to) {
        Footprint& f = sites[from];
        if (f.gates.empty()) return;
        const Vec2 target = sites[to].center;
        int pick = -1;
        double bestDot = -2;
        // Prefer unassigned gates; fall back to reusing one only if every
        // gate already carries a connector.
        for (int pass = 0; pass < 2 && pick < 0; ++pass)
            for (std::size_t i = 0; i < f.gates.size(); ++i) {
                if (pass == 0 && f.gates[i].toSite >= 0) continue;
                const double d =
                    dot(f.gates[i].outward, normalize(target - f.gates[i].pos));
                if (d > bestDot) {
                    bestDot = d;
                    pick = static_cast<int>(i);
                }
            }
        if (pick >= 0) f.gates[pick].toSite = to;
    };
    for (const auto& [a, b] : links) {
        assign(a, b);
        assign(b, a);
    }
}

}  // namespace engine
