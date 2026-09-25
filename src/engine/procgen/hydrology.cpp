#include "hydrology.h"
#include "../mesh_builder.h"
#include "city/roads/lanes/geom2d.h"   // union + constrained Delaunay: the water polygon

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>
#include <tuple>

namespace engine {

namespace {
double clampd(double x, double a, double b) { return x < a ? a : (x > b ? b : x); }
RiverNode lerpNode(const RiverNode& a, const RiverNode& b, double t) {
    RiverNode r;
    r.p = a.p + (b.p - a.p) * t;
    r.level = a.level + (b.level - a.level) * t;
    r.width = a.width + (b.width - a.width) * t;
    r.depth = a.depth + (b.depth - a.depth) * t;
    r.area = a.area + (b.area - a.area) * t;
    return r;
}
}  // namespace

Vec2 Hydrology::cellCenter(int idx) const {
    return Vec2(-p_.half + (idx % n_) * p_.cell, -p_.half + (idx / n_) * p_.cell);
}

std::shared_ptr<const Hydrology> Hydrology::build(const std::function<double(double, double)>& ground, const HydroParams& p) {
    auto hy = std::make_shared<Hydrology>();
    Hydrology& H = *hy;
    H.p_ = p;
    const int n = std::max(3, static_cast<int>(std::ceil(2.0 * p.half / p.cell)) + 1);
    H.n_ = n;
    const std::size_t N = static_cast<std::size_t>(n) * n;
    std::vector<double> z(N), filled(N);
    std::vector<int> recv(N, -1), order;
    order.reserve(N);
    std::vector<char> closed(N, 0), sea(N, 0);
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) z[static_cast<std::size_t>(j) * n + i] = ground(-p.half + i * p.cell, -p.half + j * p.cell);

    // 1. PRIORITY-FLOOD from the edge and the sea inward: a cell is reached from its lowest open
    //    neighbour, takes max(its height, that neighbour's filled height), and drains into it.
    using Item = std::tuple<double, uint64_t, int>;   // filled height, insertion order (ties), cell
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    uint64_t counter = 0;
    for (std::size_t c = 0; c < N; ++c) {
        const int i = static_cast<int>(c % n), j = static_cast<int>(c / n);
        const bool edge = i == 0 || j == 0 || i == n - 1 || j == n - 1;
        sea[c] = z[c] < p.seaLevel;
        if (edge || sea[c]) {
            filled[c] = z[c];
            closed[c] = 1;
            open.push({z[c], counter++, static_cast<int>(c)});
        }
    }
    static const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1}, dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!open.empty()) {
        const auto [h, ord, c] = open.top();
        (void)ord;
        open.pop();
        order.push_back(c);
        const int i = c % n, j = c / n;
        for (int k = 0; k < 8; ++k) {
            const int a = i + di[k], b = j + dj[k];
            if (a < 0 || b < 0 || a >= n || b >= n) continue;
            const int nb = b * n + a;
            if (closed[static_cast<std::size_t>(nb)]) continue;
            closed[static_cast<std::size_t>(nb)] = 1;
            filled[static_cast<std::size_t>(nb)] = std::max(z[static_cast<std::size_t>(nb)], h);
            recv[static_cast<std::size_t>(nb)] = c;
            open.push({filled[static_cast<std::size_t>(nb)], counter++, nb});
        }
    }

    // 2. FLOW ACCUMULATION: each cell's area, passed down to its receiver, latest-flooded first.
    std::vector<double> acc(N, p.cell * p.cell);
    for (auto it = order.rbegin(); it != order.rend(); ++it) {
        const int c = *it, r = recv[static_cast<std::size_t>(c)];
        if (r >= 0) acc[static_cast<std::size_t>(r)] += acc[static_cast<std::size_t>(c)];
    }

    // LAKES: filled basins (connected cells at one fill level), big and deep enough.
    std::vector<int> lakeOf(N, -1);
    {
        std::vector<char> seen(N, 0);
        for (std::size_t c0 = 0; c0 < N; ++c0) {
            if (seen[c0] || sea[c0] || filled[c0] - z[c0] < 0.01) continue;
            std::vector<int> comp{static_cast<int>(c0)}, stack{static_cast<int>(c0)};
            seen[c0] = 1;
            double deepest = filled[c0] - z[c0];
            while (!stack.empty()) {
                const int c = stack.back(); stack.pop_back();
                const int i = c % n, j = c / n;
                for (int k = 0; k < 8; ++k) {
                    const int a = i + di[k], b = j + dj[k];
                    if (a < 0 || b < 0 || a >= n || b >= n) continue;
                    const std::size_t nb = static_cast<std::size_t>(b) * n + a;
                    if (seen[nb] || sea[nb] || filled[nb] - z[nb] < 0.01 || std::fabs(filled[nb] - filled[c0]) > 1e-6) continue;
                    seen[nb] = 1;
                    comp.push_back(static_cast<int>(nb));
                    stack.push_back(static_cast<int>(nb));
                    deepest = std::max(deepest, filled[nb] - z[nb]);
                }
            }
            if (static_cast<double>(comp.size()) * p.cell * p.cell < p.lakeMinArea || deepest < p.lakeMinDepth) continue;
            Lake L;
            L.level = filled[c0];
            L.cells = comp;
            L.minX = L.minZ = 1e30; L.maxX = L.maxZ = -1e30;
            for (int c : comp) {
                lakeOf[static_cast<std::size_t>(c)] = static_cast<int>(H.lakes_.size());
                const Vec2 q = H.cellCenter(c);
                L.minX = std::min(L.minX, q.x); L.maxX = std::max(L.maxX, q.x);
                L.minZ = std::min(L.minZ, q.y); L.maxZ = std::max(L.maxZ, q.y);
            }
            H.lakes_.push_back(std::move(L));
        }
    }

    H.lakeOfCell_ = lakeOf;
    // 3. RIVERS: cells draining at least riverArea, outside lakes and the sea. A source has no
    //    river upstream (a lake's outlet is one: the lake above it is not river).
    std::vector<char> isRiver(N, 0);
    std::vector<int> up(N, 0);
    for (std::size_t c = 0; c < N; ++c) isRiver[c] = acc[c] >= p.riverArea && !sea[c] && lakeOf[c] < 0;
    for (std::size_t c = 0; c < N; ++c)
        if (isRiver[c] && recv[c] >= 0 && isRiver[static_cast<std::size_t>(recv[c])]) ++up[static_cast<std::size_t>(recv[c])];
    std::vector<int> sources;
    for (std::size_t c = 0; c < N; ++c)
        if (isRiver[c] && up[c] == 0) sources.push_back(static_cast<int>(c));
    // longest-draining first, so a main stem is traced whole and its tributaries join it
    std::sort(sources.begin(), sources.end(), [&](int a, int b) { return acc[static_cast<std::size_t>(a)] > acc[static_cast<std::size_t>(b)]; });
    std::vector<char> traced(N, 0);
    auto nodeAt = [&](int c) {
        RiverNode nd;
        nd.p = H.cellCenter(c);
        nd.level = filled[static_cast<std::size_t>(c)];
        nd.area = acc[static_cast<std::size_t>(c)];
        return nd;
    };
    for (int s : sources) {
        River r;
        int c = s;
        while (true) {
            r.nodes.push_back(nodeAt(c));
            if (traced[static_cast<std::size_t>(c)]) break;   // joined a river already traced (the confluence)
            traced[static_cast<std::size_t>(c)] = 1;
            const int nx = recv[static_cast<std::size_t>(c)];
            if (nx < 0) break;                                               // the map's edge
            if (sea[static_cast<std::size_t>(nx)] || lakeOf[static_cast<std::size_t>(nx)] >= 0) {   // into the sea or a lake
                r.nodes.push_back(nodeAt(nx));
                r.mouth = true;
                r.intoLake = lakeOf[static_cast<std::size_t>(nx)];
                break;
            }
            c = nx;
        }
        if (r.nodes.size() < 3) continue;
        // smooth the grid staircase into a curve (Chaikin), endpoints kept
        for (int it = 0; it < p.smoothIterations; ++it) {
            std::vector<RiverNode> sm{r.nodes.front()};
            for (std::size_t k = 0; k + 1 < r.nodes.size(); ++k) {
                sm.push_back(lerpNode(r.nodes[k], r.nodes[k + 1], 0.25));
                sm.push_back(lerpNode(r.nodes[k], r.nodes[k + 1], 0.75));
            }
            sm.push_back(r.nodes.back());
            r.nodes = std::move(sm);
        }
        // The water only ever falls downstream, and never stands above the real ground at its own
        // centre line (the filled drainage surface does, on shallow flats -- the water floated):
        // where the ground rises downstream, the channel cuts through instead. Width and depth
        // come from the area drained.
        for (std::size_t k = 0; k < r.nodes.size(); ++k) {
            RiverNode& nd = r.nodes[k];
            nd.level = std::min(nd.level, ground(nd.p.x, nd.p.y) - 0.25);
            if (k > 0) nd.level = std::min(nd.level, r.nodes[k - 1].level);
            if (p.seaLevel > -1e29) nd.level = std::max(nd.level, p.seaLevel);
            if (r.intoLake >= 0) nd.level = std::max(nd.level, H.lakes_[static_cast<std::size_t>(r.intoLake)].level);   // meets the lake, no step
            const double sq = std::sqrt(nd.area);
            nd.width = clampd(p.widthMin + p.widthK * sq, p.widthMin, p.widthMax);
            nd.depth = clampd(p.depthMin + p.depthK * sq, p.depthMin, p.depthMax);
        }
        // THE MOUTH: over its last stretch into the sea or a lake the river widens (an estuary,
        // up to three times) and its surface fades out into the water it meets, instead of ending
        // on top of it as a strip.
        if (r.mouth) {
            const double E = std::max(60.0, 4.0 * r.nodes.back().width);
            double along = 0.0;
            for (std::size_t k = r.nodes.size(); k-- > 0;) {
                if (k + 1 < r.nodes.size()) along += (r.nodes[k + 1].p - r.nodes[k].p).length();
                if (along >= E) break;
                const double t = 1.0 - along / E;   // 1 at the mouth
                r.nodes[k].width *= 1.0 + 0.6 * t * t;
                const double f = clampd(along / (E * 0.6), 0.0, 1.0);
                r.nodes[k].fade = f * f * (3.0 - 2.0 * f);
            }
        }
        H.rivers_.push_back(std::move(r));
    }
    H.index();
    return hy;
}

void Hydrology::index() {
    segs_.clear();
    reach_ = 0.0;
    for (const River& r : rivers_)
        for (std::size_t k = 0; k + 1 < r.nodes.size(); ++k) {
            const RiverNode &a = r.nodes[k], &b = r.nodes[k + 1];
            segs_.push_back({a.p, b.p, a.level, b.level, a.width, b.width, a.depth, b.depth});
            reach_ = std::max(reach_, std::max(a.width, b.width) * 0.5 + kBankReach);
        }
    bins_ = std::max(1, static_cast<int>(std::ceil(2.0 * p_.half / binSize_)));
    bin_.assign(static_cast<std::size_t>(bins_) * bins_, {});
    for (std::size_t s = 0; s < segs_.size(); ++s) {
        const Seg& g = segs_[s];
        const double r = std::max(g.wa, g.wb) * 0.5 + kBankReach;
        const int i0 = std::clamp(static_cast<int>((std::min(g.a.x, g.b.x) - r + p_.half) / binSize_), 0, bins_ - 1);
        const int i1 = std::clamp(static_cast<int>((std::max(g.a.x, g.b.x) + r + p_.half) / binSize_), 0, bins_ - 1);
        const int j0 = std::clamp(static_cast<int>((std::min(g.a.y, g.b.y) - r + p_.half) / binSize_), 0, bins_ - 1);
        const int j1 = std::clamp(static_cast<int>((std::max(g.a.y, g.b.y) + r + p_.half) / binSize_), 0, bins_ - 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) bin_[static_cast<std::size_t>(j) * bins_ + i].push_back(static_cast<int>(s));
    }
}

double Hydrology::carve(double x, double z, double h) const {
    if (segs_.empty()) return h;
    const int i = static_cast<int>((x + p_.half) / binSize_), j = static_cast<int>((z + p_.half) / binSize_);
    if (i < 0 || j < 0 || i >= bins_ || j >= bins_) return h;
    double out = h;
    const Vec2 q(x, z);
    for (int s : bin_[static_cast<std::size_t>(j) * bins_ + i]) {
        const Seg& g = segs_[static_cast<std::size_t>(s)];
        const Vec2 ab = g.b - g.a;
        const double L2 = dot(ab, ab);
        const double t = L2 > 1e-12 ? clampd(dot(q - g.a, ab) / L2, 0.0, 1.0) : 0.0;
        const double d = (q - (g.a + ab * t)).length();
        const double w = g.wa + (g.wb - g.wa) * t;
        const double half = w * 0.5, reach = half + kBankReach;
        if (d >= reach) continue;
        const double level = g.la + (g.lb - g.la) * t, depth = g.da + (g.db - g.da) * t;
        // a rounded channel to the water's edge, then banks rising at bankSlope
        const double profile = d < half ? (level - depth) + depth * (d / half) * (d / half) : level + (d - half) * p_.bankSlope;
        double cut = std::min(h, profile);
        // fade out at the reach so a deep gorge does not end in a step
        const double fade = clampd((d - reach * 0.75) / (reach * 0.25), 0.0, 1.0);
        cut = cut + (h - cut) * fade * fade * (3 - 2 * fade);
        out = std::min(out, cut);
    }
    return out;
}

double Hydrology::distanceToRiver(double x, double z, double maxDist) const {
    double best = maxDist;
    for (const Seg& g : segs_) {
        const Vec2 q(x, z), ab = g.b - g.a;
        const double L2 = dot(ab, ab);
        const double t = L2 > 1e-12 ? clampd(dot(q - g.a, ab) / L2, 0.0, 1.0) : 0.0;
        const double w = g.wa + (g.wb - g.wa) * t;
        best = std::min(best, (q - (g.a + ab * t)).length() - w * 0.5);
    }
    return best;
}

RenderMesh Hydrology::waterMesh(const std::vector<std::vector<Vec2>>& sea) const {
    namespace L = roads::lanes;
    RenderMesh out;
    // 1. THE OUTLINE: every river's corridor (a quad per segment and a disc at each node, on a
    //    path resampled to its width) and every lake's cells (dilated one cell under its shore),
    //    unioned into one polygon set. Where a river meets another, a lake or the sea's edge the
    //    shapes merge; nothing can fold or overlap.
    std::vector<L::Ring> rings;
    std::vector<Vec2> interior;   // river centre points: the triangulation's inner vertices
    auto disc = [&](const Vec2& c, double r) {
        L::Ring ring;
        for (int k = 0; k < 12; ++k) {
            const double a = 6.283185307179586 * k / 12;
            ring.push_back(c + Vec2(std::cos(a), std::sin(a)) * r);
        }
        rings.push_back(std::move(ring));
    };
    for (const River& r : rivers_) {
        // resample: a node every ~width/3 (at least 3 m), so the corridor is smooth but light
        std::vector<RiverNode> path{r.nodes.front()};
        double acc = 0.0;
        for (std::size_t k = 1; k < r.nodes.size(); ++k) {
            acc += (r.nodes[k].p - r.nodes[k - 1].p).length();
            const double step = std::max(3.0, r.nodes[k].width / 3.0);
            if (acc >= step || k + 1 == r.nodes.size()) { path.push_back(r.nodes[k]); acc = 0.0; }
        }
        for (std::size_t k = 0; k < path.size(); ++k) {
            const double hw = path[k].width * 0.5 * 1.05;
            disc(path[k].p, hw);
            interior.push_back(path[k].p);
            if (k + 1 == path.size()) continue;
            const Vec2 a = path[k].p, b = path[k + 1].p, d = b - a;
            const double len = d.length();
            if (len < 1e-6) continue;
            const Vec2 n = perp(d / len);
            const double hb = path[k + 1].width * 0.5 * 1.05;
            rings.push_back({a - n * hw, b - n * hb, b + n * hb, a + n * hw});
        }
    }
    const int n = n_;
    std::vector<L::Ring> lakeSquares;
    for (const Lake& lk : lakes_) {
        std::vector<char> in(static_cast<std::size_t>(n) * n, 0);
        for (int c : lk.cells) {
            const int i = c % n, j = c / n;
            for (int b = j - 1; b <= j + 1; ++b)
                for (int a = i - 1; a <= i + 1; ++a)
                    if (a >= 0 && b >= 0 && a < n && b < n) in[static_cast<std::size_t>(b) * n + a] = 1;
        }
        for (int c : lk.cells) interior.push_back(cellCenter(c));   // the lake's open water: away from the bank
        const double h = p_.cell * 0.5;
        for (std::size_t c = 0; c < in.size(); ++c) {
            if (!in[c]) continue;
            const Vec2 q = cellCenter(static_cast<int>(c));
            lakeSquares.push_back({q + Vec2(-h, -h), q + Vec2(h, -h), q + Vec2(h, h), q + Vec2(-h, h)});
        }
    }
    // the lakes' cell outlines are staircases: round them (Chaikin, corner cutting) before the union
    auto chaikin = [](const L::Ring& r) {
        L::Ring o;
        for (std::size_t i = 0; i < r.size(); ++i) {
            const Vec2 a = r[i], b = r[(i + 1) % r.size()];
            o.push_back(a * 0.75 + b * 0.25);
            o.push_back(a * 0.25 + b * 0.75);
        }
        return o;
    };
    for (const L::Polygon2& poly : L::unionRings(lakeSquares)) {
        L::Ring o = poly.outer;
        for (int it = 0; it < 3; ++it) o = chaikin(o);
        rings.push_back(std::move(o));
        for (L::Ring hole : poly.holes) {   // an island: keep it, as a CW ring subtracts under non-zero
            for (int it = 0; it < 3; ++it) hole = chaikin(hole);
            rings.push_back(std::move(hole));
        }
    }
    if (rings.empty()) return out;
    L::PolySet water = L::unionRings(rings);
    // the sea is the ocean surface's: stop exactly at its cells (only those near the water)
    if (!sea.empty()) {
        double x0 = 1e30, z0 = 1e30, x1 = -1e30, z1 = -1e30;
        for (const L::Polygon2& poly : water)
            for (const Vec2& q : poly.outer) { x0 = std::min(x0, q.x); z0 = std::min(z0, q.y); x1 = std::max(x1, q.x); z1 = std::max(z1, q.y); }
        std::vector<L::Ring> near;
        for (const auto& c : sea)
            if (c[2].x >= x0 && c[0].x <= x1 && c[2].y >= z0 && c[0].y <= z1) near.push_back(c);
        if (!near.empty()) water = L::differenceSets(water, L::unionRings(near));
        if (water.empty()) return out;
    }

    // 2. TRIANGULATE: the outline (densified to ~4 m, its edges constrained) plus the river centre
    //    points inside it; triangles outside the water are dropped.
    std::vector<Vec2> pts;
    std::vector<std::pair<int, int>> edges;
    std::set<std::pair<long long, long long>> bank;   // the outline's points (mm): distance to the bank 0
    auto key = [](const Vec2& q) { return std::make_pair(std::llround(q.x * 1000.0), std::llround(q.y * 1000.0)); };
    auto addRing = [&](const L::Ring& ring) {
        const int start = static_cast<int>(pts.size());
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const Vec2 a = ring[i], b = ring[(i + 1) % ring.size()];
            const int k = std::max(1, static_cast<int>(std::ceil((b - a).length() / 4.0)));
            for (int m = 0; m < k; ++m) { pts.push_back(a + (b - a) * (static_cast<double>(m) / k)); bank.insert(key(pts.back())); }
        }
        const int end = static_cast<int>(pts.size());
        for (int i = start; i < end; ++i) edges.emplace_back(i, i + 1 < end ? i + 1 : start);
    };
    for (const L::Polygon2& poly : water) {
        addRing(poly.outer);
        for (const L::Ring& hole : poly.holes) addRing(hole);
    }
    // centre points well inside (not on top of the outline's own points)
    for (const Vec2& q : interior) pts.push_back(q);
    const L::Triangulation T = L::constrainedTriangulation(pts, edges);
    auto inWater = [&](const Vec2& c) {
        for (const L::Polygon2& poly : water) {
            // point in polygon (outer minus holes), even-odd on each ring
            auto inside = [&](const L::Ring& r) {
                bool odd = false;
                for (std::size_t i = 0, j = r.size() - 1; i < r.size(); j = i++)
                    if (((r[i].y > c.y) != (r[j].y > c.y)) && (c.x < (r[j].x - r[i].x) * (c.y - r[i].y) / (r[j].y - r[i].y) + r[i].x)) odd = !odd;
                return odd;
            };
            if (!inside(poly.outer)) continue;
            bool inHole = false;
            for (const L::Ring& h : poly.holes) if (inside(h)) { inHole = true; break; }
            if (!inHole) return true;
        }
        return false;
    };

    // the river segments on a coarse grid, for the nearest-segment query below
    constexpr double kBin = 64.0;
    const int nb = std::max(1, static_cast<int>(std::ceil(2.0 * p_.half / kBin)));
    std::vector<std::vector<std::pair<int, int>>> segBin(static_cast<std::size_t>(nb) * nb);
    auto binOf = [&](double v) { return std::clamp(static_cast<int>((v + p_.half) / kBin), 0, nb - 1); };
    for (std::size_t ri = 0; ri < rivers_.size(); ++ri)
        for (std::size_t k = 0; k + 1 < rivers_[ri].nodes.size(); ++k) {
            const Vec2 a = rivers_[ri].nodes[k].p, b = rivers_[ri].nodes[k + 1].p;
            segBin[static_cast<std::size_t>(binOf((a.y + b.y) * 0.5)) * nb + binOf((a.x + b.x) * 0.5)].emplace_back(static_cast<int>(ri), static_cast<int>(k));
        }

    // 3. EACH VERTEX: its water level and flow. Inside a lake (by the drainage grid), the lake's
    //    level and still water; otherwise the nearest river segment's level, direction and speed.
    std::vector<uint32_t> remap(T.verts.size(), UINT32_MAX);
    auto vertexFor = [&](int vi) {
        if (remap[static_cast<std::size_t>(vi)] != UINT32_MAX) return remap[static_cast<std::size_t>(vi)];
        const Vec2 q = T.verts[static_cast<std::size_t>(vi)];
        double level = p_.seaLevel > -1e29 ? p_.seaLevel : 0.0, speed = 0.0, fade = 1.0;
        Vec2 flow(1, 0);
        const int gi = std::clamp(static_cast<int>(std::lround((q.x + p_.half) / p_.cell)), 0, n - 1);
        const int gj = std::clamp(static_cast<int>(std::lround((q.y + p_.half) / p_.cell)), 0, n - 1);
        int lake = -1;
        for (int b = gj - 1; b <= gj + 1 && lake < 0; ++b)
            for (int a = gi - 1; a <= gi + 1 && lake < 0; ++a)
                if (a >= 0 && b >= 0 && a < n && b < n) lake = lakeOfCell_[static_cast<std::size_t>(b) * n + a];
        // the nearest river segment (the bins first, every segment if none is near)
        double best = 1e30, bestT = 0.0;
        const River* bestR = nullptr;
        std::size_t bestK = 0;
        const int bi = binOf(q.x), bj = binOf(q.y);
        for (int b = std::max(0, bj - 1); b <= std::min(nb - 1, bj + 1); ++b)
            for (int a2 = std::max(0, bi - 1); a2 <= std::min(nb - 1, bi + 1); ++a2)
                for (const auto& [ri, k] : segBin[static_cast<std::size_t>(b) * nb + a2]) {
                    const River& r = rivers_[static_cast<std::size_t>(ri)];
                    const Vec2 a = r.nodes[static_cast<std::size_t>(k)].p, ab = r.nodes[static_cast<std::size_t>(k) + 1].p - a;
                    const double L2 = dot(ab, ab);
                    const double t = L2 > 1e-12 ? clampd(dot(q - a, ab) / L2, 0.0, 1.0) : 0.0;
                    const double d = (q - (a + ab * t)).length();
                    if (d < best) { best = d; bestT = t; bestR = &r; bestK = static_cast<std::size_t>(k); }
                }
        // near a lake its level wins: a river's surface is at the lake's level where it meets it
        if (lake >= 0) {
            level = lakes_[static_cast<std::size_t>(lake)].level;
        } else if (bestR) {
            const RiverNode& A = bestR->nodes[bestK];
            const RiverNode& B = bestR->nodes[bestK + 1];
            level = A.level + (B.level - A.level) * bestT;
            fade = A.fade + (B.fade - A.fade) * bestT;
            const Vec2 d = B.p - A.p;
            if (d.length() > 1e-9) flow = d / d.length();
            // speed from the surface's fall over ~20 m of the river
            const std::size_t k0 = bestK >= 8 ? bestK - 8 : 0, k1 = std::min(bestR->nodes.size() - 1, bestK + 8);
            const double run = std::max(1.0, (bestR->nodes[k1].p - bestR->nodes[k0].p).length());
            speed = clampd((bestR->nodes[k0].level - bestR->nodes[k1].level) / run * 6.0, 0.0, 1.0);
        }
        Vertex v(Vec3(q.x, level, q.y), Vec3(0, 1, 0), Vec3(flow.x, 0, flow.y), bank.count(key(q)) ? 0.0f : 0.5f, 0.0f);
        v.color = Vec3(speed, fade, 0.0);
        out.vertices.push_back(v);
        return remap[static_cast<std::size_t>(vi)] = static_cast<uint32_t>(out.vertices.size() - 1);
    };
    for (const auto& t : T.tris) {
        const Vec2 c = (T.verts[static_cast<std::size_t>(t[0])] + T.verts[static_cast<std::size_t>(t[1])] + T.verts[static_cast<std::size_t>(t[2])]) * (1.0 / 3.0);
        if (!inWater(c)) continue;
        const uint32_t a = vertexFor(t[0]), b = vertexFor(t[1]), cc = vertexFor(t[2]);
        MeshBuilder::triFacing(out, a, b, cc, Vec3(0, 1, 0));
    }
    return out;
}

}  // namespace engine
