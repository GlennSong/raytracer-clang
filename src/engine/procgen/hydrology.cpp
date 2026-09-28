#include "hydrology.h"
#include "../mesh_builder.h"
#include "city/roads/lanes/geom2d.h"   // union + constrained Delaunay: the water polygon

#include <algorithm>
#include <cmath>
#include <limits>
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
    std::vector<River> found;
    for (int s : sources) {
        if (!p.autoRivers) break;
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
        found.push_back(std::move(r));
    }
    // AUTHORED RIVERS (ADR-0104): a course a level lays down -- from a source in the range, through a
    // gap, out to the sea -- walked every half cell with a gentle meander, its width running from the
    // source's to the mouth's. It is carved and drawn like any other river.
    for (const HydroParams::Course& course : p.courses) {
        if (course.points.size() < 2) continue;
        River r;
        r.authored = true;
        r.width0 = course.width0;
        r.width1 = course.width1;
        double total = 0.0;
        for (std::size_t k = 0; k + 1 < course.points.size(); ++k) total += (course.points[k + 1] - course.points[k]).length();
        double along = 0.0;
        for (std::size_t k = 0; k + 1 < course.points.size(); ++k) {
            const Vec2 a = course.points[k], b = course.points[k + 1];
            const double L = (b - a).length();
            const int m = std::max(1, static_cast<int>(std::ceil(L / (p.cell * 0.5))));
            const Vec2 side = L > 1e-9 ? perp((b - a) / L) : Vec2(1, 0);
            for (int j = 0; j < m; ++j) {
                const double t = static_cast<double>(j) / m, s0 = along + L * t;
                // a meander: two slow waves, fading to nothing at the waypoints (they are where it goes)
                const double fadeW = std::sin(3.14159265358979 * t);
                const double wob = course.meander * fadeW * (std::sin(s0 / 170.0 + 1.3) + 0.5 * std::sin(s0 / 61.0 + 4.1));
                RiverNode nd;
                nd.p = a + (b - a) * t + side * wob;
                nd.level = 1e30;
                nd.area = s0 / std::max(1.0, total);   // authored: the fraction along (the width's parameter)
                r.nodes.push_back(nd);
            }
            along += L;
        }
        RiverNode last;
        last.p = course.points.back();
        last.level = 1e30;
        last.area = 1.0;
        r.nodes.push_back(last);
        const int lc = std::clamp(static_cast<int>(std::lround((last.p.x + p.half) / p.cell)), 0, n - 1) +
                       std::clamp(static_cast<int>(std::lround((last.p.y + p.half) / p.cell)), 0, n - 1) * n;
        r.mouth = sea[static_cast<std::size_t>(lc)] != 0;
        found.push_back(std::move(r));
    }
    for (River& r : found) {
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
            if (r.authored) {
                nd.width = r.width0 + (r.width1 - r.width0) * clampd(nd.area, 0.0, 1.0);
                nd.depth = clampd(0.8 + 0.08 * nd.width, p.depthMin, p.depthMax);
            } else {
                const double sq = std::sqrt(nd.area);
                nd.width = clampd(p.widthMin + p.widthK * sq, p.widthMin, p.widthMax);
                nd.depth = clampd(p.depthMin + p.depthK * sq, p.depthMin, p.depthMax);
            }
            // the lowest natural ground across the corridor (centre and just past each bank), less
            // the incision: the water can never stand above a bank, and the channel is a trench
            const Vec2 along = r.nodes[std::min(k + 1, r.nodes.size() - 1)].p - r.nodes[k > 0 ? k - 1 : 0].p;
            const Vec2 side = along.length() > 1e-9 ? perp(along / along.length()) : Vec2(1, 0);
            const double reach = nd.width * 0.5 + 3.0;
            double low = ground(nd.p.x, nd.p.y);
            for (double s : {-1.0, -0.5, 0.5, 1.0}) {
                const Vec2 q = nd.p + side * (s * reach);
                low = std::min(low, ground(q.x, q.y));
            }
            const double incision = clampd(p.incisionMin + p.incisionK * nd.width, p.incisionMin, p.incisionMax);
            nd.level = std::min(nd.level, low - incision);
            if (k > 0) nd.level = std::min(nd.level, r.nodes[k - 1].level);
            if (p.seaLevel > -1e29) nd.level = std::max(nd.level, p.seaLevel);
            if (r.intoLake >= 0) nd.level = std::max(nd.level, H.lakes_[static_cast<std::size_t>(r.intoLake)].level);   // meets the lake, no step
        }
        // WIDTH BY REACH (ADR-0121): the fall over +-60 m of the course, from the levels just set
        if (p.widthVariation > 0.0 && r.nodes.size() >= 3) {
            const double v = p.widthVariation;
            const std::size_t n = r.nodes.size();
            std::vector<double> st(n, 0.0), mul(n, 1.0);
            for (std::size_t k = 1; k < n; ++k) st[k] = st[k - 1] + (r.nodes[k].p - r.nodes[k - 1].p).length();
            const double phase = 0.37 * static_cast<double>(found.size() + H.rivers_.size());
            for (std::size_t k = 0, a = 0, b = 0; k < n; ++k) {
                while (a < k && st[k] - st[a] > 60.0) ++a;
                while (b + 1 < n && st[b + 1] - st[k] <= 60.0) ++b;
                const double run = std::max(1.0, st[b] - st[a]);
                const double slope = std::max(0.0, r.nodes[a].level - r.nodes[b].level) / run;
                const double t = std::clamp((slope - 0.004) / (0.04 - 0.004), 0.0, 1.0);
                const double tt = t * t * (3.0 - 2.0 * t);
                mul[k] = (1.0 + 0.7 * v) * (1.0 - tt) + (1.0 - 0.35 * v) * tt;
                mul[k] *= 1.0 + 0.2 * v * std::sin(st[k] / 180.0 + phase);   // pools and narrows
            }
            // smooth over ~40 m so a width never steps
            std::vector<double> sm(n, 1.0);
            for (std::size_t k = 0, a = 0, b = 0; k < n; ++k) {
                while (a < k && st[k] - st[a] > 40.0) ++a;
                while (b + 1 < n && st[b + 1] - st[k] <= 40.0) ++b;
                double sum = 0.0; for (std::size_t q = a; q <= b; ++q) sum += mul[q];
                sm[k] = sum / static_cast<double>(b - a + 1);
            }
            for (std::size_t k = 0; k < n; ++k)
                r.nodes[k].width = std::clamp(r.nodes[k].width * sm[k], p.widthMin * 0.8, p.widthMax * 1.5);
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
        // OUT ACROSS THE SHELF: a river into the SEA carries its channel on along its last heading, at
        // sea level, until the sea floor is deeper than the channel (at most 600 m). It used to end at
        // the first sea cell, where the floor is centimetres under the surface: the river's deep water
        // stopped in a rounded tip on a wide pale flat, and the ocean began a few hundred metres out
        // (Glenn: "some of the rivers don't meet the ocean nicely and just stop near the edge"). The
        // new nodes are faded out -- the surface there is the ocean's -- so only the carve sees them.
        if (r.mouth && r.intoLake < 0 && p.seaLevel > -1e29 && r.nodes.size() >= 2) {
            const RiverNode end = r.nodes.back();
            Vec2 dir = end.p - r.nodes[r.nodes.size() - 2].p;
            for (std::size_t k = r.nodes.size() - 1; k-- > 0 && dir.length() < 20.0;) dir = end.p - r.nodes[k].p;   // a heading over ~20 m
            if (dir.length() > 1e-9) {
                dir = dir / dir.length();
                r.shelf.push_back(end);
                const double stepLen = p.cell * 0.5;
                for (double t = stepLen; t <= 600.0; t += stepLen) {
                    RiverNode nd = end;
                    nd.p = end.p + dir * t;
                    nd.level = p.seaLevel;
                    nd.fade = 0.0;
                    r.shelf.push_back(nd);
                    if (ground(nd.p.x, nd.p.y) < p.seaLevel - end.depth - 1.0) break;   // the sea is deeper than the channel
                }
            }
        }
        H.rivers_.push_back(std::move(r));
    }
    H.index();
    return hy;
}

bool Hydrology::inLake(double x, double z, double margin) const {
    const int n = n_;
    if (n <= 0 || lakeOfCell_.empty()) return false;
    const int i0 = static_cast<int>(std::lround((x + p_.half) / p_.cell)), j0 = static_cast<int>(std::lround((z + p_.half) / p_.cell));
    const int r = static_cast<int>(std::ceil(margin / p_.cell));
    for (int j = j0 - r; j <= j0 + r; ++j)
        for (int i = i0 - r; i <= i0 + r; ++i) {
            if (i < 0 || j < 0 || i >= n || j >= n || lakeOfCell_[static_cast<std::size_t>(j) * n + i] < 0) continue;
            const double dx = -p_.half + i * p_.cell - x, dz = -p_.half + j * p_.cell - z;
            if (dx * dx + dz * dz <= (margin + 0.5 * p_.cell) * (margin + 0.5 * p_.cell)) return true;
        }
    return false;
}

double Hydrology::lakeLevelAt(double x, double z, double margin) const {
    const int n = n_;
    if (n <= 0 || lakeOfCell_.empty()) return std::numeric_limits<double>::quiet_NaN();
    const int i0 = static_cast<int>(std::lround((x + p_.half) / p_.cell)), j0 = static_cast<int>(std::lround((z + p_.half) / p_.cell));
    const int r = static_cast<int>(std::ceil(margin / p_.cell));
    double level = std::numeric_limits<double>::quiet_NaN();
    for (int j = j0 - r; j <= j0 + r; ++j)
        for (int i = i0 - r; i <= i0 + r; ++i) {
            if (i < 0 || j < 0 || i >= n || j >= n) continue;
            const int lk = lakeOfCell_[static_cast<std::size_t>(j) * n + i];
            if (lk < 0 || lk >= static_cast<int>(lakes_.size())) continue;
            const double dx = -p_.half + i * p_.cell - x, dz = -p_.half + j * p_.cell - z;
            if (dx * dx + dz * dz > (margin + 0.5 * p_.cell) * (margin + 0.5 * p_.cell)) continue;
            const double lv = lakes_[static_cast<std::size_t>(lk)].level;
            if (!std::isfinite(level) || lv > level) level = lv;
        }
    return level;
}

bool Hydrology::onShelf(double x, double z) const {
    const Vec2 q(x, z);
    for (const River& r : rivers_) {
        for (std::size_t k = 0; k + 1 < r.shelf.size(); ++k) {
            const Vec2 a = r.shelf[k].p, ab = r.shelf[k + 1].p - a;
            const double reach = std::max(r.shelf[k].width, r.shelf[k + 1].width) * 0.5 + kBankReach;
            if (std::fabs(q.x - a.x) > reach + std::fabs(ab.x) || std::fabs(q.y - a.y) > reach + std::fabs(ab.y)) continue;
            const double L2 = dot(ab, ab);
            const double t = L2 > 1e-12 ? clampd(dot(q - a, ab) / L2, 0.0, 1.0) : 0.0;
            if ((q - (a + ab * t)).length() <= reach) return true;
        }
    }
    return false;
}

void Hydrology::index() {
    segs_.clear();
    farBin_.clear();
    farBins_ = 0;
    reach_ = 0.0;
    for (const River& r : rivers_)
        for (const std::vector<RiverNode>* run : {&r.nodes, &r.shelf})   // the carve cuts the shelf channel too
            for (std::size_t k = 0; k + 1 < run->size(); ++k) {
                const RiverNode &a = (*run)[k], &b = (*run)[k + 1];
                segs_.push_back({a.p, b.p, a.level, b.level, a.width, b.width, a.depth, b.depth, run == &r.shelf});
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
    // the FAR index: each segment in every bin within kFarReach of it, for distance / level queries
    // (the carve's own bins reach only its banks; widening them would slow every terrain sample)
    farBins_ = std::max(1, static_cast<int>(std::ceil(2.0 * p_.half / kFarBin)));
    farBin_.assign(static_cast<std::size_t>(farBins_) * farBins_, {});
    for (std::size_t s = 0; s < segs_.size(); ++s) {
        const Seg& g = segs_[s];
        const double r = std::max(g.wa, g.wb) * 0.5 + kFarReach;
        const int i0 = std::clamp(static_cast<int>((std::min(g.a.x, g.b.x) - r + p_.half) / kFarBin), 0, farBins_ - 1);
        const int i1 = std::clamp(static_cast<int>((std::max(g.a.x, g.b.x) + r + p_.half) / kFarBin), 0, farBins_ - 1);
        const int j0 = std::clamp(static_cast<int>((std::min(g.a.y, g.b.y) - r + p_.half) / kFarBin), 0, farBins_ - 1);
        const int j1 = std::clamp(static_cast<int>((std::max(g.a.y, g.b.y) + r + p_.half) / kFarBin), 0, farBins_ - 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i) farBin_[static_cast<std::size_t>(j) * farBins_ + i].push_back(static_cast<int>(s));
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
        // a rounded channel to the water's edge, then the steep inner bank up the incision height,
        // then the outer banks at bankSlope
        const double incision = clampd(p_.incisionMin + p_.incisionK * w, p_.incisionMin, p_.incisionMax);
        const double past = d - half, steepRun = incision / p_.bankSteep;
        const double profile = d < half ? (level - depth) + depth * (d / half) * (d / half)
                                        : level + (past < steepRun ? past * p_.bankSteep : incision + (past - steepRun) * p_.bankSlope);
        double cut = std::min(h, profile);
        // fade out at the reach so a deep gorge does not end in a step
        const double fade = clampd((d - reach * 0.75) / (reach * 0.25), 0.0, 1.0);
        cut = cut + (h - cut) * fade * fade * (3 - 2 * fade);
        out = std::min(out, cut);
    }
    return out;
}

bool Hydrology::isWet(double x, double z, double margin) const {
    const int n = n_;
    if (n > 0 && !lakeOfCell_.empty()) {
        const int i = static_cast<int>(std::lround((x + p_.half) / p_.cell)), j = static_cast<int>(std::lround((z + p_.half) / p_.cell));
        if (i >= 0 && j >= 0 && i < n && j < n && lakeOfCell_[static_cast<std::size_t>(j) * n + i] >= 0) return true;
    }
    if (segs_.empty()) return false;
    const int bi = static_cast<int>((x + p_.half) / binSize_), bj = static_cast<int>((z + p_.half) / binSize_);
    if (bi < 0 || bj < 0 || bi >= bins_ || bj >= bins_) return false;
    const Vec2 q(x, z);
    for (int s : bin_[static_cast<std::size_t>(bj) * bins_ + bi]) {
        const Seg& g = segs_[static_cast<std::size_t>(s)];
        if (g.shelf) continue;   // under the sea: not a river
        const Vec2 ab = g.b - g.a;
        const double L2 = dot(ab, ab);
        const double t = L2 > 1e-12 ? clampd(dot(q - g.a, ab) / L2, 0.0, 1.0) : 0.0;
        if ((q - (g.a + ab * t)).length() < (g.wa + (g.wb - g.wa) * t) * 0.5 + margin) return true;
    }
    return false;
}

double Hydrology::shore(double x, double z, double y, double* waterline) const {
    if (waterline) *waterline = 0.0;
    auto sm = [](double a, double b, double v) { const double t = clampd((v - a) / (b - a), 0.0, 1.0); return t * t * (3.0 - 2.0 * t); };
    double best = 0.0, line = 0.0;
    double lvl = 0.0;
    const double d = distanceToRiver(x, z, 8.0, &lvl);
    if (d < 8.0 && lvl == lvl) {
        const double up = y - lvl;
        best = (1.0 - sm(1.5, 5.0, d)) * (1.0 - sm(0.6, 1.3, up));
        line = (1.0 - sm(0.2, 0.8, d)) * (1.0 - sm(0.1, 0.3, up));
    }
    // lakes: by height over the lake's level near its cells (a lake's edge is where its level meets the ground)
    if (inLake(x, z, 12.0))
        for (const Lake& L : lakes_) {
            if (x < L.minX - 30.0 || x > L.maxX + 30.0 || z < L.minZ - 30.0 || z > L.maxZ + 30.0) continue;
            const double up = y - L.level;
            if (up < -0.5 || up > 2.0) continue;
            best = std::max(best, 1.0 - sm(0.45, 1.0, up));
            line = std::max(line, 1.0 - sm(0.06, 0.18, up));
        }
    if (waterline) *waterline = line;
    return best;
}

double Hydrology::distanceToRiver(double x, double z, double maxDist, double* level) const {
    double best = maxDist;
    if (level) *level = std::numeric_limits<double>::quiet_NaN();
    if (segs_.empty() || farBins_ <= 0) return best;
    const int bi = static_cast<int>((x + p_.half) / kFarBin), bj = static_cast<int>((z + p_.half) / kFarBin);
    if (bi < 0 || bj < 0 || bi >= farBins_ || bj >= farBins_) return best;
    const Vec2 q(x, z);
    for (int s : farBin_[static_cast<std::size_t>(bj) * farBins_ + bi]) {
        const Seg& g = segs_[static_cast<std::size_t>(s)];
        if (g.shelf) continue;   // under the sea: not a river bank
        const Vec2 ab = g.b - g.a;
        const double L2 = dot(ab, ab);
        const double t = L2 > 1e-12 ? clampd(dot(q - g.a, ab) / L2, 0.0, 1.0) : 0.0;
        const double d = (q - (g.a + ab * t)).length() - (g.wa + (g.wb - g.wa) * t) * 0.5;
        if (d < best) {
            best = d;
            if (level) *level = g.la + (g.lb - g.la) * t;
        }
    }
    return best;
}

std::vector<std::vector<Vec2>> Hydrology::outlineRings(double margin, std::vector<Vec2>* interior) const {
    namespace L = roads::lanes;
    // 1. THE OUTLINE: every river's corridor (a quad per segment and a disc at each node, on a
    //    path resampled to its width) and every lake's cells (dilated one cell under its shore),
    //    unioned into one polygon set. Where a river meets another, a lake or the sea's edge the
    //    shapes merge; nothing can fold or overlap.
    std::vector<L::Ring> rings;
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
            const double hw = path[k].width * 0.5 * 1.05 + margin;
            disc(path[k].p, hw);
            if (interior) interior->push_back(path[k].p);
            if (k + 1 == path.size()) continue;
            const Vec2 a = path[k].p, b = path[k + 1].p, d = b - a;
            const double len = d.length();
            if (len < 1e-6) continue;
            const Vec2 n = perp(d / len);
            const double hb = path[k + 1].width * 0.5 * 1.05 + margin;
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
        if (interior) for (int c : lk.cells) interior->push_back(cellCenter(c));   // the lake's open water: away from the bank
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
    return rings;
}

std::vector<std::vector<Vec2>> Hydrology::corridorRings(double margin) const {
    namespace L = roads::lanes;
    std::vector<std::vector<Vec2>> out;
    const std::vector<L::Ring> rings = outlineRings(margin, nullptr);
    if (rings.empty()) return out;
    for (const L::Polygon2& poly : L::unionRings(rings)) {
        out.push_back(poly.outer);
        for (const L::Ring& h : poly.holes) out.push_back(h);
    }
    return out;
}

RenderMesh Hydrology::quayMesh(const std::function<bool(double, double)>& where,
                               const std::function<double(double, double)>& ground, double parapet) const {
    RenderMesh out;
    // the wall stands just outside the water's outline (the water mesh reaches 1.05 x the half width)
    for (const std::vector<Vec2>& ring : corridorRings(0.3)) {
        if (ring.size() < 3) continue;
        // resample the ring every <= 2.5 m
        std::vector<Vec2> pts;
        for (std::size_t i = 0; i < ring.size(); ++i) {
            const Vec2 a = ring[i], b = ring[(i + 1) % ring.size()];
            const int k = std::max(1, static_cast<int>(std::ceil((b - a).length() / 2.5)));
            for (int m = 0; m < k; ++m) pts.push_back(a + (b - a) * (static_cast<double>(m) / k));
        }
        const std::size_t n = pts.size();
        // which samples carry a wall: where the caller says, and beside a RIVER (not a lake shore)
        std::vector<char> on(n, 0);
        std::vector<double> lvl(n, 0.0), top(n, 0.0), bank(n, 0.0);
        std::vector<Vec2> outw(n, Vec2(0, 0));
        for (std::size_t i = 0; i < n; ++i) {
            double level = 0.0;
            const double d = distanceToRiver(pts[i].x, pts[i].y, 20.0, &level);
            if (d > 3.0 || !std::isfinite(level) || !where(pts[i].x, pts[i].y)) continue;
            // the bank behind the wall: a metre and a half out from the water
            const Vec2 prev = pts[(i + n - 1) % n], next = pts[(i + 1) % n];
            Vec2 t = next - prev;
            t = t.length() > 1e-9 ? t / t.length() : Vec2(1, 0);
            const Vec2 outward = Vec2(t.y, -t.x);   // the ring's right: away from the water for a CCW outer ring
            const Vec2 q = pts[i] + outward * 1.5;
            on[i] = 1;
            lvl[i] = level;
            top[i] = std::max(ground(q.x, q.y), level + 0.8) + parapet;
            outw[i] = outward;
            bank[i] = ground(q.x, q.y);
        }
        // strips over runs of wall samples
        double u = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            if (!on[i] || !on[j]) continue;
            const Vec2 a = pts[i], b = pts[j];
            const double len = (b - a).length();
            if (len < 1e-6) continue;
            const Vec2 t = (b - a) / len;
            const Vec3 nrm(-t.y, 0.0, t.x);   // toward the water (left of a CCW outer ring)
            const uint32_t base = static_cast<uint32_t>(out.vertices.size());
            auto put = [&](const Vec2& p, double y, double uu, double vv) {
                Vertex v(Vec3(p.x, y, p.y), nrm, Vec3(t.x, 0.0, t.y), static_cast<float>(uu), static_cast<float>(vv));
                v.color = Vec3(1, 1, 1);
                out.vertices.push_back(v);
            };
            put(a, lvl[i] - 1.2, u, 0.0);
            put(b, lvl[j] - 1.2, u + len, 0.0);
            put(b, top[j], u + len, top[j] - lvl[j] + 1.2);
            put(a, top[i], u, top[i] - lvl[i] + 1.2);
            out.indices.insert(out.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
            // THICKNESS (#57, Glenn: "the wall has no thickness"): a coping on top, a back face down into the
            // bank, and a cap where a run of wall ends -- each sample's back edge offset along its own outward
            // direction, so neighbouring segments share it and corners stay closed.
            constexpr double kThick = 0.45;
            const Vec2 ab = a + outw[i] * kThick, bb = b + outw[j] * kThick;
            auto quad = [&](const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, const Vec3& n, double w, double h) {
                const uint32_t q0 = static_cast<uint32_t>(out.vertices.size());
                const Vec3 tt = (p1 - p0).length() > 1e-9 ? (p1 - p0) * (1.0 / (p1 - p0).length()) : Vec3(1, 0, 0);
                const Vec3 ps[4] = {p0, p1, p2, p3};
                const double uv[4][2] = {{u, 0}, {u + w, 0}, {u + w, h}, {u, h}};
                for (int c = 0; c < 4; ++c) {
                    Vertex v(ps[c], n, tt, static_cast<float>(uv[c][0]), static_cast<float>(uv[c][1]));
                    v.color = Vec3(1, 1, 1);
                    out.vertices.push_back(v);
                }
                out.indices.insert(out.indices.end(), {q0, q0 + 1, q0 + 2, q0, q0 + 2, q0 + 3});
            };
            // coping: along the front top edge, then back across the wall -- a, b, bb, ab winds it facing up.
            // Emitted as its two triangles: at a sharp inside bend the two samples' offsets cross and a triangle
            // would face DOWN (a fold under 0.03 m^2); that one is left out rather than drawn inside out.
            {
                const Vec3 P[4] = {Vec3(a.x, top[i], a.y), Vec3(b.x, top[j], b.y), Vec3(bb.x, top[j], bb.y), Vec3(ab.x, top[i], ab.y)};
                const double UV[4][2] = {{u, 0}, {u + len, 0}, {u + len, kThick}, {u, kThick}};
                const Vec3 tt(t.x, 0.0, t.y);
                const int tris[2][3] = {{0, 1, 2}, {0, 2, 3}};
                for (const auto& tri : tris) {
                    const Vec3 w = cross(P[tri[1]] - P[tri[0]], P[tri[2]] - P[tri[0]]);
                    if (w.y <= 0.0) continue;   // folded: facing down
                    const uint32_t q0 = static_cast<uint32_t>(out.vertices.size());
                    for (int c : tri) {
                        Vertex v(P[c], Vec3(0, 1, 0), tt, static_cast<float>(UV[c][0]), static_cast<float>(UV[c][1]));
                        v.color = Vec3(1, 1, 1);
                        out.vertices.push_back(v);
                    }
                    out.indices.insert(out.indices.end(), {q0, q0 + 1, q0 + 2});
                }
            }
            // back face: facing the bank, from the top down to below the bank's ground
            const Vec3 nb(t.y, 0.0, -t.x);
            const double footI = std::min(bank[i], top[i]) - 0.3, footJ = std::min(bank[j], top[j]) - 0.3;
            quad(Vec3(bb.x, footJ, bb.y), Vec3(ab.x, footI, ab.y), Vec3(ab.x, top[i], ab.y), Vec3(bb.x, top[j], bb.y),
                 nb, len, top[i] - footI);
            // an end cap where the run stops (the next sample carries no wall), and where it starts
            auto cap = [&](const Vec2& f, const Vec2& k, double yb, double yt, const Vec3& n) {
                quad(Vec3(f.x, yb, f.y), Vec3(k.x, yb, k.y), Vec3(k.x, yt, k.y), Vec3(f.x, yt, f.y), n, kThick, yt - yb);
            };
            if (!on[(j + 1) % n]) cap(b, bb, lvl[j] - 1.2, top[j], Vec3(t.x, 0.0, t.y));
            if (!on[(i + n - 1) % n]) cap(ab, a, lvl[i] - 1.2, top[i], Vec3(-t.x, 0.0, -t.y));
            u += len;
        }
    }
    return out;
}

RenderMesh Hydrology::waterMesh(const std::vector<std::vector<Vec2>>& sea,
                                const std::function<double(double, double)>& ground) const {
    namespace L = roads::lanes;
    RenderMesh out;
    std::vector<Vec2> interior;   // river centre points: the triangulation's inner vertices
    const std::vector<L::Ring> rings = outlineRings(0.0, &interior);
    const int n = n_;
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
        // the nearest lake cell within three cells (the rounded outline strays past the lake's own cells)
        int lake = -1;
        double dLake = 1e30;
        for (int b = gj - 3; b <= gj + 3; ++b)
            for (int a = gi - 3; a <= gi + 3; ++a) {
                if (a < 0 || b < 0 || a >= n || b >= n) continue;
                const int l = lakeOfCell_[static_cast<std::size_t>(b) * n + a];
                if (l < 0) continue;
                const double d = (Vec2(-p_.half + a * p_.cell, -p_.half + b * p_.cell) - q).length();
                if (d < dLake) { dLake = d; lake = l; }
            }
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
        // No river in the bins and no lake near: every segment. It used to fall back to the SEA's level
        // here, and a mountain lake's outline vertices dropped 500 m: the spikes under Glenn's lakes.
        if (!bestR && lake < 0)
            for (const River& r : rivers_)
                for (std::size_t k = 0; k + 1 < r.nodes.size(); ++k) {
                    const Vec2 a = r.nodes[k].p, ab = r.nodes[k + 1].p - a;
                    const double L2 = dot(ab, ab);
                    const double t = L2 > 1e-12 ? clampd(dot(q - a, ab) / L2, 0.0, 1.0) : 0.0;
                    const double d = (q - (a + ab * t)).length();
                    if (d < best) { best = d; bestT = t; bestR = &r; bestK = k; }
                }
        // a lake beside the vertex wins (a river's surface is at the lake's level where it meets it);
        // farther off, whichever water is nearer
        if (lake >= 0 && (dLake <= 1.5 * p_.cell || !bestR || dLake <= best)) {
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
        // depth / 8 m, clamped: colour stays in [0, 1], so the mesh keeps the packed vertex (ADR-0096)
        const double depth = ground ? std::max(0.0, level - ground(q.x, q.y)) : 2.0;
        v.color = Vec3(speed, fade, std::min(1.0, depth / 8.0));
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
