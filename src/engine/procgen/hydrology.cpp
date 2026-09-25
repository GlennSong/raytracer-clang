#include "hydrology.h"
#include "../mesh_builder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
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
                r.nodes[k].width *= 1.0 + 2.0 * t * t;
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

RenderMesh Hydrology::riverMesh() const {
    RenderMesh out;
    for (const River& r : rivers_) {
        std::vector<Vec3> pts, cols;
        std::vector<double> hw;
        for (std::size_t k = 0; k < r.nodes.size(); ++k) {
            const RiverNode& nd = r.nodes[k];
            pts.push_back(Vec3(nd.p.x, nd.level, nd.p.y));
            hw.push_back(nd.width * 0.5 * 1.1);   // a touch wider: the edges tuck under the banks
            // flow speed from the surface's fall over a few nodes
            const std::size_t a = k >= 2 ? k - 2 : 0, b = std::min(r.nodes.size() - 1, k + 2);
            const double run = std::max(0.5, (r.nodes[b].p - r.nodes[a].p).length());
            const double speed = clampd((r.nodes[a].level - r.nodes[b].level) / run * 6.0, 0.0, 1.0);
            cols.push_back(Vec3(speed, nd.width / 70.0, nd.fade));
        }
        MeshBuilder::append(out, MeshBuilder::ribbon(pts, hw, Vec3(0, 1, 0), cols));
    }
    return out;
}

RenderMesh Hydrology::lakeMesh() const {
    RenderMesh out;
    const int n = n_;
    for (const Lake& L : lakes_) {
        // the lake's cells, dilated by one so the sheet's edge tucks under the shore
        std::vector<char> in(static_cast<std::size_t>(n) * n, 0);
        for (int c : L.cells) {
            const int i = c % n, j = c / n;
            for (int b = j - 1; b <= j + 1; ++b)
                for (int a = i - 1; a <= i + 1; ++a)
                    if (a >= 0 && b >= 0 && a < n && b < n) in[static_cast<std::size_t>(b) * n + a] = 1;
        }
        std::map<int, uint32_t> corner;   // grid corner -> vertex
        auto vtx = [&](int ci, int cj) {
            const int key = cj * (n + 1) + ci;
            auto it = corner.find(key);
            if (it != corner.end()) return it->second;
            const uint32_t id = MeshBuilder::vertex(out, Vec3(-p_.half + (ci - 0.5) * p_.cell, L.level, -p_.half + (cj - 0.5) * p_.cell),
                                                    Vec3(0, 1, 0), Vec3(0, 0, 0));   // speed 0: still water
            out.vertices[id].u = 0.5f;   // mid-river: no bank foam
            out.vertices[id].v = 0.0f;
            corner[key] = id;
            return id;
        };
        for (int j = 0; j < n; ++j)
            for (int i = 0; i < n; ++i) {
                if (!in[static_cast<std::size_t>(j) * n + i]) continue;
                const uint32_t a = vtx(i, j), b = vtx(i + 1, j), c = vtx(i, j + 1), d = vtx(i + 1, j + 1);
                MeshBuilder::triFacing(out, a, b, d, Vec3(0, 1, 0));
                MeshBuilder::triFacing(out, a, d, c, Vec3(0, 1, 0));
            }
    }
    return out;
}

}  // namespace engine
