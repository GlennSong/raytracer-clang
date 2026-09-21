#include "engine/procgen/city/roads/lanes/terrain_conform.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace engine {
namespace roads::lanes {

void conformGrid(const RoadLabGraph& g, const LaneSet& L, const DeckHeight& H, HeightGrid& grid, ConformStats& stats, const std::vector<Surface>& decks) {
    const Rules& R = g.rules; double W = R.conformW; size_t nl = L.lanes.size();
    // owners: lanes first (by rank desc), then road envelopes (rank - 0.1)
    struct Owner { bool lane; int index; double rank; double halfWidth; std::unique_ptr<SegmentGrid> grid; double x0, x1, y0, y1; };
    std::vector<Owner> owners;
    auto box = [](const std::vector<Vec2>& xy, Owner& o) { o.x0 = o.y0 = 1e300; o.x1 = o.y1 = -1e300; for (const Vec2& p : xy) { o.x0 = std::min(o.x0, p.x); o.x1 = std::max(o.x1, p.x); o.y0 = std::min(o.y0, p.y); o.y1 = std::max(o.y1, p.y); } };
    for (size_t li = 0; li < nl; ++li) { owners.push_back({true, static_cast<int>(li), L.rank(static_cast<int>(li), g), L.lanes[li].w / 2, nullptr, 0, 0, 0, 0}); box(L.lanes[li].xy, owners.back()); }
    for (size_t ei = 0; ei < g.edges.size(); ++ei) {
        const EdgeSpec& e = g.edges[ei]; if (e.laneCount() == 0) continue; const RoadClassSpec& c = g.cls(e);
        owners.push_back({false, static_cast<int>(ei), c.rank - 0.1, g.hw(e) + c.shoulder + c.sidewalk, std::make_unique<SegmentGrid>(e.xy, 8.0), 0, 0, 0, 0}); box(e.xy, owners.back());
    }
    std::stable_sort(owners.begin(), owners.end(), [](const Owner& a, const Owner& b) { return a.rank > b.rank; });
    size_t N = grid.z.size(); std::vector<double> T0 = grid.z; std::vector<int> owner(N, -1); std::vector<double> odist(N, std::numeric_limits<double>::infinity());
    std::vector<double> ownZ(N, 0.0); std::vector<char> ownerIsLane(N, 0);
    for (size_t oi = 0; oi < owners.size(); ++oi) {
        const Owner& o = owners[oi]; double reach = W + o.halfWidth + 1.0;
        int i0 = std::max(0, static_cast<int>((o.x0 - reach - grid.x0) / grid.res)), i1 = std::min(grid.nx - 1, static_cast<int>((o.x1 + reach - grid.x0) / grid.res) + 1);
        int j0 = std::max(0, static_cast<int>((o.y0 - reach - grid.y0) / grid.res)), j1 = std::min(grid.ny - 1, static_cast<int>((o.y1 + reach - grid.y0) / grid.res) + 1);
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
            size_t n = static_cast<size_t>(j) * grid.nx + i; Vec2 p(grid.x0 + i * grid.res, grid.y0 + j * grid.res);
            double d = o.lane ? H.distanceToLane(o.index, p, W) : std::max(0.0, o.grid->distanceWithin(p, W + o.halfWidth) - o.halfWidth);
            if (!(d < W)) continue;
            if (d > odist[n] || (d == odist[n] && !(o.lane && !ownerIsLane[n]))) continue;   // nearer wins; on a tie a lane (the built deck) beats an envelope (its sidewalk)
            double z = o.lane ? H.own(o.index, p) : H.ownRoad(o.index, p);
            if (z - T0[n] > R.bridgeH) {                                                // looks like structure: confirm with the BUILT deck (a connector's raw height is a straight line between its ends)
                z = o.lane ? H.deck(o.index, p) : H.deckRoad(o.index, p);
                if (z - T0[n] > R.bridgeH) continue;
            }
            owner[n] = static_cast<int>(oi); odist[n] = d; ownZ[n] = z; ownerIsLane[n] = o.lane ? 1 : 0;
        }
    }
    for (size_t oi = 0; oi < owners.size(); ++oi) {
        const Owner& o = owners[oi];
        for (size_t n = 0; n < N; ++n) {
            if (owner[n] != static_cast<int>(oi)) continue;
            Vec2 p(grid.x0 + static_cast<double>(n % static_cast<size_t>(grid.nx)) * grid.res, grid.y0 + static_cast<double>(n / static_cast<size_t>(grid.nx)) * grid.res);
            double r = o.lane ? H.deck(o.index, p) : H.deckRoad(o.index, p), t0 = T0[n], dn = odist[n];
            double skirt = std::max(R.skirt, 1.25 * grid.res);            // a heightfield cannot hold a sharper edge than its own cell
            bool inside = dn <= skirt; double dnOut = std::max(dn - skirt, 0.0);
            double tgt = inside ? r - R.skirtDrop : (r > t0 ? r - R.slope * dnOut : r + R.slope * dnOut);
            grid.z[n] = r > t0 ? std::max(t0, tgt) : std::min(t0, tgt);
        }
    }
    // DAYLIGHT THE CUT (2026-09-08). The loop above only touches nodes an owner reached (conformW + half
    // width, ~17 m). Where a road is cut into a hillside the cells beyond that keep their natural height, so
    // between two cut bands — two carriageways at different levels, a ramp beside its host — the untouched
    // cells stand as spikes right at the kerb ("a lot of deformed terrain interfering with the road", Glenn,
    // 2026-09-08: natural 52 m peaks left between bands cut to 46 and 41). A real earthwork runs its batter
    // out until it meets the ground: the daylight line. Propagate the batter cone outward from every
    // conformed node and clamp the natural surface under it — a node may not stand higher than the nearest
    // conformed node plus slope x distance. Self-limiting: far from any road the cone passes over the terrain
    // and nothing changes. CUT ONLY: the same cone downward would raise the ground under a viaduct into an
    // embankment, and a structure on piers is not an embankment.
    {
        const double step = R.slope * grid.res, diag = step * std::sqrt(2.0);
        const double kInf = std::numeric_limits<double>::infinity();
        std::vector<double> ceil(N, kInf);
        for (size_t n = 0; n < N; ++n) if (owner[n] >= 0) ceil[n] = grid.z[n];
        const int nx = grid.nx, ny = grid.ny;
        auto pull = [&](int i, int j, int di, int dj, double d) {
            const size_t a = static_cast<size_t>(j) * nx + i, b = static_cast<size_t>(j + dj) * nx + (i + di);
            if (ceil[b] + d < ceil[a]) ceil[a] = ceil[b] + d;
        };
        for (int sweep = 0; sweep < 2; ++sweep) {   // two chamfer sweeps: forward then backward, 8-neighbour
            for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) {
                if (i > 0) pull(i, j, -1, 0, step);
                if (j > 0) pull(i, j, 0, -1, step);
                if (i > 0 && j > 0) pull(i, j, -1, -1, diag);
                if (i + 1 < nx && j > 0) pull(i, j, +1, -1, diag);
            }
            for (int j = ny - 1; j >= 0; --j) for (int i = nx - 1; i >= 0; --i) {
                if (i + 1 < nx) pull(i, j, +1, 0, step);
                if (j + 1 < ny) pull(i, j, 0, +1, step);
                if (i + 1 < nx && j + 1 < ny) pull(i, j, +1, +1, diag);
                if (i > 0 && j + 1 < ny) pull(i, j, -1, +1, diag);
            }
        }
        for (size_t n = 0; n < N; ++n) if (owner[n] < 0 && ceil[n] < kInf) grid.z[n] = std::min(grid.z[n], ceil[n]);
    }
    // NO TERRAIN ABOVE A DECK (2026-09-08). A node belongs to its NEAREST road, so where two roads run close
    // at different levels — the two carriageways of a ring on a cross slope, a ramp beside its host — nodes
    // owned by the higher one stand above the lower one's deck, at its very kerb. That is the ground a driver
    // hits coming off a ramp, and the wedges of grass that push up through the carriageway.
    // The reach is the ROAD's envelope (half width + shoulder), not a lane's: clamping per lane left the
    // median between two carriageways unclamped — it sits ~4.8 m from the nearest lane centreline, past a
    // lane's own reach — so a ridge of natural ground stood between them and cut the deck (Glenn, 2026-09-08).
    // A road carried on structure is exempt, as in the ownership rule: the ground under a viaduct is not its
    // business.
    for (size_t ei = 0; ei < g.edges.size(); ++ei) {
        const EdgeSpec& e = g.edges[ei]; if (e.laneCount() == 0 || e.xy.size() < 2) continue;
        const RoadClassSpec& c = g.cls(e);
        const double reach = g.hw(e) + c.shoulder + 1.0;
        double bx0 = 1e300, bx1 = -1e300, by0 = 1e300, by1 = -1e300;
        for (const Vec2& q : e.xy) { bx0 = std::min(bx0, q.x); bx1 = std::max(bx1, q.x); by0 = std::min(by0, q.y); by1 = std::max(by1, q.y); }
        const int i0 = std::max(0, static_cast<int>((bx0 - reach - grid.x0) / grid.res)), i1 = std::min(grid.nx - 1, static_cast<int>((bx1 + reach - grid.x0) / grid.res) + 1);
        const int j0 = std::max(0, static_cast<int>((by0 - reach - grid.y0) / grid.res)), j1 = std::min(grid.ny - 1, static_cast<int>((by1 + reach - grid.y0) / grid.res) + 1);
        SegmentGrid sg(e.xy, 8.0);
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
            const size_t n = static_cast<size_t>(j) * grid.nx + i; const Vec2 p(grid.x0 + i * grid.res, grid.y0 + j * grid.res);
            if (!(sg.distanceWithin(p, reach) < reach)) continue;
            const double deck = H.deckRoad(static_cast<int>(ei), p);
            if (deck - T0[n] > R.bridgeH) continue;   // on structure: leave the ground under it alone
            grid.z[n] = std::min(grid.z[n], deck - 0.05);
        }
    }
    // and again per LANE, with the lane's own blended deck (a lane pulled by its partner sits below its
    // road's profile, so the envelope pass above can leave ground standing over it).
    for (size_t li = 0; li < nl; ++li) {
        const Lane& l = L.lanes[li]; if (l.xy.size() < 2) continue;
        // Reach past the lane far enough to cover its SHOULDER: the shoulder slab sits at the lane's own
        // (blended) deck, and terrain out there was only ever clamped to the road's profile, which on a pulled
        // lane sits higher — so grass spiked up through the hard shoulder (Glenn, 2026-09-08).
        const double sh = l.parent >= 0 ? g.cls(g.edges[static_cast<size_t>(l.parent)]).shoulder : 0.0;
        const double reach = l.w / 2 + std::max(2.5, sh + 1.0);
        double bx0 = 1e300, bx1 = -1e300, by0 = 1e300, by1 = -1e300;
        for (const Vec2& q : l.xy) { bx0 = std::min(bx0, q.x); bx1 = std::max(bx1, q.x); by0 = std::min(by0, q.y); by1 = std::max(by1, q.y); }
        const int i0 = std::max(0, static_cast<int>((bx0 - reach - grid.x0) / grid.res)), i1 = std::min(grid.nx - 1, static_cast<int>((bx1 + reach - grid.x0) / grid.res) + 1);
        const int j0 = std::max(0, static_cast<int>((by0 - reach - grid.y0) / grid.res)), j1 = std::min(grid.ny - 1, static_cast<int>((by1 + reach - grid.y0) / grid.res) + 1);
        for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
            const size_t n = static_cast<size_t>(j) * grid.nx + i; const Vec2 p(grid.x0 + i * grid.res, grid.y0 + j * grid.res);
            if (!(H.distanceToLane(static_cast<int>(li), p, reach) < reach)) continue;
            const double deck = H.deck(static_cast<int>(li), p);
            if (deck - T0[n] > R.bridgeH) continue;
            grid.z[n] = std::min(grid.z[n], deck - 0.05);
        }
    }
    stats.nodeOwner.assign(N, "-"); stats.nodeDist.assign(N, -1);
    for (size_t n = 0; n < N; ++n) if (owner[n] >= 0) { const Owner& o = owners[static_cast<size_t>(owner[n])]; stats.nodeOwner[n] = o.lane ? L.lanes[static_cast<size_t>(o.index)].id : g.edges[static_cast<size_t>(o.index)].id + "(envelope)"; stats.nodeDist[n] = odist[n]; }
    // UNDER THE DECK ITSELF (2026-09-21). Everything above reaches out from a CENTRELINE by a
    // road's or a lane's own width. The pavement union is wider than that wherever it closes a
    // junction — the fillets, the gores, the closing pieces — and those are precisely where a
    // driver leaves a ramp or crosses an intersection. On metro 392 of 817465 deck samples
    // still stood under natural ground, the worst by 0.52 m: a wedge of hillside through the
    // carriageway, which is what "you cannot drive the roads" looks like from inside a car.
    // terrainVsDeck measures deck VERTICES against the bilinear grid, so clamp the four nodes
    // that sample each one. A deck on structure is exempt, as in every pass above: the ground
    // under a viaduct is not its business.
    for (const Surface& sf : decks) {
        for (const DeckVertex& v : sf.verts) {
            const double fi = (v.xy.x - grid.x0) / grid.res, fj = (v.xy.y - grid.y0) / grid.res;
            const int i0 = static_cast<int>(std::floor(fi)), j0 = static_cast<int>(std::floor(fj));
            for (int dj = 0; dj <= 1; ++dj) for (int di = 0; di <= 1; ++di) {
                const int i = i0 + di, j = j0 + dj;
                if (i < 0 || j < 0 || i >= grid.nx || j >= grid.ny) continue;
                const size_t n = static_cast<size_t>(j) * grid.nx + i;
                if (v.z - T0[n] > R.bridgeH) continue;             // on structure
                grid.z[n] = std::min(grid.z[n], v.z - 0.05);
            }
        }
    }
    stats.cutM3 = stats.fillM3 = 0; double cell = grid.cellArea();
    for (size_t n = 0; n < N; ++n) { double dz = grid.z[n] - T0[n]; if (dz < 0) stats.cutM3 -= dz * cell; else stats.fillM3 += dz * cell; }
}

void terrainVsDeck(const std::vector<Surface>& decks, const HeightGrid& grid, ConformStats& stats) {
    stats.samples = stats.above = 0; stats.maxExcess = -1e300; stats.worst.clear();
    auto sample = [&](const Vec2& p, double z, int owner) {
        if (!grid.inside(p.x, p.y)) return; double ex = grid.sample(p.x, p.y) - z; ++stats.samples; stats.maxExcess = std::max(stats.maxExcess, ex);
        if (ex > 0.01) {
            ++stats.above; int i = static_cast<int>(std::lround((p.x - grid.x0) / grid.res)), j = static_cast<int>(std::lround((p.y - grid.y0) / grid.res)); size_t n = static_cast<size_t>(j) * grid.nx + i;
            stats.worst.push_back({p, ex, owner, n < stats.nodeOwner.size() ? stats.nodeOwner[n] : "?", n < stats.nodeDist.size() ? stats.nodeDist[n] : -1});
        }
    };
    for (const Surface& s : decks) {
        std::vector<int> vertOwner(s.verts.size(), -1);
        for (size_t ti = 0; ti < s.tris.size(); ++ti) for (int k : s.tris[ti]) vertOwner[static_cast<size_t>(k)] = s.triOwner[ti];
        for (size_t vi = 0; vi < s.verts.size(); ++vi) sample(s.verts[vi].xy, s.verts[vi].z, vertOwner[vi]);
        for (size_t ti = 0; ti < s.tris.size(); ++ti) { const auto& t = s.tris[ti]; Vec2 c = (s.verts[static_cast<size_t>(t[0])].xy + s.verts[static_cast<size_t>(t[1])].xy + s.verts[static_cast<size_t>(t[2])].xy) / 3.0; double z = (s.verts[static_cast<size_t>(t[0])].z + s.verts[static_cast<size_t>(t[1])].z + s.verts[static_cast<size_t>(t[2])].z) / 3.0; sample(c, z, s.triOwner[ti]); }
    }
    std::sort(stats.worst.begin(), stats.worst.end(), [](const ExcessSample& a, const ExcessSample& b) { return a.excess > b.excess; });
    if (stats.worst.size() > 5) stats.worst.resize(5);
}

}  // namespace roads::lanes
}  // namespace engine
