#include "engine/procgen/lanelab/pavement.h"
#include "engine/procgen/lanelab/lanelab.h"
#include "engine/procgen/lanelab/polyline_ops.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <cmath>
#include <numeric>
#include <set>
#include <unordered_map>

namespace engine {
namespace lanelab {

namespace {

constexpr double kMaxEdge = 4.0;   // faces are densified to this before triangulation

Ring railRing(const std::vector<Vec2>& left, const std::vector<Vec2>& right) {
    Ring r(left.begin(), left.end()); for (size_t i = right.size(); i-- > 0;) r.push_back(right[i]); return r;
}

Ring spineRing(const std::vector<Vec2>& xy, double d) {
    std::vector<Vec2> tan, nrm; frames(xy, tan, nrm); Ring r; r.reserve(xy.size() * 2);
    for (size_t i = 0; i < xy.size(); ++i) r.push_back(xy[i] + nrm[i] * d);
    for (size_t i = xy.size(); i-- > 0;) r.push_back(xy[i] - nrm[i] * d);
    return r;
}

PolySet unionAll(const std::vector<PolySet>& sets) {
    std::vector<Ring> rings; PolySet holes;
    for (const PolySet& s : sets) for (const Polygon2& p : s) { rings.push_back(p.outer); }
    PolySet u = unionRings(rings);
    // footprints have no holes of their own; if a caller passes some, subtract them
    for (const PolySet& s : sets) for (const Polygon2& p : s) for (const Ring& h : p.holes) { Polygon2 hp; hp.outer = h; std::reverse(hp.outer.begin(), hp.outer.end()); holes.push_back(hp); }
    return holes.empty() ? u : differenceSets(u, holes);
}

Vec2 centroidOf(const Ring& r) {
    double a = 0, cx = 0, cy = 0;
    for (size_t i = 0, n = r.size(); i < n; ++i) { const Vec2& p = r[i]; const Vec2& q = r[(i + 1) % n]; double c = p.x * q.y - q.x * p.y; a += c; cx += (p.x + q.x) * c; cy += (p.y + q.y) * c; }
    if (std::fabs(a) < 1e-12) return r.empty() ? Vec2() : r[0];
    return {cx / (3 * a), cy / (3 * a)};
}

double polyArea(const Polygon2& p) { double a = std::fabs(ringArea(p.outer)); for (const Ring& h : p.holes) a -= std::fabs(ringArea(h)); return a; }

void densify(const Ring& r, std::vector<Vec2>& pts, std::vector<std::pair<int, int>>& edges) {
    int start = static_cast<int>(pts.size()); size_t n = r.size();
    for (size_t i = 0; i < n; ++i) {
        const Vec2& a = r[i]; const Vec2& b = r[(i + 1) % n]; double L = distance(a, b); int k = std::max(1, static_cast<int>(std::ceil(L / kMaxEdge)));
        for (int m = 0; m < k; ++m) pts.push_back(lerp(a, b, static_cast<double>(m) / k));
    }
    int end = static_cast<int>(pts.size());
    for (int i = start; i < end; ++i) edges.emplace_back(i, i + 1 < end ? i + 1 : start);
}

struct WeldPool {
    struct Key { long long x, y; bool operator==(const Key& o) const { return x == o.x && y == o.y; } };
    struct Hash { size_t operator()(const Key& k) const { return std::hash<long long>()(k.x * 73856093LL ^ k.y * 19349663LL); } };
    std::unordered_map<Key, std::vector<std::pair<double, int>>, Hash> map;
    std::vector<DeckVertex> verts;
    int add(const Vec2& p, double z) {
        Key k{std::llround(p.x * 10000), std::llround(p.y * 10000)}; auto& slot = map[k];
        for (const auto& e : slot) if (std::fabs(e.first - z) < 1e-3) return e.second;
        int id = static_cast<int>(verts.size()); verts.push_back({p, z}); slot.emplace_back(z, id); return id;
    }
};

struct DSU {
    std::vector<int> p; explicit DSU(size_t n) : p(n) { std::iota(p.begin(), p.end(), 0); }
    int find(int a) { while (p[static_cast<size_t>(a)] != a) { p[static_cast<size_t>(a)] = p[static_cast<size_t>(p[static_cast<size_t>(a)])]; a = p[static_cast<size_t>(a)]; } return a; }
    void unite(int a, int b) { p[static_cast<size_t>(find(a))] = find(b); }
};

FlatMesh triangulatePolygon(const Polygon2& poly) {
    std::vector<Vec2> pts; std::vector<std::pair<int, int>> edges; densify(poly.outer, pts, edges); for (const Ring& h : poly.holes) densify(h, pts, edges);
    Triangulation t = constrainedTriangulation(pts, edges); FlatMesh m; m.verts.reserve(t.verts.size());
    for (const Vec2& v : t.verts) m.verts.push_back({v, 0.0});
    for (const auto& tri : t.tris) { Vec2 c = (t.verts[tri[0]] + t.verts[tri[1]] + t.verts[tri[2]]) / 3.0; if (contains(poly, c)) m.tris.push_back(tri); }
    std::map<std::pair<int, int>, int> count;
    for (const auto& tri : m.tris) for (int k = 0; k < 3; ++k) { int a = tri[k], b = tri[(k + 1) % 3]; count[{std::min(a, b), std::max(a, b)}]++; }
    for (const auto& kv : count) if (kv.second == 1) m.boundary.push_back(kv.first);
    return m;
}

}  // namespace

void buildFootprints(const RoadLabGraph& g, const LaneSet& L, const DeckHeight& H, Pavement& out) {
    out.footprints.clear(); out.footprints.reserve(L.lanes.size());
    for (const Lane& l : L.lanes) out.footprints.push_back(offsetSet(fromRing(railRing(l.left, l.right)), 0.01));   // 1 cm each side: neighbours weld
    double r = g.rules.closing; out.closingAdded = 0;
    if (r > 0) {
        PolySet U = unionAll(out.footprints); PolySet extra = differenceSets(closing(U, r), U);
        std::vector<Box2> boxes; for (const PolySet& fp : out.footprints) boxes.push_back(fp.empty() ? Box2{} : bounds(fp[0]));
        for (const Polygon2& piece : extra) {
            double A = polyArea(piece); if (A < 0.5 || A > 4 * r * r) continue;         // a fillet is a few r²; anything bigger is a median or a block
            PolySet pieceSet{piece}; PolySet grown = offsetSet(pieceSet, 0.05); Box2 pb = bounds(piece); std::vector<int> touching;
            for (size_t li = 0; li < out.footprints.size(); ++li) {
                const Box2& b = boxes[li]; if (b.maxX < pb.minX - 0.2 || b.minX > pb.maxX + 0.2 || b.maxY < pb.minY - 0.2 || b.minY > pb.maxY + 0.2) continue;
                if (setArea(intersectSets(grown, out.footprints[li])) > 1e-6) touching.push_back(static_cast<int>(li));
            }
            if (touching.size() < 2) continue;
            Vec2 c = centroidOf(piece.outer); bool keep = false; PolySet near = offsetSet(pieceSet, 1.0);
            for (size_t i = 0; i < touching.size() && !keep; ++i) for (size_t j = i + 1; j < touching.size() && !keep; ++j) {
                int a = touching[i], b = touching[j]; if (std::fabs(H.own(a, c) - H.own(b, c)) >= g.rules.sameLevelDz) continue;
                PolySet x = intersectSets(out.footprints[static_cast<size_t>(a)], out.footprints[static_cast<size_t>(b)]);
                if (!x.empty() && setArea(intersectSets(near, x)) > 1e-6) keep = true;
            }
            if (!keep) continue;
            int owner = touching[0]; double bd = 1e300;
            for (int li : touching) { double d = project(L.lanes[static_cast<size_t>(li)].xy, L.lanes[static_cast<size_t>(li)].s, c).distance; if (d < bd) { bd = d; owner = li; } }
            out.footprints[static_cast<size_t>(owner)] = unionSets(out.footprints[static_cast<size_t>(owner)], pieceSet); out.closingAdded += A;
        }
    }
    // layers: the median is what lies inside a road's lane envelope but is not lane; shoulders and sidewalks lie outside it
    out.surface = unionAll(out.footprints);
    std::vector<PolySet> inner, shoulders, sidewalks, medians;
    for (const EdgeSpec& e : g.edges) {
        const RoadClassSpec& c = g.cls(e); if (c.median > 0 && e.laneCount() == 0) medians.push_back(fromRing(spineRing(e.xy, c.median)));
        if (e.laneCount() == 0) continue; double hw = g.hw(e);
        inner.push_back(fromRing(spineRing(e.xy, hw)));
        if (c.shoulder > 0) shoulders.push_back(fromRing(spineRing(e.xy, hw + c.shoulder)));
        if (c.sidewalk > 0) sidewalks.push_back(fromRing(spineRing(e.xy, hw + c.sidewalk)));
    }
    PolySet hull = unionAll(inner); std::vector<PolySet> medAll = medians; medAll.push_back(hull);
    out.median = differenceSets(unionAll(medAll), out.surface);
    // Traffic islands (Glenn: "places where the roads create a skinny opening and closing"): a hole in the
    // asphalt that is THIN — an opening of 4 m radius removes it entirely — and under 2000 m² is a kerbed
    // grass island, not a block. It joins the median layer (kerb + grass slab) and is taken out of the
    // shoulder and sidewalk layers below. Anything an opening leaves standing is land: a block.
    {
        std::vector<Ring> isl;
        // A hole with a BODY is normally land — a small city block — and is left alone. Inside a freeway or ramp
        // corridor there is no land: the wedge between a ramp curving away and the road it left is a GORE, and
        // leaving it as a hole showed bare ground in the middle of the pavement ("missing segments on the
        // freeway", Glenn, 2026-09-07: 3 wedges of ~1700 m² beside the band ramps). A gore is kerbed grass at
        // deck height, like any island.
        auto inRoadCorridor = [&](const Ring& h) {
            Vec2 c; for (const Vec2& q : h) c += q; c = c / static_cast<double>(h.size());
            const EdgeSpec* best = nullptr; double bd = 1e300;
            for (const EdgeSpec& e : g.edges) { if (e.xy.size() < 2) continue; const double d = project(e.xy, e.s, c).distance; if (d < bd) { bd = d; best = &e; } }
            return best && (best->isRamp() || g.cls(*best).rank >= 3);
        };
        for (const Polygon2& sp : out.surface) for (const Ring& h : sp.holes) {
            const double a = std::fabs(ringArea(h)); if (a < 2.0 || a > 2000.0) continue;
            if (setArea(offsetSet(fromRing(h), -4.0)) > 1.0 && !inRoadCorridor(h)) continue;   // has a body and is not in a road corridor: a small block, not an island
            isl.push_back(h);
        }
        if (!isl.empty()) out.median = unionSets(out.median, unionRings(isl));
    }
    out.shoulder = shoulders.empty() ? PolySet{} : differenceSets(differenceSets(differenceSets(unionAll(shoulders), hull), out.surface), out.median);
    out.sidewalk = sidewalks.empty() ? PolySet{} : differenceSets(differenceSets(differenceSets(differenceSets(unionAll(sidewalks), hull), out.surface), out.median), out.shoulder);
    out.islands.clear(); out.enclosedBlocks = 0;
    for (const Polygon2& p : out.surface) for (const Ring& h : p.holes) { double a = std::fabs(ringArea(h)); if (a > 1.0 && a < 2000.0) out.islands.push_back(a); else if (a >= 2000.0) ++out.enclosedBlocks; }
}

namespace {
// Worker count for the mesher: every hardware thread (Glenn's Threadripper: 24), LANELAB_THREADS overrides.
unsigned laneLabThreads(unsigned requested = 0) {
    if (requested > 0) return std::min(requested, 64u);
    if (const char* e = std::getenv("LANELAB_THREADS")) { const int n = std::atoi(e); if (n > 0) return static_cast<unsigned>(std::min(n, 64)); }
    const unsigned hw = std::thread::hardware_concurrency(); return hw == 0 ? 1u : std::min(hw, 64u);
}
// Static, contiguous chunks: chunk t is [t*n/T, (t+1)*n/T). Deterministic partition, so per-chunk results
// concatenated in chunk order reproduce the sequential output exactly.
template <class F> void parallelChunks(size_t n, unsigned threads, F&& fn) {
    if (n == 0) return; const unsigned T = std::max(1u, std::min<unsigned>(threads, static_cast<unsigned>(std::min<size_t>(n, 1u << 16))));
    if (T == 1) { fn(0u, size_t(0), n); return; }
    std::vector<std::thread> pool; pool.reserve(T - 1);
    for (unsigned t = 1; t < T; ++t) pool.emplace_back([&fn, t, n, T]() { fn(t, n * t / T, n * (t + 1) / T); });
    fn(0u, size_t(0), n / T); for (std::thread& th : pool) th.join();
}
}  // namespace


LaneGrid::LaneGrid(const std::vector<Box2>& boxes, const std::vector<PolySet>& fps) {
    double x1 = -1e300, y1 = -1e300; x0 = y0 = 1e300; bool any = false;
    for (size_t i = 0; i < boxes.size(); ++i) { if (fps[i].empty()) continue; any = true; x0 = std::min(x0, boxes[i].minX); y0 = std::min(y0, boxes[i].minY); x1 = std::max(x1, boxes[i].maxX); y1 = std::max(y1, boxes[i].maxY); }
    if (!any) { x0 = y0 = 0; x1 = y1 = 1; }
    cell = std::max(16.0, std::max(x1 - x0, y1 - y0) / 400.0);
    nx = std::max(1, static_cast<int>((x1 - x0) / cell) + 1); ny = std::max(1, static_cast<int>((y1 - y0) / cell) + 1); cells.resize(static_cast<size_t>(nx) * ny);
    for (size_t i = 0; i < boxes.size(); ++i) {
        if (fps[i].empty()) continue; const Box2& b = boxes[i];
        const int i0 = std::clamp(static_cast<int>((b.minX - x0) / cell), 0, nx - 1), i1 = std::clamp(static_cast<int>((b.maxX - x0) / cell), 0, nx - 1);
        const int j0 = std::clamp(static_cast<int>((b.minY - y0) / cell), 0, ny - 1), j1 = std::clamp(static_cast<int>((b.maxY - y0) / cell), 0, ny - 1);
        for (int j = j0; j <= j1; ++j) for (int ii = i0; ii <= i1; ++ii) cells[static_cast<size_t>(j) * nx + ii].push_back(static_cast<int>(i));
    }
}
const std::vector<int>& LaneGrid::at(const Vec2& p) const {
    const int i = std::clamp(static_cast<int>((p.x - x0) / cell), 0, nx - 1), j = std::clamp(static_cast<int>((p.y - y0) / cell), 0, ny - 1);
    return cells[static_cast<size_t>(j) * nx + i];
}
std::vector<Box2> laneBoxes(const std::vector<PolySet>& footprints) {
    std::vector<Box2> boxes(footprints.size()); for (size_t li = 0; li < footprints.size(); ++li) boxes[li] = footprints[li].empty() ? Box2{} : bounds(footprints[li]); return boxes;
}

void buildSurfaces(const RoadLabGraph& g, const LaneSet& L, DeckHeight& H, Pavement& out, unsigned threads, const std::function<bool(double)>* progress) {
    size_t nl = L.lanes.size();
    // one CDT over every lane outline
    std::vector<Vec2> pts; std::vector<std::pair<int, int>> edges;
    for (const PolySet& fp : out.footprints) for (const Polygon2& p : fp) { densify(p.outer, pts, edges); for (const Ring& h : p.holes) densify(h, pts, edges); }
    const auto tCdt0 = std::chrono::steady_clock::now();
    Triangulation T = constrainedTriangulation(pts, edges); out.triangles = 0;
    const double sCdt = std::chrono::duration<double>(std::chrono::steady_clock::now() - tCdt0).count();
    std::vector<Box2> boxes(nl); for (size_t li = 0; li < nl; ++li) boxes[li] = out.footprints[li].empty() ? Box2{} : bounds(out.footprints[li]);
    // pass 1: covering lanes, levels, partners, pair stats. Parallel over triangles: the per-triangle work reads
    // only (contains(), H.own()); each worker fills its own emit list, partner sets and pair stats for a contiguous
    // chunk, and the chunks are concatenated in order, so the result is the sequential one.
    struct TriLevel { int tri; int owner; };
    std::vector<TriLevel> emit; std::vector<std::set<int>> partnerSets(nl); out.pairs.clear();
    auto rankKey = [&](int li) { const Lane& l = L.lanes[static_cast<size_t>(li)]; return std::make_pair(L.rank(li, g), -static_cast<double>(l.parent >= 0 ? l.parent : static_cast<int>(nl) + li)); };   // ROAD-level order: lanes of one road tie
    const unsigned nThreads = laneLabThreads(threads); const LaneGrid grid(boxes, out.footprints);
    // progress: the cover pass is most of this function; workers count triangles, chunk 0 (the calling thread) reports
    std::atomic<size_t> covered{0}; std::atomic<bool> stop{false}; const size_t totalTris = std::max<size_t>(1, T.tris.size());
    auto tick = [&](double f) { if (progress && !(*progress)(std::clamp(f, 0.0, 1.0))) stop.store(true); };
    tick(0.03);   // the CDT is done
    struct Local { std::vector<TriLevel> emit; std::vector<std::set<int>> partners; std::map<std::pair<int, int>, PairStats> pairs; };
    std::vector<Local> locals(nThreads); for (Local& lc : locals) lc.partners.assign(nl, {});
    const auto tCover0 = std::chrono::steady_clock::now();
    parallelChunks(T.tris.size(), nThreads, [&](unsigned t, size_t begin, size_t end) {
        Local& lc = locals[t]; std::vector<std::pair<double, int>> cover; std::vector<std::vector<int>> clusters; std::vector<double> clusterZ;
        size_t sinceTick = 0;
        for (size_t ti = begin; ti < end; ++ti) {
            if (++sinceTick == 4096) { covered.fetch_add(sinceTick, std::memory_order_relaxed); sinceTick = 0; if (stop.load(std::memory_order_relaxed)) return; if (t == 0) tick(0.03 + 0.85 * static_cast<double>(covered.load(std::memory_order_relaxed)) / static_cast<double>(totalTris)); }
            const auto& tri = T.tris[ti]; Vec2 c = (T.verts[tri[0]] + T.verts[tri[1]] + T.verts[tri[2]]) / 3.0;
            double area = 0.5 * std::fabs(cross(T.verts[tri[1]] - T.verts[tri[0]], T.verts[tri[2]] - T.verts[tri[0]])); if (area < 1e-6) continue;
            cover.clear();
            for (int liI : grid.at(c)) {
                const size_t li = static_cast<size_t>(liI); const Box2& b = boxes[li];
                if (c.x < b.minX || c.x > b.maxX || c.y < b.minY || c.y > b.maxY) continue;
                if (!contains(out.footprints[li], c)) continue;
                cover.emplace_back(H.own(liI, c), liI);
            }
            if (cover.empty()) continue;
            std::sort(cover.begin(), cover.end()); clusters.clear(); clusterZ.clear();
            for (const auto& cz : cover) {
                if (!clusters.empty() && cz.first - clusterZ.back() < g.rules.sameLevelDz) { clusters.back().push_back(cz.second); clusterZ.back() = cz.first; }
                else { clusters.push_back({cz.second}); clusterZ.push_back(cz.first); }
            }
            for (const auto& cl : clusters) {
                int owner = cl[0]; for (int m : cl) if (rankKey(m) > rankKey(owner) || (rankKey(m) == rankKey(owner) && m < owner)) owner = m;   // ties within a road: the lowest lane index
                lc.emit.push_back({static_cast<int>(ti), owner});
                for (int m : cl) for (int q : cl) { if (q == m || L.roadOf(q, g) == L.roadOf(m, g)) continue; if (rankKey(q) > rankKey(m)) lc.partners[static_cast<size_t>(m)].insert(q); }
            }
            for (size_t i = 0; i < cover.size(); ++i) for (size_t j = i + 1; j < cover.size(); ++j) {
                int a = cover[i].second, b = cover[j].second; bool same = false;
                for (const auto& cl : clusters) if (std::find(cl.begin(), cl.end(), a) != cl.end() && std::find(cl.begin(), cl.end(), b) != cl.end()) same = true;
                PairStats& ps = lc.pairs[{std::min(a, b), std::max(a, b)}]; (same ? ps.sameLevelArea : ps.separatedArea) += area; const double gap = std::fabs(cover[i].first - cover[j].first); ps.dz = std::max(ps.dz, gap);
                if (!same && gap < ps.minDz) { ps.minDz = gap; ps.minAt = c; }
            }
        }
    });
    if (stop.load()) throw BuildCancelled();
    tick(0.90);
    for (Local& lc : locals) {
        emit.insert(emit.end(), lc.emit.begin(), lc.emit.end());
        for (size_t li = 0; li < nl; ++li) partnerSets[li].insert(lc.partners[li].begin(), lc.partners[li].end());
        for (const auto& kv : lc.pairs) { PairStats& ps = out.pairs[kv.first]; ps.sameLevelArea += kv.second.sameLevelArea; ps.separatedArea += kv.second.separatedArea; ps.dz = std::max(ps.dz, kv.second.dz); if (kv.second.minDz < ps.minDz) { ps.minDz = kv.second.minDz; ps.minAt = kv.second.minAt; } }
    }
    const double sCover = std::chrono::duration<double>(std::chrono::steady_clock::now() - tCover0).count();
    // a road's lanes must share ONE deck field: union the partner sets over each parent road (connectors keep their own)
    std::map<int, std::set<int>> byRoad;
    for (size_t li = 0; li < nl; ++li) if (L.lanes[li].parent >= 0) byRoad[L.lanes[li].parent].insert(partnerSets[li].begin(), partnerSets[li].end());
    std::vector<std::vector<int>> partners(nl);
    for (size_t li = 0; li < nl; ++li) {
        const std::set<int>& src = L.lanes[li].parent >= 0 ? byRoad[L.lanes[li].parent] : partnerSets[li];
        for (int q : src) if (rankKey(q) > rankKey(static_cast<int>(li))) partners[li].push_back(q);   // strictly higher ROAD: acyclic by construction
        std::sort(partners[li].begin(), partners[li].end(), [&](int a, int b) { return rankKey(a) > rankKey(b); });
    }
    out.partners = partners; H.setPartners(partners);
    // pass 2a: the deck height of every distinct (CDT vertex, owner) pair, in parallel — the partner blends are
    // the expensive part; keys are numbered in first-use order along `emit`, so ids below match the sequential run
    const auto tH0 = std::chrono::steady_clock::now();
    std::unordered_map<long long, int> keyIndex; std::vector<std::pair<int, int>> keys; keyIndex.reserve(emit.size() * 2);
    const long long stride = static_cast<long long>(nl) + 1;
    auto keyOf = [&](int v, int owner) -> int {
        const long long k = static_cast<long long>(v) * stride + owner; auto it = keyIndex.find(k);
        if (it != keyIndex.end()) return it->second; const int id = static_cast<int>(keys.size()); keys.emplace_back(v, owner); keyIndex.emplace(k, id); return id;
    };
    std::vector<std::array<int, 3>> emitKeys(emit.size());
    for (size_t i = 0; i < emit.size(); ++i) { const auto& tri = T.tris[static_cast<size_t>(emit[i].tri)]; for (int k = 0; k < 3; ++k) emitKeys[i][static_cast<size_t>(k)] = keyOf(tri[static_cast<size_t>(k)], emit[i].owner); }
    std::vector<double> keyZ(keys.size());
    parallelChunks(keys.size(), nThreads, [&](unsigned, size_t begin, size_t end) { for (size_t i = begin; i < end; ++i) keyZ[i] = H.deck(keys[i].second, T.verts[static_cast<size_t>(keys[i].first)]); });
    const double sHeights = std::chrono::duration<double>(std::chrono::steady_clock::now() - tH0).count();
    // pass 2b: welding and components, sequential in emit order (vertex ids are assigned in first-use order)
    const auto tW0 = std::chrono::steady_clock::now();
    WeldPool pool; std::vector<int> memoId(keys.size(), -1); std::vector<std::array<int, 3>> tris; std::vector<int> owners; std::set<std::array<int, 3>> seen;
    auto vid = [&](int ki) { if (memoId[static_cast<size_t>(ki)] < 0) memoId[static_cast<size_t>(ki)] = pool.add(T.verts[static_cast<size_t>(keys[static_cast<size_t>(ki)].first)], keyZ[static_cast<size_t>(ki)]); return memoId[static_cast<size_t>(ki)]; };
    for (size_t i = 0; i < emit.size(); ++i) {
        const TriLevel& tl = emit[i]; std::array<int, 3> w{vid(emitKeys[i][0]), vid(emitKeys[i][1]), vid(emitKeys[i][2])};
        if (w[0] == w[1] || w[1] == w[2] || w[0] == w[2]) continue;
        std::array<int, 3> key = w; std::sort(key.begin(), key.end()); if (!seen.insert(key).second) continue;
        tris.push_back(w); owners.push_back(tl.owner);
    }
    out.triangles = static_cast<int>(tris.size()); const double sWeld = std::chrono::duration<double>(std::chrono::steady_clock::now() - tW0).count(); const auto tD0 = std::chrono::steady_clock::now();
    DSU dsu(pool.verts.size()); for (const auto& t : tris) { dsu.unite(t[0], t[1]); dsu.unite(t[1], t[2]); }
    std::map<int, int> compIndex; out.decks.clear();
    for (size_t ti = 0; ti < tris.size(); ++ti) {
        int root = dsu.find(tris[ti][0]); auto it = compIndex.find(root);
        if (it == compIndex.end()) { it = compIndex.emplace(root, static_cast<int>(out.decks.size())).first; out.decks.emplace_back(); }
        Surface& s = out.decks[static_cast<size_t>(it->second)]; s.tris.push_back(tris[ti]); s.triOwner.push_back(owners[ti]);
    }
    for (Surface& s : out.decks) {
        std::map<int, int> remap; std::vector<DeckVertex> verts;
        for (auto& t : s.tris) for (int& v : t) { auto it = remap.find(v); if (it == remap.end()) { it = remap.emplace(v, static_cast<int>(verts.size())).first; verts.push_back(pool.verts[static_cast<size_t>(v)]); } v = it->second; }
        s.verts = std::move(verts);
        std::map<std::pair<int, int>, int> count; std::map<std::pair<int, int>, std::pair<int, int>> oriented; std::vector<int> bdeg(s.verts.size(), 0);
        for (const auto& t : s.tris) { for (int k = 0; k < 3; ++k) { int a = t[k], b = t[(k + 1) % 3]; auto key = std::make_pair(std::min(a, b), std::max(a, b)); count[key]++; oriented.emplace(key, std::make_pair(a, b)); }
            s.area += 0.5 * std::fabs(cross(s.verts[static_cast<size_t>(t[1])].xy - s.verts[static_cast<size_t>(t[0])].xy, s.verts[static_cast<size_t>(t[2])].xy - s.verts[static_cast<size_t>(t[0])].xy)); }
        for (const auto& kv : count) { if (kv.second == 1) { s.boundary.push_back(oriented[kv.first]); bdeg[static_cast<size_t>(kv.first.first)]++; bdeg[static_cast<size_t>(kv.first.second)]++; } if (kv.second > 2) ++s.nonManifold; }
        for (int d : bdeg) if (d % 2 == 1) ++s.boundaryOdd;
        std::unordered_map<WeldPool::Key, std::vector<std::pair<double, int>>, WeldPool::Hash> xy;   // z and vertex id
        std::vector<int> vertOwner(s.verts.size(), -1); for (size_t ti = 0; ti < s.tris.size(); ++ti) for (int k : s.tris[ti]) vertOwner[static_cast<size_t>(k)] = s.triOwner[ti];
        for (size_t vi = 0; vi < s.verts.size(); ++vi) xy[{std::llround(s.verts[vi].xy.x * 10000), std::llround(s.verts[vi].xy.y * 10000)}].emplace_back(s.verts[vi].z, static_cast<int>(vi));
        for (auto& kv : xy) {
            auto& zs = kv.second; if (zs.size() < 2) continue; std::sort(zs.begin(), zs.end());
            for (size_t i = 1; i < zs.size(); ++i) if (zs[i].first - zs[i-1].first >= 1e-3 && zs[i].first - zs[i-1].first < g.rules.sameLevelDz) {
                ++s.cracks; Surface::CrackSample cs; cs.xy = s.verts[static_cast<size_t>(zs[i].second)].xy; cs.dz = zs[i].first - zs[i-1].first;
                for (const auto& zv : zs) cs.owners.push_back(vertOwner[static_cast<size_t>(zv.second)]);
                if (s.crackSamples.size() < 400) s.crackSamples.push_back(cs); break;
            }
        }
        std::sort(s.crackSamples.begin(), s.crackSamples.end(), [](const Surface::CrackSample& a, const Surface::CrackSample& b) { return a.dz > b.dz; });
        if (s.crackSamples.size() > 6) s.crackSamples.resize(6);
        std::set<int> os(s.triOwner.begin(), s.triOwner.end()); s.owners.assign(os.begin(), os.end()); s.thick = 0;
        for (int o : s.owners) { const Lane& l = L.lanes[static_cast<size_t>(o)]; s.thick = std::max(s.thick, g.classes.at(l.cls).thick); }
    }
    tick(1.0);
    std::fprintf(stderr, "lanelab: surfaces %zu tris: cdt %.1f s, cover %.1f s, heights %.1f s (%u threads), weld %.1f s, decks %.1f s\n", T.tris.size(), sCdt, sCover, sHeights, nThreads, sWeld, std::chrono::duration<double>(std::chrono::steady_clock::now() - tD0).count());
}

std::vector<FlatMesh> layerMeshes(const RoadLabGraph& g, const DeckHeight& H, const PolySet& layer, const std::vector<int>& roads, std::vector<Vec2>* spanning) {
    std::vector<FlatMesh> out;
    for (const Polygon2& p : layer) {
        FlatMesh m = triangulatePolygon(p); if (m.tris.empty() || roads.empty()) continue;
        for (DeckVertex& v : m.verts) v.z = H.layerHeight(roads, v.xy);
        // a triangle steeper than any road can be (rise over its longest edge > 0.35, and more than a metre
        // of it) joins two levels: drop it and rebuild the boundary so the slab's side faces follow the cut
        std::vector<std::array<int, 3>> keep; keep.reserve(m.tris.size());
        for (const auto& t : m.tris) {
            const DeckVertex& a = m.verts[static_cast<size_t>(t[0])]; const DeckVertex& b = m.verts[static_cast<size_t>(t[1])]; const DeckVertex& c = m.verts[static_cast<size_t>(t[2])];
            const double rise = std::max({a.z, b.z, c.z}) - std::min({a.z, b.z, c.z});
            const double run = std::max({distance(a.xy, b.xy), distance(b.xy, c.xy), distance(c.xy, a.xy)});
            if (rise > 1.0 && rise > 0.35 * run) { if (spanning) spanning->push_back((a.xy + b.xy + c.xy) / 3.0); continue; }
            keep.push_back(t);
        }
        if (keep.size() != m.tris.size()) {
            m.tris = std::move(keep); m.boundary.clear(); std::map<std::pair<int, int>, int> count;
            for (const auto& tri : m.tris) for (int k = 0; k < 3; ++k) { int a = tri[k], b = tri[(k + 1) % 3]; count[{std::min(a, b), std::max(a, b)}]++; }
            for (const auto& kv : count) if (kv.second == 1) m.boundary.push_back(kv.first);
            if (m.tris.empty()) continue;
        }
        out.push_back(std::move(m));
    }
    (void)g; return out;
}

std::vector<int> layerRoads(const RoadLabGraph& g, bool anyRoad) {
    std::vector<int> roads;
    for (size_t i = 0; i < g.edges.size(); ++i) { const RoadClassSpec& c = g.cls(g.edges[i]); if (g.edges[i].laneCount() > 0 && (anyRoad || c.sidewalk > 0 || c.shoulder > 0 || c.median > 0)) roads.push_back(static_cast<int>(i)); }
    return roads;
}

}  // namespace lanelab
}  // namespace engine
