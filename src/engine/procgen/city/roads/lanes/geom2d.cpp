#include "engine/procgen/city/roads/lanes/geom2d.h"
#include "log.h"

#include <clipper2/clipper.h>
#include <CDT.h>

#include <algorithm>
#include <memory>
#include <cmath>

namespace engine {
namespace roads::lanes {

namespace {

constexpr double kScale = 1000.0;           // 1 mm integer grid

Clipper2Lib::Path64 toPath(const Ring& r) {
    Clipper2Lib::Path64 p; p.reserve(r.size());
    for (const Vec2& v : r) p.emplace_back(static_cast<int64_t>(std::llround(v.x * kScale)),
                                            static_cast<int64_t>(std::llround(v.y * kScale)));
    return p;
}

Ring fromPath(const Clipper2Lib::Path64& p) {
    Ring r; r.reserve(p.size());
    for (const auto& q : p) r.emplace_back(static_cast<Real>(q.x) / kScale, static_cast<Real>(q.y) / kScale);
    return r;
}

Clipper2Lib::Paths64 toPaths(const PolySet& s) {
    Clipper2Lib::Paths64 out;
    for (const Polygon2& p : s) {
        out.push_back(toPath(p.outer));
        for (const Ring& h : p.holes) out.push_back(toPath(h));
    }
    return out;
}

// Rebuild outer/hole structure from a solved PolyTree: children of the root are outers,
// their children holes (deeper nesting recurses as new outers).
void collect(const Clipper2Lib::PolyPath64& node, PolySet& out) {
    for (const auto& child : node) {
        Polygon2 poly; poly.outer = fromPath(child->Polygon());
        if (Clipper2Lib::Area(child->Polygon()) < 0) std::reverse(poly.outer.begin(), poly.outer.end());
        for (const auto& hole : *child) {
            Ring h = fromPath(hole->Polygon());
            if (Clipper2Lib::Area(hole->Polygon()) > 0) std::reverse(h.begin(), h.end());
            poly.holes.push_back(std::move(h));
            collect(*hole, out);            // islands inside holes are outers again
        }
        if (poly.outer.size() >= 3) out.push_back(std::move(poly));
    }
}

PolySet solve(Clipper2Lib::ClipType op, const Clipper2Lib::Paths64& subj, const Clipper2Lib::Paths64& clip) {
    Clipper2Lib::PolyTree64 tree;
    Clipper2Lib::Clipper64 c;
    c.PreserveCollinear(false);
    c.AddSubject(subj);
    if (!clip.empty()) c.AddClip(clip);
    c.Execute(op, Clipper2Lib::FillRule::NonZero, tree);
    PolySet out; collect(tree, out); return out;
}

PolySet fromPaths(const Clipper2Lib::Paths64& paths) {
    // Normalise an arbitrary path soup into outer/hole polygons via a union with itself.
    return solve(Clipper2Lib::ClipType::Union, paths, {});
}

}  // namespace

PolySet unionRings(const std::vector<Ring>& rings) {
    Clipper2Lib::Paths64 paths;
    for (const Ring& r : rings) {
        if (r.size() < 3) continue;
        Clipper2Lib::Path64 p = toPath(r);
        if (Clipper2Lib::Area(p) < 0) std::reverse(p.begin(), p.end());   // every input ring counts as positive
        paths.push_back(std::move(p));
    }
    return solve(Clipper2Lib::ClipType::Union, paths, {});
}

PolySet unionSets(const PolySet& a, const PolySet& b) { return solve(Clipper2Lib::ClipType::Union, toPaths(a), toPaths(b)); }
PolySet differenceSets(const PolySet& a, const PolySet& b) { return solve(Clipper2Lib::ClipType::Difference, toPaths(a), toPaths(b)); }
PolySet intersectSets(const PolySet& a, const PolySet& b) { return solve(Clipper2Lib::ClipType::Intersection, toPaths(a), toPaths(b)); }

PolySet offsetSet(const PolySet& a, double delta) {
    if (a.empty()) return {};
    Clipper2Lib::Paths64 grown = Clipper2Lib::InflatePaths(toPaths(a), delta * kScale, Clipper2Lib::JoinType::Round,
                                                           Clipper2Lib::EndType::Polygon, 2.0, 0.02 * kScale);
    return fromPaths(grown);
}

PolySet closing(const PolySet& a, double r) { return offsetSet(offsetSet(a, r), -r); }

double ringArea(const Ring& r) {
    double s = 0;
    for (size_t i = 0, n = r.size(); i < n; ++i) {
        const Vec2& p = r[i]; const Vec2& q = r[(i + 1) % n];
        s += p.x * q.y - q.x * p.y;
    }
    return 0.5 * s;
}

double setArea(const PolySet& s) {
    double a = 0;
    for (const Polygon2& p : s) {
        a += std::fabs(ringArea(p.outer));
        for (const Ring& h : p.holes) a -= std::fabs(ringArea(h));
    }
    return a;
}

namespace {
bool inRing(const Ring& r, const Vec2& q) {
    Clipper2Lib::Point64 pt(static_cast<int64_t>(std::llround(q.x * kScale)), static_cast<int64_t>(std::llround(q.y * kScale)));
    return Clipper2Lib::PointInPolygon(pt, toPath(r)) != Clipper2Lib::PointInPolygonResult::IsOutside;
}
}  // namespace

bool contains(const Polygon2& p, const Vec2& q) {
    if (!inRing(p.outer, q)) return false;
    for (const Ring& h : p.holes) if (inRing(h, q)) return false;
    return true;
}

bool contains(const PolySet& s, const Vec2& q) {
    for (const Polygon2& p : s) if (contains(p, q)) return true;
    return false;
}

// A ring as integer points, its edges binned by y: a point's crossing test reads only the edges whose
// y-span covers its band, not all of them (a 10 km lane footprint has thousands).
struct BinnedRing {
    Clipper2Lib::Path64 pts;
    int64_t y0 = 0, bin = 1;
    std::vector<std::vector<uint32_t>> edges;   // per y band: edge i runs pts[i] -> pts[i + 1]
    void build(Clipper2Lib::Path64 p) {
        pts = std::move(p);
        if (pts.size() < 3) return;
        int64_t lo = pts[0].y, hi = pts[0].y;
        for (const auto& q : pts) { lo = std::min(lo, q.y); hi = std::max(hi, q.y); }
        const std::size_t n = pts.size();
        const int64_t bands = std::clamp<int64_t>(static_cast<int64_t>(n / 4), 1, 4096);
        y0 = lo; bin = std::max<int64_t>(1, (hi - lo) / bands + 1);
        edges.assign(static_cast<std::size_t>((hi - lo) / bin + 1), {});
        for (std::size_t i = 0; i < n; ++i) {
            const auto& a = pts[i]; const auto& b = pts[(i + 1) % n];
            const int64_t e0 = (std::min(a.y, b.y) - y0) / bin, e1 = (std::max(a.y, b.y) - y0) / bin;
            for (int64_t k = e0; k <= e1; ++k) edges[static_cast<std::size_t>(k)].push_back(static_cast<uint32_t>(i));
        }
    }
    // Clipper2's PointInPolygon, restricted to the edges that can matter: on an edge is on, else the
    // even-odd count of edges crossing the horizontal ray to the right.
    bool inside(const Clipper2Lib::Point64& q) const {
        if (pts.size() < 3 || q.y < y0) return false;
        const int64_t k = (q.y - y0) / bin;
        if (k >= static_cast<int64_t>(edges.size())) return false;
        const std::size_t n = pts.size();
        bool odd = false;
        for (uint32_t i : edges[static_cast<std::size_t>(k)]) {
            const auto& a = pts[i]; const auto& b = pts[(i + 1) % n];
            // on the segment?
            const __int128 cr = static_cast<__int128>(b.x - a.x) * (q.y - a.y) - static_cast<__int128>(b.y - a.y) * (q.x - a.x);
            if (cr == 0 && std::min(a.x, b.x) <= q.x && q.x <= std::max(a.x, b.x) && std::min(a.y, b.y) <= q.y && q.y <= std::max(a.y, b.y)) return true;
            if ((a.y > q.y) == (b.y > q.y)) continue;
            // x of the edge at q.y, compared exactly: cross-multiplied by (b.y - a.y)
            const __int128 lhs = static_cast<__int128>(q.x - a.x) * (b.y - a.y), rhs = static_cast<__int128>(b.x - a.x) * (q.y - a.y);
            if (b.y > a.y ? lhs < rhs : lhs > rhs) odd = !odd;
        }
        return odd;
    }
};

struct PreparedSet::Poly {
    BinnedRing outer;
    std::vector<BinnedRing> holes;
};

PreparedSet::PreparedSet(const PolySet& s) {
    auto polys = std::make_shared<std::vector<Poly>>();
    polys->reserve(s.size());
    for (const Polygon2& p : s) {
        Poly q;
        q.outer.build(toPath(p.outer));
        for (const Ring& h : p.holes) { q.holes.emplace_back(); q.holes.back().build(toPath(h)); }
        polys->push_back(std::move(q));
    }
    polys_ = std::move(polys);
}

bool PreparedSet::contains(const Vec2& q) const {
    if (!polys_) return false;
    const Clipper2Lib::Point64 pt(static_cast<int64_t>(std::llround(q.x * kScale)), static_cast<int64_t>(std::llround(q.y * kScale)));
    for (const Poly& p : *polys_) {
        if (!p.outer.inside(pt)) continue;
        bool inHole = false;
        for (const BinnedRing& h : p.holes) if (h.inside(pt)) { inHole = true; break; }
        if (!inHole) return true;
    }
    return false;
}

std::vector<PreparedSet> prepareAll(const std::vector<PolySet>& sets) {
    std::vector<PreparedSet> out;
    out.reserve(sets.size());
    for (const PolySet& s : sets) out.emplace_back(s);
    return out;
}

Box2 bounds(const Polygon2& p) {
    Box2 b; bool first = true;
    for (const Vec2& v : p.outer) {
        if (first) { b.minX = b.maxX = v.x; b.minY = b.maxY = v.y; first = false; continue; }
        b.minX = std::min(b.minX, v.x); b.maxX = std::max(b.maxX, v.x);
        b.minY = std::min(b.minY, v.y); b.maxY = std::max(b.maxY, v.y);
    }
    return b;
}
Box2 bounds(const PolySet& ps) {
    Box2 b; bool first = true;
    for (const Polygon2& p : ps) {
        if (p.outer.empty()) continue; Box2 q = bounds(p);
        if (first) { b = q; first = false; continue; }
        b.minX = std::min(b.minX, q.minX); b.maxX = std::max(b.maxX, q.maxX);
        b.minY = std::min(b.minY, q.minY); b.maxY = std::max(b.maxY, q.maxY);
    }
    return b;
}

PolySet fromRing(const Ring& r) { return unionRings({r}); }

Triangulation constrainedTriangulation(const std::vector<Vec2>& points, const std::vector<std::pair<int, int>>& edgesIn,
                                       double minDist) {
    std::vector<CDT::V2d<double>> verts; verts.reserve(points.size());
    for (const Vec2& p : points) verts.push_back(CDT::V2d<double>(p.x, p.y));
    std::vector<CDT::Edge> edges; edges.reserve(edgesIn.size());
    const int nPoints = static_cast<int>(points.size());
    int bad = 0;
    for (const auto& e : edgesIn) {
        // an edge naming a point that does not exist would be read past the end of CDT's tables (undefined
        // behaviour, and a triangulation that fails differently run to run): refused, and counted
        if (e.first < 0 || e.second < 0 || e.first >= nPoints || e.second >= nPoints) { ++bad; continue; }
        edges.emplace_back(static_cast<CDT::VertInd>(e.first), static_cast<CDT::VertInd>(e.second));
    }
    if (bad > 0) LOG_ERROR << "[roads/lanes] triangulation: " << bad << " constraint edge(s) name a point out of range (of " << nPoints << ") -- dropped";
    // Duplicate points are merged (their edges remapped) so coincident lane rails share vertices.
    CDT::RemoveDuplicatesAndRemapEdges(verts, edges);
    // One attempt per snapping distance: the first that resolves every crossing wins.
    std::unique_ptr<CDT::Triangulation<double>> built;
    for (const double snap : {minDist, 1e-4, 1e-3, 1e-2}) {
        if (snap < minDist) continue;
        auto cdt = std::make_unique<CDT::Triangulation<double>>(CDT::VertexInsertionOrder::Auto,
                                                                CDT::IntersectingConstraintEdges::TryResolve, snap);
        try {
            cdt->insertVertices(verts);
            cdt->insertEdges(edges);
        } catch (const CDT::InvalidEdgeSplitVertex&) {
            if (snap >= 1e-2) throw;   // nothing coarser to try: the caller reports it
            continue;
        }

        if (snap > minDist) LOG_WARN << "[roads/lanes] triangulation: a near-degenerate edge crossing resolved at a "
                                     << snap << " m snap (" << minDist << " m could not split it)";
        built = std::move(cdt);
        break;
    }
    CDT::Triangulation<double>& cdt = *built;
    cdt.eraseSuperTriangle();
    Triangulation out; out.verts.reserve(cdt.vertices.size());
    for (const auto& v : cdt.vertices) out.verts.emplace_back(v.x, v.y);
    out.tris.reserve(cdt.triangles.size());
    for (const auto& t : cdt.triangles) {
        std::array<int, 3> tri{static_cast<int>(t.vertices[0]), static_cast<int>(t.vertices[1]), static_cast<int>(t.vertices[2])};
        const Vec2& a = out.verts[tri[0]]; const Vec2& b = out.verts[tri[1]]; const Vec2& c = out.verts[tri[2]];
        if (cross(b - a, c - a) < 0) std::swap(tri[1], tri[2]);
        out.tris.push_back(tri);
    }
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
