#include "district.h"

#include <cmath>

namespace engine {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct Rng {
    std::uint32_t s;
    explicit Rng(std::uint32_t seed) : s(seed ? seed : 0x2545f491u) {}
    std::uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    double unit() { return (next() >> 8) * (1.0 / 16777216.0); }
};

// The span of the infinite line (pt + t*dir) within `poly`: its first and last boundary crossings.
// For a convex face that's the full chord; the street the split lays down.
bool cutSpan(const Poly2& poly, const Vec2& pt, const Vec2& dir, Vec2& a, Vec2& b) {
    Vec2 nrm(-dir.y, dir.x);
    double c = nrm.x * pt.x + nrm.y * pt.y;
    double tMin = 1e30, tMax = -1e30; int hits = 0;
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        Vec2 p0 = poly[i], p1 = poly[(i + 1) % n];
        double d0 = nrm.x * p0.x + nrm.y * p0.y - c;
        double d1 = nrm.x * p1.x + nrm.y * p1.y - c;
        if ((d0 <= 0 && d1 > 0) || (d0 > 0 && d1 <= 0)) {
            double u = d0 / (d0 - d1);
            Vec2 x = p0 + (p1 - p0) * u;
            double t = (x.x - pt.x) * dir.x + (x.y - pt.y) * dir.y;
            tMin = std::min(tMin, t); tMax = std::max(tMax, t); ++hits;
        }
    }
    if (hits < 2) return false;
    a = pt + dir * tMin; b = pt + dir * tMax;
    return true;
}

// Recursively bisect a face along its long OBB axis until block-sized; record each cut as a street.
void subdivideStreets(const Poly2& poly, double targetArea, double minEdge, double jitter,
                      Rng& rng, std::vector<std::vector<Vec2>>& streets,
                      std::vector<Poly2>& blocks, int depth) {
    double ar = area(poly);
    if (poly.size() < 3 || depth > 22) { if (ar >= 1.0) blocks.push_back(poly); return; }
    OBB2 obb = orientedBoundingBox(poly);
    int la = obb.longAxis();
    double longHalf = obb.half[la], shortHalf = obb.half[1 - la];
    if (ar <= targetArea || longHalf < minEdge || shortHalf * 2.0 < minEdge) {
        blocks.push_back(poly);
        return;
    }
    Vec2 dirLong = obb.axis[la], cutDir = obb.axis[1 - la];
    double offset = (rng.unit() - 0.5) * 2.0 * jitter * longHalf;
    Vec2 sp = obb.center + dirLong * offset;
    Poly2 left, right;
    splitByLine(poly, sp, cutDir, left, right);
    if (left.size() < 3 || right.size() < 3) { blocks.push_back(poly); return; }
    Vec2 a, b;
    if (cutSpan(poly, sp, cutDir, a, b)) streets.push_back({a, b});
    subdivideStreets(left, targetArea, minEdge, jitter, rng, streets, blocks, depth + 1);
    subdivideStreets(right, targetArea, minEdge, jitter, rng, streets, blocks, depth + 1);
}

}  // namespace

DistrictNet buildDistrict(const DistrictParams& p) {
    Rng rng(p.seed ? p.seed : 1u);
    DistrictNet d;
    std::vector<std::vector<Vec2>> arterials, streets;

    // 1. Irregular footprint (an n-gon with jittered radii).
    Poly2 footprint;
    const int N = 32;
    for (int i = 0; i < N; ++i) {
        double a = 2.0 * kPi * i / N;
        double r = p.radius * (1.0 - p.irregular * 0.5 + p.irregular * rng.unit());
        footprint.push_back(Vec2(p.center.x + std::cos(a) * r, p.center.y + std::sin(a) * r));
    }
    ensureCCW(footprint);

    // 2. Split the footprint into sectors with a few arterials through (near) the centre.
    std::vector<Poly2> faces = {footprint};
    for (int k = 0; k < p.arterials; ++k) {
        double ang = kPi * (k + rng.unit() * 0.35) / std::max(1, p.arterials);
        Vec2 dir(std::cos(ang), std::sin(ang));
        // All arterials pass through the SAME centre, so they cross at ONE point — a single clean
        // roundabout where every spoke meets, instead of jittered near-centre crossings that promote
        // into a tangle of overlapping rings.
        Vec2 pt = p.center;
        Vec2 a, b;
        if (cutSpan(footprint, pt, dir, a, b)) arterials.push_back({a, b});
        std::vector<Poly2> next;
        for (const Poly2& f : faces) {
            Poly2 l, r;
            splitByLine(f, pt, dir, l, r);
            if (l.size() >= 3 && r.size() >= 3) { next.push_back(l); next.push_back(r); }
            else next.push_back(f);
        }
        faces = std::move(next);
    }

    // 3. Subdivide each sector into a grid of blocks; the cuts are local streets.
    double targetArea = p.blockSize * p.blockSize;
    double minEdge = p.blockSize * 0.55;
    for (const Poly2& f : faces)
        subdivideStreets(f, targetArea, minEdge, p.jitter, rng, streets, d.blocks, 0);

    // 4. Assemble ONE graph and planarize: split every crossing/T into shared nodes so the
    //    arterials and street grid become a single connected network (no orphan segments).
    RoadGraph g;
    auto addSeg = [&](const std::vector<Vec2>& seg, double width, RoadClass cls) {
        if (seg.size() < 2) return;
        int a = g.addNode(seg[0], 1.0), b = g.addNode(seg[1], 1.0);
        if (a != b) g.addEdge(a, b, static_cast<Real>(width), cls);
    };
    for (const std::vector<Vec2>& s : arterials) addSeg(s, p.arteryWidth, RoadClass::Arterial);
    for (const std::vector<Vec2>& s : streets)   addSeg(s, p.streetWidth, RoadClass::Local);
    RoadGraph planar = planarize(g, 1.0);

    // 5. Drop only genuinely TINY dangling stubs — short fragments left by a near-degenerate cut.
    //    A full-length street that ends at the city edge is a legitimate degree-1 node, so prune by
    //    LENGTH, not just degree, or the whole grid unwinds. One pass is enough for the fragments.
    const double stubLen = p.blockSize * 0.4;
    {
        std::vector<int> degree(planar.nodes.size(), 0);
        for (const RoadEdge& e : planar.edges) { ++degree[e.a]; ++degree[e.b]; }
        RoadGraph next;
        for (const RoadEdge& e : planar.edges) {
            double len = (planar.nodes[e.a].pos - planar.nodes[e.b].pos).length();
            if (e.klass == RoadClass::Local && len < stubLen &&
                (degree[e.a] == 1 || degree[e.b] == 1))
                continue;
            int a = next.addNode(planar.nodes[e.a].pos, 0.01);
            int b = next.addNode(planar.nodes[e.b].pos, 0.01);
            if (a != b) next.addEdge(a, b, e.width, e.klass);
        }
        planar = std::move(next);
    }
    d.graph = std::move(planar);
    return d;
}

}  // namespace engine
