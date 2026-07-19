#include "road_semantics.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace engine {

namespace {

bool isStreet(RoadClass k) {
    return k == RoadClass::Arterial || k == RoadClass::Collector ||
           k == RoadClass::Local;
}

// World Y of a node: authored deck height, or ground + relative lift.
double nodeY(const RoadGraph& g, int v, const GroundFn& groundAt) {
    const RoadNode& n = g.nodes[v];
    if (n.elevAbsolute) return n.elev;
    return (groundAt ? groundAt(n.pos.x, n.pos.y) : 0.0) + n.elev;
}

}  // namespace

bool roadGraphUnclassified(const RoadGraph& g) {
    for (const RoadNode& n : g.nodes)
        if (n.kind == JunctionKind::Auto) return true;
    return false;
}

void classifyRoadGraph(RoadGraph& g, const GroundFn& groundAt) {
    const int n = static_cast<int>(g.nodes.size());
    std::vector<std::vector<int>> inc(n);
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        const RoadEdge& e = g.edges[ei];
        if (e.a >= 0 && e.a < n) inc[e.a].push_back(ei);
        if (e.b >= 0 && e.b < n) inc[e.b].push_back(ei);
    }

    // ---- pass 1: node kinds -------------------------------------------
    for (int v = 0; v < n; ++v) {
        const int d = static_cast<int>(inc[v].size());
        // A HINT (baked gore/landing) wins — but only while the degree
        // still supports it: roadNetStreetsOnly strips the ramp edges off a
        // hinted landing, and the leftover street run must degrade to what
        // its remaining arms actually are. Degree 2 KEEPS a hint: a ramp
        // grafted onto a street's very tip is still a Landing (street +
        // ramp, one of each) even though no third arm exists.
        if (d >= 2 && g.nodes[v].kind != JunctionKind::Auto) continue;
        JunctionKind kind;
        if (d == 0) {
            kind = JunctionKind::None;
        } else if (d == 1) {
            kind = JunctionKind::DeadEnd;
        } else if (d == 2) {
            kind = JunctionKind::None;   // through / curve sample
        } else {
            int nStreet = 0, nFwy = 0, nRamp = 0;
            for (int ei : inc[v]) {
                const RoadClass k = g.edges[ei].klass;
                if (k == RoadClass::Freeway) ++nFwy;
                else if (k == RoadClass::Ramp) ++nRamp;
                else ++nStreet;
            }
            if (nStreet > 0 && nRamp > 0)
                kind = JunctionKind::Landing;       // ramp foot meets street
            else if (nStreet > 0 && nFwy > 0)
                kind = JunctionKind::Intersection;  // pathological; audit sees it
            else if (nRamp > 0 || nFwy > 0)
                kind = JunctionKind::Merge;         // unhinted gore fallback
            else
                kind = JunctionKind::Intersection;  // streets crossing streets
        }
        g.nodes[v].kind = kind;
    }

    // ---- pass 2: edge access ------------------------------------------
    for (RoadEdge& e : g.edges) {
        const bool street = isStreet(e.klass);
        const bool elevated = (e.a >= 0 && e.a < n && g.nodes[e.a].elevAbsolute) ||
                              (e.b >= 0 && e.b < n && g.nodes[e.b].elevAbsolute);
        const bool walk = e.walkable && street;
        // APPROACH edge (issue #20): a street edge running into a ramp
        // LANDING while climbing — the on-ramp's feeder. It keeps cars but
        // loses pedestrian frontage: no sidewalk band, no lots, no bays.
        // Conservative: a flat street ending at a landing keeps frontage.
        bool approach = false;
        if (street && e.a >= 0 && e.a < n && e.b >= 0 && e.b < n &&
            (g.nodes[e.a].kind == JunctionKind::Landing ||
             g.nodes[e.b].kind == JunctionKind::Landing)) {
            if (elevated) {
                approach = true;
            } else {
                const Vec2 d2 = g.nodes[e.b].pos - g.nodes[e.a].pos;
                const double len = std::max(1.0, static_cast<double>(d2.length()));
                const double dy =
                    std::fabs(nodeY(g, e.b, groundAt) - nodeY(g, e.a, groundAt));
                approach = dy / len > 0.05;
            }
        }
        uint8_t access = 0;
        if (walk) access |= road_access::kWalkable;
        if (walk && e.layer == 0 && !elevated && !approach)
            access |= road_access::kFrontage;
        if (walk && e.layer == 0 && !approach) access |= road_access::kCrossable;
        if (street) access |= road_access::kSignalable;
        e.access = access;
        // NOTE: bool e.walkable is deliberately untouched this round —
        // nav/pathfind still read it; kWalkable mirrors it.
    }
}

std::vector<CrossingViolation> auditRoadGraph(const RoadGraph& g,
                                              const GroundFn& groundAt,
                                              double clearance) {
    std::vector<CrossingViolation> out;
    const int n = static_cast<int>(g.nodes.size());
    const int m = static_cast<int>(g.edges.size());
    if (n == 0 || m == 0) return out;

    // Spatial hash of edge SEGMENTS in ~16 m cells: candidate pairs share a
    // cell; each pair is tested once (a < b).
    const double cell = 16.0;
    std::unordered_map<long long, std::vector<int>> grid;
    auto key = [&](double x, double y) {
        const long long ix = static_cast<long long>(std::floor(x / cell));
        const long long iy = static_cast<long long>(std::floor(y / cell));
        return ix * 73856093LL ^ iy * 19349663LL;
    };
    auto edgeOk = [&](const RoadEdge& e) {
        return e.a >= 0 && e.a < n && e.b >= 0 && e.b < n && e.a != e.b;
    };
    for (int ei = 0; ei < m; ++ei) {
        const RoadEdge& e = g.edges[ei];
        if (!edgeOk(e)) continue;
        const Vec2 A = g.nodes[e.a].pos, B = g.nodes[e.b].pos;
        const double len = (B - A).length();
        const int steps = std::max(1, static_cast<int>(std::ceil(len / cell)));
        for (int s = 0; s <= steps; ++s) {
            const double t = static_cast<double>(s) / steps;
            const Vec2 p = A + (B - A) * t;
            grid[key(p.x, p.y)].push_back(ei);
        }
    }
    for (auto& [k, v] : grid) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }

    auto sharesEndpoint = [&](const RoadEdge& a, const RoadEdge& b) {
        if (a.a == b.a || a.a == b.b || a.b == b.a || a.b == b.b) return true;
        // Position-shared endpoints too (multi-net concatenations duplicate
        // nodes at the same spot without sharing indices).
        const double tol2 = 0.5 * 0.5;
        const Vec2 pa[2] = { g.nodes[a.a].pos, g.nodes[a.b].pos };
        const Vec2 pb[2] = { g.nodes[b.a].pos, g.nodes[b.b].pos };
        for (const Vec2& x : pa)
            for (const Vec2& y : pb)
                if ((x - y).lengthSquared() < tol2) return true;
        return false;
    };

    std::vector<std::pair<int, int>> seen;
    for (const auto& [k, list] : grid) {
        for (std::size_t i = 0; i < list.size(); ++i) {
            for (std::size_t j = i + 1; j < list.size(); ++j) {
                const int ea = list[i], eb = list[j];
                const RoadEdge& a = g.edges[ea];
                const RoadEdge& b = g.edges[eb];
                if (sharesEndpoint(a, b)) continue;
                // 2-D segment intersection (strict interior).
                const Vec2 p = g.nodes[a.a].pos, q = g.nodes[a.b].pos;
                const Vec2 r = g.nodes[b.a].pos, s2 = g.nodes[b.b].pos;
                const Vec2 d1 = q - p, d2 = s2 - r;
                const double den = d1.x * d2.y - d1.y * d2.x;
                if (std::fabs(den) < 1e-9) continue;   // parallel
                const Vec2 pr = r - p;
                const double t = (pr.x * d2.y - pr.y * d2.x) / den;
                const double u = (pr.x * d1.y - pr.y * d1.x) / den;
                if (t <= 0.02 || t >= 0.98 || u <= 0.02 || u >= 0.98) continue;
                // Vertical gap at the crossing.
                const double ya = nodeY(g, a.a, groundAt) +
                                  (nodeY(g, a.b, groundAt) -
                                   nodeY(g, a.a, groundAt)) * t;
                const double yb = nodeY(g, b.a, groundAt) +
                                  (nodeY(g, b.b, groundAt) -
                                   nodeY(g, b.a, groundAt)) * u;
                const double dY = std::fabs(ya - yb);
                if (dY >= clearance) continue;         // declared grade-sep
                if (a.layer != b.layer && dY >= clearance * 0.5)
                    continue;   // layered pair with reasonable spacing
                const auto pairKey = std::minmax(ea, eb);
                if (std::find(seen.begin(), seen.end(),
                              std::make_pair(pairKey.first, pairKey.second)) !=
                    seen.end())
                    continue;
                seen.push_back({ pairKey.first, pairKey.second });
                CrossingViolation cv;
                cv.edgeA = pairKey.first;
                cv.edgeB = pairKey.second;
                cv.at = p + d1 * t;
                cv.dY = dY;
                out.push_back(cv);
            }
        }
    }
    return out;
}

}  // namespace engine
