#include "engine/procgen/lanelab/road_twin.h"
#include "engine/procgen/lanelab/polyline_ops.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <unordered_map>

namespace engine {
namespace lanelab {

namespace {

constexpr double kTwinTolerance = 1.5;   // Douglas-Peucker tolerance of the twin's spines (metres)

struct Line { std::vector<Vec2> pts; std::vector<double> z, s; RoadClass k; double w = 8; bool deck = false; double lotsFrom = -1, lotsTo = -1; };

RoadClass classOf(const EdgeSpec& e) {
    if (e.isRamp() || e.cls == "ramp") return RoadClass::Ramp;
    if (e.cls == "freeway") return RoadClass::Freeway;
    if (e.cls == "arterial") return RoadClass::Arterial;
    if (e.cls == "collector") return RoadClass::Collector;
    if (e.cls == "alley") return RoadClass::Alley;
    return RoadClass::Local;
}

// Segment p+t(q-p) against a+u(b-a): true with t,u when the lines are not parallel.
bool lineParams(const Vec2& p, const Vec2& q, const Vec2& a, const Vec2& b, double& t, double& u) {
    const Vec2 r = q - p, s = b - a; const double den = r.x * s.y - r.y * s.x;
    if (std::fabs(den) < 1e-12) return false;
    const Vec2 ap = a - p;
    t = (ap.x * s.y - ap.y * s.x) / den; u = (ap.x * r.y - ap.y * r.x) / den; return true;
}

// Parameter along a-b of the point nearest p, and the distance to it.
double nearestParam(const Vec2& p, const Vec2& a, const Vec2& b, double& dist) {
    const Vec2 d = b - a; const double L2 = d.x * d.x + d.y * d.y;
    double u = L2 > 0 ? ((p.x - a.x) * d.x + (p.y - a.y) * d.y) / L2 : 0.0; u = std::max(0.0, std::min(1.0, u));
    const Vec2 c(a.x + d.x * u, a.y + d.y * u); dist = std::hypot(p.x - c.x, p.y - c.y); return u;
}

// Douglas-Peucker on a polyline, returning the indices kept (endpoints always), then
// splitting any run longer than `maxSpan` metres (station-based) so long curves still sample.
std::vector<size_t> simplifyIndices(const std::vector<Vec2>& pts, double tol, double maxSpan, const std::vector<double>& s) {
    std::vector<char> keep(pts.size(), 0); keep.front() = keep.back() = 1;
    std::vector<std::pair<size_t, size_t>> stack{{0, pts.size() - 1}};
    while (!stack.empty()) {
        auto [a, b] = stack.back(); stack.pop_back(); if (b <= a + 1) continue;
        const Vec2 d = pts[b] - pts[a]; const double L = std::hypot(d.x, d.y); size_t best = a; double bd = -1;
        for (size_t i = a + 1; i < b; ++i) {
            const Vec2 v = pts[i] - pts[a];
            const double dist = L > 1e-9 ? std::fabs(v.x * d.y - v.y * d.x) / L : std::hypot(v.x, v.y);
            if (dist > bd) { bd = dist; best = i; }
        }
        const bool tooLong = s.size() == pts.size() && s[b] - s[a] > maxSpan;
        if (bd > tol || tooLong) { if (bd <= tol) best = (a + b) / 2; keep[best] = 1; stack.push_back({a, best}); stack.push_back({best, b}); }
    }
    std::vector<size_t> out; for (size_t i = 0; i < pts.size(); ++i) if (keep[i]) out.push_back(i); return out;
}

}  // namespace

RoadEntity roadTwin(const Result& r, double nodeSpacing, bool forLots) {
    RoadEntity net; net.look.sidewalk = 3.5; net.look.autoRoundabout = false;   // lanelab owns its junctions
    std::vector<Line> lines;
    // A ramp is one lanelab edge but two things to the city: at grade, beside the street it leaves
    // (the first hundred metres or so), it is pavement a block must stop at; once it climbs it is
    // right-of-way the lot pass keeps clear and re-zones under. The lot pass skips RoadClass::Ramp as
    // block frontage, so an at-grade ramp inside a face left the face whole and lots were parcelled
    // across the ramp lane (Glenn's "lots intersect parts of the onramps"; metro: the worst lot
    // overlaps all sat at the inner ramp feet). Split each ramp at the 1.5 m mark: the at-grade
    // runs go in as Local (a face boundary), the elevated runs as Ramp.
    std::vector<EdgeSpec> split;
    for (const EdgeSpec& e : r.graph.edges) {
        if (!e.isRamp() || e.xy.size() < 2 || e.z.size() != e.xy.size() || e.t.size() != e.xy.size()) { split.push_back(e); continue; }
        size_t start = 0;
        auto flush = [&](size_t a, size_t b, bool grade) {   // [a, b] inclusive
            if (b <= a) return; EdgeSpec part = e; part.xy.assign(e.xy.begin() + a, e.xy.begin() + b + 1); part.z.assign(e.z.begin() + a, e.z.begin() + b + 1); part.t.assign(e.t.begin() + a, e.t.begin() + b + 1);
            part.s.assign(e.s.begin() + a, e.s.begin() + b + 1); part.cls = grade ? "__ramp_at_grade" : e.cls; split.push_back(std::move(part));
        };
        bool grade = e.z[0] - e.t[0] < 1.5;
        for (size_t i = 1; i < e.xy.size(); ++i) {
            const bool g = e.z[i] - e.t[i] < 1.5;
            if (g != grade) { flush(start, i, grade); start = i; grade = g; }
        }
        flush(start, e.xy.size() - 1, grade);
    }
    // The wedge between a ramp's at-grade run and the street it leaves is paved gore in the lab but a
    // thin FACE to the parceller (metro: the last 40 lots on pavement all sat there). A ribbon down the
    // wedge's midline, as wide as the wedge, keeps the clearance check honest: the sliver face collapses
    // under pushPolyClearOfRoads and no lot survives in it. Ramp class: right-of-way, never frontage.
    // OFF (kGoreRibbons): tried 2026-09-05, the chunk endpoints merged into street nodes under the
    // node tolerance and the short-edge cleanup then contracted real junctions — metro lost 49 of
    // 143 blocks. The 38 lots left in ramp-foot gores need the wedge as a face of its own, not a
    // ribbon; kept for that rework.
    constexpr bool kGoreRibbons = false;
    std::vector<Line> gores;
    for (const EdgeSpec& e : split) {
        if (!kGoreRibbons || e.cls != "__ramp_at_grade" || e.xy.size() < 2) continue;
        const EdgeSpec* whole = r.graph.find(e.id); if (!whole) continue;
        const std::string hostId = whole->from.side.empty() ? whole->from.edge : whole->to.edge;   // the street end has no side
        const EdgeSpec* host = r.graph.find(hostId); if (!host || host->xy.size() < 2) continue;
        const double laneW = whole->lanes.w;
        for (size_t i = 0; i + 1 < e.xy.size(); i += 4) {   // ~ one chunk per 4 spine samples
            const size_t j = std::min(i + 4, e.xy.size() - 1);
            Line G; G.k = RoadClass::Ramp; G.deck = true; double wmax = 0;
            for (size_t k = i; k <= j; ++k) {
                Projection pr = project(host->xy, host->s, e.xy[k]); const Vec2 hp = pointAt(host->xy, host->s, pr.station);
                G.pts.push_back((e.xy[k] + hp) * 0.5); G.z.push_back(0); G.s.push_back(e.s[k]); wmax = std::max(wmax, pr.distance);
            }
            if (wmax < 2.0 || G.pts.size() < 2) continue;
            G.w = wmax + laneW + 2.0; gores.push_back(std::move(G));
        }
    }
    for (const EdgeSpec& e : split) {
        if (e.xy.size() < 2) continue;
        const bool atGradeRamp = e.cls == "__ramp_at_grade";
        Line L; L.k = atGradeRamp ? RoadClass::Local : classOf(e); L.deck = L.k == RoadClass::Freeway || L.k == RoadClass::Ramp;
        const bool deckEarthwork = L.deck;   // the width padding stays with the real deck classes
        if (forLots && L.deck) { L.k = RoadClass::Local; L.deck = false; }   // a face boundary, planarised with the streets
        // Paved width for streets. Freeways and ramps carry their EARTHWORK too: the conform band grades
        // conform_w beyond the shoulder on cut and fill, and a building on that slope is the failure
        // Glenn saw ("lots intersect parts of the onramps"). The lot pass keeps buildings edge-width/2
        // + clearance from every sampled centreline, so the width is where the embankment lives.
        const RoadClassSpec& cs = atGradeRamp ? r.graph.classes.at(r.graph.find(e.id)->cls) : r.graph.cls(e);
        L.w = 2.0 * (r.graph.hw(atGradeRamp ? *r.graph.find(e.id) : e) + cs.shoulder + kTwinTolerance + (deckEarthwork ? r.graph.rules.conformW : 0.0));
        // Corners only (Douglas-Peucker at 0.25 m), capped at `nodeSpacing` between nodes: a straight
        // street becomes ONE chord between its junctions. The lot pass insets each block face by
        // the sidewalk with a miter offset that collapses on runs of collinear vertices — a face
        // with a node every 8 m along a straight edge came back with zero area (ring city: all 16
        // grid blocks lost). Curves keep the vertices their deviation needs.
        // Tolerance 1.5 m, not 0.25: the lot pass insets every block face by the sidewalk with a
        // miter offset that REJECTS the whole face when any edge reverses, and a vertex every
        // few metres along a curved street (0.25 m keeps them) makes edges shorter than the
        // inset — metro: 30 of 89 faces died, the rim synthesiser then treated those blocks as
        // open ground and its rectangles straddled the interior streets. At 1.5 m a 30 m-radius
        // curve keeps ~19 m chords. The half-width below is padded by the tolerance so the
        // clearance check stays honest against the built kerb.
        std::vector<size_t> keep = simplifyIndices(e.xy, kTwinTolerance, nodeSpacing, e.s);
        for (size_t i : keep) { L.pts.push_back(e.xy[i]); L.z.push_back(i < e.z.size() ? e.z[i] : 0.0); L.s.push_back(i < e.s.size() ? e.s[i] : 0.0); }
        L.lotsFrom = e.lotsFrom; L.lotsTo = e.lotsTo;
        if (classOf(e) == RoadClass::Freeway && !atGradeRamp) net.plan.freewayPlans.push_back(e.xy);   // the lot pass's 34 m fallback band, either mode
        if (atGradeRamp) {
            // An at-grade ramp run is a face boundary inside the city, but beyond the host street's
            // lots_range (the outer feet, past the freeway) it is right-of-way like the street there:
            // otherwise the rim synthesiser lines it with lots in open country.
            L.lotsFrom = -1; L.lotsTo = -1;
            const EdgeSpec* whole = r.graph.find(e.id);
            const EdgeSpec* host = whole ? r.graph.find(whole->from.side.empty() ? whole->from.edge : whole->to.edge) : nullptr;
            if (host && host->lotsTo >= 0 && host->xy.size() >= 2) {
                const Vec2 mid = e.xy[e.xy.size() / 2];
                if (project(host->xy, host->s, mid).station > host->lotsTo) { L.lotsFrom = 1e18; L.lotsTo = 1e18; }   // every sub-edge baked
            }
        }
        lines.push_back(std::move(L));
    }
    for (Line& G : gores) lines.push_back(std::move(G));   // deck lines: never split, never splitting
    // --- planarise the street subgraph: split parameters per segment ---
    const double tol = 0.5, eps = 1e-6, cell = 40.0;
    // A street ending on a CURVED road ends up to kTwinTolerance off that road's chord, so a T must be
    // recognised out to that distance and the endpoint snapped onto the chord (else it is a dead-end
    // spur inside the block, which the lot pass's inset turns into a 10 m finger — Glenn: "really thin
    // parts that stick out").
    const double tTol = kTwinTolerance + 0.6;
    std::vector<std::array<Vec2, 2>> snapTo(lines.size()); std::vector<std::array<bool, 2>> snapped(lines.size(), {false, false});
    std::vector<std::vector<std::vector<double>>> splits(lines.size());
    struct Ref { int line, seg; };
    std::unordered_map<long long, std::vector<Ref>> cells;
    auto key = [](int ix, int iy) { return (static_cast<long long>(ix) << 32) ^ static_cast<long long>(static_cast<unsigned>(iy)); };
    auto cellsOf = [&](const Vec2& a, const Vec2& b, auto&& fn) {
        const int x0 = static_cast<int>(std::floor((std::min(a.x, b.x) - tol) / cell)), x1 = static_cast<int>(std::floor((std::max(a.x, b.x) + tol) / cell));
        const int y0 = static_cast<int>(std::floor((std::min(a.y, b.y) - tol) / cell)), y1 = static_cast<int>(std::floor((std::max(a.y, b.y) + tol) / cell));
        for (int ix = x0; ix <= x1; ++ix) for (int iy = y0; iy <= y1; ++iy) fn(key(ix, iy));
    };
    for (size_t li = 0; li < lines.size(); ++li) {
        splits[li].assign(lines[li].pts.size() > 0 ? lines[li].pts.size() - 1 : 0, {});
        if (lines[li].deck) continue;
        for (size_t si = 0; si + 1 < lines[li].pts.size(); ++si)
            cellsOf(lines[li].pts[si], lines[li].pts[si + 1], [&](long long k) { cells[k].push_back({static_cast<int>(li), static_cast<int>(si)}); });
    }
    for (size_t li = 0; li < lines.size(); ++li) {
        if (lines[li].deck) continue;
        for (size_t si = 0; si + 1 < lines[li].pts.size(); ++si) {
            const Vec2 &p = lines[li].pts[si], &q = lines[li].pts[si + 1];
            std::vector<Ref> seen;
            cellsOf(p, q, [&](long long k) { auto it = cells.find(k); if (it != cells.end()) seen.insert(seen.end(), it->second.begin(), it->second.end()); });
            for (const Ref& o : seen) {
                if (o.line == static_cast<int>(li)) continue;                                   // a line never splits itself
                if (o.line < static_cast<int>(li)) continue;                                    // each pair once
                const Vec2 &a = lines[static_cast<size_t>(o.line)].pts[static_cast<size_t>(o.seg)], &b = lines[static_cast<size_t>(o.line)].pts[static_cast<size_t>(o.seg) + 1];
                double t, u;
                if (lineParams(p, q, a, b, t, u) && t > eps && t < 1 - eps && u > eps && u < 1 - eps) {   // a proper crossing
                    splits[li][si].push_back(t); splits[static_cast<size_t>(o.line)][static_cast<size_t>(o.seg)].push_back(u); continue;
                }
                double d;
                // a vertex of one segment on the interior of the other (a crossing that lands on a subdivision
                // vertex, or a T within the node tolerance): split the other there
                for (const Vec2* ep : {&p, &q}) { const double uu = nearestParam(*ep, a, b, d); if (d < tol && uu > eps && uu < 1 - eps) splits[static_cast<size_t>(o.line)][static_cast<size_t>(o.seg)].push_back(uu); }
                for (const Vec2* ep : {&a, &b}) { const double tt = nearestParam(*ep, p, q, d); if (d < tol && tt > eps && tt < 1 - eps) splits[li][si].push_back(tt); }
                // a LINE END further out, up to the simplification tolerance: split the other AND snap the end onto it
                auto tee = [&](size_t lineA, size_t segA, bool endIsLast, const Vec2& ep, size_t lineB, size_t segB, const Vec2& a2, const Vec2& b2) {
                    const size_t nA = lines[lineA].pts.size(); const bool isEnd = endIsLast ? (segA + 2 == nA) : (segA == 0); if (!isEnd) return;
                    const double uu = nearestParam(ep, a2, b2, d); if (d >= tTol || uu <= eps || uu >= 1 - eps) return;
                    splits[lineB][segB].push_back(uu); const int which = endIsLast ? 1 : 0;
                    if (!snapped[lineA][static_cast<size_t>(which)] || d < (snapTo[lineA][static_cast<size_t>(which)] - ep).length()) { snapped[lineA][static_cast<size_t>(which)] = true; snapTo[lineA][static_cast<size_t>(which)] = Vec2(a2.x + (b2.x - a2.x) * uu, a2.y + (b2.y - a2.y) * uu); }
                };
                tee(li, si, false, p, static_cast<size_t>(o.line), static_cast<size_t>(o.seg), a, b); tee(li, si, true, q, static_cast<size_t>(o.line), static_cast<size_t>(o.seg), a, b);
                tee(static_cast<size_t>(o.line), static_cast<size_t>(o.seg), false, a, li, si, p, q); tee(static_cast<size_t>(o.line), static_cast<size_t>(o.seg), true, b, li, si, p, q);
            }
        }
    }
    for (size_t li = 0; li < lines.size(); ++li) { if (snapped[li][0]) lines[li].pts.front() = snapTo[li][0]; if (snapped[li][1]) lines[li].pts.back() = snapTo[li][1]; }
    // --- the graph ---
    for (size_t li = 0; li < lines.size(); ++li) {
        const Line& L = lines[li]; int prev = -1; double prevS = 0;
        auto node = [&](const Vec2& p, double z, double st) {
            const int n = net.graph.addNode(p, tol);
            if (L.deck) { net.graph.nodes[static_cast<size_t>(n)].elev = z; net.graph.nodes[static_cast<size_t>(n)].elevAbsolute = true; }
            if (prev >= 0 && prev != n) {
                net.graph.addEdge(prev, n, L.w, L.k);
                // outside the edge's lots_range the street is right-of-way: no block frontage, no rim lots
                // (a landing street beyond the freeway; an arterial's run outside the ring). `baked` is
                // exactly what growLotBuildingsOnNets skips as a block/lot source while keeping the
                // clearance band.
                const double mid = 0.5 * (prevS + st);
                if ((L.lotsFrom >= 0 && mid < L.lotsFrom) || (L.lotsTo >= 0 && mid > L.lotsTo)) net.graph.edges.back().baked = true;
            }
            prev = n; prevS = st;
        };
        for (size_t si = 0; si + 1 < L.pts.size(); ++si) {
            node(L.pts[si], L.z[si], L.s[si]);
            std::vector<double>& ts = splits[li][si]; std::sort(ts.begin(), ts.end());
            for (double t : ts) node(Vec2(L.pts[si].x + (L.pts[si + 1].x - L.pts[si].x) * t, L.pts[si].y + (L.pts[si + 1].y - L.pts[si].y) * t), L.z[si] + (L.z[si + 1] - L.z[si]) * t, L.s[si] + (L.s[si + 1] - L.s[si]) * t);
        }
        if (!L.pts.empty()) node(L.pts.back(), L.z.back(), L.s.back());
    }
    // Short-edge cleanup. The lot pass's sidewalk inset rejects a whole face when any edge is
    // shorter than the offset can turn around, and the twin manufactures such edges: a crossing
    // node inserted a metre from one of our own corner nodes, or two junctions that all but
    // coincide. Metro at the 5 m margin: 16 of 88 faces died on edges of 0.5–7 m. Corners are
    // ours to drop (they only approximate the curve), so a corner within 6 m of a junction or an
    // end goes, and junctions closer than 2.5 m merge at their midpoint.
    {
        auto& N = net.graph.nodes; auto& E = net.graph.edges;
        for (int pass = 0; pass < 400; ++pass) {
            std::vector<std::vector<int>> inc(N.size());
            for (size_t ei = 0; ei < E.size(); ++ei) { inc[static_cast<size_t>(E[ei].a)].push_back(static_cast<int>(ei)); inc[static_cast<size_t>(E[ei].b)].push_back(static_cast<int>(ei)); }
            bool changed = false;
            // 1. corners near junctions or ends: splice the two edges into one
            for (size_t v = 0; v < N.size() && !changed; ++v) {
                if (inc[v].size() != 2) continue;
                const RoadEdge& e0 = E[static_cast<size_t>(inc[v][0])]; const RoadEdge& e1 = E[static_cast<size_t>(inc[v][1])];
                const int u0 = e0.a == static_cast<int>(v) ? e0.b : e0.a, u1 = e1.a == static_cast<int>(v) ? e1.b : e1.a; if (u0 == u1) continue;
                const double d0 = (N[static_cast<size_t>(u0)].pos - N[v].pos).length(), d1 = (N[static_cast<size_t>(u1)].pos - N[v].pos).length();
                const bool near0 = d0 < 6.0 && inc[static_cast<size_t>(u0)].size() != 2, near1 = d1 < 6.0 && inc[static_cast<size_t>(u1)].size() != 2;
                if (!near0 && !near1) continue;
                RoadEdge merged = e0; merged.a = u0; merged.b = u1; merged.baked = e0.baked || e1.baked;
                const int i0 = inc[v][0], i1 = inc[v][1];
                E[static_cast<size_t>(i0)] = merged; E.erase(E.begin() + i1); changed = true;
            }
            if (changed) continue;
            // 2. a stub: a dead-end edge shorter than 8 m (a street end that overshot the T it was split
            //    into by less than the node tolerance); the face would walk it out and back as a slit
            for (size_t ei = 0; ei < E.size() && !changed; ++ei) {
                const RoadEdge& e = E[ei]; if (inc[static_cast<size_t>(e.a)].size() != 1 && inc[static_cast<size_t>(e.b)].size() != 1) continue;
                if ((N[static_cast<size_t>(e.a)].pos - N[static_cast<size_t>(e.b)].pos).length() >= 8.0) continue;
                E.erase(E.begin() + static_cast<long>(ei)); changed = true;
            }
            if (changed) continue;
            // 3. two junctions closer than 8 m: contract the edge between them (the 5 m inset needs
            //    ~10 m between two right-angle turns; the geometry moves by at most 4 m)
            for (size_t ei = 0; ei < E.size() && !changed; ++ei) {
                const RoadEdge& e = E[ei]; if (inc[static_cast<size_t>(e.a)].size() < 3 || inc[static_cast<size_t>(e.b)].size() < 3) continue;
                if ((N[static_cast<size_t>(e.a)].pos - N[static_cast<size_t>(e.b)].pos).length() >= 8.0) continue;
                const int keep = e.a, drop = e.b; N[static_cast<size_t>(keep)].pos = (N[static_cast<size_t>(keep)].pos + N[static_cast<size_t>(drop)].pos) * 0.5;
                E.erase(E.begin() + static_cast<long>(ei));
                for (RoadEdge& f : E) { if (f.a == drop) f.a = keep; if (f.b == drop) f.b = keep; }
                E.erase(std::remove_if(E.begin(), E.end(), [](const RoadEdge& f) { return f.a == f.b; }), E.end());
                changed = true;
            }
            if (!changed) break;
        }
    }
    // Dead ends left after that: bridge each street end to the nearest other street within 20 m, the way
    // the engine's joinDanglingEnds does for its own nets, so no spur pokes into a block face.
    if (std::getenv("RT_TWIN_NO_BRIDGE") == nullptr) {
        auto& N = net.graph.nodes; auto& E = net.graph.edges;
        std::vector<int> deg(N.size(), 0); for (const RoadEdge& e : E) { ++deg[static_cast<size_t>(e.a)]; ++deg[static_cast<size_t>(e.b)]; }
        const size_t nEdges = E.size();
        for (size_t v = 0; v < deg.size(); ++v) {
            if (deg[v] != 1) continue;
            int own = -1; for (size_t ei = 0; ei < nEdges; ++ei) if (E[ei].a == static_cast<int>(v) || E[ei].b == static_cast<int>(v)) { own = static_cast<int>(ei); break; }
            if (own < 0 || E[static_cast<size_t>(own)].klass == RoadClass::Freeway || E[static_cast<size_t>(own)].klass == RoadClass::Ramp || E[static_cast<size_t>(own)].baked) continue;
            const int far = E[static_cast<size_t>(own)].a == static_cast<int>(v) ? E[static_cast<size_t>(own)].b : E[static_cast<size_t>(own)].a;
            int bestE = -1; double bestD = 20.0, bestT = 0;
            for (size_t ei = 0; ei < nEdges; ++ei) {
                const RoadEdge& e = E[ei]; if (static_cast<int>(ei) == own || e.a == far || e.b == far || e.klass == RoadClass::Freeway || e.klass == RoadClass::Ramp) continue;
                const Vec2 a = N[static_cast<size_t>(e.a)].pos, b = N[static_cast<size_t>(e.b)].pos; double d; const double tt = nearestParam(N[v].pos, a, b, d);
                if (d < bestD && tt > 0.02 && tt < 0.98) { bestD = d; bestE = static_cast<int>(ei); bestT = tt; }
            }
            if (bestE < 0) continue;
            const RoadEdge hit = E[static_cast<size_t>(bestE)]; const Vec2 a = N[static_cast<size_t>(hit.a)].pos, b = N[static_cast<size_t>(hit.b)].pos;
            const int mid = net.graph.addNode(Vec2(a.x + (b.x - a.x) * bestT, a.y + (b.y - a.y) * bestT), 0.5);
            if (mid == static_cast<int>(v)) continue;
            RoadEdge e1 = hit; e1.b = mid; RoadEdge e2 = hit; e2.a = mid; E[static_cast<size_t>(bestE)] = e1; E.push_back(e2);
            RoadEdge bridge = E[static_cast<size_t>(own)]; bridge.a = static_cast<int>(v); bridge.b = mid; E.push_back(bridge);
            deg.resize(N.size(), 0); deg[v] = 2; deg[static_cast<size_t>(mid)] += 3;
        }
    }
    // Pin the chords STRAIGHT for the engine's sampler. sampleNetGraph gives a node with no stored
    // tangent and exactly two neighbours a Catmull-Rom tangent, so a corner-only polyline became a
    // spline bowing metres off the built street, and the lot pass kept blocks clear of the bowed
    // ribbon instead of the real one (metro: 40 of 70 blocks on the asphalt). A stored tangent of
    // negligible magnitude makes both Hermite segments at the corner degenerate to their chords,
    // and the sampler's straight-run collapse then keeps exactly our nodes.
    {
        std::vector<std::vector<int>> nbr(net.graph.nodes.size());
        for (const RoadEdge& e : net.graph.edges) { nbr[static_cast<size_t>(e.a)].push_back(e.b); nbr[static_cast<size_t>(e.b)].push_back(e.a); }
        for (size_t i = 0; i < nbr.size(); ++i) {
            if (nbr[i].size() != 2) continue;
            Vec2 d = net.graph.nodes[static_cast<size_t>(nbr[i][1])].pos - net.graph.nodes[static_cast<size_t>(nbr[i][0])].pos;
            const double L = std::hypot(d.x, d.y); if (L < 1e-9) continue;
            net.graph.nodes[i].tangent = Vec2(d.x / L * 1e-3, d.y / L * 1e-3);
        }
    }
    return net;
}

}  // namespace lanelab
}  // namespace engine
