#include "engine/procgen/city/roads/lanes/lane_graph_build.h"

#include "engine/procgen/city/polygon.h"                     // convexHull, pointInPolygon
#include "engine/procgen/city/roads/lanes/deck_mesh.h"       // the junction boxes the paint finds
#include "engine/procgen/city/roads/lanes/polyline_ops.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <numeric>
#include <utility>

namespace engine {
namespace roads::lanes {

namespace {

// Two cuts along a corridor closer than this are one cut (metres).
constexpr double kCutTolerance = 1e-3;
// A lane ending nearer than this to where another begins, but not within the join tolerances, is reported (metres) ...
constexpr double kNearMiss = 0.5;
// ... when the two run within this of the same way (degrees): a lane beside, not across.
constexpr double kNearMissDegrees = 45.0;
// Centre-line points closer together than this are passed over when a bend is measured: too short to measure (metres).
constexpr double kMinBendChord = 0.05;
// A cut this close to a centre-line point drops that point rather than leave a sliver segment (metres).
constexpr double kSliceGap = 1e-3;
// How many times junctions found overlapping along a road are merged and their outlines drawn again, at most.
constexpr int kMergeRounds = 8;
constexpr double kDegreesPerRadian = 57.29577951308232;
// Room for one problem's words.
constexpr std::size_t kMessageBytes = 512;

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 2, 3)))
#endif
void addProblem(std::vector<std::string>& to, const char* f, ...) {
    char msg[kMessageBytes];
    va_list args;
    va_start(args, f);
    std::vsnprintf(msg, sizeof msg, f, args);
    va_end(args);
    to.push_back(msg);
}

// WHICH WAY IS THE OFFSIDE, for a lane `normalOffset` from its road's spine (along the spine's left normal, perp())
// running `dir` (+1 along the spine, -1 against it): the larger the result, the nearer the middle of the road. The
// builder lays a road's forward lanes on the negative side of the spine's left normal and its backward lanes on the
// positive side, and names each direction's lanes from the median outwards (laneSlots: f0 and b0 are the innermost; a
// pocket's "inner" lane is the one with the largest offset x dir), so offset x dir grows toward the middle. In the
// world the plan is mirrored (plan y is world z), so a forward lane on the plan's right of its spine is on the driver's
// left: that is how this engine's roads keep left. Nothing else here says which side is which.
double offsideness(double normalOffset, int dir) { return normalOffset * static_cast<double>(dir); }

// A builder lane read the way cars drive it: its points, the distance along it to each, and how far along its own road
// (the way it runs) each was laid (empty when the builder did not lay it point by point along its road's spine).
struct TravelLane {
    std::vector<Vec2> xy;
    std::vector<double> s;
    std::vector<double> u;
};

// A lane point laid across its road's spine stands this far from the spine's normal there at most (metres): rounding.
constexpr double kLaidTolerance = 1e-6;

// WHICH SPINE SAMPLE A ROAD'S LANE WAS LAID FROM: the builder lays a lane one point per sample of its road's spine, each
// across the spine's normal there (lane_expand.cpp ribbonLane), starting at some sample (a turn pocket starts part way
// along, a ramp's second lane stops part way). Returns that first sample, or -1 when the lane was not laid that way.
int firstSpineSample(const std::vector<Vec2>& laneXy, const std::vector<Vec2>& spine, const std::vector<Vec2>& tangent) {
    if (laneXy.empty() || laneXy.size() > spine.size() || tangent.size() != spine.size()) return -1;
    std::size_t best = 0;
    double bestAlong = 1e300;
    for (std::size_t i = 0; i + laneXy.size() <= spine.size(); ++i) {
        const double along = std::fabs(dot(laneXy[0] - spine[i], tangent[i]));
        if (along < bestAlong) {
            bestAlong = along;
            best = i;
        }
    }
    for (std::size_t k = 0; k < laneXy.size(); ++k)
        if (!(std::fabs(dot(laneXy[k] - spine[best + k], tangent[best + k])) <= kLaidTolerance)) return -1;
    return static_cast<int>(best);
}

// A STRETCH OF A BUILDER LANE IN ONE CORRIDOR: [from, to] in metres along the lane the way cars drive it; per point of
// the stretch, that distance (tau) and the distance along the corridor (u, never decreasing).
struct Member {
    int lane = -1;
    double from = 0, to = 0;
    std::vector<double> tau, u;
    double u0() const { return u.front(); }
    double u1() const { return u.back(); }
    double tauAt(double x) const { return interp(u, tau, x); }
    double uAt(double t) const { return interp(tau, u, t); }
    // The lane's distance at a cut: exactly its own end where the cut is that end.
    double tauAtCut(double c) const {
        if (std::fabs(c - u0()) <= kCutTolerance) return from;
        if (std::fabs(c - u1()) <= kCutTolerance) return to;
        return tauAt(c);
    }
};

// A CORRIDOR: one road's lanes running one way (and the ramp lanes running beside them), u measured along the road's
// spine in the way the traffic runs.
struct Corridor {
    int edge = -1, dir = 1;
    double length = 0;
    std::vector<Member> members;
};
using CorridorKey = std::pair<int, int>;   // (edge, 0 for the way along the spine, 1 against it)
CorridorKey corridorKey(int edge, int dir) { return CorridorKey(edge, dir > 0 ? 0 : 1); }

// A STRETCH OF A CORRIDOR INSIDE A JUNCTION: [ua, ub] belongs to junction `junction`. `stopLineAt*`: that end is a stop
// line of the corridor's own road (it has a junction box there), not where its lanes cross the junction's outline.
struct Interval {
    double ua = 0, ub = 0;
    int junction = -1;
    bool stopLineAtStart = true, stopLineAtEnd = true;
};

// One of the paint's junction boxes, on its street.
struct Box {
    int edge = -1;
    JunctionSpan span;
    double sa = 0, sb = 0;   // the stretch of the street's spine it takes, stop line to stop line (clamped to the road)
};

int findRoot(std::vector<int>& parent, int i) {
    while (parent[static_cast<std::size_t>(i)] != i) {
        parent[static_cast<std::size_t>(i)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(i)])];
        i = parent[static_cast<std::size_t>(i)];
    }
    return i;
}
// Joins two sets (the lower root stays the root); false when they were one already.
bool unite(std::vector<int>& parent, int a, int b) {
    a = findRoot(parent, a);
    b = findRoot(parent, b);
    if (a == b) return false;
    parent[static_cast<std::size_t>(std::max(a, b))] = std::min(a, b);
    return true;
}

// THE TIGHTEST BEND of a centre line: the smallest radius of the circle through each three consecutive points (points
// closer than kMinBendChord to the one kept before are passed over), radii of kStraightEnoughRadius and more read as
// straight. kNoBend when nothing bends.
double tightestRadius(const std::vector<Vec2>& pts) {
    std::vector<Vec2> q;
    for (const Vec2& p : pts)
        if (q.empty() || distance(q.back(), p) >= kMinBendChord) q.push_back(p);
    if (q.size() >= 2 && distance(q.back(), pts.back()) > 0) q.back() = pts.back();
    double best = kNoBend;
    for (std::size_t k = 1; k + 1 < q.size(); ++k) {
        const double r = circleRadius(q[k - 1], q[k], q[k + 1]);
        if (r < kStraightEnoughRadius) best = std::min(best, r);
    }
    return best;
}

// Where an outline's edge crosses the segment a -> b, as a fraction along it (the first crossing), or -1.
double outlineCrossing(const std::vector<Vec2>& outline, const Vec2& a, const Vec2& b) {
    double best = -1;
    for (std::size_t i = 0; i < outline.size(); ++i) {
        Vec2 at;
        if (!segmentsCross(a, b, outline[i], outline[(i + 1) % outline.size()], at)) continue;
        const double len = distance(a, b);
        const double f = len > 0 ? distance(a, at) / len : 0;
        if (best < 0 || f < best) best = f;
    }
    return best;
}

// Whether a closed outline crosses itself (two of its edges that are not neighbours cross).
bool crossesItself(const std::vector<Vec2>& ring) {
    const std::size_t n = ring.size();
    for (std::size_t a = 0; a < n; ++a)
        for (std::size_t b = a + 2; b < n; ++b) {
            if ((b + 1) % n == a) continue;   // neighbours share a corner
            Vec2 hit;
            if (segmentsCross(ring[a], ring[(a + 1) % n], ring[b], ring[(b + 1) % n], hit)) return true;
        }
    return false;
}

struct Bounds {
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    void add(const Vec2& p) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
    bool overlaps(const Bounds& o) const { return !(o.x0 > x1 || o.x1 < x0 || o.y0 > y1 || o.y1 < y0); }
};

}  // namespace

LaneGraphBuild buildLaneGraph(const Result& r, const LaneGraphOptions& opts) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    LaneGraphBuild out;
    LaneGraph& G = out.graph;
    const RoadLabGraph& g = r.graph;
    const LaneSet& L = r.lanes;
    const DeckHeight& H = *r.heights;
    const std::size_t nLanes = L.lanes.size();
    out.insideJunctions.assign(nLanes, 0.0);
    out.leftOut.assign(nLanes, 0);
    auto edgeAt = [&](int e) -> const EdgeSpec& { return g.edges[static_cast<std::size_t>(e)]; };
    auto laneAt = [&](int li) -> const Lane& { return L.lanes[static_cast<std::size_t>(li)]; };
    auto way = [](int dir) { return dir > 0 ? "along its spine" : "against its spine"; };

    // ---- which builder lanes become lane paths, and the names --------------------------------------------------------
    std::vector<char> kept(nLanes, 0);
    for (std::size_t li = 0; li < nLanes; ++li) {
        const Lane& l = L.lanes[li];
        if (l.isConnector()) {
            out.leftOut[li] = 1;
            ++out.connectorsLeftOut;
            continue;
        }
        const bool carsDrive = l.parent >= 0 && !g.cls(edgeAt(l.parent)).track && !g.cls(edgeAt(l.parent)).footway;
        if (!carsDrive || l.xy.size() < 2 || l.s.empty() || !(l.s.back() > 0)) {
            out.leftOut[li] = 1;
            ++out.otherLanesLeftOut;
            continue;
        }
        kept[li] = 1;
        G.laneTagNames.push_back(l.kind);
    }
    std::sort(G.laneTagNames.begin(), G.laneTagNames.end());
    G.laneTagNames.erase(std::unique(G.laneTagNames.begin(), G.laneTagNames.end()), G.laneTagNames.end());
    G.vehicleTagNames = opts.vehicleTags;
    std::sort(G.vehicleTagNames.begin(), G.vehicleTagNames.end());
    G.vehicleTagNames.erase(std::unique(G.vehicleTagNames.begin(), G.vehicleTagNames.end()), G.vehicleTagNames.end());
    G.controlNames = {opts.control};
    for (const std::pair<const std::string, RoadClassSpec>& c : g.classes) G.roadClassNames.push_back(c.first);
    TagMask everyone = 0;
    for (std::size_t i = 0; i < G.vehicleTagNames.size() && i < kMaxTagNames; ++i) everyone |= TagMask(1) << i;
    if (G.laneTagNames.size() > kMaxTagNames)
        addProblem(out.problems, "%zu lane kinds, more than the %u a lane's tags can hold", G.laneTagNames.size(), kMaxTagNames);

    // ---- every kept lane the way cars drive it, and how far along its own road each point was laid ------------------
    std::vector<TravelLane> travel(nLanes);
    std::map<int, std::vector<Vec2>> spineTangents;   // per road, as the builder framed it
    for (std::size_t li = 0; li < nLanes; ++li) {
        if (!kept[li]) continue;
        const Lane& l = L.lanes[li];
        const EdgeSpec& e = edgeAt(l.parent);
        TravelLane& T = travel[li];
        T.xy = l.xy;
        if (l.dir < 0) std::reverse(T.xy.begin(), T.xy.end());
        T.s = stations(T.xy);
        std::vector<Vec2>& tangent = spineTangents[l.parent];
        if (tangent.empty()) {
            std::vector<Vec2> normal;
            frames(e.xy, tangent, normal);
        }
        const int first = firstSpineSample(l.xy, e.xy, tangent);
        if (first < 0) continue;   // not laid point by point: measured by projection instead (addMember)
        const std::size_t n = l.xy.size();
        double back = 0;
        Vec2 backAt;
        for (std::size_t k = 0; k < n; ++k) {
            const std::size_t i = static_cast<std::size_t>(first) + (l.dir > 0 ? k : n - 1 - k);
            T.u.push_back(l.dir > 0 ? e.s[i] : e.s.back() - e.s[i]);
            // a lane on the inside of a sharp corner of its road runs backwards for a stretch there
            if (k > 0 && dot(T.xy[k] - T.xy[k - 1], tangent[i] * static_cast<double>(l.dir)) < 0) {
                back += distance(T.xy[k], T.xy[k - 1]);
                if (backAt.x == 0 && backAt.y == 0) backAt = T.xy[k];
            }
        }
        if (back > kCutTolerance)
            addProblem(out.problems, "lane %s runs backwards for %.2f m near (%.1f, %.1f): it lies on the inside of a corner of "
                                     "road %s sharper than its offset from the road's spine", l.id.c_str(), back, backAt.x,
                       backAt.y, e.id.c_str());
    }

    // ---- corridors and their members ---------------------------------------------------------------------------------
    std::map<CorridorKey, Corridor> corridors;
    // A member: the stretch [from, to] of lane li, measured along the corridor of `edge` running `dir`.
    auto addMember = [&](int li, int edge, int dir, double from, double to) {
        if (!(to - from > kCutTolerance)) return;
        const TravelLane& T = travel[static_cast<std::size_t>(li)];
        const EdgeSpec& e = edgeAt(edge);
        Corridor& c = corridors[corridorKey(edge, dir)];
        c.edge = edge;
        c.dir = dir;
        c.length = e.s.back();
        Member m;
        m.lane = li;
        m.from = from;
        m.to = to;
        const std::vector<Vec2> pts = sliceAtStations(T.xy, T.s, from, to, kSliceGap);
        const std::vector<double> along = stations(pts);
        // along its own road: where the builder laid each point; along another (a ramp's lane beside its host): the station
        // it lies across, as the builder offsets a ramp's lane from its host's spine
        const bool laid = edge == laneAt(li).parent && !T.u.empty();
        double worstBack = 0;
        Vec2 worstAt;
        for (std::size_t k = 0; k < pts.size(); ++k) {
            const double tau = k + 1 == pts.size() ? to : from + along[k];
            double u = 0;
            if (laid) {
                u = interp(T.s, T.u, tau);
            } else {
                const double sigma = stationAcross(e.xy, e.s, pts[k]);
                u = dir > 0 ? sigma : c.length - sigma;
            }
            if (!m.u.empty() && u < m.u.back()) {
                if (m.u.back() - u > worstBack) {
                    worstBack = m.u.back() - u;
                    worstAt = pts[k];
                }
                u = m.u.back();
            }
            m.tau.push_back(tau);
            m.u.push_back(u);
        }
        if (worstBack > kCutTolerance)
            addProblem(out.problems, "lane %s runs backwards along road %s by up to %.2f m near (%.1f, %.1f), measured on that "
                                     "road's spine", laneAt(li).id.c_str(), e.id.c_str(), worstBack, worstAt.x, worstAt.y);
        c.members.push_back(std::move(m));
    };
    // The road and direction of the host lane a ramp is anchored on, or false.
    auto hostOf = [&](const EdgeSpec& ramp, const char* end, int& edge, int& dir) {
        const std::map<std::string, std::string>::const_iterator it = ramp.anchorLanes.find(end);
        if (it == ramp.anchorLanes.end()) return false;
        const int hl = L.find(it->second);
        if (hl < 0 || !kept[static_cast<std::size_t>(hl)]) return false;
        edge = laneAt(hl).parent;
        dir = laneAt(hl).dir;
        return true;
    };
    for (std::size_t li = 0; li < nLanes; ++li) {
        if (!kept[li]) continue;
        const Lane& l = L.lanes[li];
        const EdgeSpec& e = edgeAt(l.parent);
        const TravelLane& T = travel[li];
        const double len = T.s.back();
        if (!e.isRamp()) {
            addMember(static_cast<int>(li), l.parent, l.dir, 0, len);
            continue;
        }
        // A ramp's lane runs beside its host before the gore it leaves by and after the gore it joins at (the builder's
        // anchors): those stretches are the host's corridor's, the rest the ramp's own.
        // (how far along the ramp's spine each point lies: where it was laid, else projected)
        std::vector<double> sigma = T.u;
        if (sigma.size() != T.xy.size()) {
            sigma.clear();
            for (const Vec2& p : T.xy) sigma.push_back(project(e.xy, e.s, p).station);
            for (std::size_t k = 1; k < sigma.size(); ++k) sigma[k] = std::max(sigma[k], sigma[k - 1]);
        }
        int fromEdge = -1, fromDir = 1, toEdge = -1, toDir = 1;
        const bool hasFrom = e.anchorIdx[0] >= 0 && hostOf(e, "from", fromEdge, fromDir);
        const bool hasTo = e.anchorIdx[1] >= 0 && hostOf(e, "to", toEdge, toDir);
        const double tFrom = hasFrom ? interp(sigma, T.s, e.s[static_cast<std::size_t>(e.anchorIdx[0])]) : 0.0;
        const double tTo = hasTo ? interp(sigma, T.s, e.s[static_cast<std::size_t>(e.anchorIdx[1])]) : len;
        if (hasFrom) addMember(static_cast<int>(li), fromEdge, fromDir, 0, tFrom);
        addMember(static_cast<int>(li), l.parent, l.dir, tFrom, std::max(tFrom, tTo));
        if (hasTo) addMember(static_cast<int>(li), toEdge, toDir, std::max(tFrom, tTo), len);
        if ((e.anchorIdx[0] >= 0 && !hasFrom) || (e.anchorIdx[1] >= 0 && !hasTo))
            addProblem(out.problems, "ramp %s: the host lane it is anchored on is not a lane cars drive; its lane keeps to "
                                     "its own road", e.id.c_str());
    }
    // A member's point and its left normal at corridor distance u.
    auto memberPoint = [&](const Member& m, double u, Vec2& p, Vec2& n) {
        const TravelLane& T = travel[static_cast<std::size_t>(m.lane)];
        const double t = m.tauAtCut(u);
        p = pointAt(T.xy, T.s, t);
        n = perp(tangentAtStation(T.xy, T.s, t));
    };
    auto covers = [](const Member& m, double u) { return u >= m.u0() - kCutTolerance && u <= m.u1() + kCutTolerance; };

    // ---- junctions: the paint's boxes, one junction per crossing ----------------------------------------------------
    const std::chrono::steady_clock::time_point tBoxes = std::chrono::steady_clock::now();
    const LaneLookup look(r.pavement);
    const std::map<int, std::vector<JunctionSpan>> spans = junctionSpans(r, look);
    out.boxMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tBoxes).count();
    std::vector<Box> boxes;
    std::map<int, std::pair<std::size_t, std::size_t>> boxesOfEdge;   // edge -> (first box, count)
    for (const std::pair<const int, std::vector<JunctionSpan>>& kv : spans) {
        const EdgeSpec& e = edgeAt(kv.first);
        if (g.cls(e).track || g.cls(e).footway) continue;
        boxesOfEdge[kv.first] = std::make_pair(boxes.size(), kv.second.size());
        for (const JunctionSpan& sp : kv.second) {
            Box b;
            b.edge = kv.first;
            b.span = sp;
            b.sa = std::max(0.0, sp.s0 - kStopLineBack);
            b.sb = std::min(e.s.back(), sp.s1 + kStopLineBack);
            boxes.push_back(b);
        }
    }
    std::vector<int> root(boxes.size());
    std::iota(root.begin(), root.end(), 0);
    // two streets' boxes are one junction where each street's box lies on the other street's
    for (std::size_t a = 0; a < boxes.size(); ++a) {
        const EdgeSpec& ea = edgeAt(boxes[a].edge);
        const Vec2 mid = pointAt(ea.xy, ea.s, 0.5 * (boxes[a].span.s0 + boxes[a].span.s1));
        for (int e2 : boxes[a].span.crossEdges) {
            const std::map<int, std::pair<std::size_t, std::size_t>>::const_iterator it = boxesOfEdge.find(e2);
            if (it == boxesOfEdge.end()) continue;
            const EdgeSpec& eb = edgeAt(e2);
            const double st = project(eb.xy, eb.s, mid).station;
            for (std::size_t b = it->second.first; b < it->second.first + it->second.second; ++b)
                if (st >= boxes[b].span.s0 - kJunctionMergeGap && st <= boxes[b].span.s1 + kJunctionMergeGap)
                    unite(root, static_cast<int>(a), static_cast<int>(b));
        }
    }
    // ...and boxes whose stop-line stretches overlap along one street are one junction too
    for (std::size_t b = 1; b < boxes.size(); ++b)
        if (boxes[b].edge == boxes[b - 1].edge && boxes[b].sa < boxes[b - 1].sb)
            if (unite(root, static_cast<int>(b - 1), static_cast<int>(b))) ++out.mergedJunctions;

    std::vector<int> junctionOfBox;
    std::vector<std::vector<std::size_t>> boxesOfJunction;
    std::vector<std::vector<Vec2>> outline;
    std::vector<Bounds> outlineBounds;
    std::map<CorridorKey, std::vector<Interval>> intervals;
    // Whether point p of lane li is at the level of junction j's streets.
    auto atJunctionLevel = [&](std::size_t j, int li, const Vec2& p) {
        const double z = H.deck(li, p);
        for (std::size_t b : boxesOfJunction[j])
            if (std::fabs(H.deckRoad(boxes[b].edge, p) - z) <= g.rules.sameLevelDz) return true;
        return false;
    };
    for (int round = 0; round < kMergeRounds; ++round) {
        // junction ids in the order of their first box (by street, then station)
        junctionOfBox.assign(boxes.size(), -1);
        boxesOfJunction.clear();
        std::vector<int> idOfRoot(boxes.size(), -1);
        for (std::size_t b = 0; b < boxes.size(); ++b) {
            const std::size_t rb = static_cast<std::size_t>(findRoot(root, static_cast<int>(b)));
            if (idOfRoot[rb] < 0) {
                idOfRoot[rb] = static_cast<int>(boxesOfJunction.size());
                boxesOfJunction.emplace_back();
            }
            junctionOfBox[b] = idOfRoot[rb];
            boxesOfJunction[static_cast<std::size_t>(idOfRoot[rb])].push_back(b);
        }
        // the outline: round the stop lines, each one an edge of it, joined corner to corner in order round the
        // junction (a convex hull would leave a skewed street's stop line inside it); with fewer than two stop lines,
        // the convex hull of them and of the road ends inside the junction
        outline.assign(boxesOfJunction.size(), {});
        outlineBounds.assign(boxesOfJunction.size(), Bounds());
        for (std::size_t j = 0; j < boxesOfJunction.size(); ++j) {
            struct Leg { Vec2 a, b, mid; };
            std::vector<Leg> legs;
            std::vector<Vec2> all;
            for (std::size_t b : boxesOfJunction[j]) {
                const EdgeSpec& e = edgeAt(boxes[b].edge);
                for (double s : {boxes[b].sa, boxes[b].sb}) {
                    const bool stopLine = s > kCutTolerance && s < e.s.back() - kCutTolerance;
                    const Vec2 spineAt = pointAt(e.xy, e.s, s), spineNormal = perp(tangentAtStation(e.xy, e.s, s));
                    // every lane cut there, across the road: the stop line runs through their centres (which the builder
                    // laid on one line across the road) out to the outer lanes' edges
                    struct Cut { double lat; Vec2 centre, normal; double half; };
                    std::vector<Cut> cuts;
                    for (int dir : {1, -1}) {
                        const std::map<CorridorKey, Corridor>::const_iterator it = corridors.find(corridorKey(boxes[b].edge, dir));
                        if (it == corridors.end()) continue;
                        const double u = dir > 0 ? s : it->second.length - s;
                        for (const Member& m : it->second.members) {
                            if (!covers(m, u)) continue;
                            Cut cut;
                            memberPoint(m, u, cut.centre, cut.normal);
                            cut.half = laneAt(m.lane).w / 2;
                            cut.lat = dot(cut.centre - spineAt, spineNormal);
                            cuts.push_back(cut);
                            all.push_back(cut.centre + cut.normal * cut.half);
                            all.push_back(cut.centre - cut.normal * cut.half);
                        }
                    }
                    if (!stopLine || cuts.empty()) continue;
                    std::sort(cuts.begin(), cuts.end(), [](const Cut& x, const Cut& y) { return x.lat < y.lat; });
                    const Cut& lo = cuts.front();
                    const Cut& hi = cuts.back();
                    const Vec2 across = distance(hi.centre, lo.centre) > kCutTolerance ? normalize(hi.centre - lo.centre)
                                                                                      : lo.normal * (dot(lo.normal, spineNormal) < 0 ? -1.0 : 1.0);
                    Leg leg;
                    leg.a = lo.centre - across * lo.half;
                    leg.b = hi.centre + across * hi.half;
                    leg.mid = (leg.a + leg.b) * 0.5;
                    legs.push_back(leg);
                }
            }
            if (legs.size() >= 2) {
                Vec2 centre;
                for (const Leg& leg : legs) centre += leg.mid;
                centre = centre / static_cast<double>(legs.size());
                for (Leg& leg : legs)
                    if (cross(leg.a - centre, leg.b - centre) < 0) std::swap(leg.a, leg.b);   // a to b runs anticlockwise
                std::sort(legs.begin(), legs.end(), [&](const Leg& x, const Leg& y) {
                    return std::atan2(x.mid.y - centre.y, x.mid.x - centre.x) < std::atan2(y.mid.y - centre.y, y.mid.x - centre.x);
                });
                for (const Leg& leg : legs) {
                    outline[j].push_back(leg.a);
                    outline[j].push_back(leg.b);
                }
            } else {
                outline[j] = convexHull(all);
            }
            for (const Vec2& p : outline[j]) outlineBounds[j].add(p);
        }
        // each corridor's stretches inside junctions: its own road's boxes, stop line to stop line ...
        intervals.clear();
        for (std::size_t b = 0; b < boxes.size(); ++b)
            for (int dir : {1, -1}) {
                const CorridorKey key = corridorKey(boxes[b].edge, dir);
                const std::map<CorridorKey, Corridor>::const_iterator it = corridors.find(key);
                if (it == corridors.end()) continue;
                Interval iv;
                iv.junction = junctionOfBox[b];
                iv.ua = dir > 0 ? boxes[b].sa : it->second.length - boxes[b].sb;
                iv.ub = dir > 0 ? boxes[b].sb : it->second.length - boxes[b].sa;
                intervals[key].push_back(iv);
            }
        // ... and wherever a lane runs into a junction's outline at its level, deeper than kJunctionEndReach: a road with
        // no box of its own there (a ramp ending on a street, a freeway crossing at grade), or a street's stub between
        // its box and its end that the outline takes in (a street kinked at a corner of the junction). Not where an
        // outline crosses itself: in and out mean nothing there.
        for (const std::pair<const CorridorKey, Corridor>& kc : corridors) {
            const Corridor& c = kc.second;
            for (std::size_t j = 0; j < boxesOfJunction.size(); ++j) {
                if (outline[j].size() < 3 || crossesItself(outline[j])) continue;
                for (const Member& m : c.members) {
                    const TravelLane& T = travel[static_cast<std::size_t>(m.lane)];
                    const std::vector<Vec2> pts = sliceAtStations(T.xy, T.s, m.from, m.to, kSliceGap);
                    Bounds mb;
                    for (const Vec2& p : pts) mb.add(p);
                    if (!mb.overlaps(outlineBounds[j])) continue;
                    const std::vector<double> along = stations(pts);
                    double runStart = -1, depth = 0;
                    bool wasIn = false;
                    auto endRun = [&](double t) {
                        Interval iv;
                        iv.ua = m.uAt(runStart);
                        iv.ub = m.uAt(t);
                        iv.junction = static_cast<int>(j);
                        iv.stopLineAtStart = iv.stopLineAtEnd = false;
                        if (depth > kJunctionEndReach && iv.ub - iv.ua > kCutTolerance) intervals[kc.first].push_back(iv);
                    };
                    for (std::size_t k = 0; k < pts.size(); ++k) {
                        const bool in = pointInPolygon(outline[j], pts[k]) && atJunctionLevel(j, m.lane, pts[k]);
                        double t = m.from + along[k];
                        if (k > 0 && in != wasIn) {   // where it crossed: on the outline, or halfway for a change of level
                            const double f = outlineCrossing(outline[j], pts[k - 1], pts[k]);
                            t = m.from + along[k - 1] + (f >= 0 ? f : 0.5) * (along[k] - along[k - 1]);
                        }
                        if (in && !wasIn) {
                            runStart = k == 0 ? m.from : t;
                            depth = 0;
                        }
                        if (in) depth = std::max(depth, distToPolyline(pts[k], outline[j], true));
                        if (!in && wasIn) endRun(t);
                        wasIn = in;
                    }
                    if (wasIn) endRun(m.to);
                }
            }
        }
        // one stretch per junction along a corridor; two junctions overlapping along one are one junction
        bool merged = false;
        for (std::pair<const CorridorKey, std::vector<Interval>>& ki : intervals) {
            std::vector<Interval>& list = ki.second;
            std::sort(list.begin(), list.end(), [](const Interval& a, const Interval& b) {
                if (a.ua != b.ua) return a.ua < b.ua;
                if (a.junction != b.junction) return a.junction < b.junction;
                return a.stopLineAtStart && !b.stopLineAtStart;
            });
            std::vector<Interval> one;
            for (const Interval& iv : list) {
                // a road that leaves a junction and comes back into the same one within the builder's one-junction gap
                // (a street kinked across a corner) is in it all the way
                const bool backInto = !one.empty() && iv.junction == one.back().junction && iv.ua < one.back().ub + kJunctionMergeGap;
                if (!one.empty() && (iv.ua < one.back().ub - kCutTolerance || backInto)) {
                    if (iv.junction != one.back().junction) {
                        const int a = static_cast<int>(boxesOfJunction[static_cast<std::size_t>(iv.junction)].front());
                        const int b = static_cast<int>(boxesOfJunction[static_cast<std::size_t>(one.back().junction)].front());
                        if (unite(root, a, b)) {
                            ++out.mergedJunctions;
                            merged = true;
                        }
                    }
                    // (an end the two share within a cut's tolerance is a stop line if either says so)
                    if (iv.ua <= one.back().ua + kCutTolerance) one.back().stopLineAtStart = one.back().stopLineAtStart || iv.stopLineAtStart;
                    if (iv.ub > one.back().ub + kCutTolerance) {
                        one.back().ub = iv.ub;
                        one.back().stopLineAtEnd = iv.stopLineAtEnd;
                    } else if (iv.ub >= one.back().ub - kCutTolerance) {
                        one.back().stopLineAtEnd = one.back().stopLineAtEnd || iv.stopLineAtEnd;
                    }
                } else {
                    one.push_back(iv);
                }
            }
            list = one;
        }
        if (!merged) break;
        if (round + 1 == kMergeRounds)
            addProblem(out.problems, "junctions still overlap after %d rounds of merging", kMergeRounds);
    }

    // ---- sections and lane paths -------------------------------------------------------------------------------------
    struct SectionNote { int arrivingAt = -1, leavingFrom = -1; bool outlineEnd = false, outlineStart = false; };
    std::vector<SectionNote> notes;
    std::vector<std::vector<uint32_t>> pathsOfLane(nLanes);
    for (const std::pair<const CorridorKey, Corridor>& kc : corridors) {
        const Corridor& c = kc.second;
        const EdgeSpec& e = edgeAt(c.edge);
        const std::vector<Interval>& ivs = intervals[kc.first];
        std::vector<double> cuts;
        for (const Member& m : c.members) {
            cuts.push_back(m.u0());
            cuts.push_back(m.u1());
        }
        for (const Interval& iv : ivs) {
            cuts.push_back(iv.ua);
            cuts.push_back(iv.ub);
            // where the solid lines before a stop line begin (only where there are lanes side by side to keep apart)
            const double solidFrom = iv.ua - kSolidBeforeStopLine;
            int beside = 0;
            for (const Member& m : c.members) beside += covers(m, solidFrom) ? 1 : 0;
            if (solidFrom > 0 && beside >= 2) cuts.push_back(solidFrom);
        }
        std::sort(cuts.begin(), cuts.end());
        std::vector<double> cut;
        for (double x : cuts)
            if (cut.empty() || x - cut.back() > kCutTolerance) cut.push_back(x);
        for (std::size_t i = 0; i + 1 < cut.size(); ++i) {
            const double c0 = cut[i], c1 = cut[i + 1];
            std::vector<const Member*> here;
            for (const Member& m : c.members)
                if (m.u0() <= c0 + kCutTolerance && m.u1() >= c1 - kCutTolerance) here.push_back(&m);
            if (here.empty()) continue;
            const Interval* inside = nullptr;
            const Interval* arriving = nullptr;
            const Interval* leaving = nullptr;
            bool solid = false;
            for (const Interval& iv : ivs) {
                if (c0 >= iv.ua - kCutTolerance && c1 <= iv.ub + kCutTolerance) inside = &iv;
                if (std::fabs(c1 - iv.ua) <= kCutTolerance) arriving = &iv;
                if (std::fabs(c0 - iv.ub) <= kCutTolerance) leaving = &iv;
                if (c1 <= iv.ua + kCutTolerance && c0 >= iv.ua - kSolidBeforeStopLine - kCutTolerance) solid = true;
            }
            if (inside) {   // the junction's: the connectors' ground
                for (const Member* m : here)
                    out.insideJunctions[static_cast<std::size_t>(m->lane)] += m->tauAtCut(c1) - m->tauAtCut(c0);
                continue;
            }
            // offside to kerb, by where each lane lies across the road halfway along
            const double mid = 0.5 * (c0 + c1);
            const double sigmaMid = c.dir > 0 ? mid : c.length - mid;
            const Vec2 spineAt = pointAt(e.xy, e.s, sigmaMid), spineNormal = perp(tangentAtStation(e.xy, e.s, sigmaMid));
            std::vector<std::pair<double, const Member*>> order;
            for (const Member* m : here) {
                Vec2 p, n;
                memberPoint(*m, mid, p, n);
                order.emplace_back(offsideness(dot(p - spineAt, spineNormal), c.dir), m);
            }
            std::sort(order.begin(), order.end(), [](const std::pair<double, const Member*>& a, const std::pair<double, const Member*>& b) {
                return a.first != b.first ? a.first > b.first : a.second->lane < b.second->lane;
            });
            const SectionId sid(static_cast<uint32_t>(G.sections.size()));
            Section sec;
            sec.roadClass = LaneGraph::nameIndex(G.roadClassNames, e.cls);
            sec.lanes.first = static_cast<uint32_t>(G.sectionLanes.size());
            sec.lines.first = static_cast<uint32_t>(G.laneLines.size());
            Vec2 where;
            for (std::size_t k = 0; k < order.size(); ++k) {
                const Member& m = *order[k].second;
                const Lane& l = laneAt(m.lane);
                const TravelLane& T = travel[static_cast<std::size_t>(m.lane)];
                const double ta = m.tauAtCut(c0), tb = m.tauAtCut(c1);
                const std::vector<Vec2> pts = sliceAtStations(T.xy, T.s, ta, tb, kSliceGap);
                std::vector<double> z;
                for (const Vec2& p : pts) z.push_back(H.deck(m.lane, p));
                if (k == 0) where = pts.front();
                Path p;
                p.kind = PathKind::Lane;
                const NameIndex tag = LaneGraph::nameIndex(G.laneTagNames, l.kind);
                p.laneTags = tag < kMaxTagNames ? TagMask(1) << tag : 0;
                p.vehicleTags = everyone;
                p.curve = G.addCurve(pts, z);
                p.length = G.along[p.curve.end() - 1];
                p.width = l.w;
                p.tightestRadius = tightestRadius(pts);
                p.speedLimit = 0;   // no posted limit: limits come with the control plan
                p.section = sid;
                p.indexInSection = static_cast<uint32_t>(k);
                const uint32_t id = static_cast<uint32_t>(G.paths.size());
                G.paths.push_back(p);
                G.sectionLanes.push_back(PathId(id));
                PathSource src;
                src.lane = m.lane;
                src.from = ta;
                src.to = tb;
                out.sources.push_back(src);
                pathsOfLane[static_cast<std::size_t>(m.lane)].push_back(id);
            }
            // the lines between neighbours: solid on the approach to a stop line, unless one of the two lanes ends or
            // begins here (a lane that ends must be left, one that begins entered); otherwise crossable both ways
            for (std::size_t k = 0; k + 1 < order.size(); ++k) {
                bool endsOrBegins = false;
                for (const Member* m : {order[k].second, order[k + 1].second}) {
                    const bool ends = std::fabs(m->u1() - c1) <= kCutTolerance && !arriving;
                    const bool begins = std::fabs(m->u0() - c0) <= kCutTolerance && !leaving;
                    endsOrBegins = endsOrBegins || ends || begins;
                }
                LaneLine line;
                line.crossFromOffside = line.crossFromKerbside = !solid || endsOrBegins;
                G.laneLines.push_back(line);
            }
            sec.lanes.count = static_cast<uint32_t>(G.sectionLanes.size()) - sec.lanes.first;
            sec.lines.count = static_cast<uint32_t>(G.laneLines.size()) - sec.lines.first;
            G.sections.push_back(sec);
            SectionNote note;
            if (arriving) {
                note.arrivingAt = arriving->junction;
                note.outlineEnd = !arriving->stopLineAtStart;
            }
            if (leaving) {
                note.leavingFrom = leaving->junction;
                note.outlineStart = !leaving->stopLineAtEnd;
            }
            notes.push_back(note);
            SectionSource ss;
            ss.edge = c.edge;
            ss.dir = c.dir;
            ss.from = c0;
            ss.to = c1;
            out.sectionSources.push_back(ss);
            if (c1 - c0 < kShortestSection)
                addProblem(out.problems, "section %u on road %s (%s) at (%.1f, %.1f) is %.2f m long", sid.index, e.id.c_str(),
                           way(c.dir), where.x, where.y, c1 - c0);
        }
    }

    // ---- what follows each lane --------------------------------------------------------------------------------------
    std::vector<std::vector<PathId>> next(G.paths.size());
    std::vector<char> arrivingPath(G.paths.size(), 0);
    for (uint32_t s = 0; s < G.sections.size(); ++s)
        if (notes[s].arrivingAt >= 0)
            for (uint32_t k = G.sections[s].lanes.first; k < G.sections[s].lanes.end(); ++k) arrivingPath[G.sectionLanes[k].index] = 1;
    // the same builder lane carried on into the next section
    for (std::size_t li = 0; li < nLanes; ++li) {
        std::vector<uint32_t>& ps = pathsOfLane[li];
        std::sort(ps.begin(), ps.end(), [&](uint32_t a, uint32_t b) { return out.sources[a].from < out.sources[b].from; });
        for (std::size_t k = 0; k + 1 < ps.size(); ++k)
            if (std::fabs(out.sources[ps[k]].to - out.sources[ps[k + 1]].from) <= kCutTolerance) next[ps[k]].push_back(PathId(ps[k + 1]));
    }
    // a lane leading into the next road's lane where the two roads meet end to end
    std::vector<uint32_t> starters;
    for (uint32_t p = 0; p < G.paths.size(); ++p)
        if (out.sources[p].from <= kCutTolerance) starters.push_back(p);
    const double joinCos = std::cos(kJoinDegrees / kDegreesPerRadian), nearCos = std::cos(kNearMissDegrees / kDegreesPerRadian);
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        const PathSource& src = out.sources[p];
        if (!next[p].empty() || arrivingPath[p]) continue;
        if (src.to < travel[static_cast<std::size_t>(src.lane)].s.back() - kCutTolerance) continue;
        const Vec2 end = G.pointAt(PathId(p), G.paths[p].length), dirEnd = G.directionAt(PathId(p), G.paths[p].length);
        for (uint32_t q : starters) {
            if (out.sources[q].lane == src.lane) continue;
            const Vec2 start = G.pointAt(PathId(q), 0), dirStart = G.directionAt(PathId(q), 0);
            const double gap = distance(end, start), cosine = dot(dirEnd, dirStart);
            if (gap <= kJoinDistance && cosine >= joinCos) {
                next[p].push_back(PathId(q));
                ++out.roadJoins;
            } else if (gap <= kNearMiss && cosine >= nearCos) {
                addProblem(out.problems, "lane %u ends at (%.2f, %.2f), %.3f m and %.1f degrees from where lane %u begins: "
                                         "not joined", p, end.x, end.y, gap,
                           std::acos(std::min(1.0, cosine)) * kDegreesPerRadian, q);
            }
        }
    }
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        std::sort(next[p].begin(), next[p].end());
        G.paths[p].next.first = static_cast<uint32_t>(G.nextPaths.size());
        G.nextPaths.insert(G.nextPaths.end(), next[p].begin(), next[p].end());
        G.paths[p].next.count = static_cast<uint32_t>(next[p].size());
    }

    // ---- the junction records ----------------------------------------------------------------------------------------
    std::vector<std::vector<PathId>> arrivingAt(boxesOfJunction.size()), leavingFrom(boxesOfJunction.size());
    for (uint32_t s = 0; s < G.sections.size(); ++s) {
        const Section& sec = G.sections[s];
        for (uint32_t k = sec.lanes.first; k < sec.lanes.end(); ++k) {
            if (notes[s].arrivingAt >= 0) {
                arrivingAt[static_cast<std::size_t>(notes[s].arrivingAt)].push_back(G.sectionLanes[k]);
                if (notes[s].outlineEnd) ++out.endsAtOutline;
            }
            if (notes[s].leavingFrom >= 0) {
                leavingFrom[static_cast<std::size_t>(notes[s].leavingFrom)].push_back(G.sectionLanes[k]);
                if (notes[s].outlineStart) ++out.endsAtOutline;
            }
        }
    }
    for (std::size_t j = 0; j < boxesOfJunction.size(); ++j) {
        Junction jn;
        jn.control = 0;
        jn.connectors = Range{static_cast<uint32_t>(G.junctionConnectors.size()), 0};
        jn.crossings = Range{static_cast<uint32_t>(G.crossings.size()), 0};
        jn.crosswalks = Range{static_cast<uint32_t>(G.crosswalks.size()), 0};
        jn.box = G.addPlanPoints(outline[j]);
        std::sort(arrivingAt[j].begin(), arrivingAt[j].end());
        std::sort(leavingFrom[j].begin(), leavingFrom[j].end());
        jn.arriving.first = static_cast<uint32_t>(G.junctionLanes.size());
        G.junctionLanes.insert(G.junctionLanes.end(), arrivingAt[j].begin(), arrivingAt[j].end());
        jn.arriving.count = static_cast<uint32_t>(arrivingAt[j].size());
        jn.leaving.first = static_cast<uint32_t>(G.junctionLanes.size());
        G.junctionLanes.insert(G.junctionLanes.end(), leavingFrom[j].begin(), leavingFrom[j].end());
        jn.leaving.count = static_cast<uint32_t>(leavingFrom[j].size());
        G.junctions.push_back(jn);
        const Vec2 at = outline[j].empty() ? Vec2() : centroid(outline[j]);
        std::string roads;
        int lastEdge = -1;   // (a junction's boxes come street by street)
        for (std::size_t b : boxesOfJunction[j]) {
            if (boxes[b].edge != lastEdge) roads += (roads.empty() ? "" : ", ") + edgeAt(boxes[b].edge).id;
            lastEdge = boxes[b].edge;
        }
        if (outline[j].size() < 3)
            addProblem(out.problems, "junction %zu at (%.1f, %.1f) (roads %s): its outline has %zu corners", j, at.x, at.y,
                       roads.c_str(), outline[j].size());
        // an outline that crosses itself: two of its streets' stop lines overlap, the streets meeting at a sharp angle
        // (their lanes cross outside the paint's box)
        const std::size_t n = outline[j].size();
        const bool tangled = crossesItself(outline[j]);
        if (tangled)
            addProblem(out.problems, "junction %zu at (%.1f, %.1f) (roads %s): its outline crosses itself (two streets meet at "
                                     "a sharp angle and their stop lines overlap)", j, at.x, at.y, roads.c_str());
        // a lane path lying inside the outline, at the junction's level (more than kJunctionEndReach in from it)
        std::vector<double> levels;
        for (const PathId& p : arrivingAt[j]) levels.push_back(G.heightAt(p, G.paths[p.index].length));
        for (const PathId& p : leavingFrom[j]) levels.push_back(G.heightAt(p, 0));
        if (tangled || n < 3 || levels.empty()) continue;
        for (uint32_t p = 0; p < G.paths.size(); ++p)
            for (uint32_t k = G.paths[p].curve.first; k < G.paths[p].curve.end(); ++k) {
                const Vec2 q = G.points[k];
                if (q.x < outlineBounds[j].x0 || q.x > outlineBounds[j].x1 || q.y < outlineBounds[j].y0 || q.y > outlineBounds[j].y1) continue;
                if (!pointInPolygon(outline[j], q) || distToPolyline(q, outline[j], true) <= kJunctionEndReach) continue;
                double nearest = 1e300;
                for (double z : levels) nearest = std::min(nearest, std::fabs(G.heights[k] - z));
                if (nearest > kJunctionLevelReach) continue;
                addProblem(out.problems, "junction %zu at (%.1f, %.1f) (roads %s): lane %u lies inside its outline at (%.1f, %.1f)",
                           j, at.x, at.y, roads.c_str(), p, q.x, q.y);
                break;
            }
        if (arrivingAt[j].size() < 2 || leavingFrom[j].size() < 2)
            addProblem(out.problems, "junction %zu at (%.1f, %.1f) (roads %s): %zu lanes arriving and %zu leaving", j, at.x,
                       at.y, roads.c_str(), arrivingAt[j].size(), leavingFrom[j].size());
    }

    // ---- two roads' lanes crossing at one level outside every junction: a meeting the paint does not see ------------
    {
        std::vector<Bounds> pathBounds(G.paths.size());
        std::vector<std::vector<Vec2>> pathPoints(G.paths.size());
        for (uint32_t p = 0; p < G.paths.size(); ++p) {
            const Range c = G.paths[p].curve;
            pathPoints[p].assign(G.points.begin() + c.first, G.points.begin() + c.end());
            for (const Vec2& q : pathPoints[p]) pathBounds[p].add(q);
        }
        auto roadOf = [&](uint32_t p) { return laneAt(out.sources[p].lane).parent; };
        std::map<std::pair<int, int>, std::pair<uint32_t, Vec2>> meetings;   // (road, road) -> how many, the first place
        for (uint32_t a = 0; a < G.paths.size(); ++a)
            for (uint32_t b = a + 1; b < G.paths.size(); ++b) {
                if (roadOf(a) == roadOf(b) || !pathBounds[a].overlaps(pathBounds[b])) continue;
                for (const Vec2& x : crossings(pathPoints[a], pathPoints[b])) {
                    bool nearEnd = false;   // lanes born out of or dying into another touch it at their ends
                    for (uint32_t p : {a, b})
                        nearEnd = nearEnd || distance(x, pathPoints[p].front()) < kNearMiss || distance(x, pathPoints[p].back()) < kNearMiss;
                    if (nearEnd) continue;
                    const double za = H.deck(out.sources[a].lane, x), zb = H.deck(out.sources[b].lane, x);
                    if (std::fabs(za - zb) > g.rules.sameLevelDz) continue;
                    const std::pair<int, int> key(std::min(roadOf(a), roadOf(b)), std::max(roadOf(a), roadOf(b)));
                    std::pair<uint32_t, Vec2>& m = meetings[key];
                    if (m.first++ == 0) m.second = x;
                }
            }
        for (const std::pair<const std::pair<int, int>, std::pair<uint32_t, Vec2>>& m : meetings) {
            ++out.crossingsOutside;
            addProblem(out.problems, "roads %s and %s: their lanes cross %u times at one level outside every junction, the "
                                     "first at (%.1f, %.1f)", edgeAt(m.first.first).id.c_str(), edgeAt(m.first.second).id.c_str(),
                       m.second.first, m.second.second.x, m.second.second.y);
        }
    }

    for (const std::string& p : validate(G)) out.problems.push_back("lane graph: " + p);
    out.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
