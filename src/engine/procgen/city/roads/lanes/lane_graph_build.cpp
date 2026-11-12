#include "engine/procgen/city/roads/lanes/lane_graph_build.h"

#include "engine/procgen/city/polygon.h"                     // convexHull, pointInPolygon
#include "engine/procgen/city/roads/lanes/deck_mesh.h"       // the junction boxes the paint finds
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"   // the paths through junctions
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
// How many points either side of a fold are searched for where the lane crosses itself.
constexpr std::size_t kFoldSearch = 16;
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

// CUTS THE FOLD OUT OF A LANE that runs backwards for a stretch (on the inside of a corner of its road sharper than its
// offset from the road's spine, the builder's offset points loop back on themselves): `forward[k]` is the way its road
// runs at point k. Where the lane before a backward run crosses the lane after it, the lane runs to that crossing and
// on from it; where it does not, every point that would take it backwards is dropped. `u` (each point's distance
// along its road) is kept in step. Returns whether anything was cut.
bool cutFolds(std::vector<Vec2>& xy, std::vector<double>& u, const std::vector<Vec2>& forward) {
    bool cut = false;
    std::vector<Vec2> fw = forward;
    for (std::size_t k = 1; k < xy.size(); ++k) {
        if (dot(xy[k] - xy[k - 1], fw[k]) >= 0) continue;
        // the backward run: segments kb .. ke - 1 (segment k joins point k - 1 to point k)
        const std::size_t kb = k;
        std::size_t ke = k;
        while (ke < xy.size() && dot(xy[ke] - xy[ke - 1], fw[ke]) < 0) ++ke;
        bool joined = false;
        for (std::size_t j = ke; j < xy.size() && j <= ke + kFoldSearch && !joined; ++j)
            for (std::size_t i = kb - 1; i >= 1 && i + kFoldSearch >= kb && !joined; --i) {
                Vec2 at;
                if (!segmentsCross(xy[i - 1], xy[i], xy[j - 1], xy[j], at)) continue;
                const double len = distance(xy[i - 1], xy[i]);
                const double f = len > 0 ? distance(xy[i - 1], at) / len : 0;
                const double uAt = u[i - 1] + f * (u[i] - u[i - 1]);
                // points 0 .. i - 1, the crossing, then j onwards
                xy.erase(xy.begin() + static_cast<std::ptrdiff_t>(i), xy.begin() + static_cast<std::ptrdiff_t>(j));
                u.erase(u.begin() + static_cast<std::ptrdiff_t>(i), u.begin() + static_cast<std::ptrdiff_t>(j));
                fw.erase(fw.begin() + static_cast<std::ptrdiff_t>(i), fw.begin() + static_cast<std::ptrdiff_t>(j));
                xy.insert(xy.begin() + static_cast<std::ptrdiff_t>(i), at);
                u.insert(u.begin() + static_cast<std::ptrdiff_t>(i), uAt);
                fw.insert(fw.begin() + static_cast<std::ptrdiff_t>(i), fw[i - 1]);
                joined = true;
            }
        if (!joined) {   // no loop to cut: drop the points that run back, up to one ahead of the last kept again
            std::size_t j = kb;
            while (j < xy.size() && dot(xy[j] - xy[kb - 1], fw[j]) <= 0) ++j;
            j = std::min(j, xy.size() - 1);   // (never the lane's own end)
            if (j == kb) {   // nothing to drop without losing the end: leave it
                k = ke - 1;
                continue;
            }
            xy.erase(xy.begin() + static_cast<std::ptrdiff_t>(kb), xy.begin() + static_cast<std::ptrdiff_t>(j));
            u.erase(u.begin() + static_cast<std::ptrdiff_t>(kb), u.begin() + static_cast<std::ptrdiff_t>(j));
            fw.erase(fw.begin() + static_cast<std::ptrdiff_t>(kb), fw.begin() + static_cast<std::ptrdiff_t>(j));
        }
        cut = true;
        k = kb - 1;   // look again from just before the fold
    }
    return cut;
}

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
// `solidBefore`: the lines between lanes are solid on the approach (not at a carry-on or a node, where nothing stops).
struct Interval {
    double ua = 0, ub = 0;
    int junction = -1;
    bool stopLineAtStart = true, stopLineAtEnd = true;
    bool solidBefore = true;
};

// One of the paint's junction boxes, on its street; or a road's end given up to a carry-on junction.
struct Box {
    int edge = -1;
    JunctionSpan span;
    double sa = 0, sb = 0;   // the stretch of the street's spine it takes, stop line to stop line (clamped to the road)
    int carryOn = -1;        // a carry-on's or a node's (which one): its end of the road, as far back as the connectors need
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
    for (const VehicleSize& v : opts.vehicles) G.vehicleTagNames.push_back(v.tag);
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
    out.laneLengths.assign(nLanes, 0.0);
    std::map<int, std::vector<Vec2>> spineTangents;   // per road, as the builder framed it
    for (std::size_t li = 0; li < nLanes; ++li) {
        if (!kept[li]) continue;
        const Lane& l = L.lanes[li];
        const EdgeSpec& e = edgeAt(l.parent);
        TravelLane& T = travel[li];
        T.xy = l.xy;
        if (l.dir < 0) std::reverse(T.xy.begin(), T.xy.end());
        T.s = stations(T.xy);
        out.laneLengths[li] = T.s.back();
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
        std::vector<Vec2> forward;   // the way its road runs at each point
        for (std::size_t k = 0; k < n; ++k) {
            const std::size_t i = static_cast<std::size_t>(first) + (l.dir > 0 ? k : n - 1 - k);
            T.u.push_back(l.dir > 0 ? e.s[i] : e.s.back() - e.s[i]);
            forward.push_back(tangent[i] * static_cast<double>(l.dir));
            // a lane on the inside of a sharp corner of its road runs backwards for a stretch there
            if (k > 0 && dot(T.xy[k] - T.xy[k - 1], forward[k]) < 0) {
                back += distance(T.xy[k], T.xy[k - 1]);
                if (backAt.x == 0 && backAt.y == 0) backAt = T.xy[k];
            }
        }
        if (back > kCutTolerance) {
            // tidied: the fold cut out of the lane graph's copy, so distance along it always moves forward
            cutFolds(T.xy, T.u, forward);
            T.s = stations(T.xy);
            out.laneLengths[li] = T.s.back();
            ++out.tidiedLanes;
            addProblem(out.problems, "lane %s runs backwards for %.2f m near (%.1f, %.1f): it lies on the inside of a corner of "
                                     "road %s sharper than its offset from the road's spine; tidied (the fold cut out, %.2f m "
                                     "shorter)", l.id.c_str(), back, backAt.x, backAt.y, e.id.c_str(), l.s.back() - T.s.back());
        }
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
    // The way a member runs at corridor distance u.
    auto memberDir = [&](const Member& m, double u) {
        const TravelLane& T = travel[static_cast<std::size_t>(m.lane)];
        return tangentAtStation(T.xy, T.s, m.tauAtCut(u));
    };
    // A corridor's lanes across it at u, offside to kerb (by where they lie across the road there): their builder lanes,
    // points and directions.
    struct Across { std::vector<const Member*> members; std::vector<Vec2> at, dir; };
    auto across = [&](const Corridor& c, double u) {
        const EdgeSpec& e = edgeAt(c.edge);
        const double sigma = c.dir > 0 ? u : c.length - u;
        const Vec2 spineAt = pointAt(e.xy, e.s, sigma), spineNormal = perp(tangentAtStation(e.xy, e.s, sigma));
        std::vector<std::pair<double, const Member*>> order;
        for (const Member& m : c.members) {
            if (!covers(m, u)) continue;
            Vec2 p, n;
            memberPoint(m, u, p, n);
            order.emplace_back(offsideness(dot(p - spineAt, spineNormal), c.dir), &m);
        }
        std::sort(order.begin(), order.end(), [](const std::pair<double, const Member*>& a, const std::pair<double, const Member*>& b) {
            return a.first != b.first ? a.first > b.first : a.second->lane < b.second->lane;
        });
        Across a;
        for (const std::pair<double, const Member*>& o : order) {
            Vec2 p, n;
            memberPoint(*o.second, u, p, n);
            a.members.push_back(o.second);
            a.at.push_back(p);
            a.dir.push_back(memberDir(*o.second, u));
        }
        return a;
    };

    // ---- junctions: the paint's boxes, one junction per crossing ----------------------------------------------------
    const std::chrono::steady_clock::time_point tBoxes = std::chrono::steady_clock::now();
    const LaneLookup look(r.pavement);
    const std::map<int, std::vector<JunctionSpan>> spans = junctionSpans(r, look, kJunctionParallelForGraph);
    out.boxMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tBoxes).count();
    std::vector<Box> boxes;
    for (const std::pair<const int, std::vector<JunctionSpan>>& kv : spans) {
        const EdgeSpec& e = edgeAt(kv.first);
        if (g.cls(e).track || g.cls(e).footway) continue;
        for (const JunctionSpan& sp : kv.second) {
            Box b;
            b.edge = kv.first;
            b.span = sp;
            b.sa = std::max(0.0, sp.s0 - kStopLineBack);
            b.sb = std::min(e.s.back(), sp.s1 + kStopLineBack);
            boxes.push_back(b);
        }
    }

    // ---- carry-ons and nodes: road ends meeting outside every box, their lanes just missing ---------------------------
    std::vector<int> roadEndsOfNode;   // per carry-on or node (Box::carryOn): how many road ends meet there
    {
        // whether a box of the road's own reaches spine station s
        auto boxedAt = [&](int edge, double s) {
            const std::map<int, std::vector<JunctionSpan>>::const_iterator it = spans.find(edge);
            if (it == spans.end()) return false;
            for (const JunctionSpan& sp : it->second)
                if (s >= sp.s0 - kStopLineBack - kCutTolerance && s <= sp.s1 + kStopLineBack + kCutTolerance) return true;
            return false;
        };
        struct LaneEnd { CorridorKey corridor; int lane; Vec2 at, dir; };
        std::vector<LaneEnd> laneEnds, laneStarts;
        for (const std::pair<const CorridorKey, Corridor>& kc : corridors) {
            const Corridor& c = kc.second;
            const double sEnd = c.dir > 0 ? c.length : 0.0, sStart = c.dir > 0 ? 0.0 : c.length;
            for (const Member& m : c.members) {
                const TravelLane& T = travel[static_cast<std::size_t>(m.lane)];
                if (std::fabs(m.u1() - c.length) <= kCutTolerance && m.to >= T.s.back() - kCutTolerance && !boxedAt(c.edge, sEnd))
                    laneEnds.push_back(LaneEnd{kc.first, m.lane, T.xy.back(), tangentAtStation(T.xy, T.s, T.s.back())});
                if (m.u0() <= kCutTolerance && m.from <= kCutTolerance && !boxedAt(c.edge, sStart))
                    laneStarts.push_back(LaneEnd{kc.first, m.lane, T.xy.front(), tangentAtStation(T.xy, T.s, 0)});
            }
        }
        // which corridors' lanes start near which corridors' lane ends, and whether any of them miss
        const double joinCos = std::cos(kJoinDegrees / kDegreesPerRadian), nearCos = std::cos(kNearMissDegrees / kDegreesPerRadian);
        // (end's corridor, start's corridor) -> whether any lane misses, and the builder lanes that meet (end, start)
        struct Met { bool miss = false; std::vector<std::pair<int, int>> lanes; };
        std::map<std::pair<CorridorKey, CorridorKey>, Met> met;
        for (const LaneEnd& a : laneEnds)
            for (const LaneEnd& b : laneStarts) {
                const double gap = distance(a.at, b.at), cosine = dot(a.dir, b.dir);
                if (a.lane == b.lane || gap > kNearMiss || cosine < nearCos) continue;
                Met& m = met[std::make_pair(a.corridor, b.corridor)];
                m.miss = m.miss || gap > kJoinDistance || cosine < joinCos;
                m.lanes.emplace_back(a.lane, b.lane);
            }
        // which end of its road a corridor's end (or start) is: 0 the spine's start, 1 its end
        using RoadEnd = std::pair<int, int>;
        auto roadEnd = [](const CorridorKey& k, bool corridorEnd) { return RoadEnd(k.first, corridorEnd == (k.second == 0) ? 1 : 0); };
        // the road ends that meet, grouped: two meet where a lane of one ends near where a lane of the other begins. A
        // group where any lane just misses becomes a junction of its own: a carry-on where it is two road ends, a node
        // where it is more (roads meeting end to end with no box)
        std::vector<RoadEnd> ends;
        for (const std::pair<const std::pair<CorridorKey, CorridorKey>, Met>& kv : met) {
            ends.push_back(roadEnd(kv.first.first, true));
            ends.push_back(roadEnd(kv.first.second, false));
        }
        std::sort(ends.begin(), ends.end());
        ends.erase(std::unique(ends.begin(), ends.end()), ends.end());
        auto endIndex = [&](const RoadEnd& e) { return static_cast<int>(std::lower_bound(ends.begin(), ends.end(), e) - ends.begin()); };
        std::vector<int> endRoot(ends.size());
        std::iota(endRoot.begin(), endRoot.end(), 0);
        for (const std::pair<const std::pair<CorridorKey, CorridorKey>, Met>& kv : met)
            unite(endRoot, endIndex(roadEnd(kv.first.first, true)), endIndex(roadEnd(kv.first.second, false)));
        std::map<int, std::vector<const std::pair<const std::pair<CorridorKey, CorridorKey>, Met>*>> meetings;   // group (its root) -> its meetings
        std::map<int, bool> anyMiss;
        for (const std::pair<const std::pair<CorridorKey, CorridorKey>, Met>& kv : met) {
            const int root = findRoot(endRoot, endIndex(roadEnd(kv.first.first, true)));
            meetings[root].push_back(&kv);
            anyMiss[root] = anyMiss[root] || kv.second.miss;
        }
        double widestTurn = 0;   // the largest turning radius of the vehicles: what a carry-on's connectors should allow
        for (const VehicleSize& v : opts.vehicles) widestTurn = std::max(widestTurn, v.turningRadius);
        for (const std::pair<const int, std::vector<const std::pair<const std::pair<CorridorKey, CorridorKey>, Met>*>>& node : meetings) {
            if (!anyMiss[node.first]) continue;
            std::vector<RoadEnd> members;
            for (std::size_t i = 0; i < ends.size(); ++i)
                if (findRoot(endRoot, static_cast<int>(i)) == node.first) members.push_back(ends[i]);
            // how far back each road gives up: the first trim whose straight-on connectors (each lane end into the start
            // it met) suit the widest-turning vehicle
            double shortest = kNoBend;
            for (const RoadEnd& end : members) shortest = std::min(shortest, edgeAt(end.first).s.back());
            double trim = 0;
            for (double d : kCarryOnTrims) {
                if (d > kCarryOnTrimShare * shortest) break;
                trim = d;
                double tightest = kNoBend;
                for (const std::pair<const std::pair<CorridorKey, CorridorKey>, Met>* m : node.second) {
                    const Corridor& ca = corridors.at(m->first.first);
                    const Corridor& cb = corridors.at(m->first.second);
                    const Across xa = across(ca, ca.length - d), xb = across(cb, d);
                    for (const std::pair<int, int>& lanes : m->second.lanes) {
                        std::size_t ia = xa.members.size(), ib = xb.members.size();
                        for (std::size_t i = 0; i < xa.members.size(); ++i)
                            if (xa.members[i]->lane == lanes.first) ia = i;
                        for (std::size_t i = 0; i < xb.members.size(); ++i)
                            if (xb.members[i]->lane == lanes.second) ib = i;
                        if (ia == xa.members.size() || ib == xb.members.size()) continue;
                        const ConnectorShape s = connectorShape(xa.at[ia], xa.dir[ia], xb.at[ib], xb.dir[ib]);
                        tightest = std::min(tightest, s.ok ? s.tightestRadius : 0.0);
                    }
                }
                if (tightest >= widestTurn) break;
            }
            if (trim == 0) trim = kCarryOnTrimShare * shortest;
            for (const RoadEnd& end : members) {
                const EdgeSpec& e = edgeAt(end.first);
                Box b;
                b.edge = end.first;
                b.sa = end.second == 1 ? e.s.back() - trim : 0.0;
                b.sb = end.second == 1 ? e.s.back() : trim;
                b.span.s0 = b.sa;
                b.span.s1 = b.sb;
                b.carryOn = static_cast<int>(roadEndsOfNode.size());
                boxes.push_back(b);
            }
            roadEndsOfNode.push_back(static_cast<int>(members.size()));
        }
    }
    // the boxes by road, then along it (the paint's come so already; the carry-ons' join them in place)
    std::stable_sort(boxes.begin(), boxes.end(), [](const Box& a, const Box& b) { return a.edge != b.edge ? a.edge < b.edge : a.sa < b.sa; });
    std::map<int, std::pair<std::size_t, std::size_t>> boxesOfEdge;   // edge -> (first box, count)
    for (std::size_t b = 0; b < boxes.size(); ++b) {
        std::pair<std::size_t, std::size_t>& range = boxesOfEdge.emplace(boxes[b].edge, std::make_pair(b, std::size_t(0))).first->second;
        ++range.second;
    }
    std::vector<int> root(boxes.size());
    std::iota(root.begin(), root.end(), 0);
    // a carry-on's two boxes are one junction
    for (std::size_t a = 0; a < boxes.size(); ++a)
        for (std::size_t b = a + 1; b < boxes.size(); ++b)
            if (boxes[a].carryOn >= 0 && boxes[a].carryOn == boxes[b].carryOn) unite(root, static_cast<int>(a), static_cast<int>(b));
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

    // AN OUTLINE from the lines across a junction's roads where its lanes stop (`legs`, each an edge of it), joined
    // corner to corner in order round the junction (a convex hull would leave a skewed street's stop line inside it);
    // with fewer than two, the convex hull of `all` (the lane ends there).
    struct Leg { Vec2 a, b, mid; };
    auto ringOf = [](std::vector<Leg> legs, const std::vector<Vec2>& all) -> std::vector<Vec2> {
        std::vector<Vec2> ring;
        if (legs.size() < 2) return convexHull(all);
        Vec2 centre;
        for (const Leg& leg : legs) centre += leg.mid;
        centre = centre / static_cast<double>(legs.size());
        for (Leg& leg : legs)
            if (cross(leg.a - centre, leg.b - centre) < 0) std::swap(leg.a, leg.b);   // a to b runs anticlockwise
        std::sort(legs.begin(), legs.end(), [&](const Leg& x, const Leg& y) {
            return std::atan2(x.mid.y - centre.y, x.mid.x - centre.x) < std::atan2(y.mid.y - centre.y, y.mid.x - centre.x);
        });
        for (const Leg& leg : legs) {
            ring.push_back(leg.a);
            ring.push_back(leg.b);
        }
        return ring;
    };
    std::vector<std::vector<Leg>> stopLegs;     // per junction: its roads' stop lines
    std::vector<std::vector<Vec2>> stopEnds;    // per junction: the lane ends at them (and at its roads' ends inside it)
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
        stopLegs.assign(boxesOfJunction.size(), {});
        stopEnds.assign(boxesOfJunction.size(), {});
        for (std::size_t j = 0; j < boxesOfJunction.size(); ++j) {
            std::vector<Leg>& legs = stopLegs[j];
            std::vector<Vec2>& all = stopEnds[j];
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
            outline[j] = ringOf(legs, all);
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
                iv.solidBefore = boxes[b].carryOn < 0;
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
                    if (iv.ua <= one.back().ua + kCutTolerance) {
                        one.back().stopLineAtStart = one.back().stopLineAtStart || iv.stopLineAtStart;
                        one.back().solidBefore = one.back().solidBefore || iv.solidBefore;
                    }
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
        // a ramp that ends on a street (no gore on a host lane there) runs inside the street from the junction it runs
        // into to its own end: that stretch is the junction's
        out.rampEnds = 0;
        for (std::pair<const CorridorKey, std::vector<Interval>>& ki : intervals) {
            const Corridor& c = corridors.at(ki.first);
            const EdgeSpec& e = edgeAt(c.edge);
            std::vector<Interval>& list = ki.second;
            if (!e.isRamp() || list.empty()) continue;
            for (int end = 0; end < 2; ++end) {   // the ramp's spine start, its end
                const RampAnchor& anchor = end == 0 ? e.from : e.to;
                if (anchor.edge.empty() || e.anchorIdx[end] >= 0 || !g.index.count(anchor.edge)) continue;
                if ((end == 0) == (c.dir > 0)) {   // that end is where the corridor starts
                    if (list.front().ua > kCutTolerance && list.front().ua <= kRampEndReach) {
                        list.front().ua = 0;
                        list.front().stopLineAtStart = false;
                        ++out.rampEnds;
                    }
                } else if (list.back().ub < c.length - kCutTolerance && list.back().ub >= c.length - kRampEndReach) {
                    list.back().ub = c.length;
                    list.back().stopLineAtEnd = false;
                    ++out.rampEnds;
                }
            }
        }
        if (!merged) break;
        if (round + 1 == kMergeRounds)
            addProblem(out.problems, "junctions still overlap after %d rounds of merging", kMergeRounds);
    }
    // An outline takes in the lanes cut where they cross it (a road with no box of its own there). A road meeting the
    // outline at a slant has its lanes cut together where the last of them leaves it, the first ones some way out; where
    // one stands farther out than kJunctionEndReach, the line across them there becomes an edge of the outline too.
    {
        std::vector<std::vector<Leg>> extra(boxesOfJunction.size());
        for (const std::pair<const CorridorKey, std::vector<Interval>>& ki : intervals) {
            const Corridor& c = corridors.at(ki.first);
            for (const Interval& iv : ki.second)
                for (int side = 0; side < 2; ++side) {
                    const double u = side == 0 ? iv.ua : iv.ub;
                    if ((side == 0 ? iv.stopLineAtStart : iv.stopLineAtEnd) || u <= kCutTolerance || u >= c.length - kCutTolerance) continue;
                    const std::size_t j = static_cast<std::size_t>(iv.junction);
                    const Across x = across(c, u);
                    bool far = false;
                    for (const Vec2& p : x.at) far = far || distToPolyline(p, outline[j], true) > kJunctionEndReach;
                    if (!far) continue;
                    const double halfFront = laneAt(x.members.front()->lane).w / 2, halfBack = laneAt(x.members.back()->lane).w / 2;
                    const Vec2 side0 = distance(x.at.back(), x.at.front()) > kCutTolerance ? normalize(x.at.back() - x.at.front())
                                                                                         : perp(x.dir.front());
                    Leg leg;
                    leg.a = x.at.front() - side0 * halfFront;
                    leg.b = x.at.back() + side0 * halfBack;
                    leg.mid = (leg.a + leg.b) * 0.5;
                    extra[j].push_back(leg);
                }
        }
        for (std::size_t j = 0; j < extra.size(); ++j) {
            if (extra[j].empty()) continue;
            std::vector<Leg> legs = stopLegs[j];
            std::vector<Vec2> all = stopEnds[j];
            for (const Leg& leg : extra[j]) {
                legs.push_back(leg);
                all.push_back(leg.a);
                all.push_back(leg.b);
            }
            outline[j] = ringOf(legs, all);
            outlineBounds[j] = Bounds();
            for (const Vec2& p : outline[j]) outlineBounds[j].add(p);
        }
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
            if (iv.solidBefore && solidFrom > 0 && beside >= 2) cuts.push_back(solidFrom);
        }
        std::sort(cuts.begin(), cuts.end());
        std::vector<double> cut;
        for (double x : cuts)
            if (cut.empty() || x - cut.back() > kCutTolerance) cut.push_back(x);
        // a section shorter than kShortestSection outside a junction is merged into its neighbour where one of its cuts
        // is only a lane beginning or ending (not a junction's, not the road's end): that cut goes, and the lanes that
        // began or ended there are carried straight on to the neighbour's cut
        auto fixedCut = [&](double x) {
            if (x <= kCutTolerance || x >= c.length - kCutTolerance) return true;
            for (const Interval& iv : ivs)
                if (std::fabs(x - iv.ua) <= kCutTolerance || std::fabs(x - iv.ub) <= kCutTolerance) return true;
            return false;
        };
        std::vector<double> removed;
        for (bool again = true; again;) {
            again = false;
            for (std::size_t i = 0; i + 1 < cut.size() && !again; ++i) {
                if (cut[i + 1] - cut[i] >= kShortestSection) continue;
                bool inJunction = false;
                for (const Interval& iv : ivs)
                    inJunction = inJunction || (cut[i] >= iv.ua - kCutTolerance && cut[i + 1] <= iv.ub + kCutTolerance);
                if (inJunction) continue;
                std::size_t drop = cut.size();
                if (i + 2 < cut.size() && !fixedCut(cut[i + 1])) drop = i + 1;
                else if (i > 0 && !fixedCut(cut[i])) drop = i;
                if (drop == cut.size()) continue;
                removed.push_back(cut[drop]);
                cut.erase(cut.begin() + static_cast<std::ptrdiff_t>(drop));
                ++out.mergedSections;
                again = true;
            }
        }
        // whether a member's end x is a cut merged away inside (c0, c1)
        auto mergedAway = [&](double x, double c0, double c1) {
            if (x <= c0 + kCutTolerance || x >= c1 - kCutTolerance) return false;
            for (double y : removed)
                if (std::fabs(x - y) <= kCutTolerance) return true;
            return false;
        };
        for (std::size_t i = 0; i + 1 < cut.size(); ++i) {
            const double c0 = cut[i], c1 = cut[i + 1];
            std::vector<const Member*> here;
            for (const Member& m : c.members)
                if ((m.u0() <= c0 + kCutTolerance || mergedAway(m.u0(), c0, c1)) &&
                    (m.u1() >= c1 - kCutTolerance || mergedAway(m.u1(), c0, c1)))
                    here.push_back(&m);
            if (here.empty()) continue;
            const Interval* inside = nullptr;
            const Interval* arriving = nullptr;
            const Interval* leaving = nullptr;
            bool solid = false;
            for (const Interval& iv : ivs) {
                if (c0 >= iv.ua - kCutTolerance && c1 <= iv.ub + kCutTolerance) inside = &iv;
                if (std::fabs(c1 - iv.ua) <= kCutTolerance) arriving = &iv;
                if (std::fabs(c0 - iv.ub) <= kCutTolerance) leaving = &iv;
                if (iv.solidBefore && c1 <= iv.ua + kCutTolerance && c0 >= iv.ua - kSolidBeforeStopLine - kCutTolerance) solid = true;
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
                const double ta = m.tauAtCut(std::max(c0, m.u0())), tb = m.tauAtCut(std::min(c1, m.u1()));
                std::vector<Vec2> pts = sliceAtStations(T.xy, T.s, ta, tb, kSliceGap);
                // a lane that began or ended at a cut merged away: carried straight on to the line across the road at
                // the section's own cut
                auto carried = [&](const Vec2& from, const Vec2& way, double u) {
                    const double sigma = c.dir > 0 ? u : c.length - u;
                    const Vec2 spine = pointAt(e.xy, e.s, sigma), along = tangentAtStation(e.xy, e.s, sigma) * static_cast<double>(c.dir);
                    const double rate = dot(way, along);
                    return rate > 0 ? from + way * (dot(spine - from, along) / rate) : from;
                };
                if (m.u0() > c0 + kCutTolerance) pts.insert(pts.begin(), carried(pts.front(), tangentAtStation(T.xy, T.s, ta), c0));
                if (m.u1() < c1 - kCutTolerance) pts.push_back(carried(pts.back(), tangentAtStation(T.xy, T.s, tb), c1));
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
                    // (a lane carried on to the cut ends or begins here too)
                    const bool ends = m->u1() <= c1 + kCutTolerance && !arriving;
                    const bool begins = m->u0() >= c0 - kCutTolerance && !leaving;
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
                out.unjoined.emplace_back(p, q);
                addProblem(out.problems, "lane %u ends at (%.2f, %.2f), %.3f m and %.1f degrees from where lane %u begins: "
                                         "not joined (its road does not carry on as that one alone)", p, end.x, end.y, gap,
                           std::acos(std::min(1.0, cosine)) * kDegreesPerRadian, q);
            }
        }
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
        bool carryOn = true;   // made of one carry-on's or node's boxes alone
        for (std::size_t b : boxesOfJunction[j]) {
            if (boxes[b].edge != lastEdge) roads += (roads.empty() ? "" : ", ") + edgeAt(boxes[b].edge).id;
            lastEdge = boxes[b].edge;
            carryOn = carryOn && boxes[b].carryOn >= 0 && boxes[b].carryOn == boxes[boxesOfJunction[j].front()].carryOn;
        }
        JunctionNote note;
        note.origin = !carryOn ? JunctionOrigin::Box
                      : roadEndsOfNode[static_cast<std::size_t>(boxes[boxesOfJunction[j].front()].carryOn)] == 2 ? JunctionOrigin::CarryOn
                                                                                                                 : JunctionOrigin::Node;
        note.at = at;
        note.roads = roads;
        // an outline with too few corners, or one that crosses itself (two of its streets' stop lines overlap, the
        // streets meeting at too sharp an angle for the junction test): refused, no connectors there
        const std::size_t n = outline[j].size();
        const bool tangled = crossesItself(outline[j]);
        note.refused = n < 3 || tangled;
        out.junctionNotes.push_back(note);
        if (n < 3)
            addProblem(out.problems, "junction %zu at (%.1f, %.1f) (roads %s): refused: its outline has %zu corners", j, at.x,
                       at.y, roads.c_str(), n);
        if (tangled)
            addProblem(out.problems, "junction %zu at (%.1f, %.1f) (roads %s): refused: its roads meet at too shallow an angle "
                                     "for the junction test (its outline crosses itself)", j, at.x, at.y, roads.c_str());
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
        if (!carryOn && (arrivingAt[j].size() < 2 || leavingFrom[j].size() < 2))
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

    // ---- the paths through the junctions, then every next list -----------------------------------------------------
    const std::chrono::steady_clock::time_point tConnect = std::chrono::steady_clock::now();
    connectJunctions(r, look, opts.vehicles, next, out);
    out.connectMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tConnect).count();
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        std::sort(next[p].begin(), next[p].end());
        G.paths[p].next.first = static_cast<uint32_t>(G.nextPaths.size());
        G.nextPaths.insert(G.nextPaths.end(), next[p].begin(), next[p].end());
        G.paths[p].next.count = static_cast<uint32_t>(next[p].size());
    }

    for (const std::string& p : validate(G)) out.problems.push_back("lane graph: " + p);
    reachReport(out);
    out.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
