#include "engine/procgen/city/roads/lanes/lane_graph_crossings.h"

#include "engine/procgen/city/polygon.h"    // bounds, pointInPolygon
#include "engine/procgen/city/polyline.h"   // distToPolyline

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <utility>

namespace engine {
namespace roads::lanes {

namespace {

// Room for one problem's words.
constexpr std::size_t kMessageBytes = 512;
// The spatial index's cells are at least as wide as the farthest two points can be apart and still touch; on a world so
// large that would make more cells than this along one side, they are made wider (fewer, fuller cells; the same pairs).
constexpr double kMostCellsAcross = 1 << 20;
// How many paths a "follows" search walks through at most (it stops far sooner: only paths shorter than touching
// distance are walked through).
constexpr std::size_t kMostFollowSteps = 64;

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

const char* kindWord(Meeting m) {
    switch (m) {
    case Meeting::Cross: return "a cross";
    case Meeting::Join: return "a join";
    case Meeting::Split: return "a split";
    }
    return "a meeting";
}
std::string pathWord(const LaneGraph& g, uint32_t p) {
    char b[48];
    std::snprintf(b, sizeof b, "%s %u", g.paths[p].kind == PathKind::Connector ? "connector" : "lane", p);
    return b;
}

// ONE PATH, SAMPLED: its points every kCrossingStep or less (the first at its start, the last exactly at its end), their
// distances along it and heights, their bounds, and half the widest vehicle allowed on it.
struct Sampled {
    std::vector<Vec2> xy;
    std::vector<double> s, z;
    Vec2 lo, hi;
    double halfWidth = 0;
    std::size_t last() const { return xy.size() - 1; }
};

Sampled samplePath(const LaneGraph& g, uint32_t p, double step, double halfWidth) {
    Sampled out;
    out.halfWidth = halfWidth;
    const double length = g.paths[p].length;
    const int n = std::max(1, static_cast<int>(std::ceil(length / step)));
    out.xy.reserve(static_cast<std::size_t>(n) + 1);
    for (int k = 0; k <= n; ++k) {
        const double s = k == n ? length : length * k / n;
        out.s.push_back(s);
        out.xy.push_back(g.pointAt(PathId(p), s));
        out.z.push_back(g.heightAt(PathId(p), s));
    }
    bounds(out.xy, out.lo, out.hi);
    return out;
}

bool insideGrown(const Vec2& p, const Vec2& lo, const Vec2& hi, double by) {
    return p.x >= lo.x - by && p.x <= hi.x + by && p.y >= lo.y - by && p.y <= hi.y + by;
}

// ONE MEETING OF TWO SAMPLED PATHS: a patch of sample pairs that touch, joined to its neighbours one sample along either
// path. Its reach along each (sample indices), and which corners of the two paths it holds.
struct Patch {
    std::size_t i0 = 0, i1 = 0, j0 = 0, j1 = 0;
    bool startsBoth = false;   // the start of A with the start of B
    bool endsBoth = false;     // the end of A with the end of B
    bool endStart = false;     // the end of A with the start of B
    bool startEnd = false;     // the start of A with the end of B
};

// Every meeting of A and B: sample pairs closer in plan than `reach`, at one level (heights within `clearance`).
std::vector<Patch> meetings(const Sampled& A, const Sampled& B, double reach, double clearance) {
    std::vector<Patch> out;
    // only the samples of each near the other's bounds can touch it
    std::vector<std::size_t> ia, ib;
    for (std::size_t i = 0; i < A.xy.size(); ++i)
        if (insideGrown(A.xy[i], B.lo, B.hi, reach)) ia.push_back(i);
    if (ia.empty()) return out;
    for (std::size_t j = 0; j < B.xy.size(); ++j)
        if (insideGrown(B.xy[j], A.lo, A.hi, reach)) ib.push_back(j);
    if (ib.empty()) return out;
    const std::size_t rows = ia.size(), cols = ib.size();
    const double reach2 = reach * reach;
    // each touching cell's patch, by union-find (-1: not touching)
    std::vector<int> parent(rows * cols, -1);
    auto find = [&](int c) {
        while (parent[static_cast<std::size_t>(c)] != c) {
            parent[static_cast<std::size_t>(c)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(c)])];
            c = parent[static_cast<std::size_t>(c)];
        }
        return c;
    };
    auto unite = [&](int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[static_cast<std::size_t>(std::max(a, b))] = std::min(a, b);
    };
    for (std::size_t r = 0; r < rows; ++r) {
        const Vec2 p = A.xy[ia[r]];
        const double zp = A.z[ia[r]];
        for (std::size_t c = 0; c < cols; ++c) {
            const Vec2 d = p - B.xy[ib[c]];
            if (!(d.x * d.x + d.y * d.y < reach2) || std::fabs(zp - B.z[ib[c]]) > clearance) continue;
            const int cell = static_cast<int>(r * cols + c);
            parent[static_cast<std::size_t>(cell)] = cell;
            // the neighbours already visited: one sample back along B, and the row one sample back along A
            if (c > 0 && ib[c - 1] + 1 == ib[c] && parent[static_cast<std::size_t>(cell - 1)] >= 0) unite(cell, cell - 1);
            if (r > 0 && ia[r - 1] + 1 == ia[r]) {
                for (std::size_t c2 = c > 0 ? c - 1 : 0; c2 <= c + 1 && c2 < cols; ++c2) {
                    const std::size_t far = ib[c2] > ib[c] ? ib[c2] - ib[c] : ib[c] - ib[c2];
                    const int up = static_cast<int>((r - 1) * cols + c2);
                    if (far <= 1 && parent[static_cast<std::size_t>(up)] >= 0) unite(cell, up);
                }
            }
        }
    }
    // the patches, in the order their first cells come
    std::vector<int> patchOf(rows * cols, -1);
    for (std::size_t r = 0; r < rows; ++r)
        for (std::size_t c = 0; c < cols; ++c) {
            const int cell = static_cast<int>(r * cols + c);
            if (parent[static_cast<std::size_t>(cell)] < 0) continue;
            const int root = find(cell);
            if (patchOf[static_cast<std::size_t>(root)] < 0) {
                patchOf[static_cast<std::size_t>(root)] = static_cast<int>(out.size());
                Patch p;
                p.i0 = p.i1 = ia[r];
                p.j0 = p.j1 = ib[c];
                out.push_back(p);
            }
            Patch& p = out[static_cast<std::size_t>(patchOf[static_cast<std::size_t>(root)])];
            const std::size_t i = ia[r], j = ib[c];
            p.i0 = std::min(p.i0, i);
            p.i1 = std::max(p.i1, i);
            p.j0 = std::min(p.j0, j);
            p.j1 = std::max(p.j1, j);
            p.startsBoth = p.startsBoth || (i == 0 && j == 0);
            p.endsBoth = p.endsBoth || (i == A.last() && j == B.last());
            p.endStart = p.endStart || (i == A.last() && j == 0);
            p.startEnd = p.startEnd || (i == 0 && j == B.last());
        }
    return out;
}

// A path's stretch from a patch's reach along it (samples i0 to i1), widened by one sample each way.
std::pair<double, double> stretchOf(const Sampled& P, std::size_t i0, std::size_t i1) {
    return std::make_pair(i0 > 0 ? P.s[i0 - 1] : 0.0, i1 < P.last() ? P.s[i1 + 1] : P.s.back());
}

// Whether some value is in both lists (each in ascending order).
bool shareOne(const std::vector<uint32_t>& a, const std::vector<uint32_t>& b) {
    for (std::size_t i = 0, j = 0; i < a.size() && j < b.size();) {
        if (a[i] == b[j]) return true;
        if (a[i] < b[j]) ++i;
        else ++j;
    }
    return false;
}

// A SPATIAL INDEX of every path's samples: which paths have a sample in each square cell, as sorted (cell, path) pairs.
struct CellIndex {
    double cell = 1;
    std::vector<std::pair<uint64_t, uint32_t>> entries;
    static uint64_t key(int64_t ix, int64_t iy) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(static_cast<int32_t>(ix))) << 32) |
               static_cast<uint64_t>(static_cast<uint32_t>(static_cast<int32_t>(iy)));
    }
    int64_t at(double v) const { return static_cast<int64_t>(std::floor(v / cell)); }
    // The paths with a sample in any of these cells, in id order, once each.
    std::vector<uint32_t> pathsIn(std::vector<uint64_t>& keys) const {
        std::sort(keys.begin(), keys.end());
        keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
        std::vector<uint32_t> out;
        for (uint64_t k : keys) {
            std::vector<std::pair<uint64_t, uint32_t>>::const_iterator it = std::lower_bound(
                entries.begin(), entries.end(), std::make_pair(k, uint32_t(0)));
            for (; it != entries.end() && it->first == k; ++it) out.push_back(it->second);
        }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return out;
    }
};

}  // namespace

CrossingReport findCrossingPoints(LaneGraph& g, const CrossingOptions& opts) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    CrossingReport rep;
    g.crossings.clear();
    const uint32_t n = static_cast<uint32_t>(g.paths.size());
    const double step = kCrossingStep, margin = kCrossingMargin, clearance = opts.clearance;

    // each vehicle tag's width, where the caller gave one (-1: none)
    std::vector<double> tagWidth(std::min<std::size_t>(g.vehicleTagNames.size(), kMaxTagNames), -1.0);
    for (std::size_t i = 0; i < tagWidth.size(); ++i)
        for (const VehicleSize& v : opts.vehicles)
            if (v.tag == g.vehicleTagNames[i]) tagWidth[i] = std::max(tagWidth[i], v.width);
    // half the widest vehicle allowed on each path (its own width where no vehicle on it has one), and every path sampled
    std::vector<Sampled> samples(n);
    double widest = 0;
    for (uint32_t p = 0; p < n; ++p) {
        double w = -1;
        for (std::size_t i = 0; i < tagWidth.size(); ++i)
            if (g.paths[p].vehicleTags & (TagMask(1) << i)) w = std::max(w, tagWidth[i]);
        if (w < 0) w = g.paths[p].width;
        widest = std::max(widest, w);
        samples[p] = samplePath(g, p, step, w / 2);
    }
    const double farthestReach = widest + margin;

    // what leads into each path, in id order (next lists hold the other way)
    std::vector<std::vector<uint32_t>> ledFrom(n), leadsTo(n);
    for (uint32_t p = 0; p < n; ++p)
        for (uint32_t k = g.paths[p].next.first; k < g.paths[p].next.end(); ++k) {
            const uint32_t q = g.nextPaths[k].index;
            if (q >= n) continue;
            leadsTo[p].push_back(q);
            ledFrom[q].push_back(p);
        }
    for (std::vector<uint32_t>& v : leadsTo) std::sort(v.begin(), v.end());
    // Whether `to` follows `from`: one of the paths leading on from `from`, or reached from them through paths shorter in
    // all than `reach` (a car on the one is following a car on the other there, not meeting it).
    auto follows = [&](uint32_t from, uint32_t to, double reach) {
        std::vector<std::pair<uint32_t, double>> todo;   // (path, how far its start is from `from`'s end)
        std::vector<uint32_t> seen;
        for (uint32_t q : leadsTo[from]) todo.emplace_back(q, 0.0);
        for (std::size_t k = 0; k < todo.size() && k < kMostFollowSteps; ++k) {
            const uint32_t q = todo[k].first;
            if (q == to) return true;
            const double beyond = todo[k].second + g.paths[q].length;
            if (beyond >= reach || std::find(seen.begin(), seen.end(), q) != seen.end()) continue;
            seen.push_back(q);
            for (uint32_t r : leadsTo[q]) todo.emplace_back(r, beyond);
        }
        return false;
    };

    // the spatial index
    CellIndex index;
    {
        double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
        for (const Sampled& sp : samples) {
            x0 = std::min(x0, sp.lo.x);
            y0 = std::min(y0, sp.lo.y);
            x1 = std::max(x1, sp.hi.x);
            y1 = std::max(y1, sp.hi.y);
        }
        index.cell = std::max(farthestReach, kCrossingStep);
        if (x1 > x0) index.cell = std::max(index.cell, std::max(x1 - x0, y1 - y0) / kMostCellsAcross);
        for (uint32_t p = 0; p < n; ++p) {
            uint64_t lastKey = 0;
            bool any = false;
            for (const Vec2& q : samples[p].xy) {
                const uint64_t k = CellIndex::key(index.at(q.x), index.at(q.y));
                if (any && k == lastKey) continue;
                index.entries.emplace_back(k, p);
                lastKey = k;
                any = true;
            }
        }
        std::sort(index.entries.begin(), index.entries.end());
        index.entries.erase(std::unique(index.entries.begin(), index.entries.end()), index.entries.end());
    }

    // ---- two paths: every connector against every path near it ----
    for (uint32_t c = 0; c < n; ++c) {
        if (g.paths[c].kind != PathKind::Connector) continue;
        const Sampled& A = samples[c];
        std::vector<uint64_t> keys;
        for (const Vec2& q : A.xy) {
            const int64_t ix = index.at(q.x), iy = index.at(q.y);
            for (int64_t dx = -1; dx <= 1; ++dx)
                for (int64_t dy = -1; dy <= 1; ++dy) keys.push_back(CellIndex::key(ix + dx, iy + dy));
        }
        for (uint32_t q : index.pathsIn(keys)) {
            // (a pair of connectors is taken once, from the lower; a lane never starts a pair)
            if (q == c || (g.paths[q].kind == PathKind::Connector && q < c)) continue;
            const Sampled& B = samples[q];
            const double reach = A.halfWidth + B.halfWidth + margin;
            std::vector<CrossingPoint> found;
            for (const Patch& m : meetings(A, B, reach, clearance)) {
                Meeting kind = Meeting::Cross;
                if (m.startsBoth && shareOne(ledFrom[c], ledFrom[q])) kind = Meeting::Split;
                else if (m.endsBoth && shareOne(leadsTo[c], leadsTo[q])) kind = Meeting::Join;
                else if ((m.endStart && follows(c, q, reach)) || (m.startEnd && follows(q, c, reach))) continue;
                std::pair<double, double> sa = stretchOf(A, m.i0, m.i1), sb = stretchOf(B, m.j0, m.j1);
                if (kind == Meeting::Split) sa.first = sb.first = 0;
                if (kind == Meeting::Join) {
                    sa.second = A.s.back();
                    sb.second = B.s.back();
                }
                CrossingPoint x;
                x.meeting = kind;
                x.otherSide = OtherSide::Path;
                const bool cFirst = c < q;
                x.a = PathId(cFirst ? c : q);
                x.other = cFirst ? q : c;
                x.aFrom = cFirst ? sa.first : sb.first;
                x.aTo = cFirst ? sa.second : sb.second;
                x.otherFrom = cFirst ? sb.first : sa.first;
                x.otherTo = cFirst ? sb.second : sa.second;
                const Path& pq = g.paths[q];
                if (pq.kind == PathKind::Connector && pq.junction == g.paths[c].junction) x.junction = g.paths[c].junction;
                found.push_back(x);
            }
            // two meetings of the pair whose stretches (widened) overlap on both paths are one meeting; a split or a join
            // keeps its kind
            std::sort(found.begin(), found.end(), [](const CrossingPoint& x, const CrossingPoint& y) { return x.aFrom < y.aFrom; });
            for (bool merged = true; merged;) {
                merged = false;
                for (std::size_t i = 0; i < found.size() && !merged; ++i)
                    for (std::size_t j = i + 1; j < found.size() && !merged; ++j) {
                        CrossingPoint& x = found[i];
                        const CrossingPoint& y = found[j];
                        if (!(x.aFrom <= y.aTo && y.aFrom <= x.aTo && x.otherFrom <= y.otherTo && y.otherFrom <= x.otherTo)) continue;
                        x.aFrom = std::min(x.aFrom, y.aFrom);
                        x.aTo = std::max(x.aTo, y.aTo);
                        x.otherFrom = std::min(x.otherFrom, y.otherFrom);
                        x.otherTo = std::max(x.otherTo, y.otherTo);
                        if (x.meeting == Meeting::Cross) x.meeting = y.meeting;
                        found.erase(found.begin() + static_cast<std::ptrdiff_t>(j));
                        merged = true;
                    }
            }
            for (const CrossingPoint& x : found) {
                ++rep.byMeeting[static_cast<std::size_t>(x.meeting)];
                if (!x.junction.valid()) ++(g.paths[q].kind == PathKind::Connector ? rep.betweenJunctions : rep.withLanes);
                g.crossings.push_back(x);
            }
        }
    }

    // ---- crosswalks: each path whose body could be on one ----
    for (uint32_t w = 0; w < g.crosswalks.size(); ++w) {
        const Crosswalk& cw = g.crosswalks[w];
        if (cw.outline.end() > g.planPoints.size() || cw.outline.count < 3) continue;
        const std::vector<Vec2> ring(g.planPoints.begin() + cw.outline.first, g.planPoints.begin() + cw.outline.end());
        Vec2 lo, hi;
        bounds(ring, lo, hi);
        // the way people walk across it: from the middle of its first edge to the middle of the edge halfway round
        const std::size_t half = ring.size() / 2;
        const Vec2 walkFrom = (ring[0] + ring[1]) * 0.5, walkTo = (ring[half] + ring[(half + 1) % ring.size()]) * 0.5;
        const double walkLength = distance(walkFrom, walkTo);
        const Vec2 walkDir = walkLength > 0 ? (walkTo - walkFrom) / walkLength : Vec2(1, 0);
        // its level: the heights of its junction's lane ends (none for a crosswalk out on a road: any level)
        std::vector<double> levels;
        if (cw.junction.valid() && cw.junction.index < g.junctions.size()) {
            const Junction& jn = g.junctions[cw.junction.index];
            for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k)
                levels.push_back(g.heightAt(g.junctionLanes[k], g.paths[g.junctionLanes[k].index].length));
            for (uint32_t k = jn.leaving.first; k < jn.leaving.end(); ++k) levels.push_back(g.heightAt(g.junctionLanes[k], 0));
        }
        auto atLevel = [&](double z) {
            if (levels.empty()) return true;
            for (double l : levels)
                if (std::fabs(z - l) <= clearance) return true;
            return false;
        };
        std::vector<uint64_t> keys;
        for (int64_t ix = index.at(lo.x - farthestReach); ix <= index.at(hi.x + farthestReach); ++ix)
            for (int64_t iy = index.at(lo.y - farthestReach); iy <= index.at(hi.y + farthestReach); ++iy)
                keys.push_back(CellIndex::key(ix, iy));
        for (uint32_t p : index.pathsIn(keys)) {
            const Sampled& P = samples[p];
            const double reach = P.halfWidth + margin;
            std::vector<char> on(P.xy.size(), 0);
            for (std::size_t i = 0; i < P.xy.size(); ++i)
                on[i] = insideGrown(P.xy[i], lo, hi, reach) && atLevel(P.z[i]) &&
                        (pointInPolygon(ring, P.xy[i]) || distToPolyline(P.xy[i], ring, true) < reach);
            for (std::size_t i = 0; i < on.size();) {
                if (!on[i]) {
                    ++i;
                    continue;
                }
                std::size_t e = i;
                double t0 = 1e300, t1 = -1e300;
                for (; e < on.size() && on[e]; ++e) {
                    const double t = dot(P.xy[e] - walkFrom, walkDir);
                    t0 = std::min(t0, t);
                    t1 = std::max(t1, t);
                }
                const std::pair<double, double> sp = stretchOf(P, i, e - 1);
                CrossingPoint x;
                x.a = PathId(p);
                x.otherSide = OtherSide::Crosswalk;
                x.other = w;
                x.meeting = Meeting::Cross;
                x.junction = cw.junction;
                x.aFrom = sp.first;
                x.aTo = sp.second;
                x.otherFrom = std::min(std::max(t0 - reach, 0.0), walkLength);
                x.otherTo = std::min(std::max(t1 + reach, 0.0), walkLength);
                g.crossings.push_back(x);
                ++rep.withCrosswalks;
                i = e;
            }
        }
    }

    indexCrossings(g);

    // ---- the report ----
    for (uint32_t j = 0; j < g.junctions.size(); ++j) {
        const uint32_t count = g.junctions[j].crossings.count;
        if (rep.busiestJunction == kNoIndex || count > rep.busiestCount) {
            rep.busiestJunction = j;
            rep.busiestCount = count;
        }
        if (count == 0 && g.junctions[j].connectors.count >= 2) ++rep.junctionsWithout;
        if (count > kManyCrossingPoints)
            addProblem(rep.problems, "crossing points: junction %u has %u, more than %u: a slow junction for the junction plan "
                                     "and for every car there", j, count, kManyCrossingPoints);
    }
    // the junctions' outlines, for "outside every junction"
    std::vector<std::vector<Vec2>> outlines(g.junctions.size());
    std::vector<std::pair<Vec2, Vec2>> outlineBounds(g.junctions.size());
    for (uint32_t j = 0; j < g.junctions.size(); ++j) {
        const Range& b = g.junctions[j].box;
        if (b.end() > g.planPoints.size()) continue;
        outlines[j].assign(g.planPoints.begin() + b.first, g.planPoints.begin() + b.end());
        if (!outlines[j].empty()) bounds(outlines[j], outlineBounds[j].first, outlineBounds[j].second);
    }
    for (uint32_t c = 0; c < g.crossings.size(); ++c) {
        const CrossingPoint& x = g.crossings[c];
        if (x.otherSide != OtherSide::Path) continue;
        const Path& pa = g.paths[x.a.index];
        const Path& pb = g.paths[x.other];
        const Vec2 at = g.pointAt(x.a, (x.aFrom + x.aTo) / 2);
        const std::string an = pathWord(g, x.a.index), bn = pathWord(g, x.other);
        const double shareA = pa.length > 0 ? (x.aTo - x.aFrom) / pa.length : 0;
        const double shareB = pb.length > 0 ? (x.otherTo - x.otherFrom) / pb.length : 0;
        if (shareA >= kNearlyWholeShare && shareB >= kNearlyWholeShare)
            addProblem(rep.problems, "crossing points: %s and %s are within touching distance for %.0f%% and %.0f%% of their "
                                     "lengths (%s, crossing point %u, at (%.1f, %.1f)): two paths laid nearly over each other",
                       an.c_str(), bn.c_str(), 100 * shareA, 100 * shareB, kindWord(x.meeting), c, at.x, at.y);
        bool inside = false;
        for (uint32_t j = 0; j < outlines.size() && !inside; ++j) {
            if (outlines[j].size() < 3 ||
                !insideGrown(at, outlineBounds[j].first, outlineBounds[j].second, kJunctionEndReach))
                continue;
            inside = pointInPolygon(outlines[j], at) || distToPolyline(at, outlines[j], true) <= kJunctionEndReach;
        }
        if (!inside)
            addProblem(rep.problems, "crossing points: %s and %s meet at (%.1f, %.1f) (%s, crossing point %u), outside every "
                                     "junction's outline", an.c_str(), bn.c_str(), at.x, at.y, kindWord(x.meeting), c);
    }
    rep.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return rep;
}

uint32_t placeWaitInsidePoints(LaneGraph& g) {
    uint32_t placed = 0;
    for (Path& p : g.paths) p.waitInsideAt = kNoWaitInside;
    for (uint32_t c = 0; c < g.paths.size(); ++c) {
        Path& p = g.paths[c];
        if (p.kind != PathKind::Connector || p.turn != Turn::FarTurn || !p.junction.valid() ||
            p.from.index >= g.paths.size())
            continue;
        const Vec2 mine = g.directionAt(p.from, g.paths[p.from.index].length);
        // The crossing points along it are listed by where their stretch on it begins: the first with a path from a facing
        // approach is where the front stops.
        for (uint32_t k = p.crossings.first; k < p.crossings.end() && k < g.pathCrossings.size(); ++k) {
            const CrossingPoint& x = g.crossings[g.pathCrossings[k].index];
            if (x.otherSide != OtherSide::Path || x.meeting == Meeting::Split || x.junction != p.junction) continue;
            const PathId o = x.a == PathId(c) ? x.otherPath() : x.a;
            const Path& op = g.paths[o.index];
            if (op.kind != PathKind::Connector || op.from.index >= g.paths.size()) continue;
            const Vec2 theirs = g.directionAt(op.from, g.paths[op.from.index].length);
            if (!(dot(mine, theirs) < kOncomingDot)) continue;
            const double at = x.a == PathId(c) ? x.aFrom : x.otherFrom;
            if (at > 0) {
                p.waitInsideAt = at;
                ++placed;
            }
            break;
        }
    }
    return placed;
}

}  // namespace roads::lanes
}  // namespace engine
