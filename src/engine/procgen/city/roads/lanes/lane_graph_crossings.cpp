#include "engine/procgen/city/roads/lanes/lane_graph_crossings.h"

#include "engine/procgen/city/polygon.h"    // bounds, pointInPolygon
#include "engine/procgen/city/polyline.h"   // distToPolyline, distSegmentToSegment
#include "engine/procgen/city/roads/lanes/swept_body.h"   // a long body along a path (#284 item 9)

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

// THE PATCHES of touching sample pairs, given the samples of A (`ia`, rising) and of B (`ib`) that may touch at all,
// whether sample i of A touches sample j of B (`touches`), and each path's last sample.
// `visit(patch, i, j)` is shown each touching pair once the patches are known, by the index of the patch it is in.
template <typename Touches, typename Visit>
std::vector<Patch> patchesOf(const std::vector<std::size_t>& ia, const std::vector<std::size_t>& ib, std::size_t lastA,
                             std::size_t lastB, const Touches& touches, const Visit& visit) {
    std::vector<Patch> out;
    const std::size_t rows = ia.size(), cols = ib.size();
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
        for (std::size_t c = 0; c < cols; ++c) {
            if (!touches(ia[r], ib[c])) continue;
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
            p.endsBoth = p.endsBoth || (i == lastA && j == lastB);
            p.endStart = p.endStart || (i == lastA && j == 0);
            p.startEnd = p.startEnd || (i == 0 && j == lastB);
            visit(static_cast<std::size_t>(patchOf[static_cast<std::size_t>(root)]), i, j);
        }
    return out;
}

// Every meeting of A and B: sample pairs closer in plan than `reach`, at one level (heights within `clearance`).
std::vector<Patch> meetings(const Sampled& A, const Sampled& B, double reach, double clearance) {
    // only the samples of each near the other's bounds can touch it
    std::vector<std::size_t> ia, ib;
    for (std::size_t i = 0; i < A.xy.size(); ++i)
        if (insideGrown(A.xy[i], B.lo, B.hi, reach)) ia.push_back(i);
    if (ia.empty()) return {};
    for (std::size_t j = 0; j < B.xy.size(); ++j)
        if (insideGrown(B.xy[j], A.lo, A.hi, reach)) ib.push_back(j);
    if (ib.empty()) return {};
    const double reach2 = reach * reach;
    return patchesOf(
        ia, ib, A.last(), B.last(),
        [&](std::size_t i, std::size_t j) {
            const Vec2 d = A.xy[i] - B.xy[j];
            return d.x * d.x + d.y * d.y < reach2 && std::fabs(A.z[i] - B.z[j]) <= clearance;
        },
        [](std::size_t, std::size_t, std::size_t) {});
}

// ONE PATH'S BODIES (issue #284 item 9, ADR-0412): the longest vehicle allowed on it, its front placed every
// kCrossingStep or less from the path's start to its length past the path's end (while any of it is still on the path),
// and at each front the body's middle line from its rear end to its front end (lane_graph.h bodyOnPlan, swept_body.h
// bodyAt), along every line the body may lie along there: through each path leading in where its rear is still on one,
// and each path it leads to where its front is already on one. Half its width round that line holds the whole body.
// The heights are the path's under the body's middle; the bounds are each front's lines' and all of them.
struct Bodies {
    std::vector<double> front;                    // per sample: the front, metres along the path (past its end: on the next)
    std::vector<double> z;
    std::vector<uint32_t> firstLine;              // per sample and one more: sample i's lines are lines[first[i], [i + 1])
    std::vector<std::pair<Vec2, Vec2>> lines;     // rear end, front end
    std::vector<Vec2> lo, hi;
    Vec2 allLo, allHi;
    double halfWidth = 0, length = 0, pathLength = 0;
    std::size_t last() const { return front.size() - 1; }
};

Bodies bodiesOf(const LaneGraph& g, uint32_t p, double step, double halfWidth, double bodyLength,
                const std::vector<BodyLine>& ways) {
    Bodies out;
    out.halfWidth = halfWidth;
    out.length = bodyLength;
    out.pathLength = g.paths[p].length;
    const double span = out.pathLength + bodyLength;
    const int n = std::max(1, static_cast<int>(std::ceil(span / step)));
    std::vector<Vec2> all;
    for (int k = 0; k <= n; ++k) {
        const double s = k == n ? span : span * k / n;
        out.front.push_back(s);
        out.z.push_back(g.heightAt(PathId(p), std::min(std::max(s - bodyLength / 2, 0.0), out.pathLength)));
        const std::size_t first = out.lines.size();
        out.firstLine.push_back(static_cast<uint32_t>(first));
        // every way gives the same body unless its rear is on a path leading in or its front on one it leads to
        const bool atAnEnd = s < bodyLength || s > out.pathLength;
        for (std::size_t w = 0; w < (atAnEnd ? ways.size() : std::min<std::size_t>(ways.size(), 1)); ++w) {
            const PlanRectangle body = bodyAt(ways[w], s, bodyLength, 2 * halfWidth);
            const std::pair<Vec2, Vec2> line(body.alongMiddle(0), body.alongMiddle(1));
            bool again = false;
            for (std::size_t m = first; m < out.lines.size() && !again; ++m)
                again = distance(out.lines[m].first, line.first) < kSweptSamePoint &&
                        distance(out.lines[m].second, line.second) < kSweptSamePoint;
            if (!again) out.lines.push_back(line);
        }
        Vec2 lo(1e300, 1e300), hi(-1e300, -1e300);
        for (std::size_t m = first; m < out.lines.size(); ++m)
            for (const Vec2& q : {out.lines[m].first, out.lines[m].second}) {
                lo = Vec2(std::min(lo.x, q.x), std::min(lo.y, q.y));
                hi = Vec2(std::max(hi.x, q.x), std::max(hi.y, q.y));
                all.push_back(q);
            }
        out.lo.push_back(lo);
        out.hi.push_back(hi);
    }
    out.firstLine.push_back(static_cast<uint32_t>(out.lines.size()));
    bounds(all, out.allLo, out.allHi);
    return out;
}

// Whether two boxes come within `by` of each other.
bool boxesWithin(const Vec2& alo, const Vec2& ahi, const Vec2& blo, const Vec2& bhi, double by) {
    return ahi.x >= blo.x - by && alo.x <= bhi.x + by && ahi.y >= blo.y - by && alo.y <= bhi.y + by;
}

// ONE MEETING OF TWO PATHS' BODIES, on each path (metres along it): the touching bodies' fronts and rears (the hindmost
// front, the foremost rear) and the places where they touch, nearest and farthest.
struct BodySide {
    double firstFront = 1e300, lastRear = -1e300, touchFrom = 1e300, touchTo = -1e300;
    void note(double front, double length, double touch) {
        firstFront = std::min(firstFront, front);
        lastRear = std::max(lastRear, front - length);
        touchFrom = std::min(touchFrom, touch);
        touchTo = std::max(touchTo, touch);
    }
};
struct BodyMeeting {
    BodySide a, b;
};

// Every meeting of A's bodies and B's: pairs of fronts whose bodies' middle lines come closer than `reach` (half of
// each body's width and the margin), at one level.
std::vector<BodyMeeting> bodyMeetings(const Bodies& A, const Bodies& B, double reach, double clearance) {
    std::vector<std::size_t> ia, ib;
    for (std::size_t i = 0; i < A.front.size(); ++i)
        if (boxesWithin(A.lo[i], A.hi[i], B.allLo, B.allHi, reach)) ia.push_back(i);
    if (ia.empty()) return {};
    for (std::size_t j = 0; j < B.front.size(); ++j)
        if (boxesWithin(B.lo[j], B.hi[j], A.allLo, A.allHi, reach)) ib.push_back(j);
    if (ib.empty()) return {};
    // Where along each path the two bodies of fronts i and j touch (the nearest points of their middle lines, as
    // metres along: a body's middle line runs from its length behind its front to its front); false if they do not.
    const auto touchAt = [&](std::size_t i, std::size_t j, double& atA, double& atB) {
        if (std::fabs(A.z[i] - B.z[j]) > clearance || !boxesWithin(A.lo[i], A.hi[i], B.lo[j], B.hi[j], reach)) return false;
        double nearest = reach;
        for (uint32_t a = A.firstLine[i]; a < A.firstLine[i + 1]; ++a)
            for (uint32_t b = B.firstLine[j]; b < B.firstLine[j + 1]; ++b) {
                Real ta = 0, tb = 0;
                const double d = distSegmentToSegment(A.lines[a].first, A.lines[a].second, B.lines[b].first,
                                                      B.lines[b].second, &ta, &tb);
                if (!(d < nearest)) continue;
                nearest = d;
                atA = A.front[i] - A.length * (1 - ta);
                atB = B.front[j] - B.length * (1 - tb);
            }
        return nearest < reach;
    };
    std::vector<BodyMeeting> out;
    const std::vector<Patch> patches = patchesOf(
        ia, ib, A.last(), B.last(),
        [&](std::size_t i, std::size_t j) {
            double atA = 0, atB = 0;
            return touchAt(i, j, atA, atB);
        },
        [&](std::size_t patch, std::size_t i, std::size_t j) {
            if (out.size() <= patch) out.resize(patch + 1);
            double atA = 0, atB = 0;
            touchAt(i, j, atA, atB);
            out[patch].a.note(A.front[i], A.length, atA);
            out[patch].b.note(B.front[j], B.length, atB);
        });
    return out;
}

// A MEETING'S STRETCH OF ONE PATH. The driving model counts a body as on a stretch when its front is past the stretch's
// start and its rear short of its end, so every touching body must be: the stretch starts no later than the hindmost
// touching front and ends no earlier than the foremost touching rear. Where those two overlap that is the stretch, the
// shortest that holds them all. Where they do not (a short touch, by part of a long body) any one place between them
// serves: the place they touch, kept a little way past the path's start where it can be, so that a body standing at its
// line, its front at the start and all of it on the straight lane behind, is not on a stretch only bodies half round the
// turn reach. Then widened by a sample each way (the true edge lies within a step of the last sample that touches) and
// clamped onto the path. False when the touch lies wholly off the path (on the part of a body still on the path before,
// or already on the next: their own pair's record).
bool bodyStretchOn(const BodySide& s, double pathLength, std::pair<double, double>& out) {
    if (s.touchTo < 0 || s.touchFrom > pathLength) return false;
    double from = s.firstFront, to = s.lastRear;
    if (from > to) {
        const double place = std::min(std::max(std::max((s.touchFrom + s.touchTo) / 2, 2 * kCrossingStep), s.lastRear),
                                      s.firstFront);
        from = to = place;
    }
    out.first = std::min(std::max(from - kCrossingStep, 0.0), pathLength);
    out.second = std::min(std::max(to + kCrossingStep, 0.0), pathLength);
    return true;
}

// HOW FAR A BODY `length` LONG CUTS INSIDE a bend of radius `radius` at its middle (its chord's sagitta: the bumpers on
// the curve), metres; nothing on a straight.
double swingRound(double length, double radius) {
    if (!(length > 0) || !std::isfinite(radius) || !(radius > 0)) return 0;
    return radius * (1 - std::cos(std::min(length / (2 * radius), 1.5707963267948966)));
}

// ONE BODY PART of a path (see findCrossingPoints): the tags it stands for, whether its bodies have a length that swings
// out of the first pass's reach, and its bodies (an index into the parts' bodies).
struct BodyPart {
    TagMask tags = 0;
    bool isLong = false;
    uint32_t bodies = 0;
};

// THE RECORDS OF ONE PAIR OF PATHS for one pair of body parts: the tags on `a` and on the other path they concern.
struct SizedRecords {
    TagMask aTags = kEveryVehicle, otherTags = kEveryVehicle;
    std::vector<CrossingPoint> records;
};

// RECORDS THAT COME OUT THE SAME ARE ONE: two pairs of parts with the same records and the same tags on one path become
// one, their tags on the other path joined (first across `a`, then across the other path, until nothing joins); a set of
// tags that covers every part of its path (`allA`, `allOther`: the tags of all of a path's parts) concerns every
// vehicle.
void sameRecordsJoined(std::vector<SizedRecords>& sized, TagMask allA, TagMask allOther) {
    for (bool joined = true; joined;) {
        joined = false;
        for (std::size_t i = 0; i < sized.size() && !joined; ++i)
            for (std::size_t j = i + 1; j < sized.size() && !joined; ++j) {
                SizedRecords& x = sized[i];
                const SizedRecords& y = sized[j];
                if (x.records != y.records) continue;
                if (x.otherTags == y.otherTags) x.aTags |= y.aTags;
                else if (x.aTags == y.aTags) x.otherTags |= y.otherTags;
                else continue;
                sized.erase(sized.begin() + static_cast<std::ptrdiff_t>(j));
                joined = true;
            }
    }
    for (SizedRecords& s : sized) {
        if ((s.aTags & allA) == allA) s.aTags = kEveryVehicle;
        if ((s.otherTags & allOther) == allOther) s.otherTags = kEveryVehicle;
    }
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

    // each vehicle tag's width and length, where the caller gave one (width -1: none)
    std::vector<double> tagWidth(std::min<std::size_t>(g.vehicleTagNames.size(), kMaxTagNames), -1.0);
    std::vector<double> tagLength(tagWidth.size(), 0.0);
    for (std::size_t i = 0; i < tagWidth.size(); ++i)
        for (const VehicleSize& v : opts.vehicles)
            if (v.tag == g.vehicleTagNames[i]) {
                tagWidth[i] = std::max(tagWidth[i], v.width);
                tagLength[i] = std::max(tagLength[i], v.length);
            }
    // half the widest vehicle allowed on each path (its own width where no vehicle on it has one), the longest, and
    // every path sampled
    std::vector<Sampled> samples(n);
    std::vector<double> longestOn(n, 0.0);
    double widest = 0, longestAnywhere = 0;
    for (uint32_t p = 0; p < n; ++p) {
        double w = -1;
        for (std::size_t i = 0; i < tagWidth.size(); ++i)
            if (g.paths[p].vehicleTags & (TagMask(1) << i)) {
                w = std::max(w, tagWidth[i]);
                longestOn[p] = std::max(longestOn[p], tagLength[i]);
            }
        if (w < 0) w = g.paths[p].width;
        widest = std::max(widest, w);
        longestAnywhere = std::max(longestAnywhere, longestOn[p]);
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

    // LONG BODIES (issue #284 item 9, ADR-0412), when any vehicle has a length: each path's BODY PARTS -- one per size of
    // body that swings out of the first pass's reach on it (its tags of one width and length whose swing round the path's
    // tightest bend is more than kSwingCoveredByMargin), placed along every way into and on from it; and one for all its
    // other tags, its bodies points on its line at their widest width -- and a spatial index of each path's longest
    // (points every half cell along each body's middle line, so two lines within touching distance have points within
    // two cells of each other).
    std::vector<std::vector<BodyPart>> parts;
    std::vector<Bodies> partBodies;
    std::vector<uint32_t> indexBodies;   // per path: its longest part's bodies
    std::vector<char> hasLong(n, 0);
    const auto allTagsOn = [&](uint32_t p) {
        TagMask all = 0;
        for (const BodyPart& b : parts[p]) all |= b.tags;
        return all;
    };
    CellIndex bodyIndex;
    bodyIndex.cell = index.cell;
    // the cells round a set of middle lines' points (two cells either way)
    auto bodyKeys = [&](const Bodies& b) {
        std::vector<uint64_t> keys;
        const double spacing = bodyIndex.cell / 2;
        for (const std::pair<Vec2, Vec2>& line : b.lines) {
            const int pieces = std::max(1, static_cast<int>(std::ceil(distance(line.first, line.second) / spacing)));
            for (int k = 0; k <= pieces; ++k) {
                const Vec2 q = line.first + (line.second - line.first) * (static_cast<double>(k) / pieces);
                keys.push_back(CellIndex::key(bodyIndex.at(q.x), bodyIndex.at(q.y)));
            }
        }
        std::sort(keys.begin(), keys.end());
        keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
        return keys;
    };
    if (longestAnywhere > 0) {
        parts.resize(n);
        indexBodies.assign(n, 0);
        for (uint32_t p = 0; p < n; ++p) {
            const Path& path = g.paths[p];
            const std::vector<Vec2> own = pathPoints(g, PathId(p));
            // its tags: the sizes that swing out of reach here, and the rest. Sizes whose reach here (half the width and
            // the swing round the tightest bend) is within kSwingCoveredByMargin of a larger one's go with it, its part's
            // body as wide and as long as the widest and longest of them: the margin covers the difference.
            struct Size {
                double width = 0, length = 0;
                TagMask tags = 0;
                double reach = 0;
            };
            std::vector<Size> longSizes;
            TagMask shortTags = 0;
            double shortWidth = -1;
            for (std::size_t i = 0; i < tagWidth.size(); ++i) {
                const TagMask bit = TagMask(1) << i;
                if (!(path.vehicleTags & bit)) continue;
                const double swing = swingRound(tagLength[i], path.tightestRadius);
                if (tagWidth[i] < 0 || swing <= kSwingCoveredByMargin) {
                    shortTags |= bit;
                    shortWidth = std::max(shortWidth, tagWidth[i]);
                    continue;
                }
                longSizes.push_back(Size{tagWidth[i], tagLength[i], bit, tagWidth[i] / 2 + swing});
            }
            std::sort(longSizes.begin(), longSizes.end(), [](const Size& x, const Size& y) {
                return x.reach != y.reach ? x.reach > y.reach : x.tags < y.tags;
            });
            {
                std::vector<Size> grouped;
                for (const Size& s : longSizes) {
                    std::vector<Size>::iterator it = std::find_if(grouped.begin(), grouped.end(), [&](const Size& gr) {
                        return gr.reach - s.reach <= kSwingCoveredByMargin;
                    });
                    if (it == grouped.end()) {
                        grouped.push_back(s);
                        continue;
                    }
                    it->width = std::max(it->width, s.width);
                    it->length = std::max(it->length, s.length);
                    it->tags |= s.tags;
                }
                longSizes.swap(grouped);
            }
            if (shortWidth < 0) shortWidth = path.width;
            hasLong[p] = longSizes.empty() ? 0 : 1;
            // the short part: its bodies points on its own line (it has one even with no tag left, for a path open to
            // nobody: the other path's long bodies still meet its first-pass records)
            if (shortTags != 0 || longSizes.empty()) {
                parts[p].push_back(BodyPart{shortTags, false, static_cast<uint32_t>(partBodies.size())});
                partBodies.push_back(bodiesOf(g, p, step, shortWidth / 2, 0, {makeBodyLine({}, own, {}, 0)}));
                indexBodies[p] = parts[p].back().bodies;
            }
            // each long part. A connector's bodies lie back along the lane it leaves and on along the lane it joins: its
            // swing through the turn, from the first metre to where its rear leaves it, is the connector's, and so are its
            // records. A lane's lie straight on past its own ends: a body still half round the turn behind it is the
            // turn's (its tail is on that connector), so a car that came in straight never inherits a turning bus's swing.
            double longestHere = 0;
            for (const Size& s : longSizes) {
                std::vector<std::vector<Vec2>> before, after;
                if (path.kind == PathKind::Connector) {
                    for (uint32_t q : ledFrom[p]) before.push_back(pathPoints(g, PathId(q)));
                    for (uint32_t q : leadsTo[p]) after.push_back(pathPoints(g, PathId(q)));
                }
                if (before.empty()) before.emplace_back();
                if (after.empty()) after.emplace_back();
                std::vector<BodyLine> ways;
                for (const std::vector<Vec2>& b : before)
                    for (const std::vector<Vec2>& a : after) ways.push_back(makeBodyLine(b, own, a, s.length));
                parts[p].push_back(BodyPart{s.tags, true, static_cast<uint32_t>(partBodies.size())});
                partBodies.push_back(bodiesOf(g, p, step, s.width / 2, s.length, ways));
                if (s.length > longestHere) {
                    longestHere = s.length;
                    indexBodies[p] = parts[p].back().bodies;
                }
            }
            for (uint64_t k : bodyKeys(partBodies[indexBodies[p]])) bodyIndex.entries.emplace_back(k, p);
        }
        std::sort(bodyIndex.entries.begin(), bodyIndex.entries.end());
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
        std::vector<uint32_t> near = index.pathsIn(keys);
        if (!parts.empty()) {   // ... and every path whose longest bodies come near its own
            std::vector<uint64_t> around;
            for (uint64_t k : bodyKeys(partBodies[indexBodies[c]])) {
                const int64_t ix = static_cast<int32_t>(static_cast<uint32_t>(k >> 32));
                const int64_t iy = static_cast<int32_t>(static_cast<uint32_t>(k & 0xFFFFFFFFu));
                for (int64_t dx = -2; dx <= 2; ++dx)
                    for (int64_t dy = -2; dy <= 2; ++dy) around.push_back(CellIndex::key(ix + dx, iy + dy));
            }
            const std::vector<uint32_t> more = bodyIndex.pathsIn(around);
            near.insert(near.end(), more.begin(), more.end());
            std::sort(near.begin(), near.end());
            near.erase(std::unique(near.begin(), near.end()), near.end());
        }
        for (uint32_t q : near) {
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
            const auto mergeOverlapping = [](std::vector<CrossingPoint>& list) {
                std::sort(list.begin(), list.end(), [](const CrossingPoint& x, const CrossingPoint& y) { return x.aFrom < y.aFrom; });
                for (bool merged = true; merged;) {
                    merged = false;
                    for (std::size_t i = 0; i < list.size() && !merged; ++i)
                        for (std::size_t j = i + 1; j < list.size() && !merged; ++j) {
                            CrossingPoint& x = list[i];
                            const CrossingPoint& y = list[j];
                            if (!(x.aFrom <= y.aTo && y.aFrom <= x.aTo && x.otherFrom <= y.otherTo && y.otherFrom <= x.otherTo))
                                continue;
                            x.aFrom = std::min(x.aFrom, y.aFrom);
                            x.aTo = std::max(x.aTo, y.aTo);
                            x.otherFrom = std::min(x.otherFrom, y.otherFrom);
                            x.otherTo = std::max(x.otherTo, y.otherTo);
                            if (x.meeting == Meeting::Cross) x.meeting = y.meeting;
                            list.erase(list.begin() + static_cast<std::ptrdiff_t>(j));
                            merged = true;
                        }
                }
            };
            mergeOverlapping(found);
            // LONG BODIES, SIZE BY SIZE (ADR-0412): for each pair of body sizes, one on each path (its own size groups, or
            // its short bodies), the first pass's records widened where those two bodies touch at their length, and
            // crosses of their own where they touch outside every record (longSweep); each concerns only the tags of its
            // two sizes. Pairs of sizes that come out the same are recorded once (sameRecordsJoined). Two paths one of
            // which leads into the other are a car following another, never a meeting of long bodies.
            std::vector<SizedRecords> sized;
            if (!parts.empty() && (hasLong[c] || hasLong[q])) {
                const double reachLong = reach + std::max(longestOn[c], longestOn[q]);
                const bool following = follows(c, q, reachLong) || follows(q, c, reachLong);
                const bool cFirst = c < q;
                for (const BodyPart& pc : parts[c])
                    for (const BodyPart& pq : parts[q]) {
                        SizedRecords s;
                        s.aTags = cFirst ? pc.tags : pq.tags;
                        s.otherTags = cFirst ? pq.tags : pc.tags;
                        s.records = found;
                        if ((pc.isLong || pq.isLong) && !following) {
                            const Bodies& bc = partBodies[pc.bodies];
                            const Bodies& bq = partBodies[pq.bodies];
                            const double sizedReach = bc.halfWidth + bq.halfWidth + margin;
                            for (const BodyMeeting& m : bodyMeetings(bc, bq, sizedReach, clearance)) {
                                std::pair<double, double> sa, sb;
                                if (!bodyStretchOn(m.a, g.paths[c].length, sa) || !bodyStretchOn(m.b, g.paths[q].length, sb))
                                    continue;
                                const std::pair<double, double> onA = cFirst ? sa : sb, onOther = cFirst ? sb : sa;
                                bool widened = false;
                                for (CrossingPoint& x : s.records) {
                                    if (!(x.aFrom <= onA.second && onA.first <= x.aTo && x.otherFrom <= onOther.second &&
                                          onOther.first <= x.otherTo))
                                        continue;
                                    if (onA.first < x.aFrom || onA.second > x.aTo || onOther.first < x.otherFrom ||
                                        onOther.second > x.otherTo)
                                        ++rep.widenedForLength;
                                    x.aFrom = std::min(x.aFrom, onA.first);
                                    x.aTo = std::max(x.aTo, onA.second);
                                    x.otherFrom = std::min(x.otherFrom, onOther.first);
                                    x.otherTo = std::max(x.otherTo, onOther.second);
                                    widened = true;
                                    break;
                                }
                                if (widened) continue;
                                CrossingPoint x;
                                x.meeting = Meeting::Cross;
                                x.otherSide = OtherSide::Path;
                                x.a = PathId(cFirst ? c : q);
                                x.other = cFirst ? q : c;
                                x.aFrom = onA.first;
                                x.aTo = onA.second;
                                x.otherFrom = onOther.first;
                                x.otherTo = onOther.second;
                                const Path& pathQ = g.paths[q];
                                if (pathQ.kind == PathKind::Connector && pathQ.junction == g.paths[c].junction)
                                    x.junction = g.paths[c].junction;
                                s.records.push_back(x);
                                ++rep.onlyForLength;
                            }
                            mergeOverlapping(s.records);
                        }
                        sized.push_back(std::move(s));
                    }
                sameRecordsJoined(sized, allTagsOn(cFirst ? c : q), allTagsOn(cFirst ? q : c));
            } else {
                SizedRecords s;
                s.records = found;
                sized.push_back(std::move(s));
            }
            for (const SizedRecords& s : sized)
                for (CrossingPoint x : s.records) {
                    x.aTags = s.aTags;
                    x.otherTags = s.otherTags;
                    ++rep.byMeeting[static_cast<std::size_t>(x.meeting)];
                    if (!x.junction.valid()) ++(g.paths[q].kind == PathKind::Connector ? rep.betweenJunctions : rep.withLanes);
                    if (x.aTags != kEveryVehicle || x.otherTags != kEveryVehicle) ++rep.forSomeVehicles;
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
