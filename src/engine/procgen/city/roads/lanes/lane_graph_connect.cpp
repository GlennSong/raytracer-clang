#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"

#include "engine/procgen/city/roads/lanes/deck_mesh.h"          // LaneLookup: the lane footprints, for the paved test
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/swept_body.h"         // a long body's sweep, for the paved test

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

namespace engine {
namespace roads::lanes {

namespace {

constexpr double kDegreesPerRadian = 57.29577951308232;
// Two directions closer than this (radians) are parallel when a connector is shaped.
constexpr double kParallelRadians = 1e-6;
// A point closer than this to where another is (or to a line) is there: shaping's rounding (metres).
constexpr double kShapeTolerance = 1e-6;
// An arc is drawn in pieces whose middle lies within this of the true circle (metres) ...
constexpr double kArcChordError = 0.01;
// ... and that each turn through at most this (degrees).
constexpr double kArcMaxPieceDegrees = 5.0;
// A path meets a vehicle's turning radius when its tightest bend falls short of it by no more than this (metres).
constexpr double kRadiusRounding = 1e-6;
// Room for one problem's words.
constexpr std::size_t kMessageBytes = 640;

#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
std::string fmt(const char* f, ...) {
    char msg[kMessageBytes];
    va_list args;
    va_start(args, f);
    std::vsnprintf(msg, sizeof msg, f, args);
    va_end(args);
    return msg;
}

Vec2 rotate(const Vec2& v, double radians) {
    const double c = std::cos(radians), s = std::sin(radians);
    return Vec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

// Appends a circular arc from `start` (already in `out`) round `centre` through `sweep` radians (positive: the way
// perp() turns), in pieces within kArcChordError and kArcMaxPieceDegrees, its last point exactly `end`.
void appendArc(std::vector<Vec2>& out, const Vec2& start, const Vec2& centre, double sweep, const Vec2& end) {
    const double radius = distance(start, centre);
    double step = kArcMaxPieceDegrees / kDegreesPerRadian;
    if (radius > kArcChordError) step = std::min(step, 2 * std::acos(1 - kArcChordError / radius));
    const int pieces = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / step)));
    for (int k = 1; k < pieces; ++k) out.push_back(centre + rotate(start - centre, sweep * k / pieces));
    out.push_back(end);
}

// Appends the circular arc that leaves `a` heading `ta` and reaches `b` (a straight piece where b lies ahead on that
// line), keeps the tightest radius, and returns the direction it arrives at b in.
Vec2 appendArcTo(std::vector<Vec2>& out, const Vec2& a, const Vec2& ta, const Vec2& b, double& tightest) {
    const Vec2 chord = b - a;
    const double len2 = dot(chord, chord), side = cross(ta, chord);
    if (len2 <= kShapeTolerance * kShapeTolerance) return ta;
    if (std::fabs(side) <= kShapeTolerance * std::sqrt(len2) && dot(ta, chord) > 0) {
        out.push_back(b);
        return ta;
    }
    // the circle through a tangent to ta and through b: curvature 2 sin(angle to the chord) / chord length
    const double curvature = 2 * side / len2;
    const Vec2 centre = a + perp(ta) * (1 / curvature);
    const double sweep = 2 * std::atan2(side, dot(ta, chord));
    appendArc(out, a, centre, sweep, b);
    const double radius = 1 / std::fabs(curvature);
    if (radius < kStraightEnoughRadius) tightest = std::min(tightest, radius);
    return rotate(ta, sweep);
}

// Two arcs from q0 heading t0 to q1 heading t1, meeting tangent where each has come the same way along its own
// tangent (the equal-reach biarc). False when no such pair goes forward.
bool appendBiarc(std::vector<Vec2>& out, const Vec2& q0, const Vec2& t0, const Vec2& q1, const Vec2& t1, double& tightest) {
    const Vec2 v = q1 - q0, t = t0 + t1;
    const double vt = dot(v, t), vv = dot(v, v), denom = 2 * (1 - dot(t0, t1));
    double reach = 0;
    if (denom <= kParallelRadians * kParallelRadians) {   // the same way: the reach that makes an S of two equal arcs
        const double ahead = dot(v, t1);
        if (!(ahead > kShapeTolerance)) return false;
        reach = vv / (4 * ahead);
    } else {
        reach = (-vt + std::sqrt(vt * vt + denom * vv)) / denom;
    }
    if (!(reach > kShapeTolerance)) return false;
    const Vec2 mid = (q0 + q1 + (t0 - t1) * reach) * 0.5;
    const Vec2 tm = appendArcTo(out, q0, t0, mid, tightest);
    appendArcTo(out, mid, normalize(tm), q1, tightest);
    return true;
}

}  // namespace

// ---- a connector's shape -------------------------------------------------------------------------------------------

ConnectorShape connectorShape(const Vec2& from, const Vec2& fromDir, const Vec2& to, const Vec2& toDir, double maxRadius) {
    ConnectorShape shape;
    const Vec2 q0 = from + fromDir * kConnectorLeadIn, q1 = to - toDir * kConnectorLeadIn;
    std::vector<Vec2> pts{from, q0};
    double tightest = kNoBend;
    const Vec2 v = q1 - q0;
    const double turn = std::atan2(cross(fromDir, toDir), dot(fromDir, toDir));
    bool shaped = false;
    if (std::fabs(turn) < kParallelRadians) {
        // straight on along one line
        if (std::fabs(cross(fromDir, v)) <= kShapeTolerance && dot(fromDir, v) >= -kShapeTolerance) {
            if (distance(q0, q1) > kShapeTolerance) pts.push_back(q1);
            shaped = true;
        }
    } else {
        double t = 0, u = 0;
        // where the line ahead of q0 crosses the line behind q1: `a` ahead of q0, `b` before q1
        if (lineCross(q0, q0 + fromDir, q1, q1 + toDir, t, u) && t > kShapeTolerance && -u > kShapeTolerance) {
            const double a = t, b = -u;
            const double halfTurnTan = std::tan(std::fabs(turn) / 2);
            const double radius = std::min(std::min(a, b) / halfTurnTan, maxRadius);
            const double reach = radius * halfTurnTan;
            shape.singleArc = true;
            const Vec2 corner = q0 + fromDir * a;
            const Vec2 arcStart = corner - fromDir * reach, arcEnd = corner + toDir * reach;
            const Vec2 centre = arcStart + perp(fromDir) * (turn > 0 ? radius : -radius);
            if (distance(q0, arcStart) > kShapeTolerance) pts.push_back(arcStart);
            appendArc(pts, arcStart, centre, turn, arcEnd);
            if (distance(arcEnd, q1) > kShapeTolerance) pts.push_back(q1);
            if (radius < kStraightEnoughRadius) tightest = radius;
            shaped = true;
        }
    }
    // two arcs where one does not fit; and where one fits only tightly (the lines crossing close by one end), two if
    // their tightest bend is the gentler (unless a radius was asked for)
    if (shaped && shape.singleArc && maxRadius == kNoBend) {
        std::vector<Vec2> two{from, q0};
        double twoTightest = kNoBend;
        if (appendBiarc(two, q0, fromDir, q1, toDir, twoTightest) && twoTightest > tightest) {
            pts = std::move(two);
            tightest = twoTightest;
            shape.singleArc = false;
        }
    }
    if (!shaped && !appendBiarc(pts, q0, fromDir, q1, toDir, tightest)) return shape;
    // (the end piece exactly q1 -> to, the same for every connector into that lane)
    if (distance(pts.back(), q1) > kShapeTolerance) pts.push_back(q1);
    else pts.back() = q1;
    pts.push_back(to);
    shape.points = std::move(pts);
    shape.tightestRadius = tightest;
    shape.ok = true;
    return shape;
}

// ---- fitting one turn to its corner -----------------------------------------------------------------------------------

ConnectorFit fitConnector(const Result& r, const LaneLookup& look, const LaneGraph& G, PathId a, PathId b, int laneA,
                          int laneB, const std::vector<VehicleSize>& vehicles) {
    ConnectorFit fit;
    const DeckHeight& H = *r.heights;
    const uint32_t tagCount = static_cast<uint32_t>(std::min<std::size_t>(G.vehicleTagNames.size(), kMaxTagNames));
    fit.whyClosed.assign(tagCount, std::string());
    // each vehicle tag's size, where the caller gave one
    std::vector<const VehicleSize*> sizeOf(tagCount, nullptr);
    for (uint32_t i = 0; i < tagCount; ++i)
        for (const VehicleSize& v : vehicles)
            if (v.tag == G.vehicleTagNames[i]) sizeOf[i] = &v;
    // Whether p is on a lane's footprint at height z (the lane footprints carry the kerb returns the builder closed).
    auto paved = [&](const Vec2& p, double z) {
        for (int li : look.grid.at(p)) {
            const Box2& bx = look.box[static_cast<std::size_t>(li)];
            if (p.x < bx.minX || p.x > bx.maxX || p.y < bx.minY || p.y > bx.maxY) continue;
            if (!look.footprint[static_cast<std::size_t>(li)].contains(p)) continue;
            if (std::fabs(H.deck(li, p) - z) <= r.graph.rules.sameLevelDz) return true;
        }
        return false;
    };
    const Path& from = G.paths[a.index];
    const Path& into = G.paths[b.index];
    const Vec2 p0 = G.pointAt(a, from.length), d0 = G.directionAt(a, from.length);
    const Vec2 p1 = G.pointAt(b, 0), d1 = G.directionAt(b, 0);
    fit.at = (p0 + p1) * 0.5;
    const TagMask allowed = from.vehicleTags & into.vehicleTags;
    if (tagCount > 0 && allowed == 0) {
        fit.why = "no vehicle may use both lanes";
        return fit;
    }
    // ONE SHAPE TRIED: closed to the vehicles it is too tight for, then to those it takes off the paved road
    // swept at their width (widest first)
    struct Trial {
        ConnectorShape shape;
        std::vector<double> along, z;
        TagMask open = 0, closed = 0;
        std::vector<std::string> whyClosed;   // per tag closed: why, in words
        bool tooTight = false;        // every vehicle closed for its radius
        double narrowestOff = -1;     // else the narrowest width swept off the paved road (-1: none)
        double offLength = 0;         // ... and that body's length
        Vec2 offAt;
    };
    // the lanes either side, for sweeping a long body that lies partly on them (swept_body.h)
    const std::vector<Vec2> beforePoints = pathPoints(G, a), afterPoints = pathPoints(G, b);
    auto attempt = [&](double maxRadius) {
        Trial tr;
        tr.whyClosed.assign(tagCount, std::string());
        tr.shape = connectorShape(p0, d0, p1, d1, maxRadius);
        if (!tr.shape.ok) return tr;
        tr.open = allowed;
        for (uint32_t i = 0; i < tagCount; ++i)
            if ((allowed & (TagMask(1) << i)) && sizeOf[i] && tr.shape.tightestRadius < sizeOf[i]->turningRadius - kRadiusRounding) {
                tr.closed |= TagMask(1) << i;
                tr.whyClosed[i] = fmt("its tightest bend, %.1f m, is tighter than the %.1f m it turns in", tr.shape.tightestRadius,
                                      sizeOf[i]->turningRadius);
            }
        tr.open &= ~tr.closed;
        if (tagCount > 0 && tr.open == 0) {
            tr.tooTight = true;
            return tr;
        }
        // heights: the arriving lane's deck at the start, the leaving lane's at the end, blended along the way
        // (the lanes' own height function, each lane's builder lane); the end heights are the lanes', exactly
        tr.along = stations(tr.shape.points);
        const double length = tr.along.back();
        tr.z.resize(tr.shape.points.size());
        for (std::size_t k = 0; k < tr.z.size(); ++k) {
            const double w = length > 0 ? tr.along[k] / length : 0.0;
            tr.z[k] = (1 - w) * H.deck(laneA, tr.shape.points[k]) + w * H.deck(laneB, tr.shape.points[k]);
        }
        tr.z.front() = G.heights[from.curve.end() - 1];
        tr.z.back() = G.heights[into.curve.first];
        // A body `width` wide and `bodyLength` long (and kSweepMargin more each side) swept along it: whether it
        // leaves the paved road, and where. A body of no length is the line's own point at every place along the
        // connector; a long one (swept_body.h) covers, at each place, its swept points' spread across the line --
        // from where its front comes onto the connector (its rear bodyLength back on the arriving lane) to where
        // its rear leaves it -- so a bus whose middle cuts inside a near turn's kerb corner is caught there.
        // `tryFirst`: a place along to test before the rest (where a bigger body left the road: a smaller one that
        // leaves it usually does so there too), or none.
        auto leavesPavement = [&](double width, double bodyLength, Vec2& at, double& leftAt, const double* tryFirst) {
            const bool swept = bodyLength > 0;
            const BodyLine line = swept ? makeBodyLine(beforePoints, tr.shape.points, afterPoints, bodyLength) : BodyLine();
            const double first = swept ? -bodyLength : 0.0, span = length + (swept ? 2 * bodyLength : 0.0);
            const int samples = std::max(1, static_cast<int>(std::ceil(span / kSweepStep)));
            std::vector<Vec2> spots;
            // the sample nearest `tryFirst`, when it is within this sweep (the same samples, so the same answer)
            const int hint = tryFirst && *tryFirst >= first && *tryFirst <= first + span
                                 ? static_cast<int>(std::lround((*tryFirst - first) / span * samples))
                                 : -1;
            for (int k = hint >= 0 ? -1 : 0; k <= samples; ++k) {
                const double s = first + span * (k < 0 ? hint : k) / samples;
                leftAt = s;
                const bool onIt = s >= 0 && s <= length;
                const Vec2 c = onIt ? pointAt(tr.shape.points, tr.along, s) : line.pointAt(s);
                const Vec2 n = perp(onIt ? tangentAtStation(tr.shape.points, tr.along, s) : line.directionAt(s));
                const double h = interp(tr.along, tr.z, s);
                // how far the body reaches across the line here, either way (the line itself always: the body
                // whose front is here; 0 and 0 for a body of no length)
                double lo = 0, hi = 0;
                if (swept) {
                    spots.clear();
                    sweptPoints(line, s, bodyLength, spots);
                    for (const Vec2& q : spots) {
                        lo = std::min(lo, dot(q - c, n));
                        hi = std::max(hi, dot(q - c, n));
                    }
                }
                for (const double side : {-1.0, 1.0}) {
                    // on the lanes either side, only where the body reaches beyond its own line: a lane's own band
                    // is the builder's, never questioned here
                    if (!onIt && (side < 0 ? lo : hi) * side < kSweptSamePoint) continue;
                    at = c + n * (side < 0 ? lo - width / 2 - kSweepMargin : hi + width / 2 + kSweepMargin);
                    if (!paved(at, h)) return true;
                }
            }
            return false;
        };
        // Each body size among the open vehicles, widest first (then longest). A body no wider and no longer than
        // one that stays on the road is taken to stay on it; one at least as wide and as long as one that leaves
        // it, to leave it.
        struct BodySize {
            double width = 0, length = 0;
        };
        std::vector<BodySize> sizes;
        for (uint32_t i = 0; i < tagCount; ++i)
            if (tr.open & (TagMask(1) << i))
                sizes.push_back(sizeOf[i] ? BodySize{sizeOf[i]->width, sizeOf[i]->length} : BodySize());
        if (tagCount == 0) sizes.push_back(BodySize());
        std::sort(sizes.begin(), sizes.end(), [](const BodySize& x, const BodySize& y) {
            return x.width != y.width ? x.width > y.width : x.length > y.length;
        });
        sizes.erase(std::unique(sizes.begin(), sizes.end(), [](const BodySize& x, const BodySize& y) {
                        return x.width == y.width && x.length == y.length;
                    }), sizes.end());
        std::vector<BodySize> stayOn;
        bool anyOff = false;
        double offAlong = 0, leftAt = 0;
        for (const BodySize& z : sizes) {
            bool within = false;
            for (const BodySize& on : stayOn) within = within || (z.width <= on.width && z.length <= on.length);
            if (within) continue;
            // (a body of no length is swept in order from the start, as before bodies had lengths: the same
            // answer and the same place reported)
            if (!leavesPavement(z.width, z.length, tr.offAt, leftAt, anyOff && z.length > 0 ? &offAlong : nullptr)) {
                stayOn.push_back(z);
                continue;
            }
            anyOff = true;
            offAlong = leftAt;
            tr.narrowestOff = z.width;
            tr.offLength = z.length;
            for (uint32_t i = 0; i < tagCount; ++i) {
                const double w = sizeOf[i] ? sizeOf[i]->width : 0.0, l = sizeOf[i] ? sizeOf[i]->length : 0.0;
                if ((tr.open & (TagMask(1) << i)) && w >= z.width && l >= z.length) {
                    tr.closed |= TagMask(1) << i;
                    tr.whyClosed[i] = fmt("swept %.1f m wide and %.1f m long it leaves the paved road at (%.1f, %.1f)", w, l,
                                          tr.offAt.x, tr.offAt.y);
                }
            }
            tr.open &= ~tr.closed;
        }
        return tr;
    };
    auto openCount = [&](const Trial& tr) {
        if (tagCount == 0) return tr.shape.ok && tr.narrowestOff < 0 ? 1 : 0;
        int count = 0;
        for (uint32_t i = 0; i < tagCount; ++i) count += (tr.open >> i) & 1u;
        return tr.shape.ok ? count : 0;
    };
    // the largest arc that fits between the lane ends; where it takes a vehicle off the paved road (it cuts the
    // kerb's corner), smaller arcs, down to the tightest vehicle's turning radius: the largest that keeps the most
    // vehicles is the connector
    Trial best = attempt(kNoBend);
    const double largest = best.shape.tightestRadius;
    double smallest = largest;
    if (best.shape.ok && best.shape.singleArc && best.narrowestOff >= 0 && std::isfinite(largest)) {
        for (uint32_t i = 0; i < tagCount; ++i)
            if ((allowed & (TagMask(1) << i)) && sizeOf[i]) smallest = std::min(smallest, sizeOf[i]->turningRadius);
        for (int k = 1; k <= kSmallerArcTries && smallest < largest; ++k) {
            const Trial tr = attempt(largest - (largest - smallest) * k / kSmallerArcTries);
            if (openCount(tr) > openCount(best)) best = tr;
            if (best.narrowestOff < 0) break;
        }
    }
    fit.shape = best.shape;
    fit.z = best.z;
    fit.open = best.open;
    fit.closed = best.closed;
    fit.whyClosed = best.whyClosed;
    if (!best.shape.ok) {
        fit.noArc = true;
        fit.why = "no arcs join its lane ends going forward";
        return fit;
    }
    if (openCount(best) == 0) {
        double tightestTurn = 0;
        for (uint32_t i = 0; i < tagCount; ++i)
            if ((allowed & (TagMask(1) << i)) && sizeOf[i])
                tightestTurn = tightestTurn == 0 ? sizeOf[i]->turningRadius : std::min(tightestTurn, sizeOf[i]->turningRadius);
        if (best.tooTight) {
            fit.why = fmt("its tightest bend, %.1f m, is too tight for every vehicle allowed on both lanes (the "
                          "tightest-turning needs %.1f m)", best.shape.tightestRadius, tightestTurn);
        } else {
            fit.at = best.offAt;
            fit.why = fmt("swept %.1f m wide%s (the smallest vehicle allowed on both lanes, and %.1f m more each "
                          "side) it leaves the paved road%s", best.narrowestOff,
                          best.offLength > 0 ? fmt(" and %.1f m long", best.offLength).c_str() : "", kSweepMargin,
                          smallest < largest ? fmt(", on every arc from %.1f m down to %.1f m", largest, smallest).c_str() : "");
        }
    }
    return fit;
}

// ---- how much of the graph a car can get round ---------------------------------------------------------------------

void reachReport(LaneGraphBuild& out) {
    const LaneGraph& G = out.graph;
    const uint32_t n = static_cast<uint32_t>(G.paths.size());
    out.reach = reachableParts(G);
    out.outside.clear();
    uint32_t lanes = 0, inLargest = 0;
    for (uint32_t p = 0; p < n; ++p) {
        if (G.paths[p].kind != PathKind::Lane) continue;
        ++lanes;
        inLargest += out.reach.partOf[p] == out.reach.largest ? 1 : 0;
    }
    out.reachableShare = lanes ? static_cast<double>(inLargest) / lanes : 0.0;
    if (n == 0) return;
    // every step, and every step backwards
    std::vector<std::vector<uint32_t>> ahead(n), behind(n);
    for (uint32_t p = 0; p < n; ++p)
        for (const PathId& q : stepsFrom(G, PathId(p))) {
            ahead[p].push_back(q.index);
            behind[q.index].push_back(p);
        }
    // which paths can get back to the largest part (spreading backwards from it)
    auto spread = [&](const std::vector<std::vector<uint32_t>>& steps) {
        std::vector<char> seen(n, 0);
        std::vector<uint32_t> todo;
        for (uint32_t p = 0; p < n; ++p)
            if (out.reach.partOf[p] == out.reach.largest) {
                seen[p] = 1;
                todo.push_back(p);
            }
        while (!todo.empty()) {
            const uint32_t p = todo.back();
            todo.pop_back();
            for (uint32_t q : steps[p])
                if (!seen[q]) {
                    seen[q] = 1;
                    todo.push_back(q);
                }
        }
        return seen;
    };
    const std::vector<char> getsBack = spread(behind);
    // the lanes at a stated turn with no connector (a corner fault for every class), both ways
    std::vector<char> intoRefused(n, 0), outOfRefused(n, 0);
    for (const Refusal& ref : out.refusals) {
        intoRefused[ref.from.index] = 1;
        outOfRefused[ref.to.index] = 1;
    }
    // A lane's reason: what the paths it leads to (or that lead to it) come to, the first that applies.
    const char* const kWhy[] = {
        "cannot get back: it leads into a junction with turns that fit no vehicle (corner faults)",
        "cannot get back: it leads into a road that just ends",
        "cannot get back: it goes round a loop with no way out",
        "cannot be reached: the turns into it fit no vehicle (corner faults)",
        "cannot be reached: its road starts with no way in",
        "cannot be reached: only from a loop with no way in",
    };
    constexpr std::size_t kWhyCount = sizeof(kWhy) / sizeof(kWhy[0]);
    std::vector<std::vector<uint32_t>> byWhy(kWhyCount);
    for (uint32_t p = 0; p < n; ++p) {
        if (G.paths[p].kind != PathKind::Lane || out.reach.partOf[p] == out.reach.largest) continue;
        const bool forward = !getsBack[p];
        const std::vector<std::vector<uint32_t>>& steps = forward ? ahead : behind;
        // the paths it comes to, outside the largest part
        std::vector<char> seen(n, 0);
        std::vector<uint32_t> todo{p};
        seen[p] = 1;
        bool refused = false, open = false;
        while (!todo.empty()) {
            const uint32_t q = todo.back();
            todo.pop_back();
            refused = refused || (forward ? intoRefused[q] : outOfRefused[q]);
            open = open || steps[q].empty();
            for (uint32_t x : steps[q])
                if (!seen[x] && out.reach.partOf[x] != out.reach.largest) {
                    seen[x] = 1;
                    todo.push_back(x);
                }
        }
        const std::size_t base = forward ? 0 : 3;
        byWhy[base + (refused ? 0 : open ? 1 : 2)].push_back(p);
    }
    for (std::size_t w = 0; w < kWhyCount; ++w)
        if (!byWhy[w].empty()) out.outside.emplace_back(kWhy[w], byWhy[w]);
}

}  // namespace roads::lanes
}  // namespace engine
