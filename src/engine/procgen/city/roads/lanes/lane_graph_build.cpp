#include "engine/procgen/city/roads/lanes/lane_graph_build.h"

#include "engine/procgen/city/polyline.h"   // segmentsCross

#include <algorithm>
#include <cmath>
#include <vector>

// What is left of the lane graph worked out from the built roads (ADR-0426): the measurements the design keeps -- how a
// path bends, as a vehicle feels it, and the kinks a driver would feel -- and the tidying of a lane folded back on itself.
// The graph itself is a table built from the road model (lane_level.h).

namespace engine {
namespace roads::lanes {

namespace {

// How many points either side of a fold are searched for where the lane crosses itself.
constexpr std::size_t kFoldSearch = 16;
// Centre-line points closer together than this are passed over when a bend is measured: too short to measure (metres).
constexpr double kMinBendChord = 0.05;
constexpr double kDegreesPerRadian = 57.29577951308232;

}  // namespace

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

namespace {

// ---- bends, measured as a vehicle feels them (issue #281, lane_graph_build.h measureBends) -------------------------

// A CENTRE LINE AS A BEND IS READ FROM IT: the points closer than kMinBendChord to the one kept before passed over (the
// last point kept in place of a near one), each kept point with its distance along the line (from the first point's).
struct ThinLine {
    std::vector<Vec2> xy;
    std::vector<double> s;
};
ThinLine thinLine(const Vec2* xy, const double* along, std::size_t n) {
    ThinLine t;
    for (std::size_t i = 0; i < n; ++i) {
        if (!t.xy.empty() && distance(t.xy.back(), xy[i]) < kMinBendChord) continue;
        t.xy.push_back(xy[i]);
        t.s.push_back(along[i] - along[0]);
    }
    if (t.xy.size() >= 2 && distance(t.xy.back(), xy[n - 1]) > 0) {
        t.xy.back() = xy[n - 1];
        t.s.back() = along[n - 1] - along[0];
    }
    return t;
}
// How far the heading turns at kept point i (1 .. size - 2): radians, plus one way round, minus the other.
double turnAt(const ThinLine& t, std::size_t i) {
    const Vec2 in = t.xy[i] - t.xy[i - 1], out = t.xy[i + 1] - t.xy[i];
    return std::atan2(cross(in, out), dot(in, out));
}
// A point's share of the line, before it and after it: halfway back to the point before and halfway on to the point
// after, each no more than kBendSpread (metres).
double shareBefore(const ThinLine& t, std::size_t i) { return std::min(0.5 * (t.s[i] - t.s[i - 1]), kBendSpread); }
double shareAfter(const ThinLine& t, std::size_t i) { return std::min(0.5 * (t.s[i + 1] - t.s[i]), kBendSpread); }

// A point that turns less than this (radians) is rounding in a straight line's points: it turns nowhere.
constexpr double kNoTurn = 1e-12;

// THE LINE'S TURNING SUMMED FROM ITS START, each point's turn spread evenly over its share of the line: knots where the
// sum is read, rising evenly across each point's share and level between them. Level before the first knot and after
// the last. Each point's share (from, to) and turn are kept too.
struct Turning {
    std::vector<double> x, total;
    std::vector<double> from, to, turn;   // per point that can turn: its share of the line, and its turn
    double at(double s) const {
        if (x.empty() || s <= x.front()) return 0;
        if (s >= x.back()) return total.back();
        const std::size_t k = static_cast<std::size_t>(std::upper_bound(x.begin(), x.end(), s) - x.begin());
        const double span = x[k] - x[k - 1];
        return span > 0 ? total[k - 1] + (total[k] - total[k - 1]) * (s - x[k - 1]) / span : total[k];
    }
    // The part of [a, b] over which the line turns: from where the first point turning there begins its share to where
    // the last one's ends, clipped to [a, b]. False when it turns nowhere there.
    bool turnsWithin(double a, double b, double& lo, double& hi) const {
        bool any = false;
        std::size_t k = static_cast<std::size_t>(std::upper_bound(to.begin(), to.end(), a) - to.begin());
        for (; k < turn.size() && from[k] < b; ++k) {
            if (!(std::fabs(turn[k]) > kNoTurn)) continue;
            if (!any) lo = std::max(a, from[k]);
            any = true;
            hi = std::min(b, to[k]);
        }
        return any;
    }
};
Turning turningOf(const ThinLine& t) {
    Turning T;
    double sum = 0;
    for (std::size_t i = 1; i + 1 < t.xy.size(); ++i) {
        const double turn = turnAt(t, i);
        T.from.push_back(t.s[i] - shareBefore(t, i));
        T.to.push_back(t.s[i] + shareAfter(t, i));
        T.turn.push_back(turn);
        T.x.push_back(T.from.back());
        T.total.push_back(sum);
        sum += turn;
        T.x.push_back(T.to.back());
        T.total.push_back(sum);
    }
    return T;
}

// A cut this close to a cell's edge is on the edge (metres): a stretch that only touches a cell does not cover it.
constexpr double kCellEdgeSlack = 1e-9;

}  // namespace

std::vector<BendPiece> measureBends(const Vec2* xy, const double* along, std::size_t count) {
    std::vector<BendPiece> out;
    const double length = count >= 2 ? along[count - 1] - along[0] : 0.0;
    const ThinLine line = count >= 2 ? thinLine(xy, along, count) : ThinLine();
    if (!(length > 0) || line.xy.size() < 3) return {BendPiece()};
    const Turning turning = turningOf(line);

    // The line cut into cells kBendSampleStep long (at most), read at every cell edge; each reading laid over the cells
    // its stretch covers, the tightest kept.
    const std::size_t cells = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(length / kBendSampleStep)));
    const double step = length / static_cast<double>(cells);
    std::vector<double> cellRadius(cells, kNoBend);
    for (std::size_t k = 0; k <= cells; ++k) {
        const double s = step * static_cast<double>(k);
        double radius = kNoBend, from = 0, to = 0;
        for (const double wanted : kBendMeasureLengths) {
            const double span = std::min(wanted, length);
            const double mid = std::clamp(s, 0.5 * span, length - 0.5 * span);
            from = mid - 0.5 * span;
            to = mid + 0.5 * span;
            const double turn = std::fabs(turning.at(to) - turning.at(from));
            radius = turn > 0 ? span / turn : kNoBend;
            // long enough when a curve of this radius strays no more than kBendRoom from its chord across it (a radius of
            // kNoBend never is), or when the stretch is the whole line
            if (span >= 2.0 * std::sqrt(2.0 * kBendRoom * radius) || span >= length) break;
        }
        if (!(radius < kStraightEnoughRadius)) continue;
        // laid over the part of the stretch where the line turns (a stretch that is straight but for its far end does
        // not make its straight part a bend)
        double lo = from, hi = to;
        if (!turning.turnsWithin(from, to, lo, hi)) continue;
        const std::size_t first = std::min(cells - 1, static_cast<std::size_t>(std::floor(lo / step + kCellEdgeSlack)));
        const std::size_t last =
            std::min(cells, std::max(first + 1, static_cast<std::size_t>(std::ceil(hi / step - kCellEdgeSlack))));
        for (std::size_t c = first; c < last; ++c) cellRadius[c] = std::min(cellRadius[c], radius);
    }

    // Neighbouring cells of near enough the same radius (or both straight) make one piece, at the tightest of them.
    double widest = kNoBend;
    for (std::size_t c = 0; c < cells; ++c) {
        const double r = cellRadius[c];
        if (!out.empty()) {
            BendPiece& p = out.back();
            const bool bothStraight = r == kNoBend && p.radius == kNoBend;
            const bool alike = r != kNoBend && p.radius != kNoBend &&
                               std::max(widest, r) <= std::min(p.radius, r) * (1.0 + kBendPieceShare);
            if (bothStraight || alike) {
                p.radius = std::min(p.radius, r);
                widest = std::max(widest, r);
                continue;
            }
        }
        BendPiece piece;
        piece.from = step * static_cast<double>(c);
        piece.radius = r;
        out.push_back(piece);
        widest = r;
    }
    return out;
}

std::vector<Kink> findKinks(const Vec2* xy, const double* along, std::size_t count) {
    std::vector<Kink> out;
    if (count < 3) return out;
    const ThinLine line = thinLine(xy, along, count);
    const std::size_t n = line.xy.size();
    if (n < 3) return out;
    std::vector<double> turn(n, 0.0), before(n, 0.0), after(n, 0.0);
    for (std::size_t i = 1; i + 1 < n; ++i) {
        turn[i] = turnAt(line, i);
        before[i] = shareBefore(line, i);
        after[i] = shareAfter(line, i);
    }
    // Two looks. The first judges every point against its neighbours; the second judges the points the first found
    // again, leaving out the neighbours it found, so a point beside a kink is not blamed for the kink's turn.
    std::vector<uint8_t> kinked(n, 0);
    std::vector<double> jumps(n, 0.0);
    for (int look = 0; look < 2; ++look) {
        // How sharply the line bends at point j (its turn over its share), when j turns (not an end) and is no kink.
        auto bendAt = [&](std::size_t j, double& curvature) {
            if (j < 1 || j + 1 >= n || kinked[j] || !(before[j] + after[j] > 0)) return false;
            curvature = turn[j] / (before[j] + after[j]);
            return true;
        };
        std::vector<uint8_t> found(n, 0);
        for (std::size_t i = 1; i + 1 < n; ++i) {
            if (look == 1 && !kinked[i]) continue;
            // What a smooth line turns at point i: the bend just before it over its share before, and the bend just
            // after it over its share after, each read at the neighbouring point. Round a circle that is exactly the
            // point's turn. A side with no such point (the line's end, or a kink) could run straight or carry on the
            // other side's bend: the point is given the benefit of the doubt, the smaller jump of the two.
            double bendBefore = 0, bendAfter = 0;
            const bool hasBefore = bendAt(i - 1, bendBefore), hasAfter = bendAt(i + 1, bendAfter);
            double jump = std::fabs(turn[i] - bendBefore * before[i] - bendAfter * after[i]);
            if (!hasBefore) jump = std::min(jump, std::fabs(turn[i] - bendAfter * (before[i] + after[i])));
            if (!hasAfter) jump = std::min(jump, std::fabs(turn[i] - bendBefore * (before[i] + after[i])));
            jumps[i] = jump * kDegreesPerRadian;
            found[i] = jumps[i] > kKinkDegrees;
        }
        kinked = found;
    }
    for (std::size_t i = 1; i + 1 < n; ++i) {
        if (!kinked[i]) continue;
        Kink k;
        k.along = line.s[i];
        k.degrees = jumps[i];
        k.at = line.xy[i];
        out.push_back(k);
    }
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
