#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"

#include "engine/procgen/city/roads/lanes/deck_mesh.h"          // LaneLookup: the lane footprints, for the paved test
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"

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

// Which turn kinds a lane makes, for counting and for the report.
const char* turnName(Turn t) {
    switch (t) {
    case Turn::Straight: return "straight on";
    case Turn::NearTurn: return "near turn";
    case Turn::FarTurn: return "far turn";
    case Turn::UTurn: return "U-turn";
    case Turn::None: break;
    }
    return "turn";
}

}  // namespace

// ---- which turns a junction offers ---------------------------------------------------------------------------------

std::vector<JunctionTurn> junctionTurns(const std::vector<TurnLeg>& arriving, const std::vector<TurnLeg>& leaving,
                                        const TurnRules& rules) {
    std::vector<JunctionTurn> out;
    for (uint32_t ai = 0; ai < arriving.size(); ++ai) {
        const TurnLeg& A = arriving[ai];
        const std::size_t n = A.lanes.size();
        if (n == 0 || A.ends.size() != n) continue;
        const Vec2 kerb = perp(A.heading) * rules.kerbPerpSign;
        struct Exit { uint32_t leg; Turn turn; double degrees; };
        std::vector<Exit> exits;
        int straightest = -1;
        for (uint32_t bi = 0; bi < leaving.size(); ++bi) {
            const TurnLeg& B = leaving[bi];
            if (B.lanes.empty() || B.ends.size() != B.lanes.size()) continue;
            if (B.road == A.road && B.dir == -A.dir) continue;   // back onto the road it came from
            const double degrees = std::fabs(std::atan2(cross(A.heading, B.heading), dot(A.heading, B.heading))) * kDegreesPerRadian;
            exits.push_back(Exit{bi, dot(B.heading, kerb) > 0 ? Turn::NearTurn : Turn::FarTurn, degrees});
            if (degrees < rules.straightOnDegrees &&
                (straightest < 0 || degrees < exits[static_cast<std::size_t>(straightest)].degrees))
                straightest = static_cast<int>(exits.size()) - 1;
        }
        if (straightest >= 0) exits[static_cast<std::size_t>(straightest)].turn = Turn::Straight;
        bool hasNear = false, hasFar = false;
        for (const Exit& x : exits) {
            hasNear = hasNear || x.turn == Turn::NearTurn;
            hasFar = hasFar || x.turn == Turn::FarTurn;
        }
        // how many lanes make each kind of turn, counted from that turn's side
        std::size_t nearLanes = 1, farLanes = 1;
        if (n > 1 && straightest < 0) {
            if (hasNear && hasFar) {
                farLanes = n / 2;
                nearLanes = n - farLanes;
            } else {
                nearLanes = farLanes = n;
            }
        }
        for (const Exit& x : exits) {
            const TurnLeg& B = leaving[x.leg];
            const std::size_t m = B.lanes.size();
            auto add = [&](std::size_t i, std::size_t j) {
                out.push_back(JunctionTurn{A.lanes[i], B.lanes[j], x.turn, ai, x.leg});
            };
            if (x.turn == Turn::Straight) {
                // into the leaving lane nearest it sideways, measured across the way halfway between the two legs' (so
                // a kink between them, the arriving ends as far before it as the leaving starts are after, moves neither
                // side); the more offside of two as near
                const Vec2 between = A.heading + B.heading;
                const Vec2 across = perp(between.length() > 0 ? normalize(between) : A.heading);
                for (std::size_t i = 0; i < n; ++i) {
                    const double at = dot(A.ends[i], across);
                    std::size_t best = 0;
                    for (std::size_t j = 1; j < m; ++j)
                        if (std::fabs(dot(B.ends[j], across) - at) < std::fabs(dot(B.ends[best], across) - at)) best = j;
                    add(i, best);
                }
            } else if (x.turn == Turn::NearTurn) {
                for (std::size_t k = 0; k < nearLanes && k < n; ++k) add(n - 1 - k, m - 1 - std::min(k, m - 1));
            } else {
                for (std::size_t k = 0; k < farLanes && k < n; ++k) add(k, std::min(k, m - 1));
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const JunctionTurn& a, const JunctionTurn& b) {
        if (a.fromLeg != b.fromLeg) return a.fromLeg < b.fromLeg;
        if (a.toLeg != b.toLeg) return a.toLeg < b.toLeg;
        if (a.from != b.from) return a.from < b.from;
        return a.to < b.to;
    });
    return out;
}

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

// ---- making them ---------------------------------------------------------------------------------------------------

void connectJunctions(const Result& r, const LaneLookup& look, const std::vector<VehicleSize>& vehicles,
                      std::vector<std::vector<PathId>>& next, LaneGraphBuild& out) {
    LaneGraph& G = out.graph;
    const DeckHeight& H = *r.heights;
    const uint32_t tagCount = static_cast<uint32_t>(std::min<std::size_t>(G.vehicleTagNames.size(), kMaxTagNames));
    // each vehicle tag's size, where the caller gave one
    std::vector<const VehicleSize*> sizeOf(tagCount, nullptr);
    for (uint32_t i = 0; i < tagCount; ++i)
        for (const VehicleSize& v : vehicles)
            if (v.tag == G.vehicleTagNames[i]) sizeOf[i] = &v;
    out.closedTo.assign(G.vehicleTagNames.size(), 0);
    TurnRules rules;
    rules.kerbPerpSign = kerbPerpSign();
    // Whether p is on a lane's footprint at height z (the lane footprints carry the kerb returns the builder closed).
    auto paved = [&](const Vec2& p, double z) {
        for (int li : look.grid.at(p)) {
            const Box2& b = look.box[static_cast<std::size_t>(li)];
            if (p.x < b.minX || p.x > b.maxX || p.y < b.minY || p.y > b.maxY) continue;
            if (!look.footprint[static_cast<std::size_t>(li)].contains(p)) continue;
            if (std::fabs(H.deck(li, p) - z) <= r.graph.rules.sameLevelDz) return true;
        }
        return false;
    };
    // The legs of a junction: its arriving (or leaving) lanes, a leg per section, in the order their first lanes come.
    auto legsOf = [&](const Range& lanes, bool arriving) {
        std::vector<TurnLeg> legs;
        std::vector<uint32_t> sectionOfLeg;
        for (uint32_t k = lanes.first; k < lanes.end(); ++k) {
            const PathId p = G.junctionLanes[k];
            const uint32_t s = G.paths[p.index].section.index;
            const std::size_t at = static_cast<std::size_t>(std::find(sectionOfLeg.begin(), sectionOfLeg.end(), s) - sectionOfLeg.begin());
            if (at == sectionOfLeg.size()) {
                sectionOfLeg.push_back(s);
                TurnLeg leg;
                leg.road = out.sectionSources[s].edge;
                leg.dir = out.sectionSources[s].dir;
                legs.push_back(leg);
            }
        }
        for (std::size_t i = 0; i < legs.size(); ++i) {
            const Section& sec = G.sections[sectionOfLeg[i]];
            Vec2 heading;
            for (uint32_t k = sec.lanes.first; k < sec.lanes.end(); ++k) {   // offside to kerb
                const PathId p = G.sectionLanes[k];
                const double at = arriving ? G.paths[p.index].length : 0.0;
                legs[i].lanes.push_back(p.index);
                legs[i].ends.push_back(G.pointAt(p, at));
                heading += G.directionAt(p, at);
            }
            legs[i].heading = heading.length() > 0 ? normalize(heading) : Vec2(1, 0);
        }
        return legs;
    };
    auto addRefusal = [&](uint32_t j, const JunctionTurn* t, const Vec2& at, const std::string& why) {
        const JunctionNote& note = out.junctionNotes[j];
        Refusal ref;
        ref.junction = j;
        ref.at = at;
        if (t) {
            ref.from = PathId(t->from);
            ref.to = PathId(t->to);
            ref.turn = t->turn;
            out.problems.push_back(fmt("junction %u at (%.1f, %.1f) (roads %s): the %s from lane %u into lane %u is refused, "
                                       "at (%.1f, %.1f): %s", j, note.at.x, note.at.y, note.roads.c_str(), turnName(t->turn),
                                       t->from, t->to, at.x, at.y, why.c_str()));
        }
        out.refusals.push_back(ref);
    };

    for (uint32_t j = 0; j < G.junctions.size(); ++j) {
        JunctionNote& note = out.junctionNotes[j];
        G.junctions[j].connectors = Range{static_cast<uint32_t>(G.junctionConnectors.size()), 0};
        const std::vector<TurnLeg> arrivingLegs = legsOf(G.junctions[j].arriving, true);
        const std::vector<TurnLeg> leavingLegs = legsOf(G.junctions[j].leaving, false);
        // (a carry-on's connectors all go straight on: its roads are one road, kinked)
        rules.straightOnDegrees = note.origin == JunctionOrigin::CarryOn ? kNearMissDegrees : kStraightOnDegrees;
        const std::vector<JunctionTurn> turns = junctionTurns(arrivingLegs, leavingLegs, rules);
        note.turnsStated = static_cast<uint32_t>(turns.size());
        if (note.refused) {   // its line is in the problems already: one for the junction, for all its turns
            note.turnsRefused = note.turnsStated;
            if (!turns.empty()) addRefusal(j, nullptr, note.at, "");
            continue;
        }
        for (const JunctionTurn& t : turns) {
            const PathId a(t.from), b(t.to);
            const Path& from = G.paths[a.index];
            const Path& into = G.paths[b.index];
            const Vec2 p0 = G.pointAt(a, from.length), d0 = G.directionAt(a, from.length);
            const Vec2 p1 = G.pointAt(b, 0), d1 = G.directionAt(b, 0);
            const TagMask allowed = from.vehicleTags & into.vehicleTags;
            if (tagCount > 0 && allowed == 0) {
                addRefusal(j, &t, (p0 + p1) * 0.5, "no vehicle may use both lanes");
                ++note.turnsRefused;
                continue;
            }
            const int laneA = out.sources[a.index].lane, laneB = out.sources[b.index].lane;
            // ONE SHAPE TRIED: closed to the vehicles it is too tight for, then to those it takes off the paved road
            // swept at their width (widest first)
            struct Trial {
                ConnectorShape shape;
                std::vector<double> along, z;
                TagMask open = 0, closed = 0;
                bool tooTight = false;        // every vehicle closed for its radius
                double narrowestOff = -1;     // else the narrowest width swept off the paved road (-1: none)
                Vec2 offAt;
            };
            auto attempt = [&](double maxRadius) {
                Trial tr;
                tr.shape = connectorShape(p0, d0, p1, d1, maxRadius);
                if (!tr.shape.ok) return tr;
                tr.open = allowed;
                for (uint32_t i = 0; i < tagCount; ++i)
                    if ((allowed & (TagMask(1) << i)) && sizeOf[i] && tr.shape.tightestRadius < sizeOf[i]->turningRadius - kRadiusRounding)
                        tr.closed |= TagMask(1) << i;
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
                // Swept `width` wide (and kSweepMargin more each side), whether it leaves the paved road (and where).
                auto leavesPavement = [&](double width, Vec2& at) {
                    const int samples = std::max(1, static_cast<int>(std::ceil(length / kSweepStep)));
                    for (int k = 0; k <= samples; ++k) {
                        const double s = length * k / samples;
                        const Vec2 c = pointAt(tr.shape.points, tr.along, s), n = perp(tangentAtStation(tr.shape.points, tr.along, s));
                        const double h = interp(tr.along, tr.z, s);
                        for (const double side : {-1.0, 1.0}) {
                            at = c + n * (side * (width / 2 + kSweepMargin));
                            if (!paved(at, h)) return true;
                        }
                    }
                    return false;
                };
                std::vector<double> widths;
                for (uint32_t i = 0; i < tagCount; ++i)
                    if (tr.open & (TagMask(1) << i)) widths.push_back(sizeOf[i] ? sizeOf[i]->width : 0.0);
                if (tagCount == 0) widths.push_back(0.0);
                std::sort(widths.begin(), widths.end(), [](double x, double y) { return x > y; });
                widths.erase(std::unique(widths.begin(), widths.end()), widths.end());
                for (double w : widths) {
                    if (!leavesPavement(w, tr.offAt)) break;
                    tr.narrowestOff = w;
                    for (uint32_t i = 0; i < tagCount; ++i)
                        if ((tr.open & (TagMask(1) << i)) && (sizeOf[i] ? sizeOf[i]->width : 0.0) >= w) tr.closed |= TagMask(1) << i;
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
            if (!best.shape.ok) {
                addRefusal(j, &t, (p0 + p1) * 0.5, "no arcs join its lane ends going forward");
                ++note.turnsRefused;
                continue;
            }
            if (openCount(best) == 0) {
                double tightestTurn = 0;
                for (uint32_t i = 0; i < tagCount; ++i)
                    if ((allowed & (TagMask(1) << i)) && sizeOf[i])
                        tightestTurn = tightestTurn == 0 ? sizeOf[i]->turningRadius : std::min(tightestTurn, sizeOf[i]->turningRadius);
                if (best.tooTight)
                    addRefusal(j, &t, (p0 + p1) * 0.5,
                               fmt("its tightest bend, %.1f m, is too tight for every vehicle allowed on both lanes (the "
                                   "tightest-turning needs %.1f m)", best.shape.tightestRadius, tightestTurn));
                else
                    addRefusal(j, &t, best.offAt,
                               fmt("swept %.1f m wide (the narrowest vehicle allowed on both lanes, and %.1f m more each side) "
                                   "it leaves the paved road%s", best.narrowestOff, kSweepMargin,
                                   smallest < largest ? fmt(", on every arc from %.1f m down to %.1f m", largest, smallest).c_str() : ""));
                ++note.turnsRefused;
                continue;
            }
            const TagMask tags = best.open, closed = best.closed;
            Path c;
            c.kind = PathKind::Connector;
            c.turn = t.turn;
            c.laneTags = from.laneTags | into.laneTags;
            c.vehicleTags = tags;
            c.curve = G.addCurve(best.shape.points, best.z);
            c.length = G.along[c.curve.end() - 1];
            c.width = std::min(from.width, into.width);
            c.tightestRadius = best.shape.tightestRadius;
            c.junction = JunctionId(j);
            c.from = a;
            const PathId id(static_cast<uint32_t>(G.paths.size()));
            G.paths.push_back(c);
            out.sources.push_back(PathSource());
            next[a.index].push_back(id);
            next.push_back({b});
            G.junctionConnectors.push_back(id);
            ++out.connectorsByTurn[static_cast<std::size_t>(t.turn)];
            ++note.turnsBuilt;
            for (uint32_t i = 0; i < tagCount; ++i) out.closedTo[i] += (closed >> i) & 1u;
        }
        G.junctions[j].connectors.count = static_cast<uint32_t>(G.junctionConnectors.size()) - G.junctions[j].connectors.first;
    }
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
    // the lanes at a turn not built, and the lane ends not joined (both ways)
    std::vector<char> intoRefused(n, 0), outOfRefused(n, 0), unjoinedEnd(n, 0), unjoinedStart(n, 0);
    for (const Refusal& ref : out.refusals) {
        if (ref.from.valid()) {
            intoRefused[ref.from.index] = 1;
            outOfRefused[ref.to.index] = 1;
            continue;
        }
        const Junction& jn = G.junctions[ref.junction];
        for (uint32_t k = jn.arriving.first; k < jn.arriving.end(); ++k) intoRefused[G.junctionLanes[k].index] = 1;
        for (uint32_t k = jn.leaving.first; k < jn.leaving.end(); ++k) outOfRefused[G.junctionLanes[k].index] = 1;
    }
    for (const std::pair<uint32_t, uint32_t>& u : out.unjoined) {
        unjoinedEnd[u.first] = 1;
        unjoinedStart[u.second] = 1;
    }
    // A lane's reason: what the paths it leads to (or that lead to it) come to, the first that applies.
    const char* const kWhy[] = {
        "cannot get back: it leads into a junction with turns refused",
        "cannot get back: it leads into a lane end not joined",
        "cannot get back: it leads into a road that just ends",
        "cannot get back: it goes round a loop with no way out",
        "cannot be reached: the turns into it were refused",
        "cannot be reached: it starts at a lane end not joined",
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
        bool refused = false, unjoined = false, open = false;
        while (!todo.empty()) {
            const uint32_t q = todo.back();
            todo.pop_back();
            refused = refused || (forward ? intoRefused[q] : outOfRefused[q]);
            unjoined = unjoined || (forward ? unjoinedEnd[q] : unjoinedStart[q]);
            open = open || steps[q].empty();
            for (uint32_t x : steps[q])
                if (!seen[x] && out.reach.partOf[x] != out.reach.largest) {
                    seen[x] = 1;
                    todo.push_back(x);
                }
        }
        const std::size_t base = forward ? 0 : 4;
        byWhy[base + (refused ? 0 : unjoined ? 1 : open ? 2 : 3)].push_back(p);
    }
    for (std::size_t w = 0; w < kWhyCount; ++w)
        if (!byWhy[w].empty()) out.outside.emplace_back(kWhy[w], byWhy[w]);
}

}  // namespace roads::lanes
}  // namespace engine
