#include "engine/procgen/city/roads/lanes/lane_level.h"

#include "engine/procgen/city/polygon.h"                            // convexHull
#include "engine/procgen/city/polyline.h"                           // segmentsCross: an outline crossing itself
#include "engine/procgen/city/roads/control_signs.h"                // stopLineFromMouth: ADR-0419's stop line
#include "engine/procgen/city/roads/lane_turns.h"                   // turnKey: a turn's word
#include "engine/procgen/city/roads/lanes/deck_mesh.h"              // LaneLookup: the paved road, for the corner fit
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"     // fitConnector, reachReport
#include "engine/procgen/city/roads/lanes/lane_graph_crossings.h"   // where the paths' bodies could touch
#include "engine/procgen/city/roads/lanes/polyline_ops.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <tuple>
#include <utility>

namespace engine {
namespace roads::lanes {

namespace {

// Two stations closer than this are one (metres): the road file writes stations to the centimetre, the builder to far less.
constexpr double kCutTolerance = 1e-3;
// A cut this close to a centre-line point drops that point rather than leave a sliver segment (metres).
constexpr double kSliceGap = 1e-3;
// Two lanes the model states lead one into the other whose lines miss by more than this are joined and reported (metres).
constexpr double kJoinReported = 1e-3;
constexpr double kDegreesPerRadian = 57.29577951308232;
constexpr std::size_t kMessageBytes = 768;

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

// A builder lane read the way its traffic runs: its points, the distance along it to each, and each point's distance
// along its road's spine the way the traffic runs (u: its spine station, or the spine's length less it running against
// the spine) -- the builder laid each point beside a stated spine station (RoadModel::spineStations).
struct TravelLane {
    std::vector<Vec2> xy;
    std::vector<double> s, u;
    bool ok = false;
};
// The distance along the lane at corridor distance u (exactly its own end where u is that end).
double tauAt(const TravelLane& T, double u) {
    if (std::fabs(u - T.u.front()) <= kCutTolerance) return 0;
    if (std::fabs(u - T.u.back()) <= kCutTolerance) return T.s.back();
    return interp(T.u, T.s, u);
}

// A LANE'S STRETCH ON ONE STREET, as the lane level drives it: [u0, u1] along its corridor, from the leaving arm's stop
// line (or the street's end, or the lane's start) to the arriving arm's stop line (or the street's end, or the lane's end).
struct Piece {
    uint32_t lane = 0;
    uint32_t street = kNoIndex;
    double u0 = 0, u1 = 0;
    int startJunction = -1, startArm = -1, endJunction = -1, endArm = -1;   // the stop lines it starts and ends at
    int startStreetEnd = -1, endStreetEnd = -1;   // the end of its street (0: from, 1: to) it starts or ends at, no junction there
    std::vector<uint32_t> paths;                  // its lane paths, in order
};

}  // namespace

LaneGraphBuild buildLaneLevel(const Result& r, const RoadModel& m, const LaneGraphOptions& opts) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    LaneGraphBuild out;
    LaneGraph& G = out.graph;
    const RoadLabGraph& g = r.graph;
    const LaneSet& L = r.lanes;
    const DeckHeight& H = *r.heights;
    const std::size_t nLanes = L.lanes.size();

    // ---- the facts the lane level cannot be built without ------------------------------------------------------------
    {
        std::vector<std::string> missing;
        if (!m.built || m.lanes.size() != nLanes) missing.push_back("the road model's built layer for this build (assembleRoadModel)");
        if (!g.listsJunctions || !g.listsStreets)
            missing.push_back("its junctions and streets (run `lanes_tool layout <road file> --write`)");
        if (!g.declaresJunctions)
            missing.push_back("every junction's control (run `city_plan controls <road file> rules/lab.lua --layout --write --controls-only`)");
        if (!g.statesLaneRules)
            missing.push_back("its lane rules (run `lanes_tool layout <road file> --lane-rules 20 --write`)");
        if (!missing.empty()) {
            out.error = "no lane level for " + g.name + ": the road file does not state ";
            for (std::size_t i = 0; i < missing.size(); ++i) out.error += (i ? "; nor " : "") + missing[i];
            return out;
        }
    }
    out.leftOut.assign(nLanes, 0);
    out.laneLengths.assign(nLanes, 0.0);
    auto edgeAt = [&](uint32_t e) -> const EdgeSpec& { return g.edges[e]; };
    auto uOf = [&](uint32_t lane, double station) {   // a spine station along a lane's corridor
        const ModelLane& ml = m.lanes[lane];
        return ml.dir > 0 ? station : edgeAt(ml.edge).s.back() - station;
    };

    // ---- which lanes become lane paths, and the names -----------------------------------------------------------------
    std::vector<char> kept(nLanes, 0);
    for (std::size_t li = 0; li < nLanes; ++li) {
        const Lane& l = L.lanes[li];
        if (l.isConnector()) {
            out.leftOut[li] = 1;
            ++out.connectorsLeftOut;
            continue;
        }
        const bool carsDrive = l.parent >= 0 && !g.cls(edgeAt(static_cast<uint32_t>(l.parent))).track &&
                               !g.cls(edgeAt(static_cast<uint32_t>(l.parent))).footway;
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
    for (const std::pair<const std::string, RoadClassSpec>& c : g.classes) G.roadClassNames.push_back(c.first);
    for (const ModelJunction& J : m.junctions) G.controlNames.push_back(J.control);
    std::sort(G.controlNames.begin(), G.controlNames.end());
    G.controlNames.erase(std::unique(G.controlNames.begin(), G.controlNames.end()), G.controlNames.end());
    TagMask everyone = 0;
    for (std::size_t i = 0; i < G.vehicleTagNames.size() && i < kMaxTagNames; ++i) everyone |= TagMask(1) << i;
    if (G.laneTagNames.size() > kMaxTagNames)
        out.problems.push_back(fmt("%zu lane kinds, more than the %u a lane's tags can hold", G.laneTagNames.size(), kMaxTagNames));

    // ---- every kept lane the way its traffic runs, each point at its stated spine station ----------------------------
    std::vector<TravelLane> travel(nLanes);
    std::map<uint32_t, std::vector<Vec2>> spineTangents;   // per road, as the builder framed it
    for (std::size_t li = 0; li < nLanes; ++li) {
        if (!kept[li]) continue;
        const Lane& l = L.lanes[li];
        const ModelLane& ml = m.lanes[li];
        const EdgeSpec& e = edgeAt(static_cast<uint32_t>(l.parent));
        TravelLane& T = travel[li];
        const std::size_t n = l.xy.size();
        bool laid = ml.curve.count == n;
        for (std::size_t k = 0; k < n && laid; ++k) laid = stated(m.spineStations[ml.curve.first + k]);
        if (!laid) {
            out.problems.push_back("lane " + l.id + ": not laid beside its road's spine point for point: no lane path");
            kept[li] = 0;
            continue;
        }
        std::vector<Vec2>& tangent = spineTangents[static_cast<uint32_t>(l.parent)];
        if (tangent.empty()) {
            std::vector<Vec2> normal;
            frames(e.xy, tangent, normal);
        }
        std::vector<Vec2> forward;   // the way its road runs at each point, the way its traffic runs
        double back = 0;
        Vec2 backAt;
        bool sawBack = false;
        for (std::size_t k = 0; k < n; ++k) {
            const std::size_t idx = l.dir > 0 ? k : n - 1 - k;
            const double st = m.spineStations[ml.curve.first + idx];
            T.xy.push_back(l.xy[idx]);
            T.u.push_back(l.dir > 0 ? st : e.s.back() - st);
            const std::size_t i = static_cast<std::size_t>(std::lower_bound(e.s.begin(), e.s.end(), st - kCutTolerance) - e.s.begin());
            forward.push_back(tangent[std::min(i, tangent.size() - 1)] * static_cast<double>(l.dir));
            if (k > 0 && dot(T.xy[k] - T.xy[k - 1], forward[k]) < 0) {
                back += distance(T.xy[k], T.xy[k - 1]);
                if (!sawBack) backAt = T.xy[k];
                sawBack = true;
            }
        }
        T.s = stations(T.xy);
        if (back > kCutTolerance) {
            const double before = T.s.back();
            cutFolds(T.xy, T.u, forward);
            T.s = stations(T.xy);
            ++out.tidiedLanes;
            out.problems.push_back(fmt("lane %s runs backwards for %.2f m near (%.1f, %.1f): it lies on the inside of a corner of road "
                                       "%s sharper than its offset from the road's spine; tidied (the fold cut out, %.2f m shorter)",
                                       l.id.c_str(), back, backAt.x, backAt.y, e.id.c_str(), before - T.s.back()));
        }
        for (std::size_t k = 1; k < T.u.size(); ++k) T.u[k] = std::max(T.u[k], T.u[k - 1]);
        out.laneLengths[li] = T.s.back();
        T.ok = T.xy.size() >= 2 && T.s.back() > 0;
        if (!T.ok) kept[li] = 0;
    }

    // ---- each arm's stop line: the builder's, where the paint's box holds that junction alone ---------------------------
    std::map<std::tuple<std::string, double, double>, std::vector<uint32_t>> boxHolders;
    for (uint32_t j = 0; j < m.junctions.size(); ++j)
        for (const ModelStretch& b : m.junctions[j].boxes) boxHolders[std::make_tuple(b.road, b.from, b.to)].push_back(j);
    for (const std::pair<const std::tuple<std::string, double, double>, std::vector<uint32_t>>& bh : boxHolders) {
        if (bh.second.size() < 2) continue;
        std::string names;
        for (uint32_t j : bh.second) names += (names.empty() ? "" : ", ") + m.junctions[j].id;
        const std::map<std::string, int>::const_iterator ei = g.index.find(std::get<0>(bh.first));
        const Vec2 p = ei == g.index.end() ? Vec2()
                                           : pointAt(edgeAt(static_cast<uint32_t>(ei->second)).xy, edgeAt(static_cast<uint32_t>(ei->second)).s,
                                                     0.5 * (std::get<1>(bh.first) + std::get<2>(bh.first)));
        out.paintDisagrees.push_back(fmt("the paint made one junction box on %s (stations %.1f to %.1f) at (%.1f, %.1f) of stated "
                                         "junctions %s: their arms there stop at ADR-0419's stop lines, the stated junctions followed",
                                         std::get<0>(bh.first).c_str(), std::get<1>(bh.first), std::get<2>(bh.first), p.x, p.y, names.c_str()));
    }
    std::vector<std::vector<double>> stopAt(m.junctions.size());
    for (uint32_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        stopAt[j].assign(J.arms.size(), kNotStated);
        for (uint32_t a = 0; a < J.arms.size(); ++a) {
            const ModelArm& arm = J.arms[a];
            if (!arm.street.valid()) continue;
            bool shared = false;
            if (stated(arm.stopLine))
                for (const ModelStretch& b : J.boxes)
                    if (b.road == arm.road && (arm.spineIn ? b.from == arm.boxEdge : b.to == arm.boxEdge))
                        shared = shared || boxHolders[std::make_tuple(b.road, b.from, b.to)].size() > 1;
            if (stated(arm.stopLine) && !shared) {
                stopAt[j][a] = arm.stopLine;
                ++out.stopLinesPainted;
                continue;
            }
            ModelJunction planned = J;   // ADR-0419's rule, read from the stated cross-sections, not the paint's line
            planned.arms[a].stopLine = kNotStated;
            const Real back = stopLineFromMouth(m, planned, a);
            if (!stated(back)) {
                out.problems.push_back(fmt("junction %s: arm %u (street %s) has no stop line: its street has no centre line", J.id.c_str(), a,
                                           m.streets[arm.street.index].id.c_str()));
                continue;
            }
            stopAt[j][a] = arm.spineIn ? arm.station - back : arm.station + back;
            ++(shared ? out.stopLinesSharedBox : out.stopLinesNoBox);
        }
    }
    // the arm of junction j that street `street` is, at its `to` end (spineIn) or its `from` end
    auto armOf = [&](uint32_t j, uint32_t street, bool spineIn) {
        const ModelJunction& J = m.junctions[j];
        for (uint32_t a = 0; a < J.arms.size(); ++a)
            if (J.arms[a].street.index == street && J.arms[a].spineIn == spineIn) return static_cast<int>(a);
        return -1;
    };

    // ---- each lane's stretches along its streets, stop line to stop line ------------------------------------------------
    std::vector<Piece> pieces;
    std::vector<std::vector<uint32_t>> piecesOfLane(nLanes);
    for (uint32_t li = 0; li < nLanes; ++li) {
        if (!kept[li]) continue;
        const ModelLane& ml = m.lanes[li];
        std::vector<ModelLaneSpan> spans = ml.streets;
        if (ml.dir < 0) std::reverse(spans.begin(), spans.end());
        for (const ModelLaneSpan& sp : spans) {
            const ModelStreet& S = m.streets[sp.street.index];
            // (a street's stations are written to the centimetre: a lane reaching its end runs to its own end)
            double lo = sp.from - ml.spineFrom <= kReachesStreetEnd ? ml.spineFrom : sp.from;
            double hi = ml.spineTo - sp.to <= kReachesStreetEnd ? ml.spineTo : sp.to;
            int jLo = -1, aLo = -1, jHi = -1, aHi = -1, endLo = -1, endHi = -1;
            for (int end = 0; end < 2; ++end) {
                const double at = end == 0 ? S.from : S.to;
                if (std::fabs((end == 0 ? sp.from : sp.to) - at) > kReachesStreetEnd) continue;   // the lane does not reach it
                const ModelJunctionId J = S.junctions[static_cast<std::size_t>(end)];
                if (!J.valid()) {
                    (end == 0 ? endLo : endHi) = end;
                    continue;
                }
                const int a = armOf(J.index, sp.street.index, end == 1);
                if (a < 0 || !stated(stopAt[J.index][static_cast<std::size_t>(a)])) {
                    out.problems.push_back(fmt("street %s: its %s end names junction %s, which has no arm with a stop line for it",
                                               S.id.c_str(), end == 0 ? "`from`" : "`to`", m.junctions[J.index].id.c_str()));
                    continue;
                }
                const double stop = stopAt[J.index][static_cast<std::size_t>(a)];
                if (end == 0) {
                    lo = std::max(lo, stop);
                    jLo = static_cast<int>(J.index);
                    aLo = a;
                } else {
                    hi = std::min(hi, stop);
                    jHi = static_cast<int>(J.index);
                    aHi = a;
                }
            }
            if (!(hi - lo > kCutTolerance)) {
                out.problems.push_back(fmt("lane %s on street %s: no room between its stop lines (%.2f to %.2f along road %s): no lane path there",
                                           L.lanes[li].id.c_str(), S.id.c_str(), lo, hi, S.road.c_str()));
                continue;
            }
            Piece p;
            p.lane = li;
            p.street = sp.street.index;
            p.u0 = uOf(li, ml.dir > 0 ? lo : hi);
            p.u1 = uOf(li, ml.dir > 0 ? hi : lo);
            if (ml.dir > 0) {
                p.startJunction = jLo, p.startArm = aLo, p.startStreetEnd = endLo;
                p.endJunction = jHi, p.endArm = aHi, p.endStreetEnd = endHi;
            } else {
                p.startJunction = jHi, p.startArm = aHi, p.startStreetEnd = endHi;
                p.endJunction = jLo, p.endArm = aLo, p.endStreetEnd = endLo;
            }
            piecesOfLane[li].push_back(static_cast<uint32_t>(pieces.size()));
            pieces.push_back(std::move(p));
        }
    }

    // ---- sections and lane paths, road by road, each way --------------------------------------------------------------
    const double solid = g.laneRules.solidBeforeStopLine;
    std::map<std::pair<uint32_t, int>, std::vector<uint32_t>> corridors;   // (road, way) -> its pieces
    for (uint32_t p = 0; p < pieces.size(); ++p) {
        const ModelLane& ml = m.lanes[pieces[p].lane];
        corridors[std::make_pair(ml.edge, ml.dir)].push_back(p);
    }
    // where a lane leaves or merges into its host, on the host's corridor
    std::map<std::pair<uint32_t, int>, std::vector<double>> meetCuts;
    for (const ModelConnection& c : m.connections) {
        if (c.kind == ConnectionKind::Joins || !stated(c.meets)) continue;
        const uint32_t host = c.kind == ConnectionKind::Diverge ? c.from.index : c.to.index;
        if (host >= nLanes || !kept[host]) continue;
        meetCuts[std::make_pair(m.lanes[host].edge, m.lanes[host].dir)].push_back(uOf(host, c.meets));
    }
    std::vector<std::pair<double, double>> pathU;   // per lane path: its stretch along its corridor
    for (const std::pair<const std::pair<uint32_t, int>, std::vector<uint32_t>>& kc : corridors) {
        const EdgeSpec& e = edgeAt(kc.first.first);
        const std::vector<uint32_t>& here = kc.second;
        std::vector<double> cuts, stops;
        for (uint32_t p : here) {
            cuts.push_back(pieces[p].u0);
            cuts.push_back(pieces[p].u1);
            if (pieces[p].endJunction >= 0) stops.push_back(pieces[p].u1);
        }
        for (double c : meetCuts[kc.first])
            for (uint32_t p : here)
                if (c > pieces[p].u0 + kCutTolerance && c < pieces[p].u1 - kCutTolerance) cuts.push_back(c);
        // where the solid lines before a stop line begin, where lanes run side by side there
        for (double s : stops) {
            const double from = s - solid;
            int beside = 0;
            for (uint32_t p : here) beside += pieces[p].u0 <= from + kCutTolerance && pieces[p].u1 >= s - kCutTolerance ? 1 : 0;
            if (solid > 0 && beside >= 2) cuts.push_back(from);
        }
        std::sort(cuts.begin(), cuts.end());
        std::vector<double> cut;
        for (double x : cuts)
            if (cut.empty() || x - cut.back() > kCutTolerance) cut.push_back(x);
        for (std::size_t i = 0; i + 1 < cut.size(); ++i) {
            const double c0 = cut[i], c1 = cut[i + 1];
            std::vector<uint32_t> in;
            for (uint32_t p : here)
                if (pieces[p].u0 <= c0 + kCutTolerance && pieces[p].u1 >= c1 - kCutTolerance) in.push_back(p);
            if (in.empty()) continue;
            // the centre-line side first, by the builder's slots
            std::stable_sort(in.begin(), in.end(), [&](uint32_t a, uint32_t b) {
                const int sa = m.lanes[pieces[a].lane].slot, sb = m.lanes[pieces[b].lane].slot;
                return sa != sb ? sa < sb : pieces[a].lane < pieces[b].lane;
            });
            const SectionId sid(static_cast<uint32_t>(G.sections.size()));
            Section sec;
            sec.roadClass = LaneGraph::nameIndex(G.roadClassNames, e.cls);
            sec.lanes.first = static_cast<uint32_t>(G.sectionLanes.size());
            sec.lines.first = static_cast<uint32_t>(G.laneLines.size());
            for (std::size_t k = 0; k < in.size(); ++k) {
                Piece& pc = pieces[in[k]];
                const uint32_t li = pc.lane;
                const Lane& l = L.lanes[li];
                const TravelLane& T = travel[li];
                const double ta = tauAt(T, c0), tb = tauAt(T, c1);
                const std::vector<Vec2> pts = sliceAtStations(T.xy, T.s, ta, tb, kSliceGap);
                std::vector<double> z;
                for (const Vec2& q : pts) z.push_back(H.deck(static_cast<int>(li), q));
                Path p;
                p.kind = PathKind::Lane;
                const NameIndex tag = LaneGraph::nameIndex(G.laneTagNames, l.kind);
                p.laneTags = tag < kMaxTagNames ? TagMask(1) << tag : 0;
                p.vehicleTags = everyone;
                p.curve = G.addCurve(pts, z);
                p.length = G.along[p.curve.end() - 1];
                p.width = l.w;
                p.speedLimit = m.streets[pc.street].speedLimit;
                p.section = sid;
                p.indexInSection = static_cast<uint32_t>(k);
                p.street = pc.street;
                p.modelLane = li;
                const uint32_t id = static_cast<uint32_t>(G.paths.size());
                G.paths.push_back(p);
                G.sectionLanes.push_back(PathId(id));
                PathSource src;
                src.lane = static_cast<int>(li);
                src.from = ta;
                src.to = tb;
                out.sources.push_back(src);
                pathU.emplace_back(c0, c1);
                pc.paths.push_back(id);
            }
            // the lines between neighbours: solid between two through lanes on the approach to the stop line they run to
            double approach = -1;
            for (double s : stops)
                if (c1 <= s + kCutTolerance && c0 >= s - solid - kCutTolerance) approach = s;
            for (std::size_t k = 0; k + 1 < in.size(); ++k) {
                bool solidLine = approach >= 0;
                for (uint32_t p : {in[k], in[k + 1]})
                    solidLine = solidLine && L.lanes[pieces[p].lane].kind == "through" && pieces[p].u1 >= approach - kCutTolerance;
                LaneLine line;
                line.crossFromOffside = line.crossFromKerbside = !solidLine;
                G.laneLines.push_back(line);
            }
            sec.lanes.count = static_cast<uint32_t>(G.sectionLanes.size()) - sec.lanes.first;
            sec.lines.count = static_cast<uint32_t>(G.laneLines.size()) - sec.lines.first;
            G.sections.push_back(sec);
            SectionSource ss;
            ss.edge = static_cast<int>(kc.first.first);
            ss.dir = kc.first.second;
            ss.from = c0;
            ss.to = c1;
            out.sectionSources.push_back(ss);
        }
    }
    // each piece's paths in order along it (sections came corridor by corridor, cut by cut)
    for (Piece& pc : pieces)
        std::sort(pc.paths.begin(), pc.paths.end(), [&](uint32_t a, uint32_t b) { return pathU[a].first < pathU[b].first; });

    // ---- what follows each lane: all stated ---------------------------------------------------------------------------
    std::vector<std::vector<PathId>> next(G.paths.size());
    auto joinGap = [&](uint32_t from, uint32_t into) {
        return distance(G.pointAt(PathId(from), G.paths[from].length), G.pointAt(PathId(into), 0));
    };
    // along one lane: the next stretch of it, within a piece and from one piece to the next where they meet
    for (uint32_t li = 0; li < nLanes; ++li) {
        const std::vector<uint32_t>& ps = piecesOfLane[li];
        for (std::size_t k = 0; k < ps.size(); ++k) {
            const Piece& pc = pieces[ps[k]];
            for (std::size_t q = 0; q + 1 < pc.paths.size(); ++q) next[pc.paths[q]].push_back(PathId(pc.paths[q + 1]));
            if (k + 1 < ps.size() && !pc.paths.empty() && !pieces[ps[k + 1]].paths.empty() &&
                std::fabs(pieces[ps[k + 1]].u0 - pc.u1) <= kCutTolerance)
                next[pc.paths.back()].push_back(PathId(pieces[ps[k + 1]].paths.front()));
        }
    }
    // where a street carries on as another (no junction): into the lane of the same slot on the next street
    for (const Piece& pc : pieces) {
        if (pc.endStreetEnd < 0 || pc.paths.empty()) continue;
        const ModelStreet& S = m.streets[pc.street];
        const ModelStreetId T = S.carriesOn[static_cast<std::size_t>(pc.endStreetEnd)];
        if (!T.valid()) continue;   // a dead end, or a ramp's anchored end (its merge, below)
        const ModelStreet& TS = m.streets[T.index];
        // the end of the next street that names this one back and has lanes leaving it (a ring of two streets names its
        // partner at both ends: the way the lanes run tells which; two such ends is not stated, reported)
        std::vector<int> backs;
        bool named = false;
        for (int e = 0; e < 2; ++e) {
            if (TS.carriesOn[static_cast<std::size_t>(e)].index != pc.street) continue;
            named = true;
            for (const Piece& qc : pieces)
                if (qc.street == T.index && qc.startStreetEnd == e && !qc.paths.empty()) {
                    backs.push_back(e);
                    break;
                }
        }
        if (!named) {
            out.problems.push_back(fmt("street %s carries on into %s, which does not name it back: its lanes lead nowhere there",
                                       S.id.c_str(), TS.id.c_str()));
            continue;
        }
        if (backs.size() != 1) {
            out.problems.push_back(fmt("street %s carries on into %s, %s: lane %s leads nowhere there", S.id.c_str(), TS.id.c_str(),
                                       backs.empty() ? "where no lane leaves" : "which names it back at both ends with lanes leaving each",
                                       L.lanes[pc.lane].id.c_str()));
            continue;
        }
        const int back = backs.front();
        const int slot = m.lanes[pc.lane].slot;
        int best = -1;
        for (uint32_t q = 0; q < pieces.size(); ++q) {
            const Piece& qc = pieces[q];
            if (qc.street != T.index || qc.startStreetEnd != back || qc.paths.empty()) continue;
            if (best < 0 || std::abs(m.lanes[qc.lane].slot - slot) < std::abs(m.lanes[pieces[static_cast<std::size_t>(best)].lane].slot - slot))
                best = static_cast<int>(q);
        }
        if (best < 0) {
            out.problems.push_back(fmt("street %s carries on into %s, where no lane leaves its end the way lane %s runs", S.id.c_str(),
                                       TS.id.c_str(), L.lanes[pc.lane].id.c_str()));
            continue;
        }
        const Piece& into = pieces[static_cast<std::size_t>(best)];
        if (m.lanes[into.lane].slot != slot)
            out.problems.push_back(fmt("lane %s (slot %d) carries on from street %s into %s, which has no lane in its slot: into lane %s "
                                       "(slot %d)", L.lanes[pc.lane].id.c_str(), slot, S.id.c_str(), TS.id.c_str(),
                                       L.lanes[into.lane].id.c_str(), m.lanes[into.lane].slot));
        next[pc.paths.back()].push_back(PathId(into.paths.front()));
        ++out.carryOnJoins;
        const double gap = joinGap(pc.paths.back(), into.paths.front());
        if (gap > kJoinReported) {
            ++out.joinsWithGap;
            const Vec2 at = G.pointAt(PathId(pc.paths.back()), G.paths[pc.paths.back()].length);
            out.problems.push_back(fmt("lane %s carries on from street %s into lane %s of street %s, their lines %.3f m apart at (%.2f, "
                                       "%.2f): joined", L.lanes[pc.lane].id.c_str(), S.id.c_str(), L.lanes[into.lane].id.c_str(),
                                       TS.id.c_str(), gap, at.x, at.y));
        }
    }
    // where a lane leaves its host (the host's lane path ending where their lines meet leads on into it too), and where one
    // merges into its host (its end leads into the host's lane path from where their lines meet)
    auto hostPath = [&](uint32_t host, double u, bool endingThere) {
        for (uint32_t p : piecesOfLane[host])
            for (uint32_t path : pieces[p].paths)
                if (std::fabs((endingThere ? pathU[path].second : pathU[path].first) - u) <= kCutTolerance) return static_cast<int>(path);
        return -1;
    };
    for (const ModelConnection& c : m.connections) {
        if (c.kind == ConnectionKind::Joins) continue;
        const bool diverge = c.kind == ConnectionKind::Diverge;
        const uint32_t host = diverge ? c.from.index : c.to.index, branch = diverge ? c.to.index : c.from.index;
        if (host >= nLanes || branch >= nLanes || !kept[host] || !kept[branch] || !stated(c.meets)) continue;
        const std::vector<uint32_t>& bp = piecesOfLane[branch];
        // the branch's own end there: its first stretch from its start (a diverge), its last to its end (a merge)
        const TravelLane& TB = travel[branch];
        const bool reaches = !bp.empty() && (diverge ? std::fabs(pieces[bp.front()].u0 - TB.u.front()) <= kCutTolerance
                                                     : std::fabs(pieces[bp.back()].u1 - TB.u.back()) <= kCutTolerance);
        const int hp = hostPath(host, uOf(host, c.meets), diverge);
        if (!reaches || hp < 0) {
            out.problems.push_back(fmt("lane %s %s lane %s at station %.2f along %s, where %s has no lane path to %s: not joined",
                                       L.lanes[branch].id.c_str(), diverge ? "leaves" : "merges into", L.lanes[host].id.c_str(), c.meets,
                                       edgeAt(m.lanes[host].edge).id.c_str(), !reaches ? L.lanes[branch].id.c_str() : L.lanes[host].id.c_str(),
                                       !reaches ? (diverge ? "start from" : "end at") : "meet it (inside a junction, or past its lane)"));
            continue;
        }
        const uint32_t from = diverge ? static_cast<uint32_t>(hp) : pieces[bp.back()].paths.back();
        const uint32_t into = diverge ? pieces[bp.front()].paths.front() : static_cast<uint32_t>(hp);
        next[from].push_back(PathId(into));
        ++(diverge ? out.divergeJoins : out.mergeJoins);
        const double gap = joinGap(from, into);
        if (gap > kJoinReported) {
            ++out.joinsWithGap;
            const Vec2 at = G.pointAt(PathId(into), 0);
            out.problems.push_back(fmt("lane %s %s lane %s, their lines %.3f m apart at (%.2f, %.2f): joined", L.lanes[branch].id.c_str(),
                                       diverge ? "leaves" : "merges into", L.lanes[host].id.c_str(), gap, at.x, at.y));
        }
    }

    // ---- the junctions: one per stated junction its lanes reach ------------------------------------------------------
    std::vector<int> laneJunctionOf(m.junctions.size(), -1);
    std::vector<std::vector<std::pair<uint32_t, uint32_t>>> arrivingAt(m.junctions.size()), leavingAt(m.junctions.size());   // (path, arm)
    for (const Piece& pc : pieces) {
        if (pc.paths.empty()) continue;
        if (pc.endJunction >= 0)
            arrivingAt[static_cast<std::size_t>(pc.endJunction)].emplace_back(pc.paths.back(), static_cast<uint32_t>(pc.endArm));
        if (pc.startJunction >= 0)
            leavingAt[static_cast<std::size_t>(pc.startJunction)].emplace_back(pc.paths.front(), static_cast<uint32_t>(pc.startArm));
    }
    std::unique_ptr<LaneLookup> ownLook;   // (the build keeps the one that found its junction boxes)
    if (!r.lookup) ownLook = std::make_unique<LaneLookup>(r.pavement);
    const LaneLookup& look = r.lookup ? *r.lookup : *ownLook;
    out.closedTo.assign(G.vehicleTagNames.size(), 0);
    const uint32_t tagCount = static_cast<uint32_t>(std::min<std::size_t>(G.vehicleTagNames.size(), kMaxTagNames));
    std::vector<std::vector<std::string>> regulationsOn(G.paths.size());
    const std::chrono::steady_clock::time_point tConnect = std::chrono::steady_clock::now();
    std::size_t turn = 0;   // m.turns run junction by junction
    for (uint32_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        const uint32_t firstTurn = static_cast<uint32_t>(turn);
        while (turn < m.turns.size() && m.turns[turn].junction.index == j) ++turn;
        if (arrivingAt[j].empty() && leavingAt[j].empty()) {
            if (turn > firstTurn) out.problems.push_back("junction " + J.id + ": turns are stated, but no lane path reaches it");
            continue;
        }
        const uint32_t lj = static_cast<uint32_t>(G.junctions.size());
        laneJunctionOf[j] = static_cast<int>(lj);
        Junction jn;
        jn.modelJunction = j;
        jn.control = LaneGraph::nameIndex(G.controlNames, J.control);
        jn.crossings = Range{static_cast<uint32_t>(G.crossings.size()), 0};
        jn.crosswalks = Range{static_cast<uint32_t>(G.crosswalks.size()), 0};
        // its outline: across each arm's lanes at its stop line, the arms in their stated order round it
        {
            struct Leg { Vec2 a, b, mid; double heading = 0; uint32_t arm = 0; };
            std::vector<Leg> legs;
            std::vector<Vec2> all;
            for (uint32_t a = 0; a < J.arms.size(); ++a) {
                std::vector<std::pair<Vec2, double>> ends;   // the lane ends at the arm's stop line, each with its half width
                Vec2 across;
                for (int arriving = 1; arriving >= 0; --arriving)
                    for (const std::pair<uint32_t, uint32_t>& pa : arriving ? arrivingAt[j] : leavingAt[j]) {
                        if (pa.second != a) continue;
                        const Real at = arriving ? G.paths[pa.first].length : Real(0);
                        ends.emplace_back(G.pointAt(PathId(pa.first), at), G.paths[pa.first].width / 2);
                        // the arm's way across: the arriving traffic's kerb side (the leaving traffic runs the other way)
                        across += perp(G.directionAt(PathId(pa.first), at)) * (arriving ? 1.0 : -1.0);
                    }
                if (ends.empty()) continue;
                across = across.length() > 0 ? normalize(across) : Vec2(1, 0);
                std::sort(ends.begin(), ends.end(), [&](const std::pair<Vec2, double>& x, const std::pair<Vec2, double>& y) {
                    return dot(x.first, across) < dot(y.first, across);
                });
                Leg leg;
                leg.a = ends.front().first - across * ends.front().second;
                leg.b = ends.back().first + across * ends.back().second;
                leg.mid = (leg.a + leg.b) * 0.5;
                leg.heading = stated(J.arms[a].heading) ? J.arms[a].heading : 0.0;
                leg.arm = a;
                legs.push_back(leg);
                for (const std::pair<Vec2, double>& e : ends) all.push_back(e.first);
                all.push_back(leg.a);
                all.push_back(leg.b);
            }
            std::vector<Vec2> ring;
            if (legs.size() < 2) {
                ring = convexHull(all);
            } else {
                // drawn round the legs in order round their middle (where they lie, not their stated headings: a spread or
                // skewed junction's arms leave in an order their mouths do not keep, and an outline drawn by headings
                // crosses itself). A drawing of the area that must not be blocked, not a fact the table decides.
                Vec2 centre;
                for (const Leg& leg : legs) centre += leg.mid;
                centre = centre / static_cast<double>(legs.size());
                std::stable_sort(legs.begin(), legs.end(), [&](const Leg& x, const Leg& y) {
                    return std::atan2(x.mid.y - centre.y, x.mid.x - centre.x) < std::atan2(y.mid.y - centre.y, y.mid.x - centre.x);
                });
                for (Leg& leg : legs) {
                    if (cross(leg.a - centre, leg.b - centre) < 0) std::swap(leg.a, leg.b);   // a to b runs counter-clockwise
                    ring.push_back(leg.a);
                    ring.push_back(leg.b);
                }
            }
            jn.box = G.addPlanPoints(ring);
            if (ring.size() < 3)
                out.problems.push_back(fmt("junction %s at (%.1f, %.1f): its outline has %zu corners", J.id.c_str(), J.at.x, J.at.y, ring.size()));
            bool tangled = false;
            for (std::size_t a = 0; a < ring.size() && !tangled; ++a)
                for (std::size_t b = a + 2; b < ring.size() && !tangled; ++b) {
                    if ((b + 1) % ring.size() == a) continue;   // neighbours share a corner
                    Vec2 hit;
                    tangled = segmentsCross(ring[a], ring[(a + 1) % ring.size()], ring[b], ring[(b + 1) % ring.size()], hit);
                }
            if (tangled)
                out.problems.push_back(fmt("junction %s at (%.1f, %.1f): its outline crosses itself (two arms' stop lines overlap: "
                                           "a corner for step 9)", J.id.c_str(), J.at.x, J.at.y));
        }
        std::vector<uint32_t> arriving, leaving;
        for (const std::pair<uint32_t, uint32_t>& pa : arrivingAt[j]) {
            arriving.push_back(pa.first);
            for (const std::string& reg : J.arms[pa.second].regulations) regulationsOn[pa.first].push_back(reg);
        }
        for (const std::pair<uint32_t, uint32_t>& pa : leavingAt[j]) leaving.push_back(pa.first);
        // a sign the file states on an arm no lane arrives by: impossible, reported (never stood elsewhere)
        for (uint32_t a = 0; a < J.arms.size(); ++a) {
            if (J.arms[a].regulations.empty()) continue;
            bool any = false;
            for (const std::pair<uint32_t, uint32_t>& pa : arrivingAt[j]) any = any || pa.second == a;
            if (!any)
                out.problems.push_back(fmt("junction %s: its arm on road %s carries \"%s\", but no lane arrives by it", J.id.c_str(),
                                           J.arms[a].road.c_str(), J.arms[a].regulations.front().c_str()));
        }
        std::sort(arriving.begin(), arriving.end());
        std::sort(leaving.begin(), leaving.end());
        jn.arriving.first = static_cast<uint32_t>(G.junctionLanes.size());
        for (uint32_t p : arriving) G.junctionLanes.push_back(PathId(p));
        jn.arriving.count = static_cast<uint32_t>(arriving.size());
        jn.leaving.first = static_cast<uint32_t>(G.junctionLanes.size());
        for (uint32_t p : leaving) G.junctionLanes.push_back(PathId(p));
        jn.leaving.count = static_cast<uint32_t>(leaving.size());
        JunctionNote note;
        note.at = J.at;
        for (const ModelArm& a : J.arms) note.roads += (note.roads.empty() ? "" : ", ") + a.road;
        // the connectors: one for each stated turn, fitted to its corner
        jn.connectors = Range{static_cast<uint32_t>(G.junctionConnectors.size()), 0};
        for (uint32_t t = firstTurn; t < turn; ++t) {
            const ModelTurn& mt = m.turns[t];
            ++note.turnsStated;
            int from = -1, into = -1;
            for (const std::pair<uint32_t, uint32_t>& pa : arrivingAt[j])
                if (pa.second == mt.fromArm && G.paths[pa.first].modelLane == mt.from.index) from = static_cast<int>(pa.first);
            for (const std::pair<uint32_t, uint32_t>& pa : leavingAt[j])
                if (pa.second == mt.toArm && G.paths[pa.first].modelLane == mt.to.index) into = static_cast<int>(pa.first);
            if (from < 0 || into < 0) {
                CornerFault f;
                f.turn = t;
                f.why = fmt("the %s from lane %s into lane %s has no lane path %s", turnKey(mt.turn), m.lanes[mt.from.index].id.c_str(),
                            m.lanes[mt.to.index].id.c_str(), from < 0 ? "to start from" : "to end on");
                f.at = J.at;
                out.cornerFaults.push_back(f);
                ++note.turnsRefused;
                continue;
            }
            const ConnectorFit fit = fitConnector(r, look, G, PathId(static_cast<uint32_t>(from)), PathId(static_cast<uint32_t>(into)),
                                                  static_cast<int>(mt.from.index), static_cast<int>(mt.to.index), opts.vehicles);
            for (uint32_t i = 0; i < tagCount; ++i) {
                if (fit.whyClosed[i].empty()) continue;
                CornerFault f;
                f.turn = t;
                f.tag = G.vehicleTagNames[i];
                f.why = fit.whyClosed[i];
                f.at = fit.at;
                out.cornerFaults.push_back(f);
            }
            const bool none = !fit.shape.ok || (tagCount > 0 ? fit.open == 0 : !fit.why.empty());
            if (none) {
                if (fit.noArc || tagCount == 0 || fit.why == "no vehicle may use both lanes") {
                    CornerFault f;
                    f.turn = t;
                    f.why = fit.why;
                    f.at = fit.at;
                    out.cornerFaults.push_back(f);
                }
                Refusal ref;
                ref.junction = lj;
                ref.from = PathId(static_cast<uint32_t>(from));
                ref.to = PathId(static_cast<uint32_t>(into));
                ref.turn = mt.turn;
                ref.at = fit.at;
                out.refusals.push_back(ref);
                out.problems.push_back(fmt("junction %s: the %s from lane %s into lane %s fits no vehicle: %s (it stays stated; a corner "
                                           "fault)", J.id.c_str(), turnKey(mt.turn), m.lanes[mt.from.index].id.c_str(),
                                           m.lanes[mt.to.index].id.c_str(), fit.why.c_str()));
                ++note.turnsRefused;
                continue;
            }
            const Path& fromPath = G.paths[static_cast<std::size_t>(from)];
            const Path& intoPath = G.paths[static_cast<std::size_t>(into)];
            Path c;
            c.kind = PathKind::Connector;
            c.turn = mt.turn;
            c.laneTags = fromPath.laneTags | intoPath.laneTags;
            c.vehicleTags = fit.open;
            c.curve = G.addCurve(fit.shape.points, fit.z);
            c.length = G.along[c.curve.end() - 1];
            c.width = std::min(fromPath.width, intoPath.width);
            c.tightestRadius = fit.shape.tightestRadius;
            c.speedLimit = intoPath.speedLimit;
            c.junction = JunctionId(lj);
            c.from = PathId(static_cast<uint32_t>(from));
            c.modelTurn = t;
            const PathId id(static_cast<uint32_t>(G.paths.size()));
            G.paths.push_back(c);
            out.sources.push_back(PathSource());
            regulationsOn.emplace_back();
            next[static_cast<std::size_t>(from)].push_back(id);
            next.push_back({PathId(static_cast<uint32_t>(into))});
            G.junctionConnectors.push_back(id);
            ++out.connectorsByTurn[static_cast<std::size_t>(mt.turn)];
            ++note.turnsBuilt;
            for (uint32_t i = 0; i < tagCount; ++i) out.closedTo[i] += (fit.closed >> i) & 1u;
        }
        jn.connectors.count = static_cast<uint32_t>(G.junctionConnectors.size()) - jn.connectors.first;
        G.junctions.push_back(jn);
        out.junctionNotes.push_back(note);
    }
    out.connectMilliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tConnect).count();
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        std::sort(next[p].begin(), next[p].end());
        next[p].erase(std::unique(next[p].begin(), next[p].end()), next[p].end());
        G.paths[p].next.first = static_cast<uint32_t>(G.nextPaths.size());
        G.nextPaths.insert(G.nextPaths.end(), next[p].begin(), next[p].end());
        G.paths[p].next.count = static_cast<uint32_t>(next[p].size());
    }

    // ---- the signs: each arm's, at the end of every lane arriving by it ---------------------------------------------------
    for (const std::vector<std::string>& kinds : regulationsOn) G.regulationKindNames.insert(G.regulationKindNames.end(), kinds.begin(), kinds.end());
    std::sort(G.regulationKindNames.begin(), G.regulationKindNames.end());
    G.regulationKindNames.erase(std::unique(G.regulationKindNames.begin(), G.regulationKindNames.end()), G.regulationKindNames.end());
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        std::vector<std::string>& kinds = regulationsOn[p];
        std::sort(kinds.begin(), kinds.end());
        kinds.erase(std::unique(kinds.begin(), kinds.end()), kinds.end());
        G.paths[p].regulations = Range{static_cast<uint32_t>(G.regulations.size()), 0};
        for (const std::string& kind : kinds) {
            Regulation reg;
            reg.kind = LaneGraph::nameIndex(G.regulationKindNames, kind);
            reg.path = PathId(p);
            reg.from = reg.to = G.paths[p].length;
            G.regulations.push_back(reg);
        }
        G.paths[p].regulations.count = static_cast<uint32_t>(G.regulations.size()) - G.paths[p].regulations.first;
    }

    // ---- every path's bend profile, as a vehicle feels it, and the kinks a driver would feel (#281) -----------------
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        Path& path = G.paths[p];
        const Vec2* xy = G.points.data() + path.curve.first;
        const double* s = G.along.data() + path.curve.first;
        const std::vector<BendPiece> bends = measureBends(xy, s, path.curve.count);
        double tightest = kNoBend;
        for (const BendPiece& b : bends) tightest = std::min(tightest, b.radius);
        if (path.kind == PathKind::Lane) path.tightestRadius = tightest;
        const bool plain = bends.size() == 1 && bends[0].radius == path.tightestRadius;
        path.bends = plain ? Range{static_cast<uint32_t>(G.bendPieces.size()), 0} : G.addBends(bends);
        const bool lane = path.kind == PathKind::Lane;
        for (const Kink& k : findKinks(xy, s, path.curve.count)) {
            if (lane) {
                ++out.laneKinks;
                out.worstLaneKink = std::max(out.worstLaneKink, k.degrees);
                out.problems.push_back(fmt("kink: the line of lane %s (lane path %u) jumps %.1f degrees at one point, %.1f m along it at (%.1f, "
                                           "%.1f), beyond the bend around it (more than %.0f): kept as drawn, and the bend profile slows for it",
                                           L.lanes[path.modelLane].id.c_str(), p, k.degrees, k.along, k.at.x, k.at.y, kKinkDegrees));
            } else {
                ++out.connectorKinks;
                out.worstConnectorKink = std::max(out.worstConnectorKink, k.degrees);
                out.problems.push_back(fmt("kink: connector %u (junction %u) jumps %.1f degrees at one point, %.1f m along it at (%.1f, %.1f), "
                                           "beyond the bend around it (more than %.0f)",
                                           p, path.junction.index, k.degrees, k.along, k.at.x, k.at.y, kKinkDegrees));
            }
        }
    }
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        const Vec2 leaving = G.directionAt(PathId(p), G.paths[p].length);
        for (uint32_t k = G.paths[p].next.first; k < G.paths[p].next.end(); ++k) {
            const PathId n = G.nextPaths[k];
            const double degrees = std::acos(std::clamp(dot(leaving, G.directionAt(n, 0)), -1.0, 1.0)) * kDegreesPerRadian;
            if (degrees <= kKinkDegrees) continue;
            ++out.joinKinks;
            out.worstJoinKink = std::max(out.worstJoinKink, degrees);
            const Vec2 at = G.pointAt(n, 0);
            out.problems.push_back(fmt("kink: %s %u leads into %s %u with its heading jumping %.1f degrees at (%.1f, %.1f) (more than %.0f)",
                                       G.paths[p].kind == PathKind::Lane ? "lane" : "connector", p,
                                       G.paths[n.index].kind == PathKind::Lane ? "lane" : "connector", n.index, degrees, at.x, at.y,
                                       kKinkDegrees));
        }
    }
    out.bendPieceBytes = G.bendPieces.size() * sizeof(BendPiece) + G.paths.size() * sizeof(Range);

    // ---- where the paths' bodies could touch (measured), and where a far turn may wait inside -----------------------------
    CrossingOptions co;
    co.vehicles = opts.vehicles;
    co.clearance = g.rules.sameLevelDz;
    out.crossings = findCrossingPoints(G, co);
    out.problems.insert(out.problems.end(), out.crossings.problems.begin(), out.crossings.problems.end());
    out.crossings.waitInsidePoints = placeWaitInsidePoints(G);

    // ---- the counts the old reports read ------------------------------------------------------------------------------
    out.controlPlan.declared = true;
    out.controlPlan.planned = static_cast<uint32_t>(m.junctions.size());
    for (const Junction& jn : G.junctions) {
        bool found = false;
        for (std::pair<std::string, uint32_t>& c : out.controlPlan.byControl)
            if (c.first == G.controlNames[jn.control]) {
                ++c.second;
                found = true;
            }
        if (!found) out.controlPlan.byControl.emplace_back(G.controlNames[jn.control], 1);
    }
    std::sort(out.controlPlan.byControl.begin(), out.controlPlan.byControl.end());
    for (const Path& p : G.paths) {
        if (p.kind == PathKind::Lane && p.speedLimit > 0) ++out.controlPlan.lanesPosted;
        if (p.kind == PathKind::Connector && p.speedLimit > 0) ++out.controlPlan.connectorsPosted;
    }
    for (const Regulation& reg : G.regulations) {
        bool found = false;
        for (std::pair<std::string, uint32_t>& c : out.controlPlan.regulations)
            if (c.first == G.regulationKindNames[reg.kind]) {
                ++c.second;
                found = true;
            }
        if (!found) out.controlPlan.regulations.emplace_back(G.regulationKindNames[reg.kind], 1);
    }
    std::sort(out.controlPlan.regulations.begin(), out.controlPlan.regulations.end());

    for (const std::string& p : validate(G)) out.problems.push_back("lane graph: " + p);
    reachReport(out);
    out.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

LaneGraphBuild buildLaneGraph(const Result& r, const LaneGraphOptions& opts) { return buildLaneLevel(r, assembleRoadModel(r), opts); }

}  // namespace roads::lanes
}  // namespace engine
