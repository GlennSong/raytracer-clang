#include "engine/procgen/city/roads/lanes/lane_level.h"

#include "engine/procgen/city/polygon.h"                            // convexHull
#include "engine/procgen/city/polyline.h"                           // segmentsCross: an outline crossing itself
#include "engine/procgen/city/roads/lane_turns.h"                   // turnKey: a turn's word; laneLineBetween: the lines' one rule
#include "engine/procgen/city/roads/lanes/deck_mesh.h"              // LaneLookup: the paved road, for the corner fit
#include "engine/procgen/city/roads/lanes/lane_graph_connect.h"     // fitConnector, reachReport
#include "engine/procgen/city/roads/lanes/lane_graph_crossings.h"   // where the paths' bodies could touch
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"         // kArmOrderCommand: a file whose arms are not in ring order

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
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
// A hole this small between two stretches of a lane (metres) is the road file's stations written to the centimetre, at
// the junction beside it: two written stations' worth (rt_math.h kWrittenStation, each end of the hole) and the cut
// tolerance.
constexpr double kStationRounding = 2 * kWrittenStation + kCutTolerance;
constexpr double kDegreesPerRadian = 57.29577951308232;

// A message of any length (measured first, then written: never cut short).
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
std::string fmt(const char* f, ...) {
    va_list args;
    va_start(args, f);
    va_list again;
    va_copy(again, args);
    const int n = std::vsnprintf(nullptr, 0, f, args);
    va_end(args);
    std::string msg(n > 0 ? static_cast<std::size_t>(n) : 0, '\0');
    if (n > 0) std::vsnprintf(msg.data(), msg.size() + 1, f, again);
    va_end(again);
    return msg;
}

// A builder lane read the way its traffic runs: its points, the distance along it to each, and each point's distance
// along its road's spine the way the traffic runs (u: its spine station, or the spine's length less it running against
// the spine) -- the builder laid each point beside a stated spine station (RoadModel::spineStations).
struct TravelLane {
    std::vector<Vec2> xy;
    std::vector<double> s, u;
    double width = 0;   // the width driven (drivenLine)
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
    // it starts (ends) at its street's end inside a junction that has no stop line for its arm: nothing is joined across
    // that end (each such end a problem line)
    bool startNoStop = false, endNoStop = false;
    std::vector<uint32_t> paths;                  // its lane paths, in order
};

}  // namespace

std::string laneLevelRefusal(const RoadLabGraph& g, const RoadModel& m, bool builtLayer) {
    std::vector<std::string> missing;
    if (!builtLayer) missing.push_back("the road model's built layer for this build (assembleRoadModel)");
    if (!g.listsJunctions || !g.listsStreets)
        missing.push_back("its junctions and streets (run `lanes_tool layout <road file> --write`)");
    if (!g.declaresJunctions)
        missing.push_back("every junction's control (run `city_plan controls <road file> rules/lab.lua --layout --write --controls-only`)");
    if (!g.statesLaneRules)
        missing.push_back("its lane rules (run `lanes_tool layout <road file> --lane-rules 20 --write`)");
    if (!m.armsInRing)
        missing.push_back("its junctions' arms listed in their ring (run `" + std::string(kArmOrderCommand) + "`)");
    if (missing.empty()) return std::string();
    std::string error = "no lane level for " + g.name + ": the road file does not state ";
    for (std::size_t i = 0; i < missing.size(); ++i) error += (i ? "; nor " : "") + missing[i];
    return error;
}

std::vector<Vec2> drivenLanePoints(const RoadLabGraph& g, const RoadModel& m, uint32_t lane, double* width) {
    std::vector<Vec2> out;
    if (lane >= m.lanes.size()) return out;
    const ModelLane& ml = m.lanes[lane];
    out.assign(m.lanePoints().begin() + ml.curve.first, m.lanePoints().begin() + ml.curve.end());
    if (width) *width = ml.width;
    if (!ml.onRoad() || ml.kind != "through" || ml.slot < 0) return out;
    const EdgeSpec& e = g.edges[ml.edge];
    double across = 0;
    for (const LaneSlot& sl : laneSlots(g.cls(e), e.lanes)) {
        if (sl.dir != ml.dir || std::atoi(sl.name.c_str() + 1) != ml.slot) continue;
        const DrivenLine dl = drivenLine(g, e, sl);
        across = dl.offset - sl.offset;
        if (width) *width = dl.width;
    }
    if (across == 0) return out;
    std::vector<Vec2> tangent, normal;
    frames(e.xy, tangent, normal);
    for (uint32_t k = 0; k < ml.curve.count; ++k) {
        const double st = m.spineStations[ml.curve.first + k];
        if (!stated(st)) continue;   // (a lane not laid beside its spine: the lane level leaves it out)
        const std::size_t at = static_cast<std::size_t>(std::lower_bound(e.s.begin(), e.s.end(), st - kCutTolerance) - e.s.begin());
        out[k] = out[k] + normal[std::min(at, normal.size() - 1)] * across;
    }
    return out;
}

namespace {

// ---- THE WALKS (ADR-0466) ---------------------------------------------------------------------------------------------
// WHERE WALK ENDS MEET, each a stated place: the kerb end of a junction's arm on one kerb (junction, arm, kerb; a
// footway arm has one, kerb 0: its walk is its middle); a street's end on one kerb where no junction is (street, end,
// kerb: where it carries on as another, the lower of the two streets' ends names the place); a dead end's kerb (dead end,
// kerb), where its turn-round begins or ends.
enum class MeetAt : uint8_t { Kerb, StreetEnd, DeadEnd };
using MeetKey = std::tuple<uint8_t, uint32_t, uint32_t, int>;

// LAYS THE WALKS INTO THE LANE LEVEL and joins them at their stated places (lane_graph.h Walk): each sidewalk lane and
// footway lane cut into its streets between the kerb ends at its junctions (kerbEndStation), each corner's walk whole,
// a crossing over each crosswalk people use (kerb end 0 to kerb end 1), and a turn-round across each dead end the model
// joins. After every path's curve, so no car path's points move. Every walk end that meets nothing is listed with why.
void layWalkPaths(const Result& r, RoadModel& model, LaneGraphBuild& out) {
    LaneGraph& G = model.laneLevel;
    const RoadModel& m = model;
    const RoadLabGraph& g = r.graph;
    const DeckHeight& H = *r.heights;
    G.walks.clear();
    G.walkJoins.clear();
    out.walkEndsAlone.clear();
    std::map<MeetKey, std::vector<WalkEnd>> meets;
    std::map<MeetKey, std::string> whyAlone;   // what a lone end at a place not at a junction says (the first said)
    const auto footwayArm = [&](ModelJunctionId j, uint32_t arm) {
        const ModelStreet* s = m.streetOf(m.junctions[j.index].arms[arm]);
        return s && s->section.footway;
    };
    const auto kerbKey = [&](ModelJunctionId j, uint32_t arm, int kerb) {
        return MeetKey(static_cast<uint8_t>(MeetAt::Kerb), j.index, arm, footwayArm(j, arm) ? 0 : kerb);
    };
    const auto ownEnd = [](uint32_t street, int end, int kerb) {
        return MeetKey(static_cast<uint8_t>(MeetAt::StreetEnd), street, static_cast<uint32_t>(end), kerb);
    };
    // where a street carries on as another (ModelStreet::carriesOn): its end and the end of the next street that names it
    // back are one place; the kerb runs on, unless the two spines meet head to head (both `to`, or both `from`)
    const auto carryOnKey = [&](uint32_t street, int end, int kerb) {
        const ModelStreet& S = m.streets[street];
        const ModelStreetId T = S.carriesOn[static_cast<std::size_t>(end)];
        if (!T.valid()) return ownEnd(street, end, kerb);
        const ModelStreet& TS = m.streets[T.index];
        int back = -1, named = 0;
        for (int e = 0; e < 2; ++e)
            if (TS.carriesOn[static_cast<std::size_t>(e)].index == street) {
                ++named;
                // (a ring of two streets on one road names its partner at both ends: the end at the same station)
                if (named == 1 || (TS.road == S.road && sameWrittenStation(e == 0 ? TS.from : TS.to, end == 0 ? S.from : S.to))) back = e;
            }
        if (back < 0 || (named == 2 && TS.road != S.road)) return ownEnd(street, end, kerb);
        return std::min(ownEnd(street, end, kerb), ownEnd(T.index, back, back != end ? kerb : -kerb));
    };
    std::map<std::pair<uint32_t, int>, uint32_t> deadEndAt;   // (street, end) -> its dead end
    for (uint32_t d = 0; d < m.deadEnds.size(); ++d) deadEndAt[std::make_pair(m.deadEnds[d].street.index, m.deadEnds[d].end)] = d;

    const auto addWalk = [&](const std::vector<Vec2>& pts, const std::vector<double>& z, double width, Walk k) {
        k.curve = G.addCurve(pts, z);
        k.length = G.along[k.curve.end() - 1];
        k.width = width;
        G.walks.push_back(k);
        return static_cast<uint32_t>(G.walks.size() - 1);
    };
    const auto meet = [&](const MeetKey& key, uint32_t walk, int end, const std::string& why) {
        meets[key].push_back(WalkEnd{WalkId(walk), static_cast<uint8_t>(end)});
        if (!why.empty()) whyAlone.emplace(key, why);
    };

    // ---- along the roads: each sidewalk lane (its kerb) and footway lane (kerb 0), street by street ------------------------
    const auto layAlong = [&](const ModelLane& L, Walk ids, int kerb, int deckLane) {
        if (!L.onRoad() || L.curve.count < 2) return;
        std::vector<Vec2> xy(m.lanePoints().begin() + L.curve.first, m.lanePoints().begin() + L.curve.end());
        std::vector<double> st(m.spineStations.begin() + L.curve.first, m.spineStations.begin() + L.curve.end());
        for (const double v : st)
            if (!stated(v)) return;   // (not laid beside its spine: the lane level leaves it out, as it would a car lane)
        if (st.front() > st.back()) {
            std::reverse(xy.begin(), xy.end());
            std::reverse(st.begin(), st.end());
        }
        const std::vector<double> s = stations(xy);
        for (const ModelLaneSpan& sp : L.streets) {
            const ModelStreet& S = m.streets[sp.street.index];
            double lo = sp.from, hi = sp.to;
            MeetKey keys[2];
            std::string why[2];
            for (int end = 0; end < 2; ++end) {
                keys[end] = ownEnd(sp.street.index, end, kerb);
                const LaneSpanEnd at = laneSpanEnd(m, sp, end);
                if (!at.reaches) {
                    why[end] = "the walk stops short of its street's end";
                    continue;
                }
                if (at.junction.valid()) {
                    if (at.arm < 0) {
                        why[end] = "its street's junction lists no arm for it";
                        continue;
                    }
                    const double cut = kerbEndStation(m, at.junction, static_cast<uint32_t>(at.arm));
                    if (end == 0) lo = std::max(lo, cut);
                    else hi = std::min(hi, cut);
                    keys[end] = kerbKey(at.junction, static_cast<uint32_t>(at.arm), kerb);
                    continue;
                }
                const StreetEndKind kind = streetEndKind(g, S, end);
                if (kind == StreetEndKind::CarriesOn) {
                    keys[end] = carryOnKey(sp.street.index, end, kerb);
                    why[end] = "the street it carries on into has no walk on that kerb";
                } else if (kind == StreetEndKind::DeadEnd) {
                    const std::map<std::pair<uint32_t, int>, uint32_t>::const_iterator d = deadEndAt.find(std::make_pair(sp.street.index, end));
                    const bool joined = d != deadEndAt.end() && m.deadEnds[d->second].turnRound;
                    if (joined) keys[end] = MeetKey(static_cast<uint8_t>(MeetAt::DeadEnd), d->second, 0, kerb);
                    why[end] = joined ? "a dead end with a sidewalk on one kerb only" : "a dead end not joined across";
                } else {
                    why[end] = streetEndKindWord(kind);
                }
            }
            if (!(hi - lo > kCutTolerance)) continue;
            const std::vector<Vec2> pts = sliceAtStations(xy, s, interp(st, s, lo), interp(st, s, hi), kSliceGap);
            if (pts.size() < 2) continue;
            std::vector<double> z;
            for (const Vec2& q : pts) z.push_back(deckLane >= 0 ? H.deck(deckLane, q) : H.deckRoad(static_cast<int>(L.edge), q));
            Walk k = ids;
            k.street = sp.street.index;
            k.stationFrom = lo;
            k.stationTo = hi;
            const uint32_t w = addWalk(pts, z, L.width, k);
            meet(keys[0], w, 0, why[0]);
            meet(keys[1], w, 1, why[1]);
        }
    };
    for (uint32_t wi = 0; wi < m.walks.size(); ++wi) {
        if (!m.walks[wi].onRoad()) continue;   // (a corner's walk: below)
        Walk ids;
        ids.modelWalk = wi;
        layAlong(m.walks[wi], ids, m.walks[wi].dir, -1);
    }
    for (uint32_t li = 0; li < m.lanes.size(); ++li) {
        const ModelLane& L = m.lanes[li];
        if (!L.onRoad() || !g.cls(g.edges[L.edge]).footway) continue;
        Walk ids;
        ids.modelLane = li;
        layAlong(L, ids, 0, static_cast<int>(li));
    }
    // ---- round the corners: each corner's walk whole, from arm A's kerb end to arm B's -------------------------------------
    for (const ModelCorner& c : m.corners) {
        if (c.walk >= m.walks.size()) continue;
        const ModelLane& W = m.walks[c.walk];
        const std::vector<Vec2> pts(m.lanePoints().begin() + W.curve.first, m.lanePoints().begin() + W.curve.end());
        const std::vector<double> z(m.heights.begin() + W.curve.first, m.heights.begin() + W.curve.end());
        if (pts.size() < 2) continue;
        Walk k;
        k.modelWalk = c.walk;
        const uint32_t w = addWalk(pts, z, W.width, k);
        meet(kerbKey(c.junction, c.armA, c.kerbA), w, 0, std::string());
        meet(kerbKey(c.junction, c.armB, c.kerbB), w, 1, std::string());
    }
    // ---- over the crosswalks people use: kerb end 0 (the +1 kerb's) to kerb end 1 -----------------------------------------
    for (uint32_t ci = 0; ci < m.crosswalks.size(); ++ci) {
        const ModelCrosswalk& c = m.crosswalks[ci];
        const std::map<std::string, int>::const_iterator road = g.index.find(c.road);
        if (!c.pavements || !c.junction.valid() || road == g.index.end()) continue;
        const std::vector<Vec2> pts = {c.ends[0], c.ends[1]};
        const std::vector<double> z = {H.deckRoad(road->second, c.ends[0]), H.deckRoad(road->second, c.ends[1])};
        Walk k;
        k.modelCrosswalk = ci;
        const uint32_t w = addWalk(pts, z, c.bandWidth(), k);
        meet(kerbKey(c.junction, c.arm, +1), w, 0, std::string());
        meet(kerbKey(c.junction, c.arm, -1), w, 1, std::string());
    }
    // ---- across the dead ends the model joins: from the +1 kerb's sidewalk end to the -1 kerb's ------------------------
    const auto endPoint = [&](const WalkEnd& e, double* z) {
        const Walk& k = G.walks[e.walk.index];
        const uint32_t i = e.end == 0 ? k.curve.first : k.curve.end() - 1;
        *z = G.heights[i];
        return G.points[i];
    };
    for (uint32_t d = 0; d < m.deadEnds.size(); ++d) {
        if (!m.deadEnds[d].turnRound) continue;
        const MeetKey a(static_cast<uint8_t>(MeetAt::DeadEnd), d, 0, +1), b(static_cast<uint8_t>(MeetAt::DeadEnd), d, 0, -1);
        const std::map<MeetKey, std::vector<WalkEnd>>::const_iterator ia = meets.find(a), ib = meets.find(b);
        if (ia == meets.end() || ib == meets.end()) continue;   // (one kerb's sidewalk alone: said below)
        std::vector<double> z(2, 0.0);
        const std::vector<Vec2> pts = {endPoint(ia->second.front(), &z[0]), endPoint(ib->second.front(), &z[1])};
        const ModelStreet& S = m.streets[m.deadEnds[d].street.index];
        Walk k;
        k.modelDeadEnd = d;
        const uint32_t w = addWalk(pts, z, S.section.sidewalk, k);
        meet(a, w, 0, std::string());
        meet(b, w, 1, std::string());
    }

    // ---- the joins: every walk end at a place meets every other there; a lone end is listed with why -----------------
    std::vector<std::vector<WalkEnd>> joins(G.walks.size() * 2);
    for (const std::pair<const MeetKey, std::vector<WalkEnd>>& place : meets) {
        const std::vector<WalkEnd>& ends = place.second;
        if (ends.size() == 1) {
            LaneGraphBuild::WalkEndAlone alone;
            alone.walk = ends.front().walk.index;
            alone.end = ends.front().end;
            double z = 0;
            alone.at = endPoint(ends.front(), &z);
            const MeetKey& key = place.first;
            if (std::get<0>(key) == static_cast<uint8_t>(MeetAt::Kerb)) {
                // at a junction's kerb end: what is missing there (a corner's walk the build could not lay is ADR-0465's)
                const ModelJunctionId J(std::get<1>(key));
                const uint32_t arm = std::get<2>(key);
                const int kerb = std::get<3>(key);
                bool cornerStated = false;
                for (const ModelCorner& c : m.corners)
                    if (c.junction == J && ((c.armA == arm && kerbKey(J, arm, c.kerbA) == key) || (c.armB == arm && kerbKey(J, arm, c.kerbB) == key)))
                        cornerStated = true;
                // the ring pair at this kerb end the build tried and laid no walk for, and why (walk_lanes.h): a stated
                // corner, or a spread junction's pair, which becomes a corner only where a walk was laid -- so its end is
                // never "no corner stated" (ADR-0469: those were counted as freeway arms)
                const WalkCorner* failed = nullptr;
                for (const WalkCorner& wc : r.walkCorners)
                    if (wc.lane < 0 && wc.junction == J &&
                        ((wc.armA == arm && kerbKey(J, arm, wc.kerbA) == key) || (wc.armB == arm && kerbKey(J, arm, wc.kerbB) == key)))
                        failed = &wc;
                const ModelCrosswalk* cw = m.crosswalkAcross(J, arm);
                const bool crossed = cw && cw->pavements;
                const Walk& k = G.walks[alone.walk];
                if (k.modelCrosswalk != kNoIndex) {
                    alone.why = "a crosswalk's kerb end with no other walk on its kerb";
                } else if (k.modelWalk != kNoIndex && !m.walks[k.modelWalk].onRoad()) {
                    alone.why = "a corner walk's end with no sidewalk or crossing there";
                } else {
                    alone.why = failed       ? std::string("its corner has no walk (") + walkCornerFaultWord(failed->fault) + ")"
                                : cornerStated ? "its corner has no walk (the build could not lay one)"
                                               : "no corner is stated on this kerb";
                    if (!crossed) alone.why += ", and no crosswalk people use";
                }
                alone.where = kerb == 0 ? fmt("junction %s, arm %u (a footway)", m.junctions[J.index].id.c_str(), arm)
                                        : fmt("junction %s, arm %u, kerb %+d", m.junctions[J.index].id.c_str(), arm, kerb);
            } else {
                const std::map<MeetKey, std::string>::const_iterator why = whyAlone.find(key);
                alone.why = why != whyAlone.end() ? why->second : std::string("nothing else is stated there");
                const uint32_t street = G.walks[alone.walk].street;
                alone.where = street < m.streets.size() ? fmt("street %s, its %s", m.streets[street].id.c_str(), alone.end == 0 ? "start" : "end")
                                                        : std::string();
            }
            out.walkEndsAlone.push_back(alone);
            continue;
        }
        for (const WalkEnd& a : ends)
            for (const WalkEnd& b : ends)
                if (!(a == b)) joins[a.walk.index * 2 + a.end].push_back(b);
    }
    for (uint32_t w = 0; w < G.walks.size(); ++w)
        for (int end = 0; end < 2; ++end) {
            const std::vector<WalkEnd>& j = joins[w * 2 + static_cast<uint32_t>(end)];
            const Range r{static_cast<uint32_t>(G.walkJoins.size()), static_cast<uint32_t>(j.size())};
            G.walkJoins.insert(G.walkJoins.end(), j.begin(), j.end());
            (end == 0 ? G.walks[w].atStart : G.walks[w].atEnd) = r;
        }
}

}  // namespace

LaneGraphBuild buildLaneLevel(const Result& r, RoadModel& model, const LaneGraphOptions& opts) {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    LaneGraphBuild out;
    model.laneLevel = LaneGraph();
    LaneGraph& G = model.laneLevel;
    const RoadModel& m = model;   // everything else of the model is only read
    const RoadLabGraph& g = r.graph;
    const DeckHeight& H = *r.heights;
    // (The lanes are read from the model alone, issue #292: the builder's lanes give only their count, which the model's
    // must match, and the drawn deck's height under a lane, asked by the same index.)
    const std::size_t nLanes = r.lanes.lanes.size();

    // ---- the facts the lane level cannot be built without ------------------------------------------------------------
    out.error = laneLevelRefusal(g, m, m.built && m.lanes.size() == nLanes);
    if (!out.error.empty()) return out;
    out.leftOut.assign(nLanes, 0);
    out.laneLengths.assign(nLanes, 0.0);
    auto edgeAt = [&](uint32_t e) -> const EdgeSpec& { return g.edges[e]; };
    auto uOf = [&](uint32_t lane, double station) {   // a spine station along a lane's corridor
        const ModelLane& ml = m.lanes[lane];
        return ml.dir > 0 ? station : edgeAt(ml.edge).s.back() - station;
    };
    auto stationOf = [&](uint32_t lane, double u) {   // ...and back: the spine station at a place along its corridor
        const ModelLane& ml = m.lanes[lane];
        return ml.dir > 0 ? u : edgeAt(ml.edge).s.back() - u;
    };

    // ---- which lanes become lane paths, and the names -----------------------------------------------------------------
    std::vector<char> kept(nLanes, 0);
    for (std::size_t li = 0; li < nLanes; ++li) {
        const ModelLane& ml = m.lanes[li];
        if (ml.edge == kNoIndex) {   // a lane of no road: a connector the road file declares
            out.leftOut[li] = 1;
            ++out.connectorsLeftOut;
            continue;
        }
        const bool carsDrive = g.cls(edgeAt(ml.edge)).carriesCars();
        if (!carsDrive || ml.curve.count < 2 || !(m.laneAlong()[ml.curve.end() - 1] > 0)) {
            out.leftOut[li] = 1;
            ++out.otherLanesLeftOut;
            continue;
        }
        kept[li] = 1;
    }
    for (const VehicleSize& v : opts.vehicles) G.vehicleTagNames.push_back(v.tag);
    std::sort(G.vehicleTagNames.begin(), G.vehicleTagNames.end());
    G.vehicleTagNames.erase(std::unique(G.vehicleTagNames.begin(), G.vehicleTagNames.end()), G.vehicleTagNames.end());
    TagMask everyone = 0;
    for (std::size_t i = 0; i < G.vehicleTagNames.size() && i < kMaxTagNames; ++i) everyone |= TagMask(1) << i;

    // ---- every kept lane the way its traffic runs, each point at its stated spine station ----------------------------
    std::vector<TravelLane> travel(nLanes);
    std::map<uint32_t, std::vector<Vec2>> spineTangents;   // per road, as the builder framed it
    for (std::size_t li = 0; li < nLanes; ++li) {
        if (!kept[li]) continue;
        const ModelLane& ml = m.lanes[li];   // the model's lane: the one lane the lane level reads (issue #292)
        const EdgeSpec& e = edgeAt(ml.edge);
        TravelLane& T = travel[li];
        const std::size_t n = ml.curve.count;
        bool laid = true;
        for (std::size_t k = 0; k < n && laid; ++k) laid = stated(m.spineStations[ml.curve.first + k]);
        if (!laid) {
            out.problems.push_back(RoadProblem(RoadProblemKind::LaneNotBesideSpine).onLane(ml.id));
            kept[li] = 0;
            out.leftOut[li] = 1;
            ++out.drivenLanesLeftOut;
            continue;
        }
        std::vector<Vec2>& tangent = spineTangents[ml.edge];
        if (tangent.empty()) {
            std::vector<Vec2> normal;
            frames(e.xy, tangent, normal);
        }
        // THE LINE DRIVEN (ADR-0450): the lane's own points, but a lane the kerb band overlaps
        const std::vector<Vec2> driven = drivenLanePoints(g, m, static_cast<uint32_t>(li), &T.width);
        std::vector<Vec2> forward;   // the way its road runs at each point, the way its traffic runs
        double back = 0;
        Vec2 backAt;
        bool sawBack = false;
        for (std::size_t k = 0; k < n; ++k) {
            const std::size_t idx = ml.dir > 0 ? k : n - 1 - k;
            const double st = m.spineStations[ml.curve.first + idx];
            T.xy.push_back(driven[idx]);
            T.u.push_back(ml.dir > 0 ? st : e.s.back() - st);
            const std::size_t i = static_cast<std::size_t>(std::lower_bound(e.s.begin(), e.s.end(), st - kCutTolerance) - e.s.begin());
            forward.push_back(tangent[std::min(i, tangent.size() - 1)] * static_cast<double>(ml.dir));
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
            // (numbers: how far it ran backwards, how much shorter the tidied lane is)
            out.problems.push_back(RoadProblem(RoadProblemKind::LaneFoldTidied).onLane(ml.id).onRoad(e.id).where(backAt).number(back)
                                       .number(before - T.s.back()));
        }
        for (std::size_t k = 1; k < T.u.size(); ++k) T.u[k] = std::max(T.u[k], T.u[k - 1]);
        out.laneLengths[li] = T.s.back();        T.ok = T.xy.size() >= 2 && T.s.back() > 0;
        if (!T.ok) {
            out.problems.push_back(RoadProblem(RoadProblemKind::LaneNothingLeft).onLane(ml.id));
            kept[li] = 0;
            out.leftOut[li] = 1;
            ++out.drivenLanesLeftOut;
        }
    }

    // ---- each arm's stop line: the stated junction's (ModelArm::stopLine, deck_mesh.h junctionAreas -- the one rule the
    // paint draws from too) -------------------------------------------------------------------------------------------------
    for (uint32_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        for (uint32_t a = 0; a < J.arms.size(); ++a) {
            const ModelArm& arm = J.arms[a];
            if (arm.street.valid() && !stated(arm.stopLine))
                out.problems.push_back(RoadProblem(RoadProblemKind::ArmNoStopLine).atJunction(J.id).onArm(a).onStreet(m.streets[arm.street.index].id));
        }
    }

    // ---- each lane's stretches along its streets, stop line to stop line ------------------------------------------------
    std::vector<Piece> pieces;
    std::vector<std::vector<uint32_t>> piecesOfLane(nLanes);
    // per lane: each stretch of it left out of its lane paths, recorded where it is left out, with its own cause -- a
    // junction (between its street's end there and the arm's stop line) or a reason in words (also a problem line); along
    // its corridor (u)
    struct Dropped {
        double u0 = 0, u1 = 0;
        int junction = -1;
        std::string why;
    };
    std::vector<std::vector<Dropped>> dropped(nLanes);
    auto drop = [&](uint32_t li, double stationA, double stationB, int junction, const std::string& why) {
        Dropped d;
        d.u0 = std::min(uOf(li, stationA), uOf(li, stationB));
        d.u1 = std::max(uOf(li, stationA), uOf(li, stationB));
        d.junction = junction;
        d.why = why;
        if (d.u1 - d.u0 > kCutTolerance) dropped[li].push_back(d);
    };
    std::vector<int> insideJunction(nLanes, -1);   // per lane: the junction it lies wholly inside (a stated inside stretch)
    for (uint32_t li = 0; li < nLanes; ++li) {
        if (!kept[li]) continue;
        const ModelLane& ml = m.lanes[li];
        std::vector<ModelLaneSpan> spans = ml.streets;
        if (ml.dir < 0) std::reverse(spans.begin(), spans.end());
        for (const ModelLaneSpan& sp : spans) {
            const ModelStreet& S = m.streets[sp.street.index];
            // (a street's stations are written to the centimetre: a lane reaching its end runs to its own end)
            double lo = atOrBefore(sp.from, ml.spineFrom, kWrittenStation) ? ml.spineFrom : sp.from;
            double hi = atOrPast(sp.to, ml.spineTo, kWrittenStation) ? ml.spineTo : sp.to;
            const double spanLo = lo, spanHi = hi;   // its whole stretch on this street, before the stop lines trim it
            int jLo = -1, aLo = -1, jHi = -1, aHi = -1, endLo = -1, endHi = -1;
            bool noStopLo = false, noStopHi = false;
            for (int end = 0; end < 2; ++end) {
                // which arm it meets there and its stop line: the model's one reading (road_model.h laneSpanEnd)
                const LaneSpanEnd met = laneSpanEnd(m, sp, end);
                if (!met.reaches) continue;   // the lane does not reach it
                const ModelJunctionId J = met.junction;
                if (!J.valid()) {
                    (end == 0 ? endLo : endHi) = end;
                    continue;
                }
                const int a = met.arm;
                if (a < 0 || !stated(met.stopLine)) {
                    out.problems.push_back(RoadProblem(RoadProblemKind::StreetEndNoStopLine).onStreet(S.id).word(end == 0 ? "`from`" : "`to`")
                                               .atJunction(m.junctions[J.index].id).onLane(m.lanes[li].id));
                    (end == 0 ? noStopLo : noStopHi) = true;
                    continue;
                }
                const double stop = met.stopLine;
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
                // (numbers: the two stop lines' stations)
                out.problems.push_back(RoadProblem(RoadProblemKind::NoRoomBetweenStopLines).onLane(m.lanes[li].id).onStreet(S.id).onRoad(S.road)
                                           .number(lo).number(hi));
                drop(li, spanLo, spanHi, -1, problemLine(out.problems.back()));
                continue;
            }
            // what the stop lines trim off its ends lies inside those junctions
            if (jLo >= 0) drop(li, spanLo, lo, jLo, std::string());
            if (jHi >= 0) drop(li, hi, spanHi, jHi, std::string());
            Piece p;
            p.lane = li;
            p.street = sp.street.index;
            p.u0 = uOf(li, ml.dir > 0 ? lo : hi);
            p.u1 = uOf(li, ml.dir > 0 ? hi : lo);
            if (ml.dir > 0) {
                p.startJunction = jLo, p.startArm = aLo, p.startStreetEnd = endLo, p.startNoStop = noStopLo;
                p.endJunction = jHi, p.endArm = aHi, p.endStreetEnd = endHi, p.endNoStop = noStopHi;
            } else {
                p.startJunction = jHi, p.startArm = aHi, p.startStreetEnd = endHi, p.startNoStop = noStopHi;
                p.endJunction = jLo, p.endArm = aLo, p.endStreetEnd = endLo, p.endNoStop = noStopLo;
            }
            piecesOfLane[li].push_back(static_cast<uint32_t>(pieces.size()));
            pieces.push_back(std::move(p));
        }
        // what of it lies on a stretch of road the road file states inside a junction (no street there: the junction's
        // connectors cross it) is inside that junction
        for (uint32_t j = 0; j < m.junctions.size(); ++j)
            for (const ModelStretch& st : m.junctions[j].inside)
                if (st.road == edgeAt(ml.edge).id && std::min(st.to, ml.spineTo) > std::max(st.from, ml.spineFrom))
                    drop(li, std::max(st.from, ml.spineFrom), std::min(st.to, ml.spineTo), static_cast<int>(j), std::string());
        if (!piecesOfLane[li].empty()) continue;
        // nothing of it between stop lines on any of its streets: a lane of a stretch of road the road file states inside a
        // junction (the junction's connectors cross it: no lane path, as stated), or else no lane path at all, said
        for (uint32_t j = 0; j < m.junctions.size() && insideJunction[li] < 0; ++j)
            for (const ModelStretch& st : m.junctions[j].inside)
                if (st.road == edgeAt(ml.edge).id && atOrBefore(st.from, ml.spineFrom, kWrittenStation) && atOrPast(st.to, ml.spineTo, kWrittenStation))
                    insideJunction[li] = static_cast<int>(j);
        if (insideJunction[li] >= 0) continue;
        bool said = false;   // (a street with no room between its stop lines says so itself)
        for (const Dropped& d : dropped[li]) said = said || !d.why.empty();
        if (!said)
            out.problems.push_back(RoadProblem(RoadProblemKind::LaneOnNoStreet).onLane(m.lanes[li].id));
        out.leftOut[li] = 1;
        ++out.drivenLanesLeftOut;
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
            sec.lanes.first = static_cast<uint32_t>(G.sectionLanes.size());
            sec.lines.first = static_cast<uint32_t>(G.laneLines.size());
            for (std::size_t k = 0; k < in.size(); ++k) {
                Piece& pc = pieces[in[k]];
                const uint32_t li = pc.lane;
                const TravelLane& T = travel[li];
                const double ta = tauAt(T, c0), tb = tauAt(T, c1);
                const std::vector<Vec2> pts = sliceAtStations(T.xy, T.s, ta, tb, kSliceGap);
                std::vector<double> z;
                for (const Vec2& q : pts) z.push_back(H.deck(static_cast<int>(li), q));
                Path p;
                p.kind = PathKind::Lane;
                p.vehicleTags = everyone;
                p.curve = G.addCurve(pts, z);
                p.length = G.along[p.curve.end() - 1];
                p.width = T.width;
                p.section = sid;
                p.indexInSection = static_cast<uint32_t>(k);
                p.street = pc.street;
                p.modelLane = li;
                // its stretch of its road's spine, the way it is driven (the corridor runs the way the traffic does)
                p.onSpine = true;
                p.stationFrom = stationOf(li, c0);
                p.stationTo = stationOf(li, c1);
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
            // the lines between neighbours: the one rule (lane_turns.h laneLineBetween, which the paint draws from too) --
            // solid between two lanes that hold their lane, on the approach to the stop line both run to
            for (std::size_t k = 0; k + 1 < in.size(); ++k) {
                LaneBesideLine beside[2];
                for (std::size_t side = 0; side < 2; ++side) {
                    const Piece& pc = pieces[in[k + side]];
                    beside[side].kind = m.lanes[pc.lane].kind;
                    beside[side].runsToStopLine = pc.endJunction >= 0;
                    beside[side].stopLine = pc.u1;
                }
                G.laneLines.push_back(laneLineBetween(laneTurnRules(), solid, beside[0], beside[1], c0, c1));
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
            // (never across a street's end inside a junction with no stop line for it: said where the pieces were cut)
            if (k + 1 < ps.size() && !pc.paths.empty() && !pieces[ps[k + 1]].paths.empty() && !pc.endNoStop &&
                !pieces[ps[k + 1]].startNoStop && std::fabs(pieces[ps[k + 1]].u0 - pc.u1) <= kCutTolerance)
                next[pc.paths.back()].push_back(PathId(pieces[ps[k + 1]].paths.front()));
        }
    }
    // A STREET'S END WITH NO JUNCTION AND NOTHING IT CARRIES ON INTO: a ramp's anchored end (its merge or diverge, below),
    // a road that just ends (nothing else meets it there: a true dead end), or a road meeting others where the road file
    // states no junction -- reported, once per street end, and never joined (the planner's to state)
    std::set<std::pair<uint32_t, int>> unstatedMeetings;
    auto unstatedEnd = [&](const Piece& pc, int streetEnd) {
        const ModelStreet& S = m.streets[pc.street];
        // (what is at the street's end: the model's one answer, road_model.h streetEndKind, ADR-0464)
        const StreetEndKind kind = streetEndKind(g, S, streetEnd);
        if (kind != StreetEndKind::MeetsAnother && kind != StreetEndKind::PartWay) return;
        if (!unstatedMeetings.insert(std::make_pair(pc.street, streetEnd)).second) return;
        const EdgeSpec& e = edgeAt(S.edge);
        const Vec2 p = pointAt(e.xy, e.s, streetEnd == 0 ? S.from : S.to);
        out.problems.push_back(RoadProblem(RoadProblemKind::StreetEndUnstated).onStreet(S.id).word(streetEnd == 0 ? "`from`" : "`to`").where(p)
                                   .word(kind == StreetEndKind::PartWay ? "lies part way along its road" : "meets another road"));
    };
    for (const Piece& pc : pieces)
        if (pc.startStreetEnd >= 0 && !pc.paths.empty()) unstatedEnd(pc, pc.startStreetEnd);
    // where a street carries on as another (no junction): into the lane of the same slot on the next street
    for (const Piece& pc : pieces) {
        if (pc.endStreetEnd < 0 || pc.paths.empty()) continue;
        const ModelStreet& S = m.streets[pc.street];
        const ModelStreetId T = S.carriesOn[static_cast<std::size_t>(pc.endStreetEnd)];
        if (!T.valid()) {   // a dead end, a ramp's anchored end (its merge, below), or roads meeting with no junction stated
            unstatedEnd(pc, pc.endStreetEnd);
            continue;
        }
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
            out.problems.push_back(RoadProblem(RoadProblemKind::CarryOnNotNamedBack).onStreet(S.id).andStreet(TS.id));
            continue;
        }
        if (backs.size() != 1) {
            out.problems.push_back(RoadProblem(RoadProblemKind::CarryOnAmbiguous).onStreet(S.id).andStreet(TS.id).onLane(m.lanes[pc.lane].id)
                                       .word(backs.empty() ? "where no lane leaves" : "which names it back at both ends with lanes leaving each"));
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
            out.problems.push_back(RoadProblem(RoadProblemKind::CarryOnNoLaneLeaves).onStreet(S.id).andStreet(TS.id).onLane(m.lanes[pc.lane].id));
            continue;
        }
        const Piece& into = pieces[static_cast<std::size_t>(best)];
        if (m.lanes[into.lane].slot != slot)
            // (numbers: the lane's slot, and the slot of the lane it leads into)
            out.problems.push_back(RoadProblem(RoadProblemKind::CarryOnOtherSlot).onLane(m.lanes[pc.lane].id).onStreet(S.id).andStreet(TS.id)
                                       .andLane(m.lanes[into.lane].id).number(slot).number(m.lanes[into.lane].slot));
        next[pc.paths.back()].push_back(PathId(into.paths.front()));
        ++out.carryOnJoins;
        const double gap = joinGap(pc.paths.back(), into.paths.front());
        if (gap > kJoinReported) {
            ++out.joinsWithGap;
            const Vec2 at = G.pointAt(PathId(pc.paths.back()), G.paths[pc.paths.back()].length);
            out.problems.push_back(RoadProblem(RoadProblemKind::CarryOnGap).onLane(m.lanes[pc.lane].id).onStreet(S.id).andLane(m.lanes[into.lane].id)
                                       .andStreet(TS.id).where(at).number(gap));
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
    // the stated merges made: the joining lane's last path, the host lane, the connection (marked on their Join, below)
    struct MadeMerge {
        uint32_t joining, hostLane, branchLane, connection;
    };
    std::vector<MadeMerge> merges;
    for (uint32_t ci = 0; ci < m.connections.size(); ++ci) {
        const ModelConnection& c = m.connections[ci];
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
            // (words: leaves or merges into; which of the two lanes has no lane path; what it lacks one to do)
            out.problems.push_back(RoadProblem(RoadProblemKind::ConnectionNotJoined).onLane(m.lanes[branch].id).word(diverge ? "leaves" : "merges into")
                                       .andLane(m.lanes[host].id).atStation(c.meets).onRoad(edgeAt(m.lanes[host].edge).id)
                                       .word(!reaches ? m.lanes[branch].id : m.lanes[host].id)
                                       .word(!reaches ? (diverge ? "start from" : "end at") : "meet it (inside a junction, or past its lane)"));
            continue;
        }
        const uint32_t from = diverge ? static_cast<uint32_t>(hp) : pieces[bp.back()].paths.back();
        const uint32_t into = diverge ? pieces[bp.front()].paths.front() : static_cast<uint32_t>(hp);
        next[from].push_back(PathId(into));
        ++(diverge ? out.divergeJoins : out.mergeJoins);
        if (!diverge) merges.push_back(MadeMerge{from, host, branch, ci});
        const double gap = joinGap(from, into);
        if (gap > kJoinReported) {
            ++out.joinsWithGap;
            const Vec2 at = G.pointAt(PathId(into), 0);
            out.problems.push_back(RoadProblem(RoadProblemKind::ConnectionGap).onLane(m.lanes[branch].id).word(diverge ? "leaves" : "merges into")
                                       .andLane(m.lanes[host].id).where(at).number(gap));
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
    const std::chrono::steady_clock::time_point tConnect = std::chrono::steady_clock::now();
    // the stated turns of each junction, in the model's order (every one placed: one naming no junction is said)
    std::vector<std::vector<uint32_t>> turnsAt(m.junctions.size());
    std::size_t turnsPlaced = 0;   // stated turns built, refused, or at a junction no lane path reaches: all of them, checked
    for (uint32_t t = 0; t < m.turns.size(); ++t) {
        if (m.turns[t].junction.index < m.junctions.size()) {
            turnsAt[m.turns[t].junction.index].push_back(t);
            continue;
        }
        out.problems.push_back(RoadProblem(RoadProblemKind::TurnNoJunction).number(t));
        ++turnsPlaced;
    }
    for (uint32_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        if (arrivingAt[j].empty() && leavingAt[j].empty()) {
            if (!turnsAt[j].empty())
                out.problems.push_back(RoadProblem(RoadProblemKind::JunctionNoLanePath).atJunction(J.id).number(static_cast<Real>(turnsAt[j].size())));
            turnsPlaced += turnsAt[j].size();
            continue;
        }
        const uint32_t lj = static_cast<uint32_t>(G.junctions.size());
        laneJunctionOf[j] = static_cast<int>(lj);
        Junction jn;
        jn.modelJunction = j;
        jn.crossings = Range{static_cast<uint32_t>(G.crossings.size()), 0};
        jn.crosswalks = Range{static_cast<uint32_t>(G.crosswalks.size()), 0};
        // its outline: across each arm's lanes at its stop line, the arms' lines joined in the stated ring order (below)
        {
            struct Leg { Vec2 a, b, mid; };
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
                        // the arm's way across: the arriving traffic's counter-clockwise normal (the leaving traffic runs the
                        // other way)
                        across += perp(G.directionAt(PathId(pa.first), at)) * (arriving ? 1.0 : -1.0);
                    }
                if (ends.empty()) continue;
                if (!(across.length() > 0)) {   // its arriving and leaving lanes' ways across cancel out: cannot be drawn
                    out.problems.push_back(RoadProblem(RoadProblemKind::ArmWaysCancel).atJunction(J.id).onArm(a).onRoad(J.arms[a].road));
                    continue;
                }
                across = normalize(across);
                std::sort(ends.begin(), ends.end(), [&](const std::pair<Vec2, double>& x, const std::pair<Vec2, double>& y) {
                    return dot(x.first, across) < dot(y.first, across);
                });
                Leg leg;
                leg.a = ends.front().first - across * ends.front().second;
                leg.b = ends.back().first + across * ends.back().second;
                leg.mid = (leg.a + leg.b) * 0.5;
                legs.push_back(leg);
                for (const std::pair<Vec2, double>& e : ends) all.push_back(e.first);
                all.push_back(leg.a);
                all.push_back(leg.b);
            }
            std::vector<Vec2> ring;
            if (legs.size() < 2) {
                ring = convexHull(all);
            } else {
                // drawn round the legs in the junction's stated ring order (its arms' list: counter-clockwise, road_layout.h
                // armRing, ADR-0427), nothing sorted here. `across` is the arriving lanes' counter-clockwise normal (perp of
                // their way in), which points clockwise round the junction from the arm, so a leg's `b` end (furthest along
                // it) is its clockwise end: going counter-clockwise round the junction, each leg is drawn from `b` to `a`.
                // (Plan geometry: which side the kerb is does not enter.)
                for (const Leg& leg : legs) {
                    ring.push_back(leg.b);
                    ring.push_back(leg.a);
                }
            }
            jn.box = G.addPlanPoints(ring);
            if (ring.size() < 3)
                out.problems.push_back(RoadProblem(RoadProblemKind::OutlineFewCorners).atJunction(J.id).where(J.at).number(static_cast<Real>(ring.size())));
            bool tangled = false;
            for (std::size_t a = 0; a < ring.size() && !tangled; ++a)
                for (std::size_t b = a + 2; b < ring.size() && !tangled; ++b) {
                    if ((b + 1) % ring.size() == a) continue;   // neighbours share a corner
                    Vec2 hit;
                    tangled = segmentsCross(ring[a], ring[(a + 1) % ring.size()], ring[b], ring[(b + 1) % ring.size()], hit);
                }
            if (tangled)
                out.problems.push_back(RoadProblem(RoadProblemKind::OutlineCrossesItself).atJunction(J.id).where(J.at));
        }
        std::vector<uint32_t> arriving, leaving;
        for (const std::pair<uint32_t, uint32_t>& pa : arrivingAt[j]) {
            arriving.push_back(pa.first);
            // the arm it arrives by: what the model is asked by (its signs, which approaches face each other)
            G.paths[pa.first].arm = pa.second;
        }
        for (const std::pair<uint32_t, uint32_t>& pa : leavingAt[j]) leaving.push_back(pa.first);
        // a sign the file states on an arm no lane arrives by: impossible, reported (never stood elsewhere)
        for (uint32_t a = 0; a < J.arms.size(); ++a) {
            if (J.arms[a].regulations.empty()) continue;
            bool any = false;
            for (const std::pair<uint32_t, uint32_t>& pa : arrivingAt[j]) any = any || pa.second == a;
            if (!any)
                out.problems.push_back(RoadProblem(RoadProblemKind::SignOnArmNoLane).atJunction(J.id).onArm(a).onRoad(J.arms[a].road)
                                           .word(J.arms[a].regulations.front()));
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
        for (uint32_t t : turnsAt[j]) {
            const ModelTurn& mt = m.turns[t];
            ++note.turnsStated;
            ++turnsPlaced;
            int from = -1, into = -1;
            for (const std::pair<uint32_t, uint32_t>& pa : arrivingAt[j])
                if (pa.second == mt.fromArm && G.paths[pa.first].modelLane == mt.from.index) from = static_cast<int>(pa.first);
            for (const std::pair<uint32_t, uint32_t>& pa : leavingAt[j])
                if (pa.second == mt.toArm && G.paths[pa.first].modelLane == mt.to.index) into = static_cast<int>(pa.first);
            if (from < 0 || into < 0) {   // no lane path to take it: the lanes stop short of the junction, no corner's fault
                Refusal ref;
                ref.junction = lj;
                ref.from = from < 0 ? PathId() : PathId(static_cast<uint32_t>(from));
                ref.to = into < 0 ? PathId() : PathId(static_cast<uint32_t>(into));
                ref.turn = mt.turn;
                ref.why = RefusalWhy::NoLanePath;
                ref.modelTurn = t;
                ref.at = J.at;
                out.refusals.push_back(ref);
                out.problems.push_back(RoadProblem(RoadProblemKind::TurnNoLanePath).atJunction(J.id).word(turnKey(mt.turn)).onLane(m.lanes[mt.from.index].id)
                                           .andLane(m.lanes[mt.to.index].id).word(from < 0 ? "to start from" : "to end on").where(J.at));
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
                // closed to every class at once, with no per-class reason above: the fault for every class
                if (fit.noArc || tagCount == 0 || fit.noSharedVehicle) {
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
                ref.why = RefusalWhy::FitsNoVehicle;
                ref.modelTurn = t;
                ref.at = fit.at;
                out.refusals.push_back(ref);
                // (words: the turn's kind, and the corner fit's own reason)
                out.problems.push_back(RoadProblem(RoadProblemKind::TurnFitsNoVehicle).atJunction(J.id).word(turnKey(mt.turn)).onLane(m.lanes[mt.from.index].id)
                                           .andLane(m.lanes[mt.to.index].id).word(fit.why).where(fit.at));
                ++note.turnsRefused;
                continue;
            }
            const Path& fromPath = G.paths[static_cast<std::size_t>(from)];
            const Path& intoPath = G.paths[static_cast<std::size_t>(into)];
            Path c;
            c.kind = PathKind::Connector;
            c.vehicleTags = fit.open;
            c.curve = G.addCurve(fit.shape.points, fit.z);
            c.length = G.along[c.curve.end() - 1];
            c.width = std::min(fromPath.width, intoPath.width);
            c.tightestRadius = fit.shape.tightestRadius;
            c.junction = JunctionId(lj);
            c.from = PathId(static_cast<uint32_t>(from));
            c.modelTurn = t;
            const PathId id(static_cast<uint32_t>(G.paths.size()));
            G.paths.push_back(c);
            out.sources.push_back(PathSource());
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
    if (turnsPlaced != m.turns.size())
        out.problems.push_back(RoadProblem(RoadProblemKind::TurnsNotAllPlaced).number(static_cast<Real>(m.turns.size()))
                                   .number(static_cast<Real>(turnsPlaced)));

    // ---- every stretch of a kept lane no lane path covers, and why (inside a junction, or said) ------------------------
    // Each stretch no lane path covers is split by the causes recorded where the stretches were left out (a junction's
    // trim at a stop line, a street with no room between its stop lines): each part its own cause. A part nothing
    // recorded is said in its own problem line.
    {
        for (uint32_t li = 0; li < nLanes; ++li) {
            if (out.leftOut[li] || !kept[li]) continue;
            if (insideJunction[li] >= 0) {   // wholly inside a junction, as stated
                LaneGap gp;
                gp.lane = li;
                gp.to = out.laneLengths[li];
                gp.junction = static_cast<uint32_t>(insideJunction[li]);
                out.laneGaps.push_back(gp);
                continue;
            }
            const TravelLane& T = travel[li];
            std::vector<std::pair<double, double>> covered;
            for (uint32_t q : piecesOfLane[li])
                for (uint32_t p : pieces[q].paths) covered.emplace_back(out.sources[p].from, out.sources[p].to);
            std::sort(covered.begin(), covered.end());
            // the recorded causes, along the lane (metres the way cars drive it)
            struct Cause {
                double from, to;
                const Dropped* d;
            };
            std::vector<Cause> causes;
            for (const Dropped& d : dropped[li]) causes.push_back(Cause{tauAt(T, d.u0), tauAt(T, d.u1), &d});
            std::sort(causes.begin(), causes.end(), [](const Cause& a, const Cause& b) { return a.from < b.from; });
            auto unexplained = [&](double from, double to) {
                if (!(to - from > kCutTolerance)) return;
                LaneGap gp;
                gp.lane = li;
                gp.from = from;
                gp.to = to;
                if (to - from <= kStationRounding) {
                    // two of its streets' stated stations a centimetre or so apart (the road file writes them to the
                    // centimetre), with no junction beside the hole: said, so no level hides one
                    // (numbers: the hole's length, and how far along the lane it is)
                    out.problems.push_back(RoadProblem(RoadProblemKind::LaneHoleAtStations).onLane(m.lanes[li].id).number(to - from).number(from));
                } else {
                    // (numbers: from and to, metres along the lane)
                    out.problems.push_back(RoadProblem(RoadProblemKind::LaneUncovered).onLane(m.lanes[li].id).number(from).number(to));
                }
                gp.why = problemLine(out.problems.back());
                out.laneGaps.push_back(gp);
            };
            // (a hole of a centimetre or two between two causes, or between a cause and a lane path, is the road file's
            // stations written to the centimetre: it goes with the junction beside it)
            auto gap = [&](double from, double to) {
                if (!(to - from > kCutTolerance)) return;
                double at = from;
                int last = -1;   // the gap pushed last for this stretch
                for (const Cause& c : causes) {
                    const double a = std::max(at, c.from), b = std::min(to, c.to);
                    if (!(b - a > kCutTolerance)) continue;
                    double start = a;
                    if (a - at <= kStationRounding && c.d->junction >= 0) start = at;
                    else if (a - at <= kStationRounding && last >= 0 && out.laneGaps[static_cast<std::size_t>(last)].junction != kNoIndex)
                        out.laneGaps[static_cast<std::size_t>(last)].to = a;
                    else unexplained(at, a);
                    LaneGap gp;
                    gp.lane = li;
                    gp.from = start;
                    gp.to = b;
                    if (c.d->junction >= 0) gp.junction = static_cast<uint32_t>(c.d->junction);
                    gp.why = c.d->why;
                    out.laneGaps.push_back(gp);
                    last = static_cast<int>(out.laneGaps.size()) - 1;
                    at = b;
                }
                if (to - at <= kStationRounding && last >= 0 && out.laneGaps[static_cast<std::size_t>(last)].junction != kNoIndex)
                    out.laneGaps[static_cast<std::size_t>(last)].to = to;
                else unexplained(at, to);
            };
            double reached = 0;
            for (const std::pair<double, double>& c : covered) {
                gap(reached, c.first);
                reached = std::max(reached, c.second);
            }
            gap(reached, out.laneLengths[li]);
        }
    }

    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        std::sort(next[p].begin(), next[p].end());
        next[p].erase(std::unique(next[p].begin(), next[p].end()), next[p].end());
        G.paths[p].next.first = static_cast<uint32_t>(G.nextPaths.size());
        G.nextPaths.insert(G.nextPaths.end(), next[p].begin(), next[p].end());
        G.paths[p].next.count = static_cast<uint32_t>(next[p].size());
    }

    // (THE SIGNS AND RULES ALONG THE LANES are not written here, ADR-0437: the model states them -- each arm's signs, each
    // street's regulations -- and answers for a path, RoadModel::regulationsAlong, from the arm and the stretch of
    // spine each lane path carries.)

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
                // (numbers: the jump in degrees, metres along the path, the jump that counts as a kink)
                out.problems.push_back(RoadProblem(RoadProblemKind::LaneKink).onLane(m.lanes[path.modelLane].id).onPath(p).where(k.at)
                                           .number(k.degrees).number(k.along).number(kKinkDegrees));
            } else {
                ++out.connectorKinks;
                out.worstConnectorKink = std::max(out.worstConnectorKink, k.degrees);
                // (numbers: the same three, then the lane level's junction it runs through)
                out.problems.push_back(RoadProblem(RoadProblemKind::ConnectorKink).onPath(p).where(k.at).number(k.degrees).number(k.along)
                                           .number(kKinkDegrees).number(path.junction.index));
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
            out.problems.push_back(RoadProblem(RoadProblemKind::JoinKink).word(G.paths[p].kind == PathKind::Lane ? "lane" : "connector").onPath(p)
                                       .word(G.paths[n.index].kind == PathKind::Lane ? "lane" : "connector").andPath(n.index).where(at)
                                       .number(degrees).number(kKinkDegrees));
        }
    }
    out.bendPieceBytes = G.bendPieces.size() * sizeof(BendPiece) + G.paths.size() * sizeof(Range);

    // ---- the model's crosswalks laid in (ADR-0452), so the crossing finder finds the paths over each ---------------------
    layLaneCrosswalks(model);
    // ---- where the paths' bodies could touch (measured), and where a far turn may wait inside -----------------------------
    CrossingOptions co;
    co.vehicles = opts.vehicles;
    co.clearance = g.rules.sameLevelDz;
    out.crossings = findCrossingPoints(G, co);
    out.problems.insert(out.problems.end(), out.crossings.problems.begin(), out.crossings.problems.end());
    // EACH STATED MERGE ON ITS JOIN (nothing measured decides it): every Join between the joining lane's last path and a
    // path of its host lane names the connection that states it. Which of the two joins -- and so gives way -- is asked
    // of the model by that connection (RoadModel::mergingAt), never written here.
    // A stated merge whose paths the crossings code found no Join for is said.
    for (const MadeMerge& mm : merges) {
        uint32_t marked = 0;
        for (CrossingPoint& x : G.crossings) {
            if (x.meeting != Meeting::Join || x.otherSide != OtherSide::Path) continue;
            const uint32_t other = x.a.index == mm.joining ? x.other : x.other == mm.joining ? x.a.index : kNoIndex;
            if (other == kNoIndex || other >= G.paths.size() || G.paths[other].kind != PathKind::Lane || G.paths[other].modelLane != mm.hostLane)
                continue;
            x.modelConnection = mm.connection;
            ++marked;
        }
        if (marked == 0)
            out.problems.push_back(RoadProblem(RoadProblemKind::MergeNotCarried).onLane(m.lanes[mm.branchLane].id).andLane(m.lanes[mm.hostLane].id)
                                       .number(mm.connection));
    }
    out.crossings.waitInsidePoints = placeWaitInsidePoints(model);

    // ---- the counts the old reports read ------------------------------------------------------------------------------
    out.controlPlan.planned = static_cast<uint32_t>(m.junctions.size());
    for (uint32_t lj = 0; lj < G.junctions.size(); ++lj) {
        const std::string& control = m.controlAt(JunctionId(lj));
        bool found = false;
        for (std::pair<std::string, uint32_t>& c : out.controlPlan.byControl)
            if (c.first == control) {
                ++c.second;
                found = true;
            }
        if (!found) out.controlPlan.byControl.emplace_back(control, 1);
    }
    std::sort(out.controlPlan.byControl.begin(), out.controlPlan.byControl.end());
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        if (!(m.postedLimit(PathId(p)) > 0)) continue;
        ++(G.paths[p].kind == PathKind::Lane ? out.controlPlan.lanesPosted : out.controlPlan.connectorsPosted);
    }
    for (uint32_t p = 0; p < G.paths.size(); ++p) {
        for (const PathRegulation& reg : m.regulationsAlong(PathId(p))) {
            bool found = false;
            for (std::pair<std::string, uint32_t>& c : out.controlPlan.regulations)
                if (c.first == reg.kind) {
                    ++c.second;
                    found = true;
                }
            if (!found) out.controlPlan.regulations.emplace_back(reg.kind, 1);
        }
    }
    std::sort(out.controlPlan.regulations.begin(), out.controlPlan.regulations.end());

    // ---- where people walk (ADR-0466): last, so no car path's points move ------------------------------------------------
    layWalkPaths(r, model, out);

    for (const std::string& p : validate(G)) out.problems.push_back(RoadProblem(RoadProblemKind::LaneGraphCheck).word(p));
    reachReport(G, out);
    out.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return out;
}

}  // namespace roads::lanes
}  // namespace engine
