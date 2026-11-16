#include "engine/procgen/city/roads/street_map.h"

#include "engine/procgen/city/polyline.h"    // douglasPeucker, distToLine, stations, valueAtStation
#include "engine/procgen/city/road_mesh.h"   // RoadDeckField

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <functional>
#include <map>
#include <set>

namespace engine {

namespace {

std::string fmt(const char* f, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

bool isStreetClass(RoadClass k) { return k == RoadClass::Arterial || k == RoadClass::Collector || k == RoadClass::Local; }
bool isDeckClass(RoadClass k) { return k == RoadClass::Freeway || k == RoadClass::Ramp; }

// A stretch of centre line to lay links along: its points and their stations along the road's spine.
struct Run {
    std::vector<Vec2> pts;
    std::vector<Real> st;
};

// The model's centre line `line`, between stations lo and hi (lo < hi), its ends at exactly lo and hi.
Run runOf(const RoadModel& m, const Range& line, Real lo, Real hi) {
    Run r;
    if (line.count < 2) return r;
    // (polyline.h stretchAtStations, the one function; lo and hi are written stations)
    stretchAtStations(&m.linePoints[line.first], &m.lineStations[line.first], line.count, lo, hi, kWrittenStation, r.pts, &r.st);
    return r;
}

// The run kept to within kStreetMapTolerance, a point at least every kStreetMapSpan metres, and a point at each of
// `cuts` (stations strictly inside it).
Run thinned(const Run& in, const std::vector<Real>& cuts) {
    Run out;
    if (in.pts.size() < 2) return in;
    std::vector<char> keep(in.pts.size(), 0);
    keep.front() = keep.back() = 1;
    douglasPeucker(0, in.pts.size() - 1, kStreetMapTolerance,
                   [&](std::size_t i, std::size_t a, std::size_t b) { return distToLine(in.pts[i], in.pts[a], in.pts[b]); }, keep);
    std::vector<std::pair<Real, Vec2>> kept;
    for (std::size_t i = 0; i < in.pts.size(); ++i)
        if (keep[i]) kept.emplace_back(in.st[i], in.pts[i]);
    // a node at each cut (a ramp's gore on this street), on the line as it was
    for (Real c : cuts) kept.emplace_back(c, pointAtStation(in.pts.data(), in.st.data(), in.pts.size(), c));
    std::sort(kept.begin(), kept.end(), [](const std::pair<Real, Vec2>& a, const std::pair<Real, Vec2>& b) { return a.first < b.first; });
    for (const std::pair<Real, Vec2>& k : kept) {
        if (!out.st.empty() && atOrBefore(k.first, out.st.back(), kWrittenStation)) continue;
        // ...and no longer than kStreetMapSpan between nodes: the span cut into equal parts, on the line as it was
        if (!out.st.empty()) {
            const Real span = k.first - out.st.back();
            const int parts = static_cast<int>(std::ceil(span / kStreetMapSpan - 1e-9));
            const Real s0 = out.st.back();
            for (int p = 1; p < parts; ++p) {
                const Real s = s0 + span * p / parts;
                out.pts.push_back(pointAtStation(in.pts.data(), in.st.data(), in.pts.size(), s));
                out.st.push_back(s);
            }
        }
        out.pts.push_back(k.second);
        out.st.push_back(k.first);
    }
    return out;
}

}  // namespace

bool appendStreetMap(NavGraph& map, const RoadModel& m, int city, const RoadDeckField* deck, StreetMapReport* report) {
    StreetMapReport local;
    StreetMapReport& rep = report ? *report : local;
    // ---- the facts the map cannot be built without -----------------------------------------------------------------------
    for (const ModelJunction& J : m.junctions)
        if (!J.inside.empty() && J.nodes.empty())
            rep.errors.push_back(fmt("junction %s (at %.1f, %.1f) lies over several points (stretches of road lie inside it), and the road file "
                                     "does not say which roads meet at each: run `lanes_tool layout <road file> --nodes --write` once",
                                     J.id.c_str(), J.at.x, J.at.y));
    if (!m.junctions.empty() && m.lineStations.size() != m.linePoints.size())
        rep.errors.push_back("the road model's centre lines carry no stations (a model older than ADR-0425)");
    if (!rep.errors.empty()) return false;

    // the graph's per-node vectors, sized to what it holds already (a map that grew before, or nothing)
    const std::size_t had = map.nodes.size();
    map.outLinks.resize(had);
    map.junction.resize(had, 0);
    map.nodeSpread.resize(had, 0);
    map.nodeKind.resize(had, static_cast<uint8_t>(JunctionKind::None));
    map.footNode.resize(had, 0);
    map.nodeJunction.resize(had, -1);
    map.nodeCity.resize(had, -1);
    auto addNode = [&](const Vec2& p, int junction, JunctionKind kind) {
        map.nodes.push_back(p);
        map.outLinks.emplace_back();
        map.junction.push_back(kind == JunctionKind::Intersection || kind == JunctionKind::Landing || isGore(kind) ? 1 : 0);
        map.nodeSpread.push_back(0);
        map.nodeKind.push_back(static_cast<uint8_t>(kind));
        map.footNode.push_back(0);
        map.nodeJunction.push_back(junction);
        map.nodeCity.push_back(city);
        ++rep.nodes;
        return static_cast<int>(map.nodes.size()) - 1;
    };

    // the drawn deck's height along a road (its spine in the deck field: one per road-file edge, in edge order)
    std::map<uint32_t, std::vector<Real>> deckStations;
    bool deckNoted = false;
    auto deckY = [&](uint32_t edge, Real station, Real* y) {
        if (!deck || edge >= deck->spines.size() || deck->spines[edge].yAbs.size() != deck->spines[edge].points.size()) {
            if (!deckNoted) rep.notes.push_back("the drawn deck is missing or does not hold one spine per road: deck links carry no heights");
            deckNoted = true;
            return false;
        }
        const UnionSpine& sp = deck->spines[edge];
        std::vector<Real>& s = deckStations[edge];
        if (s.empty()) s = stations(sp.points);
        *y = valueAtStation(s.data(), sp.yAbs.data(), s.size(), station);
        return true;
    };

    // ---- the links along a run of centre line between two nodes, each way its lanes run ---------------------------------
    // (`forward` / `backward`: lay that way's links, where the section has lanes that way)
    // THE ONE WRITER of a street-map link's compiled look-up (ADR-0437): its width, class, lanes, one-way, walkable,
    // access bits and kerb parking are filled here from the street's stated cross-section and nowhere else. (The arm a
    // link arrives by is not kept: RoadModel::armReachedAt answers it from the link's street and far station.)
    auto layLinks = [&](const Run& run, const std::vector<int>& nodesAlong, const ModelSection& sec, uint32_t edge, int street, int junction,
                        bool forward = true, bool backward = true) {
        const bool walk = sec.sidewalk > 0 && isStreetClass(sec.klass);
        const bool deckClass = isDeckClass(sec.klass);
        uint8_t access = 0;
        if (walk) access |= road_access::kCrossable;   // (that people walk it is the link's `walkable`, set below)
        if (walk && !deckClass) access |= road_access::kFrontage;
        if (isStreetClass(sec.klass)) access |= road_access::kSignalable;
        auto add = [&](std::size_t i, std::size_t k, bool along) {
            const int a = nodesAlong[i], b = nodesAlong[k];
            if (a == b) return;
            NavLink L;
            L.from = a;
            L.to = b;
            L.length = (map.nodes[static_cast<std::size_t>(b)] - map.nodes[static_cast<std::size_t>(a)]).length();
            L.width = sec.travelWidth();
            L.klass = sec.klass;
            L.lanes = std::max(1, along ? sec.lanesForward : sec.lanesBackward);
            L.oneWay = sec.oneWay();
            L.walkable = walk;
            L.access = access;
            L.parkOffset = sec.parking > 0 ? sec.parkingOffset : 0.0;
            L.parkWidth = sec.parking;
            L.city = city;
            L.street = street;
            L.junction = junction;
            L.road = static_cast<int>(edge);
            L.stationFrom = run.st[i];
            L.stationTo = run.st[k];
            if (deckClass) {
                Real ya = 0, yb = 0;
                if (deckY(edge, L.stationFrom, &ya) && deckY(edge, L.stationTo, &yb)) {
                    L.elevA = ya;
                    L.elevB = yb;
                    L.elevAbsolute = L.elevAbsA = L.elevAbsB = true;
                }
            }
            map.outLinks[static_cast<std::size_t>(a)].push_back(map.linkCount());
            map.links.push_back(L);
            ++rep.links;
            if (junction >= 0) ++rep.insideLinks;
        };
        for (std::size_t i = 0; i + 1 < nodesAlong.size(); ++i) {
            if (forward && sec.lanesForward > 0) add(i, i + 1, true);
            if (backward && sec.lanesBackward > 0) add(i + 1, i, false);
        }
    };

    // ---- the roads by id (arms name them) ---------------------------------------------------------------------------------
    std::map<std::string, uint32_t> edgeOf;
    for (const ModelStreet& S : m.streets) edgeOf[S.road] = S.edge;
    for (const ModelJunction& J : m.junctions)
        for (const ModelStretch& st : J.inside) edgeOf[st.road] = st.edge;

    // ---- the junctions: one node, or one at each stated point ------------------------------------------------------------
    std::vector<std::vector<int>> pointNodes(m.junctions.size());   // per junction: its nodes, in point order
    auto pointOf = [&](std::size_t j, uint32_t edge, Real station) {
        const ModelJunction& J = m.junctions[j];
        if (J.nodes.empty()) return 0;
        for (std::size_t n = 0; n < J.nodes.size(); ++n)
            for (const ModelNodePoint& p : J.nodes[n].points)
                if (p.edge == edge && sameWrittenStation(p.station, station)) return static_cast<int>(n);
        return -1;
    };
    for (std::size_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        ++rep.junctions;
        // a point where a ramp's street meets the junction is a landing; any other, an intersection
        const std::size_t points = J.nodes.empty() ? 1 : J.nodes.size();
        std::vector<char> landing(points, 0);
        for (const ModelArm& a : J.arms) {
            if (!a.street.valid() || m.streets[a.street.index].section.klass != RoadClass::Ramp) continue;
            const std::map<std::string, uint32_t>::const_iterator e = edgeOf.find(a.road);
            const int p = e == edgeOf.end() ? -1 : pointOf(j, e->second, a.station);
            if (p >= 0) landing[static_cast<std::size_t>(p)] = 1;
        }
        for (std::size_t p = 0; p < points; ++p) {
            const Vec2 at = J.nodes.empty() ? J.at : J.nodes[p].at;
            pointNodes[j].push_back(addNode(at, static_cast<int>(j), landing[p] ? JunctionKind::Landing : JunctionKind::Intersection));
            ++rep.junctionPoints;
        }
    }

    // ---- the ramps' gores on their hosts: a node splitting the host's street at the stated station ------------------------
    std::map<std::pair<uint32_t, int>, int> goreNode;           // (ramp edge, its end) -> the node on its host
    std::map<std::pair<uint32_t, int>, Real> goreAlongRamp;     // ...and the gore along the ramp itself
    std::map<int, std::vector<std::pair<Real, int>>> cutsOn;   // host street -> (station, node)
    for (const ModelRamp& R : m.ramps) {
        const std::map<std::string, uint32_t>::const_iterator re = edgeOf.find(R.edge);
        if (re == edgeOf.end()) continue;
        for (int end = 0; end < 2; ++end) {
            const ModelRampEnd& E = R.ends[static_cast<std::size_t>(end)];
            if (E.kind != RampEndKind::Anchored) continue;
            const std::map<std::string, uint32_t>::const_iterator he = edgeOf.find(E.host);
            int host = -1;
            for (std::size_t s = 0; s < m.streets.size() && he != edgeOf.end(); ++s) {
                const ModelStreet& S = m.streets[s];
                if (S.edge == he->second && stated(E.hostStation) && atOrPast(E.hostStation, S.from, kWrittenStation) && atOrBefore(E.hostStation, S.to, kWrittenStation))
                    host = static_cast<int>(s);
            }
            if (host < 0 || !stated(E.rampStation)) {
                rep.notes.push_back(fmt("ramp %s: its %s end is anchored on %s at %.1f, on no street of it (or its gore along itself is not "
                                        "stated): the ramp is not joined there", R.edge.c_str(), end == 0 ? "start" : "end", E.host.c_str(),
                                        stated(E.hostStation) ? E.hostStation : -1.0));
                continue;
            }
            const ModelStreet& S = m.streets[static_cast<std::size_t>(host)];
            const Run hostLine = runOf(m, S.line, S.from, S.to);
            const Vec2 at = hostLine.pts.size() >= 2 ? pointAtStation(hostLine.pts.data(), hostLine.st.data(), hostLine.pts.size(), E.hostStation)
                                                     : Vec2(0, 0);
            const int node = addNode(at, -1, end == 0 ? JunctionKind::Diverge : JunctionKind::Merge);
            goreNode[{re->second, end}] = node;
            goreAlongRamp[{re->second, end}] = E.rampStation;
            cutsOn[host].emplace_back(E.hostStation, node);
            ++rep.gores;
        }
    }

    // ---- where a street carries on as another: one node, shared -----------------------------------------------------------
    std::map<std::pair<int, int>, int> carryNode;   // (street, its end) -> the node
    auto otherEnd = [&](int s, int k, int t) {
        // the end of t that names s: the opposite end first (two streets carrying on run on the same way), else the same
        const ModelStreet& T = m.streets[static_cast<std::size_t>(t)];
        const int opposite = 1 - k;
        if (T.carriesOn[static_cast<std::size_t>(opposite)].valid() && T.carriesOn[static_cast<std::size_t>(opposite)].index == static_cast<uint32_t>(s))
            return opposite;
        if (T.carriesOn[static_cast<std::size_t>(k)].valid() && T.carriesOn[static_cast<std::size_t>(k)].index == static_cast<uint32_t>(s)) return k;
        return -1;
    };

    // ---- the streets ------------------------------------------------------------------------------------------------------
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        const ModelStreet& S = m.streets[s];
        if (!S.section.carriesCars()) continue;   // (footpaths are the sim's walks; tracks are the trains')
        if (S.section.lanesForward + S.section.lanesBackward == 0 || S.line.count < 2) continue;
        std::array<int, 2> endNode{-1, -1}, endArm{-1, -1};
        std::array<Real, 2> endStation{S.from, S.to};
        std::array<bool, 2> atGore{false, false};   // this end is a ramp's, joined at the node on its host
        for (int k = 0; k < 2; ++k) {
            const std::size_t kk = static_cast<std::size_t>(k);
            const Real station = k == 0 ? S.from : S.to;
            Vec2 at;   // where this end of the street is (it has a centre line: checked above)
            m.streetEnd(S, k, at);
            if (S.junctions[kk].valid()) {
                const std::size_t j = S.junctions[kk].index;
                const ModelJunction& J = m.junctions[j];
                // the arm that is this street, leaving the junction at its `from` (spine out) or entering at its `to`
                endArm[kk] = m.armAt(ModelStreetId(static_cast<uint32_t>(s)), k);
                const int p = pointOf(j, S.edge, station);
                if (endArm[kk] < 0)
                    rep.notes.push_back(fmt("street %s: its %s names junction %s, which lists no arm for it", S.id.c_str(), k == 0 ? "start" : "end",
                                            J.id.c_str()));
                if (p < 0) {
                    rep.notes.push_back(fmt("street %s: it meets junction %s at %.2f, at none of the junction's stated points: not joined there",
                                            S.id.c_str(), J.id.c_str(), station));
                } else {
                    endNode[kk] = pointNodes[j][static_cast<std::size_t>(p)];
                }
            } else if (S.carriesOn[kk].valid()) {
                const int t = static_cast<int>(S.carriesOn[kk].index);
                const int kt = otherEnd(static_cast<int>(s), k, t);
                std::map<std::pair<int, int>, int>::const_iterator it = carryNode.find({static_cast<int>(s), k});
                if (it != carryNode.end()) {
                    endNode[kk] = it->second;
                } else if (kt < 0) {
                    rep.notes.push_back(fmt("street %s: its %s carries on into %s, which does not name it back: not joined there", S.id.c_str(),
                                            k == 0 ? "start" : "end", m.streets[static_cast<std::size_t>(t)].id.c_str()));
                } else {
                    endNode[kk] = addNode(at, -1, JunctionKind::None);
                    carryNode[{static_cast<int>(s), k}] = endNode[kk];
                    carryNode[{t, kt}] = endNode[kk];
                    ++rep.carryOns;
                }
            } else {
                // a ramp's end anchored on its host: driven from (or to) its gore, joined at the host's node there. This
                // street end is the ramp's start when it begins its road, its end when no street of the road runs further
                int rampEnd = -1;
                if (k == 0 && sameWrittenStation(S.from, 0.0)) rampEnd = 0;
                if (k == 1) {
                    rampEnd = 1;
                    for (const ModelStreet& O : m.streets)
                        if (O.edge == S.edge && strictlyPast(O.to, S.to, kWrittenStation)) rampEnd = -1;
                }
                const std::map<std::pair<uint32_t, int>, int>::const_iterator g =
                    rampEnd >= 0 ? goreNode.find({S.edge, rampEnd}) : goreNode.end();
                if (g != goreNode.end()) {
                    endNode[kk] = g->second;
                    endStation[kk] = goreAlongRamp[{S.edge, rampEnd}];
                    atGore[kk] = true;
                } else {
                    endNode[kk] = addNode(at, -1, JunctionKind::DeadEnd);
                    ++rep.deadEnds;
                }
            }
            if (endNode[kk] < 0) {   // (unjoined, noted above): its own end, joined to nothing
                endNode[kk] = addNode(at, -1, JunctionKind::DeadEnd);
                ++rep.deadEnds;
            }
        }
        if (atOrBefore(endStation[1], endStation[0], kWrittenStation)) {
            rep.notes.push_back(fmt("street %s: nothing of it is left to drive between its ends (%.2f to %.2f)", S.id.c_str(), endStation[0], endStation[1]));
            continue;
        }
        // the cuts on it (ramps' gores), strictly inside
        std::vector<Real> cuts;
        std::map<Real, int> cutNode;
        const std::map<int, std::vector<std::pair<Real, int>>>::const_iterator ct = cutsOn.find(static_cast<int>(s));
        if (ct != cutsOn.end())
            for (const std::pair<Real, int>& c : ct->second) {
                if (atOrBefore(c.first, endStation[0], kWrittenStation) || atOrPast(c.first, endStation[1], kWrittenStation)) {
                    // a gore at the street's very end: that end's node is the gore's (the host's street ends there)
                    rep.notes.push_back(fmt("a ramp's gore stands at the end of street %s (%.2f): joined to that end", S.id.c_str(), c.first));
                    const int at = atOrBefore(c.first, endStation[0], kWrittenStation) ? 0 : 1;
                    // the gore's node takes the street's links there: the street's end node becomes the gore's
                    endNode[static_cast<std::size_t>(at)] = c.second;
                    continue;
                }
                cuts.push_back(c.first);
                cutNode[c.first] = c.second;
            }
        Run run = thinned(runOf(m, S.line, endStation[0], endStation[1]), cuts);
        // A RAMP KEEPS ITS OWN LINE TO ITS GORE: the node on the host stands on the host's centre line, beside the ramp's,
        // so the ramp's line runs to its gore station and one short link crosses from there to the host's node. (Put the
        // host's node in place of the ramp line's first point and that first link cut a chord across the ramp's curve:
        // river_town's cars drove up to 6 m off it, off the drawn deck.)
        if (atGore[0]) {
            run.pts.insert(run.pts.begin(), map.nodes[static_cast<std::size_t>(endNode[0])]);
            run.st.insert(run.st.begin(), run.st.front());
        }
        if (atGore[1]) {
            run.pts.push_back(map.nodes[static_cast<std::size_t>(endNode[1])]);
            run.st.push_back(run.st.back());
        }
        std::vector<int> along;
        for (std::size_t i = 0; i < run.pts.size(); ++i) {
            if (i == 0) { along.push_back(endNode[0]); continue; }
            if (i + 1 == run.pts.size()) { along.push_back(endNode[1]); continue; }
            int node = -1;
            for (const std::pair<const Real, int>& c : cutNode)
                if (sameWrittenStation(c.first, run.st[i])) node = c.second;
            along.push_back(node >= 0 ? node : addNode(run.pts[i], -1, JunctionKind::None));
        }
        layLinks(run, along, S.section, S.edge, static_cast<int>(s), -1);
    }

    // ---- the stretches of road inside the spread junctions, between the points on them --------------------------------------
    for (std::size_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        if (J.nodes.empty()) continue;
        // ONE PIECE BETWEEN TWO POINTS, EACH WAY: where the stated points put two of the junction's stretches between the
        // same two points (two roads both stated at both), the first in the junction's list is laid that way and the rest
        // are a note -- parallel links over one piece of road made cars on them blind to each other (ADR-0425)
        std::set<std::pair<int, int>> laidBetween;   // (from point's node, to point's node), directed
        for (const ModelStretch& st : J.inside) {
            if (st.edge == kNoIndex || st.line.count < 2) continue;
            if (!st.section.carriesCars() || st.section.lanesForward + st.section.lanesBackward == 0) continue;
            // the points on this road within the stretch, in station order
            std::vector<std::pair<Real, int>> on;
            for (std::size_t p = 0; p < J.nodes.size(); ++p)
                for (const ModelNodePoint& q : J.nodes[p].points)
                    if (q.edge == st.edge && atOrPast(q.station, st.from, kWrittenStation) && atOrBefore(q.station, st.to, kWrittenStation))
                        on.emplace_back(q.station, pointNodes[j][p]);
            std::sort(on.begin(), on.end());
            if (on.empty() || strictlyPast(on.front().first, st.from, kWrittenStation) || strictlyBefore(on.back().first, st.to, kWrittenStation))
                rep.notes.push_back(fmt("junction %s: its stretch of %s inside it (%.2f to %.2f) does not end at its stated points: the stretch "
                                        "beyond them is not driven", J.id.c_str(), st.road.c_str(), st.from, st.to));
            for (std::size_t i = 0; i + 1 < on.size(); ++i) {
                if (atOrBefore(on[i + 1].first, on[i].first, kWrittenStation) || on[i].second == on[i + 1].second) continue;
                const std::pair<int, int> ahead(on[i].second, on[i + 1].second), back(on[i + 1].second, on[i].second);
                const bool forward = st.section.lanesForward > 0 && laidBetween.count(ahead) == 0;
                const bool backward = st.section.lanesBackward > 0 && laidBetween.count(back) == 0;
                if (forward != (st.section.lanesForward > 0) || backward != (st.section.lanesBackward > 0))
                    rep.notes.push_back(fmt("junction %s: %s (%.2f to %.2f) runs between two of its points another of its stretches "
                                            "already joins %s: not laid again that way", J.id.c_str(), st.road.c_str(), on[i].first,
                                            on[i + 1].first, forward || backward ? "one way" : "both ways"));
                if (!forward && !backward) continue;
                if (forward) laidBetween.insert(ahead);
                if (backward) laidBetween.insert(back);
                const Run run = thinned(runOf(m, st.line, on[i].first, on[i + 1].first), {});
                std::vector<int> along;
                for (std::size_t k = 0; k < run.pts.size(); ++k) {
                    if (k == 0) along.push_back(on[i].second);
                    else if (k + 1 == run.pts.size()) along.push_back(on[i + 1].second);
                    else along.push_back(addNode(run.pts[k], static_cast<int>(j), JunctionKind::None));
                }
                layLinks(run, along, st.section, st.edge, -1, static_cast<int>(j), forward, backward);
            }
        }
        // a junction its stated points and stretches leave in pieces: a finding
        std::vector<int> part(J.nodes.size());
        for (std::size_t p = 0; p < part.size(); ++p) part[p] = static_cast<int>(p);
        std::function<int(int)> root = [&](int x) { return part[static_cast<std::size_t>(x)] == x ? x : part[static_cast<std::size_t>(x)] = root(part[static_cast<std::size_t>(x)]); };
        std::map<int, int> pointIndex;
        for (std::size_t p = 0; p < pointNodes[j].size(); ++p) pointIndex[pointNodes[j][p]] = static_cast<int>(p);
        // (walk the links inside it: a point reaches another along them, through the stretch's own bends)
        for (std::size_t p = 0; p < pointNodes[j].size(); ++p) {
            std::vector<int> stack{pointNodes[j][p]};
            std::set<int> seen{pointNodes[j][p]};
            while (!stack.empty()) {
                const int n = stack.back();
                stack.pop_back();
                const std::map<int, int>::const_iterator pi = pointIndex.find(n);
                if (pi != pointIndex.end()) part[static_cast<std::size_t>(root(pi->second))] = root(static_cast<int>(p));
                for (int li : map.outLinks[static_cast<std::size_t>(n)]) {
                    const NavLink& L = map.links[static_cast<std::size_t>(li)];
                    if (L.junction != static_cast<int>(j) || seen.count(L.to)) continue;
                    seen.insert(L.to);
                    stack.push_back(L.to);
                }
            }
        }
        std::set<int> pieces;
        for (std::size_t p = 0; p < part.size(); ++p) pieces.insert(root(static_cast<int>(p)));
        if (pieces.size() > 1)
            rep.notes.push_back(fmt("junction %s (at %.1f, %.1f): its stated points and the stretches inside it make %zu pieces; nothing joins "
                                    "them", J.id.c_str(), J.at.x, J.at.y, pieces.size()));
    }
    map.streetNodes = -1;   // (no walks yet: every node and link is a street's)
    map.streetLinks = -1;
    return true;
}

std::string describeStreetMap(const StreetMapReport& r) {
    std::string out = fmt("%u nodes, %u links: %u junctions (%u points), %u places a street carries on, %u ramp gores, %u dead ends; "
                          "%u links inside junctions",
                          r.nodes, r.links, r.junctions, r.junctionPoints, r.carryOns, r.gores, r.deadEnds, r.insideLinks);
    for (const std::string& e : r.errors) out += "\n  error: " + e;
    for (const std::string& n : r.notes) out += "\n  note: " + n;
    return out;
}

}  // namespace engine
