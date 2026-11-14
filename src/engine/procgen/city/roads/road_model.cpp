#include "engine/procgen/city/roads/road_model.h"

#include "engine/bundle/bake.h"
#include "engine/bundle/bundle.h"
#include "engine/bundle/cache_use.h"
#include "engine/procgen/city/roads/lanes/city_producer.h"     // cityEntities, loadCityGraph
#include "engine/procgen/city/roads/lanes/deck_mesh.h"         // kStopLineBack, kJunctionMergeGap, paintsJunctionBoxes
#include "engine/procgen/city/roads/lanes/lane_graph_cache.h"  // levelRoadsKey: the roads' key
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_twin.h"         // classOf: the engine's class of a road
#include "engine/procgen/city/polyline.h"                      // simplifyRun, distToSegment
#include "engine/procgen/city/polygon.h"                       // pointInPolygon, interiorPoint: the regions' outlines
#include "engine/asset_root.h"                                 // assetPath: the attachment kinds' file
#include "log.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <type_traits>

#ifndef _WIN32
#include <unistd.h>
#endif

namespace engine {

namespace {

using roads::lanes::EdgeSpec;
using roads::lanes::Lane;
using roads::lanes::PlannedArm;
using roads::lanes::PlannedJunction;
using roads::lanes::PlannedStreet;
using roads::lanes::Result;
using roads::lanes::RoadClassSpec;
using roads::lanes::RoadLabGraph;

std::string fmt(const char* f, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

constexpr const char* kMagic = "RMDL";

// A street's centre line keeps its spine's points to within this (metres): the spine is sampled every metre, and a map, a
// sign's corner or "the street nearest a point" needs its shape, not every sample.
constexpr Real kLineTolerance = 0.1;

// The first entry of `notStated` in a planned-only model (assembleRoadModel takes it out).
constexpr const char* kBuiltLayerAbsent =
    "the built layer: lanes, connections, junction heights, the paint's boxes and stop lines are not here (a planned-only "
    "model, read from the road file: assembleRoadModel adds them from a road build; cache/roadmodel holds one)";

}  // namespace

// ---- the planned layer and the model of a build -------------------------------------------------------------------------

RoadModel plannedRoadModel(const RoadLabGraph& g) {
    RoadModel m;
    m.name = g.name;
    std::map<std::string, uint32_t> streetIndex, junctionIndex;
    for (std::size_t i = 0; i < g.streets.size(); ++i) streetIndex[g.streets[i].id] = static_cast<uint32_t>(i);
    for (std::size_t i = 0; i < g.junctions.size(); ++i) junctionIndex[g.junctions[i].id] = static_cast<uint32_t>(i);
    auto streetOf = [&](const std::string& id) {
        const std::map<std::string, uint32_t>::const_iterator it = streetIndex.find(id);
        return it == streetIndex.end() ? ModelStreetId() : ModelStreetId(it->second);
    };
    auto junctionOf = [&](const std::string& id) {
        const std::map<std::string, uint32_t>::const_iterator it = junctionIndex.find(id);
        return it == junctionIndex.end() ? ModelJunctionId() : ModelJunctionId(it->second);
    };

    // ---- junctions, as the road file states them -------------------------------------------------------------------
    for (const PlannedJunction& pj : g.junctions) {
        ModelJunction J;
        J.id = pj.id;
        J.at = pj.at;
        if (g.declaresJunctions) {
            J.control = pj.control;
            J.why = pj.why;
        }
        for (const PlannedArm& a : pj.arms) {
            ModelArm arm;
            arm.street = streetOf(a.street);
            arm.road = a.road;
            arm.station = a.at;
            arm.spineIn = a.spineIn;
            arm.heading = a.hasHeading ? a.heading : kNotStated;
            arm.regulations = a.regulations;
            J.arms.push_back(arm);
        }
        for (const std::array<int, 2>& t : pj.through) J.through.push_back({static_cast<uint32_t>(t[0]), static_cast<uint32_t>(t[1])});
        for (const roads::lanes::PlannedStretch& st : pj.inside) J.inside.push_back(ModelStretch{st.road, st.from, st.to});
        m.junctions.push_back(std::move(J));
    }

    // ---- streets, their cross-sections from their road's class and lanes ----------------------------------------------
    std::set<uint32_t> parkingRefused;   // roads already reported for kerb parking that does not fit
    for (const PlannedStreet& st : g.streets) {
        ModelStreet S;
        S.id = st.id;
        S.road = st.road;
        S.edge = static_cast<uint32_t>(g.index.at(st.road));
        S.from = st.from;
        S.to = st.to;
        S.junctions = {junctionOf(st.junctions[0]), junctionOf(st.junctions[1])};
        const EdgeSpec& e = g.edges[S.edge];
        const RoadClassSpec& c = g.cls(e);
        S.section.roadClass = e.cls;
        S.section.klass = roads::lanes::classOf(e);
        S.section.lanesForward = e.lanes.fwd;
        S.section.lanesBackward = e.lanes.back;
        S.section.laneWidth = e.lanes.w;
        S.section.medianGap = e.lanes.gap;
        S.section.median = c.median;
        S.section.shoulder = c.shoulder;
        S.section.sidewalk = c.sidewalk;
        S.section.track = c.track;
        S.section.footway = c.footway;
        {
            // the kerb parking the road states (its class's, or its own), where it fits (ADR-0418)
            bool fits = true;
            S.section.parking = c.track || c.footway ? 0.0 : roads::lanes::kerbParking(g, e, &fits);
            if (!fits && !c.track && !c.footway && parkingRefused.insert(S.edge).second)
                m.problems.push_back(fmt("road %s (%s): it states %.1f m of kerb parking, which would leave a travel lane under %.1f m; none is "
                                         "built", e.id.c_str(), e.cls.c_str(), e.lanes.parking, roads::lanes::kMinTravelLane));
        }
        S.oneWay = e.lanes.back == 0 && e.lanes.fwd > 0;
        S.speedLimit = e.speedLimit;
        S.lotsFrom = e.lotsFrom;
        S.lotsTo = e.lotsTo;
        S.frontageOf = e.frontageOf;
        // its centre line: the road's spine from `from` to `to`, its straight runs thinned to within kLineTolerance
        if (e.xy.size() >= 2) {
            std::vector<Vec2> pts{roads::lanes::pointAt(e.xy, e.s, st.from)};
            for (std::size_t i = 0; i < e.xy.size(); ++i)
                if (e.s[i] > st.from + 1e-6 && e.s[i] < st.to - 1e-6) pts.push_back(e.xy[i]);
            pts.push_back(roads::lanes::pointAt(e.xy, e.s, st.to));
            S.line.first = static_cast<uint32_t>(m.linePoints.size());
            for (std::size_t i : simplifyRun(pts, kLineTolerance)) m.linePoints.push_back(pts[i]);
            S.line.count = static_cast<uint32_t>(m.linePoints.size()) - S.line.first;
        }
        m.streets.push_back(std::move(S));
    }
    // where a street only carries on into another (no junction there: ADR-0419), as the road file states it
    {
        std::map<std::string, uint32_t> streetIndex;
        for (std::size_t i = 0; i < m.streets.size(); ++i) streetIndex[m.streets[i].id] = static_cast<uint32_t>(i);
        for (std::size_t i = 0; i < g.streets.size() && i < m.streets.size(); ++i)
            for (std::size_t k = 0; k < 2; ++k) {
                const std::map<std::string, uint32_t>::const_iterator it = streetIndex.find(g.streets[i].carriesOn[k]);
                if (it != streetIndex.end()) m.streets[i].carriesOn[k] = ModelStreetId(it->second);
            }
    }

    // ---- ramps, as their anchors state them -------------------------------------------------------------------------------
    for (const EdgeSpec& e : g.edges) {
        if (!e.isRamp()) continue;
        ModelRamp R;
        R.edge = e.id;
        for (int end = 0; end < 2; ++end) {
            const roads::lanes::RampAnchor& a = end == 0 ? e.from : e.to;
            ModelRampEnd& E = R.ends[static_cast<std::size_t>(end)];
            E.host = a.edge;
            if (a.set) {
                const EdgeSpec* host = g.find(a.edge);
                const std::map<std::string, std::string>::const_iterator hl = e.anchorLanes.find(end == 0 ? "from" : "to");
                E.kind = RampEndKind::Anchored;
                E.hostLane = hl != e.anchorLanes.end() ? hl->second : std::string();
                E.hostStation = host ? roads::lanes::anchorStation(*host, a) : kNotStated;
                E.length = end == 0 ? a.decel : a.aux;
                E.taper = a.taper > 0 ? a.taper : (host ? 30 * host->lanes.w : 0.0);
            } else if (!a.edge.empty()) {
                E.kind = RampEndKind::OnStreet;
            }
        }
        if (e.landing.declared) {
            R.landingPlanned = true;
            R.landingStreet = e.landing.street;
            R.landingJoins = e.landing.joins;
            R.landingCarriageway = e.landing.carriageway;
            R.landingCrossing = e.landing.crossingKind;
        }
        m.ramps.push_back(std::move(R));
    }

    // ---- crossings, as declared ------------------------------------------------------------------------------------------
    for (const roads::lanes::CrossingSpec& c : g.crossings)
        m.crossings.push_back(ModelCrossing{c.a, c.b, c.kind, c.over, c.at, c.clearance >= 0 ? c.clearance : kNotStated});

    // ---- the streets' names, as the road file states them: one attachment on each named street (ADR-0420) ---------------
    {
        const AttachmentKinds& kinds = attachmentKinds();
        const std::string nameKind = kinds.use("street_name");
        for (std::size_t i = 0; i < g.streets.size() && i < m.streets.size(); ++i) {
            if (g.streets[i].name.empty()) continue;
            ModelAttachment a;
            a.kind = nameKind;
            a.street = ModelStreetId(static_cast<uint32_t>(i));
            a.text = g.streets[i].name;
            std::string why;
            if (!attach(m, a, kinds, &why)) m.problems.push_back("street " + g.streets[i].id + ": its name is not attached: " + why);
        }
    }

    // ---- what nothing states yet ---------------------------------------------------------------------------------------
    m.notStated.push_back(kBuiltLayerAbsent);
    if (!g.listsJunctions) m.notStated.push_back("junctions: the road file lists none (lanes_tool layout writes them)");
    if (!g.listsStreets) m.notStated.push_back("streets: the road file lists none (lanes_tool layout writes them); lanes carry no street");
    if (!g.declaresJunctions) m.notStated.push_back("controls and signs: the road file plans none (city_plan controls)");
    bool anyLimit = false, anyName = false;
    for (const EdgeSpec& e : g.edges) anyLimit = anyLimit || e.speedLimit > 0;
    for (const PlannedStreet& st : g.streets) anyName = anyName || !st.name.empty();
    if (!anyLimit) m.notStated.push_back("speed limits: no road posts one (the rule set's default for each class holds)");
    if (!anyName) m.notStated.push_back("street names: the road file names no street (lanes_tool layout --names writes them)");
    if (!g.declaresCrossings) m.notStated.push_back("crossings: the road file declares none (the design stage works them out from heights)");
    m.notStated.push_back("turn paths through junctions: nothing states them (the lane graph works them out)");
    m.notStated.push_back("which lines between lanes may be crossed: nothing states them");
    m.notStated.push_back("junction outlines on the ground: the builder has none (the paint's boxes are kept as stretches along each road)");
    m.notStated.push_back("crosswalks: nothing states them (the paint draws a zebra at each box)");
    m.notStated.push_back("parking spaces: the kerb parking band is stated (each street's cross-section), the spaces along it are not (the city sim lays them at load)");
    m.notStated.push_back("blocks and their uses: the city plan has them, the road file does not carry them");
    m.notStated.push_back("signal groups: nothing states them (each driving model works its own out)");
    return m;
}

RoadModel assembleRoadModel(const Result& r) {
    const RoadLabGraph& g = r.graph;
    RoadModel m = plannedRoadModel(g);
    m.built = true;
    m.notStated.erase(std::remove(m.notStated.begin(), m.notStated.end(), std::string(kBuiltLayerAbsent)), m.notStated.end());

    // ---- each junction's height: the drawn deck at it, along its first arm's road -------------------------------------
    for (std::size_t j = 0; j < m.junctions.size() && j < g.junctions.size(); ++j) {
        const PlannedJunction& pj = g.junctions[j];
        if (pj.arms.empty() || !r.heights) continue;
        const EdgeSpec* e = g.find(pj.arms.front().road);
        if (e && e->xy.size() >= 2)
            m.junctions[j].height = r.heights->deckRoad(g.index.at(e->id), roads::lanes::pointAt(e->xy, e->s, pj.arms.front().at));
    }

    // ---- lanes, as the builder laid them: their curves, heights, slots, streets and end junctions ----------------------
    for (std::size_t li = 0; li < r.lanes.lanes.size(); ++li) {
        const Lane& l = r.lanes.lanes[li];
        ModelLane L;
        L.id = l.id;
        L.kind = l.kind;
        L.edge = l.parent >= 0 ? static_cast<uint32_t>(l.parent) : kNoIndex;
        L.dir = l.dir;
        L.slot = l.slot;
        L.width = l.w;
        L.spineFrom = l.spineFrom;
        L.spineTo = l.spineTo;
        L.curve = Range{static_cast<uint32_t>(m.points.size()), static_cast<uint32_t>(l.xy.size())};
        // the spine sample each point was laid beside (lane_expand.cpp ribbonLane: point k of a lane laid from spineFrom)
        std::size_t i0 = 0;
        const EdgeSpec* e = l.parent >= 0 ? &g.edges[static_cast<std::size_t>(l.parent)] : nullptr;
        if (e) i0 = static_cast<std::size_t>(std::lower_bound(e->s.begin(), e->s.end(), l.spineFrom) - e->s.begin());
        for (std::size_t k = 0; k < l.xy.size(); ++k) {
            m.points.push_back(l.xy[k]);
            m.along.push_back(k < l.s.size() ? l.s[k] : 0.0);
            m.heights.push_back(r.heights ? r.heights->deck(static_cast<int>(li), l.xy[k]) : 0.0);
            m.spineStations.push_back(e && i0 + k < e->s.size() ? e->s[i0 + k] : kNotStated);
        }
        for (const roads::lanes::LaneStreetSpan& sp : l.streets) L.streets.push_back(ModelLaneSpan{ModelStreetId(static_cast<uint32_t>(sp.street)), sp.from, sp.to});
        L.atStart = l.junctionAtStart >= 0 ? ModelJunctionId(static_cast<uint32_t>(l.junctionAtStart)) : ModelJunctionId();
        L.atEnd = l.junctionAtEnd >= 0 ? ModelJunctionId(static_cast<uint32_t>(l.junctionAtEnd)) : ModelJunctionId();
        for (const ModelLaneSpan& sp : L.streets) m.streets[sp.street.index].lanes.push_back(ModelLaneId(static_cast<uint32_t>(li)));
        m.lanes.push_back(std::move(L));
    }
    for (ModelStreet& S : m.streets) {
        std::sort(S.lanes.begin(), S.lanes.end());
        S.lanes.erase(std::unique(S.lanes.begin(), S.lanes.end()), S.lanes.end());
        std::stable_sort(S.lanes.begin(), S.lanes.end(), [&](ModelLaneId a, ModelLaneId b) {
            const ModelLane& A = m.lanes[a.index];
            const ModelLane& B = m.lanes[b.index];
            return std::make_pair(-A.dir, A.slot) < std::make_pair(-B.dir, B.slot);
        });
    }

    // ---- the connections the ramps state (their planned ends: plannedRoadModel), between the lanes laid; connectors -------
    for (const ModelRamp& R : m.ramps) {
        const int rampLane = r.lanes.find(R.edge + ".f0");
        for (int end = 0; end < 2; ++end) {
            const ModelRampEnd& E = R.ends[static_cast<std::size_t>(end)];
            if (E.kind == RampEndKind::Anchored) {
                const int hostLane = r.lanes.find(E.hostLane);
                if (hostLane >= 0 && rampLane >= 0) {
                    ModelConnection C;
                    C.kind = end == 0 ? ConnectionKind::Diverge : ConnectionKind::Merge;
                    C.from = ModelLaneId(static_cast<uint32_t>(end == 0 ? hostLane : rampLane));
                    C.to = ModelLaneId(static_cast<uint32_t>(end == 0 ? rampLane : hostLane));
                    C.ramp = R.edge;
                    C.hostStation = E.hostStation;
                    C.length = E.length;
                    C.taper = E.taper;
                    m.connections.push_back(C);
                } else {
                    m.problems.push_back(fmt("ramp %s: its %s end names host lane \"%s\", which the builder did not lay", R.edge.c_str(),
                                             end == 0 ? "start" : "end", E.hostLane.c_str()));
                }
            }
        }
    }
    for (const roads::lanes::ConnectorSpec& c : g.connectors) {
        const int a = r.lanes.find(c.from), b = r.lanes.find(c.to);
        if (a < 0 || b < 0) continue;
        ModelConnection C;
        C.kind = ConnectionKind::Joins;
        C.from = ModelLaneId(static_cast<uint32_t>(a));
        C.to = ModelLaneId(static_cast<uint32_t>(b));
        C.hostStation = c.toS;
        m.connections.push_back(C);
    }

    // ---- the paint's boxes: each arm's box edge and stop line; boxes and arms that do not meet -------------------------
    // A box on an arm's road belongs to the arm's junction where it lies within kJunctionMergeGap of the junction's own
    // stretch of that road (its arms' stations and the stretches inside it) -- the paint's own rule for one junction along
    // a street; the nearest such box to the arm's station is the arm's.
    std::map<std::pair<int, std::size_t>, std::vector<uint32_t>> boxHeldBy;   // (edge, box) -> junctions holding it
    if (r.hasJunctionBoxes)
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            ModelJunction& J = m.junctions[j];
            for (ModelArm& a : J.arms) {
                const std::map<std::string, int>::const_iterator ei = g.index.find(a.road);
                if (ei == g.index.end()) continue;
                const EdgeSpec& e = g.edges[static_cast<std::size_t>(ei->second)];
                if (!roads::lanes::paintsJunctionBoxes(g, e)) continue;
                double lo = a.station, hi = a.station;
                for (const ModelArm& o : J.arms)
                    if (o.road == a.road) { lo = std::min(lo, o.station); hi = std::max(hi, o.station); }
                for (const ModelStretch& st : J.inside)
                    if (st.road == a.road) { lo = std::min(lo, st.from); hi = std::max(hi, st.to); }
                const std::map<int, std::vector<roads::lanes::JunctionSpan>>::const_iterator bx = r.junctionBoxes.find(ei->second);
                std::size_t best = 0;
                double bestGap = 1e30;
                if (bx != r.junctionBoxes.end())
                    for (std::size_t b = 0; b < bx->second.size(); ++b) {
                        const roads::lanes::JunctionSpan& sp = bx->second[b];
                        if (sp.s0 > hi + roads::lanes::kJunctionMergeGap || sp.s1 < lo - roads::lanes::kJunctionMergeGap) continue;
                        const double gap = a.station < sp.s0 ? sp.s0 - a.station : a.station > sp.s1 ? a.station - sp.s1 : 0.0;
                        if (gap < bestGap) { bestGap = gap; best = b; }
                    }
                if (bestGap > 1e29) {
                    const Vec2 p = e.xy.size() >= 2 ? roads::lanes::pointAt(e.xy, e.s, a.station) : J.at;
                    m.problems.push_back(fmt("junction %s: the paint found no box where its arm on %s (station %.1f) meets it, at (%.1f, %.1f)",
                                             J.id.c_str(), a.road.c_str(), a.station, p.x, p.y));
                    continue;
                }
                const roads::lanes::JunctionSpan& sp = bx->second[best];
                a.boxEdge = a.spineIn ? sp.s0 : sp.s1;
                a.stopLine = a.spineIn ? sp.s0 - roads::lanes::kStopLineBack : sp.s1 + roads::lanes::kStopLineBack;
                std::vector<uint32_t>& held = boxHeldBy[std::make_pair(ei->second, best)];
                if (std::find(held.begin(), held.end(), static_cast<uint32_t>(j)) == held.end()) {
                    held.push_back(static_cast<uint32_t>(j));
                    J.boxes.push_back(ModelStretch{a.road, sp.s0, sp.s1});
                }
            }
        }
    // ...and a box on a stretch of road lying inside a junction (between nodes the planner grouped into it) is that
    // junction's: it overlaps the stretch, or lies within kJunctionMergeGap of it
    if (r.hasJunctionBoxes)
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            ModelJunction& J = m.junctions[j];
            for (const ModelStretch& st : std::vector<ModelStretch>(J.inside)) {
                const std::map<std::string, int>::const_iterator ei = g.index.find(st.road);
                if (ei == g.index.end()) continue;
                const std::map<int, std::vector<roads::lanes::JunctionSpan>>::const_iterator bx = r.junctionBoxes.find(ei->second);
                if (bx == r.junctionBoxes.end()) continue;
                for (std::size_t b = 0; b < bx->second.size(); ++b) {
                    const roads::lanes::JunctionSpan& sp = bx->second[b];
                    if (sp.s0 > st.to + roads::lanes::kJunctionMergeGap || sp.s1 < st.from - roads::lanes::kJunctionMergeGap) continue;
                    std::vector<uint32_t>& held = boxHeldBy[std::make_pair(ei->second, b)];
                    if (std::find(held.begin(), held.end(), static_cast<uint32_t>(j)) != held.end()) continue;
                    held.push_back(static_cast<uint32_t>(j));
                    J.boxes.push_back(ModelStretch{st.road, sp.s0, sp.s1});
                }
            }
        }
    for (const std::pair<const int, std::vector<roads::lanes::JunctionSpan>>& bx : r.junctionBoxes)
        for (std::size_t b = 0; b < bx.second.size(); ++b) {
            const std::map<std::pair<int, std::size_t>, std::vector<uint32_t>>::const_iterator h = boxHeldBy.find(std::make_pair(bx.first, b));
            const EdgeSpec& e = g.edges[static_cast<std::size_t>(bx.first)];
            const roads::lanes::JunctionSpan& sp = bx.second[b];
            const Vec2 p = roads::lanes::pointAt(e.xy, e.s, 0.5 * (sp.s0 + sp.s1));
            // where the road file states that a street only carries on (ADR-0419): the paint still draws a box at a bend or
            // where one road runs on as another, by its own rule, until the builder paints the stated junctions (step 9)
            std::string carriesOn;
            for (const ModelStreet& S : m.streets) {
                if (S.edge != static_cast<uint32_t>(bx.first)) continue;
                for (std::size_t k = 0; k < 2; ++k) {
                    const Real end = k == 0 ? S.from : S.to;
                    if (S.carriesOn[k].valid() && end >= sp.s0 - 1.0 && end <= sp.s1 + 1.0)
                        carriesOn = S.id + " carries on into " + m.streets[S.carriesOn[k].index].id;
                }
            }
            if (h == boxHeldBy.end() && !carriesOn.empty())
                m.problems.push_back(fmt("the paint draws a junction box on %s (stations %.1f to %.1f) at (%.1f, %.1f) where street %s: the road "
                                         "file states no junction there (the paint keeps its own rule until step 9)",
                                         e.id.c_str(), sp.s0, sp.s1, p.x, p.y, carriesOn.c_str()));
            else if (h == boxHeldBy.end())
                m.problems.push_back(fmt("the paint found a junction box on %s (stations %.1f to %.1f) at (%.1f, %.1f) that no stated junction holds",
                                         e.id.c_str(), sp.s0, sp.s1, p.x, p.y));
            else if (h->second.size() > 1) {
                std::string names;
                for (uint32_t j : h->second) names += (names.empty() ? "" : ", ") + m.junctions[j].id;
                m.problems.push_back(fmt("the paint made one junction box on %s (stations %.1f to %.1f) at (%.1f, %.1f) of stated junctions %s",
                                         e.id.c_str(), sp.s0, sp.s1, p.x, p.y, names.c_str()));
            }
        }
    for (const ModelJunction& J : m.junctions)
        for (std::size_t k = 0; k < J.arms.size(); ++k)
            if (!J.arms[k].street.valid())
                m.problems.push_back(fmt("junction %s: arm %zu (road %s at %.1f) names no street of the road file", J.id.c_str(), k,
                                         J.arms[k].road.c_str(), J.arms[k].station));
    return m;
}

// ---- the questions ----------------------------------------------------------------------------------------------------

NearestOnRoad nearestOnRoad(const RoadModel& m, const Vec2& p, Real within) {
    NearestOnRoad out;
    double best = within;
    for (std::size_t li = 0; li < m.lanes.size(); ++li) {
        const ModelLane& L = m.lanes[li];
        if (L.streets.empty() || L.curve.count < 2) continue;
        for (uint32_t k = L.curve.first; k + 1 < L.curve.end(); ++k) {
            const Vec2 a = m.points[k], b = m.points[k + 1];
            const Vec2 ab = b - a;
            const double l2 = dot(ab, ab);
            const double t = l2 > 1e-18 ? std::clamp(dot(p - a, ab) / l2, 0.0, 1.0) : 0.0;
            const double d = (p - (a + ab * t)).length();
            if (d >= best) continue;
            const Real s0 = m.spineStations[k], s1 = m.spineStations[k + 1];
            if (!stated(s0) || !stated(s1)) continue;
            best = d;
            out.lane = ModelLaneId(static_cast<uint32_t>(li));
            out.station = s0 + (s1 - s0) * t;
            out.distance = d;
            out.height = m.heights[k] + (m.heights[k + 1] - m.heights[k]) * t;
        }
    }
    if (out.lane.valid()) {
        const ModelLane& L = m.lanes[out.lane.index];
        for (const ModelLaneSpan& sp : L.streets)
            if (out.station >= sp.from - 1e-9 && out.station <= sp.to + 1e-9) out.street = sp.street;
        if (!out.street.valid()) out.lane = ModelLaneId();   // on the lane, but inside a junction: no street there
    }
    return out;
}

std::vector<ModelStreetId> streetsAt(const RoadModel& m, ModelJunctionId j) {
    std::vector<ModelStreetId> out;
    if (!j.valid() || j.index >= m.junctions.size()) return out;
    const ModelJunction& J = m.junctions[j.index];
    std::vector<std::size_t> order(J.arms.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const bool ha = stated(J.arms[a].heading), hb = stated(J.arms[b].heading);
        if (ha != hb) return ha;
        return ha && J.arms[a].heading < J.arms[b].heading;
    });
    for (std::size_t i : order)
        if (J.arms[i].street.valid()) out.push_back(J.arms[i].street);
    return out;
}

std::vector<ModelLaneId> lanesOf(const RoadModel& m, ModelStreetId s) {
    if (!s.valid() || s.index >= m.streets.size()) return {};
    return m.streets[s.index].lanes;
}

LeadsTo leadsTo(const RoadModel& m, ModelLaneId lane) {
    LeadsTo out;
    if (!lane.valid() || lane.index >= m.lanes.size()) return out;
    out.junction = m.lanes[lane.index].atEnd;
    for (std::size_t c = 0; c < m.connections.size(); ++c)
        if (m.connections[c].from == lane) out.connections.push_back(static_cast<uint32_t>(c));
    return out;
}

// ---- attachments ------------------------------------------------------------------------------------------------------

const AttachmentKind* AttachmentKinds::find(const std::string& name) const {
    for (const AttachmentKind& k : kinds)
        if (k.name == name) return &k;
    return nullptr;
}

std::string AttachmentKinds::use(const std::string& role) const {
    for (const std::pair<std::string, std::string>& u : uses)
        if (u.first == role) return u.second;
    return std::string();
}

bool AttachmentKind::standsFor(const std::string& control) const {
    return controls.empty() || std::find(controls.begin(), controls.end(), control) != controls.end();
}

AttachmentKinds parseAttachmentKinds(const nlohmann::json& doc) {
    AttachmentKinds out;
    if (doc.contains("kinds") && doc["kinds"].is_object())
        for (const auto& [name, k] : doc["kinds"].items()) {
            if (!k.is_object()) continue;
            AttachmentKind kind;
            kind.name = name;
            kind.on = k.value("on", std::string());
            kind.names = k.value("names", false);
            if (k.contains("controls") && k["controls"].is_array())
                for (const nlohmann::json& c : k["controls"])
                    if (c.is_string()) kind.controls.push_back(c.get<std::string>());
            if (k.contains("sign") && k["sign"].is_object()) kind.sign = k["sign"];
            if (kind.on == "street" || kind.on == "junction" || kind.on == "arm") out.kinds.push_back(kind);
        }
    if (doc.contains("uses") && doc["uses"].is_object())
        for (const auto& [role, kind] : doc["uses"].items())
            if (kind.is_string()) out.uses.emplace_back(role, kind.get<std::string>());
    return out;
}

const AttachmentKinds& attachmentKinds() {
    static const AttachmentKinds kinds = [] {
        std::ifstream in(assetPath("assets/data/road_attachments.json"));
        nlohmann::json doc;
        if (in) {
            try {
                in >> doc;
            } catch (const std::exception& e) {
                LOG_WARN << "[roads] road_attachments.json: " << e.what();
                doc = nlohmann::json();
            }
        }
        AttachmentKinds k = parseAttachmentKinds(doc);
        if (k.kinds.empty()) {
            // the shipped file's kinds, so a stripped install still names its streets
            LOG_WARN << "[roads] no usable assets/data/road_attachments.json: the built-in attachment kinds";
            k = parseAttachmentKinds(nlohmann::json::parse(R"({"kinds": {"street_name": {"on": "street", "names": true},
                "name_sign": {"on": "arm"}, "signal_pole": {"on": "arm", "controls": ["signal"]},
                "signal_head": {"on": "arm", "controls": ["signal"]}, "street_lamp": {"on": "street"}},
                "uses": {"street_name": "street_name", "name_sign": "name_sign", "signal_pole": "signal_pole",
                "signal_head": "signal_head", "street_lamp": "street_lamp"}})"));
        }
        return k;
    }();
    return kinds;
}

bool attach(RoadModel& m, ModelAttachment a, const AttachmentKinds& kinds, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return false;
    };
    const AttachmentKind* k = kinds.find(a.kind);
    if (!k) return fail("no attachment kind \"" + a.kind + "\" (assets/data/road_attachments.json)");
    if (k->on == "street") {
        if (!a.street.valid() || a.street.index >= m.streets.size()) return fail("a " + a.kind + " attaches to a street, and names none of the model's");
        a.junction = ModelJunctionId();
        a.arm = -1;
    } else if (k->on == "junction") {
        if (!a.junction.valid() || a.junction.index >= m.junctions.size()) return fail("a " + a.kind + " attaches to a junction, and names none of the model's");
        a.arm = -1;
        a.street = ModelStreetId();
    } else {
        if (!a.junction.valid() || a.junction.index >= m.junctions.size() || a.arm < 0 ||
            static_cast<std::size_t>(a.arm) >= m.junctions[a.junction.index].arms.size())
            return fail("a " + a.kind + " attaches to an arm of a junction, and names none of the model's");
        a.street = m.junctions[a.junction.index].arms[static_cast<std::size_t>(a.arm)].street;
    }
    m.attachments.push_back(std::move(a));
    return true;
}

bool attach(RoadModel& m, ModelAttachment a, std::string* err) { return attach(m, std::move(a), attachmentKinds(), err); }

std::vector<uint32_t> attachedToStreet(const RoadModel& m, ModelStreetId s) {
    std::vector<uint32_t> out;
    for (std::size_t i = 0; i < m.attachments.size(); ++i)
        if (s.valid() && m.attachments[i].street == s) out.push_back(static_cast<uint32_t>(i));
    return out;
}

std::vector<uint32_t> attachedToJunction(const RoadModel& m, ModelJunctionId j) {
    std::vector<uint32_t> out;
    for (std::size_t i = 0; i < m.attachments.size(); ++i)
        if (j.valid() && m.attachments[i].junction == j) out.push_back(static_cast<uint32_t>(i));
    return out;
}

std::string streetName(const RoadModel& m, ModelStreetId s, const AttachmentKinds& kinds) {
    for (uint32_t i : attachedToStreet(m, s)) {
        const ModelAttachment& a = m.attachments[i];
        const AttachmentKind* k = kinds.find(a.kind);
        if (k && k->names && a.junction == ModelJunctionId()) return a.text;
    }
    return std::string();
}

std::vector<NamedArm> namedArmsAt(const RoadModel& m, ModelJunctionId j) {
    std::vector<NamedArm> out;
    if (!j.valid() || j.index >= m.junctions.size()) return out;
    const ModelJunction& J = m.junctions[j.index];
    for (std::size_t k = 0; k < J.arms.size(); ++k) {
        const ModelArm& a = J.arms[k];
        if (!a.street.valid() || !stated(a.heading)) continue;
        const ModelStreet& S = m.streets[a.street.index];
        const std::string name = streetName(m, a.street);
        NamedArm n;
        if (name.empty() || !armMouth(m, J, k, n.point)) continue;
        n.arm = static_cast<int>(k);
        n.street = a.street;
        n.dir = Vec2(std::cos(a.heading * DEG_TO_RAD), std::sin(a.heading * DEG_TO_RAD));
        n.travelWidth = S.travelWidth();
        n.name = name;
        out.push_back(n);
    }
    std::stable_sort(out.begin(), out.end(), [&](const NamedArm& x, const NamedArm& y) {
        return J.arms[static_cast<std::size_t>(x.arm)].heading < J.arms[static_cast<std::size_t>(y.arm)].heading;
    });
    return out;
}

bool armMouth(const RoadModel& m, const ModelJunction& J, std::size_t arm, Vec2& out) {
    if (arm >= J.arms.size()) return false;
    const ModelArm& a = J.arms[arm];
    if (!a.street.valid() || a.street.index >= m.streets.size()) return false;
    const Range& L = m.streets[a.street.index].line;
    if (L.count < 2) return false;
    out = a.spineIn ? m.linePoints[L.end() - 1] : m.linePoints[L.first];   // its street's end at the junction
    return true;
}

ModelStreetId nearestStreet(const RoadModel& m, const Vec2& p, Real reach, Real* distance) {
    Real best = reach;
    ModelStreetId out;
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        const Range& L = m.streets[s].line;
        if (L.count < 2) continue;
        Real d = best;
        for (uint32_t k = L.first; k + 1 < L.end(); ++k) d = std::min(d, distToSegment(p, m.linePoints[k], m.linePoints[k + 1]));
        if (d >= best) continue;
        best = d;
        out = ModelStreetId(static_cast<uint32_t>(s));
    }
    if (distance) *distance = out.valid() ? best : kNotStated;
    return out;
}

std::string streetNameNear(const RoadModel& m, const Vec2& p, Real reach, Real* distance) {
    Real best = reach;
    std::string out;
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        const Range& L = m.streets[s].line;
        if (L.count < 2) continue;
        Real d = best;
        for (uint32_t k = L.first; k + 1 < L.end(); ++k) d = std::min(d, distToSegment(p, m.linePoints[k], m.linePoints[k + 1]));
        if (d >= best) continue;
        const std::string name = streetName(m, ModelStreetId(static_cast<uint32_t>(s)));
        if (name.empty()) continue;
        best = d;
        out = name;
    }
    if (distance) *distance = out.empty() ? kNotStated : best;
    return out;
}

ModelJunctionId junctionNamed(const RoadModel& m, const std::string& id) {
    for (std::size_t i = 0; i < m.junctions.size(); ++i)
        if (m.junctions[i].id == id) return ModelJunctionId(static_cast<uint32_t>(i));
    return ModelJunctionId();
}
ModelStreetId streetNamed(const RoadModel& m, const std::string& id) {
    for (std::size_t i = 0; i < m.streets.size(); ++i)
        if (m.streets[i].id == id) return ModelStreetId(static_cast<uint32_t>(i));
    return ModelStreetId();
}
ModelLaneId laneNamed(const RoadModel& m, const std::string& id) {
    for (std::size_t i = 0; i < m.lanes.size(); ++i)
        if (m.lanes[i].id == id) return ModelLaneId(static_cast<uint32_t>(i));
    return ModelLaneId();
}

// ---- the regions enclosed by streets (a look-up: nothing here is kept) ---------------------------------------------------

namespace {

// A junction whose walked arms' mouths lie further apart than this (metres) was spread over several points by the plan.
constexpr Real kSpreadMouths = 1.0;
// How far past its furthest mouth a spread junction's circle reaches (metres): where its arms' order round it is read.
constexpr Real kSpreadMargin = 5.0;
// Two walked streets whose centre lines cross within this of an end of either meet there (metres): not a crossing.
constexpr Real kCrossingEndReach = 1.0;
// A face of the walk under this (square metres, either sign) encloses nothing.
constexpr Real kNoArea = 0.5;
// A dead end within this of a passed-over street's centre line is said to end beside it (metres; words only).
constexpr Real kDeadEndBeside = 60.0;

// One step of the walk: a street and the way it is walked (forward: from its `from` to its `to`).
uint32_t stepOf(uint32_t street, bool forward) { return 2 * street + (forward ? 0u : 1u); }

Real degreesOf(const Vec2& d) {
    const Real a = std::atan2(d.y, d.x) / DEG_TO_RAD;
    return a < 0 ? a + 360 : a;
}

void appendStreetLine(const RoadModel& m, const ModelStreet& S, bool forward, std::vector<Vec2>& out) {
    for (uint32_t i = 0; i < S.line.count; ++i) {
        const Vec2& p = m.linePoints[forward ? S.line.first + i : S.line.end() - 1 - i];
        if (out.empty() || (out.back() - p).length() > 1e-6) out.push_back(p);
    }
}

std::string regionStreet(const RoadModel& m, ModelStreetId s) { return s.valid() ? m.streets[s.index].id : std::string("(none)"); }

}  // namespace

EnclosedRegions enclosedRegions(const RoadModel& m, const RegionWalk& rules) {
    EnclosedRegions out;
    const uint32_t S = static_cast<uint32_t>(m.streets.size());
    auto note = [](std::vector<RegionWalkNote>& list, ModelStreetId s, ModelJunctionId j, const Vec2& at, const std::string& why,
                   ModelStreetId other = ModelStreetId()) {
        RegionWalkNote n;
        n.street = s;
        n.other = other;
        n.junction = j;
        n.at = at;
        n.why = why;
        list.push_back(std::move(n));
    };
    auto endName = [](int e) { return e == 0 ? "`from`" : "`to`"; };

    // ---- the streets walked: every street with a centre line, but those of a passed-over class -------------------------
    std::vector<char> walked(S, 0);
    for (uint32_t s = 0; s < S; ++s) {
        const ModelStreet& st = m.streets[s];
        if (std::find(rules.passOver.begin(), rules.passOver.end(), st.section.klass) != rules.passOver.end()) {
            out.passedOver.push_back(ModelStreetId(s));
            continue;
        }
        if (st.line.count < 2) {
            note(out.problems, ModelStreetId(s), ModelJunctionId(), Vec2(), "it has no centre line: not walked");
            continue;
        }
        walked[s] = 1;
        ++out.walked;
    }
    auto streetEnd = [&](uint32_t s, int e) {   // a street's centre line at its end e (0: `from`, 1: `to`)
        const Range& L = m.streets[s].line;
        return e == 0 ? m.linePoints[L.first] : m.linePoints[L.end() - 1];
    };

    // ---- each walked street end's arm, as the junctions list them (the arms lead the walk) ------------------------------
    struct EndArm {
        int32_t junction = -1, arm = -1;
    };
    std::vector<std::array<EndArm, 2>> endArm(S);
    for (uint32_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        for (uint32_t a = 0; a < J.arms.size(); ++a) {
            const ModelArm& arm = J.arms[a];
            if (!arm.street.valid() || arm.street.index >= S || !walked[arm.street.index]) continue;
            const uint32_t s = arm.street.index;
            const int e = arm.spineIn ? 1 : 0;
            if (endArm[s][e].junction >= 0) {
                note(out.problems, arm.street, ModelJunctionId(j), streetEnd(s, e),
                     fmt("arm %u of junction %s and arm %d of junction %s are both its %s end: the first leads the walk", a, J.id.c_str(),
                         endArm[s][e].arm, m.junctions[static_cast<std::size_t>(endArm[s][e].junction)].id.c_str(), endName(e)));
                continue;
            }
            endArm[s][e] = EndArm{static_cast<int32_t>(j), static_cast<int32_t>(a)};
            const ModelJunctionId named = m.streets[s].junctions[static_cast<std::size_t>(e)];
            if (named != ModelJunctionId(j))
                note(out.problems, arm.street, ModelJunctionId(j), streetEnd(s, e),
                     fmt("arm %u of junction %s is its %s end, which names %s: the arm leads the walk", a, J.id.c_str(), endName(e),
                         named.valid() ? ("junction " + m.junctions[named.index].id).c_str() : "no junction"));
        }
    }
    for (uint32_t s = 0; s < S; ++s) {
        if (!walked[s]) continue;
        for (int e = 0; e < 2; ++e) {
            const ModelJunctionId named = m.streets[s].junctions[static_cast<std::size_t>(e)];
            if (named.valid() && endArm[s][e].junction < 0)
                note(out.problems, ModelStreetId(s), named, streetEnd(s, e),
                     fmt("its %s end names junction %s, which lists no arm for it: walked round there", endName(e), m.junctions[named.index].id.c_str()));
        }
    }

    // ---- each junction's walked arms in order round it --------------------------------------------------------------------
    std::vector<std::vector<uint32_t>> order(m.junctions.size());
    std::vector<std::vector<int32_t>> posOf(m.junctions.size());
    for (uint32_t j = 0; j < m.junctions.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        posOf[j].assign(J.arms.size(), -1);
        std::vector<uint32_t> arms;
        std::vector<Vec2> mouths;
        for (uint32_t a = 0; a < J.arms.size(); ++a) {
            const ModelArm& arm = J.arms[a];
            if (!arm.street.valid() || arm.street.index >= S || !walked[arm.street.index]) continue;
            const EndArm& ea = endArm[arm.street.index][arm.spineIn ? 1 : 0];
            if (ea.junction != static_cast<int32_t>(j) || ea.arm != static_cast<int32_t>(a)) continue;
            arms.push_back(a);
            mouths.push_back(streetEnd(arm.street.index, arm.spineIn ? 1 : 0));
        }
        if (arms.empty()) continue;
        Real spread = 0, reach = 0;
        for (std::size_t i = 0; i < mouths.size(); ++i) {
            reach = std::max(reach, (mouths[i] - J.at).length());
            for (std::size_t k = i + 1; k < mouths.size(); ++k) spread = std::max(spread, (mouths[i] - mouths[k]).length());
        }
        const bool spreadOut = spread > kSpreadMouths;
        std::vector<Real> byHeading(arms.size()), key(arms.size());
        for (std::size_t i = 0; i < arms.size(); ++i) {
            const ModelArm& arm = J.arms[arms[i]];
            const uint32_t s = arm.street.index;
            const bool atFrom = !arm.spineIn;   // the street leaves the junction from its `from` end
            const Range& L = m.streets[s].line;
            auto lineAt = [&](uint32_t k) { return m.linePoints[atFrom ? L.first + k : L.end() - 1 - k]; };   // k-th point outward
            // its way out: the stated heading, else its centre line's first step
            byHeading[i] = stated(arm.heading) ? std::fmod(arm.heading + 360.0, 360.0) : degreesOf(lineAt(1) - lineAt(0));
            key[i] = byHeading[i];
            if (!spreadOut) continue;
            // a spread junction: where the arm's street leaves the circle holding every mouth (its far end if it never does)
            const Real R = reach + kSpreadMargin;
            Vec2 exitAt = lineAt(L.count - 1);
            for (uint32_t k = 0; k + 1 < L.count; ++k) {
                const Vec2 p = lineAt(k), q = lineAt(k + 1);
                if ((q - J.at).length() < R) continue;
                const Vec2 d = q - p, f = p - J.at;
                const Real A = dot(d, d), B = 2 * dot(f, d), C = dot(f, f) - R * R;
                const Real disc = B * B - 4 * A * C;
                const Real t = A > 1e-12 && disc >= 0 ? std::clamp((-B + std::sqrt(disc)) / (2 * A), Real(0), Real(1)) : Real(1);
                exitAt = p + d * t;
                break;
            }
            key[i] = degreesOf(exitAt - J.at);
        }
        std::vector<uint32_t> sorted(arms.size()), headingSorted(arms.size());
        for (uint32_t i = 0; i < sorted.size(); ++i) sorted[i] = headingSorted[i] = i;
        std::stable_sort(sorted.begin(), sorted.end(), [&](uint32_t x, uint32_t y) { return key[x] < key[y]; });
        std::stable_sort(headingSorted.begin(), headingSorted.end(), [&](uint32_t x, uint32_t y) { return byHeading[x] < byHeading[y]; });
        if (spreadOut) {
            ++out.spreadJunctions;
            // the same cyclic order? (turn the heading order to start where the other does)
            const std::vector<uint32_t>::iterator at = std::find(headingSorted.begin(), headingSorted.end(), sorted.front());
            std::rotate(headingSorted.begin(), at, headingSorted.end());
            if (headingSorted != sorted) ++out.spreadOrderDiffers;
        }
        for (uint32_t i = 0; i < sorted.size(); ++i) {
            order[j].push_back(arms[sorted[i]]);
            posOf[j][arms[sorted[i]]] = static_cast<int32_t>(i);
        }
    }

    // ---- where a walked street carries on into another (no junction there) ----------------------------------------------
    std::vector<std::array<int64_t, 2>> carry(S, std::array<int64_t, 2>{-1, -1});   // the step it carries on into, at each end
    for (uint32_t s = 0; s < S; ++s) {
        if (!walked[s]) continue;
        for (int e = 0; e < 2; ++e) {
            if (endArm[s][e].junction >= 0) continue;
            const ModelStreetId t = m.streets[s].carriesOn[static_cast<std::size_t>(e)];
            if (!t.valid() || t.index >= S) continue;
            const Vec2 at = streetEnd(s, e);
            if (!walked[t.index]) {
                note(out.turnedRound, ModelStreetId(s), ModelJunctionId(), at,
                     "it carries on into " + m.streets[t.index].id + " (" + m.streets[t.index].section.roadClass + ", passed over): walked round there", t);
                continue;
            }
            int back = -1;
            for (int k = 0; k < 2; ++k)
                if (m.streets[t.index].carriesOn[static_cast<std::size_t>(k)] == ModelStreetId(s) && !(t.index == s && k == e)) back = k;
            if (back < 0) {
                note(out.problems, ModelStreetId(s), ModelJunctionId(), at, "it carries on into " + m.streets[t.index].id + ", which does not name it back: walked round there", t);
                continue;
            }
            if (endArm[t.index][back].junction >= 0) {
                note(out.problems, ModelStreetId(s), ModelJunctionId(static_cast<uint32_t>(endArm[t.index][back].junction)), at,
                     "it carries on into " + m.streets[t.index].id + ", whose end there is an arm of a junction: walked round there", t);
                continue;
            }
            carry[s][static_cast<std::size_t>(e)] = stepOf(t.index, back == 0);
        }
    }

    // ---- the dead ends (walked round), each with the passed-over street passing nearest it, if one is beside it ---------
    for (uint32_t s = 0; s < S; ++s) {
        if (!walked[s]) continue;
        for (int e = 0; e < 2; ++e) {
            if (endArm[s][e].junction >= 0 || carry[s][static_cast<std::size_t>(e)] >= 0 || m.streets[s].carriesOn[static_cast<std::size_t>(e)].valid())
                continue;   // a junction, a carry-on, or a carry-on already said (turnedRound, problems)
            if (m.streets[s].junctions[static_cast<std::size_t>(e)].valid()) continue;   // said in problems
            const Vec2 at = streetEnd(s, e);
            ModelStreetId beside;
            Real near = kDeadEndBeside;
            for (ModelStreetId p : out.passedOver) {
                const Range& L = m.streets[p.index].line;
                for (uint32_t k = L.first; k + 1 < L.end(); ++k) {
                    const Real d = distToSegment(at, m.linePoints[k], m.linePoints[k + 1]);
                    if (d < near) {
                        near = d;
                        beside = p;
                    }
                }
            }
            note(out.deadEnds, ModelStreetId(s), ModelJunctionId(), at,
                 beside.valid() ? fmt("a dead end %.0f m from %s (%s, passed over): walked round", near, m.streets[beside.index].id.c_str(),
                                      m.streets[beside.index].section.roadClass.c_str())
                                : std::string("a dead end: walked round"),
                 beside);
        }
    }

    // ---- the step after each step: leave by the next arm clockwise, carry on, or turn round -----------------------------
    struct Next {
        uint32_t step;
        RegionStepEnd end;
        ModelJunctionId junction;
    };
    auto after = [&](uint32_t h) {
        const uint32_t s = h / 2;
        const bool forward = h % 2 == 0;
        const int e = forward ? 1 : 0;
        const EndArm ea = endArm[s][e];
        if (ea.junction >= 0) {
            const std::vector<uint32_t>& o = order[static_cast<std::size_t>(ea.junction)];
            const std::size_t n = o.size();
            const std::size_t p = static_cast<std::size_t>(posOf[static_cast<std::size_t>(ea.junction)][static_cast<std::size_t>(ea.arm)]);
            const ModelArm& leave = m.junctions[static_cast<std::size_t>(ea.junction)].arms[o[(p + n - 1) % n]];
            return Next{stepOf(leave.street.index, !leave.spineIn), RegionStepEnd::Junction, ModelJunctionId(static_cast<uint32_t>(ea.junction))};
        }
        if (carry[s][static_cast<std::size_t>(e)] >= 0)
            return Next{static_cast<uint32_t>(carry[s][static_cast<std::size_t>(e)]), RegionStepEnd::CarriesOn, ModelJunctionId()};
        return Next{stepOf(s, !forward), RegionStepEnd::TurnsRound, ModelJunctionId()};
    };

    // ---- the faces: every step walked once; the pieces of network they belong to ----------------------------------------
    std::vector<uint32_t> parent(S);
    for (uint32_t s = 0; s < S; ++s) parent[s] = s;
    auto root = [&](uint32_t s) {
        while (parent[s] != s) s = parent[s] = parent[parent[s]];
        return s;
    };
    std::vector<char> seen(2 * static_cast<std::size_t>(S), 0);
    std::vector<EnclosedRegion> faces;
    for (uint32_t h0 = 0; h0 < 2 * S; ++h0) {
        if (!walked[h0 / 2] || seen[h0]) continue;
        EnclosedRegion f;
        uint32_t h = h0;
        do {
            seen[h] = 1;
            const Next n = after(h);
            RegionBound b;
            b.street = ModelStreetId(h / 2);
            b.spineForward = h % 2 == 0;
            b.side = b.spineForward ? RegionSide::Ccw : RegionSide::Cw;   // the land is on the walk's counter-clockwise side
            b.end = n.end;
            b.junction = n.junction;
            f.bounds.push_back(b);
            appendStreetLine(m, m.streets[h / 2], b.spineForward, f.outline);
            parent[root(h / 2)] = root(n.step / 2);
            h = n.step;
        } while (h != h0 && !seen[h]);
        if (h != h0)   // cannot happen while each step has one step after it and one before it; said if it does
            note(out.unplanar, f.bounds.front().street, ModelJunctionId(), f.outline.empty() ? Vec2() : f.outline.front(),
                 "the walk met a step it had already walked from elsewhere: it did not close");
        if (f.outline.size() >= 2 && (f.outline.front() - f.outline.back()).length() < 1e-6) f.outline.pop_back();
        f.area = signedArea(f.outline);
        for (const RegionBound& b : f.bounds)
            if (b.junction.valid() && (f.corners.empty() || f.corners.back() != b.junction)) f.corners.push_back(b.junction);
        if (f.corners.size() > 1 && f.corners.front() == f.corners.back()) f.corners.pop_back();
        faces.push_back(std::move(f));
    }

    // ---- each piece's outer face (the one walked most clockwise); the rest are regions, medians, or say where the walk
    //      went wrong ------------------------------------------------------------------------------------------------------
    std::map<uint32_t, std::vector<std::size_t>> byPiece;
    for (std::size_t i = 0; i < faces.size(); ++i) byPiece[root(faces[i].bounds.front().street.index)].push_back(i);
    out.pieces = static_cast<uint32_t>(byPiece.size());
    auto isMedian = [&](const EnclosedRegion& f) {
        std::string name;
        for (const RegionBound& b : f.bounds) {
            if (!m.streets[b.street.index].oneWay) return false;
            const std::string n = streetName(m, b.street);
            if (n.empty() || (!name.empty() && n != name)) return false;
            name = n;
        }
        return !f.bounds.empty();
    };
    std::vector<std::pair<uint32_t, std::size_t>> outerOf;   // (piece, index into out.outer)
    std::vector<uint32_t> regionPiece;
    for (const std::pair<const uint32_t, std::vector<std::size_t>>& piece : byPiece) {
        std::size_t outer = piece.second.front();
        for (std::size_t i : piece.second)
            if (faces[i].area < faces[outer].area) outer = i;
        for (std::size_t i : piece.second) {
            EnclosedRegion& f = faces[i];
            if (i == outer) {
                outerOf.emplace_back(piece.first, out.outer.size());
                out.outer.push_back(std::move(f));
            } else if (f.area > kNoArea) {
                if (isMedian(f)) {
                    out.medians.push_back(std::move(f));
                } else {
                    regionPiece.push_back(piece.first);
                    out.regions.push_back(std::move(f));
                }
            } else {
                note(out.unplanar, f.bounds.front().street, ModelJunctionId(), f.outline.empty() ? Vec2() : f.outline.front(),
                     fmt("a face of %zu steps walked %s (%.0f m2) besides its piece's outer face: streets crossing without a junction, or "
                         "arms out of order", f.bounds.size(), f.area < 0 ? "clockwise" : "round nothing", f.area));
            }
        }
    }

    // ---- walked streets crossing with no junction there -------------------------------------------------------------------
    {
        std::vector<std::array<Vec2, 2>> box(S);
        for (uint32_t s = 0; s < S; ++s) {
            if (!walked[s]) continue;
            Vec2 lo = streetEnd(s, 0), hi = lo;
            const Range& L = m.streets[s].line;
            for (uint32_t k = L.first; k < L.end(); ++k) {
                lo = Vec2(std::min(lo.x, m.linePoints[k].x), std::min(lo.y, m.linePoints[k].y));
                hi = Vec2(std::max(hi.x, m.linePoints[k].x), std::max(hi.y, m.linePoints[k].y));
            }
            box[s] = {lo, hi};
        }
        for (uint32_t s = 0; s < S; ++s) {
            if (!walked[s]) continue;
            for (uint32_t t = s + 1; t < S; ++t) {
                if (!walked[t] || box[s][1].x < box[t][0].x || box[t][1].x < box[s][0].x || box[s][1].y < box[t][0].y || box[t][1].y < box[s][0].y)
                    continue;
                const Range& A = m.streets[s].line;
                const Range& B = m.streets[t].line;
                bool found = false;
                for (uint32_t i = A.first; i + 1 < A.end() && !found; ++i)
                    for (uint32_t k = B.first; k + 1 < B.end() && !found; ++k) {
                        Vec2 at;
                        if (!segmentsCross(m.linePoints[i], m.linePoints[i + 1], m.linePoints[k], m.linePoints[k + 1], at)) continue;
                        bool nearEnd = false;
                        for (int e = 0; e < 2; ++e)
                            nearEnd = nearEnd || (at - streetEnd(s, e)).length() < kCrossingEndReach || (at - streetEnd(t, e)).length() < kCrossingEndReach;
                        if (nearEnd) continue;
                        note(out.crossings, ModelStreetId(s), ModelJunctionId(), at, "crosses " + m.streets[t].id + " with no junction there", ModelStreetId(t));
                        found = true;
                    }
            }
        }
    }

    // ---- pieces of network standing inside another piece's region ---------------------------------------------------------
    for (const std::pair<uint32_t, std::size_t>& po : outerOf) {
        const EnclosedRegion& o = out.outer[po.second];
        const ModelStreet& first = m.streets[o.bounds.front().street.index];
        const Vec2 p = m.linePoints[first.line.first + first.line.count / 2];
        int in = -1;
        for (std::size_t r = 0; r < out.regions.size(); ++r)
            if (regionPiece[r] != po.first && pointInPolygon(out.regions[r].outline, p) && (in < 0 || out.regions[r].area < out.regions[static_cast<std::size_t>(in)].area))
                in = static_cast<int>(r);
        if (in >= 0)
            note(out.nested, o.bounds.front().street, ModelJunctionId(), p,
                 fmt("a piece of network (%zu steps round it) stands inside region %d: a hole in it", o.bounds.size(), in));
    }
    return out;
}

int regionAt(const EnclosedRegions& er, const Vec2& p) {
    int best = -1;
    for (std::size_t r = 0; r < er.regions.size(); ++r)
        if (pointInPolygon(er.regions[r].outline, p) && (best < 0 || er.regions[r].area < er.regions[static_cast<std::size_t>(best)].area))
            best = static_cast<int>(r);
    return best;
}

std::string reportEnclosedRegions(const RoadModel& m, const EnclosedRegions& er) {
    auto bounds = [&](const EnclosedRegion& f) {
        std::string s;
        for (const RegionBound& b : f.bounds) {
            s += regionStreet(m, b.street) + (b.side == RegionSide::Ccw ? " (ccw)" : " (cw)");
            s += b.end == RegionStepEnd::Junction ? " > " + m.junctions[b.junction.index].id + " > "
                 : b.end == RegionStepEnd::CarriesOn ? " > carries on > " : " > turns round > ";
        }
        return s.empty() ? s : s.substr(0, s.size() - 3);
    };
    auto notes = [&](const char* head, const std::vector<RegionWalkNote>& list) {
        std::string s = fmt("%s: %zu\n", head, list.size());
        for (const RegionWalkNote& n : list)
            s += fmt("    street %s%s at (%.1f, %.1f): %s\n", regionStreet(m, n.street).c_str(),
                     n.junction.valid() ? (", junction " + m.junctions[n.junction.index].id).c_str() : "", n.at.x, n.at.y, n.why.c_str());
        return s;
    };
    std::string passed;
    {
        std::map<std::string, uint32_t> byClass;
        for (ModelStreetId s : er.passedOver) ++byClass[m.streets[s.index].section.roadClass];
        for (const std::pair<const std::string, uint32_t>& c : byClass) passed += fmt(" %u %s;", c.second, c.first.c_str());
    }
    Real total = 0;
    for (const EnclosedRegion& r : er.regions) total += r.area;
    std::string out = fmt("THE REGIONS ENCLOSED BY STREETS OF %s (a look-up worked out from the road model; nothing here is kept)\n\n", m.name.c_str());
    out += fmt("  %u streets walked; %zu passed over (by class:%s); %u pieces of network\n", er.walked, er.passedOver.size(), passed.c_str(), er.pieces);
    out += fmt("  %zu regions (%.0f m2 in all), %zu medians (faces between the two carriageways of one road), %zu outer faces\n", er.regions.size(),
               total, er.medians.size(), er.outer.size());
    out += fmt("  junctions spread over several points: %u (%u of them ordered differently by their headings alone)\n\n", er.spreadJunctions,
               er.spreadOrderDiffers);
    out += notes("Where what is stated does not agree with itself (the walk followed the arms, turned round where nothing led on)", er.problems);
    out += notes("Where a street carries on into a passed-over street (walked round there, as a dead end)", er.turnedRound);
    out += notes("Dead ends (walked round: they split nothing, close nothing)", er.deadEnds);
    out += notes("Walked streets crossing with no junction (the regions there overlap)", er.crossings);
    out += notes("Faces the walk could not close as regions (walked clockwise or round nothing, besides an outer face)", er.unplanar);
    out += notes("Pieces of network inside another piece's region", er.nested);
    out += "\nRegions (bounds in order round each, counter-clockwise; (ccw)/(cw): the side of the street, by its from-to direction, "
           "the region lies on; then how the step ends):\n";
    for (std::size_t r = 0; r < er.regions.size(); ++r) {
        const Vec2 c = interiorPoint(er.regions[r].outline);
        out += fmt("  region %zu: %.0f m2 at (%.1f, %.1f), %zu corners: ", r, er.regions[r].area, c.x, c.y, er.regions[r].corners.size());
        out += bounds(er.regions[r]) + "\n";
    }
    out += "Medians:\n";
    for (const EnclosedRegion& f : er.medians) out += fmt("  %.0f m2: ", f.area) + bounds(f) + "\n";
    out += "Outer faces (one per piece of network):\n";
    for (const EnclosedRegion& f : er.outer) out += fmt("  %zu steps, %.0f m2 (clockwise)\n", f.bounds.size(), f.area);
    return out;
}

std::string enclosedRegionsSvg(const RoadModel& m, const EnclosedRegions& er, const std::vector<std::vector<Vec2>>* overlay) {
    double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
    for (const Vec2& p : m.linePoints) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
    if (m.linePoints.empty()) { x0 = y0 = 0; x1 = y1 = 1; }
    const double pad = 30, w = x1 - x0 + 2 * pad, h = y1 - y0 + 2 * pad;
    std::string out = fmt("<svg xmlns='http://www.w3.org/2000/svg' viewBox='%.1f %.1f %.1f %.1f' font-family='sans-serif'>\n", x0 - pad, -(y1 + pad), w, h);
    out += fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#fbfaf6'/>\n<g transform='scale(1,-1)'>\n", x0 - pad, -(y1 + pad), w, h);
    auto colour = [](std::size_t i) {   // each region its own: hues a golden angle apart, two lightnesses alternating
        const double hue = std::fmod(static_cast<double>(i) * 137.508, 360.0) / 60.0, l = i % 2 ? 0.68 : 0.78, sat = 0.55;
        const double c = (1 - std::fabs(2 * l - 1)) * sat, x = c * (1 - std::fabs(std::fmod(hue, 2.0) - 1)), mm = l - c / 2;
        double r = 0, g = 0, b = 0;
        if (hue < 1) { r = c; g = x; } else if (hue < 2) { r = x; g = c; } else if (hue < 3) { g = c; b = x; }
        else if (hue < 4) { g = x; b = c; } else if (hue < 5) { r = x; b = c; } else { r = c; b = x; }
        return fmt("#%02x%02x%02x", static_cast<int>((r + mm) * 255), static_cast<int>((g + mm) * 255), static_cast<int>((b + mm) * 255));
    };
    auto ring = [&](const std::vector<Vec2>& pts, const std::string& style) {
        out += "<polygon " + style + " points='";
        for (const Vec2& p : pts) out += fmt("%.1f,%.1f ", p.x, p.y);
        out += "'/>\n";
    };
    for (std::size_t r = 0; r < er.regions.size(); ++r) ring(er.regions[r].outline, "fill='" + colour(r) + "' stroke='none'");
    for (const EnclosedRegion& f : er.medians) ring(f.outline, "fill='#999' fill-opacity='0.5' stroke='none'");
    // the streets: walked ones by their travel width, passed-over ones dashed
    std::vector<char> passed(m.streets.size(), 0);
    for (ModelStreetId s : er.passedOver) passed[s.index] = 1;
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        const ModelStreet& S = m.streets[s];
        if (S.line.count < 2) continue;
        out += passed[s] ? "<polyline fill='none' stroke='#777' stroke-width='3' stroke-dasharray='10 6' stroke-opacity='0.8' points='"
                         : fmt("<polyline fill='none' stroke='#333' stroke-width='%.1f' stroke-opacity='0.85' stroke-linecap='round' points='",
                               std::max(2.0, 0.5 * S.travelWidth()));
        for (uint32_t k = S.line.first; k < S.line.end(); ++k) out += fmt("%.1f,%.1f ", m.linePoints[k].x, m.linePoints[k].y);
        out += "'/>\n";
    }
    if (overlay)
        for (const std::vector<Vec2>& r : *overlay) ring(r, "fill='none' stroke='#000' stroke-width='1' stroke-dasharray='4 3'");
    for (const ModelJunction& J : m.junctions) out += fmt("<circle cx='%.1f' cy='%.1f' r='3' fill='#c33'/>\n", J.at.x, J.at.y);
    for (const RegionWalkNote& n : er.crossings) out += fmt("<circle cx='%.1f' cy='%.1f' r='9' fill='none' stroke='#e00' stroke-width='3'/>\n", n.at.x, n.at.y);
    out += "</g>\n";
    // labels upright: each street's id at the middle of its centre line, each region's number inside it
    for (std::size_t s = 0; s < m.streets.size(); ++s) {
        const ModelStreet& S = m.streets[s];
        if (S.line.count < 2) continue;
        const uint32_t k = S.line.first + (S.line.count - 1) / 2;
        const Vec2 p = (m.linePoints[k] + m.linePoints[k + 1]) * 0.5;
        out += fmt("<text x='%.1f' y='%.1f' font-size='7' fill='%s' text-anchor='middle' stroke='#fff' stroke-width='1.6' paint-order='stroke'>%s</text>\n",
                   p.x, -p.y + 2.5, passed[s] ? "#666" : "#123", S.id.c_str());
    }
    for (std::size_t r = 0; r < er.regions.size(); ++r) {
        const Vec2 c = interiorPoint(er.regions[r].outline);
        out += fmt("<text x='%.1f' y='%.1f' font-size='12' font-weight='bold' fill='#000' text-anchor='middle'>R%zu</text>\n", c.x, -c.y + 4, r);
    }
    out += "</svg>\n";
    return out;
}

// ---- in words and on a map --------------------------------------------------------------------------------------------

namespace {
std::string num(Real v) { return stated(v) ? fmt("%.2f", v) : std::string("-"); }
std::string junctionName(const RoadModel& m, ModelJunctionId j) { return j.valid() ? m.junctions[j.index].id : std::string("(none)"); }
std::string streetIdOf(const RoadModel& m, ModelStreetId s) { return s.valid() ? m.streets[s.index].id : std::string("(none)"); }
const char* connectionName(ConnectionKind k) {
    return k == ConnectionKind::Diverge ? "leaves" : k == ConnectionKind::Merge ? "merges into" : "joins";
}
const char* rampEndName(RampEndKind k) { return k == RampEndKind::Anchored ? "anchored" : k == RampEndKind::OnStreet ? "on a street" : "free"; }
}  // namespace

std::string dumpRoadModel(const RoadModel& m) {
    std::string out = fmt("road model %s: %zu junctions, %zu streets, %zu lanes, %zu connections, %zu ramps, %zu crossings\n", m.name.c_str(),
                          m.junctions.size(), m.streets.size(), m.lanes.size(), m.connections.size(), m.ramps.size(), m.crossings.size());
    for (const ModelJunction& J : m.junctions) {
        out += fmt("junction %s at (%.2f, %.2f) height %s control %s\n", J.id.c_str(), J.at.x, J.at.y, num(J.height).c_str(),
                   J.control.empty() ? "(not stated)" : J.control.c_str());
        for (std::size_t k = 0; k < J.arms.size(); ++k) {
            const ModelArm& a = J.arms[k];
            out += fmt("  arm %zu: street %s (road %s, station %.2f, spine %s) heading %s, box edge %s, stop line %s", k,
                       streetIdOf(m, a.street).c_str(), a.road.c_str(), a.station, a.spineIn ? "in" : "out", num(a.heading).c_str(),
                       num(a.boxEdge).c_str(), num(a.stopLine).c_str());
            for (const std::string& reg : a.regulations) out += " [" + reg + "]";
            out += "\n";
        }
        for (const std::array<uint32_t, 2>& t : J.through) out += fmt("  through: arms %u and %u\n", t[0], t[1]);
        for (const ModelStretch& st : J.inside) out += fmt("  inside: %s %.2f to %.2f\n", st.road.c_str(), st.from, st.to);
        for (const ModelStretch& st : J.boxes) out += fmt("  paint box: %s %.2f to %.2f\n", st.road.c_str(), st.from, st.to);
    }
    for (const ModelStreet& S : m.streets) {
        out += fmt("street %s: road %s %.2f to %.2f, junctions %s -> %s, %s %d+%d lanes of %.2f m%s%s, limit %s, %zu lanes:",
                   S.id.c_str(), S.road.c_str(), S.from, S.to, junctionName(m, S.junctions[0]).c_str(), junctionName(m, S.junctions[1]).c_str(),
                   S.section.roadClass.c_str(), S.section.lanesForward, S.section.lanesBackward, S.section.laneWidth, S.oneWay ? " one-way" : "",
                   S.section.parking > 0 ? fmt(", kerb parking %.2f m", S.section.parking).c_str() : "",
                   S.speedLimit > 0 ? fmt("%.2f", S.speedLimit).c_str() : "-", S.lanes.size());
        for (ModelLaneId l : S.lanes) out += " " + m.lanes[l.index].id;
        {
            const std::string name = streetName(m, ModelStreetId(static_cast<uint32_t>(&S - m.streets.data())));
            if (!name.empty()) out += "; named \"" + name + "\"";
        }
        out += "\n";
    }
    for (const ModelLane& L : m.lanes) {
        out += fmt("lane %s: %s, dir %+d, slot %d, %.2f m wide, spine %.2f to %.2f, %u points, from %s to %s, streets", L.id.c_str(), L.kind.c_str(),
                   L.dir, L.slot, L.width, L.spineFrom, L.spineTo, L.curve.count, junctionName(m, L.atStart).c_str(), junctionName(m, L.atEnd).c_str());
        for (const ModelLaneSpan& sp : L.streets) out += fmt(" %s[%.2f..%.2f]", streetIdOf(m, sp.street).c_str(), sp.from, sp.to);
        out += "\n";
    }
    for (const ModelConnection& c : m.connections)
        out += fmt("connection: %s %s %s at host station %.2f (length %.1f, taper %.1f)%s\n", m.lanes[c.from.index].id.c_str(),
                   connectionName(c.kind), m.lanes[c.to.index].id.c_str(), c.hostStation, c.length, c.taper,
                   c.ramp.empty() ? "" : (" ramp " + c.ramp).c_str());
    for (const ModelRamp& R : m.ramps) {
        out += "ramp " + R.edge + ":";
        for (int end = 0; end < 2; ++end) {
            const ModelRampEnd& E = R.ends[static_cast<std::size_t>(end)];
            out += fmt(" %s %s", end == 0 ? "start" : "end", rampEndName(E.kind));
            if (E.kind != RampEndKind::Free) out += " " + E.host + (E.hostLane.empty() ? "" : " lane " + E.hostLane) + " at " + num(E.hostStation);
            out += ";";
        }
        if (R.landingPlanned) out += " landing planned: " + R.landingJoins + " on " + R.landingStreet + " (" + R.landingCarriageway + ", crossing " + R.landingCrossing + ")";
        out += "\n";
    }
    for (const ModelCrossing& x : m.crossings)
        out += fmt("crossing %s x %s at (%.2f, %.2f): %s%s\n", x.a.c_str(), x.b.c_str(), x.at.x, x.at.y, x.kind.c_str(),
                   x.over.empty() ? "" : (", " + x.over + " over").c_str());
    for (const ModelAttachment& a : m.attachments)
        out += fmt("attached: %s \"%s\" on %s%s%s\n", a.kind.c_str(), a.text.c_str(),
                   a.junction.valid() ? ("junction " + m.junctions[a.junction.index].id).c_str() : "",
                   a.arm >= 0 ? fmt(" arm %d", a.arm).c_str() : "",
                   a.street.valid() ? ((a.junction.valid() ? " (street " : "street ") + m.streets[a.street.index].id + (a.junction.valid() ? ")" : "")).c_str() : "");
    return out;
}

std::string reportRoadModel(const RoadModel& m) {
    std::map<std::size_t, uint32_t> byArms;
    uint32_t controlled = 0, boxed = 0, armsStopped = 0, arms = 0;
    for (const ModelJunction& J : m.junctions) {
        ++byArms[J.arms.size()];
        controlled += J.control.empty() ? 0u : 1u;
        boxed += J.boxes.empty() ? 0u : 1u;
        for (const ModelArm& a : J.arms) {
            ++arms;
            armsStopped += stated(a.stopLine) ? 1u : 0u;
        }
    }
    uint32_t lanesOnStreets = 0, lanesWithEnds = 0, roadLanes = 0;
    for (const ModelLane& L : m.lanes) {
        if (L.edge == kNoIndex) continue;
        ++roadLanes;
        lanesOnStreets += L.streets.empty() ? 0u : 1u;
        lanesWithEnds += L.atStart.valid() || L.atEnd.valid() ? 1u : 0u;
    }
    uint32_t diverge = 0, merge = 0, joins = 0;
    for (const ModelConnection& c : m.connections) (c.kind == ConnectionKind::Diverge ? diverge : c.kind == ConnectionKind::Merge ? merge : joins)++;
    std::string out = fmt("THE ROAD MODEL OF %s\n\nWhat it holds:\n", m.name.c_str());
    out += fmt("  %zu junctions (%u with a control stated; %u holding one of the paint's boxes or more)", m.junctions.size(), controlled, boxed);
    out += "; by arms:";
    for (const std::pair<const std::size_t, uint32_t>& b : byArms) out += fmt(" %u with %zu;", b.second, b.first);
    out += fmt("\n  %u arms, %u with the paint's stop line\n", arms, armsStopped);
    out += fmt("  %zu streets\n", m.streets.size());
    out += fmt("  %zu lanes (%u on roads, %u of them on stated streets, %u reaching a junction at an end; the rest connectors)\n", m.lanes.size(),
               roadLanes, lanesOnStreets, lanesWithEnds);
    out += fmt("  %zu connections: %u ramps leaving a lane, %u ramps merging into one, %u declared connectors\n", m.connections.size(), diverge,
               merge, joins);
    out += fmt("  %zu ramps, %zu crossings\n", m.ramps.size(), m.crossings.size());
    {
        std::map<std::string, uint32_t> byKind;
        for (const ModelAttachment& a : m.attachments) ++byKind[a.kind];
        out += fmt("  %zu attachments:", m.attachments.size());
        for (const std::pair<const std::string, uint32_t>& k : byKind) out += fmt(" %u %s;", k.second, k.first.c_str());
        out += "\n";
    }
    out += m.built ? "  (planned and built layers)\n" : "  (the planned layer alone)\n";
    out += "\nNot stated yet (left empty, never worked out here):\n";
    for (const std::string& s : m.notStated) out += "  - " + s + "\n";
    out += fmt("\nWhere the build does not match what was stated (%zu):\n", m.problems.size());
    for (const std::string& p : m.problems) out += "  - " + p + "\n";
    return out;
}

std::string roadModelSvg(const RoadModel& m) {
    double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
    for (const Vec2& p : m.points) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
    if (m.points.empty()) { x0 = y0 = 0; x1 = y1 = 1; }
    const double pad = 20, w = x1 - x0 + 2 * pad, h = y1 - y0 + 2 * pad;
    std::string out = fmt("<svg xmlns='http://www.w3.org/2000/svg' viewBox='%.1f %.1f %.1f %.1f' font-family='sans-serif'>\n", x0 - pad, -(y1 + pad), w, h);
    out += fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#f6f5f0'/>\n<g transform='scale(1,-1)'>\n", x0 - pad, -(y1 + pad), w, h);
    // lanes: thin, by direction
    for (const ModelLane& L : m.lanes) {
        if (L.curve.count < 2) continue;
        out += fmt("<polyline fill='none' stroke='%s' stroke-width='0.35' stroke-opacity='0.6' points='", L.edge == kNoIndex ? "#9a6" : L.dir > 0 ? "#3b6fb6" : "#c0573e");
        for (uint32_t k = L.curve.first; k < L.curve.end(); ++k) out += fmt("%.1f,%.1f ", m.points[k].x, m.points[k].y);
        out += "'/>\n";
    }
    // junctions: a dot each
    for (const ModelJunction& J : m.junctions)
        out += fmt("<circle cx='%.1f' cy='%.1f' r='3' fill='%s' fill-opacity='0.8'/>\n", J.at.x, J.at.y, J.control.empty() ? "#222" : "#d4a017");
    out += "</g>\n";
    // labels upright: street ids at the middle of each street's first lane, junction ids beside the dot
    auto label = [&](const Vec2& p, const std::string& text, const char* colour, double size) {
        out += fmt("<text x='%.1f' y='%.1f' font-size='%.1f' fill='%s'>%s</text>\n", p.x + 3.5, -p.y - 3.5, size, colour, text.c_str());
    };
    for (const ModelStreet& S : m.streets) {
        if (S.lanes.empty()) continue;
        const ModelLane& L = m.lanes[S.lanes.front().index];
        const double mid = 0.5 * (S.from + S.to);
        for (uint32_t k = L.curve.first; k + 1 < L.curve.end(); ++k)
            if (stated(m.spineStations[k]) && m.spineStations[k] <= mid && m.spineStations[k + 1] >= mid) {
                label(m.points[k], S.id, "#555", 4.0);
                break;
            }
    }
    for (const ModelJunction& J : m.junctions) label(J.at, J.id, "#000", 6.0);
    out += "</svg>\n";
    return out;
}

// ---- saved and read back ----------------------------------------------------------------------------------------------

namespace {

// THE RECORDS, each walked field by field by one `fields` list (below) that the writer and the reader share, so the two
// cannot drift apart.
template <class T> constexpr bool isRecord() {
    return std::is_same<T, ModelArm>::value || std::is_same<T, ModelStretch>::value || std::is_same<T, ModelJunction>::value ||
           std::is_same<T, ModelSection>::value || std::is_same<T, ModelStreet>::value || std::is_same<T, ModelLaneSpan>::value ||
           std::is_same<T, ModelLane>::value || std::is_same<T, ModelConnection>::value || std::is_same<T, ModelRampEnd>::value ||
           std::is_same<T, ModelRamp>::value || std::is_same<T, ModelCrossing>::value || std::is_same<T, ModelAttachment>::value;
}

struct Writer {
    bundle::BinWriter& w;
    void operator()(const std::string& s) { w.putStr(s); }
    void operator()(const Vec2& v) { w.put(v.x); w.put(v.y); }
    void operator()(double v) { w.put(v); }
    void operator()(int v) { w.put<int32_t>(v); }
    void operator()(uint32_t v) { w.put(v); }
    void operator()(bool v) { w.put<uint8_t>(v ? 1 : 0); }
    void operator()(uint8_t v) { w.put(v); }
    template <class Tag> void operator()(const GraphIndex<Tag>& i) { w.put(i.index); }
    void operator()(const Range& r) { w.put(r.first); w.put(r.count); }
    template <class T, std::size_t N> void operator()(const std::array<T, N>& a) { for (const T& v : a) (*this)(v); }
    template <class T> void operator()(const std::vector<T>& v) {
        w.put<uint64_t>(v.size());
        for (const T& x : v) (*this)(x);
    }
    template <class T, std::enable_if_t<isRecord<T>(), int> = 0> void operator()(const T& v) {
        T copy = v;   // (the field list takes its fields by reference)
        fields(*this, copy, static_cast<T*>(nullptr));
    }
};
struct Reader {
    bundle::BinReader& r;
    void operator()(std::string& s) { r.getStr(s); }
    void operator()(Vec2& v) { r.get(v.x); r.get(v.y); }
    void operator()(double& v) { r.get(v); }
    void operator()(int& v) { int32_t x = 0; r.get(x); v = x; }
    void operator()(uint32_t& v) { r.get(v); }
    void operator()(bool& v) { uint8_t x = 0; r.get(x); v = x != 0; }
    void operator()(uint8_t& v) { r.get(v); }
    template <class Tag> void operator()(GraphIndex<Tag>& i) { r.get(i.index); }
    void operator()(Range& g) { r.get(g.first); r.get(g.count); }
    template <class T, std::size_t N> void operator()(std::array<T, N>& a) { for (T& v : a) (*this)(v); }
    template <class T> void operator()(std::vector<T>& v) {
        uint64_t n = 0;
        if (!r.get(n) || n > r.remaining()) {   // (every element takes at least a byte)
            v.clear();
            return;
        }
        v.assign(static_cast<std::size_t>(n), T());
        for (T& x : v) {
            (*this)(x);
            if (!r.ok()) return;
        }
    }
    template <class T, std::enable_if_t<isRecord<T>(), int> = 0> void operator()(T& v) { fields(*this, v, static_cast<T*>(nullptr)); }
};

template <class IO, class Arm> void fields(IO& io, Arm& a, ModelArm*) {
    io(a.street); io(a.road); io(a.station); io(a.spineIn); io(a.heading); io(a.regulations); io(a.boxEdge); io(a.stopLine);
}
template <class IO, class S> void fields(IO& io, S& s, ModelStretch*) { io(s.road); io(s.from); io(s.to); }
template <class IO, class J> void fields(IO& io, J& j, ModelJunction*) {
    io(j.id); io(j.at); io(j.height); io(j.arms); io(j.through); io(j.control); io(j.why); io(j.inside); io(j.boxes);
}
template <class IO, class S> void fields(IO& io, S& s, ModelSection*) {
    io(s.roadClass);
    uint8_t klass = static_cast<uint8_t>(s.klass);
    io(klass);
    s.klass = static_cast<RoadClass>(klass);
    io(s.lanesForward); io(s.lanesBackward); io(s.laneWidth); io(s.medianGap); io(s.median); io(s.shoulder); io(s.sidewalk);
    io(s.parking); io(s.track); io(s.footway);
}
template <class IO, class S> void fields(IO& io, S& s, ModelStreet*) {
    io(s.id); io(s.road); io(s.edge); io(s.from); io(s.to); io(s.junctions); io(s.carriesOn); io(s.section); io(s.oneWay); io(s.speedLimit);
    io(s.lotsFrom); io(s.lotsTo); io(s.frontageOf); io(s.line); io(s.lanes);
}
template <class IO, class A> void fields(IO& io, A& a, ModelAttachment*) {
    io(a.kind); io(a.street); io(a.junction); io(a.arm); io(a.station); io(a.text); io(a.at);
}
template <class IO, class S> void fields(IO& io, S& s, ModelLaneSpan*) { io(s.street); io(s.from); io(s.to); }
template <class IO, class L> void fields(IO& io, L& l, ModelLane*) {
    io(l.id); io(l.kind); io(l.edge); io(l.dir); io(l.slot); io(l.width); io(l.curve); io(l.spineFrom); io(l.spineTo); io(l.streets);
    io(l.atStart); io(l.atEnd);
}
template <class IO, class C> void fields(IO& io, C& c, ModelConnection*) {
    uint8_t kind = static_cast<uint8_t>(c.kind);
    io(kind);
    c.kind = static_cast<ConnectionKind>(kind);
    io(c.from); io(c.to); io(c.ramp); io(c.hostStation); io(c.length); io(c.taper);
}
template <class IO, class E> void fields(IO& io, E& e, ModelRampEnd*) {
    uint8_t kind = static_cast<uint8_t>(e.kind);
    io(kind);
    e.kind = static_cast<RampEndKind>(kind);
    io(e.host); io(e.hostLane); io(e.hostStation); io(e.length); io(e.taper);
}
template <class IO, class R> void fields(IO& io, R& r, ModelRamp*) {
    io(r.edge); io(r.ends); io(r.landingPlanned); io(r.landingStreet); io(r.landingJoins); io(r.landingCarriageway); io(r.landingCrossing);
}
template <class IO, class X> void fields(IO& io, X& x, ModelCrossing*) { io(x.a); io(x.b); io(x.kind); io(x.over); io(x.at); io(x.clearance); }

template <class IO, class M> void modelFields(IO& io, M& m) {
    io(m.name); io(m.built); io(m.junctions); io(m.streets); io(m.lanes); io(m.connections); io(m.ramps); io(m.crossings); io(m.points);
    io(m.along); io(m.heights); io(m.spineStations); io(m.linePoints); io(m.attachments); io(m.notStated); io(m.problems);
}

}  // namespace

void putRoadModel(bundle::BinWriter& w, const RoadModel& m) {
    w.magic(kMagic, kRoadModelFormat);
    Writer io{w};
    RoadModel copy = m;   // (the walk takes its fields by reference)
    modelFields(io, copy);
}

bool getRoadModel(bundle::BinReader& r, RoadModel& m, std::string* err) {
    m = RoadModel();
    uint32_t version = 0;
    if (!r.magic(kMagic, &version)) {
        if (err) *err = "road model: not a road model (no RMDL magic)";
        return false;
    }
    if (version != kRoadModelFormat) {
        if (err) *err = fmt("road model: layout %u, this build reads %u", version, kRoadModelFormat);
        return false;
    }
    Reader io{r};
    modelFields(io, m);
    if (!r.ok()) {
        m = RoadModel();
        if (err) *err = "road model: the bytes end early or do not read";
        return false;
    }
    return true;
}

bool operator==(const RoadModel& a, const RoadModel& b) {
    bundle::BinWriter wa, wb;
    putRoadModel(wa, a);
    putRoadModel(wb, b);
    return wa.bytes == wb.bytes;
}

// ---- its own cache file -------------------------------------------------------------------------------------------------

namespace {
constexpr const char* kModelSection = "roadmodel/model";
constexpr const char* kManifestField = "roadmodel";
constexpr const char* kCacheKind = "roadmodel";

std::string processTag() {
#ifndef _WIN32
    return std::to_string(static_cast<long>(::getpid()));
#else
    return "0";
#endif
}
bool fail(std::string* err, const std::string& why) {
    if (err) *err = why;
    return false;
}
}  // namespace

uint64_t roadModelCacheKey(uint64_t roadsKey, uint64_t layoutHash, uint32_t version) {
    bundle::Fnv k;
    k.str("roadmodel");
    k.pod(kRoadModelFormat);
    k.pod(version);
    k.pod(roadsKey);
    k.pod(layoutHash);
    return k.h;
}

std::string roadModelCachePath(const std::string& dir, uint64_t key) { return dir + "/" + bundle::hex16(key) + ".roadmodel"; }

bool writeRoadModelCache(const std::string& path, uint64_t key, const RoadModel& m, std::string* err) {
    std::error_code ec;
    const std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, ec);
    const std::string tmp = path + ".tmp-" + processTag();
    {
        bundle::BundleWriter w;
        std::string why;
        if (!w.openFile(tmp, &why)) return fail(err, "road model cache: " + why);
        bundle::BinWriter bytes;
        putRoadModel(bytes, m);
        nlohmann::json manifest;
        manifest[kManifestField] = {{"format", kRoadModelFormat}, {"key", bundle::hex16(key)}};
        if (!w.add(kModelSection, bytes.bytes) || !w.finish(manifest, &why)) {
            std::filesystem::remove(tmp, ec);
            return fail(err, "road model cache: cannot write " + tmp + (why.empty() ? std::string() : ": " + why));
        }
    }
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        std::filesystem::remove(tmp, ec);
        return fail(err, "road model cache: cannot move " + tmp + " into place");
    }
    return true;
}

bool readRoadModelCache(const std::string& path, uint64_t key, RoadModel& m, std::string* err) {
    m = RoadModel();
    std::string why;
    const std::unique_ptr<bundle::Bundle> b = bundle::Bundle::open(path, &why);
    if (!b) return fail(err, why);
    const nlohmann::json& man = b->manifest();
    if (!man.contains(kManifestField) || !man[kManifestField].is_object()) return fail(err, path + ": not a road model cache");
    if (man[kManifestField].value("format", 0u) != kRoadModelFormat) return fail(err, path + ": another layout");
    if (man[kManifestField].value("key", std::string()) != bundle::hex16(key)) return fail(err, path + ": made for another key");
    if (!b->checkSection(kModelSection)) return fail(err, path + ": its model's bytes no longer match their hash");
    const bundle::Bundle::View v = b->section(kModelSection);
    bundle::BinReader r(v.data, v.size);
    return getRoadModel(r, m, err);
}

ObtainedRoadModel obtainLevelRoadModel(const bundle::LevelInputs& in, const std::string& cacheDir, std::string* err) {
    ObtainedRoadModel out;
    const std::vector<roads::lanes::CityEntity> cities = roads::lanes::cityEntities(in.level);
    if (cities.empty()) {
        fail(err, "the level has no lane-built city");
        return out;
    }
    const roads::lanes::CityEntity& city = cities.front();
    // the layout is not in the roads' key (stripped out, so the bake does not see it): hashed here, the whole file
    uint64_t layoutHash = bundle::kFnvOffset;
    if (city.inlineGraph) layoutHash = bundle::fnv1aStr(city.block.dump(), layoutHash);
    else if (!bundle::fnv1aFile(city.graphPath, layoutHash)) layoutHash = bundle::fnv1aStr("missing:" + city.graphPath);
    out.key = roadModelCacheKey(roads::lanes::levelRoadsKey(in), layoutHash);
    if (!cacheDir.empty()) {
        out.path = roadModelCachePath(cacheDir, out.key);
        std::error_code ec;
        if (std::filesystem::exists(out.path, ec)) {
            RoadModel m;
            if (readRoadModelCache(out.path, out.key, m, nullptr)) {
                out.model = std::make_shared<const RoadModel>(std::move(m));
                out.fromCache = true;
                bundle::noteCacheUse(out.path, kCacheKind);
                return out;
            }
        }
    }
    std::unique_ptr<roads::lanes::Result> roads;
    try {
        roads::lanes::BuildOptions bo;
        bo.threads = in.threads;
        roads = roads::lanes::build(roads::lanes::loadCityGraph(city), bo);
    } catch (const std::exception& ex) {
        fail(err, std::string("the road build failed: ") + ex.what());
        return out;
    }
    out.model = std::make_shared<const RoadModel>(assembleRoadModel(*roads));
    if (!out.path.empty() && writeRoadModelCache(out.path, out.key, *out.model, err)) {
        out.saved = true;
        bundle::noteCacheUse(out.path, kCacheKind);
    }
    return out;
}

}  // namespace engine
