#include "engine/procgen/city/roads/road_model.h"

#include "engine/bundle/bundle.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"         // junctionAreas: each arm's stop line, the one rule
#include "engine/procgen/city/roads/lanes/lane_level.h"        // buildLaneLevel: the model's lane level (ADR-0429)
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"       // kArmOrderCommand: an unmarked file's ring (ADR-0427)
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"   // classOf: the engine's class of a road
#include "engine/procgen/city/roads/lane_turns.h"              // the stated turns through each junction (ADR-0426)
#include "engine/procgen/city/polyline.h"                      // simplifyRun, distToSegment
#include "engine/procgen/city/polygon.h"                       // pointInPolygon, interiorPoint: the regions' outlines
#include "engine/asset_root.h"                                 // assetPath: the attachment kinds' file
#include "log.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <type_traits>

namespace engine {

namespace {

using roads::lanes::EdgeSpec;
using roads::lanes::Lane;
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
    "model, read from the road file: assembleRoadModel adds them from a road build, as the lane graph does)";

// A ROAD'S CENTRE LINE between two stations of its spine, its straight runs thinned to within kLineTolerance, appended to
// the model's lines with each point's station (ADR-0425).
Range appendLine(RoadModel& m, const EdgeSpec& e, Real from, Real to) {
    Range out;
    if (e.xy.size() < 2) return out;
    // the spine's points inside the stretch (polyline.h stretchAtStations, the one function): `from` and `to` are stations
    // the road file wrote, so a spine point within kWrittenStation of one IS that end
    std::vector<Vec2> pts;
    std::vector<Real> st;
    stretchAtStations(e.xy.data(), e.s.data(), e.xy.size(), from, to, kWrittenStation, pts, &st);
    out.first = static_cast<uint32_t>(m.linePoints.size());
    for (std::size_t i : simplifyRun(pts, kLineTolerance)) {
        m.linePoints.push_back(pts[i]);
        m.lineStations.push_back(st[i]);
    }
    out.count = static_cast<uint32_t>(m.linePoints.size()) - out.first;
    return out;
}

// A ROAD'S CROSS-SECTION, from its class and its own lanes (`fits` out: false when its stated kerb parking does not fit).
ModelSection sectionOf(const RoadLabGraph& g, const EdgeSpec& e, bool* fits) {
    const RoadClassSpec& c = g.cls(e);
    ModelSection s;
    s.roadClass = e.cls;
    s.klass = roads::lanes::classOf(e);
    s.lanesForward = e.lanes.fwd;
    s.lanesBackward = e.lanes.back;
    s.laneWidth = e.lanes.w;
    s.medianGap = e.lanes.gap;
    s.median = c.median;
    s.shoulder = c.shoulder;
    s.sidewalk = c.sidewalk;
    s.track = c.track;
    s.footway = c.footway;
    // the kerb parking the road states (its class's, or its own), where it fits (ADR-0418), and where its band lies
    *fits = true;
    s.parking = c.carriesCars() ? roads::lanes::kerbParking(g, e, fits) : 0.0;
    if (!c.carriesCars()) *fits = true;
    if (s.parking > 0) {
        double offset = 0, width = 0;
        if (roadSpecParkingBand(roads::lanes::streetSection(g, e), +1, offset, width)) s.parkingOffset = offset;
    }
    return s;
}

}  // namespace

bool junctionLit(const ModelJunction& J) {
    const AttachmentKind* pole = attachmentKinds().find(attachmentKinds().use("signal_pole"));
    return !J.control.empty() && pole && !pole->controls.empty() && pole->standsFor(J.control);
}

// ---- the planned layer and the model of a build -------------------------------------------------------------------------

RoadModel plannedRoadModel(const RoadLabGraph& g) {
    RoadModel m;
    m.name = g.name;
    // ---- junctions and streets: the road file's own, whole (road_graph_spec.h RoadLabGraph) -----------------------------
    // (their arms in the order the file lists them: the stated ring, ADR-0427, where the file marks its lists so -- trusted,
    // never measured or re-sorted here; a file with junctions and no mark is said, once, naming the command)
    m.junctions = g.junctions;
    m.streets = g.streets;
    m.armsInRing = g.armsInRing;
    if (!g.junctions.empty() && !g.armsInRing)
        m.problems.push_back("the road file does not state that its junctions' arms are listed in their ring (\"arms_in_ring\"): run `" +
                             std::string(roads::lanes::kArmOrderCommand) + "`");
    // What only the model adds to a junction: each arm's box edge and stop line and the junction's area along each road
    // (the one rule, from the stated junction, deck_mesh.h junctionAreas -- what the paint draws and the lane level ends
    // its lanes at), and what the street map needs to drive each stretch inside it (ADR-0425).
    const std::vector<roads::lanes::JunctionArea> areas = roads::lanes::junctionAreas(g);
    for (std::size_t j = 0; j < m.junctions.size(); ++j) {
        ModelJunction& J = m.junctions[j];
        const roads::lanes::JunctionArea& area = areas[j];
        if (!g.declaresJunctions) {   // a list that plans no control states none
            J.control.clear();
            J.why.clear();
        }
        for (std::size_t a = 0; a < J.arms.size(); ++a) {
            const roads::lanes::ArmStop& stop = area.arms[a];
            if (stop.edge >= 0) {
                J.arms[a].boxEdge = stop.boxEdge;
                J.arms[a].stopLine = stop.stopLine;
            }
        }
        for (const roads::lanes::JunctionSpan& sp : area.spans) J.boxes.push_back(ModelStretch{g.edges[static_cast<std::size_t>(sp.edge)].id, sp.s0, sp.s1});
        for (ModelStretch& in : J.inside) {
            const std::map<std::string, int>::const_iterator ei = g.index.find(in.road);
            if (ei == g.index.end()) continue;
            const EdgeSpec& e = g.edges[static_cast<std::size_t>(ei->second)];
            in.edge = static_cast<uint32_t>(ei->second);
            in.line = appendLine(m, e, in.from, in.to);
            bool fits = true;
            in.section = sectionOf(g, e, &fits);
        }
        for (ModelNode& n : J.nodes)
            for (ModelNodePoint& p : n.points) p.edge = static_cast<uint32_t>(g.index.at(p.road));
    }

    // What only the model adds to a street: its road's index, its cross-section from its road's class and lanes, what its
    // road posts and allows, and its centre line.
    std::set<uint32_t> parkingRefused;   // roads already reported for kerb parking that does not fit
    for (ModelStreet& S : m.streets) {
        S.edge = static_cast<uint32_t>(g.index.at(S.road));
        const EdgeSpec& e = g.edges[S.edge];
        {
            bool fits = true;
            S.section = sectionOf(g, e, &fits);
            if (!fits && parkingRefused.insert(S.edge).second)
                m.problems.push_back(fmt("road %s (%s): it states %.1f m of kerb parking, which would leave a travel lane under %.1f m; none is "
                                         "built", e.id.c_str(), e.cls.c_str(), e.lanes.parking, roads::lanes::kMinTravelLane));
        }
        S.speedLimit = e.speedLimit;
        S.lotsFrom = e.lotsFrom;
        S.lotsTo = e.lotsTo;
        S.frontageOf = e.frontageOf;
        // its centre line: the road's spine from `from` to `to`, its straight runs thinned to within kLineTolerance
        S.line = appendLine(m, e, S.from, S.to);
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
                // where the ramp's own spine reaches the gore (its composed geometry: lane_expand.h composeRamp)
                const int gore = e.anchorIdx[end];
                if (gore >= 0 && static_cast<std::size_t>(gore) < e.s.size()) E.rampStation = e.s[static_cast<std::size_t>(gore)];
                E.length = end == 0 ? a.decel : a.aux;
                E.taper = a.taper;   // the file's, or the default it was read with (road_graph_spec.h)
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
        for (std::size_t i = 0; i < m.streets.size(); ++i) {
            if (m.streets[i].name.empty()) continue;
            ModelAttachment a;
            a.kind = nameKind;
            a.street = ModelStreetId(static_cast<uint32_t>(i));
            a.text = m.streets[i].name;
            std::string why;
            if (!attach(m, a, kinds, &why)) m.problems.push_back("street " + m.streets[i].id + ": its name is not attached: " + why);
        }
    }

    // ---- what nothing states yet ---------------------------------------------------------------------------------------
    m.notStated.push_back(kBuiltLayerAbsent);
    if (!g.listsJunctions) m.notStated.push_back("junctions: the road file lists none (lanes_tool layout writes them)");
    if (!g.listsStreets) m.notStated.push_back("streets: the road file lists none (lanes_tool layout writes them); lanes carry no street");
    if (!g.declaresJunctions) m.notStated.push_back("controls and signs: the road file plans none (city_plan controls)");
    bool anyLimit = false, anyName = false;
    for (const EdgeSpec& e : g.edges) anyLimit = anyLimit || e.speedLimit > 0;
    for (const ModelStreet& st : g.streets) anyName = anyName || !st.name.empty();
    if (!anyLimit) m.notStated.push_back("speed limits: no road posts one (the rule set's default for each class holds)");
    if (!anyName) m.notStated.push_back("street names: the road file names no street (lanes_tool layout --names writes them)");
    if (!g.declaresCrossings) m.notStated.push_back("crossings: the road file declares none (the design stage works them out from heights)");
    m.notStated.push_back("turn paths through junctions: the turns are stated (the built layer's `turns`, ADR-0426); the path each "
                          "follows is the lane level's (lane_level.h), not held here");
    m.notStated.push_back("which lines between lanes may be crossed: the lane level's, from the lanes' kinds and the road file's "
                          "`lane_rules` (ADR-0426), not held here");
    m.notStated.push_back("junction outlines on the ground: the builder has none (the paint's boxes are kept as stretches along each road)");
    m.notStated.push_back("crosswalks: nothing states them (the paint draws a zebra at each box)");
    m.notStated.push_back("parking spaces: the kerb parking band is stated (each street's cross-section), the spaces along it are not (the city sim lays them at load)");
    m.notStated.push_back("blocks and their uses: the city plan has them, the road file does not carry them");
    m.notStated.push_back("signal groups: nothing states them (each driving model works its own out)");
    return m;
}

LaneSpanEnd laneSpanEnd(const RoadModel& m, const ModelLaneSpan& sp, int end) {
    LaneSpanEnd out;
    const ModelStreet& S = m.streets[sp.street.index];
    const Real at = end == 0 ? S.from : S.to;
    out.reaches = sameWrittenStation(end == 0 ? sp.from : sp.to, at);
    if (!out.reaches) return out;
    out.junction = S.junctions[static_cast<std::size_t>(end)];
    if (!out.junction.valid()) return out;
    out.arm = m.armAt(sp.street, end);
    if (out.arm >= 0) out.stopLine = m.junctions[out.junction.index].arms[static_cast<std::size_t>(out.arm)].stopLine;
    return out;
}

std::vector<std::vector<TurnArm>> turnArmsOf(const RoadModel& m) {
    // per junction and arm (in its list's order: the stated ring): the lanes arriving and leaving by it (the street it is,
    // the way the lane runs there), and its heading
    std::vector<std::vector<TurnArm>> armsOf(m.junctions.size());
    for (std::size_t j = 0; j < m.junctions.size(); ++j) {
        armsOf[j].resize(m.junctions[j].arms.size());
        for (std::size_t a = 0; a < m.junctions[j].arms.size(); ++a) {
            armsOf[j][a].hasHeading = stated(m.junctions[j].arms[a].heading);
            armsOf[j][a].heading = armsOf[j][a].hasHeading ? m.junctions[j].arms[a].heading : 0.0;
        }
    }
    // each lane, street by street: where its stretch reaches an end of the street that names a junction, it arrives at
    // that junction (the way it runs) or leaves it, by the arm that street is there (a lane runs on through a junction its
    // road passes through mid-way: the streets either side are two arms)
    for (std::size_t li = 0; li < m.lanes.size(); ++li) {
        const ModelLane& L = m.lanes[li];
        for (const ModelLaneSpan& sp : L.streets) {
            for (int end = 0; end < 2; ++end) {
                const LaneSpanEnd at = laneSpanEnd(m, sp, end);
                if (!at.reaches || !at.junction.valid() || at.arm < 0) continue;
                const bool arriving = (end == 1) == (L.dir > 0);   // it runs toward this end
                TurnArm& arm = armsOf[at.junction.index][static_cast<std::size_t>(at.arm)];
                TurnArmLane tl{static_cast<uint32_t>(li), L.kind, L.slot, L.id};
                (arriving ? arm.arriving : arm.leaving).push_back(tl);
            }
        }
    }
    return armsOf;
}

RoadModel builtRoadModel(const Result& r) {
    const RoadLabGraph& g = r.graph;
    RoadModel m = plannedRoadModel(g);
    m.built = true;
    m.notStated.erase(std::remove(m.notStated.begin(), m.notStated.end(), std::string(kBuiltLayerAbsent)), m.notStated.end());

    // ---- each junction's height: the drawn deck at it, along its first arm's road -------------------------------------
    for (ModelJunction& J : m.junctions) {
        if (J.arms.empty() || !r.heights) continue;
        const EdgeSpec* e = g.find(J.arms.front().road);
        if (e && e->xy.size() >= 2)
            J.height = r.heights->deckRoad(g.index.at(e->id), roads::lanes::pointAt(e->xy, e->s, J.arms.front().station));
    }

    // ---- lanes, as the builder laid them: their curves, heights and slots; and WHERE EACH LIES ON THE STREETS (its
    // stretches, the junction at each end), read here from the stated streets and held by the model's lane alone (the
    // builder's lane carries no copy, issue #292): nothing is measured but where the lane's own stretch of spine lies ----
    std::map<std::string, std::vector<uint32_t>> streetsOf;   // road id -> its streets, along its spine
    for (std::size_t i = 0; i < m.streets.size(); ++i) streetsOf[m.streets[i].road].push_back(static_cast<uint32_t>(i));
    for (std::pair<const std::string, std::vector<uint32_t>>& kv : streetsOf)
        std::stable_sort(kv.second.begin(), kv.second.end(), [&](uint32_t a, uint32_t b) { return m.streets[a].from < m.streets[b].from; });
    for (std::size_t li = 0; li < r.lanes.lanes.size(); ++li) {
        const Lane& l = r.lanes.lanes[li];
        ModelLane L;
        static_cast<LaneFacts&>(L) = l;   // the lane's facts, the builder's taken whole (declared once: road_model_parts.h)
        L.curve = Range{static_cast<uint32_t>(m.points.size()), static_cast<uint32_t>(l.xy.size())};
        // the spine sample each point was laid beside (lane_expand.cpp ribbonLane: point k of a lane laid from spineFrom)
        std::size_t i0 = 0;
        const EdgeSpec* e = l.onRoad() ? &g.edges[l.edge] : nullptr;
        if (e) i0 = static_cast<std::size_t>(std::lower_bound(e->s.begin(), e->s.end(), l.spineFrom) - e->s.begin());
        for (std::size_t k = 0; k < l.xy.size(); ++k) {
            m.points.push_back(l.xy[k]);
            m.along.push_back(k < l.s.size() ? l.s[k] : 0.0);
            m.heights.push_back(r.heights ? r.heights->deck(static_cast<int>(li), l.xy[k]) : 0.0);
            m.spineStations.push_back(e && i0 + k < e->s.size() ? e->s[i0 + k] : kNotStated);
        }
        const std::map<std::string, std::vector<uint32_t>>::const_iterator own = e ? streetsOf.find(e->id) : streetsOf.end();
        if (own != streetsOf.end()) {
            for (uint32_t si : own->second) {
                const ModelStreet& st = m.streets[si];
                const Real lo = std::max<Real>(L.spineFrom, st.from), hi = std::min<Real>(L.spineTo, st.to);
                if (hi > lo) L.streets.push_back(ModelLaneSpan{ModelStreetId(si), lo, hi});
            }
        }
        if (!L.streets.empty()) {
            // the junction at an end of the street it begins or ends on, when the lane reaches that end of the street
            const ModelStreet& first = m.streets[L.streets.front().street.index];
            const ModelStreet& last = m.streets[L.streets.back().street.index];
            const ModelJunctionId atFrom = atOrBefore(L.spineFrom, first.from, kWrittenStation) ? first.junctions[0] : ModelJunctionId();
            const ModelJunctionId atTo = atOrPast(L.spineTo, last.to, kWrittenStation) ? last.junctions[1] : ModelJunctionId();
            L.atStart = L.dir > 0 ? atFrom : atTo;
            L.atEnd = L.dir > 0 ? atTo : atFrom;
        }
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
                    // where the ramp's line leaves (or meets) its host lane's: the station the builder laid it at, as it
                    // recorded it (lane_expand.h composeRamp, EdgeSpec::anchorMeets). Copied, never worked out again.
                    const EdgeSpec* ramp = g.find(R.edge);
                    if (ramp) C.meets = ramp->anchorMeets[end];
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
    // ...and the lanes the builder sweeps out of a neighbour along its road (a turn pocket) or into one (a pocket that
    // tapers out, a ramp's second lane): its own records (Lane::sweptFrom, diesInto), at the lane's own ends
    for (std::size_t li = 0; li < r.lanes.lanes.size(); ++li) {
        const Lane& l = r.lanes.lanes[li];
        if (l.road() < 0) continue;
        const EdgeSpec& e = g.edges[static_cast<std::size_t>(l.road())];
        const roads::lanes::PocketSpec* pocket = nullptr;
        for (const roads::lanes::PocketSpec& pk : e.pockets)
            if (e.id + "." + pk.id == l.id) pocket = &pk;
        const double start = l.dir > 0 ? l.spineFrom : l.spineTo, end = l.dir > 0 ? l.spineTo : l.spineFrom;
        if (l.sweptFrom >= 0) {
            ModelConnection C;
            C.kind = ConnectionKind::Diverge;
            C.from = ModelLaneId(static_cast<uint32_t>(l.sweptFrom));
            C.to = ModelLaneId(static_cast<uint32_t>(li));
            C.taper = pocket ? pocket->taper : 0.0;   // the file's, or the default it was read with
            C.hostStation = start + l.dir * C.taper;
            C.meets = start;
            m.connections.push_back(C);
        }
        if (l.diesInto >= 0) {
            ModelConnection C;
            C.kind = ConnectionKind::Merge;
            C.from = ModelLaneId(static_cast<uint32_t>(li));
            C.to = ModelLaneId(static_cast<uint32_t>(l.diesInto));
            C.taper = pocket ? pocket->taperOut : 0.0;
            C.hostStation = end - l.dir * C.taper;
            C.meets = end;
            m.connections.push_back(C);
        }
    }

    // ---- the turns through each junction (ADR-0426): stated once, by the rows of assets/data/lane_turns.json over its
    // arms (their stated headings and through pairs), the lanes arriving and leaving by each (their kinds and slots) and
    // its control
    {
        const LaneTurnRules& rules = laneTurnRules();
        if (rules.rows.empty()) m.problems.push_back("no rows in assets/data/lane_turns.json: no junction offers a turn");
        const std::vector<std::vector<TurnArm>> armsOf = turnArmsOf(m);
        for (std::size_t j = 0; j < m.junctions.size(); ++j) {
            std::vector<std::string> notes;
            for (const StatedTurn& t : turnsAtJunction(armsOf[j], m.junctions[j].through, m.junctions[j].control, rules, &notes)) {
                ModelTurn T;
                T.junction = ModelJunctionId(static_cast<uint32_t>(j));
                T.fromArm = t.fromArm;
                T.toArm = t.toArm;
                T.from = ModelLaneId(t.from);
                T.to = ModelLaneId(t.to);
                T.turn = t.turn;
                T.row = t.row;
                m.turns.push_back(T);
            }
            for (const std::string& n : notes) m.problems.push_back("junction " + m.junctions[j].id + ": " + n);
        }
    }

    for (const ModelJunction& J : m.junctions)
        for (std::size_t k = 0; k < J.arms.size(); ++k)
            if (!J.arms[k].street.valid())
                m.problems.push_back(fmt("junction %s: arm %zu (road %s at %.1f) names no street of the road file", J.id.c_str(), k,
                                         J.arms[k].road.c_str(), J.arms[k].station));

    return m;
}

RoadModel assembleRoadModel(const Result& r, const roads::lanes::LaneGraphOptions& opts, roads::lanes::LaneGraphBuild* report) {
    RoadModel m = builtRoadModel(r);
    // ---- the lane level: the table of paths, from everything stated in the built layer (lanes/lane_level.h) -------------
    roads::lanes::LaneGraphBuild found = roads::lanes::buildLaneLevel(r, m, opts);
    if (report) *report = std::move(found);
    return m;
}

// ---- the questions ----------------------------------------------------------------------------------------------------

namespace {
const std::string kNoneStated;   // what a question with a name for an answer returns where nothing is stated
}

const std::string& RoadModel::roadClassOf(PathId path) const {
    const ModelStreet* s = streetOf(path);
    return s ? s->section.roadClass : kNoneStated;
}

const std::string& RoadModel::controlAt(JunctionId laneJunction) const {
    const ModelJunction* j = junctionOf(laneJunction);
    return j ? j->control : kNoneStated;
}

PathId RoadModel::mergingAt(const CrossingPoint& x) const {
    if (x.modelConnection >= connections.size() || x.otherSide != OtherSide::Path) return PathId();
    const ModelConnection& c = connections[x.modelConnection];
    if (c.kind != ConnectionKind::Merge) return PathId();
    const PathId other = x.otherPath();
    if (x.a.index < laneLevel.paths.size() && laneLevel.paths[x.a.index].modelLane == c.from.index) return x.a;
    if (other.index < laneLevel.paths.size() && laneLevel.paths[other.index].modelLane == c.from.index) return other;
    return PathId();
}

int RoadModel::armAt(ModelStreetId street, int end) const {
    if (street.index >= streets.size() || end < 0 || end > 1) return -1;
    const ModelJunctionId j = streets[street.index].junctions[static_cast<std::size_t>(end)];
    if (j.index >= junctions.size()) return -1;
    const std::vector<ModelArm>& arms = junctions[j.index].arms;
    for (std::size_t a = 0; a < arms.size(); ++a)
        if (arms[a].street == street && arms[a].spineIn == (end == 1)) return static_cast<int>(a);
    return -1;
}

std::vector<Vec2> RoadModel::centreLine(const Range& line) const {
    if (line.end() > linePoints.size()) return {};
    return std::vector<Vec2>(linePoints.begin() + line.first, linePoints.begin() + line.end());
}

bool RoadModel::streetEnd(const ModelStreet& street, int end, Vec2& out) const {
    const Range& line = street.line;
    if (line.count == 0 || line.end() > linePoints.size()) return false;
    out = linePoints[end == 0 ? line.first : line.end() - 1];
    return true;
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
    for (std::size_t t = 0; t < m.turns.size(); ++t)
        if (m.turns[t].from == lane) out.turns.push_back(static_cast<uint32_t>(t));
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
            if (k.contains("marking") && k["marking"].is_object()) kind.marking = k["marking"];
            if (kind.on == "street" || kind.on == "junction" || kind.on == "arm") out.kinds.push_back(kind);
        }
    if (doc.contains("uses") && doc["uses"].is_object())
        for (const auto& [role, kind] : doc["uses"].items())
            if (kind.is_string()) out.uses.emplace_back(role, kind.get<std::string>());
    if (doc.contains("controls") && doc["controls"].is_object() && doc["controls"].contains("map") && doc["controls"]["map"].is_object())
        for (const auto& [control, colour] : doc["controls"]["map"].items())
            if (colour.is_string() && !control.empty() && control[0] != '_') out.controlNames.push_back(control);
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
        const std::string name = streetName(m, a.street);
        NamedArm n;
        if (name.empty() || !armMouth(m, J, k, n.point)) continue;
        n.arm = static_cast<int>(k);
        n.street = a.street;
        n.dir = directionOf(a.heading);
        n.name = name;
        out.push_back(n);
    }
    return out;   // (in the stated ring order, the list's own: ADR-0427)
}

bool armMouth(const RoadModel& m, const ModelJunction& J, std::size_t arm, Vec2& out) {
    if (arm >= J.arms.size()) return false;
    const ModelArm& a = J.arms[arm];
    const ModelStreet* street = m.streetOf(a);
    if (!street || street->line.count < 2) return false;
    return m.streetEnd(*street, a.spineIn ? 1 : 0, out);   // its street's end at the junction
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
                   S.section.roadClass.c_str(), S.section.lanesForward, S.section.lanesBackward, S.section.laneWidth, S.section.oneWay() ? " one-way" : "",
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
        out += fmt("connection: %s %s %s at host station %.2f (length %.1f, taper %.1f; the lines meet at %s)%s\n",
                   m.lanes[c.from.index].id.c_str(), connectionName(c.kind), m.lanes[c.to.index].id.c_str(), c.hostStation, c.length,
                   c.taper, num(c.meets).c_str(), c.ramp.empty() ? "" : (" ramp " + c.ramp).c_str());
    for (const ModelTurn& t : m.turns)
        out += fmt("turn at %s: arm %u into arm %u, %s, lane %s into lane %s (row %u)\n", junctionName(m, t.junction).c_str(), t.fromArm,
                   t.toArm, turnKey(t.turn), m.lanes[t.from.index].id.c_str(), m.lanes[t.to.index].id.c_str(), t.row);
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
    out += fmt("  %zu connections: %u lanes leaving a lane, %u lanes merging into one, %u declared connectors\n", m.connections.size(), diverge,
               merge, joins);
    {
        uint32_t byTurn[kTurnCount] = {};
        for (const ModelTurn& t : m.turns) ++byTurn[static_cast<std::size_t>(t.turn)];
        out += fmt("  %zu turns through junctions (ADR-0426): %u straight on, %u near turns, %u far turns\n", m.turns.size(),
                   byTurn[static_cast<std::size_t>(Turn::Straight)], byTurn[static_cast<std::size_t>(Turn::NearTurn)],
                   byTurn[static_cast<std::size_t>(Turn::FarTurn)]);
    }
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
           std::is_same<T, ModelRamp>::value || std::is_same<T, ModelCrossing>::value || std::is_same<T, ModelAttachment>::value ||
           std::is_same<T, ModelNode>::value || std::is_same<T, ModelNodePoint>::value || std::is_same<T, ModelTurn>::value;
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
template <class IO, class S> void fields(IO& io, S& s, ModelStretch*) {
    io(s.road); io(s.from); io(s.to); io(s.edge); io(s.line); io(s.section);
}
template <class IO, class P> void fields(IO& io, P& p, ModelNodePoint*) { io(p.edge); io(p.station); }
template <class IO, class N> void fields(IO& io, N& n, ModelNode*) { io(n.at); io(n.points); }
template <class IO, class J> void fields(IO& io, J& j, ModelJunction*) {
    io(j.id); io(j.at); io(j.height); io(j.arms); io(j.through); io(j.control); io(j.why); io(j.inside); io(j.nodes); io(j.boxes);
}
template <class IO, class S> void fields(IO& io, S& s, ModelSection*) {
    io(s.roadClass);
    uint8_t klass = static_cast<uint8_t>(s.klass);
    io(klass);
    s.klass = static_cast<RoadClass>(klass);
    io(s.lanesForward); io(s.lanesBackward); io(s.laneWidth); io(s.medianGap); io(s.median); io(s.shoulder); io(s.sidewalk);
    io(s.parking); io(s.parkingOffset); io(s.track); io(s.footway);
}
template <class IO, class S> void fields(IO& io, S& s, ModelStreet*) {
    io(s.id); io(s.road); io(s.edge); io(s.from); io(s.to); io(s.junctions); io(s.carriesOn); io(s.section); io(s.speedLimit);
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
    io(c.from); io(c.to); io(c.ramp); io(c.hostStation); io(c.length); io(c.taper); io(c.meets);
}
template <class IO, class T> void fields(IO& io, T& t, ModelTurn*) {
    io(t.junction); io(t.fromArm); io(t.toArm); io(t.from); io(t.to);
    uint8_t turn = static_cast<uint8_t>(t.turn);
    io(turn);
    t.turn = static_cast<Turn>(turn);
    io(t.row);
}
template <class IO, class E> void fields(IO& io, E& e, ModelRampEnd*) {
    uint8_t kind = static_cast<uint8_t>(e.kind);
    io(kind);
    e.kind = static_cast<RampEndKind>(kind);
    io(e.host); io(e.hostLane); io(e.hostStation); io(e.rampStation); io(e.length); io(e.taper);
}
template <class IO, class R> void fields(IO& io, R& r, ModelRamp*) {
    io(r.edge); io(r.ends); io(r.landingPlanned); io(r.landingStreet); io(r.landingJoins); io(r.landingCarriageway); io(r.landingCrossing);
}
template <class IO, class X> void fields(IO& io, X& x, ModelCrossing*) { io(x.a); io(x.b); io(x.kind); io(x.over); io(x.at); io(x.clearance); }

template <class IO, class M> void modelFields(IO& io, M& m) {
    io(m.name); io(m.built); io(m.junctions); io(m.streets); io(m.lanes); io(m.connections); io(m.turns); io(m.ramps); io(m.crossings); io(m.points);
    io(m.along); io(m.heights); io(m.spineStations); io(m.linePoints); io(m.lineStations); io(m.attachments); io(m.notStated);
    io(m.problems); io(m.armsInRing);
}

}  // namespace

void putRoadModel(bundle::BinWriter& w, const RoadModel& m) {
    w.magic(kMagic, kRoadModelFormat);
    Writer io{w};
    RoadModel copy = m;   // (the walk takes its fields by reference)
    modelFields(io, copy);
    putLaneGraph(w, m.laneLevel);   // the lane level, in the lane graph's own bytes
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
    std::string why;
    if (!getLaneGraph(r, m.laneLevel, &why)) {
        m = RoadModel();
        if (err) *err = "road model: its lane level does not read: " + why;
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

}  // namespace engine
