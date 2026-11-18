#include "engine/procgen/city/roads/road_model.h"

#include "engine/bundle/bundle.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"         // junctionAreas: each arm's stop line, the one rule
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"  // kerbPerpSign: which side of a lane its kerb is
#include "engine/procgen/city/roads/lanes/lane_level.h"        // buildLaneLevel: the model's lane level (ADR-0429)
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/road_layout.h"       // kArmOrderCommand: an unmarked file's ring (ADR-0427)
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"   // classOf: the engine's class of a road
#include "engine/procgen/city/roads/lane_turns.h"              // the stated turns through each junction (ADR-0426)
#include "engine/procgen/city/polyline.h"                      // simplifyRun, distToSegment
#include "engine/procgen/city/polygon.h"                       // pointInPolygon, interiorPoint: the regions' outlines
#include "engine/asset_root.h"                                 // assetPath: the attachment kinds' file
#include "check.h"
#include "log.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <tuple>
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
    s.klass = roads::lanes::classOf(g, e);
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

// THE KERB PARKING SPACES OF EVERY STREET THAT PARKS (ADR-0439), stated on the model once its junctions and streets are
// in. A street parks where its cross-section states a parking band (its road class's `parking`). Along each such
// street the stretch that may hold spaces is its own stretch of spine, less:
//   - at an end that names a junction: everything from that arm's stop line toward the junction, and `clearOfStopLine`
//     more (an arm with no stop line stated: from where it meets the junction);
//   - what lies outside the stretch buildings may front, where its road states one (lots_range);
//   - every junction area lying along its road part-way (a junction a road runs on through), with the stop line's
//     set-back and the clearance either side.
// Each piece left is filled with as many whole spaces as fit, centred in it, on the kerb of each way the street has
// lanes. Nothing here measures the ground: stations, the stated stop lines and the two numbers of the rules file.
void stateKerbRuns(RoadModel& m, const KerbParkingRules& rules) {
    if (!rules.usable) {
        m.notStated.push_back("kerb parking spaces: assets/data/kerb_parking.json is missing or does not read, so no street is given a space");
        return;
    }
    std::map<std::string, std::vector<std::pair<Real, Real>>> areasOn;   // road id -> the junction areas along it
    for (const ModelJunction& J : m.junctions)
        for (const ModelStretch& box : J.boxes) areasOn[box.road].emplace_back(box.from, box.to);
    const Real keepOff = kStopLineBack + rules.clearOfStopLine;   // from a junction area's edge to the nearest space
    for (std::size_t si = 0; si < m.streets.size(); ++si) {
        const ModelStreet& S = m.streets[si];
        if (S.section.parking <= 0 || !S.section.carriesCars()) continue;
        const ModelStreetId street(static_cast<uint32_t>(si));
        Real lo = S.from, hi = S.to;
        for (int end = 0; end < 2; ++end) {
            const ModelJunctionId j = S.junctions[static_cast<std::size_t>(end)];
            if (j.index >= m.junctions.size()) continue;
            const int arm = m.armAt(street, end);
            Real line = end == 0 ? S.from : S.to;
            if (arm >= 0 && stated(m.junctions[j.index].arms[static_cast<std::size_t>(arm)].stopLine))
                line = m.junctions[j.index].arms[static_cast<std::size_t>(arm)].stopLine;
            if (end == 0) lo = std::max(lo, line + rules.clearOfStopLine);
            else hi = std::min(hi, line - rules.clearOfStopLine);
        }
        if (S.lotsFrom >= 0) lo = std::max(lo, S.lotsFrom);
        if (S.lotsTo >= 0) hi = std::min(hi, S.lotsTo);
        // the pieces left once the junction areas along the road are cut out, in order along the spine
        std::vector<std::pair<Real, Real>> pieces;
        {
            std::vector<std::pair<Real, Real>> cuts;
            const std::map<std::string, std::vector<std::pair<Real, Real>>>::const_iterator on = areasOn.find(S.road);
            if (on != areasOn.end())
                for (const std::pair<Real, Real>& a : on->second) cuts.emplace_back(a.first - keepOff, a.second + keepOff);
            std::sort(cuts.begin(), cuts.end());
            Real at = lo;
            for (const std::pair<Real, Real>& c : cuts) {
                // (wholly before what is left, or past the stretch)
                if (atOrBefore(c.second, at, kWrittenStation) || atOrPast(c.first, hi, kWrittenStation)) continue;
                if (strictlyPast(c.first, at, kWrittenStation)) pieces.emplace_back(at, c.first);
                at = std::max(at, c.second);
            }
            if (strictlyBefore(at, hi, kWrittenStation)) pieces.emplace_back(at, hi);
        }
        for (int dir : {1, -1}) {
            if ((dir > 0 ? S.section.lanesForward : S.section.lanesBackward) <= 0) continue;   // no traffic beside that kerb
            for (const std::pair<Real, Real>& piece : pieces) {
                // (a piece a written centimetre short of a whole space still holds it: stations are written to that)
                const Real room = piece.second - piece.first;
                const uint32_t count = static_cast<uint32_t>(std::floor((room + kWrittenStation) / rules.spaceLength));
                if (count == 0) continue;
                ModelKerbRun run;
                run.street = street;
                run.dir = dir;
                run.spaceLength = rules.spaceLength;
                run.count = count;
                run.from = piece.first + std::max(Real(0), room - static_cast<Real>(count) * rules.spaceLength) * Real(0.5);
                m.kerbRuns.push_back(run);
            }
        }
    }
}

// The runs of one street: their stretch of RoadModel::kerbRuns (the list is by street).
Range kerbRunsOfStreet(const RoadModel& m, ModelStreetId street) {
    const std::vector<ModelKerbRun>::const_iterator first = std::lower_bound(
        m.kerbRuns.begin(), m.kerbRuns.end(), street.index, [](const ModelKerbRun& r, uint32_t s) { return r.street.index < s; });
    std::vector<ModelKerbRun>::const_iterator last = first;
    while (last != m.kerbRuns.end() && last->street == street) ++last;
    Range out;
    out.first = static_cast<uint32_t>(first - m.kerbRuns.begin());
    out.count = static_cast<uint32_t>(last - first);
    return out;
}

// A street's centre line as the station readers take it: its points, their stations and the count (0: none).
struct StreetLine {
    const Vec2* xy = nullptr;
    const Real* stations = nullptr;
    std::size_t count = 0;
};
StreetLine lineOf(const RoadModel& m, const Range& line) {
    StreetLine out;
    if (line.count < 2 || line.end() > m.linePoints.size() || line.end() > m.lineStations.size()) return out;
    out.xy = m.linePoints.data() + line.first;
    out.stations = m.lineStations.data() + line.first;
    out.count = line.count;
    return out;
}
// The way a centre line runs between two of its stations (unit; the chord between them, and where that has no length
// the segment holding the first): the way the spine runs there.
Vec2 spineDirection(const StreetLine& l, Real from, Real to) {
    const Vec2 chord = pointAtStation(l.xy, l.stations, l.count, to) - pointAtStation(l.xy, l.stations, l.count, from);
    if (strictlyPast(chord.length(), 0, kSamePoint)) return normalize(chord);
    const StationSpot spot = spotAtStation(l.stations, l.count, from);
    const Vec2 seg = l.xy[spot.seg + 1] - l.xy[spot.seg];
    return strictlyPast(seg.length(), 0, kSamePoint) ? normalize(seg) : Vec2(1, 0);
}
// ACROSS THE ROAD TOWARD A KERB (unit): the kerb of the traffic running `dir` (+1 along the spine, -1 against it),
// where the spine runs `spine`. Which side a kerb lies is the one statement, lanes/lane_graph_build.h kerbPerpSign.
Vec2 towardKerb(const Vec2& spine, int dir) {
    return perp(spine * static_cast<Real>(dir)) * static_cast<Real>(roads::lanes::kerbPerpSign());
}

}  // namespace

bool junctionLit(const ModelJunction& J) {
    const AttachmentKind* pole = attachmentKinds().find(attachmentKinds().use("signal_pole"));
    return !J.control.empty() && pole && !pole->controls.empty() && pole->standsFor(J.control);
}

namespace {

// THE CROSSWALKS (ADR-0452): THE ONE FUNCTION that states them, from stated facts only. Across every arm of every stated
// junction whose road the paint marks (`areas`' ArmStop::painted: its class's junction_markings, never an anchored
// road's end) or whose street has pavements (its cross-section's sidewalk): its band kCrosswalkNear to kCrosswalkFar back
// from the arm's box edge, along the road's spine (deck_mesh.h, the one source of the two numbers), where the band lies
// on the road clear of its ends (kPaintRoadEndMargin: a road that ends inside the box has no leg there); its kerb ends
// level with the band's middle, the carriageway's half width either side of the spine toward each kerb
// (towardKerb, from kerbPerpSign); its control from the junction's stated control (lit: lights) and the arm's stated
// signs. The paint draws the marked ones (lanes/deck_mesh.cpp) and the lane level lays them all (layLaneCrosswalks).
void stateCrosswalks(RoadModel& m, const RoadLabGraph& g, const std::vector<roads::lanes::JunctionArea>& areas) {
    for (std::size_t j = 0; j < m.junctions.size() && j < areas.size(); ++j) {
        const ModelJunction& J = m.junctions[j];
        const bool lit = junctionLit(J);
        for (std::size_t a = 0; a < J.arms.size() && a < areas[j].arms.size(); ++a) {
            const roads::lanes::ArmStop& arm = areas[j].arms[a];
            if (arm.edge < 0) continue;
            const EdgeSpec& e = g.edges[static_cast<std::size_t>(arm.edge)];
            if (e.s.size() < 2) continue;
            const ModelStreet* street = m.streetOf(J.arms[a]);
            bool fits = true;
            const ModelSection section = street ? street->section : sectionOf(g, e, &fits);
            const bool pavements = section.sidewalk > 0 && section.carriesCars();
            if (!arm.painted && !pavements) continue;
            const int sign = arm.spineIn ? -1 : +1;   // the approach lies at stations below the box (-1) or above (+1)
            const Real nearStation = arm.boxEdge + sign * roads::lanes::kCrosswalkNear;
            const Real farStation = arm.boxEdge + sign * roads::lanes::kCrosswalkFar;
            const Real roadLength = e.s.back();
            if (std::min(nearStation, farStation) < roads::lanes::kPaintRoadEndMargin ||
                std::max(nearStation, farStation) > roadLength - roads::lanes::kPaintRoadEndMargin)
                continue;
            ModelCrosswalk c;
            c.junction = ModelJunctionId(static_cast<uint32_t>(j));
            c.arm = static_cast<uint32_t>(a);
            c.street = J.arms[a].street;
            c.road = e.id;
            c.nearStation = nearStation;
            c.farStation = farStation;
            const Real mid = (nearStation + farStation) * Real(0.5);
            const Vec2 at = roads::lanes::pointAt(e.xy, e.s, mid);
            const Vec2 spine = roads::lanes::tangentAtStation(e.xy, e.s, mid);
            const Real half = section.carriagewayHalfWidth();
            c.ends[0] = at + towardKerb(spine, +1) * half;
            c.ends[1] = at + towardKerb(spine, -1) * half;
            c.length = 2 * half;
            c.spineIn = arm.spineIn;
            c.marked = arm.painted;
            c.pavements = pavements;
            c.control = lit ? CrosswalkControl::Lights
                            : !J.arms[a].regulations.empty() ? CrosswalkControl::Signs : CrosswalkControl::None;
            // A KERB END IN ANOTHER ROAD: at a junction spread over several points two arms can stand so close that one
            // arm's band reaches into another's carriageway (or a stretch inside the junction). Nobody can wait at a kerb
            // there, so the crosswalk is not one people use (`pavements` false; still marked: the paint draws what it
            // drew), and the problem is said with its place.
            if (c.pavements) {
                const auto inRoad = [&](const Vec2& p, const Range& line, const ModelSection& s) {
                    const std::vector<Vec2> pts = m.centreLine(line);
                    return pts.size() >= 2 && strictlyBefore(distToPolyline(p, pts, false), s.carriagewayHalfWidth(), kSamePoint);
                };
                for (int end = 0; end < 2 && c.pavements; ++end) {
                    const Vec2 e = c.ends[static_cast<std::size_t>(end)];
                    for (std::size_t b = 0; b < J.arms.size() && c.pavements; ++b) {
                        if (b == a) continue;
                        const ModelStreet* other = m.streetOf(J.arms[b]);
                        if (other && other != street && inRoad(e, other->line, other->section)) c.pavements = false;
                    }
                    for (const ModelStretch& in : J.inside)
                        if (c.pavements && inRoad(e, in.line, in.section)) c.pavements = false;
                }
                if (!c.pavements)
                    m.problems.push_back(fmt("junction %s, arm %zu (%s): its crosswalk's kerb end lies in another road's carriageway; "
                                             "nobody waits there, so it is not one people cross by", J.id.c_str(), a, e.id.c_str()));
            }
            m.crosswalks.push_back(c);
        }
    }
}

}  // namespace

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
    // ---- the crosswalks (ADR-0452): stated here, from the arms' box edges, the paint's marked arms and the pavements ----
    stateCrosswalks(m, g, areas);
    // ---- the kerb parking spaces (ADR-0439): stated here, from the streets, the arms' stop lines and the rules file ------
    stateKerbRuns(m, kerbParkingRules());
    m.notStated.push_back("what else a kerb space must keep clear of: driveways and lot entrances, bus stops, hydrants and bridges are not "
                          "stated on a street, so no space is kept off them; parking off the street (a store's lot, a garage) is the "
                          "lot pass's, not held here");
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
    // THE LANES' CENTRE LINES ARE THE BUILD'S OWN OBJECT, held with it (ADR-0438): not a copy. Each lane's facts name its
    // stretch of it (`curve`); the heights and spine stations the model adds stand beside the same stretch.
    m.curves = r.lanes.curves;
    m.heights.assign(m.lanePoints().size(), 0.0);
    m.spineStations.assign(m.lanePoints().size(), kNotStated);
    for (std::size_t li = 0; li < r.lanes.lanes.size(); ++li) {
        const Lane& l = r.lanes.lanes[li];
        CHECK(l.curves == r.lanes.curves);   // every lane of a build is laid into the build's one object (expand)
        ModelLane L;
        static_cast<LaneFacts&>(L) = l;   // the lane's facts, the builder's taken whole (declared once: road_model_parts.h)
        // the spine sample each point was laid beside (lane_expand.cpp ribbonLane: point k of a lane laid from spineFrom)
        std::size_t i0 = 0;
        const EdgeSpec* e = l.onRoad() ? &g.edges[l.edge] : nullptr;
        if (e) i0 = static_cast<std::size_t>(std::lower_bound(e->s.begin(), e->s.end(), l.spineFrom) - e->s.begin());
        const Span<Vec2> xy = l.xy();
        for (std::size_t k = 0; k < xy.size(); ++k) {
            m.heights[l.curve.first + k] = r.heights ? r.heights->deck(static_cast<int>(li), xy[k]) : 0.0;
            m.spineStations[l.curve.first + k] = e && i0 + k < e->s.size() ? e->s[i0 + k] : kNotStated;
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

const ModelJunction* RoadModel::arrivesAt(PathId path) const {
    if (path.index >= laneLevel.paths.size()) return nullptr;
    const Path& p = laneLevel.paths[path.index];
    if (p.kind != PathKind::Lane || !p.onSpine || p.arm == kNoIndex || p.street >= streets.size()) return nullptr;
    const ModelJunctionId j = streets[p.street].junctions[p.alongSpine() ? 1 : 0];
    return j.index < junctions.size() ? &junctions[j.index] : nullptr;
}

std::vector<PathRegulation> RoadModel::regulationsAlong(PathId path) const {
    std::vector<PathRegulation> out;
    if (path.index >= laneLevel.paths.size()) return out;
    const Path& p = laneLevel.paths[path.index];
    if (p.kind != PathKind::Lane || !p.onSpine) return out;
    // the signs of the arm it arrives by, at its end
    const ModelJunction* J = arrivesAt(path);
    if (J && p.arm < J->arms.size()) {
        std::vector<std::string> names = J->arms[p.arm].regulations;
        std::sort(names.begin(), names.end());
        names.erase(std::unique(names.begin(), names.end()), names.end());
        for (const std::string& name : names) {
            PathRegulation r;
            r.kind = name;
            r.from = r.to = p.length;
            out.push_back(r);
        }
    }
    // the regulations stated on its street for traffic running its way, each station placed along the path
    const int way = p.alongSpine() ? 1 : -1;
    const Real lo = std::min(p.stationFrom, p.stationTo), hi = std::max(p.stationFrom, p.stationTo);
    const auto along = [&](Real station) { return alongPathAt(path, station); };   // metres along the path, the way it is driven
    for (const ModelRegulation& reg : regulations) {
        if (reg.street.index != p.street || reg.dir != way) continue;
        const bool atOnePlace = approxEqual(reg.from, reg.to, kWrittenStation);
        if (atOnePlace) {
            // a sign: on the path whose stretch holds its station (an end of the stretch counts)
            if (strictlyBefore(reg.from, lo, kWrittenStation) || strictlyPast(reg.from, hi, kWrittenStation)) continue;
        } else if (!strictlyPast(std::min(reg.to, hi), std::max(reg.from, lo), kWrittenStation)) {
            continue;   // a stretch: only where some of it lies on the path
        }
        // (the way the path is driven: a path against the spine meets the stretch's `to` first)
        const Real a = along(way > 0 ? reg.from : reg.to), b = atOnePlace ? a : along(way > 0 ? reg.to : reg.from);
        PathRegulation r;
        r.kind = reg.kind;
        r.from = a;
        r.to = b;
        r.value = reg.value;
        out.push_back(r);
    }
    // (a sort's order must be exact: two regulations a hair apart keep the order of their numbers)
    std::stable_sort(out.begin(), out.end(), [](const PathRegulation& x, const PathRegulation& y) {
        if (x.from != y.from) return x.from < y.from;
        if (x.to != y.to) return x.to < y.to;
        return x.kind < y.kind;
    });
    return out;
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

int RoadModel::armReachedAt(ModelStreetId street, Real station) const {
    if (street.index >= streets.size()) return -1;
    const ModelStreet& S = streets[street.index];
    if (sameWrittenStation(station, S.to)) return armAt(street, 1);
    if (sameWrittenStation(station, S.from)) return armAt(street, 0);
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

// ---- kerb parking spaces, and from a door to the road (ADR-0439) --------------------------------------------------------

Real RoadModel::alongPathAt(PathId path, Real station) const {
    if (path.index >= laneLevel.paths.size()) return 0;
    const Path& p = laneLevel.paths[path.index];
    if (!p.onSpine) return 0;
    const Real share = (station - p.stationFrom) / (p.stationTo - p.stationFrom);
    return std::min(std::max(share, Real(0)), Real(1)) * p.length;
}

namespace {
// Space `k` of a run, with the index it has along its kerb (`first`: the index of the run's first space).
KerbSpace spaceOfRun(const RoadModel& m, const ModelKerbRun& run, uint32_t first, uint32_t k) {
    KerbSpace s;
    s.street = run.street;
    s.dir = run.dir;
    s.index = first + k;
    s.from = run.from + static_cast<Real>(k) * run.spaceLength;
    s.to = run.from + static_cast<Real>(k + 1) * run.spaceLength;
    s.length = run.spaceLength;
    s.width = m.streets[run.street.index].section.parking;
    return s;
}
}  // namespace

std::vector<KerbSpace> RoadModel::kerbSpacesOf(ModelStreetId street) const {
    std::vector<KerbSpace> out;
    if (street.index >= streets.size()) return out;
    const Range runs = kerbRunsOfStreet(*this, street);
    uint32_t first = 0;
    int kerb = 0;
    for (uint32_t r = runs.first; r < runs.end(); ++r) {
        const ModelKerbRun& run = kerbRuns[r];
        if (run.dir != kerb) {   // (the other kerb's spaces count from 0 again)
            kerb = run.dir;
            first = 0;
        }
        for (uint32_t k = 0; k < run.count; ++k) out.push_back(spaceOfRun(*this, run, first, k));
        first += run.count;
    }
    return out;
}

KerbSpace RoadModel::kerbSpace(ModelStreetId street, int dir, uint32_t index) const {
    if (street.index >= streets.size()) return KerbSpace();
    const Range runs = kerbRunsOfStreet(*this, street);
    uint32_t first = 0;
    for (uint32_t r = runs.first; r < runs.end(); ++r) {
        const ModelKerbRun& run = kerbRuns[r];
        if (run.dir != dir) continue;
        if (index < first + run.count) return spaceOfRun(*this, run, first, index - first);
        first += run.count;
    }
    return KerbSpace();
}

std::string RoadModel::kerbSpaceName(const KerbSpace& space) const {
    if (space.street.index >= streets.size()) return std::string();
    return streets[space.street.index].id + (space.dir > 0 ? " along " : " against ") + std::to_string(space.index);
}

bool RoadModel::kerbSpacePose(const KerbSpace& space, KerbSpacePose& out) const {
    if (space.street.index >= streets.size()) return false;
    const ModelStreet& S = streets[space.street.index];
    const StreetLine l = lineOf(*this, S.line);
    if (l.count == 0) return false;
    const Vec2 spine = spineDirection(l, space.from, space.to);
    out.facing = spine * static_cast<Real>(space.dir);
    out.at = pointAtStation(l.xy, l.stations, l.count, space.middle()) + towardKerb(spine, space.dir) * S.section.parkingOffset;
    return true;
}

KerbLanePlace RoadModel::kerbLaneAt(ModelStreetId street, int dir, Real station) const {
    KerbLanePlace out;
    if (street.index >= streets.size()) return out;
    const ModelSection& sec = streets[street.index].section;
    // A one-way street asked for the kerb its traffic does not run beside: its lanes all run the other way, and the one
    // nearest THAT kerb is the first of its section (the centre-line side of the way it runs), not the last.
    const bool runsThatWay = (dir > 0 ? sec.lanesForward : sec.lanesBackward) > 0;
    if (!runsThatWay && !sec.oneWay()) return out;
    const bool along = runsThatWay ? dir > 0 : dir < 0;
    for (std::size_t i = 0; i < laneLevel.paths.size(); ++i) {
        const Path& p = laneLevel.paths[i];
        if (p.kind != PathKind::Lane || p.street != street.index || !p.onSpine || p.alongSpine() != along) continue;
        if (strictlyBefore(station, std::min(p.stationFrom, p.stationTo), kWrittenStation) ||
            strictlyPast(station, std::max(p.stationFrom, p.stationTo), kWrittenStation))
            continue;
        if (p.section.index >= laneLevel.sections.size()) continue;
        const uint32_t lanesBeside = laneLevel.sections[p.section.index].lanes.count;
        if (p.indexInSection != (runsThatWay ? lanesBeside - 1 : 0u)) continue;
        out.path = PathId(static_cast<uint32_t>(i));   // (the first in id order, where two paths meet at the station)
        out.along = alongPathAt(out.path, station);
        out.at = laneLevel.pointAt(out.path, out.along);
        return out;
    }
    return out;
}

KerbSpaceNear RoadModel::nearestKerbSpace(const Vec2& p, Real reach) const {
    KerbSpaceNear best;
    best.distance = reach;
    for (std::size_t r = 0; r < kerbRuns.size();) {
        const ModelStreetId street = kerbRuns[r].street;
        const Range runs = kerbRunsOfStreet(*this, street);
        r = runs.end();
        const ModelStreet& S = streets[street.index];
        const StreetLine l = lineOf(*this, S.line);
        if (l.count == 0) continue;
        // (a street whose centre line's box is farther than the best so far, its parking band's offset allowed for,
        // holds no nearer space)
        const LineBox box = boxOfLine(l.xy, l.count);
        const Real dx = std::max(Real(0), std::max(box.lo.x - p.x, p.x - box.hi.x));
        const Real dy = std::max(Real(0), std::max(box.lo.y - p.y, p.y - box.hi.y));
        if (std::sqrt(dx * dx + dy * dy) - S.section.parkingOffset >= best.distance) continue;
        for (const KerbSpace& s : kerbSpacesOf(street)) {
            KerbSpacePose pose;
            if (!kerbSpacePose(s, pose)) continue;
            const Real d = (pose.at - p).length();
            if (d < best.distance) {   // (a search's choice is exact; the first found wins a tie)
                best.distance = d;
                best.space = s;
                best.at = pose.at;
            }
        }
    }
    return best;
}

std::vector<RoadLine> RoadModel::roadLines() const {
    std::vector<RoadLine> out;
    auto add = [&](ModelStreetId street, ModelJunctionId junction, const Range& line, const ModelSection& section) {
        if (!section.carriesCars() || line.count < 2) return;   // (a track or a footway is no road to front or park on)
        RoadLine l;
        l.street = street;
        l.junction = junction;
        l.line = line;
        l.section = &section;
        out.push_back(l);
    };
    for (std::size_t s = 0; s < streets.size(); ++s) add(ModelStreetId(static_cast<uint32_t>(s)), ModelJunctionId(), streets[s].line, streets[s].section);
    for (std::size_t j = 0; j < junctions.size(); ++j)
        for (const ModelStretch& inside : junctions[j].inside) add(ModelStreetId(), ModelJunctionId(static_cast<uint32_t>(j)), inside.line, inside.section);
    return out;
}

RoadFrontage RoadModel::frontageAt(const Vec2& door) const {
    RoadFrontage f;
    const std::vector<RoadLine> lines = roadLines();
    std::vector<LineBox> boxes;
    boxes.reserve(lines.size());
    for (const RoadLine& l : lines) boxes.push_back(boxOfLine(linePoints.data() + l.line.first, l.line.count));
    // (the road model does not state that a street is carried over the ground on a bridge, so none is: as the lot
    // pass's roads all say, lanes/lots_producer.cpp lotRoadInputs)
    const NearestLine n = nearestLine(
        door, Real(1e30), lines.size(), boxes.data(),
        [&](std::size_t i) { return std::pair<const Vec2*, std::size_t>(linePoints.data() + lines[i].line.first, lines[i].line.count); },
        [](std::size_t) { return Real(0); }, [&](std::size_t i) { return buildingMayFront(lines[i].section->klass, false); });
    if (n.line < 0) return f;
    const RoadLine& l = lines[static_cast<std::size_t>(n.line)];
    const uint32_t k = l.line.first + static_cast<uint32_t>(n.seg);
    f.street = l.street;
    f.junction = l.junction;
    f.section = l.section;
    f.station = lineStations[k] + (lineStations[k + 1] - lineStations[k]) * n.t;
    f.distance = n.distance;
    f.on = n.on;
    const Vec2 seg = linePoints[k + 1] - linePoints[k];
    const Vec2 spine = strictlyPast(seg.length(), 0, kSamePoint) ? normalize(seg) : Vec2(1, 0);
    // which kerb the door is on: the one it lies toward from the centre line (on the line itself: the +1 kerb)
    f.dir = dot(door - n.on, towardKerb(spine, 1)) >= 0 ? 1 : -1;
    f.towards = towardKerb(spine, f.dir);
    return f;
}

RoadFrontage RoadModel::frontageOn(ModelStreetId street, Real station, int dir) const {
    RoadFrontage f;
    if (street.index >= streets.size() || (dir != 1 && dir != -1)) return f;
    const ModelStreet& S = streets[street.index];
    const StreetLine l = lineOf(*this, S.line);
    if (l.count == 0) return f;
    const Real at = std::min(std::max(station, S.from), S.to);
    const StationSpot spot = spotAtStation(l.stations, l.count, at);
    const Vec2 seg = l.xy[spot.seg + 1] - l.xy[spot.seg];
    const Vec2 spine = strictlyPast(seg.length(), 0, kSamePoint) ? normalize(seg) : Vec2(1, 0);
    f.street = street;
    f.section = &S.section;
    f.station = at;
    f.dir = dir;
    f.on = pointAtStation(l.xy, l.stations, l.count, at);
    f.towards = towardKerb(spine, dir);
    return f;
}

bool RoadModel::sidewalkBeside(const RoadFrontage& f, Vec2& out) const {
    if (!f.valid() || f.section->sidewalk <= 0) return false;
    out = f.on + f.towards * (f.section->carriagewayHalfWidth() + f.section->sidewalk * Real(0.5));
    return true;
}

std::vector<KerbSpaceNear> RoadModel::kerbSpacesNear(const RoadFrontage& f, Real reach, std::size_t most) const {
    std::vector<KerbSpaceNear> out;
    if (!f.valid() || most == 0) return out;
    // THE WALK ALONG THE STREETS: the shortest way from the frontage to each end of each street within reach, going
    // along streets that have a sidewalk and across from one street's end to another's where they share a junction
    // or one carries on as the other (the straight line between the two ends: the junction's width).
    constexpr Real kFar = std::numeric_limits<Real>::infinity();
    std::vector<std::array<Real, 2>> toEnd(streets.size(), std::array<Real, 2>{kFar, kFar});
    std::map<uint32_t, std::vector<std::pair<uint32_t, int>>> endsAt;   // junction -> the street ends at it
    for (std::size_t s = 0; s < streets.size(); ++s)
        for (int end = 0; end < 2; ++end)
            if (streets[s].junctions[static_cast<std::size_t>(end)].index < junctions.size())
                endsAt[streets[s].junctions[static_cast<std::size_t>(end)].index].emplace_back(static_cast<uint32_t>(s), end);
    const auto walkable = [&](std::size_t s) { return streets[s].section.sidewalk > 0 && streets[s].line.count >= 2; };
    std::set<std::tuple<Real, uint32_t, int>> open;   // (metres walked, street, end), the nearest first
    const auto reachEnd = [&](uint32_t s, int end, Real walked) {
        if (!walkable(s) || walked > reach || walked >= toEnd[s][static_cast<std::size_t>(end)]) return;
        open.erase(std::make_tuple(toEnd[s][static_cast<std::size_t>(end)], s, end));
        toEnd[s][static_cast<std::size_t>(end)] = walked;
        open.insert(std::make_tuple(walked, s, end));
    };
    if (f.street.valid()) {
        const ModelStreet& S = streets[f.street.index];
        reachEnd(f.street.index, 0, std::max(Real(0), f.station - S.from));
        reachEnd(f.street.index, 1, std::max(Real(0), S.to - f.station));
    } else if (f.junction.index < junctions.size()) {
        // a door fronting a stretch of road inside a junction: the walk starts for every street end at that junction
        const std::map<uint32_t, std::vector<std::pair<uint32_t, int>>>::const_iterator at = endsAt.find(f.junction.index);
        if (at != endsAt.end())
            for (const std::pair<uint32_t, int>& e : at->second) {
                Vec2 end;
                if (streetEnd(streets[e.first], e.second, end)) reachEnd(e.first, e.second, (end - f.on).length());
            }
    }
    while (!open.empty()) {
        const std::tuple<Real, uint32_t, int> next = *open.begin();
        open.erase(open.begin());
        const Real walked = std::get<0>(next);
        const uint32_t s = std::get<1>(next);
        const int end = std::get<2>(next);
        const ModelStreet& S = streets[s];
        reachEnd(s, 1 - end, walked + (S.to - S.from));   // along the street to its other end
        Vec2 here;
        if (!streetEnd(S, end, here)) continue;
        const auto across = [&](uint32_t other, int otherEnd) {
            Vec2 there;
            if (other == s || !streetEnd(streets[other], otherEnd, there)) return;
            reachEnd(other, otherEnd, walked + (there - here).length());
        };
        const ModelJunctionId j = S.junctions[static_cast<std::size_t>(end)];
        if (j.index < junctions.size())
            for (const std::pair<uint32_t, int>& e : endsAt[j.index]) across(e.first, e.second);
        const ModelStreetId on = S.carriesOn[static_cast<std::size_t>(end)];
        if (on.index < streets.size()) {
            // (the end of the other street that meets this one: the nearer of its two)
            Vec2 a, b;
            if (streetEnd(streets[on.index], 0, a) && streetEnd(streets[on.index], 1, b))
                across(on.index, (a - here).length() <= (b - here).length() ? 0 : 1);
        }
    }
    for (std::size_t r = 0; r < kerbRuns.size();) {
        const ModelStreetId street = kerbRuns[r].street;
        r = kerbRunsOfStreet(*this, street).end();
        const ModelStreet& S = streets[street.index];
        const bool own = f.street == street;
        if (!own && toEnd[street.index][0] == kFar && toEnd[street.index][1] == kFar) continue;
        for (const KerbSpace& s : kerbSpacesOf(street)) {
            const Real mid = s.middle();
            Real walked = std::min(toEnd[street.index][0] + (mid - S.from), toEnd[street.index][1] + (S.to - mid));
            // on the door's own street: straight along it, and across the road to a space on the other kerb
            if (own) walked = std::min(walked, std::fabs(mid - f.station)) + (s.dir != f.dir ? S.section.travelWidth() : Real(0));
            if (walked > reach) continue;
            KerbSpaceNear n;
            n.space = s;
            n.distance = walked;
            KerbSpacePose pose;
            if (kerbSpacePose(s, pose)) n.at = pose.at;
            out.push_back(n);
        }
    }
    // (a sort's order is exact; equal walks keep the order of street, kerb and index they were listed in)
    std::stable_sort(out.begin(), out.end(), [](const KerbSpaceNear& a, const KerbSpaceNear& b) { return a.distance < b.distance; });
    if (out.size() > most) out.resize(most);
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

bool parseKerbParkingRules(const nlohmann::json& doc, KerbParkingRules& out, std::string* err) {
    out = KerbParkingRules();
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return false;
    };
    if (!doc.is_object()) return fail("it is not a table of numbers");
    if (!doc.contains("space_length") || !doc["space_length"].is_number() || !(doc["space_length"].get<double>() > 0))
        return fail("it gives no \"space_length\" above zero");
    if (!doc.contains("clear_of_stop_line") || !doc["clear_of_stop_line"].is_number() || doc["clear_of_stop_line"].get<double>() < 0)
        return fail("it gives no \"clear_of_stop_line\" (zero or more)");
    out.spaceLength = doc["space_length"].get<double>();
    out.clearOfStopLine = doc["clear_of_stop_line"].get<double>();
    out.usable = true;
    return true;
}

const KerbParkingRules& kerbParkingRules() {
    static const KerbParkingRules rules = [] {
        const std::string path = assetPath("assets/data/kerb_parking.json");
        KerbParkingRules r;
        std::ifstream in(path, std::ios::binary);
        if (!in) {
            LOG_WARN << "[roads] " << path << " is missing: no street is given a kerb parking space";
            return r;
        }
        const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        std::string why;
        const nlohmann::json doc = nlohmann::json::parse(text, nullptr, false);
        if (doc.is_discarded() || !parseKerbParkingRules(doc, r, &why))
            LOG_WARN << "[roads] " << path << ": " << (doc.is_discarded() ? std::string("it is not JSON") : why)
                     << ": no street is given a kerb parking space";
        r.hash = bundle::fnv1aStr(text);   // (of the file as it is, usable or not: a model saved without spaces is keyed by it)
        return r;
    }();
    return rules;
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

bool stateRegulation(RoadModel& m, const ModelRegulation& r, std::string* err) {
    auto fail = [&](const std::string& why) {
        if (err) *err = why;
        return false;
    };
    if (r.kind.empty()) return fail("a regulation names no kind");
    if (!r.street.valid() || r.street.index >= m.streets.size()) return fail("a " + r.kind + " regulation names no street of the model");
    if (r.dir != 1 && r.dir != -1) return fail("a " + r.kind + " regulation holds for traffic along the spine (+1) or against it (-1), not " + std::to_string(r.dir));
    const ModelStreet& S = m.streets[r.street.index];
    if (strictlyPast(r.from, r.to, kWrittenStation))
        return fail(fmt("a %s regulation on street %s runs backwards (from %.2f to %.2f)", r.kind.c_str(), S.id.c_str(), r.from, r.to));
    if (strictlyBefore(r.from, S.from, kWrittenStation) || strictlyPast(r.to, S.to, kWrittenStation))
        return fail(fmt("a %s regulation from %.2f to %.2f lies off street %s (%.2f to %.2f)", r.kind.c_str(), r.from, r.to, S.id.c_str(),
                        S.from, S.to));
    m.regulations.push_back(r);
    return true;
}

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
    for (const ModelRegulation& r : m.regulations)
        out += fmt("regulation: %s on street %s, %s from %.2f to %.2f, value %.2f\n", r.kind.c_str(),
                   r.street.index < m.streets.size() ? m.streets[r.street.index].id.c_str() : "?",
                   r.dir > 0 ? "along the spine" : "against the spine", r.from, r.to, r.value);
    for (const ModelKerbRun& run : m.kerbRuns)
        out += fmt("kerb parking: street %s, the kerb of the traffic running %s the spine, %u spaces of %.2f m from station %.2f to %.2f\n",
                   run.street.index < m.streets.size() ? m.streets[run.street.index].id.c_str() : "?", run.dir > 0 ? "along" : "against",
                   run.count, run.spaceLength, run.from, run.to());
    return out;
}

std::vector<std::string> kerbSpaceLines(const RoadModel& m) {
    std::vector<std::string> out;
    for (std::size_t r = 0; r < m.kerbRuns.size();) {
        const ModelStreetId street = m.kerbRuns[r].street;
        r = kerbRunsOfStreet(m, street).end();
        const ModelStreet& S = m.streets[street.index];
        const std::vector<KerbSpace> spaces = m.kerbSpacesOf(street);
        for (int dir : {1, -1}) {
            uint32_t count = 0;
            for (const KerbSpace& s : spaces) count += s.dir == dir ? 1u : 0u;
            if (count == 0) continue;
            out.push_back(fmt("kerb parking on street %s, the kerb of the traffic running %s the spine: %u spaces, each %.2f m long and %.2f m wide, "
                              "their middle %.2f m from the centre line\n", S.id.c_str(), dir > 0 ? "along" : "against", count,
                              spaces.front().length, S.section.parking, S.section.parkingOffset));
            for (const KerbSpace& s : spaces) {
                if (s.dir != dir) continue;
                KerbSpacePose pose;
                m.kerbSpacePose(s, pose);
                const KerbLanePlace lane = m.kerbLaneBeside(s);
                out.push_back(fmt("  space %s: stations %.2f to %.2f, a car in it at (%.2f, %.2f) facing (%.3f, %.3f); kerb lane beside it: %s\n",
                                  m.kerbSpaceName(s).c_str(), s.from, s.to, pose.at.x, pose.at.y, pose.facing.x + Real(0), pose.facing.y + Real(0),   // (+ 0: no "-0.000")
                                  lane.valid() ? fmt("lane path %u at %.2f m along", lane.path.index, lane.along).c_str() : "none"));
            }
        }
    }
    return out;
}

KerbSpaceCounts kerbSpaceCounts(const RoadModel& m) {
    KerbSpaceCounts c;
    for (const ModelStreet& S : m.streets) c.streetsWithBand += S.section.parking > 0 ? 1u : 0u;
    uint32_t last = kNoIndex;
    for (const ModelKerbRun& run : m.kerbRuns) {
        if (run.street.index != last) ++c.streets;
        last = run.street.index;
        c.spaces += run.count;
        c.kerbMetres += static_cast<Real>(run.count) * run.spaceLength;
    }
    return c;
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
    {   // the crosswalks (ADR-0452)
        uint32_t marked = 0, pavements = 0, laid = 0, crossers = 0;
        std::array<uint32_t, kCrosswalkControlCount> byControl{};
        Real shortest = 0, longest = 0;
        for (uint32_t i = 0; i < m.crosswalks.size(); ++i) {
            const ModelCrosswalk& c = m.crosswalks[i];
            marked += c.marked ? 1u : 0u;
            pavements += c.pavements ? 1u : 0u;
            laid += c.laneCrosswalk != kNoIndex ? 1u : 0u;
            crossers += static_cast<uint32_t>(m.crossersOf(i).size());
            ++byControl[static_cast<std::size_t>(c.control)];
            shortest = i == 0 ? c.length : std::min(shortest, c.length);
            longest = std::max(longest, c.length);
        }
        out += fmt("  %zu crosswalks (ADR-0452): %u marked, %u with pavements; %u under lights, %u at signs, %u with neither; "
                   "%.1f to %.1f m kerb to kerb; %u in the lane level, crossed by %u path stretches\n",
                   m.crosswalks.size(), marked, pavements, byControl[0], byControl[1], byControl[2], shortest, longest, laid, crossers);
    }
    {
        std::map<std::string, uint32_t> byKind;
        for (const ModelAttachment& a : m.attachments) ++byKind[a.kind];
        out += fmt("  %zu attachments:", m.attachments.size());
        for (const std::pair<const std::string, uint32_t>& k : byKind) out += fmt(" %u %s;", k.second, k.first.c_str());
        out += "\n";
    }
    {
        const KerbSpaceCounts c = kerbSpaceCounts(m);
        out += fmt("  kerb parking (ADR-0439): %u streets state a parking band, %u of them hold spaces: %u spaces along %.1f m of kerb\n",
                   c.streetsWithBand, c.streets, c.spaces, c.kerbMetres);
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
    const std::vector<Vec2>& lanePoints = m.lanePoints();
    for (const Vec2& p : lanePoints) { x0 = std::min(x0, p.x); y0 = std::min(y0, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
    if (lanePoints.empty()) { x0 = y0 = 0; x1 = y1 = 1; }
    const double pad = 20, w = x1 - x0 + 2 * pad, h = y1 - y0 + 2 * pad;
    std::string out = fmt("<svg xmlns='http://www.w3.org/2000/svg' viewBox='%.1f %.1f %.1f %.1f' font-family='sans-serif'>\n", x0 - pad, -(y1 + pad), w, h);
    out += fmt("<rect x='%.1f' y='%.1f' width='%.1f' height='%.1f' fill='#f6f5f0'/>\n<g transform='scale(1,-1)'>\n", x0 - pad, -(y1 + pad), w, h);
    // lanes: thin, by direction
    for (const ModelLane& L : m.lanes) {
        if (L.curve.count < 2) continue;
        out += fmt("<polyline fill='none' stroke='%s' stroke-width='0.35' stroke-opacity='0.6' points='", L.edge == kNoIndex ? "#9a6" : L.dir > 0 ? "#3b6fb6" : "#c0573e");
        for (uint32_t k = L.curve.first; k < L.curve.end(); ++k) out += fmt("%.1f,%.1f ", lanePoints[k].x, lanePoints[k].y);
        out += "'/>\n";
    }
    // kerb parking spaces (ADR-0439): each an outline where a car in it stands, as long and as wide as the space
    for (std::size_t r = 0; r < m.kerbRuns.size();) {
        const ModelStreetId street = m.kerbRuns[r].street;
        r = kerbRunsOfStreet(m, street).end();
        for (const KerbSpace& s : m.kerbSpacesOf(street)) {
            KerbSpacePose pose;
            if (!m.kerbSpacePose(s, pose)) continue;
            const Vec2 along = pose.facing * (s.length * Real(0.5)), across = perp(pose.facing) * (s.width * Real(0.5));
            out += "<polygon fill='#2e9d6b' fill-opacity='0.35' stroke='#1c6b47' stroke-width='0.15' points='";
            for (const Vec2& c : {pose.at - along - across, pose.at + along - across, pose.at + along + across, pose.at - along + across})
                out += fmt("%.2f,%.2f ", c.x, c.y);
            out += "'/>\n";
        }
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
                label(lanePoints[k], S.id, "#555", 4.0);
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
           std::is_same<T, ModelNode>::value || std::is_same<T, ModelNodePoint>::value || std::is_same<T, ModelTurn>::value ||
           std::is_same<T, ModelRegulation>::value || std::is_same<T, ModelKerbRun>::value || std::is_same<T, ModelCrosswalk>::value;
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
template <class IO, class R> void fields(IO& io, R& r, ModelRegulation*) {
    io(r.kind); io(r.street); io(r.from); io(r.to); io(r.dir); io(r.value);
}
template <class IO, class K> void fields(IO& io, K& k, ModelKerbRun*) {
    io(k.street); io(k.dir); io(k.from); io(k.spaceLength); io(k.count);
}
template <class IO, class C> void fields(IO& io, C& c, ModelCrosswalk*) {
    io(c.junction); io(c.arm); io(c.street); io(c.road); io(c.nearStation); io(c.farStation); io(c.ends); io(c.length);
    io(c.spineIn); io(c.marked); io(c.pavements);
    uint8_t control = static_cast<uint8_t>(c.control);
    io(control);
    c.control = static_cast<CrosswalkControl>(control);
    io(c.laneCrosswalk);
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
    io(m.name); io(m.built); io(m.junctions); io(m.streets); io(m.lanes); io(m.connections); io(m.turns); io(m.ramps); io(m.crossings);
    // the lanes' centre lines: written from the object the model holds; read into one of the model's own
    std::shared_ptr<LaneCurves> curves = std::make_shared<LaneCurves>();
    if (m.curves) *curves = *m.curves;
    io(curves->points); io(curves->along);
    m.curves = std::move(curves);
    io(m.heights); io(m.spineStations); io(m.linePoints); io(m.lineStations); io(m.attachments); io(m.regulations); io(m.kerbRuns);
    io(m.crosswalks);
    io(m.notStated);
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

// ---- crosswalks (ADR-0452) ----------------------------------------------------------------------------------------------

const char* crosswalkControlWord(CrosswalkControl c) {
    switch (c) {
    case CrosswalkControl::Lights: return "lights";
    case CrosswalkControl::Signs: return "signs";
    case CrosswalkControl::None: return "none";
    }
    return "none";
}

const ModelCrosswalk* RoadModel::crosswalkAcross(ModelJunctionId junction, uint32_t arm) const {
    for (const ModelCrosswalk& c : crosswalks)
        if (c.junction == junction && c.arm == arm) return &c;
    return nullptr;
}

std::vector<CrosswalkCrosser> RoadModel::crossersOf(uint32_t crosswalk) const {
    std::vector<CrosswalkCrosser> out;
    if (crosswalk >= crosswalks.size()) return out;
    const uint32_t w = crosswalks[crosswalk].laneCrosswalk;
    if (w >= laneLevel.crosswalks.size()) return out;
    const JunctionId lj = laneLevel.crosswalks[w].junction;
    // (a crosswalk's crossing points are filed under its junction: its range of the list, else the whole list)
    uint32_t first = 0, end = static_cast<uint32_t>(laneLevel.crossings.size());
    if (lj.valid() && lj.index < laneLevel.junctions.size()) {
        first = laneLevel.junctions[lj.index].crossings.first;
        end = laneLevel.junctions[lj.index].crossings.end();
    }
    for (uint32_t x = first; x < end && x < laneLevel.crossings.size(); ++x) {
        const CrossingPoint& cp = laneLevel.crossings[x];
        if (cp.otherSide != OtherSide::Crosswalk || cp.other != w) continue;
        CrosswalkCrosser c;
        c.path = cp.a;
        c.from = cp.aFrom;
        c.to = cp.aTo;
        c.acrossFrom = cp.otherFrom;
        c.acrossTo = cp.otherTo;
        out.push_back(c);
    }
    return out;
}

std::vector<WalkCrossing> RoadModel::crosswalksOnWalk(ModelJunctionId junction, int armIn, int armOut) const {
    std::vector<WalkCrossing> out;
    if (junction.index >= junctions.size()) return out;
    const int n = static_cast<int>(junctions[junction.index].arms.size());
    if (armIn < 0 || armOut < 0 || armIn >= n || armOut >= n) return out;
    // s: the ring step (+1 counter-clockwise, the list's order) toward the arriving traffic's kerb. A person arriving by
    // arm i stands at the corner between i and i+s; leaving by arm k, at the corner between k-s and k.
    const int s = kerbSideCounterClockwise() ? 1 : -1;
    const auto wrap = [n](int a) { return ((a % n) + n) % n; };
    const auto stepsFrom = [&](int dir) {   // steps from armIn to armOut going `dir` round the ring
        for (int k = 0; k < n; ++k)
            if (wrap(armIn + dir * k) == armOut) return k;
        return n;
    };
    // going s: across armIn+s ... armOut-s (steps - 1 arms); going -s: across armIn ... armOut (steps + 1 arms)
    const int forward = stepsFrom(s) - 1, backward = stepsFrom(-s) + 1;
    const bool goForward = forward <= backward;
    const int count = goForward ? forward : backward;
    for (int k = 0; k < count; ++k) {
        const int arm = goForward ? wrap(armIn + s * (k + 1)) : wrap(armIn - s * k);
        const ModelCrosswalk* c = crosswalkAcross(junction, static_cast<uint32_t>(arm));
        if (!c || !c->pavements) continue;
        // which kerb end is at which corner: end 0 is on the kerb of the traffic running along the spine. Where the spine
        // runs into the junction that traffic arrives by this arm, so end 0 is at the corner (arm, arm+s); else at
        // (arm-s, arm). Going s a person comes from (arm-s, arm); going -s from (arm, arm+s).
        WalkCrossing w;
        w.crosswalk = static_cast<uint32_t>(c - crosswalks.data());
        w.fromEnd = goForward == c->spineIn ? 1 : 0;
        out.push_back(w);
    }
    return out;
}

bool stateCrosswalk(RoadModel& m, const ModelCrosswalk& c, uint32_t* id, std::string* err) {
    if (c.junction.index >= m.junctions.size()) {
        if (err) *err = fmt("crosswalk: junction %u is not one of the model's %zu", c.junction.index, m.junctions.size());
        return false;
    }
    if (c.arm >= m.junctions[c.junction.index].arms.size()) {
        if (err) *err = fmt("crosswalk: junction %u lists no arm %u", c.junction.index, c.arm);
        return false;
    }
    if (id) *id = static_cast<uint32_t>(m.crosswalks.size());
    m.crosswalks.push_back(c);
    return true;
}

void layLaneCrosswalks(RoadModel& m) {
    LaneGraph& g = m.laneLevel;
    g.crosswalks.clear();
    for (ModelCrosswalk& c : m.crosswalks) c.laneCrosswalk = kNoIndex;
    for (uint32_t lj = 0; lj < g.junctions.size(); ++lj) {
        Junction& jn = g.junctions[lj];
        jn.crosswalks = Range{static_cast<uint32_t>(g.crosswalks.size()), 0};
        if (jn.modelJunction == kNoIndex) continue;
        for (uint32_t id = 0; id < m.crosswalks.size(); ++id) {
            ModelCrosswalk& c = m.crosswalks[id];
            if (c.junction.index != jn.modelJunction) continue;
            // the band, kerb to kerb: the edge at end 0 first, then the edge at end 1 (lane_graph.h Crosswalk)
            const Vec2 along = perp(c.across()) * (c.bandWidth() * Real(0.5));
            Crosswalk w;
            w.outline = g.addPlanPoints({c.ends[0] - along, c.ends[0] + along, c.ends[1] + along, c.ends[1] - along});
            w.junction = JunctionId(lj);
            w.modelCrosswalk = id;
            c.laneCrosswalk = static_cast<uint32_t>(g.crosswalks.size());
            g.crosswalks.push_back(w);
            ++jn.crosswalks.count;
        }
    }
}

bool operator==(const RoadModel& a, const RoadModel& b) {
    bundle::BinWriter wa, wb;
    putRoadModel(wa, a);
    putRoadModel(wb, b);
    return wa.bytes == wb.bytes;
}

}  // namespace engine
