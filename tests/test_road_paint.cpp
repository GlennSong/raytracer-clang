// THE ROAD PAINT SAYS WHAT THE ROAD MODEL STATES (issue #292, lane level finding 1; ADR-0431). The lines between lanes,
// the stop bars and the lane arrows are read off the built road model, each through the one function that answers its
// question: lane_turns.h laneLineBetween (which the lane level fills its table from), control_signs.h markingsOnArm,
// and RoadModel::turns (lane_turns.h turnsAtJunction). buildMeshes hands back the list it draws from (deck_mesh.h
// RoadPaint); these tests read that list on a hand scene and compare it with values worked out here from the scene.
//
// THE SCENE (flat, plan metres; stations are along each road's spine, which starts at its first point):
//   main  (-300, 0) -> (300, 0): two lanes each way, 3.5 m, a 4 m median gap; its station is x + 300. Half width 9.0
//         (the gap's 2, two lanes' 7). A far-turn pocket "P" for the traffic heading +x, on the centre-line side, from
//         station 200 to the four-way at 300.
//   cross (0, -200) -> (0, 200): two lanes each way, 3.5 m, no gap; its station is y + 200. Half width 7.0.
//   stem  (150, -200) -> (150, 0): the same section; it ends on main at main's station 450.
//   Junction A, the four-way at (0, 0): main's station 300, cross's station 200.
//   Junction B, the T at (150, 0): main's station 450, stem's station 200 (its end).
// STOP LINES (deck_mesh.h junctionAreas: back from the mouth by the widest crossing road's half width, then 5.2 m):
//   A, main's arms: 300 -/+ 7.0 -/+ 5.2 = 287.8 (arriving from the west) and 312.2;
//   A, cross's arms: 200 -/+ 9.0 -/+ 5.2 = 185.8 and 214.2;
//   B, main's arms: 450 -/+ 7.0 -/+ 5.2 = 437.8 and 462.2; B, stem's arm: 200 - 9.0 - 5.2 = 185.8.
// THE KERB: traffic keeps left in the world; in plan coordinates the kerb of traffic heading +x lies toward -y.

#include "test_framework.h"

#include "lane_level_scene.h"

#include "engine/procgen/city/roads/control_signs.h"
#include "engine/procgen/city/roads/lane_turns.h"
#include "engine/procgen/city/roads/lanes/deck_mesh.h"
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/road_model.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace {

using namespace engine;
using namespace engine::roads::lanes;

constexpr double kPaintStation = 0.02;   // a painted station against one worked out by hand (metres)

nlohmann::json paintSceneJson() {
    return nlohmann::json::parse(R"({
      "name": "paint_scene",
      "classes": {"collector": {"w": 3.5, "fwd": 2, "back": 2, "sidewalk": 2.0}},
      "edges": [
        {"id": "main", "class": "collector", "path": {"type": "polyline", "points": [[-300, 0], [300, 0]]},
         "lanes": {"gap": 4.0, "pockets": [{"id": "P", "side": "left", "s0": 200, "s1": 300, "taper": 30, "kind": "turn"}]}},
        {"id": "cross", "class": "collector", "path": {"type": "polyline", "points": [[0, -200], [0, 200]]}},
        {"id": "stem", "class": "collector", "path": {"type": "polyline", "points": [[150, -200], [150, 0]]}}
      ]
    })");
}

// One arm's stated control, written by hand into the scene: the junction near `at`, its arm on `road` with the road's
// spine running in or out.
struct ArmSigns {
    const char* road;
    bool spineIn;
    std::vector<std::string> regulations;
};
struct JunctionControl {
    Vec2 at;
    std::string control;
    std::vector<ArmSigns> arms;
};

struct PaintScene {
    std::unique_ptr<Result> r;
    RoadPaint paint;
    int A = -1, B = -1;   // the four-way and the T, as indices into the road file's junctions
    int arm(int junction, const std::string& road, bool spineIn) const {
        const ModelJunction& J = r->graph.junctions[static_cast<std::size_t>(junction)];
        for (std::size_t a = 0; a < J.arms.size(); ++a)
            if (J.arms[a].road == road && J.arms[a].spineIn == spineIn) return static_cast<int>(a);
        return -1;
    }
    int lane(const std::string& id) const { return r->lanes.find(id); }
    const std::string& laneId(int li) const { return r->lanes.lanes[static_cast<std::size_t>(li)].id; }
};

// The scene built with the layout derived (as `lanes_tool layout --write` would), lane rules of 20 m, and the controls
// and signs given written onto its junctions; painted. `classFacts`: members written over the scene's one class row
// (its roads are all collectors) before anything is derived.
PaintScene paintScene(const std::vector<JunctionControl>& facts, const nlohmann::json& classFacts = nlohmann::json::object()) {
    nlohmann::json scene = paintSceneJson();
    for (const auto& [key, value] : classFacts.items()) scene["classes"]["collector"][key] = value;
    nlohmann::json g = lane_level_test::withLaneLevelFacts(scene);
    for (nlohmann::json& j : g["junctions"]) {
        const Vec2 at(j["at"][0].get<double>(), j["at"][1].get<double>());
        for (const JunctionControl& f : facts) {
            if (distance(at, f.at) > 1.0) continue;
            j["control"] = f.control;
            for (nlohmann::json& a : j["arms"])
                for (const ArmSigns& af : f.arms)
                    if (a["road"].get<std::string>() == af.road && (a["spine"].get<std::string>() == "in") == af.spineIn)
                        a["regulations"] = af.regulations;
        }
    }
    PaintScene s;
    s.r = build(RoadLabGraph::fromJson(g));
    for (std::size_t j = 0; j < s.r->graph.junctions.size(); ++j) {
        if (distance(s.r->graph.junctions[j].at, Vec2(0, 0)) < 1.0) s.A = static_cast<int>(j);
        if (distance(s.r->graph.junctions[j].at, Vec2(150, 0)) < 1.0) s.B = static_cast<int>(j);
    }
    buildMeshes(*s.r, &s.paint);
    return s;
}

// The four-way under traffic lights and the T with a stop sign on its stem: the scene the arrows and the lines are read on.
const PaintScene& signalScene() {
    static const PaintScene s = paintScene({{Vec2(0, 0), "signal", {}}, {Vec2(150, 0), "stop", {{"stem", true, {"stop"}}}}});
    return s;
}

// The line painted along `lane`'s kerb-side (or centre-line-side) edge at spine station `station`: its kind (None:
// nothing is painted there), and the lane beyond it.
struct LineAt {
    PaintedLineKind kind = PaintedLineKind::None;
    int other = -1;
};
LineAt lineAt(const RoadPaint& paint, int lane, bool kerbSide, double station) {
    LineAt out;
    for (const PaintedLine& l : paint.lines)
        if (l.lane == lane && l.kerbSide == kerbSide && station > l.from && station < l.to) {
            out.kind = l.kind;
            out.other = l.other;
        }
    return out;
}
const char* kindName(PaintedLineKind k) {
    switch (k) {
        case PaintedLineKind::Edge: return "edge";
        case PaintedLineKind::Centre: return "centre";
        case PaintedLineKind::Dashed: return "dashed";
        case PaintedLineKind::Solid: return "solid";
        default: return "none";
    }
}

LaneGraphOptions paintTestCar() {
    LaneGraphOptions o;
    VehicleSize car;
    car.tag = "car";
    car.turningRadius = 5.0;
    car.width = 1.9;
    car.length = 4.5;
    o.vehicles = {car};
    return o;
}

}  // namespace

// (c) THE LINE BETWEEN TWO LANES IS DRAWN AS THE LANE TABLE STATES IT: solid for the 20 m before the stop line two
// through lanes run to, dashed elsewhere and beside a pocket, nothing past the stop line; edge and centre lines where no
// lane of the same way lies beyond.
TEST_CASE(road_paint_lane_lines_are_solid_exactly_where_the_lane_table_forbids_crossing) {
    const PaintScene& s = signalScene();
    CHECK(s.A >= 0 && s.B >= 0);
    if (s.A < 0 || s.B < 0) return;
    const int f0 = s.lane("main.f0"), f1 = s.lane("main.f1"), b0 = s.lane("main.b0"), b1 = s.lane("main.b1"), P = s.lane("main.P");
    const int c0 = s.lane("cross.f0"), c1 = s.lane("cross.f1");
    CHECK(f0 >= 0 && f1 >= 0 && b0 >= 0 && b1 >= 0 && P >= 0 && c0 >= 0 && c1 >= 0);
    if (f0 < 0 || f1 < 0 || b0 < 0 || b1 < 0 || P < 0 || c0 < 0 || c1 < 0) return;
    // ---- by hand, at written stations --------------------------------------------------------------------------------
    struct Want {
        int lane;
        bool kerbSide;
        double station;
        PaintedLineKind kind;
        int other;
        const char* why;
    };
    const std::vector<Want> wants = {
        // main, heading +x, toward A's stop line at 287.8: f0 | f1, both through lanes
        {f0, true, 100.0, PaintedLineKind::Dashed, f1, "mid-block"},
        {f0, true, 267.0, PaintedLineKind::Dashed, f1, "0.8 m before the solid stretch begins (287.8 - 20 = 267.8)"},
        {f0, true, 268.5, PaintedLineKind::Solid, f1, "0.7 m into the solid stretch"},
        {f0, true, 280.0, PaintedLineKind::Solid, f1, "inside the solid stretch"},
        {f0, true, 287.0, PaintedLineKind::Solid, f1, "0.8 m before the stop line"},
        {f0, true, 290.0, PaintedLineKind::None, -1, "past the stop line (287.8), before the junction's mouth"},
        // ...and on beyond A toward B's stop line at 437.8: from A's far stop line 312.2
        {f0, true, 310.0, PaintedLineKind::None, -1, "between A's mouth and its far stop line (312.2)"},
        {f0, true, 350.0, PaintedLineKind::Dashed, f1, "between A and B"},
        {f0, true, 417.0, PaintedLineKind::Dashed, f1, "0.8 m before the solid stretch before B (437.8 - 20 = 417.8)"},
        {f0, true, 425.0, PaintedLineKind::Solid, f1, "inside the solid stretch before B"},
        // past B the road runs to its end, no stop line: never solid
        {f0, true, 590.0, PaintedLineKind::Dashed, f1, "10 m before the road's end, no junction there"},
        // the pocket (a turn lane, on the centre-line side of f0): its line may be crossed right up to the stop line
        {P, true, 250.0, PaintedLineKind::Dashed, f0, "beside the pocket, mid-way"},
        {P, true, 280.0, PaintedLineKind::Dashed, f0, "beside the pocket, where two through lanes would be solid"},
        {P, true, 290.0, PaintedLineKind::None, -1, "past the stop line"},
        // main, heading -x (its lanes run against the spine): toward A's stop line at 312.2, solid from 332.2 down to it
        {b0, true, 500.0, PaintedLineKind::Dashed, b1, "between B and the road's end... heading for B's stop line at 462.2"},
        {b0, true, 470.0, PaintedLineKind::Solid, b1, "inside the solid stretch before B (462.2 to 482.2)"},
        {b0, true, 400.0, PaintedLineKind::Dashed, b1, "between B and A"},
        {b0, true, 333.0, PaintedLineKind::Dashed, b1, "0.8 m before the solid stretch before A begins (332.2)"},
        {b0, true, 320.0, PaintedLineKind::Solid, b1, "inside the solid stretch before A"},
        {b0, true, 100.0, PaintedLineKind::Dashed, b1, "past A, running to the road's end"},
        // edges and centre lines. main states a 4 m gap: no shared centre line, each way has its own edge
        {f1, true, 100.0, PaintedLineKind::Edge, -1, "the kerb lane's kerb side: the road's edge"},
        {f0, false, 100.0, PaintedLineKind::Edge, -1, "the lane nearest the centre, beside a stated gap"},
        {f1, false, 100.0, PaintedLineKind::None, -1, "its centre-line side is f0's kerb side: drawn there, once"},
        {f0, false, 250.0, PaintedLineKind::None, -1, "the pocket lies nearer the centre here"},
        {P, false, 250.0, PaintedLineKind::Edge, -1, "the pocket is nearest the centre"},
        // cross states no gap: one centre line, drawn by the lane running along the spine
        {c0, false, 100.0, PaintedLineKind::Centre, -1, "two ways, no gap: the centre line"},
        {s.lane("cross.b0"), false, 100.0, PaintedLineKind::None, -1, "the same centre line: drawn once, by the other way's lane"},
        {c0, true, 170.0, PaintedLineKind::Solid, c1, "cross toward A's stop line at 185.8: solid from 165.8"},
        {c0, true, 160.0, PaintedLineKind::Dashed, c1, "5.8 m before the solid stretch"},
    };
    for (const Want& w : wants) {
        const LineAt got = lineAt(s.paint, w.lane, w.kerbSide, w.station);
        if (got.kind != w.kind || got.other != w.other)
            std::printf("    lane %s %s side at %.1f: painted %s (beyond: %s), expected %s (%s)\n", s.laneId(w.lane).c_str(),
                        w.kerbSide ? "kerb" : "centre-line", w.station, kindName(got.kind), got.other >= 0 ? s.laneId(got.other).c_str() : "-",
                        kindName(w.kind), w.why);
        CHECK(got.kind == w.kind);
        CHECK(got.other == w.other);
    }
    // the solid stretch's own ends, as painted: 267.8 to 287.8
    bool foundSolid = false;
    for (const PaintedLine& l : s.paint.lines)
        if (l.lane == f0 && l.kind == PaintedLineKind::Solid && l.from < 280.0 && l.to > 280.0) {
            foundSolid = true;
            CHECK(std::fabs(l.from - 267.8) < kPaintStation);
            CHECK(std::fabs(l.to - 287.8) < kPaintStation);
            CHECK(std::fabs(l.metres - 20.0) < kPaintStation);
        }
    CHECK(foundSolid);

    // ---- against the lane table, everywhere: each line of each section is painted, solid exactly where it may not be
    // crossed; and each painted line between lanes lies on a section that states it so -------------------------------
    LaneGraphBuild lg;
    const RoadModel m = assembleRoadModel(*s.r, paintTestCar(), &lg);
    const LaneGraph& G = m.laneLevel;
    CHECK(lg.error.empty());
    int tableLines = 0, tableSolid = 0, paintedBetween = 0;
    for (std::size_t sec = 0; sec < G.sections.size(); ++sec) {
        const SectionSource& ss = lg.sectionSources[sec];
        const double roadLength = s.r->graph.edges[static_cast<std::size_t>(ss.edge)].s.back();
        const double midU = 0.5 * (ss.from + ss.to), station = ss.dir > 0 ? midU : roadLength - midU;
        for (uint32_t k = 0; k + 1 < G.sections[sec].lanes.count; ++k) {
            const int offside = lg.sources[G.sectionLanes[G.sections[sec].lanes.first + k].index].lane;
            const int kerbside = lg.sources[G.sectionLanes[G.sections[sec].lanes.first + k + 1].index].lane;
            const LaneLine& line = G.laneLines[G.sections[sec].lines.first + k];
            const bool mayCross = line.crossFromOffside && line.crossFromKerbside;
            const LineAt got = lineAt(s.paint, offside, true, station);
            ++tableLines;
            tableSolid += mayCross ? 0 : 1;
            if (got.kind != (mayCross ? PaintedLineKind::Dashed : PaintedLineKind::Solid) || got.other != kerbside)
                std::printf("    section %zu (%s, %.1f): the table's line between %s and %s %s; painted %s\n", sec,
                            s.r->graph.edges[static_cast<std::size_t>(ss.edge)].id.c_str(), station, s.laneId(offside).c_str(),
                            s.laneId(kerbside).c_str(), mayCross ? "may be crossed" : "may not be crossed", kindName(got.kind));
            CHECK(got.kind == (mayCross ? PaintedLineKind::Dashed : PaintedLineKind::Solid));
            CHECK(got.other == kerbside);
        }
    }
    for (const PaintedLine& l : s.paint.lines) {
        if (l.kind != PaintedLineKind::Dashed && l.kind != PaintedLineKind::Solid) continue;
        ++paintedBetween;
        const Lane& lane = s.r->lanes.lanes[static_cast<std::size_t>(l.lane)];
        const double roadLength = s.r->graph.edges[static_cast<std::size_t>(lane.road())].s.back();
        const double station = 0.5 * (l.from + l.to), u = lane.dir > 0 ? station : roadLength - station;
        bool stated = false;
        for (std::size_t sec = 0; sec < G.sections.size() && !stated; ++sec) {
            const SectionSource& ss = lg.sectionSources[sec];
            if (ss.edge != lane.road() || ss.dir != lane.dir || u < ss.from || u > ss.to) continue;
            for (uint32_t k = 0; k + 1 < G.sections[sec].lanes.count; ++k) {
                if (lg.sources[G.sectionLanes[G.sections[sec].lanes.first + k].index].lane != l.lane ||
                    lg.sources[G.sectionLanes[G.sections[sec].lanes.first + k + 1].index].lane != l.other)
                    continue;
                const LaneLine& line = G.laneLines[G.sections[sec].lines.first + k];
                stated = (line.crossFromOffside && line.crossFromKerbside) == (l.kind == PaintedLineKind::Dashed);
            }
        }
        if (!stated)
            std::printf("    painted %s between %s and %s from %.1f to %.1f: no section of the lane table states it\n", kindName(l.kind),
                        s.laneId(l.lane).c_str(), s.laneId(l.other).c_str(), l.from, l.to);
        CHECK(stated);
    }
    std::printf("    the lane table: %d lines between lanes, %d of them solid; the paint: %d runs between lanes\n", tableLines, tableSolid,
                paintedBetween);
    // the scene's solid lines, counted by hand: before a stop line two through lanes run to. A: main from the west (f0|f1)
    // and from the east (b0|b1), cross from the south and the north; B: main from the west and the east, the stem: 7.
    CHECK(tableSolid == 7);
    CHECK(s.paint.problems.empty());
}

// (b) STOP BARS FOLLOW THE STATED CONTROL AND SIGNS, never the roads' classes (every road here is of one class, which the
// old rule painted on every arm): the four-way A under each control, arm by arm.
TEST_CASE(road_paint_stop_bars_are_painted_where_the_control_states_a_stop) {
    struct Case {
        const char* name;
        JunctionControl A;
        bool west, east, south, north;   // a bar on main arriving from the west / the east, cross from the south / the north
    };
    const std::vector<ArmSigns> allStop = {{"main", true, {"stop"}}, {"main", false, {"stop"}}, {"cross", true, {"stop"}}, {"cross", false, {"stop"}}};
    const std::vector<ArmSigns> crossStops = {{"cross", true, {"stop"}}, {"cross", false, {"stop"}}};
    const std::vector<ArmSigns> crossGivesWay = {{"cross", true, {"give_way"}}, {"cross", false, {"give_way"}}};
    const std::vector<Case> cases = {
        {"an all-way stop: every arm", {Vec2(0, 0), "all_way_stop", allStop}, true, true, true, true},
        {"a two-way stop: the minor road's arms only", {Vec2(0, 0), "stop", crossStops}, false, false, true, true},
        {"traffic lights: every arm", {Vec2(0, 0), "signal", {}}, true, true, true, true},
        {"no control: none", {Vec2(0, 0), "unmarked", {}}, false, false, false, false},
        {"give-way signs on the minor road: none (the paint has no give-way marking)", {Vec2(0, 0), "give_way", crossGivesWay}, false, false, false, false},
    };
    for (const Case& c : cases) {
        // the T keeps one stop sign on its stem throughout: a bar there, none on main's two arms at it
        const PaintScene s = paintScene({c.A, {Vec2(150, 0), "stop", {{"stem", true, {"stop"}}}}});
        CHECK(s.A >= 0 && s.B >= 0);
        if (s.A < 0 || s.B < 0) return;
        struct WantBar {
            int junction, arm;
            bool painted;
            double station;
            std::vector<std::string> lanes;   // the arriving lanes it lies across
            const char* what;
        };
        const std::vector<WantBar> wants = {
            {s.A, s.arm(s.A, "main", true), c.west, 287.8, {"main.P", "main.f0", "main.f1"}, "A, main from the west"},
            {s.A, s.arm(s.A, "main", false), c.east, 312.2, {"main.b0", "main.b1"}, "A, main from the east"},
            {s.A, s.arm(s.A, "cross", true), c.south, 185.8, {"cross.f0", "cross.f1"}, "A, cross from the south"},
            {s.A, s.arm(s.A, "cross", false), c.north, 214.2, {"cross.b0", "cross.b1"}, "A, cross from the north"},
            {s.B, s.arm(s.B, "main", true), false, 437.8, {}, "B, main from the west"},
            {s.B, s.arm(s.B, "main", false), false, 462.2, {}, "B, main from the east"},
            {s.B, s.arm(s.B, "stem", true), true, 185.8, {"stem.f0", "stem.f1"}, "B, the stem"},
        };
        std::size_t expected = 0;
        for (const WantBar& w : wants) {
            CHECK(w.arm >= 0);
            const PaintedStopBar* got = nullptr;
            for (const PaintedStopBar& b : s.paint.stopBars)
                if (b.junction == w.junction && b.arm == w.arm) got = &b;
            if ((got != nullptr) != w.painted) std::printf("    %s; %s: %s, expected %s\n", c.name, w.what, got ? "a bar" : "no bar", w.painted ? "a bar" : "none");
            CHECK((got != nullptr) == w.painted);
            if (!got || !w.painted) continue;
            ++expected;
            CHECK(std::fabs(got->station - w.station) < kPaintStation);
            std::set<std::string> across, wanted(w.lanes.begin(), w.lanes.end());
            for (int li : got->lanes) across.insert(s.laneId(li));
            CHECK(across == wanted);
            CHECK(got->marking == "stop_bar");
        }
        CHECK(s.paint.stopBars.size() == expected);   // and nowhere else
    }
}

// (a) LANE ARROWS ARE THE STATED TURNS: every painted arrow shows the turns the model states from its lane at that
// junction, and every arriving lane with stated turns has its arrows. Written lane by lane from the turn table
// (assets/data/lane_turns.json) read by hand on the scene:
//   - with a way straight on: every through lane goes straight; the near turn from the one lane nearest the kerb; the far
//     turn from the one lane nearest the centre line (at A from the west that is the pocket P, which only turns);
//   - the stem of the T (no way straight on): its two lanes share the two turns, the kerb lane the near turn.
// At the T, main heading +x has the stem on its kerb side (toward -y): a near turn; heading -x it lies across the
// oncoming lanes: a far turn.
TEST_CASE(road_paint_arrows_show_the_turns_the_model_states_lane_by_lane) {
    const PaintScene& s = signalScene();
    CHECK(s.A >= 0 && s.B >= 0);
    if (s.A < 0 || s.B < 0) return;
    struct Want {
        int junction;
        const char* lane;
        bool straight, nearTurn, farTurn;
        std::vector<double> tails;   // each arrow's tail: 12 m and 35 m before its arm's stop line, the way the lane runs
    };
    const std::vector<Want> wants = {
        // A from the west (stop line 287.8)
        {s.A, "main.P", false, false, true, {275.8, 252.8}},
        {s.A, "main.f0", true, false, false, {275.8, 252.8}},
        {s.A, "main.f1", true, true, false, {275.8, 252.8}},
        // A from the east (stop line 312.2; the lanes run toward lower stations)
        {s.A, "main.b0", true, false, true, {324.2, 347.2}},
        {s.A, "main.b1", true, true, false, {324.2, 347.2}},
        // A from the south (185.8) and the north (214.2)
        {s.A, "cross.f0", true, false, true, {173.8, 150.8}},
        {s.A, "cross.f1", true, true, false, {173.8, 150.8}},
        {s.A, "cross.b0", true, false, true, {226.2, 249.2}},
        {s.A, "cross.b1", true, true, false, {226.2, 249.2}},
        // B from the west (437.8): the stem is a near turn; from the east (462.2): a far turn
        {s.B, "main.f0", true, false, false, {425.8, 402.8}},
        {s.B, "main.f1", true, true, false, {425.8, 402.8}},
        {s.B, "main.b0", true, false, true, {474.2, 497.2}},
        {s.B, "main.b1", true, false, false, {474.2, 497.2}},
        // B's stem (185.8): no way straight on
        {s.B, "stem.f0", false, false, true, {173.8, 150.8}},
        {s.B, "stem.f1", false, true, false, {173.8, 150.8}},
    };
    std::size_t expected = 0;
    for (const Want& w : wants) {
        const int li = s.lane(w.lane);
        CHECK(li >= 0);
        std::vector<double> tails;
        for (const PaintedArrow& a : s.paint.arrows) {
            if (a.junction != w.junction || a.lane != li) continue;
            tails.push_back(a.station);
            if (a.straight != w.straight || a.nearTurn != w.nearTurn || a.farTurn != w.farTurn)
                std::printf("    %s at junction %d: painted%s%s%s, expected%s%s%s\n", w.lane, w.junction, a.straight ? " straight" : "",
                            a.nearTurn ? " near" : "", a.farTurn ? " far" : "", w.straight ? " straight" : "", w.nearTurn ? " near" : "",
                            w.farTurn ? " far" : "");
            CHECK(a.straight == w.straight);
            CHECK(a.nearTurn == w.nearTurn);
            CHECK(a.farTurn == w.farTurn);
            // it lies in its lane and points the way the lane runs
            const Lane& lane = s.r->lanes.lanes[static_cast<std::size_t>(li)];
            CHECK(project(lane.xy, lane.s, a.at).distance < kPaintStation);
        }
        if (tails.size() != w.tails.size()) std::printf("    %s at junction %d: %zu arrows, expected %zu\n", w.lane, w.junction, tails.size(), w.tails.size());
        CHECK(tails.size() == w.tails.size());
        for (std::size_t k = 0; k < tails.size() && k < w.tails.size(); ++k) CHECK(std::fabs(tails[k] - w.tails[k]) < kPaintStation);
        expected += w.tails.size();
    }
    CHECK(s.paint.arrows.size() == expected);   // 15 lanes, two arrows each: no arrow anywhere else
    // which way a glyph's branches point: main.f1 heads +x at A, its kerb toward -y
    for (const PaintedArrow& a : s.paint.arrows)
        if (a.lane == s.lane("main.f1") && a.junction == s.A) {
            CHECK(std::fabs(a.heading.x - 1.0) < kPaintStation && std::fabs(a.heading.y) < kPaintStation);
            CHECK(std::fabs(a.at.y - (-7.25)) < kPaintStation);   // the gap's 2, f0's 3.5, half of its own 3.5
        }
    CHECK(kerbPerpSign() < 0);   // the kerb of a lane lies toward -perp(its way): toward -y heading +x

    // ...and against the model, everywhere: an arrow's turns are exactly the kinds of turn stated from its lane there
    const RoadModel m = builtRoadModel(*s.r);
    for (const PaintedArrow& a : s.paint.arrows) {
        bool straight = false, nearTurn = false, farTurn = false;
        for (const ModelTurn& t : m.turns) {
            if (static_cast<int>(t.junction.index) != a.junction || static_cast<int>(t.from.index) != a.lane) continue;
            straight = straight || t.turn == Turn::Straight;
            nearTurn = nearTurn || t.turn == Turn::NearTurn;
            farTurn = farTurn || t.turn == Turn::FarTurn;
        }
        CHECK(a.straight == straight && a.nearTurn == nearTurn && a.farTurn == farTurn);
    }
    // every lane the model states a turn from, at these two junctions, has its arrows
    std::set<std::pair<int, int>> withArrows, withTurns;
    for (const PaintedArrow& a : s.paint.arrows) withArrows.insert(std::make_pair(a.junction, a.lane));
    for (const ModelTurn& t : m.turns) withTurns.insert(std::make_pair(static_cast<int>(t.junction.index), static_cast<int>(t.from.index)));
    CHECK(withArrows == withTurns);
    CHECK(withTurns.size() == 15);
}

// WHETHER A ROAD'S ARMS AT A JUNCTION ARE MARKED IS ITS CLASS'S STATED DATUM (issue #292; road_graph_spec.h
// RoadClassSpec::junctionMarkings, "junction_markings" in the file's `classes`), not its class's name or rank. The old
// rule marked every class but one named "freeway", a track, or one ranked below 1. Two scenes where the two disagree:
//   - the scene's class stated rank 0 and nothing else: the old rule painted nothing; its arms are marked as before;
//   - the scene's class stated "junction_markings": false (rank 1, not named freeway): the old rule painted every arm;
//     nothing is painted at them -- no crosswalk leg's arrows or bars -- and the lines between lanes are still drawn.
TEST_CASE(road_paint_a_class_states_whether_its_junction_arms_are_marked) {
    const std::vector<JunctionControl> lights = {{Vec2(0, 0), "signal", {}}, {Vec2(150, 0), "stop", {{"stem", true, {"stop"}}}}};
    const PaintScene& usual = signalScene();
    // as the scene is: five bars (A's four arms under lights, B's stem), 30 arrows (15 lanes, two each)
    CHECK(usual.paint.stopBars.size() == 5u && usual.paint.arrows.size() == 30u);
    for (const JunctionArea& A : junctionAreas(usual.r->graph))
        for (const ArmStop& arm : A.arms) CHECK(arm.painted);

    const PaintScene lowRank = paintScene(lights, nlohmann::json{{"rank", 0}});
    CHECK(lowRank.paint.stopBars.size() == 5u && lowRank.paint.arrows.size() == 30u);
    for (const JunctionArea& A : junctionAreas(lowRank.r->graph))
        for (const ArmStop& arm : A.arms) CHECK(arm.painted);

    const PaintScene unmarked = paintScene(lights, nlohmann::json{{"junction_markings", false}});
    CHECK(unmarked.paint.stopBars.empty() && unmarked.paint.arrows.empty());
    std::size_t arms = 0;
    for (const JunctionArea& A : junctionAreas(unmarked.r->graph))
        for (const ArmStop& arm : A.arms) {
            ++arms;
            CHECK(!arm.painted);
            CHECK(arm.stopLine != 0);   // the arm keeps its stop line: the lane level still ends its lanes there
        }
    CHECK(arms == 7u);
    CHECK(!unmarked.paint.lines.empty());

    // the table's rows: streets marked; a freeway, a ramp, a track, a footway and a divider not; a class a file adds
    // starts marked, and the datum is written only where it differs from the row it would be read with
    const std::map<std::string, RoadClassSpec>& table = roadClasses();
    for (const char* marked : {"local", "collector", "arterial", "boulevard", "rural", "mountain"}) CHECK(table.at(marked).junctionMarkings);
    for (const char* not_ : {"freeway", "ramp", "divider", "rail", "light_rail", "footway"}) CHECK(!table.at(not_).junctionMarkings);
    CHECK(!classJson(table.at("freeway")).contains("junction_markings"));
    CHECK(!classJson(table.at("local")).contains("junction_markings"));
    RoadClassSpec quiet = table.at("local");
    quiet.junctionMarkings = false;
    CHECK(classJson(quiet).value("junction_markings", true) == false);
}

// THE ONE RULE FOR A LINE BETWEEN LANES, by itself (lane_turns.h laneLineBetween): the kinds that hold their lane come
// from the table's "lane_lines", not from the code.
TEST_CASE(road_paint_the_line_rule_reads_its_kinds_from_the_table) {
    LaneTurnRules rules;
    rules.holdBeforeStopLine = {"through"};
    LaneBesideLine through, pocket, ends;
    through.kind = "through";
    through.runsToStopLine = true;
    through.stopLine = 100.0;
    pocket = through;
    pocket.kind = "turn";
    ends = through;
    ends.runsToStopLine = false;
    auto solid = [&](const LaneBesideLine& a, const LaneBesideLine& b, double from, double to) {
        const LaneLine l = laneLineBetween(rules, 20.0, a, b, from, to);
        CHECK(l.crossFromOffside == l.crossFromKerbside);
        return !l.crossFromOffside;
    };
    CHECK(solid(through, through, 80.0, 100.0));      // the whole approach
    CHECK(solid(through, through, 90.0, 95.0));
    CHECK(!solid(through, through, 79.0, 100.0));     // a stretch that begins before the approach
    CHECK(!solid(through, through, 60.0, 70.0));
    CHECK(!solid(through, pocket, 90.0, 95.0));       // a pocket is not a kind that holds
    CHECK(!solid(pocket, through, 90.0, 95.0));
    CHECK(!solid(through, ends, 90.0, 95.0));         // the lane beside does not run to the stop line
    rules.holdBeforeStopLine = {"through", "turn"};   // the table names the pocket's kind too: now solid
    CHECK(solid(through, pocket, 90.0, 95.0));
    rules.holdBeforeStopLine.clear();                 // the table names none: nothing is solid
    CHECK(!solid(through, through, 90.0, 95.0));
    // the shipped table names the kinds
    CHECK(!laneTurnRules().holdBeforeStopLine.empty());
}
