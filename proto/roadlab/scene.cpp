#include "scene.h"

#include <set>

#include "sim.h"
#include <cstdio>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>

namespace roadlab {

using nlohmann::json;

void finalizeScene(Scene& scene, bool withTerrain, bool withProps) {
    scene.net.build();
    scene.lint = scene.net.validate();
    generateRoadPaint(scene.net, scene.paint.roads);
    scene.paint.junctions.assign(size_t(scene.net.junctionCount()), {});

    scene.mesh = Mesh{};
    tessellateNetwork(scene.net, scene.mesh, scene.tess, true);

    if (withTerrain && scene.terrain.enabled) {
        Vec2 lo, hi;
        scene.net.planBounds(lo, hi);
        lo.x -= 60;
        lo.y -= 60;
        hi.x += 60;
        hi.y += 60;
        double span = std::max(hi.x - lo.x, hi.y - lo.y);
        double cell = clampd(span / 320.0, 1.5, 12.0);
        tessellateTerrain(scene.net, scene.terrain, lo, hi, cell, scene.mesh);
    }
    if (withProps) {
        generateProps(scene.net, scene.props, scene.propRules);
        tessellateProps(scene.props, scene.mesh);
    }
    scene.built = true;
}

void appendAgents(Scene& scene, const Simulation& sim, Mesh& out) {
    out = scene.mesh;
    tessellateAgents(sim, out);
}

// --- JSON -----------------------------------------------------------------

namespace {

Vec2 vec2Of(const json& j) {
    if (j.is_array() && j.size() >= 2) return {j[0].get<double>(), j[1].get<double>()};
    if (j.is_object()) return {j.value("x", 0.0), j.value("z", j.value("y", 0.0))};
    return {0, 0};
}

JunctionControl controlOf(const std::string& s) {
    if (s == "signalized" || s == "signalised") return JunctionControl::Signalized;
    if (s == "stop" || s == "priority-stop") return JunctionControl::PriorityStop;
    if (s == "all-way-stop") return JunctionControl::AllWayStop;
    if (s == "yield") return JunctionControl::Yield;
    if (s == "roundabout") return JunctionControl::RoundaboutEntry;
    if (s == "merge") return JunctionControl::Merge;
    return JunctionControl::Uncontrolled;
}

const char* controlJsonName(JunctionControl c) {
    switch (c) {
        case JunctionControl::Signalized: return "signalized";
        case JunctionControl::PriorityStop: return "priority-stop";
        case JunctionControl::AllWayStop: return "all-way-stop";
        case JunctionControl::Yield: return "yield";
        case JunctionControl::RoundaboutEntry: return "roundabout";
        case JunctionControl::Merge: return "merge";
        case JunctionControl::Uncontrolled: return "uncontrolled";
    }
    return "uncontrolled";
}

}  // namespace

bool loadSceneJson(const std::string& path, Scene& out, std::string& error) {
    std::ifstream in(path);
    if (!in) {
        error = "cannot open " + path;
        return false;
    }
    json j;
    try {
        in >> j;
    } catch (const std::exception& e) {
        error = std::string("parse error: ") + e.what();
        return false;
    }

    out.name = j.value("name", path);
    if (j.contains("terrain")) {
        const json& t = j["terrain"];
        out.terrain.enabled = t.value("enabled", true);
        out.terrain.base = t.value("base", 0.0);
        out.terrain.amplitude = t.value("amplitude", 6.0);
        out.terrain.frequency = t.value("frequency", 1.0 / 260.0);
        out.terrain.seed = uint32_t(t.value("seed", 5));
        out.terrain.cutBatter = t.value("cutBatter", 1.0);
        out.terrain.fillBatter = t.value("fillBatter", 0.5);
        out.terrain.batterReach = t.value("batterReach", 45.0);
    }
    if (j.contains("shade")) {
        const json& s = j["shade"];
        out.shade.wear = s.value("wear", 0.30);
        out.shade.grime = s.value("grime", 0.35);
        out.shade.seed = uint32_t(s.value("seed", 7));
        out.shade.markings = s.value("markings", true);
        out.shade.decals = s.value("decals", true);
    }

    // Roads first: their array index is the id every later section refers to.
    std::vector<int> roadIds;
    for (const json& r : j.value("roads", json::array())) {
        RoadDesc d;
        d.name = r.value("name", "road");
        d.preset = r.value("preset", "street2");
        for (const json& p : r.value("points", json::array())) d.points.push_back(vec2Of(p));
        if (d.points.size() < 2) continue;
        d.cornerRadius = r.value("cornerRadius", 45.0);
        d.spirals = r.value("spirals", true);
        d.baseHeight = r.value("height", 0.0);
        d.designSpeed = r.value("designSpeed", -1.0);
        d.crossfall = r.value("crossfall", 0.02);
        d.autoSuperelevation = r.value("superelevation", true);
        for (const json& e : r.value("elevation", json::array())) d.elevationKnots.push_back(vec2Of(e));
        roadIds.push_back(buildRoad(out.net, d));
    }

    auto roadAt = [&](int index) { return (index >= 0 && size_t(index) < roadIds.size())
                                              ? roadIds[size_t(index)]
                                              : -1; };

    // Cross-section edits, then vertical, before anything splits a road.
    int index = 0;
    for (const json& r : j.value("roads", json::array())) {
        int id = roadAt(index++);
        if (id < 0) continue;
        if (r.value("twltl", false)) applyTwltl(out.net, id);
        for (const json& lc : r.value("laneChanges", json::array())) {
            applyLaneChange(out.net, id, lc.value("s", 0.0), lc.value("side", -1),
                            lc.value("delta", -1), lc.value("run", -1.0));
        }
        for (const json& tb : r.value("turnBays", json::array())) {
            applyTurnBay(out.net, id, tb.value("s", 0.0), tb.value("atEnd", true),
                         tb.value("side", -1), tb.value("length", 60.0),
                         tb.value("width", 3.3));
        }
        if (r.contains("overpass")) {
            const json& o = r["overpass"];
            makeOverpass(out.net, id, o.value("s0", 0.0), o.value("s1", 0.0),
                         o.value("rise", 7.0), o.value("approach", 90.0),
                         o.value("pierSpacing", 30.0));
        }
        if (r.contains("tunnel")) {
            const json& o = r["tunnel"];
            makeTunnel(out.net, id, o.value("s0", 0.0), o.value("s1", 0.0), o.value("drop", 9.0),
                       o.value("approach", 90.0));
        }
    }

    for (const json& rb : j.value("roundabouts", json::array())) {
        RoundaboutDesc d;
        d.name = rb.value("name", "roundabout");
        d.center = vec2Of(rb.value("center", json::array({0, 0})));
        d.radius = rb.value("radius", 14.0);
        d.ringPreset = rb.value("preset", "roundabout2");
        d.height = rb.value("height", 0.0);
        for (const json& a : rb.value("arms", json::array())) {
            int id = roadAt(a.at(0).get<int>());
            if (id >= 0) d.arms.push_back({id, a.at(1).get<bool>()});
        }
        buildRoundabout(out.net, d);
    }

    for (const json& jn : j.value("junctions", json::array())) {
        std::vector<std::pair<int, bool>> arms;
        for (const json& a : jn.value("arms", json::array())) {
            int id = roadAt(a.at(0).get<int>());
            if (id >= 0) arms.push_back({id, a.at(1).get<bool>()});
        }
        if (arms.size() < 2) continue;
        buildIntersection(out.net, jn.value("name", "junction"), arms,
                          controlOf(jn.value("control", "uncontrolled")),
                          jn.value("cornerRadius", 7.0));
    }

    // Ramps last: they split the mainline, which would invalidate the array
    // indices everything above uses.
    for (int pass = 0; pass < 2; ++pass) {
        const char* key = pass == 0 ? "onRamps" : "offRamps";
        for (const json& rp : j.value(key, json::array())) {
            int main = roadAt(rp.value("main", -1));
            if (main < 0) continue;
            RampDesc d;
            d.name = rp.value("name", pass == 0 ? "on-ramp" : "off-ramp");
            d.preset = rp.value("preset", "ramp1");
            d.outerPoint = vec2Of(rp.value("from", json::array({0, 0})));
            d.outerHeading = rp.value("heading", 0.0) * kDeg2Rad;
            d.outerHeight = rp.value("fromHeight", 0.0);
            d.auxLength = rp.value("auxLength", 180.0);
            d.designSpeed = rp.value("designSpeed", 60.0);
            d.side = rp.value("side", -1);
            if (pass == 0)
                buildOnRamp(out.net, main, rp.value("s", 0.0), d);
            else
                buildOffRamp(out.net, main, rp.value("s", 0.0), d);
        }
    }
    return true;
}

bool saveSceneJson(const Scene& scene, const std::string& path, std::string& error) {
    json j;
    j["name"] = scene.name;
    j["_note"] = "roadlab scene dump: geometry is re-emitted as sampled polylines, so a "
                 "round trip loses the original clothoid parameters.";
    json roads = json::array();
    for (const Road& r : scene.net.roads()) {
        if (r.kind == RoadKind::Connector) continue;
        json rr;
        rr["name"] = r.name;
        rr["designSpeed"] = r.designSpeed;
        rr["lanes"] = r.xs.sectionAt(r.begin()).laneCount();
        rr["length"] = r.activeLength();
        json pts = json::array();
        int n = std::max(2, int(r.activeLength() / 20.0));
        for (int i = 0; i <= n; ++i) {
            Vec2 p = r.spine.toPlan(lerp(r.begin(), r.end(), double(i) / n), 0);
            pts.push_back(json::array({p.x, p.y}));
        }
        rr["points"] = pts;
        roads.push_back(rr);
    }
    j["roads"] = roads;
    json js = json::array();
    for (const Junction& jn : scene.net.junctions()) {
        json o;
        o["name"] = jn.name;
        o["control"] = controlJsonName(jn.control);
        o["arms"] = int(jn.arms.size());
        o["connections"] = int(jn.connections.size());
        o["conflicts"] = int(jn.conflicts.size());
        o["phases"] = int(jn.phases.size());
        js.push_back(o);
    }
    j["junctions"] = js;
    std::ofstream out(path);
    if (!out) {
        error = "cannot write " + path;
        return false;
    }
    out << j.dump(2) << "\n";
    return true;
}

// --- demos ----------------------------------------------------------------

std::vector<std::string> demoNames() {
    return {"lanes", "roundabout", "interchange", "urban", "tiers", "grades", "showcase"};
}

namespace {

// A straight arterial that loses two lanes, gains a two-way left-turn lane and
// ends at a signalised T. Everything visible is a consequence of the width
// functions: the tapers, the merge arrows, the LANE ENDS signs.
void demoLanes(Scene& sc) {
    sc.name = "lanes";
    sc.terrain.amplitude = 2.0;

    RoadDesc main;
    main.name = "arterial";
    main.preset = "arterial6_median";
    main.points = {{-300, 0}, {60, 0}, {140, 4}};
    main.cornerRadius = 220;
    main.designSpeed = 70;
    int m = buildRoad(sc.net, main);
    // Six lanes down to four: one strip's width function reaching zero.
    applyLaneChange(sc.net, m, 150.0, -1, -1);

    RoadDesc side;
    side.name = "cross";
    side.preset = "street2_park";
    side.points = {{150, 200}, {143, 60}, {141, 4}};
    side.cornerRadius = 90;
    int c = buildRoad(sc.net, side);

    RoadDesc east;
    east.name = "arterial-east";
    east.preset = "boulevard_twltl";
    east.points = {{142, 4}, {300, 18}, {470, 42}};
    east.cornerRadius = 220;
    east.designSpeed = 60;
    int e = buildRoad(sc.net, east);

    applyTurnBay(sc.net, m, sc.net.road(m).spineLength(), true, +1, 90.0);
    buildIntersection(sc.net, "signal", {{m, true}, {c, true}, {e, false}},
                      JunctionControl::Signalized, 8.0);
}

void demoRoundabout(Scene& sc) {
    sc.name = "roundabout";
    sc.terrain.amplitude = 3.0;
    Vec2 centre{0, 0};
    double R = 20.0;
    double approach = 150.0;
    // Where an approach STOPS matters more than it looks. The junction bridges
    // whatever gap is left between the arm's end and the ring, so a generous
    // stand-off does not read as caution — it reads as a tarmac plain with an
    // entry somewhere in the middle of it. This was 26 m past the reference
    // line: the ring's outer edge is about 9 m out, so every "entry" was a 17 m
    // apron. Derive it from the ring instead, and leave only the few metres the
    // kerb return actually needs.
    LaneSection ringSec = roadPreset("roundabout2").section;
    double ringOuter = 0.5 * ringSec.totalWidthAt(0.0);
    double armEnd = R + ringOuter + 4.0;
    std::vector<std::pair<int, bool>> arms;
    const char* names[4] = {"north", "east", "south", "west"};
    for (int i = 0; i < 4; ++i) {
        double a = kPi * 0.5 * i + 0.12;
        Vec2 outer{centre.x + std::cos(a) * (R + approach), centre.y + std::sin(a) * (R + approach)};
        Vec2 inner{centre.x + std::cos(a) * armEnd, centre.y + std::sin(a) * armEnd};
        RoadDesc d;
        d.name = names[i];
        d.preset = "collector4";
        d.points = {outer, inner};
        d.designSpeed = 50;
        int id = buildRoad(sc.net, d);
        arms.push_back({id, true});
    }
    RoundaboutDesc rd;
    rd.center = centre;
    rd.radius = R;
    rd.arms = arms;
    buildRoundabout(sc.net, rd);
}

void demoInterchange(Scene& sc) {
    sc.name = "interchange";
    sc.terrain.amplitude = 5.0;
    sc.terrain.frequency = 1.0 / 320.0;

    RoadDesc fw;
    fw.name = "freeway";
    fw.preset = "freeway3";
    fw.points = {{-700, -40}, {-150, 0}, {350, 20}, {800, -30}};
    fw.cornerRadius = 700;
    fw.designSpeed = 110;
    int f = buildRoad(sc.net, fw);

    // A surface road crossing over the freeway. Two roads at different heights
    // in the same place: no junction, and the clearance lint is what says so.
    RoadDesc over;
    over.name = "overpass";
    over.preset = "collector4";
    over.points = {{60, -260}, {80, 0}, {100, 260}};
    over.cornerRadius = 300;
    over.designSpeed = 60;
    int o = buildRoad(sc.net, over);
    double sMid = sc.net.road(o).spineLength() * 0.5;
    makeOverpass(sc.net, o, sMid - 34, sMid + 34, 9.0, 110.0, 28.0);

    RampDesc on;
    on.name = "on-ramp";
    on.preset = "ramp1";
    on.outerPoint = {150, -150};
    on.outerHeading = 128 * kDeg2Rad;
    on.outerHeight = 1.5;
    on.auxLength = 200.0;
    int mainTail = -1;
    buildOnRamp(sc.net, f, 720.0, on, &mainTail);

    RampDesc off;
    off.name = "off-ramp";
    off.preset = "ramp1";
    off.outerPoint = {-262, -235};
    off.outerHeading = -87 * kDeg2Rad;
    off.outerHeight = 1.5;
    off.auxLength = 160.0;
    buildOffRamp(sc.net, f, 430.0, off);
}

void demoUrban(Scene& sc) {
    sc.name = "urban";
    sc.terrain.amplitude = 2.5;
    CityParams cp;
    cp.blocksX = 3;
    cp.blocksZ = 3;
    cp.blockSize = 140;
    cp.freeway = false;
    cp.overpass = false;
    cp.roundabout = false;
    generateCity(sc, cp);
    sc.name = "urban";
}

// Three carriageways in the same plan footprint at three heights. Nothing in the
// model knows this is unusual — it is just three elevation profiles.
void demoTiers(Scene& sc) {
    sc.name = "tiers";
    sc.terrain.amplitude = 1.5;

    RoadDesc ground;
    ground.name = "ground";
    ground.preset = "arterial4";
    ground.points = {{-320, 0}, {0, 0}, {320, 0}};
    ground.cornerRadius = 200;
    buildRoad(sc.net, ground);

    RoadDesc tier1;
    tier1.name = "tier1";
    tier1.preset = "bridge2";
    tier1.points = {{-330, -110}, {-40, -20}, {60, 20}, {330, 110}};
    tier1.cornerRadius = 320;
    tier1.designSpeed = 80;
    int t1 = buildRoad(sc.net, tier1);
    makeOverpass(sc.net, t1, 150, sc.net.road(t1).spineLength() - 150, 8.5, 140.0, 34.0);

    RoadDesc tier2;
    tier2.name = "tier2";
    tier2.preset = "bridge2";
    tier2.points = {{-330, 120}, {-30, 30}, {70, -20}, {330, -120}};
    tier2.cornerRadius = 320;
    tier2.designSpeed = 80;
    int t2 = buildRoad(sc.net, tier2);
    makeOverpass(sc.net, t2, 240, sc.net.road(t2).spineLength() - 240, 17.5, 235.0, 34.0);

    RoadDesc bored;
    bored.name = "tunnel";
    bored.preset = "tunnel2";
    bored.points = {{-30, -320}, {10, 0}, {40, 320}};
    bored.cornerRadius = 320;
    bored.designSpeed = 70;
    int tu = buildRoad(sc.net, bored);
    double L = sc.net.road(tu).spineLength();
    makeTunnel(sc.net, tu, L * 0.34, L * 0.66, 11.0, 165.0);
}

// A skewed junction whose four arms arrive at four different heights, on four
// different grades, each carrying some curvature (and so some bank) into the
// pad. This is the case the pad's height interpolation exists for; with a flat
// blend it produces a visible lip at every approach.
void demoGrades(Scene& sc) {
    sc.name = "grades";
    sc.terrain.amplitude = 3.0;
    sc.terrain.frequency = 1.0 / 220.0;

    struct Arm {
        const char* name;
        double bearingDeg;
        double outerHeight;
        const char* preset;
        double reach;
    };
    // Deliberately not at 90-degree spacing: a skewed junction is the hard case.
    const Arm arms[4] = {
        {"north", 96, 3.2, "arterial4", 170},
        {"east", 8, -1.4, "collector4", 190},
        {"south", -104, -2.6, "arterial4", 175},
        {"west", 168, 1.1, "street2_park", 150},
    };
    std::vector<std::pair<int, bool>> junctionArms;
    for (const Arm& a : arms) {
        double ang = a.bearingDeg * kDeg2Rad;
        Vec2 outer{std::cos(ang) * a.reach, std::sin(ang) * a.reach};
        Vec2 mid{std::cos(ang) * a.reach * 0.5, std::sin(ang) * a.reach * 0.5};
        Vec2 inner{std::cos(ang) * 16.0, std::sin(ang) * 16.0};
        RoadDesc d;
        d.name = a.name;
        d.preset = a.preset;
        // A slight sway so each arm carries curvature and superelevation into the
        // junction rather than arriving dead straight.
        Vec2 nrm = perpLeft(normalize(inner - outer));
        d.points = {outer, mid + nrm * 7.0, inner};
        d.cornerRadius = 400;
        // The whole point: each approach reaches the pad at its own height, on
        // its own grade.
        d.elevationKnots = {{0.0, a.outerHeight},
                            {a.reach * 0.55, a.outerHeight * 0.45},
                            {a.reach, 0.0}};
        int id = buildRoad(sc.net, d);
        junctionArms.push_back({id, true});
    }
    buildIntersection(sc.net, "graded", junctionArms, JunctionControl::PriorityStop, 8.0);
}

void demoShowcase(Scene& sc) {
    sc.name = "showcase";
    sc.terrain.amplitude = 4.0;
    sc.terrain.frequency = 1.0 / 300.0;

    // A country road that becomes a street, loses its shoulders and gains kerbs,
    // parking and footways — a cross-section transition solved, not authored.
    RoadDesc approach;
    approach.name = "north-road";
    approach.preset = "country2";
    approach.points = {{-40, -420}, {10, -240}, {0, -80}};
    approach.cornerRadius = 420;
    approach.designSpeed = 90;
    int nr = buildRoad(sc.net, approach);
    {
        Road& r = sc.net.road(nr);
        ProfileTimeline tl;
        LaneSection country = roadPreset("country2").section;
        LaneSection street = roadPreset("street2_park").section;
        tl.at(0.0, country);
        tl.at(r.spineLength() * 0.45, street, -1.0);
        r.xs = tl.build(r.spineLength(), r.designSpeed);
    }

    RoadDesc high;
    high.name = "high-street";
    high.preset = "street2_park";
    high.points = {{-420, -60}, {-120, -50}, {160, -70}, {430, -40}};
    high.cornerRadius = 140;
    high.designSpeed = 40;
    int hs = buildRoad(sc.net, high);

    RoadDesc south;
    south.name = "south-road";
    south.preset = "collector4";
    south.points = {{10, -40}, {30, 180}, {60, 420}};
    south.cornerRadius = 200;
    south.designSpeed = 50;
    int sr = buildRoad(sc.net, south);

    // Split high-street where the north-south pair actually crosses it, not at a
    // guessed station. A hand-typed 300.0 here put the split at x = -120 while
    // the crossing is at x = 5, so the junction's four arms stood 120 m apart:
    // the pad spanned the gap, its outline doubled back on itself, and ear
    // clipping fell through to a centroid fan. Projecting the crossing point onto
    // the spine cannot drift when either road is re-authored.
    Vec2 crossing = (Vec2{0, -80} + Vec2{10, -40}) * 0.5;
    double sCross = 0, tCross = 0;
    if (!sc.net.road(hs).spine.toST(crossing, sCross, tCross)) sCross = 300.0;
    int hsTail = sc.net.splitRoad(hs, sCross);
    buildIntersection(sc.net, "crossroads",
                      {{hs, true}, {hsTail, false}, {nr, true}, {sr, false}},
                      JunctionControl::Signalized, 8.0);

    // A freeway on embankment crossing the whole scene, with a bridge over the
    // south road and an on-ramp off the high street.
    RoadDesc fw;
    fw.name = "freeway";
    fw.preset = "freeway3";
    fw.points = {{-520, 300}, {-100, 250}, {320, 300}, {620, 250}};
    fw.cornerRadius = 600;
    fw.designSpeed = 110;
    int f = buildRoad(sc.net, fw);
    makeOverpass(sc.net, f, 520, 610, 8.0, 190.0, 30.0);

    RampDesc on;
    on.name = "on-ramp";
    on.outerPoint = {330, 140};
    on.outerHeading = 118 * kDeg2Rad;
    on.outerHeight = 1.0;
    on.auxLength = 190.0;
    buildOnRamp(sc.net, f, 760.0, on);
}

}  // namespace

bool buildDemo(const std::string& name, Scene& out) {
    if (name == "lanes") demoLanes(out);
    else if (name == "roundabout") demoRoundabout(out);
    else if (name == "interchange") demoInterchange(out);
    else if (name == "urban") demoUrban(out);
    else if (name == "tiers") demoTiers(out);
    else if (name == "grades") demoGrades(out);
    else if (name == "showcase") demoShowcase(out);
    else return false;
    return true;
}

// --- generation -----------------------------------------------------------

void generateCity(Scene& out, const CityParams& params) {
    out.name = "city";
    Rng rng(params.seed);
    const int nx = std::max(2, params.blocksX + 1);
    const int nz = std::max(2, params.blocksZ + 1);
    const double B = params.blockSize;
    const double originX = -0.5 * B * params.blocksX;
    const double originZ = -0.5 * B * params.blocksZ;

    // Grid nodes, jittered so the generated network is not a perfect lattice —
    // curvature is where a road model earns or loses its keep.
    std::vector<std::vector<Vec2>> node(size_t(nx), std::vector<Vec2>(size_t(nz), Vec2{}));
    for (int i = 0; i < nx; ++i) {
        for (int k = 0; k < nz; ++k) {
            bool edge = (i == 0 || k == 0 || i == nx - 1 || k == nz - 1);
            double jx = edge ? 0.0 : rng.range(-params.jitter, params.jitter);
            double jz = edge ? 0.0 : rng.range(-params.jitter, params.jitter);
            node[size_t(i)][size_t(k)] = {originX + B * i + jx, originZ + B * k + jz};
        }
    }

    // Roundabout node, if asked for: it replaces an interior crossing.
    int rbI = params.roundabout && nx > 2 ? 1 + int(rng.nextU64() % uint64_t(std::max(1, nx - 2)))
                                          : -1;
    int rbK = params.roundabout && nz > 2 ? 1 + int(rng.nextU64() % uint64_t(std::max(1, nz - 2)))
                                          : -1;
    if (!params.roundabout) rbI = rbK = -1;

    auto isArterial = [&](int line) {
        return params.arterialEvery > 0 && (line % int(params.arterialEvery)) == 0;
    };

    // One road per grid EDGE rather than one per grid line, so every crossing is
    // a junction between road ends and nothing has to be split.
    struct Edge {
        int road;
        int i0, k0, i1, k1;
    };
    std::vector<Edge> edges;
    auto addEdge = [&](int i0, int k0, int i1, int k1, bool arterial, bool horizontal) {
        RoadDesc d;
        char buf[64];
        std::snprintf(buf, sizeof buf, "%s-%d-%d", horizontal ? "ew" : "ns", i0, k0);
        d.name = buf;
        if (arterial) {
            d.preset = "arterial4";
            d.designSpeed = 60;
        } else {
            d.preset = params.parking && rng.chance(0.6) ? "street2_park" : "street2";
            d.designSpeed = 40;
        }
        Vec2 a = node[size_t(i0)][size_t(k0)];
        Vec2 b = node[size_t(i1)][size_t(k1)];
        Vec2 mid = (a + b) * 0.5;
        Vec2 nrm = perpLeft(normalize(b - a));
        mid = mid + nrm * rng.range(-B * 0.05, B * 0.05);
        d.points = {a, mid, b};
        d.cornerRadius = arterial ? 160.0 : 90.0;
        d.baseHeight = 0.0;
        edges.push_back({buildRoad(out.net, d), i0, k0, i1, k1});
    };

    for (int k = 0; k < nz; ++k)
        for (int i = 0; i + 1 < nx; ++i) addEdge(i, k, i + 1, k, isArterial(k), true);
    for (int i = 0; i < nx; ++i)
        for (int k = 0; k + 1 < nz; ++k) addEdge(i, k, i, k + 1, isArterial(i), false);

    // A junction per node, from whichever edges touch it.
    for (int i = 0; i < nx; ++i) {
        for (int k = 0; k < nz; ++k) {
            std::vector<std::pair<int, bool>> arms;
            for (const Edge& e : edges) {
                if (e.i0 == i && e.k0 == k) arms.push_back({e.road, false});
                if (e.i1 == i && e.k1 == k) arms.push_back({e.road, true});
            }
            if (arms.size() < 2) continue;
            char buf[64];
            std::snprintf(buf, sizeof buf, "node-%d-%d", i, k);
            if (i == rbI && k == rbK) {
                RoundaboutDesc rd;
                rd.name = buf;
                rd.center = node[size_t(i)][size_t(k)];
                rd.radius = 13.0;
                rd.arms = arms;
                buildRoundabout(out.net, rd);
                continue;
            }
            // Signals where two arterials cross, stop control where a street
            // meets an arterial, uncontrolled between streets. The rule is the
            // same one a traffic engineer would apply, and the signage follows.
            bool arterialCross = isArterial(i) && isArterial(k);
            JunctionControl ctl = JunctionControl::Uncontrolled;
            if (arterialCross && params.signals) ctl = JunctionControl::Signalized;
            else if (isArterial(i) || isArterial(k)) ctl = JunctionControl::PriorityStop;
            else if (arms.size() > 2) ctl = JunctionControl::AllWayStop;
            buildIntersection(out.net, buf, arms, ctl, arms.size() > 3 ? 8.0 : 6.0);
        }
    }

    if (params.freeway) {
        double z = originZ - B * 0.85;
        double x0 = originX - B * 0.6, x1 = originX + B * params.blocksX + B * 0.6;
        RoadDesc fw;
        fw.name = "freeway";
        fw.preset = "freeway3";
        fw.points = {{x0, z - 40}, {(x0 + x1) * 0.5, z}, {x1, z - 30}};
        fw.cornerRadius = 800;
        fw.designSpeed = 110;
        int f = buildRoad(out.net, fw);
        double L = out.net.road(f).spineLength();
        if (params.overpass) makeOverpass(out.net, f, L * 0.42, L * 0.5, 8.5, 210.0, 30.0);

        RampDesc on;
        on.name = "on-ramp";
        on.outerPoint = {originX + B * 0.9, originZ - B * 0.42};
        on.outerHeading = -80 * kDeg2Rad;
        on.outerHeight = 1.0;
        on.auxLength = 190.0;
        buildOnRamp(out.net, f, L * 0.74, on);

        RampDesc off;
        off.name = "off-ramp";
        off.outerPoint = {originX + B * (params.blocksX - 0.5), originZ - B * 0.42};
        off.outerHeading = 82 * kDeg2Rad;
        off.outerHeight = 1.0;
        off.auxLength = 150.0;
        buildOffRamp(out.net, f, L * 0.30, off);
    }
}

// --- the metro generator ----------------------------------------------------

namespace {

// A ramp splits its mainline, and the tail is another window over the SAME
// spine keeping absolute stations — which is what makes several interchanges on
// one carriageway tractable. Keep the pieces in order and the station a ramp was
// planned at still identifies the piece it belongs on.
struct Carriageway {
    std::vector<int> pieces;

    int at(const Network& net, double s) const {
        for (int id : pieces) {
            const Road& r = net.road(id);
            if (s >= r.begin() - 1e-6 && s <= r.end() + 1e-6) return id;
        }
        return pieces.empty() ? -1 : pieces.back();
    }
    void split(int tail) {
        if (tail >= 0) pieces.push_back(tail);
    }
};

// Where on a carriageway a world point sits. Every piece is a window over the
// same spine, so any piece answers for the whole of it.
bool stationNear(const Network& net, const Carriageway& cw, Vec2 p, double& sOut, double& tOut) {
    if (cw.pieces.empty()) return false;
    double s = 0, t = 0;
    if (!net.road(cw.pieces.front()).spine.toST(p, s, t)) return false;
    sOut = s;
    tOut = t;
    return true;
}

}  // namespace

void generateMetro(Scene& out, const MetroParams& p) {
    out.name = "metro";
    Rng rng(p.seed ? p.seed : 1);
    const int nx = std::max(3, p.cols + 1);
    const int nz = std::max(3, p.rows + 1);

    // --- 1. An irregular lattice, warped smoothly ---------------------------
    //
    // Irregular spacing is what makes blocks different sizes; the warp is what
    // makes the streets curve. Doing the warp with noise rather than per-node
    // jitter keeps a street curving as one continuous line instead of wiggling
    // between every pair of junctions.
    std::vector<double> xs(size_t(nx), 0.0), zs(size_t(nz), 0.0);
    double acc = 0;
    for (int i = 0; i < nx; ++i) {
        xs[size_t(i)] = acc;
        acc += rng.range(p.minCell, p.maxCell);
    }
    acc = 0;
    for (int k = 0; k < nz; ++k) {
        zs[size_t(k)] = acc;
        acc += rng.range(p.minCell, p.maxCell);
    }
    const double spanX = xs.back(), spanZ = zs.back();
    for (double& v : xs) v -= spanX * 0.5;
    for (double& v : zs) v -= spanZ * 0.5;

    const double cell = 0.5 * (p.minCell + p.maxCell);
    const double warpAmp = p.warp * cell;
    double noiseOx = rng.range(-500, 500), noiseOz = rng.range(-500, 500);
    auto warped = [&](int i, int k) {
        double x = xs[size_t(i)], z = zs[size_t(k)];
        double u = (x + noiseOx) / (cell * 5.2), v = (z + noiseOz) / (cell * 5.2);
        double dx = fbm(u, v, 3) - 0.5;
        double dz = fbm(u + 31.7, v - 12.3, 3) - 0.5;
        // The rim is pinned so the street network keeps a clean outline for the
        // bypass to be planned around.
        double edgeFade = 1.0;
        if (i == 0 || k == 0 || i == nx - 1 || k == nz - 1) edgeFade = 0.35;
        return Vec2{x + dx * warpAmp * 2.0 * edgeFade, z + dz * warpAmp * 2.0 * edgeFade};
    };
    std::vector<std::vector<Vec2>> node(size_t(nx), std::vector<Vec2>(size_t(nz), Vec2{}));
    for (int i = 0; i < nx; ++i)
        for (int k = 0; k < nz; ++k) node[size_t(i)][size_t(k)] = warped(i, k);

    // --- 2. Superblocks -----------------------------------------------------
    //
    // A rectangle of the lattice with its interior streets removed. Real cities
    // are full of them and they are most of why block sizes vary by an order of
    // magnitude rather than a little.
    auto edgeKey = [&](int i0, int k0, bool horizontal) {
        return (i0 * 1000 + k0) * 2 + (horizontal ? 1 : 0);
    };
    std::set<int> removed;
    for (int n = 0; n < p.superblocks; ++n) {
        int w = rng.rangeI(1, 2), h = rng.rangeI(1, 2);
        if (nx - 1 - w < 1 || nz - 1 - h < 1) continue;
        int i0 = rng.rangeI(1, nx - 2 - w), k0 = rng.rangeI(1, nz - 2 - h);
        for (int i = i0; i <= i0 + w; ++i)
            for (int k = k0; k <= k0 + h; ++k) {
                // Interior edges only: the rectangle's own perimeter stays, so
                // the superblock is enclosed rather than open.
                if (i < i0 + w && k > k0 && k < k0 + h) removed.insert(edgeKey(i, k, true));
                if (k < k0 + h && i > i0 && i < i0 + w) removed.insert(edgeKey(i, k, false));
            }
    }

    // --- 3. Street hierarchy ------------------------------------------------
    //
    // "More multilane" is a question of which lines get promoted. Every third
    // line is an arterial and the middle one of each axis is a divided
    // boulevard, so the network has a real spine rather than one uniform grade
    // of street.
    const int midI = nx / 2, midK = nz / 2;
    auto tierOf = [&](int line, int mid) {
        if (line == mid) return 2;                   // divided boulevard
        if (line % 3 == 0) return 1;                 // arterial
        return 0;                                    // street or collector
    };

    struct Edge {
        int road;
        int i0, k0, i1, k1;
    };
    std::vector<Edge> edges;

    auto addEdge = [&](int i0, int k0, int i1, int k1, bool horizontal) {
        if (removed.count(edgeKey(i0, k0, horizontal))) return;
        int tier = horizontal ? tierOf(k0, midK) : tierOf(i0, midI);
        RoadDesc d;
        char buf[64];
        std::snprintf(buf, sizeof buf, "%s-%d-%d", horizontal ? "ew" : "ns", i0, k0);
        d.name = buf;
        switch (tier) {
            case 2:
                d.preset = rng.chance(0.35) ? "boulevard_twltl" : "arterial6_median";
                d.designSpeed = 60;
                d.cornerRadius = 220;
                break;
            case 1:
                d.preset = "arterial4";
                d.designSpeed = 55;
                d.cornerRadius = 170;
                break;
            default:
                // Collectors are still four lanes; they are what keeps a city
                // from being all boulevards and all lanes-of-two.
                if (rng.chance(0.30)) {
                    d.preset = "collector4";
                    d.designSpeed = 45;
                } else {
                    d.preset = p.parking && rng.chance(0.65) ? "street2_park" : "street2";
                    d.designSpeed = 40;
                }
                d.cornerRadius = 95;
                break;
        }
        Vec2 a = node[size_t(i0)][size_t(k0)];
        Vec2 b = node[size_t(i1)][size_t(k1)];
        if (rng.chance(p.curveChance)) {
            // Bow the edge. Straight and curved streets in the same plan is the
            // point — a network that is all curves reads as sloppy, and one that
            // is all straight never exercises the geometry.
            Vec2 mid = (a + b) * 0.5;
            double bow = length(b - a) * p.curveBow * rng.range(-1.0, 1.0);
            d.points = {a, mid + perpLeft(normalize(b - a)) * bow, b};
        } else {
            d.points = {a, b};
        }
        d.baseHeight = 0.0;
        edges.push_back({buildRoad(out.net, d), i0, k0, i1, k1});
    };

    for (int k = 0; k < nz; ++k)
        for (int i = 0; i + 1 < nx; ++i) addEdge(i, k, i + 1, k, true);
    for (int i = 0; i < nx; ++i)
        for (int k = 0; k + 1 < nz; ++k) addEdge(i, k, i, k + 1, false);

    // --- 4. Radial arterials out to the ring --------------------------------
    //
    // Planned before the junction pass so a radial is just another arm at the
    // rim node it leaves from, rather than something bolted on afterwards.
    struct Radial {
        int road = -1;
        Vec2 outer{0, 0};
        double outerHeading = 0;
    };
    std::vector<Radial> radials;

    Vec2 gridLo{xs.front(), zs.front()}, gridHi{xs.back(), zs.back()};
    Vec2 gridMid = (gridLo + gridHi) * 0.5;
    // Standoff is measured from the half-DIAGONAL, not the half-width. Measured
    // from the width, a ring clears the middle of each side by the full standoff
    // and the corners by almost nothing — and a radial aimed at a corner then
    // has no room to exist between the grid and the ring.
    const double halfDiag = 0.5 * length(gridHi - gridLo);
    const double ringBase = halfDiag + p.bypassStandoff;

    if (p.bypass && p.interchanges > 0) {
        for (int n = 0; n < p.interchanges; ++n) {
            // Half a step offset, so no interchange lands on the ring's seam at
            // bearing zero — the one place the carriageway cannot carry a ramp.
            double ang = (2.0 * kPi * (double(n) + 0.5)) / p.interchanges +
                         rng.range(-0.14, 0.14);
            Vec2 dir{std::cos(ang), std::sin(ang)};

            // The rim node furthest along this bearing is where the radial
            // leaves the street network.
            int bi = 0, bk = 0;
            double best = -1e300;
            for (int i = 0; i < nx; ++i) {
                for (int k = 0; k < nz; ++k) {
                    if (i != 0 && k != 0 && i != nx - 1 && k != nz - 1) continue;
                    double score = dot(node[size_t(i)][size_t(k)] - gridMid, dir);
                    if (score > best) {
                        best = score;
                        bi = i;
                        bk = k;
                    }
                }
            }
            Vec2 start = node[size_t(bi)][size_t(bk)];
            // Stop short of the ring: the ramps cover the last stretch.
            double reach = std::max(ringBase, minRadiusForSpeed(110.0) * 1.06) - 130.0;
            Vec2 target{gridMid.x + dir.x * reach, gridMid.y + dir.y * reach};
            if (length(target - start) < 140.0) continue;

            RoadDesc d;
            char buf[64];
            std::snprintf(buf, sizeof buf, "radial-%d", n);
            d.name = buf;
            d.preset = "arterial4";
            d.designSpeed = 70;
            d.cornerRadius = 240;
            // Bow proportional to the run. A fixed offset on a short radial is a
            // curve far below the minimum radius for 70 km/h, and the design lint
            // is right to say so.
            double run = length(target - start);
            Vec2 mid = (start + target) * 0.5;
            mid = mid + perpLeft(normalize(target - start)) * run * rng.range(-0.11, 0.11);
            d.points = {start, mid, target};
            int r = buildRoad(out.net, d);
            edges.push_back({r, bi, bk, -1, -1});   // an arm at its inner end only

            Radial rad;
            rad.road = r;
            const Road& rr = out.net.road(r);
            Frame f = rr.spine.frameAt(rr.end());
            rad.outer = f.planPos;
            rad.outerHeading = f.heading;
            radials.push_back(rad);
        }
    }

    // --- 5. A junction per lattice node -------------------------------------
    int rbI = p.roundabout ? rng.rangeI(1, nx - 2) : -1;
    int rbK = p.roundabout ? rng.rangeI(1, nz - 2) : -1;

    for (int i = 0; i < nx; ++i) {
        for (int k = 0; k < nz; ++k) {
            std::vector<std::pair<int, bool>> arms;
            for (const Edge& e : edges) {
                if (e.i0 == i && e.k0 == k) arms.push_back({e.road, false});
                if (e.i1 == i && e.k1 == k) arms.push_back({e.road, true});
            }
            if (arms.size() < 2) continue;
            char buf[64];
            std::snprintf(buf, sizeof buf, "node-%d-%d", i, k);
            if (i == rbI && k == rbK && arms.size() >= 3) {
                RoundaboutDesc rd;
                rd.name = buf;
                rd.center = node[size_t(i)][size_t(k)];
                rd.radius = 15.0;
                rd.arms = arms;
                buildRoundabout(out.net, rd);
                continue;
            }
            int ti = tierOf(i, midI), tk = tierOf(k, midK);
            JunctionControl ctl = JunctionControl::Uncontrolled;
            if (ti > 0 && tk > 0) ctl = p.signals ? JunctionControl::Signalized
                                                  : JunctionControl::AllWayStop;
            else if (ti > 0 || tk > 0) ctl = JunctionControl::PriorityStop;
            else if (arms.size() > 2) ctl = JunctionControl::AllWayStop;
            buildIntersection(out.net, buf, arms, ctl, arms.size() > 3 ? 9.0 : 6.5);
        }
    }

    if (!p.bypass) return;

    // --- 6. The ring --------------------------------------------------------
    //
    // Two halves linked at both seams, which closes the loop without needing a
    // road whose spine bites its own tail. Traffic can circulate the whole way.
    // A circle rather than an ellipse, and built from ARCS rather than from a
    // filleted polyline. A polyline's corner radius is what it can fit between
    // adjacent points, so asking for a 560 m fillet on a polygon that only has
    // room for 400 m silently gets 400 — and the design lint then correctly
    // reports a motorway below its minimum radius. An arc spine simply is the
    // radius it says it is.
    //
    // The ring is sized by the DESIGN SPEED first: a bypass below the minimum
    // radius for 110 km/h is not a bypass. A small town therefore gets a ring
    // standing further out, not a tighter one.
    const double bypassSpeed = 110.0;
    const double ringR = std::max(ringBase, minRadiusForSpeed(bypassSpeed) * 1.06);
    RoadPreset ringPreset = roadPreset(p.bypassPreset);

    // One closed carriageway rather than two halves: every seam is a place an
    // interchange cannot go, so the fewer of them the better. Ramps split it
    // into pieces as they are added, and the pieces stay windows over this one
    // circular spine with absolute stations — which is what lets an interchange
    // planned at a station still find its piece after the ones upstream of it
    // have already cut the road up.
    Carriageway ring;
    {
        Road r;
        r.name = "bypass";
        r.designSpeed = bypassSpeed;
        r.allowsPedestrians = false;
        r.spine = spineCircle(gridMid, ringR, true);
        r.spine.setCrossfall(0.02);
        r.spine.setFlatElevation(0.0);
        r.spine.applySuperelevation(bypassSpeed);
        r.spine.finalize();
        LaneSection sec = ringPreset.section;
        sec.s0 = 0;
        sec.assignIds();
        r.xs.sections.push_back(sec);
        r.xs.finalize(r.spine.length());
        ring.pieces.push_back(out.net.addRoad(std::move(r)));
    }

    // --- 7. Interchanges ----------------------------------------------------
    //
    // Each radial gets a diverge upstream of it and a merge downstream, and both
    // ramps land on the radial's outer end, so the terminal is a real junction
    // rather than two ramps stopping in a field.
    struct Planned {
        double s;
        size_t radial;
    };
    std::vector<Planned> planned;
    const double ringLen = out.net.road(ring.pieces.front()).spineLength();
    for (size_t n = 0; n < radials.size(); ++n) {
        double s = 0, t = 0;
        if (!stationNear(out.net, ring, radials[n].outer, s, t)) continue;
        // Clear of the seam, and with room for the ramps either side.
        if (s < 260.0 || s > ringLen - 260.0) continue;
        planned.push_back({s, n});
    }
    // Upstream first: each ramp splits the carriageway, and working along it in
    // order keeps every later station on a piece that still exists.
    std::sort(planned.begin(), planned.end(),
              [](const Planned& a, const Planned& b) { return a.s < b.s; });

    int rampCount = 0;
    for (const Planned& pl : planned) {
        const Radial& rad = radials[pl.radial];
        // The terminal faces the city, so the off-ramp arrives pointing inward
        // and the on-ramp leaves pointing back out.
        double inward = wrapPi(rad.outerHeading + kPi);

        RampDesc off;
        off.name = "off-ramp";
        off.preset = "ramp1";
        off.outerPoint = rad.outer;
        off.outerHeading = inward;
        off.auxLength = 150.0;
        off.designSpeed = 60;
        int offRoad = -1, tailA = -1;
        {
            int on = ring.at(out.net, pl.s - 150.0);
            if (on >= 0) offRoad = buildOffRamp(out.net, on, pl.s - 150.0, off, &tailA);
            ring.split(tailA);
        }

        RampDesc onr;
        onr.name = "on-ramp";
        onr.preset = "ramp1";
        onr.outerPoint = rad.outer;
        onr.outerHeading = rad.outerHeading;
        onr.auxLength = 170.0;
        onr.designSpeed = 60;
        int onRoad = -1, tailB = -1;
        {
            int on = ring.at(out.net, pl.s + 150.0);
            if (on >= 0) onRoad = buildOnRamp(out.net, on, pl.s + 150.0, onr, &tailB);
            ring.split(tailB);
        }

        std::vector<std::pair<int, bool>> arms;
        arms.push_back({rad.road, true});
        if (offRoad >= 0) arms.push_back({offRoad, true});     // arrives at its end
        if (onRoad >= 0) arms.push_back({onRoad, false});      // leaves from its start
        if (arms.size() >= 2) {
            char buf[64];
            std::snprintf(buf, sizeof buf, "terminal-%zu", pl.radial);
            buildIntersection(out.net, buf, arms,
                              p.signals ? JunctionControl::Signalized : JunctionControl::Yield,
                              9.0);
            ++rampCount;
        }
    }
    (void)rampCount;

    // Close the ring. Plain links, because a carriageway continuing into another
    // carriageway is not a junction — nothing crosses.
    // Close the loop. A plain link, because a carriageway continuing into
    // itself is not a junction — nothing crosses.
    if (ring.pieces.size() > 1) {
        int from = ring.pieces.back(), to = ring.pieces.front();
        out.net.road(from).succ = RoadLink{LinkType::Road, to, true, {}};
        out.net.road(to).pred = RoadLink{LinkType::Road, from, false, {}};
    }
}

}  // namespace roadlab
