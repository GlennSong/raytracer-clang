#include "scene.h"

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
        out.terrain.slopeWidth = t.value("slopeWidth", 8.0);
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
    std::vector<std::pair<int, bool>> arms;
    const char* names[4] = {"north", "east", "south", "west"};
    for (int i = 0; i < 4; ++i) {
        double a = kPi * 0.5 * i + 0.12;
        Vec2 outer{centre.x + std::cos(a) * (R + approach), centre.y + std::sin(a) * (R + approach)};
        Vec2 inner{centre.x + std::cos(a) * (R + 26.0), centre.y + std::sin(a) * (R + 26.0)};
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

}  // namespace roadlab
