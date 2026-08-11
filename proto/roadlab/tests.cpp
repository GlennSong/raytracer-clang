// roadlab unit tests. No framework — the prototype is meant to stay
// dependency-free and buildable anywhere.
//
// The tests worth having here are the INVARIANT ones: the claims the whole
// design rests on. That a taper's paint tracks the lane boundary it is drawn
// from. That a signal phase never runs two conflicting movements. That the
// simulator's lane-change legality is the same field the shader paints. If those
// hold, the "one source of truth" story is true; if they do not, it is a slogan.

#include "builders.h"
#include "junction.h"
#include "network.h"
#include "odr.h"
#include "profile.h"
#include "props.h"
#include "scene.h"
#include "sim.h"
#include "spine.h"
#include "structure.h"
#include "surface.h"
#include "tessellate.h"

#include <cstdio>
#include <cstring>
#include <string>

using namespace roadlab;

namespace {

int gChecks = 0;
int gFailures = 0;
const char* gGroup = "";

void group(const char* name) {
    gGroup = name;
    std::printf("\n-- %s\n", name);
}

void check(bool ok, const char* what) {
    ++gChecks;
    if (!ok) {
        ++gFailures;
        std::printf("  FAIL [%s] %s\n", gGroup, what);
    }
}

void checkNear(double a, double b, double tol, const char* what) {
    ++gChecks;
    if (!(std::fabs(a - b) <= tol)) {
        ++gFailures;
        std::printf("  FAIL [%s] %s (got %.6f, want %.6f +- %.6f)\n", gGroup, what, a, b, tol);
    }
}

// --- geometry -------------------------------------------------------------

void testPoly() {
    group("Poly3");
    Poly3 p = Poly3::smooth(2.0, 5.0, 10.0);
    checkNear(p.eval(0), 2.0, 1e-12, "smooth starts at v0");
    checkNear(p.eval(10), 5.0, 1e-12, "smooth ends at v1");
    checkNear(p.deriv(0), 0.0, 1e-12, "smooth has zero slope at the start");
    checkNear(p.deriv(10), 0.0, 1e-12, "smooth has zero slope at the end");
    Poly3 h = Poly3::hermite(0.0, 0.1, 4.0, -0.05, 20.0);
    checkNear(h.eval(20), 4.0, 1e-10, "hermite hits its end value");
    checkNear(h.deriv(20), -0.05, 1e-10, "hermite hits its end slope");
}

void testSpine() {
    group("Spine");
    Spine s;
    s.setStart({0, 0}, 0.0);
    s.addLine(100.0);
    s.finalize();
    checkNear(s.length(), 100.0, 1e-9, "line length");
    Frame f = s.frameAt(50.0);
    checkNear(f.planPos.x, 50.0, 1e-9, "line advances along +x");
    checkNear(f.planPos.y, 0.0, 1e-9, "line stays on the axis");

    // A quarter arc of radius 50 must land exactly on the circle.
    Spine a;
    a.setStart({0, 0}, 0.0);
    a.addArc(0.5 * kPi * 50.0, 1.0 / 50.0);
    a.finalize();
    Frame end = a.frameAt(a.length());
    checkNear(end.planPos.x, 50.0, 1e-6, "quarter arc endpoint x");
    checkNear(end.planPos.y, 50.0, 1e-6, "quarter arc endpoint y");
    checkNear(end.heading, kPi * 0.5, 1e-9, "quarter arc turns 90 degrees");

    // A clothoid's curvature is linear in s — that is its whole reason to exist.
    Spine c;
    c.setStart({0, 0}, 0.0);
    c.addSpiral(80.0, 0.0, 1.0 / 60.0);
    c.finalize();
    checkNear(c.curvatureAt(0.0), 0.0, 1e-12, "spiral starts straight");
    checkNear(c.curvatureAt(40.0), 0.5 / 60.0, 1e-12, "spiral curvature is linear in s");
    checkNear(c.curvatureAt(80.0), 1.0 / 60.0, 1e-12, "spiral reaches the target curvature");
    // ...and its length is arc length, so a fine polyline of it agrees.
    double walk = 0;
    Vec2 prev = c.frameAt(0).planPos;
    for (double t = 0.05; t <= 80.0; t += 0.05) {
        Vec2 p = c.frameAt(t).planPos;
        walk += length(p - prev);
        prev = p;
    }
    checkNear(walk, 80.0, 0.02, "spiral arc length matches its parameter");

    // Polyline chaining: G2 corners, and the result still passes near the
    // corner points it was built from.
    Spine poly = spineFromPolyline({{0, 0}, {200, 0}, {200, 200}}, 60.0);
    check(poly.length() > 300.0 && poly.length() < 420.0, "polyline length is plausible");
    Frame pe = poly.frameAt(poly.length());
    checkNear(pe.planPos.x, 200.0, 0.5, "polyline ends at the last point (x)");
    checkNear(pe.planPos.y, 200.0, 0.5, "polyline ends at the last point (y)");
    bool sawSpiral = false;
    for (const GeomPrim& g : poly.prims())
        if (g.kind == GeomKind::Spiral) sawSpiral = true;
    check(sawSpiral, "polyline corners get transition spirals");
}

void testInverseMapping() {
    group("world -> (s,t)");
    Spine s = spineFromPolyline({{0, 0}, {150, 0}, {260, 90}}, 70.0);
    for (double station : {10.0, 80.0, 150.0, 220.0}) {
        for (double lat : {-6.0, 0.0, 4.5}) {
            Vec2 p = s.toPlan(station, lat);
            double gs = 0, gt = 0;
            bool ok = s.toST(p, gs, gt);
            check(ok, "round trip projects");
            checkNear(gs, station, 0.05, "round trip recovers s");
            checkNear(gt, lat, 0.05, "round trip recovers t");
        }
    }
    // A point well past the end must NOT come back as a small lateral offset:
    // that bug is what punched holes in the terrain.
    Vec2 beyond = s.toPlan(s.length(), 0) + dirOf(s.frameAt(s.length()).heading) * 40.0;
    double bs = 0, bt = 0;
    check(!s.toST(beyond, bs, bt), "a point past the end is rejected");

    Spine ring = spineArc({0, 0}, 20.0, 0.0, kPi * 0.5, true);
    double rs = 0, rt = 0;
    check(!ring.toST({-8, -52}, rs, rt), "a point far off an arc is rejected");
}

void testSuperelevation() {
    group("superelevation");
    Spine s;
    s.setStart({0, 0}, 0.0);
    s.addLine(100);
    s.addSpiral(80, 0.0, 1.0 / 300.0);
    s.addArc(200, 1.0 / 300.0);
    s.finalize();
    s.applySuperelevation(100.0, 0.06);
    checkNear(s.superelevation().eval(50.0), 0.0, 1e-9, "tangent is unbanked");
    double inCurve = s.superelevation().eval(250.0);
    // e = v^2 / (127 R) = 10000 / 38100 = 0.262 -> clamped to 0.06, and a LEFT
    // turn banks by dropping its left side, so the roll is negative.
    checkNear(inCurve, -std::atan(0.06), 1e-9, "curve banks into the turn, clamped to maxE");
    double runoff = s.superelevation().eval(140.0);
    check(runoff < 0.0 && runoff > inCurve, "the runoff rides the spiral");
}

// --- cross-section --------------------------------------------------------

void testPresetsAndSections() {
    group("cross-section");
    for (const std::string& name : roadPresetNames()) {
        RoadPreset p = roadPreset(name);
        check(!p.section.left.empty() || !p.section.right.empty(), "preset has strips");
        check(p.designSpeedKph > 0, "preset has a design speed");
    }
    RoadPreset fw = roadPreset("freeway3");
    check(fw.section.laneCount() == 6, "freeway3 has six travel lanes");
    check(!fw.allowsPedestrians, "a freeway is not walkable");
    RoadPreset st = roadPreset("street2_park");
    check(st.section.laneCount() == 2, "street2_park has two travel lanes");

    CrossSection xs;
    LaneSection sec = st.section;
    sec.s0 = 0;
    xs.sections.push_back(sec);
    xs.finalize(100.0);

    std::vector<Boundary> b;
    xs.boundariesAt(50.0, b);
    check(b.size() >= 5, "boundaries exist for every strip edge");
    for (size_t i = 1; i < b.size(); ++i)
        check(b[i].t >= b[i - 1].t - 1e-9, "boundaries are ordered by t");

    // The kerb is a ramp, not a step: heightAt must interpolate across it.
    double hRoad = xs.heightAt(50.0, 0.0);
    double hWalk = xs.heightAt(50.0, xs.leftExtentAt(50.0) - 0.5);
    checkNear(hRoad, 0.0, 1e-9, "carriageway is at zero height");
    check(hWalk > 0.10, "footway stands above the carriageway");
    bool sawMid = false;
    for (double t = 0; t < xs.leftExtentAt(50.0); t += 0.02) {
        double h = xs.heightAt(50.0, t);
        if (h > 0.02 && h < hWalk - 0.02) sawMid = true;
    }
    check(sawMid, "the kerb face ramps rather than steps");
}

void testTapersAndTransitions() {
    group("tapers and transitions");
    // MUTCD, in metric. 3.6 m at 110 km/h wants about 245 m.
    checkNear(taperLength(3.6, 110.0), 3.6 * 110.0 / 1.609344, 0.01, "high-speed taper is W*S");
    check(taperLength(3.6, 50.0) < taperLength(3.6, 110.0), "slower roads taper shorter");
    checkNear(minRadiusForSpeed(designSpeedForRadius(300.0)), 300.0, 1e-6,
              "radius and design speed are inverses");

    LaneSection wide = roadPreset("arterial4").section;
    LaneSection narrow = sectionWithLanesChanged(wide, -1, -1);
    check(narrow.laneCount() == wide.laneCount() - 1, "a lane is removed");

    double len = minTransitionLength(wide, narrow, 60.0);
    check(len > 40.0, "the solver demands a real taper length");
    LaneSection blend = blendSection(wide, narrow, 100.0, len);

    // The dropped lane's width really does reach zero, and the paint that closes
    // it is solid.
    bool sawDrop = false;
    for (const Strip& s : blend.right) {
        double w0 = s.width.eval(0), w1 = s.width.eval(len);
        if (w0 > 1.5 && w1 < 0.05) {
            sawDrop = true;
            check(s.outerMark.style == MarkStyle::Solid || s.outerMark.style == MarkStyle::None,
                  "a closing lane is bounded by a solid line, not a dashed one");
        }
    }
    check(sawDrop, "the transition contains a lane tapering to zero");

    ProfileTimeline tl;
    tl.at(0, wide);
    tl.at(200, narrow, len);
    CrossSection xs = tl.build(600.0, 60.0);
    check(xs.sections.size() == 3, "timeline emits steady/transition/steady");
    checkNear(xs.sections[1].s0, 200.0, 1e-6, "transition starts where asked");
    checkNear(xs.sections[2].s0, 200.0 + len, 1e-6, "steady resumes after the taper");
    check(xs.sectionAt(500.0).laneCount() == narrow.laneCount(), "downstream has the new count");
}

// --- markings -------------------------------------------------------------

void testMarkingsFollowGeometry() {
    group("markings");
    Network net;
    RoadDesc d;
    d.name = "taper";
    d.preset = "arterial4";
    d.points = {{0, 0}, {600, 0}};
    d.autoSuperelevation = false;
    int id = buildRoad(net, d);
    applyLaneChange(net, id, 200.0, -1, -1);
    net.build();
    std::vector<RoadPaint> paint;
    generateRoadPaint(net, paint);
    const Road& r = net.road(id);

    // THE claim: the stripe is drawn about the lane boundary polynomial, so as
    // the lane tapers the paint tapers with it. Find the painted edge line at two
    // stations inside the taper and check it tracks the boundary, not a constant.
    auto paintedEdgeT = [&](double s) {
        double best = 1e9, bestT = 0;
        double lo = r.xs.rightExtentAt(s), hi = 0.0;
        for (double t = lo; t < hi; t += 0.01) {
            SurfaceSample smp = shadeRoadSurface(r, &paint[size_t(id)], s, t, 0.004, ShadeParams{});
            if (!smp.paint) continue;
            // The edge line is the outermost painted band.
            if (t - lo < best) {
                best = t - lo;
                bestT = t;
            }
        }
        return bestT;
    };
    // Two stations INSIDE the taper section, so the width polynomial is the one
    // actually in force at each.
    int sec = r.xs.sectionIndexAt(215.0);
    check(r.xs.sections[size_t(sec)].length > 40.0, "the taper section is a real length");
    double sEarly = r.xs.sections[size_t(sec)].s0 + 8.0;
    double sLate = r.xs.sections[size_t(sec)].s0 + r.xs.sections[size_t(sec)].length * 0.72;
    int outer = 0;
    for (const Strip& st : r.xs.sections[size_t(sec)].right)
        if (st.isLane()) outer = st.id;
    double a0 = 0, a1 = 0, b0 = 0, b1 = 0;
    check(r.xs.laneSpanAt(sec, outer, sEarly, a0, a1), "outer lane exists early in the taper");
    check(r.xs.laneSpanAt(sec, outer, sLate, b0, b1), "outer lane exists late in the taper");
    check(std::fabs(a1) > std::fabs(b1) + 0.3, "the lane boundary really is moving");

    double pa = paintedEdgeT(sEarly);
    double pb = paintedEdgeT(sLate);
    check(pa != 0.0 && pb != 0.0, "an edge line is painted at both stations");
    checkNear(pa, a1, 0.3, "paint sits on the lane boundary early in the taper");
    checkNear(pb, b1, 0.3, "paint follows the boundary later in the taper");
    check(std::fabs(pa - pb) > 0.3, "the painted edge moved with the taper");

    // Filter width is the whole antialiasing story: a huge footprint must fade
    // the stripe rather than alias it.
    SurfaceSample sharp = shadeRoadSurface(r, nullptr, sEarly, a1, 0.004, ShadeParams{});
    SurfaceSample blurry = shadeRoadSurface(r, nullptr, sEarly, a1, 2.0, ShadeParams{});
    check(sharp.albedo.x > blurry.albedo.x + 0.02, "a stripe fades as the filter widens");
}

void testMarkingLegalityMatchesPaint() {
    group("marking legality");
    // A style's picture and its rule are the same field. If these ever disagree
    // the "one source of truth" claim is dead.
    check(!markSolid().crossableFromLeft(), "solid is not crossable");
    check(!markSolid().crossableFromRight(), "solid is not crossable either way");
    check(markDashed().crossableFromLeft(), "dashed is crossable");
    check(!markDouble().crossableFromLeft(), "double yellow is not crossable");
    Marking sd;
    sd.style = MarkStyle::SolidDashed;
    check(sd.crossableFromRight() && !sd.crossableFromLeft(),
          "solid-dashed is crossable from the dashed side only");
}

// --- network and junctions ------------------------------------------------

void testSplitAndLaneGraph() {
    group("split and lane graph");
    Network net;
    RoadDesc d;
    d.name = "main";
    d.preset = "collector4";
    d.points = {{0, 0}, {400, 0}};
    d.autoSuperelevation = false;
    int a = buildRoad(net, d);
    double whole = net.road(a).activeLength();
    int b = net.splitRoad(a, 180.0);
    check(b > 0, "the split produced a second road");
    checkNear(net.road(a).activeLength() + net.road(b).activeLength(), whole, 1e-6,
              "splitting conserves length");
    check(net.road(a).succ.type == LinkType::Road && net.road(a).succ.id == b,
          "the head links to the tail");
    net.build();
    check(!net.lanes().nodes.empty(), "the lane graph has nodes");

    // Every lane on the head must reach a lane on the tail.
    int linked = 0;
    for (const LaneNode& n : net.lanes().nodes) {
        if (n.ref.road != a) continue;
        for (int s : n.successors)
            if (net.lanes().nodes[size_t(s)].ref.road == b) ++linked;
    }
    check(linked >= 2, "lanes continue across the split");

    // Neighbour crossability comes from the marking on the shared boundary.
    for (const LaneNode& n : net.lanes().nodes) {
        if (n.leftNeighbor < 0) continue;
        const LaneNode& l = net.lanes().nodes[size_t(n.leftNeighbor)];
        check(l.dir == n.dir, "a travel-frame neighbour runs the same way");
    }
}

void testJunctionAndSignals() {
    group("junction");
    Network net;
    RoadDesc d;
    d.preset = "arterial4";
    d.autoSuperelevation = false;
    d.cornerRadius = 200;
    int arms[4];
    const double L = 160.0;
    for (int i = 0; i < 4; ++i) {
        double ang = kPi * 0.5 * i;
        d.name = "arm" + std::to_string(i);
        d.points = {{std::cos(ang) * L, std::sin(ang) * L},
                    {std::cos(ang) * 12.0, std::sin(ang) * 12.0}};
        arms[i] = buildRoad(net, d);
    }
    buildIntersection(net, "x", {{arms[0], true}, {arms[1], true}, {arms[2], true}, {arms[3], true}},
                      JunctionControl::Signalized, 8.0);
    net.build();
    const Junction& j = net.junction(0);

    check(j.arms.size() == 4, "four arms");
    check(j.boundary.size() >= 4, "the pad has a boundary polygon");
    check(!j.connections.empty(), "movements were generated");
    bool through = false, left = false, right = false;
    for (const Connection& c : j.connections) {
        if (c.turn == TurnKind::Through) through = true;
        if (c.turn == TurnKind::Left) left = true;
        if (c.turn == TurnKind::Right) right = true;
        check(c.connectorRoad >= 0, "every movement has a connector road");
        check(net.road(c.connectorRoad).kind == RoadKind::Connector, "connectors are marked");
    }
    check(through && left && right, "through, left and right movements all exist");
    check(!j.conflicts.empty(), "conflict points were found");
    check(!j.phases.empty(), "signal phases were generated");

    // THE invariant: no signal phase may contain two movements that cross.
    for (const ConflictPoint& cp : j.conflicts) {
        if (cp.kind != ConflictKind::Crossing) continue;
        int ga = j.connections[size_t(cp.connA)].signalGroup;
        int gb = j.connections[size_t(cp.connB)].signalGroup;
        check(ga != gb, "two crossing movements are never green together");
    }
    // Arms were trimmed back rather than destroyed.
    for (const JunctionArm& a : j.arms) {
        check(a.trim > 0.0, "the arm was pulled back for the pad");
        checkNear(net.road(a.road).spineLength(), 148.0, 6.0,
                  "the underlying geometry is untouched by trimming");
    }
    // Left turns give way even on a signalised approach.
    for (const Connection& c : j.connections)
        if (c.turn == TurnKind::Left) check(c.yields, "a permissive left yields");
}

void testRoundabout() {
    group("roundabout");
    Scene sc;
    buildDemo("roundabout", sc);
    finalizeScene(sc, false, false);
    int rings = 0, entries = 0;
    for (const Road& r : sc.net.roads())
        if (r.kind == RoadKind::RoundaboutRing) ++rings;
    check(rings == 4, "the ring is four arcs, one per gap between arms");
    for (const Junction& j : sc.net.junctions()) {
        check(j.control == JunctionControl::RoundaboutEntry, "arms are roundabout entries");
        for (const Connection& c : j.connections) {
            const Road& from = sc.net.road(j.arms[size_t(c.fromArm)].road);
            if (from.kind == RoadKind::RoundaboutRing) {
                check(!c.yields, "circulating traffic has priority");
            } else {
                check(c.yields, "entering traffic gives way");
                ++entries;
            }
        }
    }
    check(entries > 0, "there are entry movements that yield");
    check(sc.lint.empty(), "the generated roundabout passes its own design lint");
}

void testRamps() {
    group("ramps");
    Network net;
    RoadDesc fw;
    fw.name = "freeway";
    fw.preset = "freeway3";
    fw.points = {{-600, 0}, {600, 0}};
    fw.autoSuperelevation = false;
    int f = buildRoad(net, fw);
    int lanesBefore = net.road(f).xs.sectionAt(10.0).laneCount();

    RampDesc rd;
    rd.outerPoint = {300, -220};
    rd.outerHeading = 70 * kDeg2Rad;
    rd.auxLength = 180;
    int tail = -1;
    int ramp = buildOnRamp(net, f, 700.0, rd, &tail);
    check(ramp >= 0, "the ramp was built");
    check(tail >= 0, "the mainline was split at the merge");
    check(net.road(ramp).kind == RoadKind::Ramp, "the ramp is marked as one");
    check(net.extraLinks.size() == 1, "the merge is one explicit lane link");

    // The auxiliary lane grows out of nothing: at the merge station the mainline
    // has one more lane than it started with.
    const Road& t = net.road(tail);
    check(t.xs.sectionAt(t.begin() + 1.0).laneCount() == lanesBefore + 1,
          "an auxiliary lane exists at the merge");
    net.build();
    const ExtraLaneLink& el = net.extraLinks.front();
    int nFrom = net.lanes().find({el.fromRoad,
                                  net.road(el.fromRoad).xs.sectionIndexAt(
                                      net.road(el.fromRoad).end() - 1e-3),
                                  el.fromLane});
    check(nFrom >= 0, "the ramp lane is in the lane graph");
    bool reaches = false;
    if (nFrom >= 0)
        for (int s : net.lanes().nodes[size_t(nFrom)].successors)
            if (net.lanes().nodes[size_t(s)].ref.road == el.toRoad) reaches = true;
    check(reaches, "the ramp lane continues into the mainline auxiliary lane");
}

// --- structures -----------------------------------------------------------

void testStructuresAndClearance() {
    group("structures");
    Network net;
    RoadDesc lower;
    lower.name = "under";
    lower.preset = "arterial4";
    lower.points = {{-200, 0}, {200, 0}};
    lower.autoSuperelevation = false;
    buildRoad(net, lower);

    RoadDesc upper;
    upper.name = "over";
    upper.preset = "collector4";
    upper.points = {{0, -220}, {0, 220}};
    upper.autoSuperelevation = false;
    int o = buildRoad(net, upper);
    double L = net.road(o).spineLength();
    makeOverpass(net, o, L * 0.5 - 30, L * 0.5 + 30, 7.5, 120.0);
    net.build();
    check(!net.road(o).structures.empty(), "the overpass declared spans");
    check(carrierAt(net.road(o), L * 0.5) == CarrierKind::Bridge, "mid-span is a bridge");
    check(carrierAt(net.road(o), 5.0) == CarrierKind::AtGrade, "the far approach is at grade");

    // Terrain conforms under an at-grade road and NOT under a bridge. That one
    // difference is the whole distinction between a causeway and an overpass.
    TerrainParams tp;
    tp.amplitude = 4.0;
    double underBridge = terrainHeightAt(net, tp, 0.0, 0.0);
    double bridgeDeck = net.road(o).surfacePoint(L * 0.5, 0).y;
    check(bridgeDeck > underBridge + 5.0, "the deck stands clear of the ground below");

    Vec2 approach = net.road(o).spine.toPlan(8.0, 0);
    double atGrade = terrainHeightAt(net, tp, approach.x, approach.y);
    checkNear(atGrade, net.road(o).surfacePoint(8.0, 0).y, 0.35,
              "the ground follows an at-grade road");

    check(net.validate().empty(), "a 7.5 m overpass clears the road beneath it");

    // Drop it too low and the lint must say so.
    Network tight;
    buildRoad(tight, lower);
    int o2 = buildRoad(tight, upper);
    makeOverpass(tight, o2, L * 0.5 - 30, L * 0.5 + 30, 3.0, 120.0);
    tight.build();
    bool flagged = false;
    for (const std::string& m : tight.validate())
        if (m.find("clearance") != std::string::npos) flagged = true;
    check(flagged, "an under-height overpass is reported");
}

void testTunnelKeepsTerrain() {
    group("tunnel");
    Network net;
    RoadDesc d;
    d.name = "bore";
    d.preset = "tunnel2";
    d.points = {{0, -300}, {0, 300}};
    d.autoSuperelevation = false;
    int t = buildRoad(net, d);
    double L = net.road(t).spineLength();
    makeTunnel(net, t, L * 0.35, L * 0.65, 12.0, 120.0);
    net.build();
    TerrainParams tp;
    tp.amplitude = 3.0;
    Vec2 mid = net.road(t).spine.toPlan(L * 0.5, 0);
    double ground = terrainHeightAt(net, tp, mid.x, mid.y);
    double road = net.road(t).surfacePoint(L * 0.5, 0).y;
    check(ground > road + 8.0, "the ground closes over the bore");
    checkNear(ground, terrainBaseHeight(tp, mid.x, mid.y), 1e-9,
              "a tunnel does not deform the terrain at all");
}

// --- junction surfaces ----------------------------------------------------

void testPolygonUtilities() {
    group("polygon utilities");
    // A concave polygon (an L) must still triangulate into n-2 triangles that
    // cover exactly its own area — the fan-from-centroid it replaces does not.
    std::vector<Vec2> el = {{0, 0}, {6, 0}, {6, 2}, {2, 2}, {2, 6}, {0, 6}};   // area 20
    auto tris = triangulatePolygon(el);
    check(tris.size() == el.size() - 2, "ear clipping yields n-2 triangles");
    double area = 0;
    for (const auto& t : tris) {
        Vec2 a = el[t[0]], b = el[t[1]], c = el[t[2]];
        area += std::fabs(cross(b - a, c - a)) * 0.5;
    }
    checkNear(area, 20.0, 1e-6, "the triangles cover the concave polygon exactly");

    // Mean value coordinates: partition of unity, and exact at the vertices.
    std::vector<Vec2> quad = {{0, 0}, {10, 0}, {12, 8}, {-1, 7}};
    std::vector<double> w;
    check(meanValueCoords(quad, {4, 3}, w), "MVC solves for an interior point");
    double sum = 0;
    for (double v : w) sum += v;
    checkNear(sum, 1.0, 1e-9, "MVC is a partition of unity");
    for (double v : w) check(v > -1e-9, "MVC is positive inside a convex polygon");
    for (size_t i = 0; i < quad.size(); ++i) {
        check(meanValueCoords(quad, quad[i], w), "MVC solves at a vertex");
        checkNear(w[i], 1.0, 1e-9, "MVC reproduces the vertex exactly");
    }
    // On an edge it must be the linear blend of that edge's endpoints only.
    check(meanValueCoords(quad, (quad[0] + quad[1]) * 0.5, w), "MVC solves on an edge");
    checkNear(w[0], 0.5, 1e-9, "MVC on an edge blends its endpoints");
    checkNear(w[1], 0.5, 1e-9, "MVC on an edge blends its endpoints");
    checkNear(w[2] + w[3], 0.0, 1e-9, "MVC on an edge ignores the far vertices");
}

void testGradedJunctionSurface() {
    group("graded junction");
    Scene sc;
    check(buildDemo("grades", sc), "the graded demo builds");
    finalizeScene(sc, true, false);
    const Junction& j = sc.net.junction(0);
    check(j.arms.size() == 4, "four skewed arms");
    check(j.boundary.size() == j.boundaryHeight.size(), "every boundary vertex carries a height");

    // The arms really do arrive at different heights and grades — otherwise the
    // test proves nothing.
    double lo = 1e9, hi = -1e9;
    for (const JunctionArm& a : j.arms) {
        double y = sc.net.road(a.road).surfacePoint(a.sContact, 0).y;
        lo = std::min(lo, y);
        hi = std::max(hi, y);
    }
    check(hi - lo > 0.15, "the approaches meet the pad at genuinely different heights");

    // THE claim: the pad meets every arm at that arm's own surface, across the
    // arm's full width — including whatever crossfall and bank it carries. A
    // single averaged pad height cannot do this.
    for (const JunctionArm& a : j.arms) {
        const Road& r = sc.net.road(a.road);
        for (double f : {0.05, 0.5, 0.95}) {
            double t = lerp(a.rightExtent, a.leftExtent, f);
            Vec2 p = r.spine.toPlan(a.sContact, t);
            double armY = r.surfacePoint(a.sContact, t).y;
            double padY = junctionElevationAt(sc.net, j, p);
            checkNear(padY, armY, 0.05, "the pad meets the arm at the arm's own height");
        }
    }

    // ...and the interior is a smooth blend, not a plateau or a set of steps.
    double prev = junctionElevationAt(sc.net, j, j.center);
    check(prev > lo - 0.5 && prev < hi + 0.5, "the pad centre lies between the arms");
    double maxJump = 0;
    for (int i = 0; i <= 60; ++i) {
        Vec2 a = j.arms[0].contact, b = j.arms[2].contact;
        Vec2 p = a + (b - a) * (double(i) / 60.0);
        double h = junctionElevationAt(sc.net, j, p);
        if (i > 0) maxJump = std::max(maxJump, std::fabs(h - prev));
        prev = h;
    }
    check(maxJump < 0.25, "the pad surface has no steps across it");

    check(sc.lint.empty(), "the graded junction passes the design lint");
}

// --- props ----------------------------------------------------------------

void testPropsFollowSemantics() {
    group("props");
    Scene sc;
    buildDemo("lanes", sc);
    finalizeScene(sc, false, true);
    int signals = 0, lights = 0, laneEnds = 0;
    for (const Prop& p : sc.props) {
        if (p.kind == PropKind::TrafficSignal) ++signals;
        if (p.kind == PropKind::StreetLight) ++lights;
        if (p.kind == PropKind::SignLaneEnds) ++laneEnds;
    }
    check(signals >= 3, "a signalised junction gets a head per approach");
    check(lights > 5, "lamps were placed along the road");
    check(laneEnds >= 1, "the lane drop produced a LANE ENDS sign");

    // A stop sign must never appear at a signalised junction: both come from the
    // same control policy, so they cannot both be right.
    for (const Prop& p : sc.props) check(p.kind != PropKind::SignStop, "no stop signs under signals");

    Scene rb;
    buildDemo("roundabout", rb);
    finalizeScene(rb, false, true);
    int yields = 0;
    for (const Prop& p : rb.props)
        if (p.kind == PropKind::SignYield) ++yields;
    check(yields >= 4, "every roundabout entry gets a give-way sign");
}

// --- simulation -----------------------------------------------------------

void testSimulation() {
    group("simulation");
    Scene sc;
    buildDemo("urban", sc);
    finalizeScene(sc, false, false);

    SimParams sp;
    sp.seed = 12;
    Simulation sim(sc.net, sp);
    sim.seedParking(0.5);
    sim.seedVehicles(120);
    sim.seedPedestrians(30);
    check(sim.stats().vehicles > 50, "vehicles were seeded");
    check(sim.stats().parkedCars > 0, "parking bays were filled from the strip rule");

    // A parked car must be IN a parking strip — not in a travel lane, not on the
    // footway. The slot rule and the strip it came from have to agree.
    for (const ParkingSlot& slot : sim.parking()) {
        const Road& r = sc.net.road(slot.road);
        int laneId = 0;
        const Strip* st = r.xs.stripAtT(slot.s, slot.t, &laneId);
        check(st != nullptr, "a parking slot lands on a strip");
        if (st) check(st->kind == StripKind::Parking, "a parking slot sits in a parking strip");
    }

    sim.run(60.0, 0.1);
    SimStats st = sim.stats();
    check(st.vehicles > 50, "vehicles survive a minute of simulation");
    check(st.meanSpeedKph > 3.0, "traffic is actually moving");
    check(st.meanSpeedKph < 90.0, "traffic obeys something like a speed limit");
    check(st.completedTrips > 0, "some vehicles got somewhere");

    // No two vehicles may occupy the same metre of the same lane.
    for (size_t i = 0; i < sim.vehicles().size(); ++i) {
        for (size_t k = i + 1; k < sim.vehicles().size(); ++k) {
            const Vehicle& a = sim.vehicles()[i];
            const Vehicle& b = sim.vehicles()[k];
            if (!a.active || !b.active || a.lane != b.lane) continue;
            // Two vehicles legitimately share longitudinal space for a moment
            // while one is committing a lane change; the invariant is about
            // settled traffic.
            if (a.changingTo >= 0 || b.changingTo >= 0) continue;
            double gap = std::fabs(a.travelled - b.travelled) - 0.5 * (a.length + b.length);
            check(gap > -1.2, "vehicles in a lane do not overlap");
        }
    }
    // Every vehicle is somewhere real.
    for (const Vehicle& v : sim.vehicles()) {
        Vec3 pos;
        double yaw = 0;
        if (!v.active) continue;
        check(sim.vehiclePose(v, pos, yaw), "every active vehicle has a pose");
        check(std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z),
              "poses are finite");
    }
}

void testRouting() {
    group("routing");
    Scene sc;
    buildDemo("urban", sc);
    finalizeScene(sc, false, false);
    const std::vector<LaneNode>& nodes = sc.net.lanes().nodes;

    SimParams sp;
    sp.seed = 9;
    Simulation sim(sc.net, sp);
    sim.seedVehicles(90);

    int routed = 0;
    for (const Vehicle& v : sim.vehicles()) {
        if (v.route.size() < 2) continue;
        ++routed;
        check(v.route.front() == v.lane, "a route starts at the vehicle's own lane");
        check(v.targetRoad == nodes[size_t(v.route.back())].ref.road,
              "the target road is the route's last road");
        // Every step of the plan must be an edge the lane graph actually has.
        for (size_t i = 0; i + 1 < v.route.size(); ++i) {
            const LaneNode& a = nodes[size_t(v.route[i])];
            bool linked = false;
            for (int nx : a.successors)
                if (nx == v.route[i + 1]) linked = true;
            if (!linked) {
                check(false, "consecutive route nodes are connected in the lane graph");
                break;
            }
        }
    }
    check(routed > 40, "most vehicles were given a route");

    sim.run(100.0, 0.1);
    SimStats st = sim.stats();
    check(st.replans > 0, "routes are planned");
    check(st.completedTrips > 0, "trips complete");
    check(st.mandatoryChanges > 0, "the route forces lane changes");
    check(st.mandatoryChanges <= st.laneChanges, "mandatory changes are a subset of all changes");

    // With routing off nothing plans, which is the control for the above.
    SimParams plainParams = sp;
    plainParams.routing = false;
    Simulation plain(sc.net, plainParams);
    plain.seedVehicles(90);
    plain.run(30.0, 0.1);
    check(plain.stats().replans == 0, "routing off means no plans");
    for (const Vehicle& v : plain.vehicles()) check(v.route.empty(), "routing off means no routes");
}

void testLaneChoiceForAnExit() {
    group("lane choice");
    // THE query the two-resolution graph exists for. On a freeway with an
    // off-ramp, not every lane can reach the ramp — so "which lane must I be in"
    // has a non-trivial answer, and it has to come out of the topology rather
    // than a hand-written rule.
    Scene sc;
    buildDemo("interchange", sc);
    finalizeScene(sc, false, false);
    const LaneGraph& lg = sc.net.lanes();

    int rampRoad = -1;
    for (const Road& r : sc.net.roads())
        if (r.kind == RoadKind::Ramp && r.name == "off-ramp") rampRoad = r.id;
    check(rampRoad >= 0, "the off-ramp exists");
    if (rampRoad < 0) return;

    // The mainline lane that feeds the ramp, found by walking the link the
    // builder recorded rather than by guessing.
    int feederRoad = -1, feederLane = 0;
    for (const ExtraLaneLink& el : sc.net.extraLinks)
        if (el.toRoad == rampRoad) {
            feederRoad = el.fromRoad;
            feederLane = el.fromLane;
        }
    check(feederRoad >= 0, "the diverge is recorded as a lane link");
    if (feederRoad < 0) return;

    const Road& main = sc.net.road(feederRoad);
    int sec = main.xs.sectionIndexAt(main.end() - 1e-3);
    int node = lg.find({feederRoad, sec, feederLane});
    check(node >= 0, "the deceleration lane is in the lane graph");
    if (node < 0) return;

    check(lg.reaches(node, rampRoad, 4), "the deceleration lane reaches the ramp");

    // Count how many lanes of that section can, and how many exist. If the
    // answer were "all of them" the query would be meaningless.
    std::vector<int> can = lg.lanesReaching(node, rampRoad, 4);
    int total = 0;
    for (const LaneNode& n : lg.nodes)
        if (n.ref.road == feederRoad && n.ref.section == sec && n.dir == lg.nodes[size_t(node)].dir)
            ++total;
    check(total >= 3, "the mainline has several lanes here");
    check(!can.empty(), "at least one lane can reach the exit");
    check(int(can.size()) < total, "not every lane can reach the exit");

    // And an inner lane genuinely cannot, so a driver in it must move over.
    int inner = -1;
    for (const LaneNode& n : lg.nodes) {
        int idx = int(&n - &lg.nodes[0]);
        if (n.ref.road != feederRoad || n.ref.section != sec) continue;
        if (n.dir != lg.nodes[size_t(node)].dir) continue;
        bool listed = false;
        for (int c : can)
            if (c == idx) listed = true;
        if (!listed) inner = idx;
    }
    check(inner >= 0, "some lane cannot reach the exit");
    if (inner >= 0) check(!lg.reaches(inner, rampRoad, 4), "and it really cannot");
}

void testSignalsStopTraffic() {
    group("signals");
    Scene sc;
    buildDemo("lanes", sc);
    finalizeScene(sc, false, false);
    SimParams sp;
    sp.seed = 4;
    Simulation sim(sc.net, sp);
    sim.seedVehicles(70);
    sim.run(90.0, 0.1);
    SimStats st = sim.stats();
    // With one signalised junction and a lane drop, SOMETHING should be queueing.
    check(st.stopped + st.moving == st.vehicles, "every vehicle is accounted for");
    check(st.laneChanges > 0, "the lane drop forced lane changes");
    const Junction& j = sc.net.junction(0);
    check(j.cycleLength() > 20.0, "the signal has a plausible cycle");
    int greenNow = 0;
    for (size_t i = 0; i < j.connections.size(); ++i)
        if (j.isGreen(int(i), 5.0)) ++greenNow;
    check(greenNow > 0, "something is green at any given moment");
    check(greenNow < int(j.connections.size()), "not everything is green at once");
}

void testPedestrianGraph() {
    group("pedestrians");
    Scene sc;
    buildDemo("urban", sc);
    finalizeScene(sc, false, false);
    SimParams sp;
    sp.seed = 21;
    Simulation sim(sc.net, sp);

    check(!sim.walkNodes().empty(), "footways became graph nodes");
    check(!sim.walkLinks().empty(), "footways are linked up");

    // Every crossing in the graph must correspond to a zebra the paint generator
    // actually drew. A pedestrian crossing where there is no paint would mean
    // the two systems had separate ideas about where crossings are.
    std::vector<Crosswalk> painted;
    for (const Junction& j : sc.net.junctions())
        for (const Crosswalk& cw : junctionCrosswalks(sc.net, j)) painted.push_back(cw);
    int crossings = 0;
    for (const WalkLink& link : sim.walkLinks()) {
        if (!link.crossing) continue;
        ++crossings;
        bool matched = false;
        for (const Crosswalk& cw : painted)
            if (cw.road == link.road && std::fabs(cw.s - link.s) < 0.01) matched = true;
        check(matched, "every crossing link sits on a painted crosswalk");
    }
    check(crossings > 0, "the network has crossings");
    // Both ends of a crossing are the two footways of the SAME road.
    for (const WalkLink& link : sim.walkLinks()) {
        if (!link.crossing) continue;
        const WalkNode& a = sim.walkNodes()[size_t(link.a)];
        const WalkNode& b = sim.walkNodes()[size_t(link.b)];
        check(a.road == b.road && a.side == -b.side,
              "a crossing joins the two footways of one road");
    }

    sim.seedPedestrians(140);
    sim.seedVehicles(90);
    sim.run(140.0, 0.1);
    SimStats st = sim.stats();
    check(st.crossingsMade > 10, "pedestrians actually use the crossings");
    for (const Pedestrian& p : sim.pedestrians()) {
        Vec3 pos;
        double yaw = 0;
        if (!p.active) continue;
        check(sim.pedestrianPose(p, pos, yaw), "every pedestrian has a pose");
        check(std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z),
              "pedestrian poses are finite");
    }

    // Drivers give way. Any vehicle closing on an occupied crossing must be
    // slow — that is the whole content of "the crossing the driver stops for is
    // the crossing the shader painted".
    const std::vector<LaneNode>& nodes = sc.net.lanes().nodes;
    int checked = 0;
    for (const Pedestrian& p : sim.pedestrians()) {
        if (!p.active || p.crossingLink < 0) continue;
        const WalkLink& link = sim.walkLinks()[size_t(p.crossingLink)];
        for (const Vehicle& v : sim.vehicles()) {
            if (!v.active || v.lane < 0) continue;
            const LaneNode& n = nodes[size_t(v.lane)];
            if (n.ref.road != link.road) continue;
            double ahead = (link.s - n.roadS(v.travelled)) * (n.dir > 0 ? 1.0 : -1.0);
            if (ahead < 0.0 || ahead > 12.0) continue;
            ++checked;
            check(v.speed < 7.0, "a vehicle closing on an occupied crossing is slowing");
        }
    }
    (void)checked;
}

void testPedestrianSignalsAgreeWithDrivers() {
    group("crossing signals");
    Scene sc;
    buildDemo("lanes", sc);
    finalizeScene(sc, false, false);
    Simulation sim(sc.net, SimParams{});
    const Junction& j = sc.net.junction(0);
    check(j.control == JunctionControl::Signalized, "the demo junction is signalised");
    check(!j.phases.empty(), "it has phases");

    int link = -1;
    for (size_t i = 0; i < sim.walkLinks().size(); ++i)
        if (sim.walkLinks()[i].crossing && sim.walkLinks()[i].junction == j.id) link = int(i);
    check(link >= 0, "the signalised junction has a crossing");
    if (link < 0) return;
    int road = sim.walkLinks()[size_t(link)].road;

    // THE invariant: a pedestrian may step out exactly when no movement using
    // that arm's carriageway is green. Both sides read the same phase table, so
    // they cannot disagree — and this asserts it over a whole cycle.
    bool sawWalk = false, sawWait = false;
    double cycle = j.cycleLength();
    Simulation probe(sc.net, SimParams{});
    for (double t = 0; t < cycle; t += 0.25) {
        probe.run(0.25, 0.25);
        bool anyGreen = false;
        for (size_t ci = 0; ci < j.connections.size(); ++ci) {
            const Connection& c = j.connections[ci];
            if (c.from.road != road && c.to.road != road) continue;
            if (j.isGreen(int(ci), probe.time())) anyGreen = true;
        }
        bool mayCross = probe.pedestrianMayCross(link, 0.0);
        check(mayCross != anyGreen, "the crossing is walkable exactly when its arm is not green");
        sawWalk = sawWalk || mayCross;
        sawWait = sawWait || !mayCross;
    }
    check(sawWalk, "the crossing is walkable at some point in the cycle");
    check(sawWait, "and blocked at some point in the cycle");
}

// --- OpenDRIVE ------------------------------------------------------------

namespace {

// Value of the first `key="..."` at or after `from`, or NaN.
double attrAfter(const std::string& doc, const char* key, size_t from, size_t limit) {
    std::string needle = std::string(key) + "=\"";
    size_t at = doc.find(needle, from);
    if (at == std::string::npos || at > limit) return std::nan("");
    at += needle.size();
    return std::atof(doc.c_str() + at);
}

}  // namespace

void testPolyShift() {
    group("polynomial rebasing");
    // Re-basing a cubic must be exact, because every clipped profile in the
    // exporter depends on it.
    Poly3 p{1.5, -0.25, 0.03, -0.0007};
    for (double c : {0.0, 3.0, 41.7}) {
        Poly3 q = p.shifted(c);
        for (double u : {0.0, 1.0, 7.5, 30.0}) {
            checkNear(q.eval(u), p.eval(u + c), 1e-9, "shifted cubic matches the original");
            checkNear(q.deriv(u), p.deriv(u + c), 1e-9, "shifted derivative matches too");
        }
    }
}

void testOpenDriveExport() {
    group("OpenDRIVE export");
    for (const std::string& name : {std::string("lanes"), std::string("interchange"),
                                    std::string("roundabout"), std::string("grades")}) {
        Scene sc;
        buildDemo(name, sc);
        finalizeScene(sc, false, false);
        OdrOptions oo;
        oo.name = name;
        std::string doc = openDriveString(sc.net, oo);

        check(doc.find("<OpenDRIVE>") != std::string::npos, "the document has a root element");
        check(doc.find("</OpenDRIVE>") != std::string::npos, "and closes it");
        check(doc.find("revMajor=\"1\"") != std::string::npos, "it declares a version");
        check(doc.find("nan") == std::string::npos, "no NaNs leaked into the file");
        check(doc.find("inf") == std::string::npos, "no infinities either");

        // One <road> per exported road, and every one carries geometry.
        size_t roads = 0, planViews = 0;
        for (size_t at = doc.find("<road "); at != std::string::npos;
             at = doc.find("<road ", at + 1))
            ++roads;
        for (size_t at = doc.find("<planView>"); at != std::string::npos;
             at = doc.find("<planView>", at + 1))
            ++planViews;
        size_t expected = 0;
        for (const Road& r : sc.net.roads())
            if (r.activeLength() >= 1e-3) ++expected;
        check(roads == expected, "every road with length is exported");
        check(planViews == roads, "every road has a plan view");

        // THE structural invariant: the geometry pieces of a road must add up to
        // the road's declared length. A clipped clothoid getting its length or
        // its re-derived start pose wrong shows up here immediately.
        size_t at = doc.find("<road ");
        while (at != std::string::npos) {
            size_t headEnd = doc.find('>', at);
            double roadLen = attrAfter(doc, "length", at, headEnd);
            size_t pvStart = doc.find("<planView>", at);
            size_t pvEnd = doc.find("</planView>", at);
            check(pvStart != std::string::npos && pvEnd != std::string::npos,
                  "the road has a plan view");
            double sum = 0;
            for (size_t g = doc.find("<geometry ", pvStart); g != std::string::npos && g < pvEnd;
                 g = doc.find("<geometry ", g + 1)) {
                size_t gEnd = doc.find('>', g);
                sum += attrAfter(doc, "length", g, gEnd);
            }
            checkNear(sum, roadLen, 0.02, "geometry lengths sum to the road length");
            at = doc.find("<road ", pvEnd);
        }

        // Junction connections must name roads that exist in the file.
        for (size_t c = doc.find("<connection "); c != std::string::npos;
             c = doc.find("<connection ", c + 1)) {
            size_t cEnd = doc.find('>', c);
            double incoming = attrAfter(doc, "incomingRoad", c, cEnd);
            double connecting = attrAfter(doc, "connectingRoad", c, cEnd);
            check(incoming >= 0 && incoming < sc.net.roadCount(), "incomingRoad exists");
            check(connecting >= 0 && connecting < sc.net.roadCount(), "connectingRoad exists");
            check(sc.net.road(int(connecting)).junctionId >= 0,
                  "the connecting road is marked as a junction connector");
        }
    }
}

void testOpenDriveClipping() {
    group("OpenDRIVE clipping");
    // A junction-trimmed arm exports as a road starting at s = 0. The exported
    // start pose must be the pose at the TRIM, not at the original road start —
    // getting this wrong silently shifts every trimmed road in the file.
    Scene sc;
    buildDemo("grades", sc);
    finalizeScene(sc, false, false);
    std::string doc = openDriveString(sc.net);

    const Junction& j = sc.net.junction(0);
    check(!j.arms.empty(), "the demo has arms");
    for (const JunctionArm& arm : j.arms) {
        const Road& r = sc.net.road(arm.road);
        check(arm.trim > 0.0, "the arm really was trimmed");
        // Find this road's block and its first geometry.
        std::string needle = "id=\"" + std::to_string(r.id) + "\" junction=";
        size_t at = doc.find(needle);
        check(at != std::string::npos, "the trimmed arm is in the file");
        if (at == std::string::npos) continue;
        size_t g = doc.find("<geometry ", at);
        size_t gEnd = doc.find('>', g);
        double s0 = attrAfter(doc, "s", g, gEnd);
        double x = attrAfter(doc, "x", g, gEnd);
        double y = attrAfter(doc, "y", g, gEnd);
        Frame f = r.spine.frameAt(r.begin());
        checkNear(s0, 0.0, 1e-9, "an exported road starts at s = 0");
        checkNear(x, f.planPos.x, 1e-6, "the first geometry starts at the trimmed pose (x)");
        checkNear(y, f.planPos.y, 1e-6, "the first geometry starts at the trimmed pose (y)");
        // The declared length is the ACTIVE length, not the whole spine.
        size_t roadTag = doc.rfind("<road ", at);
        size_t headEnd = doc.find('>', roadTag);
        checkNear(attrAfter(doc, "length", roadTag, headEnd), r.activeLength(), 1e-6,
                  "the exported length is the active window, not the whole spine");
        check(r.activeLength() < r.spineLength() - 0.5,
              "which is genuinely shorter than the spine");
    }
}

// --- end to end -----------------------------------------------------------

void testAllDemosBuild() {
    group("demos");
    for (const std::string& name : demoNames()) {
        Scene sc;
        check(buildDemo(name, sc), (name + " builds").c_str());
        finalizeScene(sc, true, true);
        check(sc.net.roadCount() > 0, (name + " has roads").c_str());
        check(sc.mesh.triangleCount() > 100, (name + " tessellates").c_str());
        check(!sc.mesh.verts.empty(), (name + " has vertices").c_str());
        for (const Vertex& v : sc.mesh.verts) {
            if (!std::isfinite(v.pos.x) || !std::isfinite(v.pos.y) || !std::isfinite(v.pos.z)) {
                check(false, (name + " has finite vertex positions").c_str());
                break;
            }
        }
        if (!sc.lint.empty()) {
            std::printf("  note: %s lint: %s\n", name.c_str(), sc.lint.front().c_str());
        }
        check(sc.lint.empty(), (name + " passes its own design lint").c_str());
    }
}

void testGeneratedCity() {
    group("generated city");
    for (uint32_t seed : {1u, 7u, 23u}) {
        Scene sc;
        CityParams cp;
        cp.seed = seed;
        cp.blocksX = 3;
        cp.blocksZ = 3;
        generateCity(sc, cp);
        finalizeScene(sc, false, true);
        check(sc.net.roadCount() > 20, "the generator produced a network");
        check(sc.net.junctionCount() > 5, "it produced junctions");
        check(!sc.net.lanes().nodes.empty(), "it produced a lane graph");
        if (!sc.lint.empty())
            std::printf("  note: seed %u lint: %s\n", seed, sc.lint.front().c_str());
        check(sc.lint.empty(), "a generated city passes the design lint");
    }
    // Determinism: the same seed must give the same network.
    Scene a, b;
    CityParams cp;
    cp.seed = 5;
    cp.blocksX = 2;
    cp.blocksZ = 2;
    generateCity(a, cp);
    generateCity(b, cp);
    finalizeScene(a, false, false);
    finalizeScene(b, false, false);
    check(a.net.roadCount() == b.net.roadCount(), "generation is deterministic (roads)");
    check(a.mesh.verts.size() == b.mesh.verts.size(), "generation is deterministic (mesh)");
}

}  // namespace

int main() {
    std::printf("roadlab tests\n");
    testPoly();
    testSpine();
    testInverseMapping();
    testSuperelevation();
    testPresetsAndSections();
    testTapersAndTransitions();
    testMarkingsFollowGeometry();
    testMarkingLegalityMatchesPaint();
    testSplitAndLaneGraph();
    testJunctionAndSignals();
    testRoundabout();
    testRamps();
    testStructuresAndClearance();
    testTunnelKeepsTerrain();
    testPolygonUtilities();
    testGradedJunctionSurface();
    testPropsFollowSemantics();
    testSimulation();
    testRouting();
    testLaneChoiceForAnExit();
    testSignalsStopTraffic();
    testPedestrianGraph();
    testPedestrianSignalsAgreeWithDrivers();
    testPolyShift();
    testOpenDriveExport();
    testOpenDriveClipping();
    testAllDemosBuild();
    testGeneratedCity();

    std::printf("\n%d checks, %d failure(s)\n", gChecks, gFailures);
    return gFailures == 0 ? 0 : 1;
}
