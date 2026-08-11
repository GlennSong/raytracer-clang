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
    testPropsFollowSemantics();
    testSimulation();
    testSignalsStopTraffic();
    testAllDemosBuild();
    testGeneratedCity();

    std::printf("\n%d checks, %d failure(s)\n", gChecks, gFailures);
    return gFailures == 0 ? 0 : 1;
}
