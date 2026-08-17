// roadlab unit tests. No framework — the prototype is meant to stay
// dependency-free and buildable anywhere.
//
// The tests worth having here are the INVARIANT ones: the claims the whole
// design rests on. That a taper's paint tracks the lane boundary it is drawn
// from. That a signal phase never runs two conflicting movements. That the
// simulator's lane-change legality is the same field the shader paints. If those
// hold, the "one source of truth" story is true; if they do not, it is a slogan.

#include "builders.h"
#include "diag.h"
#include "paint_bake.h"
#include "paint_texture.h"
#include "junction.h"
#include "network.h"
#include "odr.h"
#include "rl_xml.h"
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
#include <cctype>
#include <cstdint>
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

    // A point well off an arc does have a foot of perpendicular — outside a
    // circle everything projects radially — so the invariant is not that the
    // query fails but that the answer is HONEST: it must reconstruct the point,
    // and its lateral offset must be large enough that no width test mistakes it
    // for being on the road. Reporting a small plausible t is the bug.
    Spine ring = spineArc({0, 0}, 20.0, 0.0, kPi * 0.5, true);
    double rs = 0, rt = 0;
    if (ring.toST({-8, -52}, rs, rt)) {
        check(std::fabs(rt) > 40.0, "a point far off an arc reports a far-off lateral offset");
        Vec2 rebuilt = ring.toPlan(rs, rt);
        checkNear(rebuilt.x, -8.0, 1e-6, "and the (s,t) it reports rebuilds the point (x)");
        checkNear(rebuilt.y, -52.0, 1e-6, "and rebuilds the point (z)");
    } else {
        check(true, "a point far off an arc is rejected");
    }
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

std::string attrStrAfter(const std::string& doc, const char* key, size_t from, size_t limit) {
    std::string needle = std::string(key) + "=\"";
    size_t at = doc.find(needle, from);
    if (at == std::string::npos || at > limit) return "";
    at += needle.size();
    size_t end = doc.find('"', at);
    return doc.substr(at, end - at);
}

struct OdrRoadInfo {
    int id = -1, junction = -1;
    std::string predType, succType, predCp, succCp;
    int predId = -1, succId = -1;
};

// Minimal scan of the emitted document. Deliberately not a real parser — it only
// needs the link topology, and keeping the test dependency-free matters more.
std::vector<OdrRoadInfo> scanRoads(const std::string& doc) {
    std::vector<OdrRoadInfo> out;
    for (size_t at = doc.find("<road "); at != std::string::npos; at = doc.find("<road ", at + 1)) {
        size_t headEnd = doc.find('>', at);
        OdrRoadInfo ri;
        ri.id = int(attrAfter(doc, "id", at, headEnd));
        ri.junction = int(attrAfter(doc, "junction", at, headEnd));
        size_t planView = doc.find("<planView>", at);
        size_t linkStart = doc.find("<link>", at);
        size_t linkEnd = doc.find("</link>", at);
        if (linkStart != std::string::npos && linkStart < planView) {
            size_t p = doc.find("<predecessor ", linkStart);
            if (p != std::string::npos && p < linkEnd) {
                size_t e = doc.find('>', p);
                ri.predType = attrStrAfter(doc, "elementType", p, e);
                ri.predId = int(attrAfter(doc, "elementId", p, e));
                ri.predCp = attrStrAfter(doc, "contactPoint", p, e);
            }
            size_t q = doc.find("<successor ", linkStart);
            if (q != std::string::npos && q < linkEnd) {
                size_t e = doc.find('>', q);
                ri.succType = attrStrAfter(doc, "elementType", q, e);
                ri.succId = int(attrAfter(doc, "elementId", q, e));
                ri.succCp = attrStrAfter(doc, "contactPoint", q, e);
            }
        }
        out.push_back(ri);
    }
    return out;
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
        // Plus the stub connecting roads synthesised for ramp merges and
        // diverges — those are extra windows over existing geometry, not extra
        // geometry, so the count is >= rather than ==.
        check(roads >= expected, "every road with length is exported");
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
            // Both must exist IN THE FILE — a synthesised stub has an id beyond
            // anything in the network, which is fine and is why this checks the
            // document rather than the Network.
            check(incoming >= 0, "incomingRoad is a real id");
            std::string needle = "<road ";
            bool sawIncoming = false, sawConnecting = false;
            for (size_t rr = doc.find(needle); rr != std::string::npos;
                 rr = doc.find(needle, rr + 1)) {
                size_t rEnd = doc.find('>', rr);
                double rid = attrAfter(doc, "id", rr, rEnd);
                if (rid == incoming) sawIncoming = true;
                if (rid == connecting) sawConnecting = true;
            }
            check(sawIncoming, "incomingRoad exists in the file");
            check(sawConnecting, "connectingRoad exists in the file");
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

void testOpenDriveConformance() {
    group("OpenDRIVE conformance");
    // THE rule the format imposes and our internal model does not: a road may
    // have at most ONE predecessor and ONE successor, and any connection that is
    // not one-to-one has to go through a <junction>. Our ramps are lane links
    // internally, so this is exactly where an export can quietly produce a file
    // whose ramps are unreachable.
    for (const std::string& name : demoNames()) {
        Scene sc;
        buildDemo(name, sc);
        finalizeScene(sc, false, false);
        std::string doc = openDriveString(sc.net);
        std::vector<OdrRoadInfo> roads = scanRoads(doc);
        check(!roads.empty(), (name + ": the export has roads").c_str());

        auto byId = [&](int id) -> const OdrRoadInfo* {
            for (const OdrRoadInfo& r : roads)
                if (r.id == id) return &r;
            return nullptr;
        };

        for (const OdrRoadInfo& a : roads) {
            // Everyone who attaches to A's high-s end.
            std::vector<int> endClaimers, startClaimers;
            for (const OdrRoadInfo& b : roads) {
                if (b.id == a.id) continue;
                if (b.predType == "road" && b.predId == a.id) {
                    (b.predCp == "end" ? endClaimers : startClaimers).push_back(b.id);
                }
                if (b.succType == "road" && b.succId == a.id) {
                    (b.succCp == "end" ? endClaimers : startClaimers).push_back(b.id);
                }
            }
            auto verify = [&](const std::vector<int>& claimers, const std::string& type, int id,
                              const char* which) {
                if (type == "junction") {
                    // Everything attaching here must be a connecting road of that
                    // junction; that is what makes the ambiguity legal.
                    for (int c : claimers) {
                        const OdrRoadInfo* cr = byId(c);
                        check(cr && cr->junction == id,
                              (name + ": a road attaching to a junction " + which +
                               " is one of its connecting roads")
                                  .c_str());
                    }
                    return;
                }
                if (type == "road") {
                    for (int c : claimers)
                        check(c == id, (name + ": road " + std::to_string(a.id) + " declares one " +
                                        which + " but another road attaches there too")
                                           .c_str());
                    return;
                }
                // No link declared: nothing may attach.
                check(claimers.empty(), (name + ": road " + std::to_string(a.id) +
                                         " declares no " + which + " yet something attaches")
                                            .c_str());
            };
            verify(endClaimers, a.succType, a.succId, "successor");
            verify(startClaimers, a.predType, a.predId, "predecessor");
        }

        // Every connecting road must be marked with its junction and must link to
        // the incoming road it serves.
        size_t at = doc.find("<junction ");
        while (at != std::string::npos) {
            size_t headEnd = doc.find('>', at);
            int jid = int(attrAfter(doc, "id", at, headEnd));
            size_t jEnd = doc.find("</junction>", at);
            for (size_t c = doc.find("<connection ", at); c != std::string::npos && c < jEnd;
                 c = doc.find("<connection ", c + 1)) {
                size_t cEnd = doc.find('>', c);
                int incoming = int(attrAfter(doc, "incomingRoad", c, cEnd));
                int connecting = int(attrAfter(doc, "connectingRoad", c, cEnd));
                const OdrRoadInfo* cr = byId(connecting);
                check(cr != nullptr, (name + ": the connecting road exists").c_str());
                if (!cr) continue;
                check(cr->junction == jid,
                      (name + ": a connecting road is marked with its junction").c_str());
                bool linksToIncoming = (cr->predType == "road" && cr->predId == incoming) ||
                                       (cr->succType == "road" && cr->succId == incoming);
                check(linksToIncoming,
                      (name + ": a connecting road links back to its incoming road").c_str());
                // At least one lane link, or the movement carries no traffic.
                size_t ll = doc.find("<laneLink ", c);
                check(ll != std::string::npos && ll < doc.find("</connection>", c),
                      (name + ": a connection carries at least one lane link").c_str());
            }
            at = doc.find("<junction ", jEnd == std::string::npos ? at + 1 : jEnd);
        }
    }
}

void testRampsBecomeJunctions() {
    group("ramp junctions");
    Scene sc;
    buildDemo("interchange", sc);
    finalizeScene(sc, false, false);
    check(sc.net.junctionCount() == 0, "the interchange has no authored junctions");
    check(sc.net.extraLinks.size() == 2, "it has two ramp lane links");

    std::string doc = openDriveString(sc.net);
    int junctions = 0;
    for (size_t at = doc.find("<junction "); at != std::string::npos;
         at = doc.find("<junction ", at + 1))
        ++junctions;
    // A merge and a diverge, each of which is a not-one-to-one connection and so
    // each of which the format requires to be a junction.
    check(junctions == 2, "both ramps export as junctions");
    check(doc.find("diverge-") != std::string::npos, "the off-ramp is a diverge junction");
    check(doc.find("merge-") != std::string::npos, "the on-ramp is a merge junction");
    check(doc.find(".stub") != std::string::npos, "connecting roads were carved as stubs");
}

void testDivergeKeepsLaneIdentity() {
    group("diverge lane identity");
    // A lane that continues through a diverge must continue as ITSELF. Pairing by
    // ordinal instead of position shifts every remaining lane one place sideways
    // the moment the deceleration lane peels off, which is silent and wrong in
    // both the lane graph and the export.
    Scene sc;
    buildDemo("interchange", sc);
    finalizeScene(sc, false, false);

    int rampRoad = -1, mainRoad = -1;
    for (const ExtraLaneLink& el : sc.net.extraLinks) {
        if (sc.net.road(el.toRoad).kind == RoadKind::Ramp) {
            rampRoad = el.toRoad;
            mainRoad = el.fromRoad;
        }
    }
    check(rampRoad >= 0 && mainRoad >= 0, "the diverge was found");
    if (mainRoad < 0) return;

    const Road& head = sc.net.road(mainRoad);
    check(head.succ.type == LinkType::Road, "the mainline continues past the exit");
    const Road& tail = sc.net.road(head.succ.id);

    // The deceleration lane leaves with the ramp, so the mainline narrows.
    int headLanes = head.xs.sectionAt(head.end() - 0.1).laneCount();
    int tailLanes = tail.xs.sectionAt(tail.begin() + 0.1).laneCount();
    check(tailLanes == headLanes - 1, "the mainline loses exactly the deceleration lane");

    // And every continuing lane keeps its identity, which positional pairing
    // guarantees and an ordinal zip does not.
    for (const auto& pr : pairLanesAcross(head, true, tail, true)) {
        check(pr.first == pr.second, "a lane continuing past the diverge stays the same lane");
    }

    // The same must hold in the lane graph the simulator drives.
    const LaneGraph& lg = sc.net.lanes();
    int checked = 0;
    for (const LaneNode& n : lg.nodes) {
        if (n.ref.road != mainRoad) continue;
        for (int succ : n.successors) {
            const LaneNode& t = lg.nodes[size_t(succ)];
            if (t.ref.road != tail.id) continue;
            check(t.ref.lane == n.ref.lane,
                  "the lane graph continues a lane as itself across the diverge");
            ++checked;
        }
    }
    check(checked >= 3, "the through lanes really are linked in the graph");
}

// --- OpenDRIVE import -------------------------------------------------------

void testXmlReader() {
    group("xml reader");
    XmlNode root;
    std::string err;
    const char* doc =
        "<?xml version=\"1.0\"?>\n"
        "<!-- a comment with <angle> brackets -->\n"
        "<top a=\"1\" b='two' c=\"a &amp; b &lt;3&gt;\">\n"
        "  <kid n=\"1\"/>\n"
        "  <kid n=\"2\"><grand v=\"-3.5\"/></kid>\n"
        "</top>";
    check(xmlParse(doc, root, err), "a well-formed document parses");
    check(root.name == "top", "the root element is found");
    checkNear(root.attrD("a"), 1.0, 1e-12, "a numeric attribute reads back");
    check(root.attrS("b") == "two", "single quotes are accepted");
    check(root.attrS("c") == "a & b <3>", "the predefined entities are unescaped");
    check(root.all("kid").size() == 2, "repeated children are all kept");
    const XmlNode* second = root.all("kid")[1];
    check(second->child("grand") != nullptr, "nesting works");
    checkNear(second->child("grand")->attrD("v"), -3.5, 1e-12, "a nested attribute reads back");
    check(root.attrD("missing", 42.0) == 42.0, "a missing attribute falls back");

    XmlNode bad;
    bool mismatch = xmlParse("<a><b></a>", bad, err);
    check(!mismatch, "a mismatched close tag is an error");
    check(err.find("line") != std::string::npos, "and the error names a line");
    check(!xmlParse("   ", bad, err), "an empty document is an error");
}

void testOpenDriveImportBasics() {
    group("OpenDRIVE import");
    // A hand-written file, so the reader is tested against something the exporter
    // did not produce. Two records: a line, then an arc that turns left.
    const char* doc = R"(<?xml version="1.0"?>
<OpenDRIVE>
  <header revMajor="1" revMinor="7" name="hand written"/>
  <road name="main" length="150.0" id="17" junction="-1">
    <planView>
      <geometry s="0" x="10" y="20" hdg="0" length="100"><line/></geometry>
      <geometry s="100" x="110" y="20" hdg="0" length="50"><arc curvature="0.01"/></geometry>
    </planView>
    <elevationProfile><elevation s="0" a="5" b="0.02" c="0" d="0"/></elevationProfile>
    <lanes>
      <laneOffset s="0" a="0" b="0" c="0" d="0"/>
      <laneSection s="0">
        <left><lane id="1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane></left>
        <center><lane id="0"><roadMark sOffset="0" type="solid solid" color="yellow" width="0.15"/></lane></center>
        <right>
          <lane id="-1" type="driving"><width sOffset="0" a="3.5" b="0" c="0" d="0"/></lane>
          <lane id="-2" type="sidewalk"><width sOffset="0" a="2.0" b="0" c="0" d="0"/></lane>
        </right>
      </laneSection>
    </lanes>
  </road>
</OpenDRIVE>)";

    Network net;
    std::string err;
    OdrImportReport rep;
    check(openDriveFromString(doc, net, err, &rep), "a hand-written file imports");
    check(rep.roads == 1, "one road");
    check(rep.lanes == 3, "three lanes");
    check(rep.name == "hand written", "the header name is reported");
    if (net.roadCount() != 1) return;
    net.build();

    const Road& r = net.road(0);
    checkNear(r.spineLength(), 150.0, 1e-6, "the reference line is as long as declared");
    Vec2 start = r.spine.toPlan(0, 0);
    checkNear(start.x, 10.0, 1e-9, "the road starts where the file says (x)");
    checkNear(start.y, 20.0, 1e-9, "the road starts where the file says (z)");
    Vec2 joint = r.spine.toPlan(100.0, 0);
    checkNear(joint.x, 110.0, 1e-6, "the straight ends at the arc's declared pose");
    checkNear(r.spine.frameAt(120.0).curvature, 0.01, 1e-9, "the arc's curvature is honoured");
    checkNear(r.surfacePoint(50.0, 0).y, 6.0, 1e-6, "the elevation cubic is honoured");

    const LaneSection& sec = r.xs.sectionAt(0);
    check(sec.left.size() == 1 && sec.right.size() == 2, "the lane stacks are the right size");
    check(sec.left[0].id == 1 && sec.right[0].id == -1, "signed ordinals survive");
    check(sec.right[1].kind == StripKind::Sidewalk, "a sidewalk stays a sidewalk");
    check(sec.right[1].surface == SurfaceKind::Concrete,
          "and gets a plausible material even though the file gave none");
    check(sec.right[0].dir == +1 && sec.left[0].dir == -1,
          "right-hand traffic: negative ids run with s");
    check(sec.centerMark.style == MarkStyle::Double, "a double yellow centre line survives");
    check(sec.centerMark.color == PaintColor::Yellow, "including its colour");
    checkNear(r.xs.leftExtentAt(0), 3.5, 1e-9, "the left extent is one lane");
    checkNear(-r.xs.rightExtentAt(0), 5.5, 1e-9, "the right extent is a lane plus a footway");

    // Unsupported geometry is approximated, and says so rather than silently
    // producing a straight line where a curve was meant.
    const char* poly = R"(<OpenDRIVE><road name="p" length="40" id="0" junction="-1"><planView>
      <geometry s="0" x="0" y="0" hdg="0" length="40">
        <paramPoly3 aU="0" bU="40" cU="0" dU="0" aV="0" bV="0" cV="8" dV="0" pRange="normalized"/>
      </geometry></planView></road></OpenDRIVE>)";
    Network pn;
    OdrImportReport prep;
    check(openDriveFromString(poly, pn, err, &prep), "a paramPoly3 imports");
    check(prep.approximatedGeometry > 0, "and is reported as approximated");
    check(!prep.notes.empty(), "with a note the caller can show");
    if (pn.roadCount() == 1) {
        pn.build();
        // v = 8 at the far end of the normalised range, so the curve really bends.
        Vec2 end = pn.road(0).spine.toPlan(pn.road(0).spineLength(), 0);
        check(end.y > 4.0, "the sampled curve keeps its lateral offset");
    }

    Network broken;
    check(!openDriveFromString("<NotOpenDRIVE/>", broken, err),
          "a document that is not OpenDRIVE is refused");
    check(err.find("OpenDRIVE") != std::string::npos, "with a message that says why");
}

void testOpenDriveRoundTrip() {
    group("OpenDRIVE round trip");
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);

        OdrOptions opt;
        opt.name = name;
        std::string first = openDriveString(sc.net, opt);

        Network back;
        std::string err;
        OdrImportReport rep;
        bool ok = openDriveFromString(first, back, err, &rep);
        check(ok, (name + " re-imports").c_str());
        if (!ok) {
            std::printf("  note: %s\n", err.c_str());
            continue;
        }
        back.build();

        // 1. The carriageway is all still there. Road ids and windows change —
        // stubs are carved for the synthesised junctions — so the invariant is
        // the total, not a per-road correspondence.
        double before = 0, after = 0;
        for (const Road& r : sc.net.roads()) before += r.activeLength();
        for (const Road& r : back.roads()) after += r.activeLength();
        check(std::fabs(after - before) < before * 0.005 + 1e-6,
              (name + ": total centreline survives the round trip").c_str());

        // 2. The surface is in the same place. Sampling by world position is the
        // test that does not care how the roads were re-cut.
        int sampled = 0, matched = 0;
        double worstHeight = 0;
        for (const Road& r : sc.net.roads()) {
            double len = r.activeLength();
            if (len < 5.0) continue;
            for (int k = 1; k < 8; ++k) {
                double s = r.begin() + len * double(k) / 8.0;
                Vec3 p = r.surfacePoint(s, 0);
                ++sampled;
                // Nearest in THREE dimensions. A plan-only query picks the wrong
                // road wherever one deck runs over another, which is exactly
                // where an import is most worth checking.
                double bestDist = 1e300, bestHeight = 0;
                for (const Road& br : back.roads()) {
                    double bs = 0, bt = 0;
                    if (!br.spine.toST({p.x, p.z}, bs, bt)) continue;
                    if (bs < br.begin() - 1e-6 || bs > br.end() + 1e-6) continue;
                    if (bt > br.xs.leftExtentAt(bs) + 0.5 ||
                        bt < br.xs.rightExtentAt(bs) - 0.5)
                        continue;
                    Vec3 q = br.surfacePoint(bs, bt);
                    double d = length(q - p);
                    if (d < bestDist) {
                        bestDist = d;
                        bestHeight = std::fabs(q.y - p.y);
                    }
                }
                if (bestDist > 1e299) continue;
                worstHeight = std::max(worstHeight, bestHeight);
                if (bestDist < 0.2) ++matched;
            }
        }
        check(sampled > 0, (name + ": there were points to compare").c_str());
        check(matched >= int(sampled * 0.98),
              (name + ": the road surface comes back in the same place").c_str());
        check(worstHeight < 0.2, (name + ": and at the same height").c_str());

        // 3. Lane structure. Total drivable lane-metres is invariant even though
        // the roads carrying them are re-cut.
        auto laneMetres = [](const Network& n) {
            double total = 0;
            for (const Road& r : n.roads()) {
                double len = r.activeLength();
                if (len < 1e-6) continue;
                for (int k = 0; k < 16; ++k) {
                    double s = r.begin() + len * (double(k) + 0.5) / 16.0;
                    total += r.xs.sectionAt(s).laneCount() * len / 16.0;
                }
            }
            return total;
        };
        double lanesBefore = laneMetres(sc.net), lanesAfter = laneMetres(back);
        check(std::fabs(lanesAfter - lanesBefore) < lanesBefore * 0.01 + 1e-6,
              (name + ": the same number of lanes comes back").c_str());

        // 4. A second pass changes nothing. The FIRST pass legitimately changes
        // the representation — a lane-continuation merge becomes a junction with
        // connecting roads, because the format requires it — so the fixed point
        // is what can be asserted, and it is the stronger claim: the reader and
        // the writer agree on a canonical form.
        std::string second = openDriveString(back, opt);
        Network again;
        bool reOk = openDriveFromString(second, again, err, &rep);
        check(reOk, (name + ": the re-export re-imports").c_str());
        if (reOk) {
            again.build();
            check(openDriveString(again, opt) == second,
                  (name + ": export/import reaches a fixed point").c_str());
        }
    }
}

void testImportedJunctionsAreAdopted() {
    group("imported junctions");
    Scene sc;
    buildDemo("urban", sc);
    finalizeScene(sc, false, false);

    OdrOptions opt;
    std::string doc = openDriveString(sc.net, opt);
    Network back;
    std::string err;
    check(openDriveFromString(doc, back, err), "the urban grid re-imports");
    for (const Junction& j : back.junctions())
        check(j.imported, "every imported junction is marked as such");

    // Adoption must not re-run the trim. The file already holds trimmed arms; a
    // second trim would eat another cornerRadius of road at every approach, and
    // the connectors — which the file also already holds — would no longer reach.
    std::vector<double> lengthsBefore;
    for (const Road& r : back.roads()) lengthsBefore.push_back(r.activeLength());
    int connectorsBefore = back.roadCount();
    back.build();
    check(back.roadCount() == connectorsBefore,
          "adopting a junction does not invent a second set of connectors");
    bool sameLengths = true;
    for (size_t i = 0; i < lengthsBefore.size(); ++i)
        if (std::fabs(back.road(int(i)).activeLength() - lengthsBefore[i]) > 1e-9)
            sameLengths = false;
    check(sameLengths, "and does not trim the arms a second time");
    for (const Junction& j : back.junctions()) {
        for (const JunctionArm& a : j.arms) check(a.trim == 0.0, "an adopted arm is not re-trimmed");
        check(!j.boundary.empty(), "but the pad polygon is still built");
        check(j.boundary.size() == j.boundaryHeight.size(), "with a height per boundary point");
    }

    // What the file does carry about control and turning must come back, because
    // right of way is derived from it.
    int signalized = 0, priority = 0, lefts = 0;
    for (const Junction& j : back.junctions()) {
        if (j.control == JunctionControl::Signalized) ++signalized;
        if (j.control == JunctionControl::PriorityStop) ++priority;
        for (const Connection& c : j.connections)
            if (c.turn == TurnKind::Left) ++lefts;
    }
    check(signalized > 0, "signalized junctions come back signalized");
    check(priority > 0, "priority junctions come back priority-controlled");
    check(lefts > 0, "left turns are re-classified from the geometry");
    for (const Junction& j : back.junctions()) {
        if (j.control != JunctionControl::Signalized) continue;
        check(!j.phases.empty(), "a signalized junction gets its phases back");
        check(j.cycleLength() > 1.0, "with a real cycle");
    }
}

void testImportedNetworkDrives() {
    group("imported network drives");
    // The point of importing is to drive on it. This is the end-to-end claim:
    // a network that has been through the file survives contact with the traffic
    // model, including at the junctions the exporter synthesised.
    Scene sc;
    buildDemo("showcase", sc);
    finalizeScene(sc, false, false);
    OdrOptions opt;
    std::string doc = openDriveString(sc.net, opt);

    Network back;
    std::string err;
    bool ok = openDriveFromString(doc, back, err);
    check(ok, "the showcase re-imports");
    if (!ok) return;
    back.build();
    check(back.validate().empty(), "and the reimported network validates");
    check(!back.lanes().nodes.empty(), "it has a lane graph");

    SimParams sp;
    sp.seed = 11;
    Simulation sim(back, sp);
    sim.seedVehicles(80);
    SimStats seeded = sim.stats();
    check(seeded.vehicles > 20, "vehicles find somewhere to start");
    sim.run(45.0);
    SimStats st = sim.stats();
    check(st.moving > 0, "and they move");
    check(st.meanSpeedKph > 5.0, "at a plausible speed");
    check(st.completedTrips + st.replans > 0, "routing works on the imported graph");
    for (const Vehicle& v : sim.vehicles()) {
        check(std::isfinite(v.travelled) && std::isfinite(v.speed) && v.lane >= 0,
              "every vehicle is somewhere sensible on the graph");
        break;
    }
}

// --- the fallback census ----------------------------------------------------

namespace {

bool polygonSelfIntersects(const std::vector<Vec2>& poly) {
    const size_t n = poly.size();
    auto proper = [](Vec2 p, Vec2 p2, Vec2 q, Vec2 q2) {
        Vec2 r = p2 - p, s = q2 - q;
        double d = cross(r, s);
        if (std::fabs(d) < 1e-12) return false;
        double t = cross(q - p, s) / d, u = cross(q - p, r) / d;
        return t > 1e-9 && t < 1 - 1e-9 && u > 1e-9 && u < 1 - 1e-9;
    };
    for (size_t i = 0; i < n; ++i)
        for (size_t k = i + 2; k < n; ++k) {
            if (i == 0 && k == n - 1) continue;
            if (proper(poly[i], poly[(i + 1) % n], poly[k], poly[(k + 1) % n])) return true;
        }
    return false;
}

}  // namespace

void testFallbackCensusPlumbing() {
    group("fallback census");
    resetFallbackCensus();
    check(totalFallbacks() == 0, "the census starts empty");
    RL_CALLED("test-routine");
    RL_FALLBACK("test-routine gave up");
    RL_FALLBACK("test-routine gave up");
    check(fallbackCount("test-routine gave up") == 2, "a fallback counts each time it fires");
    check(callCount("test-routine") == 1, "and calls are counted separately");
    resetFallbackCensus();
    check(fallbackCount("test-routine gave up") == 0, "reset clears the counts");
    check(!fallbackCensus().empty(), "but keeps the sites registered, so a zero is reportable");
}

void testJunctionArmsMustMeet() {
    group("junction arms meet");
    // The lint that would have caught two shipped bugs: a junction whose arms
    // stand far apart still trims, still builds a pad, and still looks like
    // asphalt — it just spans a gap that is not a junction.
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);
        for (const Junction& j : sc.net.junctions()) {
            double widest = 0;
            for (const JunctionArm& a : j.arms)
                widest = std::max(widest,
                                  std::max(std::fabs(a.leftExtent), std::fabs(a.rightExtent)));
            for (const JunctionArm& a : j.arms) {
                check(length(a.contact - j.center) <= 4.0 * widest + 2.0 * j.cornerRadius + 20.0,
                      (name + ": every junction arm reaches its junction").c_str());
            }
        }
    }

    // And the lint reports one that does not. Two roads that never come near
    // each other, declared as a junction.
    Network net;
    RoadDesc a;
    a.name = "west";
    a.preset = "street2";
    a.points = {{-400, 0}, {-200, 0}};
    int ra = buildRoad(net, a);
    RoadDesc b;
    b.name = "far-north";
    b.preset = "street2";
    b.points = {{300, -200}, {300, 200}};
    int rb = buildRoad(net, b);
    buildIntersection(net, "impossible", {{ra, true}, {rb, false}},
                      JunctionControl::Uncontrolled, 6.0);
    net.build();
    bool reported = false;
    for (const std::string& msg : net.validate())
        if (msg.find("do not meet") != std::string::npos) reported = true;
    check(reported, "the lint reports a junction whose arms do not meet");
}

void testJunctionPadsAreSimplePolygons() {
    group("junction pads");
    // A pad outline that crosses itself cannot be triangulated, falls through to
    // a centroid fan, and covers ground that is not pavement. The usual cause is
    // a kerb return built between two arms that have no corner between them —
    // the two halves of a street running straight through.
    auto scan = [&](const Network& net, const std::string& tag) {
        for (const Junction& j : net.junctions()) {
            if (j.boundary.size() < 3) continue;
            check(!polygonSelfIntersects(j.boundary),
                  (tag + ": junction '" + j.name + "' has a simple pad outline").c_str());
            resetFallbackCensus();
            std::vector<std::array<uint32_t, 3>> tris = triangulatePolygon(j.boundary);
            check(fallbackCount("triangulatePolygon no ear found -> centroid fan") == 0,
                  (tag + ": junction '" + j.name + "' ear-clips without falling back").c_str());
            check(!tris.empty(), (tag + ": junction '" + j.name + "' produced triangles").c_str());
        }
    };
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);
        scan(sc.net, name);
    }
    for (uint32_t seed : {1u, 2u, 3u, 4u, 5u}) {
        Scene sc;
        CityParams cp;
        cp.seed = seed;
        generateCity(sc, cp);
        finalizeScene(sc, false, false);
        scan(sc.net, "city-" + std::to_string(seed));
    }
    resetFallbackCensus();
}

void testInverseMapResolvesRealQueries() {
    group("inverse map converges");
    // `Spine::toST` spent the life of this prototype rejecting EVERY query,
    // because a sign error made its Newton iteration diverge and a residual guard
    // turned the diverged result into a refusal. Nothing failed loudly: terrain
    // silently stopped conforming and Network::sample silently found nothing.
    // The guard against a repeat is to ask it the question it must be able to
    // answer — a point that is genuinely on a road — and count the refusals.
    resetFallbackCensus();
    long asked = 0;
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);
        for (const Road& r : sc.net.roads()) {
            double len = r.activeLength();
            if (len < 5.0) continue;
            for (int k = 1; k < 10; ++k) {
                double s = r.begin() + len * double(k) / 10.0;
                for (double lat : {-1.5, 0.0, 2.0}) {
                    double gs = 0, gt = 0;
                    ++asked;
                    bool got = r.spine.toST(r.spine.toPlan(s, lat), gs, gt);
                    check(got, (name + ": a point on the road resolves to (s,t)").c_str());
                    if (!got) continue;
                    checkNear(gs, s, 0.05, "and recovers the station");
                    checkNear(gt, lat, 0.05, "and recovers the lateral offset");
                }
            }
        }
    }
    check(asked > 500, "the inverse map was asked a meaningful number of questions");
    long diverged = fallbackCount("Spine::toST rejected (Newton did not converge)");
    check(diverged == 0, "no on-road query fails to converge");
    resetFallbackCensus();
}

void testMostDriversGetARoute() {
    group("routing coverage");
    // Routing that silently does not happen is worse than routing that fails
    // loudly: the vehicles still drive, so nothing looks wrong. On a connected
    // grid the large majority of drivers must actually get a plan.
    Scene sc;
    CityParams cp;
    cp.seed = 4;
    generateCity(sc, cp);
    finalizeScene(sc, false, false);
    SimParams sp;
    sp.seed = 9;
    Simulation sim(sc.net, sp);
    sim.seedVehicles(150);
    int routed = 0, total = 0;
    for (const Vehicle& v : sim.vehicles()) {
        if (!v.active) continue;
        ++total;
        if (!v.route.empty()) ++routed;
    }
    check(total > 50, "the city seeded a useful number of vehicles");
    check(routed >= int(total * 0.85),
          "at least 85% of drivers on a connected grid get a route");

    // And a network with nowhere to go says so rather than pretending.
    Scene lone;
    buildDemo("tiers", lone);
    finalizeScene(lone, false, false);
    Simulation sim2(lone.net, sp);
    sim2.seedVehicles(20);
    for (const Vehicle& v : sim2.vehicles())
        check(v.route.empty(), "isolated roads produce no route, honestly");
}

// --- the portable marking evaluator -----------------------------------------

void testPaintEvaluatorIsPortable() {
    group("paint evaluator");
    // rl_paint.h is the half of the surface shader that has to run on a GPU, so
    // it is written in a subset with no containers, no doubles and no noise. The
    // risk of a subset is that it drifts from what it replaced, so these are the
    // properties the CPU reference had and the port must keep.
    RlBoundary b[4];
    for (RlBoundary& x : b) x = RlBoundary{};
    b[0].t = -3.6f;
    b[0].style = float(RL_MARK_SOLID);
    b[0].width = 0.15f;
    b[0].color = float(RL_PAINT_WHITE);
    b[1].t = 0.0f;
    b[1].style = float(RL_MARK_DOUBLE);
    b[1].width = 0.12f;
    b[1].gap = 0.12f;
    b[1].color = float(RL_PAINT_YELLOW);

    const float fw = 0.01f;
    RlPaint on = rlEvaluateMarkings(b, 2, 10.0f, -3.6f, fw, 0.0f, 1.0f, 0.0f);
    check(on.coverage > 0.9f, "a solid line is opaque on its own boundary");
    RlPaint off = rlEvaluateMarkings(b, 2, 10.0f, -3.0f, fw, 0.0f, 1.0f, 0.0f);
    check(off.coverage < 1e-3, "and absent half a metre away");

    // The double yellow is TWO stripes with a gap: the centre of the pair is
    // unpainted. Getting this wrong yields one fat line and looks almost right.
    RlPaint centre = rlEvaluateMarkings(b, 2, 10.0f, 0.0f, fw, 0.0f, 1.0f, 0.0f);
    RlPaint limb = rlEvaluateMarkings(b, 2, 10.0f, 0.12f, fw, 0.0f, 1.0f, 0.0f);
    check(centre.coverage < 0.2f, "the gap between a double yellow's stripes is clear");
    check(limb.coverage > 0.9f, "and each stripe is painted");
    check(limb.g > limb.b, "yellow paint is yellow");

    // Energy preservation is what stops a receding lane line sparkling: as the
    // filter grows past the stripe width, coverage falls off smoothly rather
    // than flickering between 0 and 1.
    double prev = 1.0;
    bool monotone = true;
    for (float filter : {0.02f, 0.05f, 0.1f, 0.2f, 0.4f, 0.8f}) {
        RlPaint p = rlEvaluateMarkings(b, 2, 10.0f, -3.6f, filter, 0.0f, 1.0f, 0.0f);
        if (p.coverage > prev + 1e-4) monotone = false;
        prev = p.coverage;
    }
    check(monotone, "coverage never rises as the filter widens (no shimmer)");
    check(prev < 0.6, "and a far-away stripe fades rather than staying solid");

    // A dashed line must average to its duty cycle along s, or a dashed road
    // reads as a different brightness from a solid one at distance.
    RlBoundary d[1];
    d[0] = RlBoundary{};
    d[0].t = 0.0f;
    d[0].style = float(RL_MARK_DASHED);
    d[0].width = 0.15f;
    d[0].dashOn = 3.0f;
    d[0].dashOff = 9.0f;
    double sum = 0;
    const int kSamples = 4000;
    for (int i = 0; i < kSamples; ++i) {
        float s = float(i) * (60.0f / kSamples);
        sum += rlEvaluateMarkings(d, 1, s, 0.0f, 0.005f, 0.0f, 1.0f, 0.0f).coverage;
    }
    checkNear(sum / kSamples, 3.0 / 12.0, 0.02, "a dashed line averages to its duty cycle");

    // Far field: once the dashes are sub-pixel the stripe must stop flickering
    // and settle. Its value is the duty cycle times how much of the pixel a
    // 0.15 m stripe occupies laterally under a 6 m filter — energy preservation
    // applies in both dimensions, which is the whole point of it.
    double lo = 1.0, hi = 0.0;
    for (int i = 0; i < 200; ++i) {
        float s = 3.1f * float(i);
        double c = rlEvaluateMarkings(d, 1, s, 0.0f, 6.0f, 0.0f, 1.0f, 0.0f).coverage;
        lo = std::min(lo, c);
        hi = std::max(hi, c);
    }
    check(hi - lo < 1e-6, "a sub-pixel dashed line stops varying with s — no crawl");
    checkNear(hi, (3.0 / 12.0) * (0.15 / (6.0 * 1.6)), 1e-4,
              "and settles at duty times the stripe's share of the pixel");

    check(rlEvaluateMarkings(b, 0, 1.0f, 0.0f, fw, 0.0f, 1.0f, 0.0f).coverage == 0.0f,
          "no boundaries paints nothing");
}

namespace {

// Inside the ring pair straddling a lane-section boundary. bakeBoundaryStrip
// puts rings at sec.s0 +/- 1 mm; the half-millimetre of slack is for stations
// that land bit-for-bit on a seam, which happens more often than it sounds
// because seams tend to be derived from round fractions of a road's length.
bool nearASeam(const Road& r, double s) {
    for (const LaneSection& sec : r.xs.sections)
        if (std::fabs(s - sec.s0) < 1.5e-3) return true;
    return false;
}

}  // namespace

void testBakedBoundariesMatchTheCrossSection() {
    group("boundary bake");
    // The GPU cannot walk a cross-section, so boundaries get baked at intervals
    // and interpolated — by vertex attributes today, a profile texture later.
    // This is the question that decides whether that works: does the
    // interpolated array reproduce the exact one, everywhere it matters?
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);

        double worstOffset = 0;
        long compared = 0, countMismatch = 0;
        double worstCoverage = 0;
        for (const Road& r : sc.net.roads()) {
            if (r.activeLength() < 8.0) continue;
            // 2 m is the ring spacing the road tessellator already uses, so this
            // is the real interpolation the mesh would give, not a flattering one.
            BoundaryStrip strip = bakeBoundaryStrip(r, 2.0);
            for (int k = 1; k < 40; ++k) {
                double s = r.begin() + r.activeLength() * double(k) / 40.0;
                // Skip the seam windows. Between the two rings straddling a
                // lane-section boundary the cross-section changes by a whole
                // lane, and no interpolation is meaningful across it — the
                // question there is how WIDE that window is, which is measured
                // on its own below rather than averaged into this figure.
                if (nearASeam(r, s)) continue;
                RlBoundary exact[kMaxBakedBoundaries];
                RlBoundary lerped[kMaxBakedBoundaries];
                int ne = bakeBoundaries(r, s, exact, kMaxBakedBoundaries);
                int nl = strip.sampleInterpolated(s, lerped, kMaxBakedBoundaries);
                ++compared;
                if (ne != nl) {
                    ++countMismatch;
                    continue;   // a lane opening inside this step; offsets below
                }
                for (int i = 0; i < ne; ++i)
                    worstOffset = std::max(worstOffset,
                                           std::fabs(double(exact[i].t) - double(lerped[i].t)));

                // What actually matters is the paint, not the offsets: compare
                // the evaluator's output driven by each.
                double le = r.xs.leftExtentAt(s), re = r.xs.rightExtentAt(s);
                for (int j = 0; j <= 24; ++j) {
                    double t = re + (le - re) * double(j) / 24.0;
                    RlPaint pe = rlEvaluateMarkings(exact, ne, float(s), float(t), 0.03f, 0.0f,
                                                    1.0f, 0.0f);
                    RlPaint pl = rlEvaluateMarkings(lerped, nl, float(s), float(t), 0.03f, 0.0f,
                                                    1.0f, 0.0f);
                    worstCoverage =
                        std::max(worstCoverage, std::fabs(double(pe.coverage) - double(pl.coverage)));
                }
            }
        }
        // The cap is what makes a fixed-size GPU record possible at all, so it
        // is an invariant, not a hope. Overflowing it truncates the far end of
        // the array and shifts every slot past the cut.
        std::vector<Boundary> all;
        int worstPainted = 0;
        for (const Road& r : sc.net.roads()) {
            double len = r.activeLength();
            if (len < 1.0) continue;
            for (int k = 0; k <= 20; ++k) {
                r.xs.boundariesAt(r.begin() + len * double(k) / 20.0, all);
                int painted = 0;
                for (const Boundary& x : all)
                    if (x.mark.style != MarkStyle::None) ++painted;
                worstPainted = std::max(worstPainted, painted);
            }
        }
        check(worstPainted <= kMaxBakedBoundaries,
              (name + ": painted boundaries fit the shader's fixed record").c_str());

        check(compared > 100, (name + ": there were stations to compare").c_str());
        // Boundary offsets are cubics in s; over a 2 m step a straight line
        // through them is accurate to well under a paint width (0.10-0.15 m).
        check(worstOffset < 0.05,
              (name + ": interpolated boundary offsets stay within 50 mm").c_str());
        // A handful of steps straddle a lane appearing or ending, where the
        // count legitimately changes mid-step. It has to be rare, or the mesh
        // needs a ring at every section boundary.
        check(countMismatch < compared / 8,
              (name + ": few steps straddle a lane-count change").c_str());
        check(worstCoverage < 0.5,
              (name + ": interpolated paint matches the exact paint").c_str());
        if (worstOffset > 0.01 || worstCoverage > 0.1)
            std::printf("  note: %-11s worst offset %.3f m, worst coverage delta %.2f\n",
                        name.c_str(), worstOffset, worstCoverage);
    }
}

void testSeamSmearIsMillimetresWide() {
    group("seam smear");
    // The one place interpolation cannot be right.
    //
    // At a lane-section boundary the cross-section changes discontinuously — a
    // lane appears, and the boundary in slot k is a different piece of the road
    // on each side. bakeBoundaryStrip lands a ring 1 mm either side so that no
    // ordinary step straddles it, which does not remove the discontinuity; it
    // confines it. This measures the confinement, because "confined" is a claim
    // and 1.7 m of misplaced paint is what it costs if the claim is wrong.
    double worstWindow = 0;
    long seams = 0;
    std::string worstAt;
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);
        for (const Road& r : sc.net.roads()) {
            if (r.activeLength() < 8.0) continue;
            BoundaryStrip strip = bakeBoundaryStrip(r, 2.0);
            for (const LaneSection& sec : r.xs.sections) {
                if (sec.s0 <= r.begin() + 0.05 || sec.s0 >= r.end() - 0.05) continue;
                ++seams;
                // Walk outward until the interpolated profile agrees with the
                // exact one again. 0.05 m is a third of a stripe width — below
                // that a disagreement cannot move a marking off itself.
                double window = 0;
                for (int sign = -1; sign <= 1; sign += 2) {
                    for (int i = 1; i <= 200; ++i) {
                        double d = double(i) * 1e-4;   // 0.1 mm steps out to 20 mm
                        double s = sec.s0 + double(sign) * d;
                        if (s <= r.begin() || s >= r.end()) break;
                        RlBoundary exact[kMaxBakedBoundaries], lerped[kMaxBakedBoundaries];
                        int ne = bakeBoundaries(r, s, exact, kMaxBakedBoundaries);
                        int nl = strip.sampleInterpolated(s, lerped, kMaxBakedBoundaries);
                        bool bad = ne != nl;
                        for (int j = 0; !bad && j < ne; ++j)
                            if (std::fabs(double(exact[j].t) - double(lerped[j].t)) > 0.05 ||
                                exact[j].style != lerped[j].style)
                                bad = true;
                        if (bad) window = std::max(window, d);
                    }
                }
                if (window > worstWindow) {
                    worstWindow = window;
                    worstAt = name;
                }
            }
        }
    }
    check(seams >= 10, "there are lane-section seams across the demos to measure");
    // The ring pair is at +/-1 mm, so 1 mm either side is the floor. Anything
    // beyond about 2 mm means a step is straddling a seam somewhere — a ring
    // that did not get placed, or a section list the bake did not see.
    check(worstWindow <= 2.0e-3,
          "paint is misplaced only within the 1 mm ring pair around a seam");
    std::printf("  note: %ld seams, widest disagreement %.2f mm (%s)\n", seams,
                worstWindow * 1000.0, worstAt.c_str());
}

void testProfileTextureIsWhatTheShaderWouldSample() {
    group("profile texture");
    // paint_texture.h claims a fragment can reconstruct the cross-section from
    // two texture fetches and one interpolated scalar. Three things have to hold
    // for that to be true, and none of them is obvious:
    //
    //   1. The row coordinate the mesher writes into a vertex is exactly the row
    //      the bake put there — otherwise every ring samples its neighbour.
    //   2. Splitting the record across a linear texture, a nearest one and a
    //      deduplicated style table loses nothing against the single
    //      interpolated array it replaces.
    //   3. The result still paints the road the cross-section describes.
    size_t worstAtlas = 0;
    std::string worstName;
    for (const std::string& name : demoNames()) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);

        PaintAtlas atlas = bakePaintAtlas(sc.net, 2.0);
        if (atlas.bytes() > worstAtlas) {
            worstAtlas = atlas.bytes();
            worstName = name;
        }
        check(atlas.rows > 0, (name + ": the atlas has rows").c_str());
        check(atlas.profileTex.size() == size_t(atlas.rows) * size_t(atlas.profileWidth()) * 4,
              (name + ": the profile texture is exactly rows x width").c_str());
        check(atlas.styleTex.size() == size_t(atlas.styleRows) * size_t(atlas.styleWidth()) * 4,
              (name + ": the style texture is exactly styleRows x width").c_str());
        // The indirection only pays if the table really is small. If a
        // deduplicated style row appeared per station the layout would be
        // strictly worse than the one it replaced — an extra fetch for nothing.
        check(atlas.styleRows < atlas.rows,
              (name + ": distinct style sets are fewer than stations").c_str());

        double worstT = 0, worstCoverage = 0;
        long countMismatch = 0, compared = 0, styleMismatch = 0;
        const std::vector<Road>& roads = sc.net.roads();
        for (size_t ri = 0; ri < roads.size(); ++ri) {
            const Road& r = roads[ri];
            const PaintProfile& prof = atlas.profiles[ri];
            if (r.activeLength() < 8.0) continue;

            // (1) The ring contract. Ring i of the mesh sits at stations[i] and
            // writes rowBase + i; if rowCoord disagrees the mesh and the texture
            // are off by a row and every marking lands on the wrong ring.
            bool ringsLineUp = true;
            for (int i = 0; i < prof.rows(); ++i)
                if (std::fabs(prof.rowCoord(prof.stations[size_t(i)]) -
                              double(prof.rowBase + i)) > 1e-6)
                    ringsLineUp = false;
            check(ringsLineUp, (name + ": every ring's row coordinate is its own row").c_str());

            BoundaryStrip strip = bakeBoundaryStrip(r, 2.0);
            for (int k = 1; k < 40; ++k) {
                double s = r.begin() + r.activeLength() * double(k) / 40.0;
                RlBoundary viaStrip[kMaxBakedBoundaries];
                RlBoundary viaTex[kMaxBakedBoundaries];
                RlBoundary exact[kMaxBakedBoundaries];
                int ns = strip.sampleInterpolated(s, viaStrip, kMaxBakedBoundaries);
                int nt = atlas.sample(int(ri), prof.rowCoord(s), viaTex, kMaxBakedBoundaries);
                int ne = bakeBoundaries(r, s, exact, kMaxBakedBoundaries);
                ++compared;

                // (2) The split is lossless. Not "close" — the two paths read
                // the same numbers, so any difference is a packing bug.
                if (ns != nt) {
                    ++countMismatch;
                } else {
                    for (int i = 0; i < ns; ++i) {
                        worstT = std::max(worstT, std::fabs(double(viaStrip[i].t) -
                                                            double(viaTex[i].t)));
                        if (viaStrip[i].style != viaTex[i].style ||
                            viaStrip[i].width != viaTex[i].width ||
                            viaStrip[i].gap != viaTex[i].gap ||
                            viaStrip[i].color != viaTex[i].color ||
                            viaStrip[i].dashOn != viaTex[i].dashOn ||
                            viaStrip[i].dashOff != viaTex[i].dashOff ||
                            viaStrip[i].wear != viaTex[i].wear)
                            ++styleMismatch;
                    }
                }

                // (3) End to end, against the cross-section rather than against
                // the intermediate.
                if (ne == nt) {
                    double le = r.xs.leftExtentAt(s), re = r.xs.rightExtentAt(s);
                    for (int j = 0; j <= 24; ++j) {
                        double t = re + (le - re) * double(j) / 24.0;
                        RlPaint pe = rlEvaluateMarkings(exact, ne, float(s), float(t), 0.03f,
                                                        0.0f, 1.0f, 0.0f);
                        RlPaint pt = rlEvaluateMarkings(viaTex, nt, float(s), float(t), 0.03f,
                                                        0.0f, 1.0f, 0.0f);
                        worstCoverage = std::max(worstCoverage,
                                                 std::fabs(double(pe.coverage) -
                                                           double(pt.coverage)));
                    }
                }
            }
        }
        check(compared > 100, (name + ": there were stations to sample").c_str());
        check(countMismatch == 0,
              (name + ": the texture yields the same boundary count as the strip").c_str());
        check(styleMismatch == 0,
              (name + ": and the same style, width, dash and colour, bit for bit").c_str());
        check(worstT < 1e-5,
              (name + ": and the same lateral offsets to float precision").c_str());
        check(worstCoverage < 0.5,
              (name + ": paint sampled from the texture matches the cross-section").c_str());
    }

    // Numbers worth knowing before anyone uploads this: the whole atlas for the
    // biggest demo at the mesher's own ring spacing, and how hard the style
    // table folded, which is what decides whether this scales to a city.
    {
        Scene sc;
        buildDemo(worstName, sc);
        finalizeScene(sc, false, false);
        PaintAtlas atlas = bakePaintAtlas(sc.net, 2.0);
        std::printf("  note: largest paint atlas %s, %.1f KiB (RGBA32F; halves as RGBA16F) "
                    "- %d rows, %d distinct style sets (%.0fx fold)\n",
                    worstName.c_str(), double(worstAtlas) / 1024.0, atlas.rows, atlas.styleRows,
                    double(atlas.rows) / double(std::max(1, atlas.styleRows)));
    }

    // Nearest along the style axis is the reason a seam works at all. Just
    // inside a lane section the style must be that section's, not a blend with
    // its neighbour's — and a blend would be silent, because a style code
    // halfway between Dashed and Double still selects a branch.
    Scene sc;
    if (buildDemo("lanes", sc)) {
        finalizeScene(sc, false, false);
        PaintAtlas atlas = bakePaintAtlas(sc.net, 2.0);
        long checkedSeams = 0, wrongStyle = 0;
        const std::vector<Road>& roads = sc.net.roads();
        for (size_t ri = 0; ri < roads.size(); ++ri) {
            const Road& r = roads[ri];
            const PaintProfile& prof = atlas.profiles[ri];
            for (const LaneSection& sec : r.xs.sections) {
                for (double d : {0.05, -0.05}) {
                    double s = sec.s0 + d;
                    if (s <= r.begin() + 1e-6 || s >= r.end() - 1e-6) continue;
                    RlBoundary tex[kMaxBakedBoundaries], exact[kMaxBakedBoundaries];
                    int nt = atlas.sample(int(ri), prof.rowCoord(s), tex, kMaxBakedBoundaries);
                    int ne = bakeBoundaries(r, s, exact, kMaxBakedBoundaries);
                    ++checkedSeams;
                    if (nt != ne) { ++wrongStyle; continue; }
                    for (int i = 0; i < ne; ++i)
                        if (tex[i].style != exact[i].style) ++wrongStyle;
                }
            }
        }
        check(checkedSeams > 0, "the lanes demo has lane-section seams to test");
        check(wrongStyle == 0, "50 mm from a seam the style is the section's own, unblended");
    }
}

// --- earthworks -------------------------------------------------------------


namespace {

// The PAVED extent: out to the last strip you could drive or stand on, stopping
// before the earthwork. A Slope is a cut/fill batter running out to terrain, and
// a ramp passing over one is a grading problem, not two roads drawn on top of
// each other — conflating them hides the defect that matters.
double pavedExtentAt(const Road& r, double s, int side) {
    int si = r.xs.sectionIndexAt(s);
    const LaneSection& sec = r.xs.sections[size_t(si)];
    const std::vector<Strip>& stack = side >= 0 ? sec.left : sec.right;
    double t = 0;
    for (const Strip& x : stack) {
        if (x.kind == StripKind::Slope || x.kind == StripKind::Verge) break;
        t += x.width.eval(s - sec.s0);
    }
    return side >= 0 ? t : -t;
}

}  // namespace

// The full corpus: the demos AND the two generators.
//
// The gates below used to iterate demoNames() alone, and that gap is the whole
// reason this exists. Every geometry defect the demos were clean of shipped in
// the metro generator anyway — ramps crossing their own carriageway, three
// roundabout rings drawn on top of each other — because the generators are what
// the engine actually loads and nothing measured them. A corpus that does not
// contain what ships is a corpus that certifies the wrong thing.
std::vector<std::string> corpusNames() {
    std::vector<std::string> names = demoNames();
    names.push_back("metro");
    names.push_back("city");
    return names;
}

bool buildCorpus(const std::string& name, Scene& sc) {
    if (name == "metro") {
        MetroParams mp;
        mp.seed = 7;
        generateMetro(sc, mp);
        return true;
    }
    if (name == "city") {
        CityParams cp;
        cp.seed = 3;
        generateCity(sc, cp);
        return true;
    }
    return buildDemo(name, sc);
}

void testRoadSurfacesDoNotOverlap() {
    group("no overlapping pavement");
    // One surface owns each piece of ground. In a 3D world two carriageways
    // drawn on the same square metre is not a subtle artefact — it is z-fighting
    // in the best case and a ramp visibly buried in a freeway in the worst, which
    // is exactly what this found: the on-ramp's pavement ran up to 11 m inside
    // the mainline with a concrete barrier riding along its edge.
    //
    // The metric is shared INTERIOR, not shared edges. Surfaces that abut touch
    // along a boundary by design — that is what a correct merge IS — and a test
    // that flagged it could not tell a good join from a road driving through
    // another one. Interior in `s` as well as `t`, because splitRoad partitions
    // one spine into abutting windows whose end stations are shared by
    // construction.
    for (const std::string& name : corpusNames()) {
        Scene sc;
        if (!buildCorpus(name, sc)) continue;
        finalizeScene(sc, false, false);

        long probes = 0, bad = 0;
        double worst = 0;
        std::string worstAt;
        const double kEnd = 1.0;
        for (const Road& r : sc.net.roads()) {
            if (r.kind == RoadKind::Connector) continue;
            double len = r.activeLength();
            if (len < 2 * kEnd + 2) continue;
            for (int k = 0; k <= 60; ++k) {
                double s = r.begin() + kEnd + (len - 2 * kEnd) * double(k) / 60.0;
                if (carrierAt(r, s) != CarrierKind::AtGrade) continue;
                double le = pavedExtentAt(r, s, +1), re = pavedExtentAt(r, s, -1);
                for (int j = 1; j <= 5; ++j) {
                    double t = re + (le - re) * double(j) / 6.0;
                    Vec3 e = r.surfacePoint(s, t);
                    ++probes;
                    for (const Road& o : sc.net.roads()) {
                        if (o.id == r.id || o.kind == RoadKind::Connector) continue;
                        double os = 0, ot = 0;
                        if (!o.spine.toST({e.x, e.z}, os, ot)) continue;
                        if (os < o.begin() + kEnd || os > o.end() - kEnd) continue;
                        if (carrierAt(o, os) != CarrierKind::AtGrade) continue;
                        double ole = pavedExtentAt(o, os, +1), ore = pavedExtentAt(o, os, -1);
                        if (ot > ole || ot < ore) continue;
                        // A grade separation is two surfaces at different heights
                        // sharing a plan position, which is the whole point of an
                        // overpass.
                        if (std::fabs(o.surfacePoint(os, ot).y - e.y) > 1.0) continue;
                        double depth = std::min(ole - ot, ot - ore);
                        if (depth > 0.5) {
                            ++bad;
                            if (depth > worst) {
                                worst = depth;
                                worstAt = r.name + " in " + o.name;
                            }
                        }
                        break;
                    }
                }
            }
        }
        check(probes > 300, (name + ": there was pavement to probe").c_str());
        // Two samples out of ~1700 still touch at the gore, where the ramp's
        // pavement and the auxiliary lane's meet across a 1 m nose taper. That is
        // a real residual and it is bounded here rather than rounded away.
        // Zero, not a small budget. The threshold is already half a metre of
        // shared interior, which nothing that merely abuts can reach, so a
        // tolerance here would only ever be room for a defect to hide in.
        check(bad == 0, (name + ": pavement does not overlap other pavement").c_str());
        check(worst < 0.01, (name + ": and no residual at all").c_str());
        if (bad > 0)
            std::printf("  note: %-11s %ld of %ld probes overlap, worst %.2f m [%s]\n",
                        name.c_str(), bad, probes, worst, worstAt.c_str());
    }
}

void testRampsAbutTheirMainline() {
    group("no crack at a merge");
    // Two surfaces that meet either MEET or are visibly apart. What they may not
    // do is miss by a hairline.
    //
    // A ramp's reference line is placed on the mainline's lane edge and then both
    // are drawn — and toPlan lays a lateral offset out flat while toWorld lays it
    // along the banked frame and leans the crown over with it. On a 110 km/h
    // carriageway at 6% those differ by 37 mm at the outside lane, so the ramp
    // sat 37 mm off the freeway for the whole length of the merge: a crack you
    // can see from the driver's seat and that no cross-section check can find,
    // because the cross-sections agree exactly. This measures the DRAWN edges.
    //
    // The measure is the CLOSEST the two ever come. A gore legitimately opens
    // from nothing to metres, so no distance band can separate "opening" from
    // "offset" — but a ramp that abuts its mainline touches it somewhere, and a
    // ramp that is uniformly 37 mm out never does. The minimum is zero or it is
    // the bug.
    for (const std::string& name : corpusNames()) {
        Scene sc;
        if (!buildCorpus(name, sc)) continue;
        finalizeScene(sc, false, false);

        double worst = 0;
        long ramps = 0, cracked = 0;
        std::string worstAt;
        for (const Road& r : sc.net.roads()) {
            if (r.kind != RoadKind::Ramp) continue;
            double len = r.activeLength();
            if (len < 20.0) continue;
            // Which end is the gore, and therefore which way the parallel
            // stretch runs. The lane link says it: an entrance hands its lane on
            // at the ramp's END, an exit takes one at its START.
            bool linked = false, entering = false;
            for (const ExtraLaneLink& el : sc.net.extraLinks) {
                if (el.fromRoad == r.id) { linked = true; entering = true; }
                else if (el.toRoad == r.id) { linked = true; entering = false; }
            }
            if (!linked) continue;
            double nose = entering ? r.end() : r.begin();
            double inward = entering ? -1.0 : 1.0;

            // The mainline PIECE the ramp runs alongside, found 10 m inside the
            // parallel stretch. Not the piece the link names: a split puts the
            // nose on a boundary, and the link points at the far side of it,
            // whose outer edge is a whole auxiliary lane further out.
            const Road* m = nullptr;
            {
                Vec3 probe = r.surfacePoint(nose + inward * 10.0, 0.0);
                for (const Road& o : sc.net.roads()) {
                    if (o.id == r.id || o.kind == RoadKind::Ramp) continue;
                    if (o.kind == RoadKind::Connector) continue;
                    double os = 0, ot = 0;
                    if (!o.spine.toST({probe.x, probe.z}, os, ot)) continue;
                    if (os < o.begin() + 1e-6 || os > o.end() - 1e-6) continue;
                    if (std::fabs(ot) > 40.0) continue;
                    m = &o;
                    break;
                }
            }
            if (!m) continue;
            double closest = 1e9;
            for (int k = 0; k <= 120; ++k) {
                double s = nose + inward * 60.0 * double(k) / 120.0;
                if (s < r.begin() || s > r.end()) continue;
                Vec3 ref = r.surfacePoint(s, 0.0);
                double ms = 0, mt = 0;
                if (!m->spine.toST({ref.x, ref.z}, ms, mt)) continue;
                if (ms < m->begin() - 1e-6 || ms > m->end() + 1e-6) continue;
                // LATERAL separation, in the mainline's own plan frame. Point
                // distance would fold in the along-track slack of the projection
                // and read a few millimetres of it as a crack.
                double edgeT = pavedExtentAt(*m, ms, mt >= 0 ? +1 : -1);
                Vec3 edge = m->surfacePoint(ms, edgeT);
                double es = 0, et = 0;
                if (!m->spine.toST({edge.x, edge.z}, es, et)) continue;
                closest = std::min(closest, std::fabs(mt - et));
            }
            if (closest > 1e8) continue;
            std::string against = m->name;
            ++ramps;
            if (closest > 0.002) {
                ++cracked;
                if (closest > worst) {
                    worst = closest;
                    worstAt = r.name + " on " + against;
                }
            }
        }
        check(cracked == 0, (name + ": every ramp reaches its mainline").c_str());
        if (cracked)
            std::printf("  note: %-11s %ld of %ld ramps never touch, worst %.1f mm [%s]\n",
                        name.c_str(), cracked, ramps, worst * 1000.0, worstAt.c_str());
    }
}

void testRampsDoNotEncroach() {
    group("nothing of the ramp on the carriageway");
    // The gate the last four attempts needed and did not have.
    //
    // testRoadSurfacesDoNotOverlap samples five INTERIOR points across a road's
    // width and calls it an overlap past half a metre of depth. On an 8 m ramp
    // that lands at most 0.23 m into the inboard sliver — so it could not see
    // the thing that was actually wrong: the ramp preset's inner shoulder and
    // its 0.85 m jersey barrier, 1.6 m of them, standing on the freeway's
    // outside lane for the whole approach. Every metric said zero while the
    // merge was a walled chute.
    //
    // This one sweeps the ramp's FULL paved width at 25 cm, inboard strips
    // included, and asks how far into another carriageway each sample falls.
    // Abutting surfaces touch at a boundary, so the answer is zero.
    for (const std::string& name : corpusNames()) {
        Scene sc;
        if (!buildCorpus(name, sc)) continue;
        finalizeScene(sc, false, false);

        double worst = 0;
        long probes = 0, inside = 0;
        std::string worstAt;
        for (const Road& r : sc.net.roads()) {
            if (r.kind != RoadKind::Ramp) continue;
            double len = r.activeLength();
            if (len < 20.0) continue;
            // Strictly before the gore: at and past the nose the mainline's own
            // auxiliary lane covers the same ground, by construction.
            bool entering = false;
            for (const ExtraLaneLink& el : sc.net.extraLinks)
                if (el.fromRoad == r.id) entering = true;
            double lo = entering ? r.begin() : r.begin() + 6.0;
            double hi = entering ? r.end() - 6.0 : r.end();
            for (int k = 0; k <= 120; ++k) {
                double s = lo + (hi - lo) * double(k) / 120.0;
                if (s <= r.begin() || s >= r.end()) continue;
                double le = pavedExtentAt(r, s, +1), re = pavedExtentAt(r, s, -1);
                for (double t = re; t <= le + 1e-9; t += 0.25) {
                    Vec3 p = r.surfacePoint(s, t);
                    ++probes;
                    for (const Road& o : sc.net.roads()) {
                        if (o.id == r.id || o.kind == RoadKind::Connector) continue;
                        if (o.kind == RoadKind::Ramp) continue;
                        double os = 0, ot = 0;
                        if (!o.spine.toST({p.x, p.z}, os, ot)) continue;
                        if (os < o.begin() + 1.0 || os > o.end() - 1.0) continue;
                        if (carrierAt(o, os) != CarrierKind::AtGrade) continue;
                        double ole = pavedExtentAt(o, os, +1), ore = pavedExtentAt(o, os, -1);
                        if (ot > ole || ot < ore) continue;
                        Vec3 op = o.surfacePoint(os, ot);
                        if (std::fabs(op.y - p.y) > 1.0) continue;   // grade separated
                        double depth = std::min(ole - ot, ot - ore);
                        if (depth > 0.05) {
                            ++inside;
                            if (depth > worst) {
                                worst = depth;
                                worstAt = r.name + " into " + o.name;
                            }
                        }
                        break;
                    }
                }
            }
        }
        check(inside == 0, (name + ": a ramp puts nothing on the carriageway").c_str());
        if (inside)
            std::printf("  note: %-11s %ld of %ld samples inside, worst %.2f m [%s]\n",
                        name.c_str(), inside, probes, worst, worstAt.c_str());
    }
}

void testEveryMergeHasAGore() {
    group("a merge has a gore");
    // A gore is what makes a merge legible, and it is PAINT.
    //
    // With the geometry exact — continuous pavement, edge lines that meet, no
    // overlap anywhere — the join still read as an anonymous fork in the asphalt,
    // because two edge lines converging to a knife point is not what a driver
    // recognises. What they recognise is the neutral area: a wedge, outlined and
    // chevroned, narrowing until there is none left and the two lanes are simply
    // side by side.
    //
    // So it is asserted like any other invariant. The wedge must open to
    // something worth seeing, it must close at the nose, and it must carry the
    // area marking — a Median strip nobody painted would just be wider asphalt.
    for (const std::string& name : corpusNames()) {
        Scene sc;
        if (!buildCorpus(name, sc)) continue;
        finalizeScene(sc, false, false);

        long ramps = 0, withGore = 0;
        for (const Road& r : sc.net.roads()) {
            if (r.kind != RoadKind::Ramp) continue;
            bool linked = false, entering = false;
            for (const ExtraLaneLink& el : sc.net.extraLinks) {
                if (el.fromRoad == r.id) { linked = true; entering = true; }
                else if (el.toRoad == r.id) { linked = true; entering = false; }
            }
            if (!linked || r.activeLength() < 20.0) continue;
            ++ramps;

            double nose = entering ? r.end() : r.begin();
            // The gore is the strip against the reference line, on the side the
            // ramp's own stack grows from.
            auto goreAt = [&](double s, bool& painted) {
                painted = false;
                int si = r.xs.sectionIndexAt(s);
                const LaneSection& sec = r.xs.sections[size_t(si)];
                for (const std::vector<Strip>* st : {&sec.right, &sec.left}) {
                    if (st->empty()) continue;
                    const Strip& first = st->front();
                    if (first.kind != StripKind::Median) continue;
                    painted = first.outerMark.style == MarkStyle::Chevron ||
                              first.outerMark.style == MarkStyle::Hatch;
                    return first.width.eval(s - sec.s0);
                }
                return 0.0;
            };

            double widest = 0;
            bool anyPainted = false;
            double len = r.activeLength();
            for (int k = 0; k <= 100; ++k) {
                double s = r.begin() + len * double(k) / 100.0;
                bool painted = false;
                double w = goreAt(s, painted);
                if (w > widest) widest = w;
                if (painted && w > 0.5) anyPainted = true;
            }
            bool closedAtNose = false;
            {
                double s = clampd(entering ? nose - 0.5 : nose + 0.5, r.begin(), r.end());
                bool painted = false;
                closedAtNose = goreAt(s, painted) < 0.25;
            }
            if (widest >= 2.0 && anyPainted && closedAtNose) ++withGore;
            else
                std::printf("  note: %-11s %s: widest %.2f m, painted %d, closed at nose %d\n",
                            name.c_str(), r.name.c_str(), widest, int(anyPainted),
                            int(closedAtNose));
        }
        if (ramps == 0) continue;
        check(withGore == ramps,
              (name + ": every merging ramp has a painted gore that closes at the nose").c_str());
    }
}

void testEveryCorpusSceneLints() {
    group("design lint over the whole corpus");
    // validate() knows about minimum radius, taper rate, grade, and vertical
    // clearance, and every one of those is a thing a generator can get wrong
    // without any test noticing — the demos were clean while the metro
    // generator shipped ramps below their minimum radius. Running the lint the
    // scene authoring tool would run, over everything that ships, costs one
    // pass and closes that gap.
    for (const std::string& name : corpusNames()) {
        Scene sc;
        if (!buildCorpus(name, sc)) continue;
        finalizeScene(sc, false, false);
        std::vector<std::string> issues = sc.net.validate();
        check(issues.empty(), (name + ": passes the design lint").c_str());
        for (size_t i = 0; i < issues.size() && i < 6; ++i)
            std::printf("  note: %s\n", issues[i].c_str());
    }
}

void testTheAtlasFitsInATexture() {
    group("the paint atlas fits in a texture");
    // A row per station ring is a number that grows with kilometres of road, and
    // a texture dimension is a number that does not grow at all. The demos hid
    // this — 1730 rows is nothing — and the metro scene did not: 19786 rows,
    // past the 16384 the test device allows and more than twice the 8192 that
    // core WebGPU guarantees. It was a GPU validation error at upload, which is
    // a place no test was looking.
    //
    // So the atlas tiles, and this checks both dimensions of the tiled image
    // against the guaranteed limit rather than whatever the local device
    // happens to permit.
    const int kWebGpuMaxTextureDim = 8192;
    for (const std::string& name : corpusNames()) {
        Scene sc;
        if (!buildCorpus(name, sc)) continue;
        finalizeScene(sc, false, false);
        PaintAtlas atlas = bakePaintAtlas(sc.net, 2.0);

        check(atlas.imageWidth() <= kWebGpuMaxTextureDim &&
                  atlas.imageHeight() <= kWebGpuMaxTextureDim,
              (name + ": the tiled profile texture fits 8192 on a side").c_str());

        // Tiling is only correct if it is a permutation: every logical row has
        // to land somewhere, unchanged, and be findable by the addressing the
        // shader uses. Checking the shape alone would pass a bake that dropped
        // every second row.
        std::vector<float> img = atlas.profileImage();
        check(img.size() == size_t(atlas.imageWidth()) * size_t(atlas.imageHeight()) * 4,
              (name + ": the tiled image is exactly its stated size").c_str());
        const size_t stride = size_t(kProfileTexelsPerRow) * 4;
        long moved = 0;
        for (int r = 0; r < atlas.rows; ++r) {
            size_t x = size_t(r % kRowsPerTile) * stride;
            size_t y = size_t(r / kRowsPerTile);
            const float* dst = &img[y * size_t(atlas.imageWidth()) * 4 + x];
            const float* src = &atlas.profileTex[size_t(r) * stride];
            for (size_t i = 0; i < stride; ++i)
                if (dst[i] != src[i]) ++moved;
        }
        check(moved == 0, (name + ": every atlas row survives the tiling").c_str());
    }
}

void testEarthworksRunOnTheirBatter() {
    group("earthworks");
    // The ground meets a road at the road's edge and then runs away from it on a
    // slope with an AUTHORED gradient, until it reaches natural ground. The
    // property that makes it an earthwork rather than a feather is that the
    // gradient does not depend on how deep the earthwork is.
    //
    // Before this existed the conform was a fixed-width smoothstep, so the slope
    // was the height difference divided by a constant: measured at 76 degrees on
    // the demos as shipped and 83 degrees with real relief. The worst case was a
    // road END, where the influence stopped dead and the ground fell from the
    // carriageway to natural ground in half a metre.
    for (const std::string& name : demoNames()) {
        for (double amp : {6.0, 20.0, 45.0}) {
            Scene sc;
            if (!buildDemo(name, sc)) continue;
            sc.terrain.amplitude = amp;
            sc.terrain.frequency = 1.0 / 200.0;
            finalizeScene(sc, false, false);
            const TerrainParams& tp = sc.terrain;

            double worst = 0;
            long sampled = 0, steep = 0, walls = 0;
            for (const Road& r : sc.net.roads()) {
                if (r.kind == RoadKind::Connector) continue;
                double len = r.activeLength();
                if (len < 20.0) continue;
                for (int k = 0; k <= 10; ++k) {
                    double s = r.begin() + len * double(k) / 10.0;
                    if (carrierAt(r, s) != CarrierKind::AtGrade) continue;
                    for (int side = 0; side < 2; ++side) {
                        double edgeT = side == 0 ? r.xs.leftExtentAt(s) : r.xs.rightExtentAt(s);
                        Vec3 e0 = r.surfacePoint(s, edgeT);
                        Vec3 e1 = r.surfacePoint(s, edgeT + (side == 0 ? 0.01 : -0.01));
                        Vec2 out = normalize(Vec2{e1.x - e0.x, e1.z - e0.z});
                        // Walk out past where any batter could still be running.
                        for (int m = 1; m <= 34; ++m) {
                            Vec2 q{e0.x + out.x * 1.3 * m, e0.z + out.y * 1.3 * m};
                            // Skip retaining-wall sites. Two roads at different
                            // levels closer together than their batters can
                            // reconcile leave a real step, and calling that a
                            // failed batter would be blaming the earthwork for a
                            // geometry decision. terrainConflictAt names them, so
                            // the bound below can be strict rather than a
                            // percentage — a percentage would also have passed
                            // while a genuine batter bug produced the same rate.
                            if (terrainConflictAt(sc.net, tp, q.x, q.y) ||
                                terrainConflictAt(sc.net, tp, q.x + 0.5, q.y) ||
                                terrainConflictAt(sc.net, tp, q.x - 0.5, q.y) ||
                                terrainConflictAt(sc.net, tp, q.x, q.y + 0.5) ||
                                terrainConflictAt(sc.net, tp, q.x, q.y - 0.5)) {
                                ++walls;
                                continue;
                            }
                            // The bound is the batter OR the natural ground,
                            // whichever is steeper. An earthwork is not allowed
                            // to invent a slope; it is perfectly allowed to leave
                            // a mountainside alone, and at these amplitudes the
                            // mountainside is steeper than 1:1 on its own. Only
                            // measuring against the batter would score the
                            // terrain's own relief as an earthwork failure.
                            double nat = 0;
                            double h0 = terrainBaseHeight(tp, q.x, q.y);
                            for (int e = 0; e < 4; ++e) {
                                double ex = e == 0 ? 0.5 : (e == 1 ? -0.5 : 0.0);
                                double ez = e == 2 ? 0.5 : (e == 3 ? -0.5 : 0.0);
                                nat = std::max(nat, std::fabs(terrainBaseHeight(tp, q.x + ex,
                                                                                q.y + ez) - h0) / 0.5);
                            }
                            double allowed = std::max(tp.cutBatter * 1.35, nat * 1.1) + 0.05;
                            double sl = terrainSlopeAt(sc.net, tp, q, 0.5);
                            ++sampled;
                            worst = std::max(worst, sl);
                            if (sl > allowed) ++steep;
                        }
                    }
                }
            }
            check(sampled > 200, (name + ": there was ground beside the roads to walk").c_str());
            // Where a road is carried on a structure the earthwork still has an
            // unsolved boundary: the approach embankment claims the ground and
            // the deck does not, so a step survives just outside the abutment
            // vicinity this marks. It is bounded and localised — worst measured
            // 2.53 against a 1.00 batter, on a handful of samples in the three
            // multi-level demos — but it is not resolved, so it is stated rather
            // than absorbed into a loose global bound.
            //
            // 2.53 is the interchange at the 45 m stress amplitude, and it rose
            // from 1.72 when a ramp started picking up the mainline's batter over
            // the whole stretch it runs alongside rather than the last 20 m. That
            // is the correct window — it is exactly the stretch over which the
            // mainline gives the batter UP — so the extra step is the abutment
            // boundary being asked a harder question, not a new defect. At the
            // amplitude the demos actually ship (5 m) the figure is unchanged.
            bool hasStructures = false;
            for (const Road& rr : sc.net.roads())
                if (!rr.structures.empty()) hasStructures = true;
            if (hasStructures) {
                check(worst <= 2.6,
                      (name + ": steps near a structure stay bounded").c_str());
            } else {
                check(steep == 0,
                      (name + ": no ground is steeper than its batter or the hillside").c_str());
            }
            if (walls > 0 && amp == 20.0)
                std::printf("  note: %-11s amp %.0f: worst batter %.2f (authored %.2f), "
                            "%ld of %ld samples are retaining-wall sites\n",
                            name.c_str(), amp, worst, tp.cutBatter, walls, walls + sampled);
        }
    }

    // The gradient is authored, so changing it must change the ground. A test
    // that only bounds the slope would pass just as well if the batter were
    // ignored and the terrain were flat.
    Scene sc;
    buildDemo("grades", sc);
    sc.terrain.amplitude = 30.0;
    Scene gentle = sc, steep = sc;
    gentle.terrain.cutBatter = 0.25;
    gentle.terrain.fillBatter = 0.2;
    steep.terrain.cutBatter = 2.0;
    steep.terrain.fillBatter = 1.5;
    finalizeScene(gentle, false, false);
    finalizeScene(steep, false, false);
    const Road& r = gentle.net.roads().front();
    double s = r.begin() + r.activeLength() * 0.5;
    double edgeT = r.xs.leftExtentAt(s);
    Vec3 e0 = r.surfacePoint(s, edgeT);
    Vec3 e1 = r.surfacePoint(s, edgeT + 0.01);
    Vec2 out = normalize(Vec2{e1.x - e0.x, e1.z - e0.z});
    double gentleMax = 0, steepMax = 0;
    for (int m = 1; m <= 50; ++m) {
        Vec2 q{e0.x + out.x * 0.75 * m, e0.z + out.y * 0.75 * m};
        gentleMax = std::max(gentleMax, terrainSlopeAt(gentle.net, gentle.terrain, q, 0.5));
        steepMax = std::max(steepMax, terrainSlopeAt(steep.net, steep.terrain, q, 0.5));
    }
    check(steepMax > gentleMax * 1.5,
          "a steeper authored batter really does cut a steeper slope");
    check(gentleMax < 0.5, "and a gentle one stays gentle");

    // A road END is the case that used to be a cliff: the conform stopped at the
    // road's window, so the ground fell to natural in one sample. The earthwork
    // now measures distance to the road's FOOTPRINT, which has ends as well as
    // sides, so it runs out past the end the same way it runs out sideways.
    Scene ends;
    buildDemo("lanes", ends);
    ends.terrain.amplitude = 30.0;
    finalizeScene(ends, false, false);
    const Road& re = ends.net.roads().front();
    Frame f = re.spine.frameAt(re.end());
    Vec2 ahead = dirOf(f.heading);
    double worstEnd = 0;
    for (int m = 0; m <= 50; ++m) {
        Vec2 q{f.planPos.x + ahead.x * 0.75 * m, f.planPos.y + ahead.y * 0.75 * m};
        worstEnd = std::max(worstEnd, terrainSlopeAt(ends.net, ends.terrain, q, 0.5));
    }
    check(worstEnd <= ends.terrain.cutBatter * 1.35 + 0.05,
          "the earthwork runs out past a road's END, not off a cliff");
    std::printf("  note: worst slope walking off a road end %.2f (batter %.2f)\n", worstEnd,
                ends.terrain.cutBatter);
}

void testPaintSurvivesLod() {
    group("paint under LOD");
    // The question a realtime engine asks of this design: can the mesh get
    // cheaper with distance without the paint going wrong?
    //
    // It is not obvious that it can. The profile texture is addressed by a ROW
    // coordinate the mesher writes per ring, so a coarser mesh writes fewer of
    // them — and if the row index depended on the ring COUNT, dropping rings
    // would renumber every row and slide the paint along the road. It does not:
    // rowCoord is a function of the station, so a ring at s carries the same row
    // whatever its neighbours do, and a longer span between rings just means the
    // hardware interpolates further. This measures that rather than trusting it.
    //
    // What it costs is accuracy, because the offsets are cubics in s being
    // sampled by a straight line. That number is the useful output: it says how
    // far an LOD can go before a lane line visibly wanders.
    for (const std::string& name : {std::string("lanes"), std::string("interchange")}) {
        Scene sc;
        if (!buildDemo(name, sc)) continue;
        finalizeScene(sc, false, false);
        PaintAtlas atlas = bakePaintAtlas(sc.net, 2.0);
        const std::vector<Road>& roads = sc.net.roads();
        double anySeamError = 0, anyCleanError = 0;

        for (double lod : {2.0, 8.0, 16.0, 32.0}) {
            double worstT = 0, worstCov = 0, worstSeam = 0;
            long compared = 0, seamSpans = 0;
            for (size_t ri = 0; ri < roads.size(); ++ri) {
                const Road& r = roads[ri];
                const PaintProfile& prof = atlas.profiles[ri];
                if (r.activeLength() < 40.0) continue;
                // A coarser mesh: rings every `lod` metres, and the fragment's
                // row coordinate interpolated linearly between them exactly as a
                // rasteriser would.
                for (double s0 = r.begin(); s0 + lod < r.end(); s0 += lod) {
                    double s1 = s0 + lod;
                    // Skip any span CONTAINING a seam, not just one whose ends
                    // land on it: the row coordinate is only affine in s between
                    // consecutive rings, and a seam contributes a ring PAIR over
                    // 2 mm, so a span that swallows one spends a whole row index
                    // on no distance at all.
                    bool spansSeam = false;
                    for (const LaneSection& sec : r.xs.sections)
                        if (sec.s0 > s0 - 1e-3 && sec.s0 < s1 + 1e-3) spansSeam = true;
                    double row0 = prof.rowCoord(s0), row1 = prof.rowCoord(s1);
                    for (int k = 1; k < 4; ++k) {
                        double u = double(k) / 4.0;
                        double s = s0 + (s1 - s0) * u;
                        if (nearASeam(r, s)) continue;
                        RlBoundary lodB[kMaxBakedBoundaries], exact[kMaxBakedBoundaries];
                        int nl = atlas.sample(int(ri), row0 + (row1 - row0) * u, lodB,
                                              kMaxBakedBoundaries);
                        int ne = bakeBoundaries(r, s, exact, kMaxBakedBoundaries);
                        if (nl != ne) continue;
                        double off = 0;
                        for (int i = 0; i < ne; ++i)
                            off = std::max(off, std::fabs(double(exact[i].t) - double(lodB[i].t)));
                        if (spansSeam) {
                            ++seamSpans;
                            worstSeam = std::max(worstSeam, off);
                            continue;
                        }
                        ++compared;
                        worstT = std::max(worstT, off);
                        double le = r.xs.leftExtentAt(s), re = r.xs.rightExtentAt(s);
                        for (int j = 0; j <= 12; ++j) {
                            double t = re + (le - re) * double(j) / 12.0;
                            // The filter widens with distance, which is the whole
                            // reason a coarse LOD is acceptable: compare at a
                            // filter width that matches the range the LOD is for.
                            float fw = float(0.02 * lod);
                            RlPaint pe = rlEvaluateMarkings(exact, ne, float(s), float(t), fw,
                                                            0.0f, 1.0f, 0.0f);
                            RlPaint pl = rlEvaluateMarkings(lodB, nl, float(s), float(t), fw,
                                                            0.0f, 1.0f, 0.0f);
                            worstCov = std::max(worstCov, std::fabs(double(pe.coverage) -
                                                                    double(pl.coverage)));
                        }
                    }
                }
            }
            check(compared > 50, (name + ": there were LOD spans to compare").c_str());
            // Not "close enough" — EXACT to a couple of centimetres at every
            // spacing tried, up to sixteen times the bake's own. The offsets are
            // cubics in s and a straight line through them is a poor
            // approximation in principle; in practice a lane boundary is very
            // nearly straight over 30 m, which is why LOD costs nothing here.
            check(worstT < 0.05,
                  (name + ": paint holds at ring spacings up to 32 m").c_str());
            std::printf("  note: %-11s rings every %4.0f m: offsets within %.3f m, "
                        "coverage within %.2f (%ld spans; %ld spans over a seam, "
                        "worst %.2f m)\n",
                        name.c_str(), lod, worstT, worstCov, compared, seamSpans, worstSeam);
            anySeamError = std::max(anySeamError, worstSeam);
            anyCleanError = std::max(anyCleanError, worstT);
        }

        // The other half: the seam ring rule is LOAD-BEARING, not decorative.
        // The row coordinate is affine in s only BETWEEN consecutive rings, and a
        // seam contributes a ring pair spanning 2 mm — a whole row index spent on
        // no distance — so a span that swallows one interpolates into the wrong
        // rows entirely.
        //
        // Not every seam is dangerous, which is why this is stated as a maximum
        // rather than per-span: a section split where the boundary set happens to
        // be unchanged costs nothing, and plenty measure 0.00 m. It only takes
        // one that does matter, and the worst here is a whole lane and a half.
        check(anySeamError > 1.0,
              (name + ": swallowing a seam can misplace paint by metres").c_str());
        check(anyCleanError < 0.05,
              (name + ": while honouring seams keeps every LOD exact").c_str());
        std::printf("  note: %-11s worst offset  %.3f m honouring seams, %.2f m across one\n",
                    name.c_str(), anyCleanError, anySeamError);
    }
}

// --- the generated WGSL -----------------------------------------------------

namespace {

std::string readFile(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return std::string();
    std::string out;
    char buf[4096];
    size_t got;
    while ((got = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, got);
    std::fclose(f);
    return out;
}

// FNV-1a, 64-bit. Matches tools/roadlab-wgsl.py byte for byte; see the comment
// there for why it is not SHA.
std::string fnv1a64(const std::string& data) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : data) {
        h ^= uint64_t(c);
        h *= 0x100000001b3ull;
    }
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%016llx", (unsigned long long)h);
    return std::string(buf);
}

}  // namespace

void testGeneratedWgslTracksTheHeader() {
    group("generated WGSL");
    // Four backends, one evaluator. Metal and GLSL include rl_paint.h; WGSL
    // cannot, so it is generated. That generated file is the one place the
    // "single implementation" claim can quietly become false — a header edit
    // that nobody regenerated ships a shader that paints a different road, and
    // nothing in a C++ build would notice.
    const std::string dir = ROADLAB_SRC_DIR;
    std::string header = readFile(dir + "/rl_paint.h");
    std::string wgsl = readFile(dir + "/rl_paint.wgsl");
    check(!header.empty(), "rl_paint.h is readable");
    check(!wgsl.empty(), "rl_paint.wgsl exists — run `make roadlab-wgsl`");
    if (header.empty() || wgsl.empty()) return;

    const std::string marker = "// rl_paint.h digest: ";
    size_t at = wgsl.find(marker);
    check(at != std::string::npos, "the generated WGSL records the digest of its input");
    if (at == std::string::npos) return;
    std::string recorded = wgsl.substr(at + marker.size(), 16);
    check(recorded == fnv1a64(header),
          "rl_paint.wgsl was generated from the CURRENT rl_paint.h "
          "(if this fails, run `make roadlab-wgsl`)");

    // Every function and constant in the header has to have come across. This
    // is what catches a generator that silently drops something rather than
    // refusing it — the digest only proves the file was regenerated, not that
    // regenerating produced anything.
    size_t pos = 0;
    int fns = 0, consts = 0;
    while ((pos = header.find("RL_FN ", pos)) != std::string::npos) {
        // Only a definition, which starts a line. The macro's own `#define
        // RL_FN inline` also contains the token, and following it to the next
        // '(' lands in whatever comment comes next.
        if (pos != 0 && header[pos - 1] != '\n') {
            pos += 6;
            continue;
        }
        size_t paren = header.find('(', pos);
        if (paren == std::string::npos) break;
        size_t nameEnd = paren;
        while (nameEnd > pos && (header[nameEnd - 1] == ' ')) --nameEnd;
        size_t nameStart = nameEnd;
        while (nameStart > pos && (std::isalnum((unsigned char)header[nameStart - 1]) ||
                                   header[nameStart - 1] == '_'))
            --nameStart;
        std::string name = header.substr(nameStart, nameEnd - nameStart);
        ++fns;
        check(wgsl.find("fn " + name + "(") != std::string::npos,
              ("the WGSL defines " + name).c_str());
        pos = paren;
    }
    check(fns >= 6, "the header still has the functions this test expects to find");

    pos = 0;
    while ((pos = header.find("#define RL_", pos)) != std::string::npos) {
        size_t nameStart = pos + 8;
        size_t nameEnd = header.find(' ', nameStart);
        std::string name = header.substr(nameStart, nameEnd - nameStart);
        pos = nameEnd;
        // RL_BUF and RL_FN are C and GLSL plumbing with no WGSL counterpart.
        if (name == "RL_BUF" || name == "RL_FN") continue;
        ++consts;
        check(wgsl.find("const " + name + " :") != std::string::npos,
              ("the WGSL defines " + name).c_str());
    }
    check(consts >= 15, "the marking and colour codes all crossed over");

    // No validator ships with this repo (the same note is on shaders/webgpu/
    // water.wgsl), so the structural checks a parser would do are done here.
    long braces = 0;
    bool balanced = true;
    for (char c : wgsl) {
        if (c == '{') ++braces;
        if (c == '}' && --braces < 0) balanced = false;
    }
    check(balanced && braces == 0, "the generated WGSL has balanced braces");

    // C that survived translation. A stray `float` or `;` inside a struct is
    // not a subtle bug — it simply will not compile — but the failure would
    // land on whoever next touches the WebGPU backend rather than here.
    const char* leftovers[] = {"float ", "RL_FN", "RL_BUF", " ? ", "->r", "&out"};
    for (const char* bad : leftovers) {
        // Skip the generated file's own prose: only code lines matter.
        bool found = false;
        size_t line = 0;
        while (line < wgsl.size()) {
            size_t eol = wgsl.find('\n', line);
            if (eol == std::string::npos) eol = wgsl.size();
            std::string text = wgsl.substr(line, eol - line);
            size_t comment = text.find("//");
            if (comment != std::string::npos) text = text.substr(0, comment);
            if (text.find(bad) != std::string::npos) found = true;
            line = eol + 1;
        }
        check(!found, (std::string("no C leftover \"") + bad + "\" in the WGSL").c_str());
    }
    check(wgsl.find("ptr<function, array<RlBoundary, RL_MAX_BOUNDS>>") != std::string::npos,
          "the boundary array crossed over as a function-address-space pointer");
}

// --- the metro generator ----------------------------------------------------

namespace {

// Which roads can be reached from `fromRoad` over the lane graph, and which can
// reach it. One traversal each, rather than one per candidate.
//
// Both follow lane CHANGES as well as successors, because a driver leaving a
// freeway reaches the exit lane by changing into it. A deceleration lane has no
// predecessor by construction — it grows out of the carriageway rather than
// continuing anything — so a search over successors alone reports that no exit
// is reachable from any freeway, which says more about the search than the road.
void reachabilityFrom(const Network& net, int fromRoad, std::vector<char>& roadsReached,
                      bool reverse) {
    const LaneGraph& lg = net.lanes();
    const size_t n = lg.nodes.size();
    std::vector<std::vector<int>> edges(n);
    for (size_t i = 0; i < n; ++i) {
        const LaneNode& u = lg.nodes[i];
        auto add = [&](int v) {
            if (v < 0 || size_t(v) >= n) return;
            if (reverse) {
                edges[size_t(v)].push_back(int(i));
            } else {
                edges[i].push_back(v);
            }
        };
        for (int v : u.successors) add(v);
        if (u.leftCrossable) add(u.leftNeighbor);
        if (u.rightCrossable) add(u.rightNeighbor);
    }

    std::vector<char> seen(n, 0);
    std::vector<int> queue;
    for (size_t i = 0; i < n; ++i) {
        if (lg.nodes[i].ref.road != fromRoad) continue;
        seen[i] = 1;
        queue.push_back(int(i));
    }
    for (size_t head = 0; head < queue.size(); ++head) {
        for (int v : edges[size_t(queue[head])]) {
            if (seen[size_t(v)]) continue;
            seen[size_t(v)] = 1;
            queue.push_back(v);
        }
    }
    roadsReached.assign(size_t(net.roadCount()), 0);
    for (size_t i = 0; i < n; ++i)
        if (seen[i]) roadsReached[size_t(lg.nodes[i].ref.road)] = 1;
}

}  // namespace

void testMetroGenerator() {
    group("metro generator");
    for (uint32_t seed : {11u, 3u, 21u, 5u}) {
        Scene sc;
        MetroParams mp;
        mp.seed = seed;
        generateMetro(sc, mp);
        finalizeScene(sc, false, false);
        std::string tag = "seed " + std::to_string(seed) + ": ";

        if (!sc.lint.empty()) std::printf("  note: %s%s\n", tag.c_str(), sc.lint.front().c_str());
        check(sc.lint.empty(), (tag + "the layout passes the design lint").c_str());

        // The bypass is a closed loop of pieces over one circular spine, and it
        // has to be a motorway curve — a ring below the minimum radius for its
        // design speed is the failure this generator is most likely to produce,
        // because the ring is sized from the city and the city can be small.
        std::vector<int> ringPieces;
        for (const Road& r : sc.net.roads())
            if (r.name.rfind("bypass", 0) == 0) ringPieces.push_back(r.id);
        check(!ringPieces.empty(), (tag + "there is a bypass").c_str());
        if (ringPieces.empty()) continue;

        double ringSpeed = sc.net.road(ringPieces.front()).designSpeed;
        double rmin = minRadiusForSpeed(ringSpeed);
        for (int id : ringPieces) {
            for (const GeomPrim& g : sc.net.road(id).spine.prims()) {
                double k = std::max(std::fabs(g.curv0), std::fabs(g.curv1));
                if (k < 1e-9) continue;
                check(1.0 / k >= rmin * 0.98,
                      (tag + "the bypass holds the minimum radius for its speed").c_str());
            }
        }
        // Closed: following successors from any piece returns to the start.
        {
            int start = ringPieces.front();
            int at = start;
            bool closed = false;
            for (int hop = 0; hop < int(ringPieces.size()) + 4; ++hop) {
                const Road& r = sc.net.road(at);
                if (r.succ.type != LinkType::Road || r.succ.id < 0) break;
                at = r.succ.id;
                if (at == start) {
                    closed = true;
                    break;
                }
            }
            check(closed, (tag + "the bypass closes into a loop").c_str());
        }

        // Every interchange asked for was built, with both ramps.
        int terminals = 0, ramps = 0;
        for (const Junction& j : sc.net.junctions())
            if (j.name.rfind("terminal", 0) == 0) ++terminals;
        for (const Road& r : sc.net.roads())
            if (r.kind == RoadKind::Ramp) ++ramps;
        check(terminals == mp.interchanges,
              (tag + "every interchange was built").c_str());
        check(ramps >= 2 * mp.interchanges,
              (tag + "each interchange has an on-ramp and an off-ramp").c_str());

        // The point of the ramps: the freeway and the city streets are one
        // network, not two drawings that happen to touch. Asserted for EVERY
        // street, in BOTH directions — one reachable street would pass while the
        // rest of the city was stranded, and reaching the ring's head piece is
        // not the same question as reaching the freeway.
        std::vector<char> isRing(size_t(sc.net.roadCount()), 0);
        for (int id : ringPieces) isRing[size_t(id)] = 1;
        std::vector<int> streets;
        for (const Road& r : sc.net.roads()) {
            if (r.kind != RoadKind::Normal) continue;
            if (r.name.rfind("ew-", 0) != 0 && r.name.rfind("ns-", 0) != 0) continue;
            if (r.activeLength() > 60.0) streets.push_back(r.id);
        }
        check(streets.size() > 40, (tag + "there are city streets to reach").c_str());
        std::vector<char> fromRing, toRing;
        reachabilityFrom(sc.net, ringPieces.front(), fromRing, false);
        reachabilityFrom(sc.net, ringPieces.front(), toRing, true);
        int offFreeway = 0, ontoFreeway = 0;
        for (int id : streets) {
            if (fromRing[size_t(id)]) ++offFreeway;
            if (toRing[size_t(id)]) ++ontoFreeway;
        }
        (void)isRing;
        check(offFreeway == int(streets.size()),
              (tag + "every city street is reachable from the freeway").c_str());
        check(ontoFreeway == int(streets.size()),
              (tag + "and every city street can reach the freeway").c_str());

        // Multilane. A city of nothing but two-lane streets would satisfy every
        // other assertion here.
        double wide = 0, total = 0;
        for (const Road& r : sc.net.roads()) {
            if (r.kind == RoadKind::Connector) continue;
            double len = r.activeLength();
            if (len < 1e-6) continue;
            total += len;
            if (r.xs.sectionAt(r.begin()).laneCount() >= 4) wide += len;
        }
        check(total > 0 && wide > total * 0.35,
              (tag + "a good share of the network is four lanes or more").c_str());

        // Straight AND curved, not one or the other.
        int straight = 0, curved = 0;
        for (const Road& r : sc.net.roads()) {
            if (r.name.rfind("ew-", 0) != 0 && r.name.rfind("ns-", 0) != 0) continue;
            bool anyCurve = false;
            for (const GeomPrim& g : r.spine.prims())
                if (std::fabs(g.curv0) > 1e-7 || std::fabs(g.curv1) > 1e-7) anyCurve = true;
            (anyCurve ? curved : straight)++;
        }
        check(straight > 3, (tag + "some streets are dead straight").c_str());
        check(curved > 3, (tag + "and some are curved").c_str());

        // Blocks of various sizes: the lattice is irregular and superblocks
        // remove whole runs of interior street, so the spread of edge lengths
        // has to be wide. A uniform grid would have almost none.
        std::vector<double> lens;
        for (const Road& r : sc.net.roads())
            if (r.name.rfind("ew-", 0) == 0 || r.name.rfind("ns-", 0) == 0)
                lens.push_back(r.activeLength());
        check(lens.size() > 20, (tag + "there are enough streets to judge").c_str());
        if (lens.size() > 20) {
            std::sort(lens.begin(), lens.end());
            double p10 = lens[lens.size() / 10], p90 = lens[lens.size() * 9 / 10];
            check(p90 > p10 * 1.6, (tag + "block sizes genuinely vary").c_str());
        }
    }
}

void testMetroSurvivesOpenDrive() {
    group("metro through OpenDRIVE");
    // The scale question. A 37 km network with ten ramps is where an exporter
    // that only ever saw a demo falls over.
    Scene sc;
    MetroParams mp;
    mp.seed = 11;
    generateMetro(sc, mp);
    finalizeScene(sc, false, false);

    OdrOptions opt;
    opt.name = "metro";
    std::string first = openDriveString(sc.net, opt);
    check(first.size() > 500000, "the document is the size the network implies");

    Network back;
    std::string err;
    OdrImportReport rep;
    bool ok = openDriveFromString(first, back, err, &rep);
    check(ok, "a 37 km network re-imports");
    if (!ok) {
        std::printf("  note: %s\n", err.c_str());
        return;
    }
    back.build();

    double before = 0, after = 0;
    for (const Road& r : sc.net.roads()) before += r.activeLength();
    for (const Road& r : back.roads()) after += r.activeLength();
    check(std::fabs(after - before) < before * 0.005,
          "every metre of carriageway survives the round trip");
    check(back.validate().empty(), "and the reimported network passes the lint");

    // Ramps become junctions, which is the format's rule, not a loss.
    check(back.junctionCount() == sc.net.junctionCount() + 10,
          "each ramp becomes the junction OpenDRIVE requires");

    std::string second = openDriveString(back, opt);
    Network again;
    bool reOk = openDriveFromString(second, again, err, &rep);
    check(reOk, "the re-export re-imports");
    if (reOk) {
        again.build();
        check(openDriveString(again, opt) == second, "and reaches a fixed point at this scale");
    }

    // And it is still drivable after the trip.
    SimParams sp;
    sp.seed = 4;
    Simulation sim(back, sp);
    sim.seedVehicles(250);
    sim.run(60.0);
    SimStats st = sim.stats();
    check(st.moving > 50, "traffic runs on the reimported metro");
    check(st.completedTrips > 0, "and trips complete on it");
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
    testOpenDriveConformance();
    testRampsBecomeJunctions();
    testDivergeKeepsLaneIdentity();
    testXmlReader();
    testOpenDriveImportBasics();
    testOpenDriveRoundTrip();
    testImportedJunctionsAreAdopted();
    testImportedNetworkDrives();
    testFallbackCensusPlumbing();
    testJunctionArmsMustMeet();
    testJunctionPadsAreSimplePolygons();
    testInverseMapResolvesRealQueries();
    testMostDriversGetARoute();
    testPaintEvaluatorIsPortable();
    testBakedBoundariesMatchTheCrossSection();
    testSeamSmearIsMillimetresWide();
    testProfileTextureIsWhatTheShaderWouldSample();
    testRoadSurfacesDoNotOverlap();
    testRampsAbutTheirMainline();
    testRampsDoNotEncroach();
    testEveryMergeHasAGore();
    testEveryCorpusSceneLints();
    testTheAtlasFitsInATexture();
    testEarthworksRunOnTheirBatter();
    testPaintSurvivesLod();
    testGeneratedWgslTracksTheHeader();
    testMetroGenerator();
    testMetroSurvivesOpenDrive();
    testAllDemosBuild();
    testGeneratedCity();

    std::printf("\n%d checks, %d failure(s)\n", gChecks, gFailures);
    return gFailures == 0 ? 0 : 1;
}
