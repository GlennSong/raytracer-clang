#include "test_framework.h"
#include "mesh_invariants.h"

#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/road_offset.h"

#include <limits>

using namespace engine;
using namespace mesh_invariants;

namespace {

// A degree-N junction: a centre node at the origin with `dirs` arms radiating to
// `armLen`. `widths`/`classes` parallel the arms (missing/zero = the look's
// default) — resolved onto each edge at construction, as the graph requires.
RoadEntity junction(const std::vector<Vec2>& dirs, double armLen,
                 std::vector<double> widths = {},
                 std::vector<RoadClass> classes = {}) {
    RoadEntity n;
    n.graph.nodes.push_back(RoadNode{Vec2(0, 0)});   // centre = node 0
    for (std::size_t i = 0; i < dirs.size(); ++i) {
        Vec2 d = dirs[i];
        double l = d.length();
        Vec2 u = l > 1e-9 ? d * (1.0 / l) : Vec2(1, 0);
        n.graph.nodes.push_back(RoadNode{u * armLen});
        const double w = (i < widths.size() && widths[i] > 0.0) ? widths[i]
                                                                : n.look.defaultWidth;
        const RoadClass k = i < classes.size() ? classes[i] : RoadClass::Local;
        n.graph.addEdge(0, static_cast<int>(i + 1), w, k);
    }
    return n;
}

// Assert the universal weld invariants on a fixture's meshed output.
void checkWeld(const RoadEntity& net, const char* /*label*/) {
    RenderMesh m = buildRoadNetMesh(net, nullptr);
    CHECK(triangleCount(m) > 0);                   // something was welded
    CHECK(!hasNonFinite(m));                       // no NaN/Inf blow-up
    CHECK(indicesInRange(m));                      // every index is real
    CHECK(degenerateTriangles(m) == 0);            // no zero-area / dup-index tris
    // A real up-facing deck exists (not flipped). Valid junctions run ~0.37-0.71
    // up (acute corners add sidewall, lowering it); a flipped/missing deck
    // collapses toward ~0.15, which this catches.
    CHECK(upwardFraction(m) > 0.3);
    // vertices stay within a sane box around the ~90 m fixture (no vertex flung off)
    double minX, maxX, minZ, maxZ;
    bboxXZ(m, minX, maxX, minZ, maxZ);
    CHECK(minX > -200.0 && maxX < 200.0);
    CHECK(minZ > -200.0 && maxZ < 200.0);
}

}  // namespace

// A — a straight through road (degree-2, one chain).
TEST_CASE(weld_straight_chain_is_clean) {
    RoadEntity n;
    n.graph.nodes = { RoadNode{Vec2(-40, 0)}, RoadNode{Vec2(40, 0)} };
    n.graph.addEdge(0, 1, n.look.defaultWidth);
    checkWeld(n, "straight");
}

// B — a T-junction (degree-3).
TEST_CASE(weld_tee_is_clean) {
    checkWeld(junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0, 1)}, 40.0), "tee");
}

// C — a 4-way cross (degree-4).
TEST_CASE(weld_cross_is_clean) {
    checkWeld(junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0, -1), Vec2(0, 1)}, 40.0), "cross");
}

// D — a skewed cross (arms at odd angles) — the acute-corner stress case.
TEST_CASE(weld_skewed_cross_is_clean) {
    checkWeld(junction({Vec2(-1, 0.3), Vec2(1, 0.1), Vec2(-0.2, -1), Vec2(0.4, 1)}, 40.0),
              "skewed");
}

// E — WIDE meets NARROW: a wide arterial through-road crossed by a narrow local.
// The mixed-width junction that drove the crosswalk-jut; assert its weld is clean.
TEST_CASE(weld_wide_meets_narrow_is_clean) {
    RoadEntity n = junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0, 1)}, 40.0,
                         /*widths*/ {13.0, 13.0, 7.0},
                         /*classes*/ {RoadClass::Arterial, RoadClass::Arterial,
                                      RoadClass::Local});
    checkWeld(n, "wide_meets_narrow");
}

// F — a very ACUTE Y: two arms nearly parallel (~15 deg apart). The audit flags
// acute corners as the sidewalk-tear / ground-wedge case; assert the weld stays
// geometrically clean here too.
TEST_CASE(weld_acute_y_is_clean) {
    checkWeld(junction({Vec2(1, 0), Vec2(1, 0.27), Vec2(-1, 0.05)}, 40.0), "acute_y");
}

// G — a shallow-angle T: the stem meets the through road at ~20 deg (a slip road
// tee-ing into a street), the classic acute-junction stress.
TEST_CASE(weld_shallow_tee_is_clean) {
    checkWeld(junction({Vec2(-1, 0), Vec2(1, 0), Vec2(0.94, 0.34)}, 40.0), "shallow_tee");
}

// (weld_crosswalk_clears_skewed_arm deleted with the weld: the lattice bakes
// the crosswalk window from the junction TRIM distance, which is radius-based
// rather than skew-aware — an accepted simplification recorded at the S6
// deletion. Crosswalk gating itself is covered in test_road_net.)

// --- 3-D channel (welder-goes-3D): an ELEVATED deck rides its authored
// absolute Y through the SAME welder that meshes streets. This is the unit-level
// proof that one mesher can carry a corridor/ramp — the fork's whole reason for
// existing (a single-valued drape can't lift a deck) is what UnionSpine.yAbs and
// weldChainProfiles' authored-height path retire.

// H — weldChainProfiles hands back the authored deck Y UNTOUCHED: not draped to
// terrain, not pulled to a junction min, not offset by topY. A spine at a
// constant +9 m must come back at +9 m regardless of the ground beneath it.
TEST_CASE(weld_authored_deck_rides_its_own_height) {
    UnionSpine s;
    s.points = { Vec2(-40, 0), Vec2(0, 0), Vec2(40, 0) };
    s.halfWidth = 6.0;
    s.klass = RoadClass::Freeway;
    s.yAbs = { 9.0, 9.0, 9.0 };                    // absolute deck Y
    // A sloped ground far BELOW the deck: drape would put the road here, not +9.
    auto ground = [](double x, double) { return -20.0 + 0.05 * x; };
    auto profs = weldChainProfiles({ s }, ground, /*topY*/ 0.06, /*maxGrade*/ 0.08);
    CHECK(profs.size() == 1);
    CHECK(profs[0].size() == 3);
    for (double h : profs[0]) CHECK(std::fabs(h - 9.0) < 1e-6);   // rode yAbs exactly
}

// I — a draped spine in the SAME batch still drapes: the authored-height path is
// per-spine, so mixing an elevated deck with an at-grade street leaves the street
// on the ground. Proves the 3-D path is additive, not a global mode switch.
TEST_CASE(weld_authored_and_draped_spines_coexist) {
    UnionSpine deck;
    deck.points = { Vec2(-40, 0), Vec2(40, 0) };
    deck.halfWidth = 6.0;
    deck.yAbs = { 9.0, 9.0 };
    UnionSpine street;                             // no yAbs -> drapes
    street.points = { Vec2(0, -40), Vec2(0, 40) };
    street.halfWidth = 4.0;
    auto ground = [](double, double) { return 2.5; };
    auto profs = weldChainProfiles({ deck, street }, ground, 0.06, 0.08);
    CHECK(profs.size() == 2);
    for (double h : profs[0]) CHECK(std::fabs(h - 9.0) < 1e-6);   // deck: authored
    for (double h : profs[1]) CHECK(std::fabs(h - (2.5 + 0.06)) < 1e-6);  // street: drape+topY
}

// K — the WHOLE authoring path: a RoadEntity with per-node absolute elevation
// (nodeElev) is meshed by buildRoadNetMesh — the SAME entry point streets use —
// into an elevated deck. Proves an author (or a future corridor publisher) can
// express a 3-D road as graph data and the one welder builds it; no separate
// bridge mesher, no layer>0 bail.
TEST_CASE(road_net_meshes_an_authored_elevated_span) {
    RoadEntity n;
    n.graph.nodes = { RoadNode{Vec2(-40, 0)}, RoadNode{Vec2(40, 0)} };
    for (RoadNode& nd : n.graph.nodes) {           // both ends at +9 -> elevated span
        nd.elev = 9.0;
        nd.elevAbsolute = true;
    }
    n.graph.addEdge(0, 1, n.look.defaultWidth);
    const RoadGroundFn ground = [](double, double) { return -12.0; };   // ground far below
    RenderMesh m = buildRoadNetMesh(n, ground);
    CHECK(triangleCount(m) > 0);
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    CHECK(upwardFraction(m) > 0.3);
    double minY, maxY;
    bboxY(m, minY, maxY);
    CHECK(maxY > 8.0 && maxY < 10.0);              // deck rode to the authored +9
    CHECK(minY < -8.0);                            // piers reach down toward the -12 ground
}

// P1 (one mesher) — a FREEWAY-class deck gets divided-highway BARRIERS from the
// SAME welder that meshes streets: edge parapets + a solid median box standing
// above the deck. A Local street of the identical shape gets NONE. This is the
// weld gaining a freeway feature that used to live only in corridor_mesh.
TEST_CASE(weld_freeway_deck_gets_barriers) {
    auto makeNet = [](RoadClass k) {
        RoadEntity n;
        n.look.defaultWidth = 22.0; n.look.sidewalk = 0.0;
        n.look.markings = true; n.look.autoRoundabout = false;
        const double elev[9] = { 0, 3, 7, 9, 9, 9, 7, 3, 0 };  // elevated span, +9 in the middle
        for (int i = 0; i <= 8; ++i) {
            RoadNode nd;
            nd.pos = Vec2(-140 + i * 35, 0);
            nd.elev = elev[i];
            nd.elevAbsolute = true;
            n.graph.nodes.push_back(nd);
        }
        for (int i = 0; i < 8; ++i) n.graph.addEdge(i, i + 1, n.look.defaultWidth, k);
        return n;
    };
    RenderMesh fw = buildRoadNetMesh(makeNet(RoadClass::Freeway), nullptr);
    RenderMesh st = buildRoadNetMesh(makeNet(RoadClass::Local), nullptr);
    CHECK(!hasNonFinite(fw));
    CHECK(degenerateTriangles(fw) == 0);
    double fwMinY, fwMaxY, stMinY, stMaxY;
    bboxY(fw, fwMinY, fwMaxY); bboxY(st, stMinY, stMaxY);
    // Count vertices standing clearly above the deck: barrier walls on the
    // freeway, none on the street of the same shape.
    auto above = [](const RenderMesh& m, double y) {
        int c = 0; for (const Vertex& v : m.vertices) if (v.position.y > y) ++c; return c;
    };
    CHECK(fwMaxY > 9.3);
    CHECK(stMaxY < 9.3);                                // a Local deck tops out at the asphalt (+9)
    CHECK(above(fw, 9.3) > 0);
    CHECK(above(st, 9.3) == 0);
}

// P8b (one mesher) — corridorAuthor is now the corridor's ONLY non-geometry
// output: the nav graph AND the weld's ramp spines are both built from its
// rampPaths, and the terrain grades to its flatten windows. This began as an
// equivalence ratchet against the corridor sweep mesher; that mesher is deleted
// (P8b-2), so what it was really pinning is stated directly — the drop rules fire,
// the paths stay index-parallel to the exits, and the at-grade ends carve.
// Exercised on a corridor with real exits, on-ramps, hilly ground, and a ramp
// aimed so it MUST drop — so the drop rules are covered, not just happy paths.
TEST_CASE(corridor_author_authors_ramp_paths_and_flatten) {
    CorridorDef c;
    c.horizontal = Alignment::fromPolyline(
        { Vec2(-400, 0), Vec2(0, 0), Vec2(500, 0) }, 260.0, 40.0);
    const Real L = c.horizontal.length();
    // Ends AT GRADE (flatten carves), middle flown on a viaduct (no carve) —
    // so the elevated[] gate is exercised in both states.
    c.vertical.pvis = { {0.0, 0.0, 0.0}, {L * 0.5, 12.0, 120.0}, {L, 0.0, 0.0} };
    c.lanes.throughLanes = 3;
    {   // Exits on BOTH sides so one survives whichever side upStation picks,
        // plus one aimed onto the mainline itself so a DROP rule fires.
        ExitDef a; a.station = 300.0; a.upStation = true;
        a.target = Vec2(150, -180); a.targetY = 0.0;
        c.exits.push_back(a);
        ExitDef b; b.station = 300.0; b.upStation = false;
        b.target = Vec2(150, 180); b.targetY = 0.0;
        c.exits.push_back(b);
        ExitDef o; o.station = 620.0; o.upStation = false; o.onRamp = true;
        o.target = Vec2(430, 190); o.targetY = 0.0;
        c.exits.push_back(o);
        ExitDef bad; bad.station = 500.0; bad.upStation = true;
        bad.target = Vec2(505, 2);           // sits on the mainline: must drop
        bad.targetY = 0.0;
        c.exits.push_back(bad);
    }
    auto ground = [](Real x, Real z) {
        return 1.0 * std::sin(x * 0.01) + 0.8 * std::cos(z * 0.013);   // gentle hills
    };
    CorridorAuthoring au = corridorAuthor(c, ground, 3.0);

    // rampPaths are STRICTLY index-parallel to exits — the loader indexes
    // rampPaths[exitIndex], so a dropped ramp must leave an EMPTY entry rather
    // than shrink the vector, or every later ramp silently mis-pairs.
    CHECK(au.rampPaths.size() == c.exits.size());
    bool sawDrop = false, sawPath = false;
    for (std::size_t i = 0; i < au.rampPaths.size(); ++i) {
        const std::vector<Vec3>& pts = au.rampPaths[i].pts;
        if (pts.empty()) { sawDrop = true; continue; }
        sawPath = true;
        CHECK(pts.size() >= 4);              // nav needs a real chain
        for (const Vec3& q : pts) {
            CHECK(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z));
        }
        // FLOW order with absolute heights: an exit leaves the deck for the
        // street, an on-ramp arrives from it. Either way one end is up on the
        // structure and the other is down at the target's grade.
        const double y0 = pts.front().y, y1 = pts.back().y;
        CHECK(std::fabs(y0 - y1) > 1.0);
        const Vec3& streetEnd = c.exits[i].onRamp ? pts.front() : pts.back();
        CHECK((Vec2(streetEnd.x, streetEnd.z) - c.exits[i].target).length() < 30.0);
    }
    CHECK(sawPath);   // the fixture really does author ramps...
    CHECK(sawDrop);   // ...and the ramp aimed at the mainline really does drop

    // flatten: the AT-GRADE ends carve to the deck plane; the flown middle must
    // not (the ground stays under a viaduct instead of following it up).
    CHECK(!au.flatten.empty());
    for (const TerrainFlatten& f : au.flatten) {
        CHECK(std::isfinite(f.c) && std::isfinite(f.dx) && std::isfinite(f.dz));
        CHECK(f.polygon.size() >= 3);
        // No window may carve up at the viaduct: the profile crests at 12 m
        // mid-corridor, and a carve plane anywhere near that height would mean
        // the terrain is being dragged up to meet a flown deck.
        CHECK(f.c < 6.0);
    }
}

// P5 (one mesher) — VARIABLE-WIDTH ribbon outline: a centreline whose per-vertex
// half-width grows produces a FLARING ribbon (the aux-lane / gore widening). The
// hard piece that lets a freeway deck diverge instead of running one width.
TEST_CASE(variable_ribbon_outline_flares) {
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(30, 0), Vec2(60, 0) };
    std::vector<double> hw = { 3.0, 3.0, 8.0 };            // widens toward the end
    Poly2 out = ribbonOutline(cl, hw);
    CHECK(out.size() >= 4);
    double maxLat = 0.0, startLat = 0.0;
    for (const Vec2& p : out) {
        maxLat = std::max(maxLat, std::fabs(p.y));
        if (std::fabs(p.x) < 1.0) startLat = std::max(startLat, std::fabs(p.y));
    }
    CHECK(maxLat > 7.0 && maxLat < 9.0);      // flared to the hw 8 end
    CHECK(startLat > 2.0 && startLat < 4.0);  // still hw 3 at the start
}

// L — a NaN/short nodeElev leaves the road at grade: authoring is opt-in and
// per-node, so an ordinary street with no elevation drapes exactly as before.
TEST_CASE(road_net_without_node_elev_stays_at_grade) {
    RoadEntity n;
    n.graph.nodes = { RoadNode{Vec2(-40, 0)}, RoadNode{Vec2(40, 0)} };
    n.graph.addEdge(0, 1, n.look.defaultWidth);
    const RoadGroundFn ground = [](double, double) { return 3.0; };
    RenderMesh m = buildRoadNetMesh(n, ground);
    double minY, maxY;
    bboxY(m, minY, maxY);
    CHECK(maxY < 4.0);                             // near the 3.0 ground (+ lift), not lifted
}

// --- Grade separation (P4): a deck flies OVER a live road ---------------------
// The 2-D weld used to fuse a deck and the street it crosses into one surface +
// a curtain wall to the ground. weldSolid now welds each grade group separately,
// so the deck and the street beneath it are two independent surfaces with clear
// air between. Height-band assertions are robust to the thin marking overlay.


namespace {
// The grade-separation fixture, rebuilt on the ONE mesher: a street at grade
// along X and an elevated deck (+9 absolute) along Z crossing over it — no
// shared node, so the lattice meshes two independent chains, gives the deck
// an underside and piers (which must dodge the street below), and leaves
// clear air between the two surfaces.
RenderMesh flyover() {
    RoadGraph g;
    auto node = [&](double x, double z, double e, bool abs) {
        RoadNode n;
        n.pos = Vec2(x, z);
        n.elev = e;
        n.elevAbsolute = abs;
        g.nodes.push_back(n);
        return static_cast<int>(g.nodes.size() - 1);
    };
    const int s0 = node(-60, 0, 0, false), s1 = node(60, 0, 0, false);
    const int d0 = node(0, -60, 9, true), d1 = node(0, 60, 9, true);
    auto edge = [&](int a, int b) {
        RoadEdge e;
        e.a = a;
        e.b = b;
        e.width = 8.0;
        g.edges.push_back(e);
    };
    edge(s0, s1);
    edge(d0, d1);
    return buildRoadNetLattice(g, nullptr);
}
}  // namespace

// N — two surfaces over the crossing, clear air between (not one merged sheet).
TEST_CASE(gradesep_two_surfaces_over_the_crossing) {
    RenderMesh m = flyover();
    CHECK(!hasNonFinite(m));
    CHECK(indicesInRange(m));
    CHECK(degenerateTriangles(m) == 0);
    CHECK(upwardFraction(m) > 0.3);
    int low = 0, high = 0, mid = 0;
    for (double h : surfaceHitsAt(m, 0.0, 0.0)) {
        if (h < 0.6) ++low; else if (h > 8.0) ++high; else ++mid;
    }
    CHECK(low >= 1);                                // the street at grade
    CHECK(high >= 1);                               // the deck aloft
    CHECK(mid == 0);                                // nothing merged in between
    CHECK(hasClearSpanAt(m, 0.0, 0.0, 0.6, 8.0));   // clear air / sky under the deck
}

// O — no curtain wall: no geometry in the mid-height band over the crossing.
TEST_CASE(gradesep_no_curtain_wall) {
    RenderMesh m = flyover();
    CHECK(verticesInBox(m, -5.0, 5.0, 1.0, 8.0, -5.0, 5.0) == 0);
}

// P — the street is continuous UNDER the deck and the deck continuous OVER it.
TEST_CASE(gradesep_continuous_under_and_over) {
    RenderMesh m = flyover();
    for (double x = -30; x <= 30; x += 5.0) {       // march the street under the deck
        bool grade = false;
        for (double h : surfaceHitsAt(m, x, 0.0)) if (h < 1.0) grade = true;
        CHECK(grade);
    }
    for (double z = -30; z <= 30; z += 5.0) {       // march over the deck
        bool deck = false;
        for (double h : surfaceHitsAt(m, 0.0, z)) if (h > 8.0) deck = true;
        CHECK(deck);
    }
}

// Q — piers straddle the street: columns on both approaches, none on the road.
// Detect piers by their SHAFTS (mid-height vertices near the deck centreline) —
// the old ground-level proxy now also catches the street's own curb-lip bases,
// which sit at y = 0 beside the carriageway by design (S5 band ownership).
TEST_CASE(gradesep_piers_straddle_the_street) {
    RenderMesh m = flyover();
    int onRoad = 0, north = 0, south = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const double minY = std::min({a.y, b.y, c.y});
        const double maxY = std::max({a.y, b.y, c.y});
        const double cx = (a.x + b.x + c.x) / 3.0, cz = (a.z + b.z + c.z) / 3.0;
        // PORTAL bents (roads-v2.1 2d): two legs at the deck edges — shafts
        // sit near |x| = halfWidth-1.2, not on the centreline.
        if (maxY - minY < 5.0 || std::fabs(cx) > 6.0) continue;   // shaft faces
        if (std::fabs(cz) < 6.0) ++onRoad; else if (cz < -6.0) ++north; else ++south;
    }
    CHECK(onRoad == 0);
    CHECK(north > 0);
    CHECK(south > 0);
}

// R — end-to-end through buildRoadNetMesh: an authored elevated E-W road and a
// SEPARATE at-grade N-S street (no shared node) grade-separate at their crossing.
TEST_CASE(gradesep_end_to_end_via_road_net) {
    RoadEntity n;
    n.graph.nodes = { RoadNode{Vec2(-40, 0)}, RoadNode{Vec2(0, 0)},
                      RoadNode{Vec2(40, 0)},             // E-W: 0,1,2 (deck at +9)
                      RoadNode{Vec2(0, -40)}, RoadNode{Vec2(0, 40)} };  // N-S: 3,4 (at grade)
    for (int i = 0; i < 3; ++i) {
        n.graph.nodes[i].elev = 9.0;
        n.graph.nodes[i].elevAbsolute = true;
    }
    n.graph.addEdge(0, 1, n.look.defaultWidth);
    n.graph.addEdge(1, 2, n.look.defaultWidth);
    n.graph.addEdge(3, 4, n.look.defaultWidth);
    const RoadGroundFn ground = [](double, double) { return 0.0; };
    RenderMesh m = buildRoadNetMesh(n, ground);
    CHECK(!hasNonFinite(m));
    CHECK(degenerateTriangles(m) == 0);
    bool grade = false, deck = false;
    for (double h : surfaceHitsAt(m, 0.0, 0.0)) { if (h < 1.0) grade = true; if (h > 8.0) deck = true; }
    CHECK(grade);                                   // the street passes under
    CHECK(deck);                                    // the deck flies over
    CHECK(verticesInBox(m, -4.0, 4.0, 1.0, 8.0, -4.0, 4.0) == 0);   // no curtain wall
}

// S — ramp foot welds while the crest grade-separates. An E-W road climbs 0->9,
// stays elevated, descends; a cross street meets its WEST foot at grade, and a
// street passes UNDER the elevated crest. The split reclassifies the low ramp
// feet to ground (welding the cross street) while the crest stays a viaduct that
// grade-separates from the flown-over street.
TEST_CASE(gradesep_ramp_foot_welds_crest_separates) {
    RoadEntity n;
    n.graph.nodes = {
        RoadNode{Vec2(-150, 0)}, RoadNode{Vec2(-80, 0)},
        RoadNode{Vec2(80, 0)}, RoadNode{Vec2(150, 0)},           // 0..3: E-W 0->9->9->0
        RoadNode{Vec2(-150, -40)}, RoadNode{Vec2(-150, 40)},     // 4,5: cross street at west foot
        RoadNode{Vec2(0, -50)}, RoadNode{Vec2(0, 50)} };         // 6,7: street under the crest
    const double elev[4] = { 0.0, 9.0, 9.0, 0.0 };               // nodes 4..7 stay at-grade
    for (int i = 0; i < 4; ++i) {
        n.graph.nodes[i].elev = elev[i];
        n.graph.nodes[i].elevAbsolute = true;
    }
    n.graph.addEdge(0, 1, n.look.defaultWidth);
    n.graph.addEdge(1, 2, n.look.defaultWidth);
    n.graph.addEdge(2, 3, n.look.defaultWidth);
    n.graph.addEdge(4, 0, n.look.defaultWidth);
    n.graph.addEdge(0, 5, n.look.defaultWidth);
    n.graph.addEdge(6, 7, n.look.defaultWidth);
    const RoadGroundFn ground = [](double, double) { return 0.0; };
    RenderMesh m = buildRoadNetMesh(n, ground);
    CHECK(!hasNonFinite(m));
    CHECK(degenerateTriangles(m) == 0);
    // Crest over the flown-over street (x=0): deck aloft + street at grade, clear between.
    bool grade = false, deck = false;
    for (double h : surfaceHitsAt(m, 0.0, 0.0)) { if (h < 1.0) grade = true; if (h > 8.0) deck = true; }
    CHECK(grade);
    CHECK(deck);
    CHECK(verticesInBox(m, -4.0, 4.0, 1.0, 8.0, -4.0, 4.0) == 0);
    // West foot (x=-150): at grade, one welded surface with the cross street (no
    // second surface aloft, no curtain wall there).
    bool foot = false;
    for (double h : surfaceHitsAt(m, -150.0, 0.0)) if (h < 1.5) foot = true;
    CHECK(foot);
    CHECK(hasClearSpanAt(m, -150.0, 0.0, 2.0, 100.0));   // nothing elevated over the foot
}
