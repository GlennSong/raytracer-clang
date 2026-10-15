#include "test_framework.h"
#include "mesh_invariants.h"

#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/road_offset.h"
#include "../src/engine/procgen/city/road_rules.h"   // DesignRules (per-class grade)

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

}  // namespace

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

// --- THE JOINT VERTICAL SOLVE (terrain-earthwork plan, Phase 1) ---------------
// weldChainProfiles used to solve each chain alone, then patch the nodes: take
// the lowest arriving deck, smear the difference along the chain, then ease
// each chain back to grade LOWER-ONLY with nothing re-agreeing the arms. On
// metro_v2 that left three arms at one node 8.02 m apart (RT_JUNCTION_DUMP at
// -877,-423). Now a node is ONE variable and every consecutive pair is a grade
// constraint, so these are the properties, checked on a fixture that would have
// broken the old scheme the same way: a steep short stem hitting a flat street.

namespace {
// The maximum |dy| / ds over consecutive samples, minus the allowed grade.
double worstGradeExcess(const std::vector<Vec2>& pts, const std::vector<double>& y,
                        double g) {
    double worst = -1e30;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const double ds = (pts[i + 1] - pts[i]).length();
        worst = std::max(worst, std::fabs(y[i + 1] - y[i]) / ds - g);
    }
    return worst;
}
UnionSpine spineAlong(const Vec2& a, const Vec2& b, int samples, RoadClass k,
                      double hw = 4.0) {
    UnionSpine s;
    for (int i = 0; i < samples; ++i)
        s.points.push_back(a + (b - a) * (static_cast<double>(i) / (samples - 1)));
    s.halfWidth = hw;
    s.klass = k;
    return s;
}
}  // namespace

// A T on a hillside: two arterial arms along a flat street, and a collector
// stem climbing 30% away from the node — with only TWO samples, the shape the
// old roadProfile returned raw (n < 3) and the old ease then dragged down 9.6 m.
TEST_CASE(weld_joint_solve_agrees_at_a_tee_on_a_slope) {
    const double g = 0.08;
    UnionSpine A = spineAlong(Vec2(-100, 0), Vec2(0, 0), 11, RoadClass::Arterial, 8.5);
    UnionSpine B = spineAlong(Vec2(0, 0), Vec2(100, 0), 11, RoadClass::Arterial, 8.5);
    UnionSpine C = spineAlong(Vec2(0, 0), Vec2(0, 80), 2, RoadClass::Collector, 6.5);
    auto ground = [](double, double z) { return 0.30 * z; };   // 24 m up the stem
    auto profs = weldChainProfiles({ A, B, C }, ground, 0.0, g, /*overlapReach*/ 7.5);
    CHECK(profs.size() == 3);
    // One node, one height: the three arms agree to machine precision.
    CHECK(std::fabs(profs[0].back() - profs[1].front()) < 1e-9);
    CHECK(std::fabs(profs[0].back() - profs[2].front()) < 1e-9);
    // Every pair within grade — including the two-point stem.
    CHECK(worstGradeExcess(A.points, profs[0], g) < 1e-6);
    CHECK(worstGradeExcess(B.points, profs[1], g) < 1e-6);
    CHECK(worstGradeExcess(C.points, profs[2], g) < 1e-6);
    // The stem cannot climb 24 m in 80 m at 8%: the solve must have moved
    // BOTH ends toward each other (a cut at the top, a fill at the node), not
    // dragged one end the whole way — the old scheme's failure mode.
    CHECK(profs[2].back() < 24.0 - 1.0);
    CHECK(profs[2].front() > 0.0 + 0.5);
    // Order-independent: the same network in a different spine order yields
    // the same heights per spine.
    auto profs2 = weldChainProfiles({ C, B, A }, ground, 0.0, g, 7.5);
    for (std::size_t i = 0; i < profs[0].size(); ++i)
        CHECK(std::fabs(profs[0][i] - profs2[2][i]) < 1e-9);
    for (std::size_t i = 0; i < profs[2].size(); ++i)
        CHECK(std::fabs(profs[2][i] - profs2[0][i]) < 1e-9);
}

// A square ring of four chains with one corner on a knoll: the projections
// cycle around the loop and must converge with all four corners agreed.
TEST_CASE(weld_joint_solve_converges_around_a_loop) {
    const double g = 0.08;
    std::vector<UnionSpine> ring = {
        spineAlong(Vec2(0, 0), Vec2(100, 0), 6, RoadClass::Local),
        spineAlong(Vec2(100, 0), Vec2(100, 100), 6, RoadClass::Local),
        spineAlong(Vec2(100, 100), Vec2(0, 100), 6, RoadClass::Local),
        spineAlong(Vec2(0, 100), Vec2(0, 0), 6, RoadClass::Local),
    };
    auto ground = [](double x, double z) {   // a 20 m knoll on the (100,100) corner
        const double d = std::hypot(x - 100, z - 100);
        return d < 40 ? 20.0 * (1.0 - d / 40.0) : 0.0;
    };
    auto profs = weldChainProfiles(ring, ground, 0.0, g, 0.0);
    for (int i = 0; i < 4; ++i) {
        CHECK(std::fabs(profs[i].back() - profs[(i + 1) % 4].front()) < 1e-9);
        CHECK(worstGradeExcess(ring[i].points, profs[i], g) < 1e-6);
    }
}

// Per-class limits: the same 12% hillside street is held to 8% as an arterial
// and allowed 12% as a local — the table DesignRules always had and no solve
// ever read (kRoadMaxGrade everywhere).
TEST_CASE(weld_joint_solve_grades_by_class) {
    const DesignRules rules;
    auto ground = [](double, double z) { return 0.12 * z; };
    UnionSpine art = spineAlong(Vec2(0, 0), Vec2(0, 100), 11, RoadClass::Arterial);
    UnionSpine loc = spineAlong(Vec2(50, 0), Vec2(50, 100), 11, RoadClass::Local);
    auto profs = weldChainProfiles({ art, loc }, ground, 0.0, 0.08, 0.0, &rules);
    CHECK(worstGradeExcess(art.points, profs[0], rules.maxGrade(RoadClass::Arterial)) < 1e-6);
    CHECK(worstGradeExcess(loc.points, profs[1], rules.maxGrade(RoadClass::Local)) < 1e-6);
    // The local street keeps (nearly) its 12% hill; the arterial had to cut it.
    CHECK(profs[1].back() - profs[1].front() > 10.0);
    CHECK(profs[0].back() - profs[0].front() < 8.5);
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


// --- Grade separation (P4): a deck flies OVER a live road ---------------------
// The 2-D weld used to fuse a deck and the street it crosses into one surface +
// a curtain wall to the ground. weldSolid now welds each grade group separately,
// so the deck and the street beneath it are two independent surfaces with clear
// air between. Height-band assertions are robust to the thin marking overlay.


namespace {
}  // namespace
