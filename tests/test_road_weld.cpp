#include "test_framework.h"
#include "mesh_invariants.h"

#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
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


