// Can you actually DRIVE the freeway the engine builds?
//
// The citysim's drive tests run cars on an abstract NAV GRAPH; the mesh tests
// check the geometry in isolation. Nothing checked the two AGREE — so a route the
// sim plans can run straight through a wall, and both suites stay green. That is
// how a 0.85 m parapet came to stand across every on-ramp merge in the shipping
// flagship while 932 tests passed.
//
// This drives the SAME centrelines the nav chain is built from (rampPaths and the
// deck alignment) across the SAME mesh the player collides with, and reports what
// a car would hit.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include "../src/engine/procgen/city/road_lattice.h"
#include "../src/engine/procgen/city/alignment.h"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace engine;
using namespace driveprobe;

namespace {

// A corridor with real exits AND on-ramps on both carriageways, elevated on a
// viaduct — i.e. metropolis's shape, small enough to drive in a unit test.
CorridorDef labCorridor() {
    CorridorDef c;
    c.horizontal = Alignment::fromPolyline({ Vec2(-400, 0), Vec2(0, 0), Vec2(400, 60) },
                                           260.0, 40.0);
    const Real L = c.horizontal.length();
    c.designSpeed = 30.0;
    c.laneWidth = 3.6;
    c.lanes.throughLanes = 3;
    c.medianWidth = 1.4;
    // Flown the whole way: +11 m, so ramps must descend to the street grid.
    c.vertical.pvis = { {0.0, 11.0, 0.0}, {L * 0.5, 11.0, 0.0}, {L, 11.0, 0.0} };
    ExitDef ex;                       // an EXIT leaving the deck
    ex.station = L * 0.45; ex.upStation = true;
    ex.target = Vec2(60, -150); ex.targetY = 0.0;
    ex.rampRadius = 65; ex.rampSpiral = 28; ex.decelLength = 190;
    c.exits.push_back(ex);
    ExitDef on;                       // an ON-RAMP merging onto it
    on.station = L * 0.62; on.upStation = true; on.onRamp = true;
    on.target = Vec2(150, -150); on.targetY = 0.0;
    on.rampRadius = 65; on.rampSpiral = 28; on.decelLength = 200;
    c.exits.push_back(on);
    return c;
}

// Build the corridor mesh the loader builds — the SWEPT-LATTICE freeway, deck
// parapets gapped over each gore — and hand back the ramp centrelines so the
// tests drive the exact polylines the nav chain is built from.
RenderMesh sweptCorridor(const CorridorDef& c, std::vector<RampPath>& rampsOut) {
    auto ground = [](Real, Real) -> Real { return 0.0; };
    CorridorAuthoring au = corridorAuthor(c, ground, 3.0);
    rampsOut = au.rampPaths;
    std::vector<UnionSpine> deckSpines = corridorDeckSpines(c, ground, 3.0);
    std::vector<UnionSpine> ramps = corridorRampSpines(au.rampPaths);
    CorridorLatticeParams clp;
    clp.ground = ground;
    const Real Lc = c.horizontal.length();
    for (const ExitDef& e : c.exits) {
        if (e.station < 0) continue;
        const double sg = std::min(std::max<double>(e.station, 0.0), (double)Lc);
        clp.deckGaps.push_back({ std::max(0.0, sg - e.decelLength - 20.0),
                                 std::min((double)Lc, sg + 30.0) });
    }
    return deckSpines.empty() ? RenderMesh{}
                              : sweepCorridor(deckSpines.front(), ramps, clp);
}

}  // namespace

// THE MAINLINE. Driving a carriageway LANE must be unobstructed — parapets
// belong at the verges and the median wall down the centre, neither across a
// lane. (A divided road is driven in a lane, not down its median, which is where
// the geometric centreline runs.)
TEST_CASE(drive_freeway_mainline_is_clear) {
    CorridorDef c = labCorridor();
    std::vector<RampPath> ramps;
    RenderMesh m = sweptCorridor(c, ramps);

    const Real L = c.horizontal.length();
    for (const Real side : { Real(+1), Real(-1) }) {   // both carriageways
        std::vector<Vec3> path;
        for (Real s = 4; s <= L - 4; s += 4.0) {
            const Vec2 p = c.horizontal.pos(s);
            const Vec2 nrm = c.horizontal.normal(s);
            const Real off = side * c.halfWidthAt(s, side) * 0.5;   // a lane centre
            path.push_back(Vec3(p.x + nrm.x * off, c.vertical.elevation(s),
                                p.y + nrm.y * off));
        }
        Report rep;
        drivePath(m, path, rep);
        rep.print(side > 0 ? "mainline+" : "mainline-");
        CHECK(rep.samples > 50);
        CHECK(rep.holes == 0);
        CHECK(rep.blocked == 0);
        // The deck sweep is step-free on its own (freeway_deck_is_drivable /
        // freeway_full_section_has_structure_and_drives). Any residual step here
        // is the RAMP's top seam where it overlaps the mainline deck — the
        // ramp<->deck weld, the next task (#64), not the deck mesher.
    }
}

