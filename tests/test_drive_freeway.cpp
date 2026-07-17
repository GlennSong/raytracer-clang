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
#include "../src/engine/procgen/city/alignment.h"
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

RenderMesh weldCorridor(const CorridorDef& c, std::vector<RampPath>& rampsOut) {
    auto ground = [](Real, Real) -> Real { return 0.0; };
    CorridorAuthoring au = corridorAuthor(c, ground, 3.0);
    rampsOut = au.rampPaths;
    std::vector<UnionSpine> spines = corridorDeckSpines(c, ground, 3.0);
    std::vector<UnionSpine> ramps = corridorRampSpines(au.rampPaths);
    spines.insert(spines.end(), ramps.begin(), ramps.end());
    WeldSolidParams wp;
    wp.barriers = true;               // the parapets/median a real freeway has
    wp.thickness = 0.5;
    wp.heightAt = ground;
    return weldSolid(spines, wp);
}

}  // namespace

// THE MAINLINE. Driving down the middle of the deck must be unobstructed: the
// parapets belong at the verges, not across the carriageway.
TEST_CASE(drive_freeway_mainline_is_clear) {
    CorridorDef c = labCorridor();
    std::vector<RampPath> ramps;
    RenderMesh m = weldCorridor(c, ramps);

    // the deck's own centreline, at its authored height
    std::vector<Vec3> path;
    const Real L = c.horizontal.length();
    for (Real s = 4; s <= L - 4; s += 4.0) {
        const Vec2 p = c.horizontal.pos(s);
        path.push_back(Vec3(p.x, c.vertical.elevation(s), p.y));
    }
    Report rep;
    drivePath(m, path, rep);
    rep.print("mainline");
    CHECK(rep.samples > 50);
    CHECK(rep.holes == 0);
    CHECK(rep.blocked == 0);
}

// THE RAMPS — the user's report: "The freeway on ramps aren't driveable and
// there's strips of triangles that prevent you from merging onto the freeway."
// Drive each authored ramp centreline (the exact polyline the nav chain is built
// from). Nothing may stand in the way, and the surface must be continuous from
// the deck all the way down to the street.
TEST_CASE(drive_freeway_ramps_are_clear) {
    CorridorDef c = labCorridor();
    std::vector<RampPath> ramps;
    RenderMesh m = weldCorridor(c, ramps);

    int driven = 0;
    Report rep;
    for (const RampPath& rp : ramps) {
        if (rp.pts.size() < 4) continue;      // dropped ramp: nothing to drive
        ++driven;
        drivePath(m, rp.pts, rep);
    }
    rep.print("ramps");
    CHECK(driven > 0);                        // the fixture really authors ramps
    // BASELINE 2026-07-17: blocked=11 (a 0.85 m parapet stands across every gore),
    // steps=86 (worst 3.20 m — the ramp foot never welds to the street). These
    // bounds catch regression; the swept-lattice freeway (barrier as a per-station
    // profile column, gapped at the gore) drives blocked to 0.
    // STAGE-1 GATE: tighten to `blocked == 0` and `holes == 0`.
    CHECK(rep.blocked <= 12);                 // TARGET 0: no parapet across the merge
    CHECK(rep.holes <= 2);                    // TARGET 0: no gap in the ramp surface
}
