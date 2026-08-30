#include "test_framework.h"

#include "../src/engine/procgen/earthwork.h"
#include "../src/engine/procgen/terrain.h"

#include <cmath>

using namespace engine;

// THE EARTHWORK FIELD (terrain-earthwork plan, Phase 2): one smooth
// displacement fitted to the road carve, added to the natural ground. These
// are its contract, on a hillside with a single road cut across it.

namespace {
// A 20% hillside rising in +z.
double hillside(double, double z) { return 0.20 * z; }

// One road across the hill at z = 0 running along x, carved flat at y = 0.
// makeFlattenRamp with yA = yB = 0 over a 10 m half-width; priority = road.
std::vector<TerrainFlatten> oneRoad() {
    TerrainFlatten r = makeFlattenRamp(Vec3(-200, 0, 0), Vec3(200, 0, 0), 0.0, 0.0,
                                       10.0, 8.0);
    r.priority = kRoadFlattenPriority;
    return { r };
}
}  // namespace

// Where the road covers the ground, the field reproduces the carve exactly:
// natural + D == the carve plane (the SAME applyFlatten the terrain uses).
TEST_CASE(earthwork_matches_the_carve_where_the_road_is) {
    EarthworkParams p;
    p.cell = 4.0; p.reach = 60.0;
    EarthworkStats st;
    auto D = buildEarthworkField(oneRoad(), hillside, p, -1e9, &st);
    CHECK(D != nullptr);
    CHECK(st.cells > 1000);
    // Strictly inside the footprint: the road's own edge nodes (|z| = 10) are
    // solved, not pinned (coverage is exact, not dilated — see earthwork.cpp),
    // so the samples adjacent to them carry a few cm of the solver's smoothing.
    for (double x : {-150.0, -50.0, 0.0, 50.0, 150.0})
        for (double z : {-6.0, -2.0, 0.0, 2.0, 6.0}) {
            const double want = 0.0 - hillside(x, z);          // plane - natural
            CHECK(std::fabs((*D)(x, z) - want) < 0.05);        // grid-node exact, bilerp between
        }
    // ...and at the edge itself the drift is bounded by ONE CELL of the cross
    // slope (the last pinned node can sit a full cell inside the edge, and the
    // field is harmonic past it): 20% x 4 m = 0.8 m. Measured 0.7 m. That is
    // the resolution limit of a 4 m grid, not a bank — and inside the
    // footprint the carve plane wins regardless of the field.
    CHECK(std::fabs((*D)(0, 9.0) - (0.0 - hillside(0, 9.0))) < 0.20 * p.cell + 0.1);
}

// The field is a SLOPE between the road and the untouched hill, not a bank:
// 30 m uphill of the road the ground has come most of the way back to natural,
// smoothly, and the steepest gradient of D is bounded by the pinned jump
// divided by the cell (no step can exceed one cell's worth).
TEST_CASE(earthwork_decays_smoothly_away_from_the_road) {
    EarthworkParams p;
    p.cell = 4.0; p.reach = 60.0;
    auto D = buildEarthworkField(oneRoad(), hillside, p, -1e9);
    CHECK(D != nullptr);
    // Monotone decay along +z from the road edge outward.
    double prev = std::fabs((*D)(0, 12));
    double worstStep = 0;
    for (double z = 16; z <= 300; z += 4) {
        const double v = std::fabs((*D)(0, z));
        CHECK(v <= prev + 1e-6);
        worstStep = std::max(worstStep, std::fabs(v - prev));
        prev = v;
    }
    // At the road edge the jump is |0.2 * 10| = 2 m; per 4 m cell the field
    // cannot change by more than that.
    CHECK(worstStep <= 2.0 + 1e-6);
    // Far away (3 * reach) it is under 5% of the road-edge value.
    CHECK(std::fabs((*D)(0, 12 + 3 * 60)) < 0.05 * 2.0 + 1e-6);
    // And zero outside its own domain.
    CHECK((*D)(0, 5000) == 0.0);
}

// Sea-floor cells are pinned: land under water never moves, so a coastal road
// cannot lift or sink the sea bed beside it.
TEST_CASE(earthwork_never_moves_the_sea_floor) {
    EarthworkParams p;
    p.cell = 4.0; p.reach = 60.0;
    // Sea level at z = -20 on the 20% hill -> y = -4; everything below is wet.
    auto D = buildEarthworkField(oneRoad(), hillside, p, -4.0);
    CHECK(D != nullptr);
    for (double z = -24; z >= -200; z -= 8)
        CHECK(std::fabs((*D)(0, z)) < 1e-9);
}

// Deterministic: two builds are bit-identical.
TEST_CASE(earthwork_is_deterministic) {
    EarthworkParams p;
    p.cell = 4.0; p.reach = 60.0;
    auto A = buildEarthworkField(oneRoad(), hillside, p, -1e9);
    auto B = buildEarthworkField(oneRoad(), hillside, p, -1e9);
    for (double x = -180; x <= 180; x += 37)
        for (double z = -150; z <= 150; z += 23)
            CHECK((*A)(x, z) == (*B)(x, z));
}

// Off is off: disabled params, or no road regions, give no field.
TEST_CASE(earthwork_is_null_when_disabled_or_empty) {
    EarthworkParams p;
    p.enabled = false;
    CHECK(buildEarthworkField(oneRoad(), hillside, p, -1e9) == nullptr);
    p.enabled = true;
    CHECK(buildEarthworkField({}, hillside, p, -1e9) == nullptr);
}
