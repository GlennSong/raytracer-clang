#include "test_framework.h"

#include "../src/engine/procgen/terrain_horizon.h"

#include <cmath>
#include <cstdio>

using namespace engine;

// TERRAIN HORIZON (device: "when the sun set behind the mountains I thought I
// might start seeing more darkness right away, or at least some shadow
// casting from the mountain"). A synthetic world: a flat plain with a 300 m
// wall from x = 500 eastward. From the origin the horizon toward the wall
// stands at atan(298 / 500) = 30.8 deg; toward the west, nothing.

namespace {
HeightRaster wallWorld() {
    return rasterizeHeights(
        [](double x, double /*z*/) { return x >= 500.0 ? 300.0 : 0.0; },
        -1000.0, -1000.0, 2000.0, 200);   // 10 m texels
}
}

TEST_CASE(terrain_horizon_raster_samples_the_field) {
    const HeightRaster r = wallWorld();
    CHECK(r.size == 200);
    CHECK_APPROX(r.texel(), 10.0, 1e-9);
    CHECK_APPROX(r.at(0.0, 0.0), 0.0, 1e-6);
    CHECK_APPROX(r.at(800.0, 100.0), 300.0, 1e-6);
    CHECK(r.at(500.0, 0.0) > 100.0);   // the wall's face, bilinear
    CHECK(r.at(500.0, 0.0) < 300.0);
}

TEST_CASE(terrain_horizon_stands_toward_the_wall_and_not_away) {
    const HeightRaster r = wallWorld();
    const HorizonMap east = computeHorizonMap(r, 90.0, 3000.0, 2.0);
    const HorizonMap west = computeHorizonMap(r, 270.0, 3000.0, 2.0);
    const HorizonMap north = computeHorizonMap(r, 0.0, 3000.0, 2.0);
    const double expect = 298.0 / std::sqrt(298.0 * 298.0 + 500.0 * 500.0);   // sin 30.8 deg
    std::printf("    [horizon] toward the wall sin %.3f (expect %.3f), away %.3f, along %.3f\n",
                east.sinElevationAt(0.0, 0.0), expect, west.sinElevationAt(0.0, 0.0),
                north.sinElevationAt(0.0, 0.0));
    CHECK_APPROX(east.sinElevationAt(0.0, 0.0), expect, 0.03);
    CHECK(std::fabs(west.sinElevationAt(0.0, 0.0)) < 0.01);
    CHECK(std::fabs(north.sinElevationAt(0.0, 0.0)) < 0.01);
    // Nearer the wall the ridge stands higher; on top of it, nothing.
    CHECK(east.sinElevationAt(300.0, 0.0) > east.sinElevationAt(0.0, 0.0));
    CHECK(std::fabs(east.sinElevationAt(800.0, 0.0)) < 0.01);
    // THE PREDICATE the shader applies: the light below the horizon is
    // behind the ridge. A sun 20 deg up in the east is shadowed at the
    // origin; 40 deg up it is not; 20 deg up in the west never is.
    const double sun20 = std::sin(20.0 * 3.14159265358979323846 / 180.0);
    const double sun40 = std::sin(40.0 * 3.14159265358979323846 / 180.0);
    CHECK(sun20 < east.sinElevationAt(0.0, 0.0));
    CHECK(sun40 > east.sinElevationAt(0.0, 0.0));
    CHECK(sun20 > west.sinElevationAt(0.0, 0.0));
    CHECK(east.azimuthDeg == 90.0);
}

TEST_CASE(terrain_horizon_encoding_round_trips) {
    for (double s = HorizonMap::kEncodeLo; s <= HorizonMap::kEncodeHi; s += 0.05)
        CHECK_APPROX(HorizonMap::decode(HorizonMap::encode(s)), s, 1.0 / 255.0 + 1e-9);
    CHECK(HorizonMap::encode(-5.0) == 0);
    CHECK(HorizonMap::encode(5.0) == 255);
    CHECK_APPROX(HorizonMap::decode(HorizonMap::encode(0.0)), 0.0, 1.0 / 255.0);
}
