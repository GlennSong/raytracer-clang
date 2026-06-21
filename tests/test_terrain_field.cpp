#include "test_framework.h"

#include "../src/engine/procgen/terrain_field.h"
#include "../src/engine/procgen/erosion.h"
#include <cmath>

using namespace engine;

TEST_CASE(terrain_field_composes_and_bakes) {
    // ADR-0043: terrain is built from primitives (fbm + ridged ridges + warp),
    // not only a preset. Compose, then tessellate to a mesh.
    HeightField base   = heightFbm(7, 0.01, 20.0, 5);
    HeightField ridges = heightRidged(7, 0.006, 30.0, 5);
    HeightField warped = heightWarp(heightMax(base, ridges), heightFbm(3, 0.02, 1.0, 3), 25.0);

    // Deterministic.
    CHECK_APPROX(warped(12.0, -7.0),
                 heightWarp(heightMax(heightFbm(7, 0.01, 20.0, 5),
                                      heightRidged(7, 0.006, 30.0, 5)),
                            heightFbm(3, 0.02, 1.0, 3), 25.0)(12.0, -7.0),
                 1e-9);

    // A varied field: min and max heights differ across the region.
    double lo = 1e9, hi = -1e9;
    for (int j = 0; j < 40; ++j)
        for (int i = 0; i < 40; ++i) {
            double y = warped(i * 8.0 - 160.0, j * 8.0 - 160.0);
            lo = std::min(lo, y); hi = std::max(hi, y);
        }
    CHECK(hi - lo > 10.0);                 // real relief, not a plane

    // Bake to a mesh: a 32-cell grid -> 33x33 vertices, 32*32*2 triangles.
    RenderMesh m = bakeHeightMesh(warped, 200.0, 32, Vec3(0.3, 0.4, 0.25));
    CHECK(m.vertices.size() == 33u * 33u);
    CHECK(m.indices.size() == 32u * 32u * 6u);
    // Vertices carry the field's height and a roughly-upward normal.
    bool upward = true;
    for (const Vertex& v : m.vertices) if (v.normal.y < 0.0) upward = false;
    CHECK(upward);
}

TEST_CASE(terrain_field_erode_returns_a_modified_field) {
    // ADR-0043: erosion is a bake pass over a grid (not a pointwise op); it takes
    // a field, simulates, and hands back a field that samples the eroded grid.
    HeightField mountains = heightRidged(7, 0.01, 40.0, 5);
    ErosionParams ep;
    ep.droplets = 4000;          // light — this is a unit test
    ep.thermalIterations = 4;
    ep.seed = 3;
    HeightField carved = erodeField(mountains, 300.0, 96, ep);

    // The eroded field differs from the source (material moved) but stays in a
    // sane range, and still varies across the region.
    double maxDiff = 0, lo = 1e9, hi = -1e9;
    for (int j = 0; j < 24; ++j)
        for (int i = 0; i < 24; ++i) {
            double x = i * 12.0 - 140.0, z = j * 12.0 - 140.0;
            double a = mountains(x, z), b = carved(x, z);
            maxDiff = std::max(maxDiff, std::fabs(a - b));
            lo = std::min(lo, b); hi = std::max(hi, b);
        }
    CHECK(maxDiff > 0.5);        // erosion actually changed the surface
    CHECK(hi - lo > 5.0);        // still real relief, not flattened away
}

TEST_CASE(terrain_field_terrace_quantises_to_steps) {
    HeightField ramp = [](double x, double) { return x; };   // height == x
    HeightField stepped = heightTerrace(ramp, 5.0);
    // Every sampled height is a multiple of the step.
    for (double x = -20; x <= 20; x += 1.3) {
        double h = stepped(x, 0);
        CHECK_APPROX(h - std::round(h / 5.0) * 5.0, 0.0, 1e-9);
    }
    // combinators: add / scale / clamp.
    CHECK_APPROX(heightAdd(heightConstant(2), heightConstant(3))(0, 0), 5.0, 1e-9);
    CHECK_APPROX(heightScale(heightConstant(4), 0.5)(0, 0), 2.0, 1e-9);
    CHECK_APPROX(heightClamp(heightConstant(9), 0, 5)(0, 0), 5.0, 1e-9);
}
