#include "test_framework.h"

#include "../src/engine/procgen/terrain_field.h"
#include "../src/engine/procgen/terrain.h"     // TerrainFlatten, makeFlattenRamp
#include "../src/engine/procgen/erosion.h"
#include <cmath>
#include <vector>

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

TEST_CASE(terrain_conform_levels_a_flat_road_corridor) {
    // A tilted ground; a flat road corridor forces the terrain flat inside it and
    // eases back to the natural slope outside — the cut/fill an urban road needs.
    HeightField base = [](double x, double z) { return 0.1 * x + 0.2 * z; };
    std::vector<TerrainFlatten> regs;
    regs.push_back(makeFlattenRamp(Vec3(-20, 5, 0), Vec3(20, 5, 0),
                                   5.0, 5.0, /*halfWidth=*/4.0, /*falloff=*/6.0));
    HeightField land = conformField(base, regs);
    CHECK_APPROX(land(0, 0), 5.0, 1e-6);     // dead centre: flat road height
    CHECK_APPROX(land(10, 3), 5.0, 1e-6);    // still within the half-width
    CHECK_APPROX(land(0, 100), base(0, 100), 1e-6);   // far away: untouched
}

TEST_CASE(terrain_conform_holds_a_constant_grade) {
    // A road climbing a hill: flat across its width, a single linear incline along
    // its length (not following bumps) — endpoints meet the terrain height.
    HeightField base = [](double, double) { return 0.0; };
    std::vector<TerrainFlatten> regs;
    regs.push_back(makeFlattenRamp(Vec3(0, 0, 0), Vec3(100, 10, 0),
                                   0.0, 10.0, /*halfWidth=*/4.0, /*falloff=*/6.0));
    HeightField land = conformField(base, regs);
    CHECK_APPROX(land(2, 0), 0.2, 1e-6);     // linear along the length
    CHECK_APPROX(land(50, 0), 5.0, 1e-6);
    CHECK_APPROX(land(98, 0), 9.8, 1e-6);
    CHECK_APPROX(land(50, 2), 5.0, 1e-6);    // flat across the width
}

TEST_CASE(terrain_conform_levels_a_block_pad) {
    // A block pad forces a square region flat at its plot height while the
    // surrounds ease back — the level building plot a developed block sits on.
    HeightField base = [](double x, double z) { return 0.05 * x + 0.05 * z; };
    std::vector<TerrainFlatten> regs;
    std::vector<Vec3> poly = { Vec3(-10, 0, -10), Vec3(10, 0, -10),
                               Vec3(10, 0, 10), Vec3(-10, 0, 10) };
    regs.push_back(makeFlattenPad(poly, 5.0, /*falloff=*/6.0));
    HeightField land = conformField(base, regs);
    CHECK_APPROX(land(0, 0), 5.0, 1e-6);     // centre forced to the pad height
    CHECK_APPROX(land(8, -8), 5.0, 1e-6);    // anywhere inside the pad
    CHECK_APPROX(land(0, 100), base(0, 100), 1e-6);   // far away: untouched
}

TEST_CASE(flat_sites_finds_the_plain_not_the_mountain) {
    // A world split in two: a flat plain on the left (x < 0), a steep cone on the
    // right (x > 0). findFlatSites must return a buildable disc in the plain and
    // never centre one on the slope.
    HeightField world = [](double x, double z) {
        if (x < -10) return 0.5;                       // flat plain
        double r = std::sqrt((x - 120) * (x - 120) + z * z);
        return std::max(0.5, 140.0 - r);               // a tall steep cone at (120,0)
    };
    FlatSiteParams p;
    p.region = 200; p.cell = 8; p.maxSlope = 0.25; p.maxHeight = 40;
    p.minRadius = 20; p.count = 3;
    std::vector<FlatSite> sites = findFlatSites(world, p);
    CHECK(!sites.empty());
    const FlatSite& best = sites[0];
    CHECK(best.cx < -10);                  // the site sits in the flat plain
    CHECK(best.radius >= 20);              // a real, usable flat disc
    // No returned site sits on the steep cone.
    for (const FlatSite& s : sites) CHECK(s.cx < 60);
}

TEST_CASE(smooth_centripetal_catmull_rom_rounds_a_staircase) {
    // ADR-0048: the grid router emits an 8-direction staircase. Smoothing must round
    // its corners — the sharpest turn between consecutive segments drops well below
    // the raw 90 degrees — while subdividing finely and pinning the exact endpoints
    // (so a road still meets its junctions).
    std::vector<Vec3> stair = { {0,0,0}, {10,0,0}, {10,0,10}, {20,0,10}, {20,0,20} };
    auto maxTurn = [](const std::vector<Vec3>& p) {
        double m = 0;
        for (std::size_t i = 1; i + 1 < p.size(); ++i) {
            double ax = p[i].x-p[i-1].x, az = p[i].z-p[i-1].z;
            double bx = p[i+1].x-p[i].x, bz = p[i+1].z-p[i].z;
            double la = std::hypot(ax,az), lb = std::hypot(bx,bz);
            if (la < 1e-9 || lb < 1e-9) continue;
            double c = std::max(-1.0, std::min(1.0, (ax*bx + az*bz)/(la*lb)));
            m = std::max(m, std::acos(c));
        }
        return m;
    };
    std::vector<Vec3> sm = smoothCentripetalCatmullRom(stair, /*closed=*/false, 0.2, 0.0);
    CHECK(sm.size() > stair.size());                  // adaptively subdivided
    CHECK_APPROX(sm.front().x, 0.0, 1e-6);  CHECK_APPROX(sm.front().z, 0.0, 1e-6);
    CHECK_APPROX(sm.back().x, 20.0, 1e-6);  CHECK_APPROX(sm.back().z, 20.0, 1e-6);
    CHECK(maxTurn(stair) > 1.5);                       // ~90-degree corners
    CHECK(maxTurn(sm) < maxTurn(stair) * 0.4);         // rounded into gentle turns
}

TEST_CASE(round_road_corners_holds_a_minimum_radius) {
    // ADR-0048: a road ribbon is the centerline offset by its half-width, and an
    // offset self-intersects on the inside of any bend tighter than that offset. So
    // filleting a corner to a guaranteed minRadius (>= half-width) is what stops the
    // widened geometry folding. A 90-degree corner with long legs must come out with
    // no point sharper than minRadius, endpoints intact.
    std::vector<Vec3> corner = { {0,0,0}, {100,0,0}, {100,0,100} };
    double R = 20.0;
    std::vector<Vec3> filleted = roundRoadCorners(corner, R, /*chordTol=*/0.3, /*decimate=*/0.0);
    CHECK(filleted.size() > 3);                          // arc inserted
    CHECK_APPROX(filleted.front().x, 0.0, 1e-6);
    CHECK_APPROX(filleted.back().z, 100.0, 1e-6);

    // Minimum radius of curvature over consecutive triples (circumradius = abc/4A).
    double minR = 1e18;
    for (std::size_t i = 1; i + 1 < filleted.size(); ++i) {
        double ax = filleted[i].x-filleted[i-1].x, az = filleted[i].z-filleted[i-1].z;
        double bx = filleted[i+1].x-filleted[i].x, bz = filleted[i+1].z-filleted[i].z;
        double cx = filleted[i+1].x-filleted[i-1].x, cz = filleted[i+1].z-filleted[i-1].z;
        double area2 = std::fabs(ax*bz - az*bx);        // 2 * triangle area
        if (area2 < 1e-9) continue;                      // collinear -> infinite radius
        double circum = (std::hypot(ax,az) * std::hypot(bx,bz) * std::hypot(cx,cz)) / (2*area2);
        minR = std::min(minR, circum);
    }
    CHECK(minR > R * 0.9);                               // radius held (sampling slack)
}

TEST_CASE(flat_sites_respects_max_height_and_separation) {
    // An everywhere-flat field at two elevations: low (z<0) and a high mesa
    // (z>0, height 80). With maxHeight 50 only the low half is buildable.
    HeightField field = [](double, double z) { return z > 0 ? 80.0 : 1.0; };
    FlatSiteParams p;
    p.region = 200; p.cell = 8; p.maxSlope = 0.2; p.maxHeight = 50;
    p.minRadius = 15; p.count = 4;
    std::vector<FlatSite> sites = findFlatSites(field, p);
    CHECK(!sites.empty());
    for (const FlatSite& s : sites) CHECK(s.cz <= 0.0);   // only the low half
}
