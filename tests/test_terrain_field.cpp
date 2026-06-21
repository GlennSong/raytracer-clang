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

TEST_CASE(route_road_on_flat_ground_runs_straight) {
    // Nothing to climb: the route is one straight shot, grade 0 the whole way.
    HeightField flat = [](double, double) { return 3.0; };
    RouteParams rp; rp.cell = 10; rp.maxGrade = 0.12;
    std::vector<Vec3> path = routeRoad(flat, Vec3(-90, 0, 0), Vec3(90, 0, 0), rp);
    CHECK(path.size() >= 2);
    CHECK_APPROX(path.front().x, -90.0, 1e-6);
    CHECK_APPROX(path.back().x, 90.0, 1e-6);
    // Total length is close to the straight-line distance (no needless detour).
    double len = 0;
    for (std::size_t i = 1; i < path.size(); ++i)
        len += std::hypot(path[i].x - path[i - 1].x, path[i].z - path[i - 1].z);
    CHECK(len < 180.0 * 1.2);
}

TEST_CASE(route_road_keeps_grade_and_bends_around_a_peak) {
    // A tall, steep cone (slope 3 — far over the 0.12 grade) blocks the straight
    // line. The router must route around its base, where the ground is flat, and
    // never take a leg steeper than the grade limit.
    HeightField world = [](double x, double z) {
        double r = std::sqrt(x * x + z * z);
        return std::max(0.0, 120.0 - 3.0 * r);          // cone, height 0 past r=40
    };
    RouteParams rp; rp.cell = 10; rp.maxGrade = 0.12; rp.turnPenalty = 4;
    std::vector<Vec3> path = routeRoad(world, Vec3(-90, 0, 0), Vec3(90, 0, 0), rp);
    CHECK(path.size() >= 2);

    // Every leg holds the grade limit (small epsilon for the simplifier).
    double maxGrade = 0.0, maxAbsZ = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        double horiz = std::hypot(path[i].x - path[i - 1].x, path[i].z - path[i - 1].z);
        double grade = horiz > 1e-6 ? std::fabs(path[i].y - path[i - 1].y) / horiz : 0.0;
        maxGrade = std::max(maxGrade, grade);
        maxAbsZ = std::max(maxAbsZ, std::fabs(static_cast<double>(path[i].z)));
    }
    CHECK(maxGrade <= 0.12 + 1e-3);        // walkable everywhere
    CHECK(maxAbsZ > 35.0);                 // genuinely bent around the cone's base
    // Endpoints are seated on the ground at the requested xz.
    CHECK_APPROX(path.front().x, -90.0, 1e-6);
    CHECK_APPROX(path.back().x, 90.0, 1e-6);
}

TEST_CASE(route_road_returns_empty_when_walled_in) {
    // The goal sits inside a steep crater wall the road can't climb — no grade-legal
    // route exists, so the router reports failure (the caller falls back to straight).
    HeightField crater = [](double x, double z) {
        double r = std::sqrt(x * x + z * z);
        // A ring wall at r~50: steep everywhere between the outside and the centre.
        return (r < 60.0 && r > 40.0) ? 300.0 : 0.0;
    };
    RouteParams rp; rp.cell = 8; rp.maxGrade = 0.12;
    std::vector<Vec3> path = routeRoad(crater, Vec3(-90, 0, 0), Vec3(0, 0, 0), rp);
    CHECK(path.empty());
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
