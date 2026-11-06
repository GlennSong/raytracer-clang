#include "test_framework.h"

#include "../src/engine/procgen/regrade.h"
#include "../src/engine/procgen/texture_field.h"   // chamferSweepGrid
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {

// a hillside falling 10 % to +x, broken by crags and gullies 12 m high every ~45 m
double roughHill(double x, double z) {
    return 300.0 - 0.10 * x + 12.0 * std::sin(x * 0.14) * std::cos(z * 0.11) + 5.0 * std::sin(x * 0.31 + z * 0.27);
}

Regrade squareTown(double half) {
    Regrade g;
    g.outline = {Vec2(-half, -half), Vec2(half, -half), Vec2(half, half), Vec2(-half, half)};
    return g;
}

// the steepest the ground runs at (x, z), over `d` metres
double slopeAt(const HeightField& h, double x, double z, double d) {
    const double gx = (h(x + d, z) - h(x - d, z)) / (2 * d), gz = (h(x, z + d) - h(x, z - d)) / (2 * d);
    return std::sqrt(gx * gx + gz * gz);
}

}  // namespace

TEST_CASE(chamfer_sweep_is_the_distance_from_its_seeds_and_a_slope_cap_on_heights) {
    // seeded 0 at one cell, huge elsewhere: the chamfer distance
    const int n = 9;
    std::vector<float> d(n * n, 1e9f);
    d[4 * n + 4] = 0.0f;
    chamferSweepGrid(d, n, 1.0f, 1.5f);
    CHECK_APPROX(d[4 * n + 8], 4.0, 1e-6);    // four straight steps
    CHECK_APPROX(d[8 * n + 8], 6.0, 1e-6);    // four diagonal ones
    CHECK_APPROX(d[8 * n + 6], 5.0, 1e-6);    // two diagonal, two straight
    // seeded with heights: a spike is cut down to the cone the rise allows, and nothing is raised
    std::vector<float> h(n * n, 10.0f);
    h[4 * n + 4] = 50.0f;
    chamferSweepGrid(h, n, 2.0f, 2.8f);
    CHECK_APPROX(h[4 * n + 4], 10.0 + 2.0, 1e-6);   // the spike: its flat neighbours plus one step
    for (std::size_t k = 0; k < h.size(); ++k)
        if (k != 4u * n + 4u) CHECK_APPROX(h[k], 10.0, 1e-6);   // nothing else moves
    // a pit pulls the ground round it down to its cone
    std::vector<float> p(n * n, 10.0f);
    p[0] = 0.0f;
    chamferSweepGrid(p, n, 2.0f, 2.8f);
    CHECK_APPROX(p[0], 0.0, 1e-6);
    CHECK_APPROX(p[1], 2.0, 1e-6);
    CHECK_APPROX(p[n + 1], 2.8, 1e-6);
    CHECK_APPROX(p[8], 10.0, 1e-6);
}

TEST_CASE(terrace_height_holds_a_shelf_then_banks_up_to_the_next) {
    // bank 0 is heightTerrace's hard step
    for (double x = -20; x <= 20; x += 1.3) CHECK_APPROX(terraceHeight(x, 5.0), std::round(x / 5.0) * 5.0, 1e-12);
    // a bank of 0.4: level for the first 60 % of a step, then a smooth climb, meeting the next shelf
    CHECK_APPROX(terraceHeight(-2.4, 8.0, 0.4), 0.0, 1e-12);   // q = 0.2 into the step
    CHECK_APPROX(terraceHeight(0.7, 8.0, 0.4), 0.0, 1e-12);    // q = 0.5875: still on the shelf
    CHECK_APPROX(terraceHeight(3.99, 8.0, 0.4), 8.0, 1e-3);    // the top of the bank is the next shelf
    double last = -1e9;
    for (double x = -30; x <= 30; x += 0.25) {
        const double v = terraceHeight(x, 8.0, 0.4);
        CHECK(v >= last - 1e-12);   // never falls
        last = v;
    }
}

TEST_CASE(regrade_levels_a_rough_hillside_but_keeps_its_fall) {
    Regrade g = squareTown(300.0);
    g.blur = 100.0;
    g.maxSlope = 0.12;
    const HeightField out = regradeField(roughHill, {g});
    // inside: no steeper than the cap (a little over, for the bilinear grid), and the crags gone
    double steepest = 0.0, steepestRaw = 0.0, worstOff = 0.0;
    for (double z = -280; z <= 280; z += 20)
        for (double x = -280; x <= 280; x += 20) {
            steepest = std::max(steepest, slopeAt(out, x, z, 6.0));
            steepestRaw = std::max(steepestRaw, slopeAt(roughHill, x, z, 6.0));
            worstOff = std::max(worstOff, std::abs(out(x, z) - (300.0 - 0.10 * x)));   // the hill's own plane
        }
    std::printf("    [regrade] inside: steepest %.1f %% (raw %.1f %%), furthest from the hill's plane %.1f m\n",
                100 * steepest, 100 * steepestRaw, worstOff);
    CHECK(steepest < 0.13);
    CHECK(steepestRaw > 0.5);
    CHECK(worstOff < 4.0);
    // the hill still falls the way it did: 10 % over the town, end to end
    CHECK_APPROX((out(-250, 0) - out(250, 0)) / 500.0, 0.10, 0.015);
    // cut about fills: the mean change over the town is small next to the crags
    double sum = 0.0;
    int cnt = 0;
    for (double z = -280; z <= 280; z += 10)
        for (double x = -280; x <= 280; x += 10) { sum += out(x, z) - roughHill(x, z); ++cnt; }
    std::printf("    [regrade] mean change over the town %.2f m\n", sum / cnt);
    CHECK(std::abs(sum / cnt) < 1.5);
    // past the feather the raw ground is untouched, and within it eases back
    for (double z = -500; z <= 500; z += 37) {
        CHECK_APPROX(out(410.0, z), roughHill(410.0, z), 1e-9);
        CHECK_APPROX(out(z, -420.0), roughHill(z, -420.0), 1e-9);
    }
    double edge = 0.0, mid = 0.0;
    for (double z = -250; z <= 250; z += 5) {
        edge += std::abs(out(302.0, z) - roughHill(302.0, z));
        mid += std::abs(out(350.0, z) - roughHill(350.0, z));
    }
    CHECK(mid < 0.75 * edge);
}

TEST_CASE(regrade_terraces_cut_the_slope_into_shelves) {
    Regrade g = squareTown(300.0);
    g.terrace = 8.0;
    g.bank = 0.3;
    const HeightField out = regradeField(roughHill, {g});
    // most of the town stands on level shelves at whole steps; the banks between climb steeper than the cap
    int level = 0, onStep = 0, total = 0;
    double steepest = 0.0;
    for (double z = -280; z <= 280; z += 7)
        for (double x = -280; x <= 280; x += 7) {
            ++total;
            const double s = slopeAt(out, x, z, 2.0);
            steepest = std::max(steepest, s);
            if (s < 0.02) ++level;
            const double h = out(x, z);
            if (std::abs(h - std::round(h / 8.0) * 8.0) < 0.05) ++onStep;
        }
    std::printf("    [regrade] terraces: %d of %d samples level, %d on a whole step, steepest bank %.0f %%\n", level, total,
                onStep, 100 * steepest);
    CHECK(level > total / 2);
    CHECK(onStep > total / 2);
    CHECK(steepest > 0.2);
    CHECK_APPROX((out(-250, 0) - out(250, 0)) / 500.0, 0.10, 0.03);   // the fall is kept, shelf to shelf
}

TEST_CASE(regrade_without_one_is_the_ground_itself) {
    const HeightField out = regradeField(roughHill, {});
    for (double x = -500; x <= 500; x += 61) CHECK_APPROX(out(x, 0.7 * x), roughHill(x, 0.7 * x), 0.0);
    nlohmann::json t = {{"regrade", nlohmann::json::array({{{"outline", {{0, 0}, {10, 0}, {10, 10}}}, {"terrace", 6}}, {{"outline", {{0, 0}}}}})}};
    const std::vector<Regrade> rg = regradesFromJson(t);
    CHECK(rg.size() == 1u);   // the one with too few corners is dropped
    if (!rg.empty()) { CHECK_APPROX(rg[0].terrace, 6.0, 0.0); CHECK_APPROX(rg[0].maxSlope, 0.12, 0.0); }
    CHECK(regradesFromJson(nlohmann::json::object()).empty());
}

TEST_CASE(regrade_caps_a_hillside_steeper_than_its_town_can_stand) {
    // a 20 % hillside with a knoll on it: the cap lays it back to 12 %, cut above the middle and fill below it
    auto steep = [](double x, double z) { return 400.0 - 0.20 * x + 25.0 * std::exp(-((x - 80) * (x - 80) + z * z) / (60.0 * 60.0)); };
    Regrade g = squareTown(250.0);
    const HeightField out = regradeField(steep, {g});
    double steepest = 0.0;
    for (double z = -230; z <= 230; z += 10)
        for (double x = -230; x <= 230; x += 10) steepest = std::max(steepest, slopeAt(out, x, z, 4.0));
    const double fall = (out(-200, 0) - out(200, 0)) / 400.0;
    std::printf("    [regrade] 20 %% hill: steepest %.1f %%, fall over the town %.1f %%, middle moved %.1f m\n", 100 * steepest,
                100 * fall, out(0, 150) - steep(0, 150));
    CHECK(steepest < 0.135);          // the chamfer's eight directions allow a little over the cap between them
    CHECK(fall > 0.10 && fall < 0.125);
    CHECK(out(-200, 0) < steep(-200, 0));   // cut uphill
    CHECK(out(200, 0) > steep(200, 0));     // filled downhill
    CHECK(std::abs(out(0, 150) - steep(0, 150)) < 6.0);   // the middle about where it was
}
