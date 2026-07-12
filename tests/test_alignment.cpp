#include "test_framework.h"

#include "../src/engine/procgen/city/alignment.h"

#include <cmath>

using namespace engine;

// The corridor model (plan §8): highway-grade geometry the freeway build
// stands on. These pin the geometric CONTRACTS — clothoid corners bend
// gradually to the design radius and never sharper, profiles are continuous
// through crest/sag curves, lane schedules count aux spans — headless, so
// the freeway lab only has to judge the LOOK.

TEST_CASE(alignment_fillets_a_right_angle_with_bounded_curvature) {
    // An L: two 200 m legs meeting at 90 degrees, R=60 fillet with 25 m spirals.
    std::vector<Vec2> control{{0, 0}, {200, 0}, {200, 200}};
    Alignment a = Alignment::fromPolyline(control, 60.0, 25.0, 2.0);
    CHECK(!a.empty());
    // Shorter than the corner route, longer than the straight line.
    CHECK(a.length() < 400.0);
    CHECK(a.length() > std::sqrt(200.0 * 200.0 * 2) + 1.0);
    // Ends where the control ends, heading north.
    CHECK((a.pos(a.length()) - Vec2(200, 200)).length() < 0.5);
    CHECK(std::fabs(a.tangent(a.length()).y - 1.0) < 0.05);
    // Curvature never exceeds the design radius, and REACHES (close to) it.
    Real kMax = 0;
    for (const AlignmentSample& s : a.samples())
        kMax = std::max(kMax, std::fabs(s.curvature));
    CHECK(kMax <= 1.0 / 60.0 + 1e-6);
    CHECK(kMax > 0.8 / 60.0);
    // Clothoid contract: curvature ramps — no sample-to-sample jump bigger
    // than the linear spiral rate allows (k step = ds / (R * Ls), padded).
    for (std::size_t i = 1; i < a.samples().size(); ++i) {
        const Real ds = a.samples()[i].station - a.samples()[i - 1].station;
        const Real dk = std::fabs(a.samples()[i].curvature -
                                  a.samples()[i - 1].curvature);
        CHECK(dk <= ds / (60.0 * 25.0) + 1e-4);
    }
    // Tangent continuity: heading turns no faster than curvature allows.
    for (std::size_t i = 1; i < a.samples().size(); ++i) {
        const Real ds = a.samples()[i].station - a.samples()[i - 1].station;
        const Real cosTurn = dot(a.samples()[i].tangent, a.samples()[i - 1].tangent);
        CHECK(cosTurn > std::cos(ds / 60.0 + 0.02));
    }
}

TEST_CASE(alignment_shrinks_the_fillet_when_legs_are_short) {
    // Legs too short for a 200 m-radius corner: the fitter tightens rather
    // than overrunning the legs, and the curve still gets where it's going.
    std::vector<Vec2> control{{0, 0}, {40, 0}, {40, 40}};
    Alignment a = Alignment::fromPolyline(control, 200.0, 60.0, 1.0);
    CHECK(!a.empty());
    CHECK((a.pos(0) - Vec2(0, 0)).length() < 0.5);
    CHECK((a.pos(a.length()) - Vec2(40, 40)).length() < 0.5);
}

TEST_CASE(alignment_offset_is_left_of_travel) {
    std::vector<Vec2> control{{0, 0}, {100, 0}};
    Alignment a = Alignment::fromPolyline(control, 50.0, 20.0, 2.0);
    // Heading +X, left is +Y.
    const Vec2 p = a.offset(50.0, 3.0);
    CHECK(std::fabs(p.y - 3.0) < 1e-6);
    CHECK(std::fabs(p.x - 50.0) < 1.0);
}

TEST_CASE(vertical_profile_is_continuous_through_crest_and_sag) {
    // Climb at 4%, crest over a 120 m curve, descend at -3%: the standard
    // freeway profile. Continuity + bounded grade is the whole contract.
    VerticalProfile v;
    v.pvis = {{0, 10, 0}, {400, 26, 120}, {800, 14, 0}};
    // Endpoint elevations honoured.
    CHECK(std::fabs(v.elevation(0) - 10.0) < 1e-6);
    CHECK(std::fabs(v.elevation(800) - 14.0) < 1e-6);
    // Continuous: no step bigger than grade * ds anywhere.
    Real prev = v.elevation(0);
    for (Real s = 1; s <= 800; s += 1) {
        const Real z = v.elevation(s);
        CHECK(std::fabs(z - prev) < 0.08);   // |grade| < 8% on this design
        prev = z;
    }
    // Grades: 4% climbing, -3% descending, and INSIDE the crest curve the
    // grade is strictly between them.
    CHECK(std::fabs(v.grade(100) - 0.04) < 0.002);
    CHECK(std::fabs(v.grade(700) - (-0.03)) < 0.002);
    const Real gMid = v.grade(400);
    CHECK(gMid < 0.04);
    CHECK(gMid > -0.03);
}

TEST_CASE(lane_schedule_counts_aux_spans_and_corridor_width_follows) {
    LaneSchedule ls;
    ls.throughLanes = 4;                       // CA-style: 4 per direction
    ls.aux.push_back({300, 520, true});        // exit deceleration lane
    CHECK(ls.lanesAt(100) == 4);
    CHECK(ls.lanesAt(400) == 5);
    CHECK(ls.lanesAt(600) == 4);

    CorridorDef c;
    c.lanes = ls;
    c.laneWidth = 3.6;
    // Width grows by exactly one lane over the aux span.
    CHECK(std::fabs((c.halfWidthAt(400) - c.halfWidthAt(100)) - 3.6) < 1e-6);
}

TEST_CASE(superelevation_banks_into_curves_and_caps) {
    std::vector<Vec2> control{{0, 0}, {300, 0}, {300, 300}};
    CorridorDef c;
    c.horizontal = Alignment::fromPolyline(control, 80.0, 30.0, 2.0);
    c.designSpeed = 30.0;                      // ~110 km/h
    // Straight entry: flat. Mid-corner: banked, capped at the max.
    CHECK(std::fabs(c.superelevationAt(10.0)) < 1e-6);
    Real eMax = 0;
    for (Real s = 0; s < c.horizontal.length(); s += 2)
        eMax = std::max(eMax, std::fabs(c.superelevationAt(s)));
    CHECK(eMax > 0.02);                        // it really banks
    CHECK(eMax <= c.superelevationMax + 1e-9); // and never beyond the cap
}
