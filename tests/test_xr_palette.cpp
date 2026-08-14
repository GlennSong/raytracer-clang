#include "test_framework.h"

#include "../src/engine/xr/xr_palette.h"

#include <cmath>

using namespace engine;

// The palm palette FSM (engine/xr/xr_palette.h), driven with synthetic palm
// poses. The hysteresis and linger behaviours encoded here were tuned
// against on-device strobing — the palette blinking several times a second
// while a palm sat at the threshold (device logs, sessions three and four).

namespace {

XrPalette::Config config() {
    XrPalette::Config c;
    c.itemCount = 5;
    return c;
}

// A valid palm whose normal is `angle` radians off +Y, fingers along -Z.
XrPalmPose palmAt(Real angleFromUp, const Vec3& position = Vec3(0, 1, -0.3)) {
    XrPalmPose palm;
    palm.valid = true;
    palm.position = position;
    palm.normal = Vec3(std::sin(angleFromUp), std::cos(angleFromUp), 0);
    palm.fingers = Vec3(0, 0, -1);
    palm.across = normalize(cross(palm.normal, palm.fingers));
    return palm;
}

XrPalmPose noPalm() { return XrPalmPose{}; }

}  // namespace

TEST_CASE(palette_opens_on_palm_up_and_prefers_first_free_hand) {
    XrPalette palette(config());
    XrPalette::Inputs in;
    in.palm[0] = palmAt(0.2);
    in.palm[1] = palmAt(0.2);
    palette.update(in, 0.0);
    CHECK(palette.anchorHand() == 0);
    CHECK(palette.shownEdge());

    // A busy left hand cedes the palette to the right.
    XrPalette palette2(config());
    in.handBusy[0] = true;
    palette2.update(in, 0.0);
    CHECK(palette2.anchorHand() == 1);
}

TEST_CASE(palette_hysteresis_holds_between_show_and_keep_angles) {
    XrPalette palette(config());
    XrPalette::Inputs in;
    in.palm[0] = palmAt(0.5);   // inside showAngle 0.7
    palette.update(in, 0.0);
    CHECK(palette.anchorHand() == 0);

    // Tilted to 1.0 rad: past show, inside keep — HELD, no strobing.
    for (int i = 0; i < 60; i++) {
        in.palm[0] = palmAt(1.0);
        palette.update(in, 0.1 + i * 0.011);
        CHECK(palette.anchorHand() == 0);
        CHECK(!palette.hiddenEdge());
    }
    // And a fresh palette at 1.0 rad does NOT open (show is tight).
    XrPalette fresh(config());
    fresh.update(in, 0.0);
    CHECK(fresh.anchorHand() == -1);
}

TEST_CASE(palette_linger_survives_a_dip_but_not_a_put_away) {
    XrPalette palette(config());
    XrPalette::Inputs in;
    in.palm[0] = palmAt(0.2);
    palette.update(in, 0.0);
    CHECK(palette.anchorHand() == 0);

    // Palm dips well past keep for 0.4s (< linger 0.6): still shown.
    in.palm[0] = palmAt(1.6);
    palette.update(in, 0.4);
    CHECK(palette.anchorHand() == 0);
    // Recovers: timer re-arms.
    in.palm[0] = palmAt(0.2);
    palette.update(in, 0.5);
    CHECK(palette.anchorHand() == 0);
    // Down past the linger: hidden, with the edge reported once.
    in.palm[0] = palmAt(1.6);
    palette.update(in, 0.7);
    CHECK(palette.anchorHand() == 0);   // 0.2s in — lingering
    palette.update(in, 1.2);
    CHECK(palette.anchorHand() == -1);
    CHECK(palette.hiddenEdge());

    // Tracking loss (invalid palm) ends the linger immediately.
    XrPalette palette2(config());
    in.palm[0] = palmAt(0.2);
    palette2.update(in, 0.0);
    CHECK(palette2.anchorHand() == 0);
    in.palm[0] = noPalm();
    palette2.update(in, 0.1);
    CHECK(palette2.anchorHand() == -1);
}

TEST_CASE(palette_row_runs_along_the_fingers_axis) {
    XrPalette palette(config());
    const XrPalmPose palm = palmAt(0.0);
    const Vec3 first = palette.slotPosition(palm, 0);
    const Vec3 last = palette.slotPosition(palm, 4);
    // Fingers point -Z: the row spans Z, is centred on the palm, and every
    // slot floats `lift` above it.
    CHECK(std::abs((last - first).z - (-4 * 0.09)) < 1e-9);
    CHECK(std::abs(first.x - palm.position.x) < 1e-9);
    const Vec3 mid = palette.slotPosition(palm, 2);
    CHECK(std::abs(mid.y - (palm.position.y + 0.09)) < 1e-9);
    CHECK(std::abs(mid.z - palm.position.z) < 1e-9);
}

TEST_CASE(palette_hover_picks_nearest_slot_with_the_free_hand) {
    XrPalette palette(config());
    XrPalette::Inputs in;
    in.palm[0] = palmAt(0.1);
    palette.update(in, 0.0);
    CHECK(palette.anchorHand() == 0);
    CHECK(palette.hoverItem() == -1);

    // Right index tip lands on slot 3.
    in.fingertipValid[1] = true;
    in.fingertip[1] = palette.slotPosition(in.palm[0], 3) + Vec3(0.01, 0, 0);
    palette.update(in, 0.1);
    CHECK(palette.hoverItem() == 3);
    CHECK(palette.hoverChanged());

    // Off the row entirely: hover clears.
    in.fingertip[1] = in.palm[0].position + Vec3(0.5, 0.5, 0.5);
    palette.update(in, 0.2);
    CHECK(palette.hoverItem() == -1);
}

TEST_CASE(palette_left_right_symmetry) {
    // The same geometry mirrored to the right hand anchors there — the
    // lefty support is that there is no preferred hand at all.
    XrPalette palette(config());
    XrPalette::Inputs in;
    in.palm[1] = palmAt(0.2, Vec3(0.3, 1.0, -0.3));
    palette.update(in, 0.0);
    CHECK(palette.anchorHand() == 1);
    in.fingertipValid[0] = true;
    in.fingertip[0] = palette.slotPosition(in.palm[1], 1);
    palette.update(in, 0.1);
    CHECK(palette.hoverItem() == 1);
}

TEST_CASE(palette_wraps_into_centred_rows_past_max_per_row) {
    // 9 items, 5 per row: rows of 5 and 4, each centred on its own count,
    // offset across the palm. 5 items stay a single centred row (the layout
    // every earlier test pins).
    XrPalette::Config c;
    c.itemCount = 9;
    XrPalette palette(c);
    const XrPalmPose palm = palmAt(0.0);

    // Row 0 spans slots 0..4 along fingers at across-offset -rowSpacing/2.
    const Vec3 s0 = palette.slotPosition(palm, 0);
    const Vec3 s4 = palette.slotPosition(palm, 4);
    CHECK(std::abs((s4 - s0).z - (-4 * c.slotSpacing)) < 1e-9);
    const Vec3 mid0 = palette.slotPosition(palm, 2);
    CHECK(std::abs(mid0.z - palm.position.z) < 1e-9);   // centred row

    // Row 1 (4 items) sits one rowSpacing across from row 0 and is centred
    // on ITS count: slots 5..8 straddle the palm centre.
    const Vec3 s5 = palette.slotPosition(palm, 5);
    const Vec3 s8 = palette.slotPosition(palm, 8);
    const Vec3 acrossDelta = s5 - s0;
    CHECK(std::abs(dot(acrossDelta, palm.across) - c.rowSpacing) < 1e-9);
    CHECK(std::abs((s5.z + s8.z) * 0.5 - palm.position.z) < 1e-9);
    // Every slot floats the same lift above the palm.
    CHECK(std::abs(s8.y - (palm.position.y + c.lift)) < 1e-9);
}
