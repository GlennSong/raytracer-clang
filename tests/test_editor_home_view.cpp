// The editor's camera home policy (editor_home_view.h): reset to the origin
// framing on editor STARTUP and on LEVEL SWITCH, but never on a same-level
// re-entry — that's the Esc-out-of-a-playtest round trip, where snapping away
// would lose the spot being edited.
#include "test_framework.h"

#include "../src/engine/states/editor_home_view.h"

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(editor_home_first_entry_of_the_process_resets) {
    EditorHomeViewPolicy p;
    CHECK(p.shouldReset("assets/levels/metropolis_sky.json"));
}

TEST_CASE(editor_home_playtest_round_trip_keeps_the_pose) {
    EditorHomeViewPolicy p;
    CHECK(p.shouldReset("a.json"));
    // Play, then Esc back into the editor on the SAME level: no reset.
    CHECK(!p.shouldReset("a.json"));
    CHECK(!p.shouldReset("a.json"));
}

TEST_CASE(editor_home_level_switch_resets_each_direction) {
    EditorHomeViewPolicy p;
    CHECK(p.shouldReset("a.json"));
    CHECK(p.shouldReset("b.json"));    // switch away
    CHECK(!p.shouldReset("b.json"));   // playtest round trip on b
    CHECK(p.shouldReset("a.json"));    // and switching BACK is a switch too
}

TEST_CASE(editor_home_pose_is_an_aerial_origin_framing) {
    // The pose frames the origin from above and south — not eye-at-origin
    // (which buries the camera in whatever stands at 0,0). Pin the intent,
    // not the exact numbers: south of origin, meaningfully high, pitched
    // down, facing -Z.
    CHECK(kEditorHomeEyeZ > 0.0);
    CHECK(kEditorHomeEyeY > 10.0);
    CHECK(kEditorHomePitchDeg < 0.0);
    CHECK(kEditorHomeYawDeg == 0.0);
    CHECK(kEditorHomeEyeX == 0.0);
}
