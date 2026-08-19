#ifndef RAYTRACER_ENGINE_STATES_EDITOR_HOME_VIEW_H
#define RAYTRACER_ENGINE_STATES_EDITOR_HOME_VIEW_H

#include <string>

namespace engine {

// When does the editor snap its camera back to the HOME view? On editor
// STARTUP and on every LEVEL SWITCH — but not on a same-level re-entry, which
// is the playtest round-trip (Esc out of a playtest): yanking the camera to
// the origin after every playtest would lose the spot being edited, the exact
// annoyance this policy exists to remove in the other direction (Glenn:
// "always start at the origin when you start the editor or switch levels";
// previously the fly pose persisted in settings.json, so opening a new level
// left the camera wherever the LAST level's session wandered — kilometres of
// empty terrain away).
//
// Pure and process-lifetime by design: the caller keeps one instance alive
// across EditorState swaps (a function-local static — states themselves are
// destroyed on every transition). Headless-tested in run_tests.
struct EditorHomeViewPolicy {
    std::string lastLevel;   // empty = no editor entry yet this process

    // True when entering `level` should reset the camera to home. Also
    // records the entry, so ask exactly once per onEnter.
    bool shouldReset(const std::string& level) {
        const bool reset = lastLevel != level;   // first entry: "" != level
        lastLevel = level;
        return reset;
    }
};

// The home pose: an aerial framing of the ORIGIN (not eye-at-origin, which
// buries the view in whatever geometry sits at 0,0) — south of it, looking
// north down -Z, high enough that a couple of city blocks read at a glance.
constexpr double kEditorHomeEyeX = 0.0;
constexpr double kEditorHomeEyeY = 90.0;
constexpr double kEditorHomeEyeZ = 180.0;
constexpr double kEditorHomeYawDeg = 0.0;      // facing -Z, +X screen-right
constexpr double kEditorHomePitchDeg = -24.0;  // tipped down at the origin

}  // namespace engine

#endif
