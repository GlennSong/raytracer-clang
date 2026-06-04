#ifndef RAYTRACER_ENGINE_INPUT_MAP_H
#define RAYTRACER_ENGINE_INPUT_MAP_H

#include "../../math.h"
#include "../../renderer/event.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Named input actions decoupled from physical keys. Systems ask for
// "move_forward" or "pause" rather than KeyCode::W or KeyCode::Space, and the
// bindings live in a data-driven table (settable from Settings/config). This
// retires the hardcoded keybindings in DevControlSystem and is the layer the
// fly camera (ROADMAP 2.2) builds on.
//
// Two flavours of action:
//   - Buttons: digital. Query held() for continuous "is down", or the per-frame
//     edges pressed()/released() for discrete reactions.
//   - Axes: analog in [-1, 1], summed from held keys with per-key scales (e.g.
//     W = +1, S = -1 -> "move_forward").
//
// Backend-neutral: driven entirely by our own Event/KeyCode types, so it is
// testable without a window. Application feeds it: beginFrame() once per frame,
// then processEvent() for each event, before systems run.
class InputMap {
public:
    // --- Binding configuration ---
    void bindButton(const std::string& action, KeyCode key);
    void bindButton(const std::string& action, MouseButton button);
    void bindAxis(const std::string& axis, KeyCode key, Real scale);

    // Same, but from a string key name (e.g. "Space", "W") so bindings can come
    // from Settings or a config file. Returns false (and binds nothing) if the
    // name is unrecognized.
    bool bindButtonByName(const std::string& action, const std::string& keyName);
    bool bindAxisByName(const std::string& axis, const std::string& keyName,
                        Real scale);

    void clearBindings();

    // --- Per-frame lifecycle (driven by Application) ---
    void beginFrame();                      // clears this frame's pressed/released
    void processEvent(const Event& event);  // updates held + edge state

    // --- Queries ---
    bool held(const std::string& action) const;      // currently down
    bool pressed(const std::string& action) const;   // went down this frame
    bool released(const std::string& action) const;  // went up this frame
    Real axis(const std::string& name) const;        // summed, clamped [-1, 1]

private:
    static int encodeKey(KeyCode key);
    static int encodeMouse(MouseButton button);
    bool anyBoundSourceIn(const std::string& action,
                          const std::unordered_set<int>& sources) const;

    std::unordered_map<std::string, std::vector<int>> buttons;
    std::unordered_map<std::string, std::vector<std::pair<int, Real>>> axes;

    std::unordered_set<int> heldSources;
    std::unordered_set<int> pressedSources;
    std::unordered_set<int> releasedSources;
};

// Canonical name <-> KeyCode mapping for data-driven bindings. Names match the
// KeyCode enumerator spelling ("A", "Space", "LeftShift", "Comma", ...).
// keyCodeFromName returns KeyCode::Unknown if the name is unrecognized.
KeyCode keyCodeFromName(const std::string& name);
const char* keyCodeName(KeyCode key);

#endif
