#ifndef RAYTRACER_ENGINE_INPUT_MAP_H
#define RAYTRACER_ENGINE_INPUT_MAP_H

#include "../../rt_math.h"
#include "../../renderer/event.h"
#include "../../renderer/gamepad.h"

#include <array>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace engine {

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
// WHICH MODE AN ACTION BELONGS TO (Glenn, 2026-09-18: "There seems like a
// separation of concerns exist for in vehicle vs on foot"). Space is jump on
// foot and brake in a car; T is teleport on foot and flip in a car. Those are
// not collisions, they are one key meaning different things in different
// modes -- and until now nothing SAID that: both handlers fired and only their
// own guards kept them apart. With a context, only the active mode's action
// resolves, and a genuine clash (two actions, one key, SAME mode) is what gets
// reported.
enum class InputContext : uint8_t { Always, OnFoot, InVehicle };
const char* inputContextName(InputContext c);

class InputMap {
public:
    // --- Binding configuration ---
    void bindButton(const std::string& action, KeyCode key);
    void bindButton(const std::string& action, MouseButton button);
    void bindButton(const std::string& action, GamepadButton button);
    void bindButton(const std::string& action, XrButton button);  // pinch etc.
    void bindAxis(const std::string& axis, KeyCode key, Real scale);
    // Bind an analog gamepad axis (stick/trigger) to a named axis. Bindings are
    // device-relative ("left stick X"); the player layer routes a hardware pad
    // to the owning slot (ADR-0010).
    void bindAxis(const std::string& axis, GamepadAxis gamepadAxis, Real scale);

    // Below which magnitude a stick axis reads as zero (rescaled beyond it).
    void setDeadzone(Real value);

    // Same, but from a string key name (e.g. "Space", "W") so bindings can come
    // from Settings or a config file. Returns false (and binds nothing) if the
    // name is unrecognized.
    bool bindButtonByName(const std::string& action, const std::string& keyName);
    bool bindAxisByName(const std::string& axis, const std::string& keyName,
                        Real scale);

    void clearBindings();

    // WHO ELSE HAS THIS KEY. Eight keys in this engine already carry two or
    // three actions (F is cam_detach AND editor_frame; Space is drive_brake AND
    // player_jump), and that works only because their owners are never active
    // at once -- an accident that holds, not a rule that is enforced. Nothing
    // warned when a NINTH was added: binding E to transit_board beside
    // elevator_call silently let a passing bus capture the player, which then
    // beat the fast-travel key (Glenn, 2026-09-18: "This is the fourth time
    // today. Why does it keep breaking??").
    //
    // So: every binding is reported, and a clash is logged at startup where it
    // can be seen, rather than discovered from the far end of a bug.
    std::vector<std::string> actionsFor(KeyCode key) const;

    // Tag an action with the mode it belongs to (untagged = Always).
    void setActionContext(const std::string& action, InputContext c);
    InputContext actionContext(const std::string& action) const;
    // The mode the player is in. Only actions of this context (or Always)
    // answer held/pressed/released.
    void setContext(InputContext c) { context_ = c; }
    InputContext context() const { return context_; }
    // Keys whose actions can fire TOGETHER: same context, or one of them
    // Always. Shared keys split across OnFoot/InVehicle are deliberate and
    // are not listed.
    std::vector<std::string> collisions() const;
    // Every (key, actions) pair, sorted, for `keys?` and for tests.
    std::vector<std::pair<std::string, std::vector<std::string>>> bindingReport() const;

    // --- Per-frame lifecycle (driven by Application) ---
    void beginFrame();                      // clears this frame's pressed/released
    void processEvent(const Event& event);  // keyboard/mouse held + edge state
    // While a UI text field owns the keyboard, key presses are TEXT, not
    // actions: a comma typed into the Teleport paste box halved the sim
    // speed (sim_slower) and the setting persisted — "the player and the
    // car move sluggish". Releases still clear held state.
    void setTextInputCaptured(bool captured);
    bool textInputCaptured() const { return textCaptured; }
    // Polled gamepad snapshot for this map's assigned device. Derives button
    // edges by diffing against the previous frame and stores deadzoned axis
    // values. Pass a disconnected state to clear (e.g. on unassign).
    void updateGamepad(const GamepadState& pad);

    // --- Queries ---
    bool held(const std::string& action) const;      // currently down
    bool pressed(const std::string& action) const;   // went down this frame
    bool released(const std::string& action) const;  // went up this frame
    Real axis(const std::string& name) const;        // summed, clamped [-1, 1]

    // One row per (action, physical input) pair — the whole live keymap, for
    // the debug overlay's Controls table. `input` is a display name ("Space",
    // "Pad B", "Mouse Left", "Pad LeftX"); axes carry their per-input scale so
    // the table can show W(+)/S(-). Sorted by input then action, so every
    // multi-action key sits on adjacent rows (that adjacency is how the
    // overlay spots binding collisions).
    struct BindingDesc {
        std::string action;
        std::string input;
        bool isAxis = false;
        Real scale = 0;    // axis contributions only; 0 for buttons
    };
    std::vector<BindingDesc> listBindings() const;

private:
    // An axis contribution is either a digital source (held → +scale) or an
    // analog gamepad axis (+scale * value).
    struct AxisContribution {
        enum class Kind { Digital, Gamepad } kind;
        int source;   // Digital: encoded source; Gamepad: GamepadAxis index
        Real scale;
    };

    static int encodeKey(KeyCode key);
    static int encodeMouse(MouseButton button);
    static int encodeGamepadButton(GamepadButton button);
    static int encodeXrButton(XrButton button);
    Real applyDeadzone(Real value) const;
    bool anyBoundSourceIn(const std::string& action,
                          const std::unordered_set<int>& sources) const;

    std::unordered_map<std::string, std::vector<int>> buttons;
    std::unordered_map<std::string, InputContext> actionContext_;
    InputContext context_ = InputContext::OnFoot;
    bool live(const std::string& action) const;
    std::unordered_map<std::string, std::vector<AxisContribution>> axes;

    std::unordered_set<int> heldSources;
    std::unordered_set<int> pressedSources;
    bool textCaptured = false;   // setTextInputCaptured: a UI text field owns the keys
    std::unordered_set<int> releasedSources;

    std::array<bool, GAMEPAD_BUTTON_COUNT> prevGamepadButtons{};
    std::array<Real, GAMEPAD_AXIS_COUNT> gamepadAxisValues{};
    Real deadzone = 0.15;
};

// Canonical name <-> KeyCode mapping for data-driven bindings. Names match the
// KeyCode enumerator spelling ("A", "Space", "LeftShift", "Comma", ...).
// keyCodeFromName returns KeyCode::Unknown if the name is unrecognized.
KeyCode keyCodeFromName(const std::string& name);
const char* keyCodeName(KeyCode key);


}  // namespace engine

#endif
