#include "test_framework.h"

#include "../src/engine/input/input_map.h"

#include <cmath>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// Build the events the window layer would produce, for driving InputMap.
Event keyDown(KeyCode key, bool repeat = false) {
    Event e(EventType::KeyPressed);
    e.key = key;
    e.repeat = repeat;
    return e;
}

Event keyUp(KeyCode key) {
    Event e(EventType::KeyReleased);
    e.key = key;
    return e;
}

Event mouseDown(MouseButton button) {
    Event e(EventType::MouseButtonPressed);
    e.button = button;
    return e;
}

GamepadState pad() {
    GamepadState s;
    s.connected = true;
    return s;
}

void setButton(GamepadState& s, GamepadButton b, bool down) {
    s.buttons[static_cast<std::size_t>(b)] = down;
}

void setAxis(GamepadState& s, GamepadAxis a, float v) {
    s.axes[static_cast<std::size_t>(a)] = v;
}

constexpr Real EPS = 1e-9;

}  // namespace

TEST_CASE(input_button_press_held_release) {
    InputMap map;
    map.bindButton("jump", KeyCode::Space);

    map.beginFrame();
    map.processEvent(keyDown(KeyCode::Space));
    CHECK(map.pressed("jump"));
    CHECK(map.held("jump"));
    CHECK(!map.released("jump"));

    // Next frame: still held, but no longer a fresh press.
    map.beginFrame();
    CHECK(!map.pressed("jump"));
    CHECK(map.held("jump"));

    map.beginFrame();
    map.processEvent(keyUp(KeyCode::Space));
    CHECK(map.released("jump"));
    CHECK(!map.held("jump"));
    CHECK(!map.pressed("jump"));
}

TEST_CASE(input_key_repeat_is_not_an_edge) {
    InputMap map;
    map.bindButton("fire", KeyCode::F);

    map.beginFrame();
    map.processEvent(keyDown(KeyCode::F));
    CHECK(map.pressed("fire"));

    // A repeat in a later frame must not count as a new press, but stays held.
    map.beginFrame();
    map.processEvent(keyDown(KeyCode::F, /*repeat=*/true));
    CHECK(!map.pressed("fire"));
    CHECK(map.held("fire"));
}

TEST_CASE(input_multiple_sources_one_action) {
    InputMap map;
    map.bindButton("select", KeyCode::Enter);
    map.bindButton("select", MouseButton::Left);

    map.beginFrame();
    map.processEvent(mouseDown(MouseButton::Left));
    CHECK(map.pressed("select"));
    CHECK(map.held("select"));

    // The other bound source independently triggers the same action.
    InputMap map2;
    map2.bindButton("select", KeyCode::Enter);
    map2.bindButton("select", MouseButton::Left);
    map2.beginFrame();
    map2.processEvent(keyDown(KeyCode::Enter));
    CHECK(map2.pressed("select"));
}

TEST_CASE(input_unknown_action_is_inert) {
    InputMap map;
    map.beginFrame();
    map.processEvent(keyDown(KeyCode::A));
    CHECK(!map.pressed("nope"));
    CHECK(!map.held("nope"));
    CHECK_APPROX(map.axis("nope"), 0, EPS);
}

TEST_CASE(input_axis_from_opposing_keys) {
    InputMap map;
    map.bindAxis("move", KeyCode::W, 1.0);
    map.bindAxis("move", KeyCode::S, -1.0);

    map.beginFrame();
    map.processEvent(keyDown(KeyCode::W));
    CHECK_APPROX(map.axis("move"), 1.0, EPS);

    // Holding both opposing keys cancels out.
    map.processEvent(keyDown(KeyCode::S));
    CHECK_APPROX(map.axis("move"), 0.0, EPS);

    // Release forward, only backward remains.
    map.beginFrame();
    map.processEvent(keyUp(KeyCode::W));
    CHECK_APPROX(map.axis("move"), -1.0, EPS);
}

TEST_CASE(input_axis_clamped_to_unit_range) {
    InputMap map;
    map.bindAxis("strafe", KeyCode::D, 1.0);
    map.bindAxis("strafe", KeyCode::Right, 1.0);

    map.beginFrame();
    map.processEvent(keyDown(KeyCode::D));
    map.processEvent(keyDown(KeyCode::Right));
    // Two +1 contributions sum to 2 but clamp to 1.
    CHECK_APPROX(map.axis("strafe"), 1.0, EPS);
}

TEST_CASE(input_focus_loss_clears_held) {
    InputMap map;
    map.bindButton("walk", KeyCode::W);

    map.beginFrame();
    map.processEvent(keyDown(KeyCode::W));
    CHECK(map.held("walk"));

    Event unfocus(EventType::WindowUnfocused);
    map.processEvent(unfocus);
    CHECK(!map.held("walk"));
}

TEST_CASE(input_key_name_round_trip) {
    CHECK(keyCodeFromName("Space") == KeyCode::Space);
    CHECK(keyCodeFromName("W") == KeyCode::W);
    CHECK(keyCodeFromName("LeftShift") == KeyCode::LeftShift);
    CHECK(keyCodeFromName("Comma") == KeyCode::Comma);
    CHECK(keyCodeFromName("bogus") == KeyCode::Unknown);

    CHECK(std::string(keyCodeName(KeyCode::Space)) == "Space");
    CHECK(std::string(keyCodeName(KeyCode::Up)) == "Up");
}

TEST_CASE(input_gamepad_button_edges) {
    InputMap map;
    map.bindButton("jump", GamepadButton::A);

    // Press: held + pressed edge derived from the polled snapshot.
    GamepadState s = pad();
    setButton(s, GamepadButton::A, true);
    map.beginFrame();
    map.updateGamepad(s);
    CHECK(map.pressed("jump"));
    CHECK(map.held("jump"));

    // Next frame still down: held, but no fresh press edge.
    map.beginFrame();
    map.updateGamepad(s);
    CHECK(!map.pressed("jump"));
    CHECK(map.held("jump"));

    // Release.
    GamepadState up = pad();
    map.beginFrame();
    map.updateGamepad(up);
    CHECK(map.released("jump"));
    CHECK(!map.held("jump"));
}

TEST_CASE(input_gamepad_axis_with_deadzone) {
    InputMap map;
    map.bindAxis("move_x", GamepadAxis::LeftX, 1.0);

    // Inside the default 0.15 deadzone reads as zero.
    GamepadState small = pad();
    setAxis(small, GamepadAxis::LeftX, 0.1f);
    map.beginFrame();
    map.updateGamepad(small);
    CHECK_APPROX(map.axis("move_x"), 0.0, 1e-6);

    // 0.575 past a 0.15 deadzone rescales to 0.5 ((0.575-0.15)/0.85).
    GamepadState half = pad();
    setAxis(half, GamepadAxis::LeftX, 0.575f);
    map.beginFrame();
    map.updateGamepad(half);
    CHECK_APPROX(map.axis("move_x"), 0.5, 1e-5);

    // Full deflection saturates at 1.
    GamepadState full = pad();
    setAxis(full, GamepadAxis::LeftX, 1.0f);
    map.beginFrame();
    map.updateGamepad(full);
    CHECK_APPROX(map.axis("move_x"), 1.0, 1e-6);
}

TEST_CASE(input_axis_mixes_keyboard_and_gamepad) {
    InputMap map;
    map.bindAxis("move", KeyCode::W, 1.0);
    map.bindAxis("move", GamepadAxis::LeftY, -1.0);

    // Keyboard contribution alone.
    map.beginFrame();
    map.processEvent(keyDown(KeyCode::W));
    map.updateGamepad(pad());
    CHECK_APPROX(map.axis("move"), 1.0, EPS);

    // Stick pushing the other way cancels the held key.
    GamepadState s = pad();
    setAxis(s, GamepadAxis::LeftY, 1.0f);  // *-1 scale -> -1, plus +1 key = 0
    map.beginFrame();
    map.updateGamepad(s);
    CHECK_APPROX(map.axis("move"), 0.0, 1e-6);
}

TEST_CASE(input_gamepad_disconnect_clears_state) {
    InputMap map;
    map.bindButton("fire", GamepadButton::RightBumper);
    map.bindAxis("aim", GamepadAxis::RightX, 1.0);

    GamepadState s = pad();
    setButton(s, GamepadButton::RightBumper, true);
    setAxis(s, GamepadAxis::RightX, 1.0f);
    map.beginFrame();
    map.updateGamepad(s);
    CHECK(map.held("fire"));

    // A disconnected snapshot releases the button and zeroes axes.
    GamepadState gone;  // connected == false
    map.beginFrame();
    map.updateGamepad(gone);
    CHECK(!map.held("fire"));
    CHECK(map.released("fire"));
    CHECK_APPROX(map.axis("aim"), 0.0, EPS);
}

TEST_CASE(input_bind_by_name) {
    InputMap map;
    CHECK(map.bindButtonByName("pause", "Space"));
    CHECK(!map.bindButtonByName("pause", "NotAKey"));

    map.beginFrame();
    map.processEvent(keyDown(KeyCode::Space));
    CHECK(map.pressed("pause"));

    InputMap axisMap;
    CHECK(axisMap.bindAxisByName("throttle", "Up", 1.0));
    CHECK(!axisMap.bindAxisByName("throttle", "Nope", 1.0));
    axisMap.beginFrame();
    axisMap.processEvent(keyDown(KeyCode::Up));
    CHECK_APPROX(axisMap.axis("throttle"), 1.0, EPS);
}

TEST_CASE(input_list_bindings_names_every_source_sorted_by_input) {
    // The debug overlay's Controls table reads this: one row per
    // (action, input) pair, inputs decoded to display names, sorted by input
    // so a key bound to several actions lands on adjacent rows.
    InputMap map;
    map.bindButton("drive_brake", KeyCode::Space);
    map.bindButton("pause", KeyCode::Space);          // a deliberate collision
    map.bindButton("fire", MouseButton::Left);
    map.bindButton("drive_brake", GamepadButton::B);
    map.bindAxis("drive_throttle", KeyCode::W, 1.0);
    map.bindAxis("drive_throttle", KeyCode::S, -1.0);
    map.bindAxis("drive_steer", GamepadAxis::LeftX, 1.0);

    std::vector<InputMap::BindingDesc> rows = map.listBindings();
    CHECK(rows.size() == 7u);
    for (size_t i = 1; i < rows.size(); i++)
        CHECK(rows[i - 1].input <= rows[i].input);      // sorted by input name

    auto has = [&](const char* action, const char* input, bool isAxis,
                   Real scale) {
        for (const auto& r : rows)
            if (r.action == action && r.input == input && r.isAxis == isAxis &&
                std::fabs(r.scale - scale) < 1e-9)
                return true;
        return false;
    };
    CHECK(has("drive_brake", "Space", false, 0.0));
    CHECK(has("pause", "Space", false, 0.0));           // the collision is visible
    CHECK(has("fire", "Mouse Left", false, 0.0));
    CHECK(has("drive_brake", "Pad B", false, 0.0));
    CHECK(has("drive_throttle", "W", true, 1.0));
    CHECK(has("drive_throttle", "S", true, -1.0));
    CHECK(has("drive_steer", "Pad LeftX", true, 1.0));

    // The two Space rows are adjacent — the property the overlay's
    // collision-highlight pass depends on.
    for (size_t i = 0; i < rows.size(); i++)
        if (rows[i].input == "Space") {
            CHECK(i + 1 < rows.size());
            CHECK(rows[i + 1].input == "Space");
            break;
        }
}
