#include "test_framework.h"

#include "../src/engine/input/input_map.h"

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
