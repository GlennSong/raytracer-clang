#include "test_framework.h"

#include "../src/engine/input/player_input.h"

using namespace engine;  // namespace migration (ADR-0015)

namespace {

Event keyDown(KeyCode key) {
    Event e(EventType::KeyPressed);
    e.key = key;
    return e;
}

Event gamepadConnected(int id) {
    Event e(EventType::GamepadConnected);
    e.gamepad = id;
    return e;
}

GamepadState padWithButton(GamepadButton b) {
    GamepadState s;
    s.connected = true;
    s.buttons[static_cast<std::size_t>(b)] = true;
    return s;
}

}  // namespace

TEST_CASE(players_add_and_index) {
    PlayerInputs players;
    int p0 = players.addPlayer({InputDeviceKind::KeyboardMouse, -1});
    int p1 = players.addPlayer({InputDeviceKind::Gamepad, 0});
    CHECK(p0 == 0);
    CHECK(p1 == 1);
    CHECK(players.playerCount() == 2);
    CHECK(players.player(p0).usesKeyboardMouse());
    CHECK(players.player(p1).usesGamepad(0));
}

TEST_CASE(players_gamepad_autojoin_fills_empty_slot) {
    PlayerInputs players;
    int slot = players.addPlayer();  // device None, waiting to join
    CHECK(players.player(slot).device().kind == InputDeviceKind::None);

    int joined = players.onGamepadConnected(3);
    CHECK(joined == slot);
    CHECK(players.player(slot).usesGamepad(3));

    // Reconnecting the same pad is idempotent (no new slot).
    CHECK(players.onGamepadConnected(3) == slot);
    CHECK(players.playerCount() == 1);
}

TEST_CASE(players_gamepad_autojoin_creates_new_slot) {
    PlayerInputs players;
    players.addPlayer({InputDeviceKind::KeyboardMouse, -1});

    int joined = players.onGamepadConnected(0);
    // No free slot (the only one holds KBM), so a new one is created.
    CHECK(joined == 1);
    CHECK(players.playerCount() == 2);
    CHECK(players.player(1).usesGamepad(0));
}

TEST_CASE(players_autojoin_can_be_disabled) {
    PlayerInputs players;
    players.setAutoJoinGamepads(false);
    int joined = players.onGamepadConnected(0);
    CHECK(joined == -1);
    CHECK(players.playerCount() == 0);
}

TEST_CASE(players_keyboard_event_routes_to_kbm_player_only) {
    PlayerInputs players;
    int kbm = players.addPlayer({InputDeviceKind::KeyboardMouse, -1});
    int pad = players.addPlayer({InputDeviceKind::Gamepad, 0});
    players.player(kbm).actions().bindButton("jump", KeyCode::Space);
    players.player(pad).actions().bindButton("jump", KeyCode::Space);

    players.beginFrame();
    players.routeEvent(keyDown(KeyCode::Space));

    // Only the keyboard player sees the key; the gamepad player does not.
    CHECK(players.player(kbm).actions().pressed("jump"));
    CHECK(!players.player(pad).actions().pressed("jump"));
}

TEST_CASE(players_gamepad_state_routes_by_device_id) {
    PlayerInputs players;
    int p0 = players.addPlayer({InputDeviceKind::Gamepad, 0});
    int p1 = players.addPlayer({InputDeviceKind::Gamepad, 1});
    players.player(p0).actions().bindButton("fire", GamepadButton::A);
    players.player(p1).actions().bindButton("fire", GamepadButton::A);

    GamepadSet pads;
    pads[1] = padWithButton(GamepadButton::A);  // only device 1 pressed

    players.beginFrame();
    players.updateGamepads(pads);

    CHECK(!players.player(p0).actions().held("fire"));
    CHECK(players.player(p1).actions().held("fire"));
}

TEST_CASE(players_disconnect_unassigns_but_keeps_slot) {
    PlayerInputs players;
    int slot = players.onGamepadConnected(2);
    CHECK(players.player(slot).usesGamepad(2));

    players.onGamepadDisconnected(2);
    CHECK(players.playerCount() == 1);  // slot retained for rejoin
    CHECK(players.player(slot).device().kind == InputDeviceKind::None);

    // The freed slot accepts a reconnecting pad.
    int rejoined = players.onGamepadConnected(2);
    CHECK(rejoined == slot);
}

TEST_CASE(players_route_connect_event_joins) {
    PlayerInputs players;
    players.beginFrame();
    players.routeEvent(gamepadConnected(0));
    CHECK(players.playerCount() == 1);
    CHECK(players.player(0).usesGamepad(0));
}
