#include "player_input.h"

namespace engine {

int PlayerInputs::addPlayer(InputDevice device) {
    players.push_back(std::make_unique<PlayerInput>());
    players.back()->assign(device);
    return static_cast<int>(players.size()) - 1;
}

int PlayerInputs::findPlayerForGamepad(int gamepadId) const {
    for (int i = 0; i < playerCount(); i++)
        if (players[i]->usesGamepad(gamepadId)) return i;
    return -1;
}

int PlayerInputs::onGamepadConnected(int gamepadId) {
    int existing = findPlayerForGamepad(gamepadId);
    if (existing >= 0) return existing;
    if (!autoJoin) return -1;

    // Prefer an existing slot with no device (a player waiting to (re)join),
    // otherwise spin up a new slot for the newcomer.
    for (int i = 0; i < playerCount(); i++) {
        if (players[i]->device().kind == InputDeviceKind::None) {
            players[i]->assign({InputDeviceKind::Gamepad, gamepadId});
            return i;
        }
    }
    return addPlayer({InputDeviceKind::Gamepad, gamepadId});
}

void PlayerInputs::onGamepadDisconnected(int gamepadId) {
    int index = findPlayerForGamepad(gamepadId);
    if (index < 0) return;
    // Keep the slot (the player may rejoin and keep their entity), but drop the
    // device and clear any held gamepad state.
    players[index]->assign({InputDeviceKind::None, -1});
    GamepadState empty;
    players[index]->actions().updateGamepad(empty);
}

void PlayerInputs::beginFrame() {
    for (auto& p : players) p->actions().beginFrame();
}

void PlayerInputs::routeEvent(const Event& event) {
    switch (event.type) {
        case EventType::GamepadConnected:
            onGamepadConnected(event.gamepad);
            return;
        case EventType::GamepadDisconnected:
            onGamepadDisconnected(event.gamepad);
            return;
        default:
            break;
    }

    // Keyboard/mouse events go to the player(s) holding that device.
    for (auto& p : players)
        if (p->usesKeyboardMouse()) p->actions().processEvent(event);
}

void PlayerInputs::updateGamepads(const GamepadSet& pads) {
    for (auto& p : players) {
        const InputDevice& dev = p->device();
        if (dev.kind != InputDeviceKind::Gamepad) continue;
        if (dev.gamepadId < 0 || dev.gamepadId >= MAX_GAMEPADS) continue;
        p->actions().updateGamepad(pads[dev.gamepadId]);
    }
}

}  // namespace engine

