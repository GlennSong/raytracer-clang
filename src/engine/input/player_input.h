#ifndef RAYTRACER_ENGINE_PLAYER_INPUT_H
#define RAYTRACER_ENGINE_PLAYER_INPUT_H

#include "input_map.h"
#include "../../renderer/event.h"
#include "../../renderer/gamepad.h"

#include <memory>
#include <vector>

namespace engine {

// Local-player input infrastructure (ADR-0010). A "player" here is purely an
// input concept — a human at a device with their own bindings — and carries NO
// gameplay state. Gameplay associates an entity with a player via the generic
// ControlledBy component; this layer never decides what a player *is*.

enum class InputDeviceKind { None, KeyboardMouse, Gamepad };

struct InputDevice {
    InputDeviceKind kind = InputDeviceKind::None;
    int gamepadId = -1;   // meaningful when kind == Gamepad
};

// One local player: an assigned device plus its own InputMap, so bindings and
// rebinding are per-player. Bindings are device-relative; routing (below)
// directs the right hardware to this slot.
class PlayerInput {
public:
    InputMap& actions() { return map; }
    const InputMap& actions() const { return map; }

    const InputDevice& device() const { return dev; }
    void assign(InputDevice device) { dev = device; }

    bool usesKeyboardMouse() const {
        return dev.kind == InputDeviceKind::KeyboardMouse;
    }
    bool usesGamepad(int id) const {
        return dev.kind == InputDeviceKind::Gamepad && dev.gamepadId == id;
    }

private:
    InputDevice dev;
    InputMap map;
};

// Owns the set of local players and routes hardware to the owning slot. A new
// gamepad auto-joins (assigned to a free slot, or a fresh one) unless that is
// turned off. Single-player is just the one-slot case.
class PlayerInputs {
public:
    int addPlayer(InputDevice device = {});
    int playerCount() const { return static_cast<int>(players.size()); }
    PlayerInput& player(int index) { return *players[index]; }
    const PlayerInput& player(int index) const { return *players[index]; }

    void setAutoJoinGamepads(bool enabled) { autoJoin = enabled; }

    // Returns the player index now owning the pad, or -1 if none took it.
    int onGamepadConnected(int gamepadId);
    void onGamepadDisconnected(int gamepadId);

    // Per-frame, in order: beginFrame() -> routeEvent() per event ->
    // updateGamepads() with the polled snapshot.
    void beginFrame();
    void routeEvent(const Event& event);
    void updateGamepads(const GamepadSet& pads);

private:
    int findPlayerForGamepad(int gamepadId) const;

    std::vector<std::unique_ptr<PlayerInput>> players;
    bool autoJoin = true;
};


}  // namespace engine

#endif
