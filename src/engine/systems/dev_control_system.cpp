#include "dev_control_system.h"

#include <algorithm>
#include <iostream>

namespace engine {

namespace {

// Default key for each action, overridable via a "bind.<action>" Settings entry
// (e.g. bind.pause = "Enter"). The action names are the stable contract;
// physical keys are data.
struct ActionBinding {
    const char* action;
    KeyCode defaultKey;
};

const ActionBinding DEV_ACTIONS[] = {
    {"quit", KeyCode::Escape},
    {"pause", KeyCode::Space},
    {"sim_slower", KeyCode::Comma},
    {"sim_faster", KeyCode::Period},
    {"sim_reset", KeyCode::Num0},
};

}  // namespace

void DevControlSystem::onStart(FrameContext& ctx) {
    simSpeed = ctx.settings.getDouble("timeScale", 1.0);

    for (const ActionBinding& binding : DEV_ACTIONS) {
        std::string overrideKey = ctx.settings.getString(
            std::string("bind.") + binding.action, "");
        if (!overrideKey.empty() &&
            ctx.actions.bindButtonByName(binding.action, overrideKey)) {
            continue;  // settings supplied a valid key for this action
        }
        ctx.actions.bindButton(binding.action, binding.defaultKey);
    }

    std::cerr << "Controls:\n"
              << "  Left-drag=orbit, Right-drag=pan, Scroll=zoom\n"
              << "  WASD=move, QE=up/down, Shift=fast\n"
              << "  Up/Down=exposure, Esc=quit\n"
              << "  Space=pause, ','/'.'=slower/faster sim, 0=reset speed\n"
              << "  P=toggle perspective/orthographic camera\n"
              << "  C=place camera here, V/B=cycle viewports, Backspace=editor view\n"
              << "  (keys configurable via bind.<action> in settings)\n";
}

void DevControlSystem::update(FrameContext& ctx) {
    // Edge actions are consumed once per frame here (not per event), so a single
    // key press toggles exactly once regardless of how many events the frame saw.
    if (ctx.actions.pressed("quit")) ctx.quit = true;
    if (ctx.actions.pressed("pause")) paused = !paused;
    if (ctx.actions.pressed("sim_slower"))
        simSpeed = std::clamp(simSpeed * 0.5, 0.0625, 16.0);
    if (ctx.actions.pressed("sim_faster"))
        simSpeed = std::clamp(simSpeed * 2.0, 0.0625, 16.0);
    if (ctx.actions.pressed("sim_reset")) {
        simSpeed = 1.0;
        paused = false;
    }

    ctx.clock.setTimeScale(paused ? 0.0 : simSpeed);
}

void DevControlSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("timeScale", simSpeed);
}

}  // namespace engine

