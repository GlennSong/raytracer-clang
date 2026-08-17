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
    // Enter, not Space: Space is the car's BRAKE pedal (VehicleSystem), and
    // pausing the whole sim every time the player braked was the collision that
    // prompted the Controls section in the debug overlay. Override with
    // bind.pause in settings if Enter doesn't suit.
    {"pause", KeyCode::Enter},
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
              << "  Enter=pause, ','/'.'=slower/faster sim, 0=reset speed\n"
              << "  P=toggle perspective/orthographic camera\n"
              << "  F=detach/attach freecam (mouse looks, WASD/QE fly)\n"
              << "  C=place camera here, V/B=cycle viewports, X=editor view\n"
              << "  On foot: V first/third person, R respawn, T teleport, 1/2 gun away/out\n"
              << "  Car: G in/out, W/S gas/reverse, A/D steer, Space=BRAKE,\n"
              << "       Ctrl=handbrake, H=horn, T=flip upright, L=lights\n"
              << "  (` debug overlay -> Controls lists every live binding;\n"
              << "   keys configurable via bind.<action> in settings)\n";
}

void DevControlSystem::update(FrameContext& ctx) {
    // Edge actions are consumed once per frame here (not per event), so a single
    // key press toggles exactly once regardless of how many events the frame saw.
    if (ownsQuit && ctx.actions.pressed("quit")) ctx.quit = true;
    if (ctx.actions.pressed("pause")) ctx.clock.setPaused(!ctx.clock.paused());
    if (ctx.actions.pressed("sim_slower"))
        simSpeed = std::clamp(simSpeed * 0.5, 0.0625, 16.0);
    if (ctx.actions.pressed("sim_faster"))
        simSpeed = std::clamp(simSpeed * 2.0, 0.0625, 16.0);
    if (ctx.actions.pressed("sim_reset")) {
        simSpeed = 1.0;
        ctx.clock.setPaused(false);
    }

    // Pause lives on the clock itself (shared with the editor shell's Pause
    // button); this system owns only the speed.
    ctx.clock.setTimeScale(simSpeed);
}

void DevControlSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("timeScale", simSpeed);
}

}  // namespace engine

