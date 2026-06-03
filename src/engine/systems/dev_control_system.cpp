#include "dev_control_system.h"

#include <algorithm>
#include <iostream>

void DevControlSystem::onStart(FrameContext& ctx) {
    simSpeed = ctx.settings.getDouble("timeScale", 1.0);

    std::cerr << "Controls:\n"
              << "  Left-drag=orbit, Right-drag=pan, Scroll=zoom\n"
              << "  WASD=move, QE=up/down, Shift=fast\n"
              << "  Up/Down=exposure, Esc=quit\n"
              << "  Space=pause, ','/'.'=slower/faster sim, 0=reset speed\n";
}

void DevControlSystem::onEvent(const Event& event, FrameContext& ctx) {
    if (event.type != EventType::KeyPressed || event.repeat) return;

    switch (event.key) {
        case KeyCode::Escape: ctx.quit = true; break;
        case KeyCode::Space:  paused = !paused; break;
        case KeyCode::Comma:  simSpeed = std::clamp(simSpeed * 0.5, 0.0625, 16.0); break;
        case KeyCode::Period: simSpeed = std::clamp(simSpeed * 2.0, 0.0625, 16.0); break;
        case KeyCode::Num0:   simSpeed = 1.0; paused = false; break;
        default: break;
    }
}

void DevControlSystem::update(FrameContext& ctx) {
    ctx.clock.setTimeScale(paused ? 0.0 : simSpeed);
}

void DevControlSystem::onStop(FrameContext& ctx) {
    ctx.settings.setDouble("timeScale", simSpeed);
}
