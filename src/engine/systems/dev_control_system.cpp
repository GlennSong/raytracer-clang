#include "dev_control_system.h"
#include "../../log.h"
#include "../screenshot.h"

#include <algorithm>
#include <cstdio>
#include <iostream>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

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
    // Sim SPEED lives in the debug menu (` -> Sim speed), not on the keyboard:
    // ',' and '.' were one stray press from half or double speed, and the car
    // picker shipped on the same keys and ran the world at 8x. Unbound unless
    // settings give a key (bind.sim_slower / bind.sim_faster).
    {"sim_slower", KeyCode::Unknown},
    {"sim_faster", KeyCode::Unknown},
    {"sim_reset", KeyCode::Num0},
    {"screenshot", KeyCode::F12},   // engine/screenshot.h
};

}  // namespace

void DevControlSystem::onStart(FrameContext& ctx) {
    // The sim speed is a SESSION knob. It used to persist as "timeScale", so
    // one stray ',' (sim_slower) made every later boot run at half speed —
    // "the player moves sluggish now and so does the car". Boot at 1.0.
    ctx.clock.setTimeScale(1.0);
    shownSpeed = 1.0;
    if (ctx.settings.getDouble("timeScale", 1.0) != 1.0)
        LOG_INFO << "Ignoring persisted timeScale=" << ctx.settings.getDouble("timeScale", 1.0)
                 << " — the sim speed no longer survives a restart";
    ctx.settings.setDouble("timeScale", 1.0);

    for (const ActionBinding& binding : DEV_ACTIONS) {
        std::string overrideKey = ctx.settings.getString(
            std::string("bind.") + binding.action, "");
        if (!overrideKey.empty() &&
            ctx.actions.bindButtonByName(binding.action, overrideKey)) {
            continue;  // settings supplied a valid key for this action
        }
        if (binding.defaultKey != KeyCode::Unknown)
            ctx.actions.bindButton(binding.action, binding.defaultKey);
    }

    std::cerr << "Controls:\n"
              << "  Left-drag=orbit, Right-drag=pan, Scroll=zoom\n"
              << "  WASD=move, QE=up/down, Shift=fast\n"
              << "  Up/Down=exposure, Esc=quit\n"
              << "  Enter=pause, 0=real-time speed (sim speed: ` debug menu)\n"
              << "  F12=screenshot (into " << screenshotFolder(ctx.settings) << ")\n"
              << "  P=toggle perspective/orthographic camera\n"
              << "  F=detach/attach freecam (mouse looks, WASD/QE fly)\n"
              << "  C=place camera here, V/B=cycle viewports, X=editor view\n"
              << "  On foot: Space=jump, Ctrl=crouch (hold), V first/third person,\n"
              << "           R respawn, T teleport, 1/2 gun away/out, 3 map, 4 flashlight\n"
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
    if (ctx.actions.pressed("screenshot")) {
        const std::string folder = screenshotFolder(ctx.settings);
        const std::string path = nextScreenshotPath(folder);
        if (path.empty())
            LOG_WARN << "Screenshot: cannot create the folder " << folder
                     << " (set " << kScreenshotFolderKey << " in settings.json)";
        else if (ctx.renderer.requestFrameDump(path))
            LOG_INFO << "Screenshot: " << path;
        else
            LOG_WARN << "Screenshot: this renderer cannot capture frames";
    }
    // The CLOCK holds the speed (the debug menu, the control channel's `sim speed`
    // and these keys all set it); this system only nudges it and shows it.
    double speed = ctx.clock.timeScale();
    if (ctx.actions.pressed("sim_slower")) speed = std::clamp(speed * 0.5, 0.0625, 16.0);
    if (ctx.actions.pressed("sim_faster")) speed = std::clamp(speed * 2.0, 0.0625, 16.0);
    if (ctx.actions.pressed("sim_reset")) {
        speed = 1.0;
        ctx.clock.setPaused(false);
    }
    ctx.clock.setTimeScale(speed);
    wallClock += ctx.frameDelta;   // frameDelta is real (window) time
    if (speed != shownSpeed || ctx.clock.paused() != shownPaused) {
        LOG_INFO << "Sim speed x" << speed << (ctx.clock.paused() ? " (paused)" : "");
        shownSpeed = speed;
        shownPaused = ctx.clock.paused();
        popupUntil = wallClock + 2.5;
    }
}

// THE SPEED BADGE (Glenn: "we need some onscreen UI popup for gamespeed slow/fast... so that I know what's
// going on"). Whenever the world is not running at real time: a big notice for a moment after it changes,
// then a small badge in the top corner for as long as it lasts.
void DevControlSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr) return;
    const bool paused = ctx.clock.paused();
    const double speed = ctx.clock.timeScale();
    const bool offNormal = paused || speed != 1.0;
    const bool popup = wallClock < popupUntil;
    if (!offNormal && !popup) return;
    char text[64];
    if (paused) std::snprintf(text, sizeof(text), "PAUSED");
    else if (speed == 1.0) std::snprintf(text, sizeof(text), "SIM SPEED 1x (real time)");
    else if (speed < 1.0) std::snprintf(text, sizeof(text), "SIM SLOW  1/%gx", 1.0 / speed);
    else std::snprintf(text, sizeof(text), "SIM FAST  %gx", speed);
    const ImVec4 colour = paused ? ImVec4(1.0f, 0.85f, 0.3f, 1.0f)
                        : speed < 1.0 ? ImVec4(0.45f, 0.75f, 1.0f, 1.0f)
                        : speed > 1.0 ? ImVec4(1.0f, 0.45f, 0.35f, 1.0f) : ImVec4(0.7f, 1.0f, 0.7f, 1.0f);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;
    if (popup) {   // the moment it changes: centre screen, large
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(ctx.windowWidth) * 0.5f, static_cast<float>(ctx.windowHeight) * 0.3f),
                                ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowBgAlpha(0.65f);
        ImGui::Begin("##simspeedpopup", nullptr, flags);
        ImGui::SetWindowFontScale(2.0f);
        ImGui::TextColored(colour, "%s", text);
        ImGui::SetWindowFontScale(1.0f);
        ImGui::TextDisabled("change it in the debug menu (`)  -  0 = real time");
        ImGui::End();
    } else {       // while it lasts: a small badge, top right
        ImGui::SetNextWindowPos(ImVec2(static_cast<float>(ctx.windowWidth) - 12.0f, 12.0f), ImGuiCond_Always,
                                ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.55f);
        ImGui::Begin("##simspeedbadge", nullptr, flags);
        ImGui::TextColored(colour, "%s", text);
        ImGui::End();
    }
#else
    (void)ctx;
#endif
}

void DevControlSystem::onStop(FrameContext&) {
    // Deliberately not persisted: see onStart.
}

}  // namespace engine

