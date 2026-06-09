#include "day_night_system.h"

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void DayNightSystem::onStart(FrameContext& ctx) {
    auto& s = ctx.settings;
    enabled       = s.getBool("daynight.enabled", enabled);
    cycle.timeOfDay = s.getDouble("daynight.timeOfDay", cycle.timeOfDay);
    cycle.speed     = s.getDouble("daynight.speed", cycle.speed);
    cycle.paused    = s.getBool("daynight.paused", cycle.paused);
    if (enabled) apply(ctx);
}

void DayNightSystem::onStop(FrameContext& ctx) {
    auto& s = ctx.settings;
    s.setBool("daynight.enabled", enabled);
    s.setDouble("daynight.timeOfDay", cycle.timeOfDay);
    s.setDouble("daynight.speed", cycle.speed);
    s.setBool("daynight.paused", cycle.paused);
    s.save("settings.json");
}

void DayNightSystem::update(FrameContext& ctx) {
    if (!enabled) return;
    cycle.advance(ctx.frameDelta);
    apply(ctx);
}

// Push the current cycle state into both the procedural sky and the directional
// sun so the rendered sky and the lighting agree.
void DayNightSystem::apply(FrameContext& ctx) {
    DayNightState st = cycle.evaluate();
    auto& lit = ctx.view.lighting;

    lit.sun.direction = st.sunDirection;
    lit.sun.color     = st.sunColor;
    lit.sun.intensity = st.sunIntensity;

    lit.sky.sunDirection   = st.sunDirection;
    lit.sky.sunColor       = st.sunColor;
    lit.sky.sunDiscIntensity = st.skyDiscIntensity;
    lit.sky.zenithColor    = st.zenithColor;
    lit.sky.horizonColor   = st.horizonColor;
    lit.sky.groundColor    = st.groundColor;

    lit.ambientMultiplier = st.ambient;
}

void DayNightSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (ImGui::CollapsingHeader("Day / Night")) {
        ImGui::Checkbox("Enabled##daynight", &enabled);
        float t = static_cast<float>(cycle.timeOfDay);
        if (ImGui::SliderFloat("Time of Day", &t, 0.0f, 1.0f, "%.3f")) {
            cycle.timeOfDay = t;
            if (enabled) apply(ctx);
        }
        ImGui::SameLine();
        ImGui::Checkbox("Pause", &cycle.paused);
        float speed = static_cast<float>(cycle.speed);
        if (ImGui::SliderFloat("Speed (days/sec)", &speed, 0.0f, 0.2f, "%.3f")) {
            cycle.speed = speed;
        }
        // Readout: 24h clock derived from time-of-day (0.0 == midnight).
        int totalMin = static_cast<int>(cycle.timeOfDay * 24.0 * 60.0) % (24 * 60);
        ImGui::Text("Clock: %02d:%02d", totalMin / 60, totalMin % 60);
    }
#else
    (void)ctx;
#endif
}

}  // namespace engine
