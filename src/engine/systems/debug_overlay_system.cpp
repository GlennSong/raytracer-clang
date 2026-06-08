#include "debug_overlay_system.h"

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void DebugOverlaySystem::loadSettings(FrameContext& ctx) {
    auto& s = ctx.settings;
    auto& ao = ctx.renderer.ssaoParams;
    auto& ssr = ctx.renderer.ssrParams;
    auto& lit = ctx.view.lighting;

    ao.radius    = static_cast<float>(s.getDouble("ssao.radius", ao.radius));
    ao.intensity = static_cast<float>(s.getDouble("ssao.intensity", ao.intensity));
    ao.bias      = static_cast<float>(s.getDouble("ssao.bias", ao.bias));
    ao.directions = static_cast<int>(s.getDouble("ssao.directions", ao.directions));
    ao.steps     = static_cast<int>(s.getDouble("ssao.steps", ao.steps));

    ssr.maxRayDist    = static_cast<float>(s.getDouble("ssr.maxRayDist", ssr.maxRayDist));
    ssr.thickness     = static_cast<float>(s.getDouble("ssr.thickness", ssr.thickness));
    ssr.thicknessFar  = static_cast<float>(s.getDouble("ssr.thicknessFar", ssr.thicknessFar));
    ssr.stride        = static_cast<float>(s.getDouble("ssr.stride", ssr.stride));
    ssr.blendStrength = static_cast<float>(s.getDouble("ssr.blendStrength", ssr.blendStrength));

    lit.exposure          = static_cast<float>(s.getDouble("lighting.exposure", lit.exposure));
    lit.ambientMultiplier = static_cast<float>(s.getDouble("lighting.ambient", lit.ambientMultiplier));
    lit.sun.intensity     = static_cast<float>(s.getDouble("lighting.sunIntensity", lit.sun.intensity));

    auto& bloom = ctx.renderer.bloomParams;
    bloom.threshold = static_cast<float>(s.getDouble("bloom.threshold", bloom.threshold));
    bloom.knee      = static_cast<float>(s.getDouble("bloom.knee", bloom.knee));
    bloom.intensity = static_cast<float>(s.getDouble("bloom.intensity", bloom.intensity));

    ctx.renderer.showHud = s.getBool("hud.show", ctx.renderer.showHud);
    ctx.renderer.targetFps = static_cast<int>(s.getDouble("targetFps", ctx.renderer.targetFps));
}

void DebugOverlaySystem::saveSettings(FrameContext& ctx) {
    auto& s = ctx.settings;
    auto& ao = ctx.renderer.ssaoParams;
    auto& ssr = ctx.renderer.ssrParams;
    auto& lit = ctx.view.lighting;

    s.setDouble("ssao.radius", ao.radius);
    s.setDouble("ssao.intensity", ao.intensity);
    s.setDouble("ssao.bias", ao.bias);
    s.setDouble("ssao.directions", ao.directions);
    s.setDouble("ssao.steps", ao.steps);

    s.setDouble("ssr.maxRayDist", ssr.maxRayDist);
    s.setDouble("ssr.thickness", ssr.thickness);
    s.setDouble("ssr.thicknessFar", ssr.thicknessFar);
    s.setDouble("ssr.stride", ssr.stride);
    s.setDouble("ssr.blendStrength", ssr.blendStrength);

    s.setDouble("lighting.exposure", lit.exposure);
    s.setDouble("lighting.ambient", lit.ambientMultiplier);
    s.setDouble("lighting.sunIntensity", lit.sun.intensity);

    auto& bloom = ctx.renderer.bloomParams;
    s.setDouble("bloom.threshold", bloom.threshold);
    s.setDouble("bloom.knee", bloom.knee);
    s.setDouble("bloom.intensity", bloom.intensity);

    s.setBool("hud.show", ctx.renderer.showHud);
    s.setDouble("targetFps", ctx.renderer.targetFps);

    s.save("settings.json");
}

void DebugOverlaySystem::resetDefaults(FrameContext& ctx) {
    ctx.renderer.ssaoParams = Renderer::SSAOParams{};
    ctx.renderer.ssrParams = Renderer::SSRParams{};
    ctx.renderer.bloomParams = Renderer::BloomParams{};
    ctx.view.lighting.exposure = 1.0f;
    ctx.view.lighting.ambientMultiplier = 0.3f;
    ctx.view.lighting.sun.intensity = 1.5f;
    ctx.renderer.targetFps = 0;
}

void DebugOverlaySystem::onStart(FrameContext& ctx) {
    loadSettings(ctx);
}

void DebugOverlaySystem::onStop(FrameContext& ctx) {
    saveSettings(ctx);
}

void DebugOverlaySystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    ImGui::Begin("Debug");

    ImGui::Checkbox("Show HUD", &ctx.renderer.showHud);
    ImGui::Separator();

    double fps = ctx.frameDelta > 0.0 ? 1.0 / ctx.frameDelta : 0.0;
    ImGui::Text("FPS: %.1f (%.2f ms)", fps, ctx.frameDelta * 1000.0);
    ImGui::Text("Entities: %zu", ctx.world.entityCount());

    const CameraState& cam = ctx.view.camera;
    ImGui::Text("Camera pos: %.2f, %.2f, %.2f",
                cam.position.x, cam.position.y, cam.position.z);

    RenderStats rs = ctx.renderer.getRenderStats();
    uint32_t culled = static_cast<uint32_t>(ctx.world.entityCount()) - rs.entitiesSubmitted;
    ImGui::Text("Visible: %u  Culled: %u", rs.entitiesSubmitted, culled);
    ImGui::Text("Draw calls: %u (instanced: %u)", rs.drawCalls, rs.instancedDrawCalls);

    ImGui::Separator();
    const char* viewNames[] = {"Normal", "AO Only", "SSR Only", "Depth", "Normals"};
    ImGui::Combo("View", &ctx.renderer.debugView, viewNames, 5);

    if (ImGui::CollapsingHeader("Lighting")) {
        auto& lit = ctx.view.lighting;
        ImGui::SliderFloat("Sun Intensity", &lit.sun.intensity, 0.0f, 20.0f);
        ImGui::SliderFloat("Exposure", &lit.exposure, 0.01f, 5.0f);
        ImGui::SliderFloat("Ambient", &lit.ambientMultiplier, 0.0f, 1.0f);
    }

    if (ImGui::CollapsingHeader("SSAO")) {
        ImGui::Checkbox("Enabled##ssao", &ctx.renderer.ssaoEnabled);
        auto& ao = ctx.renderer.ssaoParams;
        ImGui::SliderFloat("Radius##ao", &ao.radius, 0.1f, 5.0f);
        ImGui::SliderFloat("Intensity##ao", &ao.intensity, 0.0f, 3.0f);
        ImGui::SliderFloat("Bias##ao", &ao.bias, 0.0f, 0.3f);
        ImGui::SliderInt("Directions", &ao.directions, 1, 8);
        ImGui::SliderInt("Steps", &ao.steps, 1, 8);
    }

    if (ImGui::CollapsingHeader("SSR")) {
        ImGui::Checkbox("Enabled##ssr", &ctx.renderer.ssrEnabled);
        auto& ssr = ctx.renderer.ssrParams;
        ImGui::SliderFloat("Max Ray Dist", &ssr.maxRayDist, 1.0f, 100.0f);
        ImGui::SliderFloat("Thickness Near", &ssr.thickness, 0.01f, 2.0f);
        ImGui::SliderFloat("Thickness Far", &ssr.thicknessFar, 0.1f, 10.0f);
        ImGui::SliderFloat("Stride (px)", &ssr.stride, 1.0f, 8.0f);
        ImGui::SliderFloat("Blend Strength", &ssr.blendStrength, 0.0f, 1.0f);
    }

    if (ImGui::CollapsingHeader("Bloom")) {
        ImGui::Checkbox("Enabled##bloom", &ctx.renderer.bloomEnabled);
        auto& bloom = ctx.renderer.bloomParams;
        ImGui::SliderFloat("Threshold", &bloom.threshold, 0.0f, 3.0f);
        ImGui::SliderFloat("Knee", &bloom.knee, 0.0f, 1.0f);
        ImGui::SliderFloat("Intensity##bloom", &bloom.intensity, 0.0f, 2.0f);
    }

    if (ImGui::CollapsingHeader("Other")) {
        ImGui::Checkbox("Reflection Probes", &ctx.renderer.reflectionProbesEnabled);
        const char* fpsOptions[] = {"Uncapped", "30", "60"};
        int fpsIdx = (ctx.renderer.targetFps == 30) ? 1 : (ctx.renderer.targetFps == 60) ? 2 : 0;
        if (ImGui::Combo("FPS Cap", &fpsIdx, fpsOptions, 3)) {
            ctx.renderer.targetFps = (fpsIdx == 1) ? 30 : (fpsIdx == 2) ? 60 : 0;
        }
    }

    ImGui::Separator();
    if (ImGui::Button("Save Settings")) {
        saveSettings(ctx);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load Settings")) {
        loadSettings(ctx);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Defaults")) {
        resetDefaults(ctx);
    }

    ImGui::End();
#else
    (void)ctx;
#endif
}

}  // namespace engine
