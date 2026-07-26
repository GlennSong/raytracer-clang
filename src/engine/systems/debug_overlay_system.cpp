#include "debug_overlay_system.h"

#include <algorithm>

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
    ao.temporal  = static_cast<float>(s.getDouble("ssao.temporal", ao.temporal));

    ssr.maxRayDist    = static_cast<float>(s.getDouble("ssr.maxRayDist", ssr.maxRayDist));
    ssr.thickness     = static_cast<float>(s.getDouble("ssr.thickness", ssr.thickness));
    ssr.thicknessFar  = static_cast<float>(s.getDouble("ssr.thicknessFar", ssr.thicknessFar));
    ssr.stride        = static_cast<float>(s.getDouble("ssr.stride", ssr.stride));
    ssr.blendStrength = static_cast<float>(s.getDouble("ssr.blendStrength", ssr.blendStrength));
    ssr.maxRoughness  = static_cast<float>(s.getDouble("ssr.maxRoughness", ssr.maxRoughness));

    auto& sh = ctx.renderer.shadowParams;
    sh.distance     = static_cast<float>(s.getDouble("shadow.distance", sh.distance));
    sh.cascadeCount = static_cast<int>(s.getDouble("shadow.cascades", sh.cascadeCount));
    sh.splitLambda  = static_cast<float>(s.getDouble("shadow.splitLambda", sh.splitLambda));

    // NOTE: scene lighting (exposure, ambient, sun) is owned by the LEVEL file, not
    // settings.json — the cascade is code defaults -> level JSON -> runtime (sliders
    // / day-night). Persisting it here silently overrode the level on load, which
    // caused stale exposure/ambient to fight the level. Sliders still edit it live.
    (void)lit;

    // Bloom is LEVEL-AUTHORED now (environment.bloom): the same silent-override
    // rule as lighting above. Sliders still edit it live, nothing persists.

    ctx.renderer.tonemapOperator =
        static_cast<int>(s.getDouble("tonemap.op", ctx.renderer.tonemapOperator));
    ctx.renderer.gradeParams.contrast =
        static_cast<float>(s.getDouble("grade.contrast", ctx.renderer.gradeParams.contrast));
    ctx.renderer.gradeParams.saturation =
        static_cast<float>(s.getDouble("grade.saturation", ctx.renderer.gradeParams.saturation));

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
    s.setDouble("ssao.temporal", ao.temporal);

    s.setDouble("ssr.maxRayDist", ssr.maxRayDist);
    s.setDouble("ssr.thickness", ssr.thickness);
    s.setDouble("ssr.thicknessFar", ssr.thicknessFar);
    s.setDouble("ssr.stride", ssr.stride);
    s.setDouble("ssr.blendStrength", ssr.blendStrength);
    s.setDouble("ssr.maxRoughness", ssr.maxRoughness);

    s.setDouble("shadow.distance", ctx.renderer.shadowParams.distance);
    s.setDouble("shadow.cascades", ctx.renderer.shadowParams.cascadeCount);
    s.setDouble("shadow.splitLambda", ctx.renderer.shadowParams.splitLambda);

    // Scene lighting is level-owned (see loadSettings) — not persisted here, so a
    // session's slider tweaks don't silently override the level on next launch.
    (void)lit;


    s.setDouble("tonemap.op", ctx.renderer.tonemapOperator);
    s.setDouble("grade.contrast", ctx.renderer.gradeParams.contrast);
    s.setDouble("grade.saturation", ctx.renderer.gradeParams.saturation);

    s.setBool("hud.show", ctx.renderer.showHud);
    s.setDouble("targetFps", ctx.renderer.targetFps);

    s.save("settings.json");
}

void DebugOverlaySystem::resetDefaults(FrameContext& ctx) {
    ctx.renderer.ssaoParams = Renderer::SSAOParams{};
    ctx.renderer.ssrParams = Renderer::SSRParams{};
    ctx.renderer.shadowParams = Renderer::ShadowParams{};
    ctx.renderer.bloomParams = Renderer::BloomParams{};
    ctx.renderer.tonemapOperator = 0;
    ctx.renderer.gradeParams = Renderer::GradeParams{};
    ctx.view.lighting.exposure = 1.0f;
    ctx.view.lighting.ambientMultiplier = 0.3f;
    ctx.view.lighting.sun.intensity = 4.7f;  // illuminance units (ADR-0017 Phase 1)
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
    // No ImGui context (e.g. a backend without debug-UI support): stay inert.
    if (ImGui::GetCurrentContext() == nullptr) return;
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
    // entitiesSubmitted counts submitted draws (instances included), which can
    // exceed the entity count — clamp so the difference never underflows to
    // "4.2 billion culled".
    const uint32_t total = static_cast<uint32_t>(ctx.world.entityCount());
    uint32_t culled = rs.entitiesSubmitted <= total ? total - rs.entitiesSubmitted : 0;
    ImGui::Text("Visible: %u  Culled: %u", rs.entitiesSubmitted, culled);
    ImGui::Text("Draw calls: %u (instanced: %u)", rs.drawCalls, rs.instancedDrawCalls);
    ImGui::Text("Instances: %u  Triangles: %.2fM", rs.totalInstances,
                rs.trianglesDrawn / 1e6);
    if (rs.instanceOverflow || rs.shadowOverflow || rs.foliageOverflow)
        ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.25f, 1.0f),
                           "OVERFLOW inst %u  shadow %u  foliage %u",
                           rs.instanceOverflow, rs.shadowOverflow,
                           rs.foliageOverflow);

    ImGui::Separator();
    const char* viewNames[] = {"Normal", "AO Only", "SSR Only", "Depth", "Normals",
                               "Shadow", "Albedo", "Facing", "Cascades"};
    ImGui::Combo("View", &ctx.renderer.debugView, viewNames, 9);
    const char* wireNames[] = {"Off", "Wireframe", "Wire overlay"};
    ImGui::Combo("Wireframe", &ctx.renderer.wireframe, wireNames, 3);
    if (ctx.renderer.wireframe != 0) {
        float wc[3] = {static_cast<float>(ctx.renderer.wireframeColor.x),
                       static_cast<float>(ctx.renderer.wireframeColor.y),
                       static_cast<float>(ctx.renderer.wireframeColor.z)};
        if (ImGui::ColorEdit3("Wire color", wc))
            ctx.renderer.wireframeColor = Vec3(wc[0], wc[1], wc[2]);
    }

    if (ImGui::CollapsingHeader("Lighting")) {
        auto& lit = ctx.view.lighting;
        ImGui::SliderFloat("Sun Intensity", &lit.sun.intensity, 0.0f, 20.0f);
        ImGui::SliderFloat("Exposure", &lit.exposure, 0.01f, 5.0f);
        // Auto-exposure: map the HDR's log-average luminance to middle grey (0.18).
        // Only meaningful when an HDR environment is bound.
        float avgLum = ctx.renderer.environmentAvgLuminance;
        ImGui::BeginDisabled(avgLum <= 0.0f);
        if (ImGui::Button("Auto Exposure") && avgLum > 0.0f)
            lit.exposure = std::clamp(0.18f / avgLum, 0.01f, 5.0f);
        ImGui::EndDisabled();
        if (avgLum > 0.0f) {
            ImGui::SameLine();
            ImGui::TextDisabled("(avg lum %.3f)", avgLum);
        }
        ImGui::SliderFloat("Ambient", &lit.ambientMultiplier, 0.0f, 1.0f);
        float ambTint[3] = {static_cast<float>(lit.ambientTint.x),
                            static_cast<float>(lit.ambientTint.y),
                            static_cast<float>(lit.ambientTint.z)};
        if (ImGui::ColorEdit3("Ambient Tint", ambTint))
            lit.ambientTint = Vec3(ambTint[0], ambTint[1], ambTint[2]);

        // Artistic shadow response (ADR-0017 Phase 2)
        ImGui::SeparatorText("Shadows");
        auto& sa = lit.shadowArtistic;
        ImGui::SliderFloat("Strength##shadow", &sa.strength, 0.0f, 1.0f);
        ImGui::SliderFloat("Ambient Occl.##shadow", &sa.ambientStrength, 0.0f, 1.0f);
        float tint[3] = {static_cast<float>(sa.tint.x),
                         static_cast<float>(sa.tint.y),
                         static_cast<float>(sa.tint.z)};
        if (ImGui::ColorEdit3("Tint##shadow", tint))
            sa.tint = Vec3(tint[0], tint[1], tint[2]);

        // Aerial-perspective fog (level-owned, like the rest of lighting). Fades
        // distant geometry toward the fog color by 1-exp(-density*dist) — a depth
        // cue that also hides the terrain/LOD cull edge. Tune live, then bake the
        // value you like into the level JSON (not persisted to settings.json).
        ImGui::SeparatorText("Fog");
        auto& fog = lit.fog;
        ImGui::Checkbox("Enabled##fog", &fog.enabled);
        ImGui::SliderFloat("Density##fog", &fog.density, 0.0f, 0.02f, "%.5f");
        float fogCol[3] = {static_cast<float>(fog.color.x),
                           static_cast<float>(fog.color.y),
                           static_cast<float>(fog.color.z)};
        if (ImGui::ColorEdit3("Color##fog", fogCol))
            fog.color = Vec3(fogCol[0], fogCol[1], fogCol[2]);
        if (ImGui::Button("Match Sky##fog"))
            fog.color = lit.sky.horizonColor;

        // Vegetation draw distance — pull trees in to where the fog hides them so
        // we stop paying to render the far field. 0 = use the level's value.
        ImGui::SliderFloat("Veg Draw Dist", &ctx.renderer.vegetationDrawDistance,
                           0.0f, 1500.0f, "%.0f m");
        ImGui::SameLine();
        ImGui::TextDisabled("(0=level)");
    }

    if (ImGui::CollapsingHeader("SSAO")) {
        ImGui::Checkbox("Enabled##ssao", &ctx.renderer.ssaoEnabled);
        auto& ao = ctx.renderer.ssaoParams;
        ImGui::SliderFloat("Radius##ao", &ao.radius, 0.1f, 5.0f);
        ImGui::SliderFloat("Intensity##ao", &ao.intensity, 0.0f, 3.0f);
        ImGui::SliderFloat("Bias##ao", &ao.bias, 0.0f, 0.3f);
        ImGui::SliderFloat("Floor##ao", &ao.aoFloor, 0.0f, 1.0f);
        ImGui::SliderInt("Directions", &ao.directions, 1, 12);
        ImGui::SliderInt("Steps", &ao.steps, 1, 16);   // higher = smoother, costlier
        ImGui::SliderFloat("Temporal", &ao.temporal, 0.0f, 0.97f);  // 0 = off; higher = steadier but ghostier
    }

    if (ImGui::CollapsingHeader("SSR")) {
        ImGui::Checkbox("Enabled##ssr", &ctx.renderer.ssrEnabled);
        auto& ssr = ctx.renderer.ssrParams;
        ImGui::SliderFloat("Max Ray Dist", &ssr.maxRayDist, 1.0f, 100.0f);
        ImGui::SliderFloat("Thickness Near", &ssr.thickness, 0.01f, 2.0f);
        ImGui::SliderFloat("Thickness Far", &ssr.thicknessFar, 0.1f, 10.0f);
        ImGui::SliderFloat("Stride (px)", &ssr.stride, 1.0f, 8.0f);
        ImGui::SliderFloat("Blend Strength", &ssr.blendStrength, 0.0f, 1.0f);
        ImGui::SliderFloat("Max Roughness", &ssr.maxRoughness, 0.0f, 1.0f);  // rougher = no SSR
    }

    if (ImGui::CollapsingHeader("Shadows")) {
        auto& sh = ctx.renderer.shadowParams;
        ImGui::SliderFloat("Distance", &sh.distance, 20.0f, 500.0f);  // sun-shadow range (m)
        ImGui::SliderInt("Cascades", &sh.cascadeCount, 1, 4);   // RT_MAX_CASCADES
        ImGui::SliderFloat("Split lambda", &sh.splitLambda, 0.0f, 1.0f);  // 0=uniform, 1=log
    }

    if (ImGui::CollapsingHeader("Bloom")) {
        ImGui::Checkbox("Enabled##bloom", &ctx.renderer.bloomEnabled);
        auto& bloom = ctx.renderer.bloomParams;
        ImGui::SliderFloat("Threshold", &bloom.threshold, 0.0f, 3.0f);
        ImGui::SliderFloat("Knee", &bloom.knee, 0.0f, 1.0f);
        ImGui::SliderFloat("Intensity##bloom", &bloom.intensity, 0.0f, 2.0f);
    }

    if (ImGui::CollapsingHeader("Tonemap / Grade")) {
        // View transform (the film curve). ACES = punchy; AgX = gentler highlights,
        // less hue skew (Blender 4's default).
        const char* tmNames[] = {"ACES", "AgX"};
        ImGui::Combo("View Transform", &ctx.renderer.tonemapOperator, tmNames, 2);
        // The "Look": graded in scene-linear before the tone map (HDR-safe). 1.0 is
        // neutral — raise contrast to de-wash, saturation to re-punch color.
        auto& grade = ctx.renderer.gradeParams;
        ImGui::SliderFloat("Contrast", &grade.contrast, 0.5f, 2.0f);
        ImGui::SliderFloat("Saturation", &grade.saturation, 0.0f, 2.0f);
    }

    if (ImGui::CollapsingHeader("Other")) {
        ImGui::Checkbox("Reflection Probes", &ctx.renderer.reflectionProbesEnabled);
        ImGui::Checkbox("HDR Environment", &ctx.renderer.environmentMapEnabled);
        ImGui::Checkbox("Foliage Depth Prepass", &ctx.renderer.depthPrepassEnabled);
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
