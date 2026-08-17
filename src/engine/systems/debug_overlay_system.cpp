#include "debug_overlay_system.h"

#include "../frame_stats.h"

#include <algorithm>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void DebugOverlaySystem::loadSettings(Settings& s, Renderer& r) {
    auto& ao = r.ssaoParams;
    auto& ssr = r.ssrParams;

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

    auto& sh = r.shadowParams;
    sh.distance     = static_cast<float>(s.getDouble("shadow.distance", sh.distance));
    sh.cascadeCount = static_cast<int>(s.getDouble("shadow.cascades", sh.cascadeCount));
    sh.splitLambda  = static_cast<float>(s.getDouble("shadow.splitLambda", sh.splitLambda));

    // NOTE: scene lighting (exposure, ambient, sun) is owned by the LEVEL file, not
    // settings.json — the cascade is code defaults -> level JSON -> runtime (sliders
    // / day-night). Persisting it here silently overrode the level on load, which
    // caused stale exposure/ambient to fight the level. Sliders still edit it live.

    // Bloom is LEVEL-AUTHORED now (environment.bloom): the same silent-override
    // rule as lighting above. Sliders still edit it live, nothing persists.
    // (The visionOS render panel is unaffected — it keeps its own on-device pref
    // store in vision_spike.mm, seeded from the live renderer params, and never
    // touches settings.json.)

    // ...including the ON/OFF bit. This one was left behind when the bloom
    // PARAMS stopped persisting, and loadSettings runs AFTER the level load in
    // both states, so a single stale `bloom.enabled: false` in settings.json
    // silently un-bloomed every level ever after (measured on metropolis_sky:
    // mean frame luma 208 -> 115). The FX checkbox still toggles it live; it
    // just no longer outlives the session.
    r.ssaoEnabled  = s.getBool("ssao.enabled", r.ssaoEnabled);
    r.ssrEnabled   = s.getBool("ssr.enabled", r.ssrEnabled);

    r.tonemapOperator =
        static_cast<int>(s.getDouble("tonemap.op", r.tonemapOperator));
    r.gradeParams.contrast =
        static_cast<float>(s.getDouble("grade.contrast", r.gradeParams.contrast));
    r.gradeParams.saturation =
        static_cast<float>(s.getDouble("grade.saturation", r.gradeParams.saturation));

    r.showHud = s.getBool("hud.show", r.showHud);
    r.targetFps = static_cast<int>(s.getDouble("targetFps", r.targetFps));
}

void DebugOverlaySystem::loadSettings(FrameContext& ctx) {
    // Scene lighting (exposure, ambient, sun) is owned by the LEVEL file, not
    // settings.json — the cascade is code defaults -> level JSON -> runtime
    // (sliders / day-night). Persisting it here silently overrode the level on
    // load, which caused stale exposure/ambient to fight the level. Sliders
    // still edit it live.
    loadSettings(ctx.settings, ctx.renderer);
}

void DebugOverlaySystem::saveSettings(Settings& s, Renderer& r) {
    auto& ao = r.ssaoParams;
    auto& ssr = r.ssrParams;

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

    s.setDouble("shadow.distance", r.shadowParams.distance);
    s.setDouble("shadow.cascades", r.shadowParams.cascadeCount);
    s.setDouble("shadow.splitLambda", r.shadowParams.splitLambda);

    // Scene lighting is level-owned (see loadSettings) — not persisted here, so a
    // session's slider tweaks don't silently override the level on next launch.
    // Bloom rides the same rule.


    s.setBool("ssao.enabled", r.ssaoEnabled);
    s.setBool("ssr.enabled", r.ssrEnabled);

    s.setDouble("tonemap.op", r.tonemapOperator);
    s.setDouble("grade.contrast", r.gradeParams.contrast);
    s.setDouble("grade.saturation", r.gradeParams.saturation);

    s.setBool("hud.show", r.showHud);
    s.setDouble("targetFps", r.targetFps);
}

void DebugOverlaySystem::saveSettings(FrameContext& ctx) {
    // Scene lighting is level-owned (see loadSettings) — not persisted here, so a
    // session's slider tweaks don't silently override the level on next launch.
    saveSettings(ctx.settings, ctx.renderer);
    ctx.settings.save("settings.json");
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
    // Put the passes back BEFORE persisting: quitting mid-run must not
    // save a measurement configuration as if it were the user's choice.
    passCost.cancel(ctx);
    saveSettings(ctx);
}

void DebugOverlaySystem::render(FrameContext& ctx) {
    // Advanced FIRST and unconditionally: a running measurement has passes
    // disabled to time them, so it must keep ticking to its end and restore
    // them even if the panel is collapsed, the window hidden, or ImGui absent
    // entirely. Driving it from inside the panel's if-block once left a pass
    // switched off for the rest of the session.
    passCost.update(ctx);

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

    // The frame ledger (ADR-0077): where the frame's CPU time goes, over the
    // last ~4 s. Wait is FPS-cap sleep — headroom, not cost. For anything the
    // per-phase split can't answer, attach Tracy (RT_ENABLE_PROFILER).
    if (ImGui::CollapsingHeader("Performance")) {
        FrameStats& fs = ctx.stats;
        FrameStats::Summary sum = fs.summarize();
        ImGui::Text("frame  %5.2f ms avg   %5.2f p95   %5.2f max",
                    sum.avgTotalMs, sum.p95TotalMs, sum.maxTotalMs);
        ImGui::Text("update %5.2f   fixed %5.2f   render %5.2f   wait %5.2f",
                    sum.avgUpdateMs, sum.avgFixedMs, sum.avgRenderMs,
                    sum.avgWaitMs);
        // The render split is the actionable part: acquire is time BLOCKED on
        // the GPU/vsync (CPU idle), encode+submit is real CPU work.
        ImGui::Text("  acquire %5.2f (gpu wait)   encode %5.2f   submit %5.2f",
                    sum.avgAcquireMs, sum.avgEncodeMs, sum.avgSubmitMs);
        if (sum.avgGpuMs > 0.0f) {
            ImGui::Text("  GPU     %5.2f ms/frame", sum.avgGpuMs);
            ImGui::SameLine();
            ImGui::TextDisabled(sum.avgGpuMs > sum.avgEncodeMs + sum.avgSubmitMs
                                    ? "(GPU-bound)" : "(CPU-bound)");
        } else {
            ImGui::TextDisabled("  GPU     n/a (backend reports no timing)");
        }

        static float plotBuf[FrameStats::HISTORY];
        const int n = fs.historySize();
        for (int i = 0; i < n; i++) plotBuf[i] = fs.historyAt(i).totalMs;
        // Fixed 0..2x-budget scale so spikes read against 16.6/33.3 ms rather
        // than autoscale flattening everything.
        float budgetMs = ctx.renderer.targetFps > 0
                             ? 1000.0f / static_cast<float>(ctx.renderer.targetFps)
                             : 16.6f;
        ImGui::PlotLines("##frameTimes", plotBuf, n, 0, nullptr, 0.0f,
                         budgetMs * 2.0f, ImVec2(-1, 64));
        ImGui::TextDisabled("last %d frames, scale 0-%.1f ms", n,
                            budgetMs * 2.0f);

        // Pass-cost probe: the only reliable way to rank the screen-space
        // passes while gpu_ms is present-contaminated (ADR-0077). Holds each
        // configuration for a fixed window, then reports the median frame time
        // of each — measured, not eyeballed, and no toggling by hand.
        if (passCost.state == PassCost::Idle) {
            if (ImGui::Button("Rank post passes (~10s)")) passCost.begin(ctx);
            ImGui::SameLine();
            ImGui::TextDisabled("toggles SSAO/SSR/bloom in turn, times each");
        } else {
            ImGui::Text("ranking: %s  (%.1fs left)", passCost.label(),
                        passCost.secondsLeft());
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) passCost.cancel(ctx);
        }
        if (!passCost.result.empty()) {
            ImGui::TextUnformatted(passCost.result.c_str());
            ImGui::SameLine();
            if (ImGui::Button("Clear##passcost")) passCost.result.clear();
        }

        if (!fs.capturing()) {
            if (ImGui::Button("Start CSV capture"))
                fs.startCapture("frame-capture.csv",
                                describeCaptureContext(ctx));
            ImGui::SameLine();
            ImGui::TextDisabled("-> frame-capture.csv (tools/frame-report.py)");
        } else {
            if (ImGui::Button("Stop CSV capture")) fs.stopCapture();
            ImGui::SameLine();
            ImGui::Text("recording: %ld frames", fs.capturedFrames());
        }
    }

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

    // Every live binding, key-centric ("what does Space do?"), because keys
    // quietly accumulating meanings across systems is exactly how Space ended
    // up as both the brake pedal and pause. A key feeding more than one action
    // is painted yellow — visible the moment a new binding collides.
    if (ImGui::CollapsingHeader("Controls")) {
        const std::vector<InputMap::BindingDesc> rows = ctx.actions.listBindings();
        if (ImGui::BeginTable("controls", 2,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Input");
            ImGui::TableSetupColumn("Action(s)");
            ImGui::TableHeadersRow();
            // listBindings sorts by input, so one table row per input is a
            // single pass: gather the run, join the action names.
            for (size_t i = 0; i < rows.size();) {
                size_t j = i;
                std::string actions;
                while (j < rows.size() && rows[j].input == rows[i].input) {
                    if (!actions.empty()) actions += ",  ";
                    actions += rows[j].action;
                    if (rows[j].isAxis)
                        actions += rows[j].scale >= 0 ? "(+)" : "(-)";
                    ++j;
                }
                // Same key driving several AXIS scales (W = cam_forward AND
                // drive_throttle) is a design choice; several BUTTON actions on
                // one key is the collision worth flagging.
                int buttonActions = 0;
                for (size_t k = i; k < j; ++k)
                    if (!rows[k].isAxis) ++buttonActions;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                if (buttonActions > 1)
                    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.25f, 1.0f), "%s",
                                       rows[i].input.c_str());
                else
                    ImGui::TextUnformatted(rows[i].input.c_str());
                ImGui::TableSetColumnIndex(1);
                if (buttonActions > 1)
                    ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.25f, 1.0f), "%s",
                                       actions.c_str());
                else
                    ImGui::TextUnformatted(actions.c_str());
                i = j;
            }
            ImGui::EndTable();
        }
        ImGui::TextDisabled("yellow = one key, several button actions");
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
        ImGui::SliderFloat("Intensity##bloom", &bloom.intensity, 0.0f, 0.5f);
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
