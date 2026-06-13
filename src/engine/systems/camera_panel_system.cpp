#include "camera_panel_system.h"

#include "../components.h"
#include "../camera_store.h"
#include "../imgui_properties.h"
#include "../../log.h"
#include <cstdio>
#include <cstdlib>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

#ifdef RT_ENABLE_IMGUI

void CameraPanelSystem::drawFramingOverlay() const {
    // Background draw list: over the scene, under every ImGui window.
    ImDrawList* draw = ImGui::GetBackgroundDrawList();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    float x = vp->Pos.x, y = vp->Pos.y, w = vp->Size.x, h = vp->Size.y;

    if (showThirds) {
        const ImU32 line = IM_COL32(255, 255, 255, 70);
        for (int i = 1; i <= 2; i++) {
            draw->AddLine(ImVec2(x + w * i / 3.0f, y),
                          ImVec2(x + w * i / 3.0f, y + h), line);
            draw->AddLine(ImVec2(x, y + h * i / 3.0f),
                          ImVec2(x + w, y + h * i / 3.0f), line);
        }
    }

    if (letterbox != 0 && w > 0 && h > 0) {
        const float aspects[] = {0.0f, 2.39f, 1.85f, 16.0f / 9.0f};
        float target = aspects[letterbox];
        const ImU32 bar = IM_COL32(0, 0, 0, 220);
        float content = w / h;
        if (content < target) {
            // Window is taller than the frame: horizontal bars.
            float barH = (h - w / target) / 2.0f;
            draw->AddRectFilled(ImVec2(x, y), ImVec2(x + w, y + barH), bar);
            draw->AddRectFilled(ImVec2(x, y + h - barH), ImVec2(x + w, y + h), bar);
        } else if (content > target) {
            // Window is wider than the frame: pillarbox.
            float barW = (w - h * target) / 2.0f;
            draw->AddRectFilled(ImVec2(x, y), ImVec2(x + barW, y + h), bar);
            draw->AddRectFilled(ImVec2(x + w - barW, y), ImVec2(x + w, y + h), bar);
        }
    }
}

void CameraPanelSystem::render(FrameContext& ctx) {
    // No ImGui context (e.g. a backend without debug-UI support): stay inert.
    if (ImGui::GetCurrentContext() == nullptr) return;
    drawFramingOverlay();   // an opted-into creative overlay, not debug UI

    // Debug-mode only (backtick): plain play looks like the shipped game.
    if (!ctx.debugOverlayActive) return;

    // Appends into the same window as DebugOverlaySystem ("Debug"), so the
    // debug UI is one window with sections instead of a window per system.
    ImGui::Begin("Debug");
    if (!ImGui::CollapsingHeader("Cameras", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::End();
        return;
    }

    Entity viewing = cameras.activeSceneCamera();

    ImGui::BeginDisabled(viewing.valid());
    if (ImGui::Button("Place Camera Here")) {
        selected = cameras.placeCameraAtView(ctx);
        if (preview) cameras.setActiveSceneCamera(selected);
    }
    ImGui::EndDisabled();
    if (viewing.valid()) {
        ImGui::SameLine();
        if (ImGui::Button("Back To Editor"))
            cameras.setActiveSceneCamera(Entity{});
    }
    // Live preview: selecting a camera looks through it, so the transform and
    // lens controls below adjust the picture you are watching.
    ImGui::SameLine();
    if (ImGui::Checkbox("Preview", &preview)) {
        if (preview && ctx.world.alive(selected))
            cameras.setActiveSceneCamera(selected);
        else if (!preview && viewing == selected)
            cameras.setActiveSceneCamera(Entity{});
    }

    std::vector<Entity> list = collectCameras(ctx.world);
    if (!ctx.world.alive(selected)) selected = Entity{};

    for (Entity e : list) {
        SceneCamera* cam = ctx.world.get<SceneCamera>(e);
        if (!cam) continue;
        ImGui::PushID(static_cast<int>(e.index));
        char label[96];
        std::snprintf(label, sizeof(label), "%s%s", cam->name.c_str(),
                      (e == viewing) ? "  [viewing]" : "");
        if (ImGui::Selectable(label, e == selected)) {
            selected = e;
            if (preview) cameras.setActiveSceneCamera(e);
        }
        ImGui::PopID();
    }

    SceneCamera* cam = ctx.world.get<SceneCamera>(selected);
    Transform* t = ctx.world.get<Transform>(selected);
    if (cam && t) {
        ImGui::Separator();

        if (selected == viewing) {
            if (ImGui::Button("Stop Looking Through"))
                cameras.setActiveSceneCamera(Entity{});
        } else {
            if (ImGui::Button("Look Through"))
                cameras.setActiveSceneCamera(selected);
        }
        ImGui::SameLine();
        ImGui::BeginDisabled(viewing.valid());
        if (ImGui::Button("Snap To My View")) {
            // The published view is the editor/first-person view when no
            // placed camera is active (guaranteed by the disable above).
            const CameraState& view = ctx.view.camera;
            t->position = view.position;
            t->orientation = orientationFromForward(view.target - view.position);
        }
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Delete")) {
            if (selected == viewing) cameras.setActiveSceneCamera(Entity{});
            ctx.world.destroy(selected);
            selected = Entity{};
        }

        if (ctx.world.alive(selected)) {
            float pos[3] = {static_cast<float>(t->position.x),
                            static_cast<float>(t->position.y),
                            static_cast<float>(t->position.z)};
            if (ImGui::DragFloat3("Position", pos, 0.05f))
                t->position = Vec3(pos[0], pos[1], pos[2]);

            Real yaw, pitch;
            yawPitchFromForward(t->orientation.rotate(Vec3(0, 0, -1)), yaw, pitch);
            float angles[2] = {static_cast<float>(yaw), static_cast<float>(pitch)};
            bool turned = ImGui::SliderFloat("Yaw", &angles[0], -180.0f, 180.0f);
            turned |= ImGui::SliderFloat("Pitch", &angles[1], -89.0f, 89.0f);
            if (turned)
                t->orientation = orientationFromYawPitch(angles[0], angles[1]);

            static float lookAt[3] = {0, 0, 0};
            ImGui::DragFloat3("##lookat", lookAt, 0.05f);
            ImGui::SameLine();
            if (ImGui::Button("Look At"))
                t->orientation = orientationFromForward(
                    Vec3(lookAt[0], lookAt[1], lookAt[2]) - t->position);

            // Cameras are stationary: keep the interpolation pair in sync so
            // inspector nudges don't smear the gizmo across frames.
            if (auto* prev = ctx.world.get<PrevTransform>(selected))
                prev->value = *t;

            ImGui::SeparatorText("Camera");
            auto& lens = cam->lens;
            const float presets[] = {24, 35, 50, 85};
            for (float p : presets) {
                char b[8];
                std::snprintf(b, sizeof(b), "%.0fmm", p);
                if (ImGui::Button(b)) lens.focalLength = p;
                ImGui::SameLine();
            }
            ImGui::NewLine();

            // Name + every lens field, generated from describeProperties —
            // ranges, log scaling, and units all come from the FieldMeta.
            ImGuiPropertyVisitor lensUi;
            describeProperties(*cam, lensUi);
            ImGui::Text("FOV %.1f deg, aperture %.2f mm",
                        lens.verticalFovDegrees(),
                        lens.apertureDiameter() * 1000.0);

            ImGui::SeparatorText("Offline Render");
            if (ImGui::Button("Render In Path Tracer")) {
                std::string level = ctx.settings.getString("levelPath", "");
                std::string store = ctx.settings.getString("cameraStorePath", "");
                if (level.empty() || store.empty()) {
                    LOG_WARN << "No level path known — offline render unavailable";
                } else {
                    // Save first so the tracer renders the camera as edited,
                    // then launch detached; the shell returns immediately.
                    CameraStore::save(store, ctx.world);
                    std::string out = cam->name + ".png";
                    std::string cmd = "./raytracer --level \"" + level +
                                      "\" --camera \"" + cam->name +
                                      "\" --out \"" + out + "\"";
                    LOG_INFO << "Launching: " << cmd;
                    // Run detached; on success open the result in the OS viewer
                    // (the whole subshell is backgrounded, so the editor never
                    // blocks). `open` is the macOS opener — the editor's home.
                    int rc = std::system(
                        ("(" + cmd + " > offline_render.log 2>&1 && open \"" +
                         out + "\") &").c_str());
                    if (rc != 0)
                        LOG_WARN << "Could not launch the offline tracer "
                                    "(is ./raytracer built? try `make release`)";
                    else
                        LOG_INFO << "Offline render running in the background "
                                 << "-> " << out
                                 << " (progress: offline_render.log)";
                }
            }
        }
    }

    ImGui::SeparatorText("Viewer");
    ImGui::Checkbox("Lens Effects", &ctx.renderer.lensEffectsEnabled);
    ImGui::Checkbox("Depth of Field", &ctx.renderer.dofEnabled);
    ImGui::Checkbox("Rule of Thirds", &showThirds);
    const char* boxes[] = {"No letterbox", "2.39:1", "1.85:1", "16:9"};
    ImGui::Combo("Letterbox", &letterbox, boxes, 4);

    ImGui::End();
}

#else  // !RT_ENABLE_IMGUI

void CameraPanelSystem::drawFramingOverlay() const {}
void CameraPanelSystem::render(FrameContext& ctx) {
    // Reference the members so ImGui-less builds stay warning-clean.
    (void)ctx;
    (void)cameras;
    (void)selected;
    (void)preview;
    (void)showThirds;
    (void)letterbox;
}

#endif

}  // namespace engine
