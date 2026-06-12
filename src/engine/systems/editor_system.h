#ifndef RAYTRACER_ENGINE_EDITOR_SYSTEM_H
#define RAYTRACER_ENGINE_EDITOR_SYSTEM_H

#include "../system.h"
#include "camera_system.h"
#include "../app_state.h"
#include "../editor_bridge.h"
#include <functional>
#include <string>

namespace engine {

// The edit-mode workhorse (docs/edit-mode-plan.md): click-to-select with
// bounding-sphere picking, an ImGuizmo move/rotate/scale gizmo, an inspector
// for the selected entity (transform, material, shape, physics spec), an Add
// menu (primitives / glTF models), and the Save / Play toolbar. Runs only in
// EditorState, where no simulation systems exist — the world it edits IS the
// level document, and Play saves it before swapping to the game state.
class EditorSystem : public System {
public:
    using PlayFactory = std::function<std::unique_ptr<AppState>()>;

    // `bridge`, when given, connects this system to a native shell (the Qt
    // editor): selection is shared and the shell's panels read the world
    // through it while the editor state is active.
    EditorSystem(CameraSystem& cameras, std::string levelFile,
                 PlayFactory makePlayState, EditorBridge* bridge = nullptr);

    void onStart(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

    Entity selectedEntity() const { return selected; }
    void setSelected(Entity entity) { selected = entity; }
    void setGizmoOp(int op) { gizmoOp = op; }
    int gizmoOpMode() const { return gizmoOp; }

private:
    void pickAtCursor(FrameContext& ctx);
    void frameSelected(FrameContext& ctx);
    Entity addPrimitive(FrameContext& ctx, const std::string& shape);
    Entity duplicateSelected(FrameContext& ctx);
    Vec3 spawnPoint(FrameContext& ctx) const;

    void drawToolbar(FrameContext& ctx);
    void drawInspector(FrameContext& ctx);
    void drawGizmo(FrameContext& ctx);
    void drawSelectionMarker(FrameContext& ctx) const;

    CameraSystem& cameras;
    std::string levelFile;
    PlayFactory makePlayState;
    EditorBridge* bridge = nullptr;

    Entity selected;
    bool prevMouseLeft = false;
    bool gizmoBusy = false;     // ImGuizmo hovered/dragging (blocks picking)
    int gizmoOp = 0;            // 0 translate, 1 rotate, 2 scale
    char modelPath[256] = "assets/models/";
};

}  // namespace engine

#endif
