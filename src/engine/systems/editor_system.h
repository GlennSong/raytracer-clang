#ifndef RAYTRACER_ENGINE_EDITOR_SYSTEM_H
#define RAYTRACER_ENGINE_EDITOR_SYSTEM_H

#include "../system.h"
#include "camera_system.h"
#include "../app_state.h"
#include "../editor_bridge.h"
#include "../undo_stack.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

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

    // The edit session's command log (created on first use, lives until the
    // state ends). The Qt shell reaches it through the bridge. Inline, like
    // every member the bridge calls: pulling editor_system.o into renderer-
    // less hosts (run_tests) would drag the whole state stack with it.
    UndoStack* undoStack() {
        if (!undo) undo = std::make_unique<UndoStack>(componentRegistry());
        return undo.get();
    }
    // --- shell requests (via EditorBridge) ---------------------------------
    // Entity creation needs the frame context (renderer, spawn point from the
    // live view), which the shell doesn't hold; requests queue here and the
    // next update() applies them — selected + recorded on the command log.
    void requestAddPrimitive(const std::string& shape) {
        pendingAdds.push_back(shape);
    }
    void requestPlaceCamera() { pendingAdds.push_back("camera"); }
    void requestDuplicate() { pendingDuplicate = true; }

    // The component vocabulary the inspector renders from: the bridge's
    // registry when a shell is connected, a private one otherwise.
    ComponentRegistry& componentRegistry() {
        if (bridge) return bridge->registry();
        if (!fallbackRegistry) {
            fallbackRegistry = std::make_unique<ComponentRegistry>();
            registerEngineComponents(*fallbackRegistry);
        }
        return *fallbackRegistry;
    }

private:
    void pickAtCursor(FrameContext& ctx);
    void frameSelected(FrameContext& ctx);
    Entity addPrimitive(FrameContext& ctx, const std::string& shape);
    Entity duplicateSelected(FrameContext& ctx);
    Vec3 spawnPoint(FrameContext& ctx) const;
    void processShellRequests(FrameContext& ctx);

    void drawToolbar(FrameContext& ctx);
    void drawInspector(FrameContext& ctx);
    void drawGizmo(FrameContext& ctx);
    void drawGrid(FrameContext& ctx) const;
    void drawSelectionMarker(FrameContext& ctx) const;

    CameraSystem& cameras;
    std::string levelFile;
    PlayFactory makePlayState;
    EditorBridge* bridge = nullptr;

    // Declared before `undo`, which holds a reference into whichever
    // registry componentRegistry() picked.
    std::unique_ptr<ComponentRegistry> fallbackRegistry;
    std::unique_ptr<UndoStack> undo;

    Entity selected;
    bool prevMouseLeft = false;
    bool gizmoBusy = false;     // ImGuizmo hovered/dragging (blocks picking)
    bool gizmoWasUsing = false; // drag-edge detection for undo recording
    Transform gizmoDragStart;
    // One continuous inspector interaction (drag/typing) being bracketed for
    // undo: component state at activation, committed on release.
    struct PendingEdit {
        bool active = false;
        Entity entity;
        std::string component;
        nlohmann::json before;
    } pendingEdit;
    int gizmoOp = 0;            // 0 translate, 1 rotate, 2 scale
    char modelPath[256] = "assets/models/";
    std::vector<std::string> pendingAdds;   // shape names; "camera" places one
    bool pendingDuplicate = false;
};

}  // namespace engine

#endif
