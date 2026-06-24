#ifndef RAYTRACER_ENGINE_EDITOR_SYSTEM_H
#define RAYTRACER_ENGINE_EDITOR_SYSTEM_H

#include "../system.h"
#include "camera_system.h"
#include "../app_state.h"
#include "../editor_bridge.h"
#include "../path_edit_tool.h"   // road/curve handle dragging in the viewport (ADR-0050)
#include "../undo_stack.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace engine {

class Renderer;

// Road sub-object editing (ADR-0049), for the viewport's node handles. The engine
// owns the data + regen; the shell owns only the handle drawing and mouse picking.
//   roadNodeHandles  -> a world position per control node (seated on the road), so
//                       the viewport can draw a draggable handle at each.
//   moveRoadNode     -> drop a node at a new world XZ: updates the RoadNet, rebuilds
//                       the carriageway mesh, and keeps the saved recipe in sync.
std::vector<Vec3> roadNodeHandles(World& world, Entity e);
bool moveRoadNode(World& world, Entity e, int node, const Vec3& worldPos, Renderer& renderer);
// Spline tangent handles (ADR-0049): a world point per node offset along its current
// tangent (drawn distinct from the node handles). Dragging one sets that knot's
// tangent (handle - node), shaping the curve, and regenerates. roadTangentHandles
// returns an empty list for a straight (non-curved) road.
std::vector<Vec3> roadTangentHandles(World& world, Entity e);
bool moveRoadTangent(World& world, Entity e, int node, const Vec3& worldPos, Renderer& renderer);
// Road topology editing for the viewport (ADR-0049). Each regenerates the mesh +
// syncs the saved recipe. nearestRoadEdge picks which edge a world point is over
// (-1 = none within maxDist), so the shell can route a click on the road to a split;
// splitRoadEdge / extendRoad return the new node's index (to select it); deleteRoadNode
// removes a node + its edges.
int  nearestRoadEdge(World& world, Entity e, const Vec3& worldPos, double maxDist);
bool setRoadEdgeWidth(World& world, Entity e, int edge, double width, Renderer& renderer);
int  splitRoadEdge(World& world, Entity e, int edge, const Vec3& worldPos, Renderer& renderer);
int  extendRoad(World& world, Entity e, int fromNode, const Vec3& worldPos, Renderer& renderer);
bool deleteRoadNode(World& world, Entity e, int node, Renderer& renderer);

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
    void setSelected(Entity entity) {
        if (entity.valid()) setSelection({entity}, entity);
        else setSelection({}, Entity{});
    }
    // Multi-selection: `selection` is the full set, `selected` the primary
    // (the gizmo anchor + what the inspector shows). primary is forced into
    // the set; an empty set clears the primary. Inline (like the other
    // selection accessors) so the bridge can call them without force-linking
    // this TU — and the state stack it drags — into renderer-less hosts.
    const std::vector<Entity>& selectionList() const { return selection; }
    bool isSelected(Entity e) const {
        for (Entity s : selection)
            if (s == e) return true;
        return false;
    }
    void setSelection(std::vector<Entity> sel, Entity primary) {
        selection = std::move(sel);
        if (!primary.valid() || !isSelected(primary))
            primary = selection.empty() ? Entity{} : selection.back();
        selected = primary;
    }
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
    Entity addGroup(FrameContext& ctx);
    Entity addPlayerSpawn(FrameContext& ctx);
    Entity duplicateSelected(FrameContext& ctx);
    Vec3 spawnPoint(FrameContext& ctx) const;
    void processShellRequests(FrameContext& ctx);

    void drawToolbar(FrameContext& ctx);
    void drawInspector(FrameContext& ctx);
    void drawGizmo(FrameContext& ctx);
    void drawGrid(FrameContext& ctx) const;
    void drawSelectionMarker(FrameContext& ctx) const;
    // Road handle dragging: rebind the tool to the selected road and run the click/drag
    // this frame; returns true when it grabbed a handle (so the click isn't also a pick).
    bool updatePathEdit(FrameContext& ctx, bool click);
    void drawPathHandles(FrameContext& ctx) const;
    // Re-grade the CDLOD terrain to every road in the world (ADR-0044): recompute each
    // road's cut/fill footprints from the natural ground and rebuild terrain flatten =
    // base + roads, then re-drape the roads on the carved result. The "Conform" action.
    void conformTerrainToRoads(FrameContext& ctx);
    void drawGroupMarkers(FrameContext& ctx) const;
    void drawCameraFrustums(FrameContext& ctx) const;

    CameraSystem& cameras;
    std::string levelFile;
    PlayFactory makePlayState;
    EditorBridge* bridge = nullptr;

    // Declared before `undo`, which holds a reference into whichever
    // registry componentRegistry() picked.
    std::unique_ptr<ComponentRegistry> fallbackRegistry;
    std::unique_ptr<UndoStack> undo;

    Entity selected;                  // primary (gizmo anchor, inspector)
    std::vector<Entity> selection;    // full set, includes the primary
    Entity lastNoticedSelection;   // edge detection for SelectionChanged
    std::size_t selectionSig = 0;  // folds the whole set for change detection
    bool prevMouseLeft = false;
    // Viewport path editing (ADR-0050): the tool keeps drag state across frames; the
    // source is re-seated when the selected road (or its net pointer) changes.
    PathEditTool pathTool;
    std::unique_ptr<RoadHandleSource> roadSource;
    Entity pathEditEntity;
    struct RoadNet* pathRoadNet = nullptr;
    bool pathWasDragging = false;
    bool gizmoBusy = false;     // ImGuizmo hovered/dragging (blocks picking)
    bool gizmoWasUsing = false; // drag-edge detection for undo recording
    Transform gizmoDragStart;
    // Captured at gizmo-drag start so the whole selection moves rigidly with
    // the primary and one undo entry covers the group.
    Mat4 dragStartPrimaryWorld;
    std::vector<Entity> dragEntities;
    std::vector<Transform> dragStartLocals;
    std::vector<Mat4> dragStartWorlds;
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
