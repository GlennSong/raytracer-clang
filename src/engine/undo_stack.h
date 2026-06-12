#ifndef RAYTRACER_ENGINE_UNDO_STACK_H
#define RAYTRACER_ENGINE_UNDO_STACK_H

#include "component_registry.h"
#include "components.h"
#include "camera/scene_camera.h"
#include "world.h"
#include <nlohmann/json.hpp>
#include <deque>
#include <string>

namespace engine {

// The editor's command log (editor-app plan A5). Edits are recorded at the
// existing chokepoints — PropertyInspector::writeField, the ImGui inspector,
// gizmo drag ends, add/duplicate/delete — as small reversible commands:
//
//  - Field edits hold before/after JSON of one component, captured and
//    re-applied through the property layer (describeProperties + the JSON
//    visitors), so undo can restore any described component without naming
//    fields. Consecutive edits to the same field coalesce (spinner steps,
//    drag ticks become one entry).
//  - Transform edits hold the struct directly (orientation is deliberately
//    not a described property).
//  - Create/delete hold a component snapshot of the document entity. Undoing
//    a delete recreates it under a NEW handle; every other command that
//    referenced the old handle is remapped, so the rest of the log stays
//    valid. Mesh handles are reused (uploads outlive entities).
//
// Owned by EditorSystem, one per edit session; both inspectors and the Qt
// shell (via EditorBridge) share it.
class UndoStack {
public:
    explicit UndoStack(ComponentRegistry& registry) : registry(registry) {}

    // --- recording (call at the moment the edit happens) -------------------
    // `fieldLabel` may be null (unknown/bulk); equal before/after is dropped.
    void recordFieldEdit(Entity e, const std::string& component,
                         const char* fieldLabel, nlohmann::json before,
                         nlohmann::json after);
    void recordTransformEdit(Entity e, const Transform& before,
                             const Transform& after);
    void recordCreate(Entity e);
    // Capture BEFORE destroying / removing — they snapshot the live state.
    void recordDelete(World& world, Entity e);
    void recordComponentAdd(World& world, Entity e, const std::string& component);
    void recordComponentRemove(World& world, Entity e,
                               const std::string& component);

    // --- applying -----------------------------------------------------------
    bool canUndo() const { return !undoList.empty(); }
    bool canRedo() const { return !redoList.empty(); }
    // Returns the affected entity (a selection hint; invalid when nothing
    // was applied or the entity is gone).
    Entity undo(World& world);
    Entity redo(World& world);
    void clear();
    std::size_t depth() const { return undoList.size(); }

    // One component's described state as JSON — the snapshot inspectors take
    // around a write. Null JSON when the component is absent.
    nlohmann::json componentState(World& world, Entity e,
                                  const std::string& component);

private:
    struct EntitySnapshot {
        Transform transform;
        SourceSpec spec;
        Renderable renderable;
        SceneCamera camera;
        bool hasTransform = false, hasPrev = false, hasSpec = false,
             hasRenderable = false, hasCamera = false, hasSpawn = false;
    };

    struct Command {
        enum Kind {
            FieldEdit,
            TransformEdit,
            Create,
            Delete,
            ComponentAdd,
            ComponentRemove
        } kind = FieldEdit;
        Entity entity;
        std::string component;    // FieldEdit / ComponentAdd / ComponentRemove
        std::string fieldLabel;   // FieldEdit coalescing key ("" = none)
        nlohmann::json before, after;   // FieldEdit; ComponentRemove uses before
        Transform transformBefore, transformAfter;
        EntitySnapshot snapshot;        // Create (filled on undo) / Delete
    };

    void push(Command cmd);
    Entity apply(Command& cmd, World& world, bool forward);
    void applyComponentJson(World& world, Entity e, const std::string& component,
                            const nlohmann::json& state);
    EntitySnapshot capture(World& world, Entity e);
    Entity recreate(World& world, const EntitySnapshot& snapshot);
    void remap(Entity from, Entity to);

    ComponentRegistry& registry;
    std::deque<Command> undoList;
    std::deque<Command> redoList;
};

}  // namespace engine

#endif
