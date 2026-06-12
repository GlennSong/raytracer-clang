#include "editor_bridge.h"

#include "components.h"
#include "camera/scene_camera.h"
#include "camera_store.h"
#include "level_writer.h"
#include "systems/editor_system.h"

namespace engine {

void EditorBridge::attach(World* world, EditorSystem* editor,
                          std::string level) {
    worldPtr = world;
    editorPtr = editor;
    observerMode = false;
    levelFile = std::move(level);
    // A session begins on a freshly loaded (or just-saved) document.
    savedRevision = editor && editor->undoStack()
                        ? editor->undoStack()->revision()
                        : 0;
    notify(EditorNotice::ModeChanged);
}

void EditorBridge::attachObserver(World* world, std::string level) {
    worldPtr = world;
    editorPtr = nullptr;
    observerMode = true;
    observerSelection = Entity{};
    levelFile = std::move(level);
    notify(EditorNotice::ModeChanged);
}

void EditorBridge::detach() {
    worldPtr = nullptr;
    editorPtr = nullptr;
    observerMode = false;
    observerSelection = Entity{};
    notify(EditorNotice::ModeChanged);
}

Entity EditorBridge::selected() const {
    return editorPtr ? editorPtr->selectedEntity() : observerSelection;
}

void EditorBridge::select(Entity entity) {
    if (editorPtr)
        editorPtr->setSelected(entity);   // EditorSystem notices the change
    else if (!(observerSelection == entity)) {
        observerSelection = entity;
        notify(EditorNotice::SelectionChanged);
    }
}

std::vector<EditorBridge::EntityInfo> EditorBridge::listEntities() {
    std::vector<EntityInfo> out;
    if (!worldPtr) return out;

    worldPtr->each<Transform, SourceSpec>(
        [&](Entity e, Transform&, SourceSpec& spec) {
            EntityInfo info;
            info.entity = e;
            if (!spec.meshFile.empty()) {
                // Basename keeps the hierarchy readable for long asset paths.
                std::size_t slash = spec.meshFile.find_last_of("/\\");
                info.label = (slash == std::string::npos)
                                 ? spec.meshFile
                                 : spec.meshFile.substr(slash + 1);
            } else {
                info.label = spec.shape;
            }
            info.label += " #" + std::to_string(e.index);
            out.push_back(std::move(info));
        });
    worldPtr->each<Transform, SceneCamera>(
        [&](Entity e, Transform&, SceneCamera& cam) {
            out.push_back({e, cam.name, true});
        });
    worldPtr->each<Transform, PlayerSpawn>(
        [&](Entity e, Transform&, PlayerSpawn&) {
            out.push_back({e, "Player Spawn", false});
        });
    // During a playtest (observer mode) the live player is the entity worth
    // watching; it carries no SourceSpec, so list it explicitly.
    worldPtr->each<Transform, ControlledBy>(
        [&](Entity e, Transform&, ControlledBy&) {
            out.push_back({e, "Player (live)", false});
        });
    return out;
}

bool EditorBridge::saveDocument() {
    // Observer mode must never compile a live playtest over the document.
    if (!editable()) return false;
    bool ok = LevelWriter::save(levelFile, *worldPtr);
    ok &= CameraStore::save(levelFile + ".cameras.json", *worldPtr);
    if (ok) {
        if (UndoStack* undo = undoStack()) savedRevision = undo->revision();
        notify(EditorNotice::DocumentSaved);
    }
    return ok;
}

void EditorBridge::addPrimitive(const std::string& shape) {
    if (editorPtr && worldPtr) editorPtr->requestAddPrimitive(shape);
}

void EditorBridge::placeCamera() {
    if (editorPtr && worldPtr) editorPtr->requestPlaceCamera();
}

void EditorBridge::duplicateSelected() {
    if (editorPtr && worldPtr) editorPtr->requestDuplicate();
}

bool EditorBridge::documentDirty() {
    UndoStack* undo = undoStack();
    return attached() && undo && undo->revision() != savedRevision;
}

void EditorBridge::deleteEntity(Entity entity) {
    if (!editable() || !worldPtr->alive(entity)) return;
    if (selected() == entity) select(Entity{});
    if (UndoStack* undo = undoStack()) undo->recordDelete(*worldPtr, entity);
    worldPtr->destroy(entity);
}

void EditorBridge::addComponent(Entity entity, const std::string& componentName) {
    ComponentRegistry::Entry* entry = registry_.find(componentName);
    if (!editable() || !worldPtr->alive(entity) || !entry || !entry->addTo ||
        entry->has(*worldPtr, entity))
        return;
    entry->addTo(*worldPtr, entity);
    if (UndoStack* undo = undoStack())
        undo->recordComponentAdd(*worldPtr, entity, componentName);
}

void EditorBridge::removeComponent(Entity entity,
                                   const std::string& componentName) {
    ComponentRegistry::Entry* entry = registry_.find(componentName);
    if (!editable() || !worldPtr->alive(entity) || !entry || !entry->removeFrom ||
        !entry->has(*worldPtr, entity))
        return;
    if (UndoStack* undo = undoStack())
        undo->recordComponentRemove(*worldPtr, entity, componentName);
    entry->removeFrom(*worldPtr, entity);
}

UndoStack* EditorBridge::undoStack() {
    return editorPtr ? editorPtr->undoStack() : nullptr;
}

bool EditorBridge::canUndo() {
    UndoStack* undo = undoStack();
    return undo && undo->canUndo();
}

bool EditorBridge::canRedo() {
    UndoStack* undo = undoStack();
    return undo && undo->canRedo();
}

void EditorBridge::undo() {
    UndoStack* undo = undoStack();
    if (!undo || !worldPtr) return;
    select(undo->undo(*worldPtr));
}

void EditorBridge::redo() {
    UndoStack* undo = undoStack();
    if (!undo || !worldPtr) return;
    select(undo->redo(*worldPtr));
}

}  // namespace engine
