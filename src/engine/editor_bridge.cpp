#include "editor_bridge.h"

#include "components.h"
#include "camera/scene_camera.h"
#include "camera_store.h"
#include "level_writer.h"
#include "systems/editor_system.h"
#include "../log.h"
#include <nlohmann/json.hpp>
#include <fstream>

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

std::vector<Entity> EditorBridge::selectionList() const {
    if (editorPtr) return editorPtr->selectionList();
    if (observerSelection.valid()) return {observerSelection};
    return {};
}

void EditorBridge::setSelection(const std::vector<Entity>& entities,
                                Entity primary) {
    if (editorPtr)
        editorPtr->setSelection(entities, primary);
    else
        select(primary.valid() ? primary
                               : (entities.empty() ? Entity{} : entities.back()));
}

std::vector<EditorBridge::EntityInfo> EditorBridge::listEntities() {
    std::vector<EntityInfo> out;
    if (!worldPtr) return out;

    worldPtr->each<Transform, SourceSpec>(
        [&](Entity e, Transform&, SourceSpec& spec) {
            EntityInfo info;
            info.entity = e;
            info.id = spec.id;
            info.parentId = spec.parentId;
            info.isGroup = spec.isGroup();
            if (!spec.name.empty()) {
                info.label = spec.name;   // user-given names stand alone
            } else if (info.isGroup) {
                info.label = "Group #" + std::to_string(e.index);
            } else if (!spec.meshFile.empty()) {
                // Basename keeps the hierarchy readable for long asset paths.
                std::size_t slash = spec.meshFile.find_last_of("/\\");
                info.label = (slash == std::string::npos)
                                 ? spec.meshFile
                                 : spec.meshFile.substr(slash + 1);
                info.label += " #" + std::to_string(e.index);
            } else {
                info.label = spec.shape + " #" + std::to_string(e.index);
            }
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

bool EditorBridge::bakePlaytestToDocument() {
    if (!attached() || !observerMode) return false;
    bool ok = LevelWriter::save(levelFile, *worldPtr);
    ok &= CameraStore::save(levelFile + ".cameras.json", *worldPtr);
    if (ok) notify(EditorNotice::DocumentSaved);
    return ok;
}

std::string EditorBridge::environmentHdr() {
    std::ifstream in(levelFile);
    if (!in.is_open()) return "";
    try {
        nlohmann::json root = nlohmann::json::parse(in);
        if (root.contains("environment"))
            return root["environment"].value("hdr", std::string());
    } catch (const nlohmann::json::exception&) {
    }
    return "";
}

bool EditorBridge::setEnvironmentHdr(const std::string& hdrPath) {
    if (!editable()) return false;
    // Persist the entities first so the JSON edit below starts from the
    // current world, then touch ONLY the environment's hdr key (skyColor
    // and friends survive).
    if (!saveDocument()) return false;

    nlohmann::json root;
    {
        std::ifstream in(levelFile);
        if (!in.is_open()) return false;
        try {
            root = nlohmann::json::parse(in);
        } catch (const nlohmann::json::exception& e) {
            LOG_ERROR << "Level " << levelFile << " unreadable: " << e.what();
            return false;
        }
    }
    if (hdrPath.empty()) {
        if (root.contains("environment")) root["environment"].erase("hdr");
    } else {
        root["environment"]["hdr"] = hdrPath;
    }

    std::ofstream out(levelFile);
    if (!out.is_open()) return false;
    out << root.dump(2) << "\n";
    LOG_INFO << "Environment HDR "
             << (hdrPath.empty() ? "removed (procedural sky)" : hdrPath);
    return true;
}

void EditorBridge::addPrimitive(const std::string& shape) {
    if (editorPtr && worldPtr) editorPtr->requestAddPrimitive(shape);
}

void EditorBridge::addGroup() {
    if (editorPtr && worldPtr) editorPtr->requestAddPrimitive("group");
}

void EditorBridge::placeCamera() {
    if (editorPtr && worldPtr) editorPtr->requestPlaceCamera();
}

void EditorBridge::reparent(Entity child, Entity parent) {
    if (!editable() || !worldPtr->alive(child)) return;
    SourceSpec* childSpec = worldPtr->get<SourceSpec>(child);
    Transform* childT = worldPtr->get<Transform>(child);
    if (!childSpec || !childT) return;

    // An invalid parent means "move to root". A valid one must be a document
    // entity and must not close a cycle.
    uint32_t newParentId = 0;
    Entity parentEntity;
    if (parent.valid()) {
        SourceSpec* parentSpec = worldPtr->get<SourceSpec>(parent);
        if (!parentSpec || child == parent) return;
        if (wouldCreateCycle(*worldPtr, child, parent)) {
            LOG_WARN << "Reparent refused: would create a cycle";
            return;
        }
        newParentId = parentSpec->id;
        parentEntity = parent;
    }
    if (newParentId == childSpec->parentId) return;

    // Keep the child's WORLD position: its local transform is rewritten to be
    // relative to the new parent (Unity's worldPositionStays). Grabbing the
    // group doesn't teleport its children; moving the group then carries them
    // along by their preserved relative offsets.
    Mat4 childWorld = worldMatrix(*worldPtr, child);    // uses the OLD parent
    Mat4 newParentWorld =
        parentEntity.valid() ? worldMatrix(*worldPtr, parentEntity) : Mat4();
    Transform before = *childT;
    Transform after = transformFromMatrix(newParentWorld.inverse() * childWorld);

    if (UndoStack* undo = undoStack())
        undo->recordReparent(child, childSpec->parentId, newParentId, before,
                             after);
    childSpec->parentId = newParentId;
    *childT = after;
    if (auto* prev = worldPtr->get<PrevTransform>(child)) prev->value = after;
    notify(EditorNotice::SelectionChanged);   // tree shape changed
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

void EditorBridge::deleteSelection() {
    if (!editable()) return;
    // Copy first: deleteEntity mutates the live selection set.
    std::vector<Entity> doomed = selectionList();
    if (doomed.empty()) doomed = {selected()};
    setSelection({}, Entity{});
    for (Entity e : doomed) deleteEntity(e);
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
