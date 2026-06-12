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
    levelFile = std::move(level);
}

void EditorBridge::detach() {
    worldPtr = nullptr;
    editorPtr = nullptr;
}

Entity EditorBridge::selected() const {
    return editorPtr ? editorPtr->selectedEntity() : Entity{};
}

void EditorBridge::select(Entity entity) {
    if (editorPtr) editorPtr->setSelected(entity);
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
    return out;
}

bool EditorBridge::saveDocument() {
    if (!worldPtr) return false;
    bool ok = LevelWriter::save(levelFile, *worldPtr);
    ok &= CameraStore::save(levelFile + ".cameras.json", *worldPtr);
    return ok;
}

void EditorBridge::deleteEntity(Entity entity) {
    if (!worldPtr || !worldPtr->alive(entity)) return;
    if (selected() == entity) select(Entity{});
    worldPtr->destroy(entity);
}

}  // namespace engine
