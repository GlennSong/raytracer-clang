#ifndef RAYTRACER_ENGINE_EDITOR_BRIDGE_H
#define RAYTRACER_ENGINE_EDITOR_BRIDGE_H

#include "component_registry.h"
#include "world.h"
#include <string>
#include <vector>

namespace engine {

class EditorSystem;

// The single conduit between a native shell (the Qt editor) and the engine
// (editor-app plan, Phase A3). The shell owns the bridge for the application's
// lifetime; the engine attaches while an EditorState is active and detaches
// when it isn't (e.g. during Play), so panels can gray out instead of touching
// a world that gameplay owns. Same-thread by construction: the shell's UI and
// Application::runFrame interleave on one event loop, so direct World access
// between frames is safe.
class EditorBridge {
public:
    EditorBridge() { registerEngineComponents(registry_); }

    // The component vocabulary panels render from (engine components built
    // in; game code may register more before the shell builds its UI).
    ComponentRegistry& registry() { return registry_; }

    // --- engine side (EditorSystem) ---------------------------------------
    void attach(World* world, EditorSystem* editor, std::string levelFile);
    void detach();

    // --- shell side --------------------------------------------------------
    bool attached() const { return worldPtr != nullptr; }
    World* world() { return worldPtr; }
    const std::string& levelPath() const { return levelFile; }

    // Selection is shared with EditorSystem (viewport ring + gizmo follow).
    Entity selected() const;
    void select(Entity entity);

    // The document entities a hierarchy panel shows: SourceSpec-bearing
    // objects and placed cameras, with display labels.
    struct EntityInfo {
        Entity entity;
        std::string label;
        bool isCamera = false;
    };
    std::vector<EntityInfo> listEntities();

    // Document actions (LevelWriter + camera sidecar — the same "compile"
    // step the in-viewport Play uses).
    bool saveDocument();
    void deleteEntity(Entity entity);

private:
    ComponentRegistry registry_;
    World* worldPtr = nullptr;
    EditorSystem* editorPtr = nullptr;
    std::string levelFile;
};

}  // namespace engine

#endif
