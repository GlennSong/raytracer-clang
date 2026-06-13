#ifndef RAYTRACER_ENGINE_EDITOR_BRIDGE_H
#define RAYTRACER_ENGINE_EDITOR_BRIDGE_H

#include "component_registry.h"
#include "world.h"
#include <string>
#include <vector>

namespace engine {

class EditorSystem;
class UndoStack;

// Engine -> shell notifications: things the panels should react to NOW
// instead of discovering on their next poll. Same-thread by construction
// (like the bridge itself); the shell drains the queue once per frame.
enum class EditorNotice {
    ModeChanged,        // attach / attachObserver / detach (edit <-> play)
    SelectionChanged,   // viewport pick, hierarchy click, undo, delete
    DocumentSaved,      // dirty flag cleared
};

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
    // Observer mode (play states): panels see the live world — hierarchy and
    // inspector values update in real time — but nothing edits: no editor,
    // no undo log, document writes refused. editable() distinguishes.
    void attachObserver(World* world, std::string levelFile);
    void detach();

    // --- shell side --------------------------------------------------------
    bool attached() const { return worldPtr != nullptr; }
    // Editing rights come from HOW the engine attached (attach vs
    // attachObserver), not from which pointers happen to be set.
    bool editable() const { return worldPtr != nullptr && !observerMode; }
    World* world() { return worldPtr; }
    const std::string& levelPath() const { return levelFile; }

    // Selection is shared with EditorSystem (viewport ring + gizmo follow)
    // while editing; in observer mode the bridge holds it for the panels.
    Entity selected() const;
    void select(Entity entity);

    // The document entities a hierarchy panel shows: SourceSpec-bearing
    // objects and placed cameras, with display labels. `id`/`parentId` are
    // the document hierarchy (0 = root / no id); a tree view builds from them.
    struct EntityInfo {
        Entity entity;
        std::string label;
        bool isCamera = false;
        bool isGroup = false;
        uint32_t id = 0;
        uint32_t parentId = 0;
    };
    std::vector<EntityInfo> listEntities();

    // Reparent `child` under `parent` (an invalid parent = move to root).
    // Refused if it would form a cycle; recorded on the command log. Both
    // must be document (SourceSpec) entities; cameras don't parent yet.
    void reparent(Entity child, Entity parent);

    // Document actions (LevelWriter + camera sidecar — the same "compile"
    // step the in-viewport Play uses).
    bool saveDocument();
    void deleteEntity(Entity entity);

    // The one observer-mode write, and it is deliberately explicit: bake the
    // LIVE playtest over the document — settled physics stacks, pushed
    // props. Serializes exactly what saveDocument would (SourceSpec-bearing
    // entities + cameras; runtime spawns lack SourceSpec and never leak in).
    // Whole-world by design: entities have no stable ids yet, so a
    // per-component "apply this back" cannot find its document counterpart.
    bool bakePlaytestToDocument();

    // Creation requests, queued onto the editor and applied next frame (the
    // spawn point comes from the live view, which only the engine holds).
    void addPrimitive(const std::string& shape);
    void addGroup();
    void placeCamera();
    void duplicateSelected();

    // Anything recorded on the command log since the last saveDocument()
    // (conservative: undoing back to the saved state still reads dirty).
    bool documentDirty();

    // The level's HDR environment is DOCUMENT state, not an entity: these
    // edit the level JSON's environment block directly (saving entities
    // first). Path is level-relative ("../env/x.hdr"); empty removes the
    // HDR so the procedural sky + day/night cycle take over. The shell
    // reloads the editor state afterwards — environment swaps re-cook IBL.
    std::string environmentHdr();
    bool setEnvironmentHdr(const std::string& hdrPath);

    // Notification queue: engine side pushes, shell drains each frame and
    // refreshes immediately when anything arrived.
    void notify(EditorNotice notice) { notices.push_back(notice); }
    std::vector<EditorNotice> drainNotices() {
        std::vector<EditorNotice> out;
        out.swap(notices);
        return out;
    }

    // Component menu actions, routed through the registry's thunks; both are
    // recorded on the editor's command log.
    void addComponent(Entity entity, const std::string& componentName);
    void removeComponent(Entity entity, const std::string& componentName);

    // The edit session's command log (null while detached). Shell panels use
    // it to record their own edits; undo()/redo() apply and re-select.
    UndoStack* undoStack();
    bool canUndo();
    bool canRedo();
    void undo();
    void redo();

private:
    ComponentRegistry registry_;
    World* worldPtr = nullptr;
    EditorSystem* editorPtr = nullptr;
    bool observerMode = false;
    Entity observerSelection;     // panel selection while no editor owns one
    std::string levelFile;
    uint64_t savedRevision = 0;   // command-log revision at the last save
    std::vector<EditorNotice> notices;
};

}  // namespace engine

#endif
