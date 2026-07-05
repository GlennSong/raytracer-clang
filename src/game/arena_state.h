#ifndef RAYTRACER_GAME_ARENA_STATE_H
#define RAYTRACER_GAME_ARENA_STATE_H

#include "../engine/states/playing_state.h"
#include <functional>
#include <memory>
#include <string>

namespace engine {
class Renderer;
class EditorBridge;
}

class ArenaState : public engine::PlayingState {
public:
    using EditorFactory = std::function<std::unique_ptr<engine::AppState>()>;

    // makeEditorState, when provided, reroutes Esc from "quit" to "stop and
    // return to the editor" (the play half of the edit/play loop). `bridge`,
    // when provided, is attached in OBSERVER mode for the playtest: the
    // shell's hierarchy/inspector show live values, editing stays off.
    ArenaState(engine::Window& window, engine::Renderer& renderer,
               const std::string& levelFile,
               EditorFactory makeEditorState = nullptr,
               engine::EditorBridge* bridge = nullptr);

    void onEnter(engine::FrameContext& ctx) override;
    void onExit(engine::FrameContext& ctx) override;
    void update(engine::FrameContext& ctx) override;

private:
    engine::Renderer& arenaRenderer;
    std::string levelFile;
    EditorFactory makeEditorState;
    engine::EditorBridge* bridge = nullptr;

    // Recipe hot-reload (the BUILDING LAB loop, building-grammar-plan.md P1):
    // a level with `"watch_scripts": true` watches its own file plus every
    // shape:"script" recipe file; when one changes on disk the state replaces
    // itself through the state machine — the same clean reset as the
    // editor→play transition — so a saved Lua edit shows up in seconds.
    // Desktop only (the web bundle bakes assets; nothing changes on disk).
    struct WatchedFile { std::string path; long long stamp = 0; };
    std::vector<WatchedFile> watchFiles;   // empty = watching disabled
    double watchTimer = 0.0;
    void collectWatchFiles();
    bool watchedFilesChanged();
};

#endif
