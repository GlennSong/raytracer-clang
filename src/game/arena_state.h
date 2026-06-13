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
};

#endif
