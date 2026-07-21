#ifndef RAYTRACER_ENGINE_STATES_EDITOR_STATE_H
#define RAYTRACER_ENGINE_STATES_EDITOR_STATE_H

#include "playing_state.h"
#include "../systems/editor_system.h"
#include "../script_watch.h"
#include <string>

namespace engine {

// Edit mode (docs/edit-mode-plan.md): the level JSON is the document. The
// world holds exactly what the document describes, with NO simulation systems
// — nothing can move except by editing. Play saves the document and swaps to
// the game state built by the factory; the game's Stop swaps back here, and
// whatever the simulation did is discarded.
class EditorState : public PlayingState {
public:
    EditorState(Window& window, Renderer& renderer, std::string levelFile,
                EditorSystem::PlayFactory makePlayState,
                EditorBridge* bridge = nullptr);

    void onEnter(FrameContext& ctx) override;
    void onResume(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;

private:
    // Reload the document in place — the same sequence onEnter runs. Used by
    // recipe hot-reload, which the editor previously did not have at all:
    // `watch_scripts` was implemented inside the viewer's play state, so a
    // scene open in the editor never noticed a Lua edit.
    void reloadDocument(FrameContext& ctx);

    Renderer& editorRenderer;
    std::string levelFile;
    ScriptWatch watch;
};

}  // namespace engine

#endif
