#ifndef RAYTRACER_ENGINE_STATES_EDITOR_STATE_H
#define RAYTRACER_ENGINE_STATES_EDITOR_STATE_H

#include "playing_state.h"
#include "../systems/editor_system.h"
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
                EditorBridge* bridge = nullptr,
                EditorSystem::OpenLevelFactory openLevel = nullptr);

    void onEnter(FrameContext& ctx) override;
    void onResume(FrameContext& ctx) override;

private:
    Renderer& editorRenderer;
    std::string levelFile;
};

}  // namespace engine

#endif
