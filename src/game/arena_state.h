#ifndef RAYTRACER_GAME_ARENA_STATE_H
#define RAYTRACER_GAME_ARENA_STATE_H

#include "../engine/states/playing_state.h"
#include <string>

namespace engine { class Renderer; }

class ArenaState : public engine::PlayingState {
public:
    ArenaState(engine::Window& window, engine::Renderer& renderer,
               const std::string& levelFile);

    void onEnter(engine::FrameContext& ctx) override;

private:
    engine::Renderer& arenaRenderer;
    std::string levelFile;
};

#endif
