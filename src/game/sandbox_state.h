#ifndef RAYTRACER_GAME_SANDBOX_STATE_H
#define RAYTRACER_GAME_SANDBOX_STATE_H

#include "../engine/states/playing_state.h"

namespace engine {
class Renderer;
}

// The AR sandbox: mixed-immersion passthrough of the user's real room, with
// detected surfaces as physics colliders and (coming rounds) hand-driven
// placement of procedurally generated objects. Deliberately minimal next to
// ArenaState: no level file, no city, no terrain, no PlayerSystem — locomotion
// is the user physically walking, so the pinch gesture belongs entirely to
// interaction, and the world scale is locked to life-size (1 world unit per
// real metre) so a spawned car is car-sized on the real floor.
//
// Boots via the launcher's "AR sandbox" entry (vision_spike.mm maps the
// reserved level name "sandbox" here); harmless on desktop, where it renders
// an empty world.
class SandboxState : public engine::PlayingState {
public:
    SandboxState(engine::Window& window, engine::Renderer& renderer);

    void onEnter(engine::FrameContext& ctx) override;

private:
    engine::Renderer& renderer_;
};

#endif  // RAYTRACER_GAME_SANDBOX_STATE_H
