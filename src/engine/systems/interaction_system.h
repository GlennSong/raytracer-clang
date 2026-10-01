#ifndef RAYTRACER_ENGINE_SYSTEMS_INTERACTION_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_INTERACTION_SYSTEM_H

#include "../interaction.h"
#include "../system.h"
#include <string>

namespace engine {

// THE PLAYER'S INTERACTIONS with furniture (the furniture library, M1; engine/interaction.h). Each frame, on foot,
// the best piece within reach that the camera faces (findInteraction over every Interactables set) and a prompt --
// "E  sit   ·   hold E  lie down". TAP the interact key for the primary verb, HOLD it for the secondary. While the
// player is Seated, PlayerSystem pins the camera to the pose's eye; any move key or E stands them up where they were.
class InteractionSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

    static constexpr double HOLD_S = 0.4;   // hold this long for the secondary verb

private:
    void standUp(FrameContext& ctx, Entity player);
    InteractChoice choice_;
    double holdT_ = 0;
    bool holding_ = false;
    bool holdFired_ = false;
    std::string prompt_;
};

}  // namespace engine

#endif
