#ifndef RAYTRACER_ENGINE_SYSTEMS_INTERACTION_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_INTERACTION_SYSTEM_H

#include "../interaction.h"
#include "../system.h"

namespace engine {

// FURNITURE as an interaction PROVIDER (the furniture library, M1; engine/interaction.h, interact_broker.h). Each
// frame, on foot, every piece within reach with a free verb is OFFERED to the broker -- its anchor where the body
// would go, the tap verb and the hold verb; the broker decides which one the player means. On the broker's command
// the player sits or lies (Seated). While Seated the only offer is "stand up" (exclusive), and any move key stands
// them up too, where they stood.
class InteractionSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;

    static constexpr double REACH = 1.8;

private:
    void standUp(FrameContext& ctx, Entity player);
};

}  // namespace engine

#endif
