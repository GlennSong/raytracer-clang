#ifndef RAYTRACER_ENGINE_FLASHLIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_FLASHLIGHT_SYSTEM_H

#include "../system.h"

namespace engine {

// THE LIGHT (ADR-0249; Glenn: "We should give the player a flashlight", then "fix the flashlight option to allow it
// to turn on and off"). A switch of its own -- L, or up on the D-pad -- that goes with whatever is in hand: held in
// the right hand with nothing else in it, under the barrel with the gun out, a headlamp with anything else (the
// map, a chute, the grapple, a board). It was tool slot 4, so stepping past it with LB / RB switched it on.
// While on and the player is on foot, a narrow warm cone, aimed where the view looks, is rebuilt every frame in
// render() into its own list (SceneLighting::toolSpots), which RenderSystem merges with the headlights.
// Registered before RenderSystem.
class FlashlightSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

private:
    bool on_ = false;
};

}  // namespace engine

#endif
