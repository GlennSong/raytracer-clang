#ifndef RAYTRACER_ENGINE_FLASHLIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_FLASHLIGHT_SYSTEM_H

#include "../system.h"

namespace engine {

// THE FLASHLIGHT (tool slot 4; Glenn: "We should give the player a flashlight as a 4th tool"). 4 switches
// it on and off. While on and the player is on foot, a narrow warm cone rides the camera -- held a little
// low and to the right, aimed where the view looks -- staged into the same spot list the headlights use
// (SceneLighting::vehicleSpots), so the renderer lights it with no new path. Staged in render() (VehicleSystem
// rebuilds that list in its fixed step) and registered before RenderSystem.
class FlashlightSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    bool on() const { return on_; }

private:
    bool on_ = false;
};

}  // namespace engine

#endif
