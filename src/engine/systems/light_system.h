#ifndef RAYTRACER_ENGINE_LIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_LIGHT_SYSTEM_H

#include "../system.h"
#include "../event_bus.h"
#include "../lights/light_board.h"
#include <vector>

namespace engine {

// THE ONE LIGHT SYSTEM (ADR-0362). Every frame, just before RenderSystem: the street's lamps and the stations' canopy
// lamps (read from StreetFurniture and RailTracks), and every channel the producers published on the board
// (ctx.view.lights: headlights, the flashlight, rooms, cabs, beacons), picked by pool (light_picker.h) into the
// frame's lights, which RenderSystem adds to the level's own. The level's light block sets the street and beacon
// pools; `overlay lamplights off` (LampLightsRequest) empties the street's.
class LightSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

private:
    bool lampLights_ = true;
    EventBus::Subscription onLampLights_;
    std::vector<LightCandidate> cands_;   // reused
};

}  // namespace engine

#endif
