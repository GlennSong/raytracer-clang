#ifndef RAYTRACER_ENGINE_SYSTEMS_TOOL_CYCLE_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_TOOL_CYCLE_SYSTEM_H

#include "../system.h"

namespace engine {

// THE TOOLS ON A CONTROLLER (Glenn, 2026-10-02: "the gamepad should be able to get to the different tools (gun,
// map, flashlight). Maybe something simple like lb/rb to cycle through"). The tools answer the number keys --
// slot_1 hands, slot_2 the gun, slot_3 the map, slot_4 the flashlight, each read by its own system -- so RB / LB
// step through them by PRESSING the next slot (InputMap::injectPress). The number keys keep the cycle in step.
class ToolCycleSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;

private:
    int slot_ = 1;   // 1..4, the tool in hand
};

}  // namespace engine

#endif
