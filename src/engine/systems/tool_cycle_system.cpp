#include "tool_cycle_system.h"
#include "../input/input_map.h"
#include "../player/tools.h"
#include <string>

namespace engine {

constexpr int kSlots = kToolSlots;   // (engine/player/tools.h; the light is a switch of its own, not a slot)

void ToolCycleSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("tool_next", GamepadButton::RightBumper);
    ctx.actions.bindButton("tool_prev", GamepadButton::LeftBumper);
    ctx.actions.setActionContext("tool_next", InputContext::OnFoot);
    ctx.actions.setActionContext("tool_prev", InputContext::OnFoot);
}

void ToolCycleSystem::update(FrameContext& ctx) {
    for (int k = 1; k <= kSlots; ++k)
        if (ctx.actions.pressed(slotAction(k))) slot_ = k;
    int step = 0;
    if (ctx.actions.pressed("tool_next")) step = 1;
    if (ctx.actions.pressed("tool_prev")) step = -1;
    if (step == 0) return;
    slot_ = (slot_ - 1 + step + kSlots) % kSlots + 1;
    ctx.actions.injectPress(slotAction(slot_));
}

}  // namespace engine
