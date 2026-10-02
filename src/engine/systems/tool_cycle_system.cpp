#include "tool_cycle_system.h"
#include "../input/input_map.h"
#include <string>

namespace engine {

void ToolCycleSystem::onStart(FrameContext& ctx) {
    ctx.actions.bindButton("tool_next", GamepadButton::RightBumper);
    ctx.actions.bindButton("tool_prev", GamepadButton::LeftBumper);
    ctx.actions.setActionContext("tool_next", InputContext::OnFoot);
    ctx.actions.setActionContext("tool_prev", InputContext::OnFoot);
}

void ToolCycleSystem::update(FrameContext& ctx) {
    for (int k = 1; k <= 4; ++k)
        if (ctx.actions.pressed("slot_" + std::to_string(k))) slot_ = k;
    int step = 0;
    if (ctx.actions.pressed("tool_next")) step = 1;
    if (ctx.actions.pressed("tool_prev")) step = -1;
    if (step == 0) return;
    slot_ = (slot_ - 1 + step + 4) % 4 + 1;
    ctx.actions.injectPress("slot_" + std::to_string(slot_));
}

}  // namespace engine
