#include "state_stack.h"

namespace engine {

void StateStack::pushState(std::unique_ptr<AppState> state) {
    pending.push_back({OpType::Push, std::move(state)});
}

void StateStack::popState() {
    pending.push_back({OpType::Pop, nullptr});
}

void StateStack::onStart(FrameContext& ctx) {
    applyPending(ctx);
}

void StateStack::onStop(FrameContext& ctx) {
    while (!stack.empty()) {
        stack.back()->onExit(ctx);
        stack.pop_back();
    }
}

void StateStack::onEvent(const Event& event, FrameContext& ctx) {
    if (!stack.empty())
        stack.back()->onEvent(event, ctx);
}

void StateStack::forEachActive(const std::function<void(AppState&)>& fn) {
    if (stack.empty()) return;

    // Walk top-down to find the lowest state that blocks input
    int floor = static_cast<int>(stack.size()) - 1;
    for (int i = static_cast<int>(stack.size()) - 1; i >= 0; --i) {
        floor = i;
        if (stack[i]->blocksInput()) break;
    }

    for (int i = floor; i < static_cast<int>(stack.size()); ++i)
        fn(*stack[i]);
}

void StateStack::forEachRenderable(const std::function<void(AppState&)>& fn) {
    if (stack.empty()) return;

    // Walk top-down through transparent states to find the opaque floor
    int floor = static_cast<int>(stack.size()) - 1;
    for (int i = static_cast<int>(stack.size()) - 1; i >= 0; --i) {
        floor = i;
        if (!stack[i]->isTransparent()) break;
    }

    // Render bottom-up so overlays draw on top
    for (int i = floor; i < static_cast<int>(stack.size()); ++i)
        fn(*stack[i]);
}

void StateStack::applyPending(FrameContext& ctx) {
    for (auto& op : pending) {
        if (op.type == OpType::Push) {
            if (!stack.empty())
                stack.back()->onSuspend(ctx);
            stack.push_back(std::move(op.state));
            stack.back()->onEnter(ctx);
        } else if (op.type == OpType::Pop && !stack.empty()) {
            stack.back()->onExit(ctx);
            stack.pop_back();
            if (!stack.empty())
                stack.back()->onResume(ctx);
        }
    }
    pending.clear();
}

}  // namespace engine
