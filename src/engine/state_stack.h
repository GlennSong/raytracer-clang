#ifndef RAYTRACER_ENGINE_STATE_STACK_H
#define RAYTRACER_ENGINE_STATE_STACK_H

#include "app_state.h"
#include <memory>
#include <vector>
#include <functional>

namespace engine {

class StateStack {
public:
    void pushState(std::unique_ptr<AppState> state);
    void popState();

    void onStart(FrameContext& ctx);
    void onStop(FrameContext& ctx);
    void onEvent(const Event& event, FrameContext& ctx);

    void forEachActive(const std::function<void(AppState&)>& fn);
    void forEachRenderable(const std::function<void(AppState&)>& fn);

    void applyPending(FrameContext& ctx);

    bool empty() const { return stack.empty(); }
    AppState* top() const { return stack.empty() ? nullptr : stack.back().get(); }

private:
    enum class OpType { Push, Pop };
    struct PendingOp {
        OpType type;
        std::unique_ptr<AppState> state;
    };

    std::vector<std::unique_ptr<AppState>> stack;
    std::vector<PendingOp> pending;
};

}  // namespace engine

#endif
