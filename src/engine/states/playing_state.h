#ifndef RAYTRACER_ENGINE_STATES_PLAYING_STATE_H
#define RAYTRACER_ENGINE_STATES_PLAYING_STATE_H

#include "../app_state.h"
#include <memory>
#include <vector>
#include <utility>

namespace engine {

class Window;

class PlayingState : public AppState {
public:
    explicit PlayingState(Window& window);

    bool isTransparent() const override { return false; }
    bool blocksInput() const override { return true; }

    template <typename T, typename... Args>
    T& addSystem(Args&&... args) {
        auto system = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *system;
        systems.push_back(std::move(system));
        return ref;
    }

    void onEnter(FrameContext& ctx) override;
    void onExit(FrameContext& ctx) override;
    void onResume(FrameContext& ctx) override;
    void onEvent(const Event& event, FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

protected:
    Window& window;   // states tune cursor mode (editor wants it visible)

private:
    std::vector<std::unique_ptr<System>> systems;
};

}  // namespace engine

#endif
