#ifndef RAYTRACER_ENGINE_RENDER_SYSTEM_H
#define RAYTRACER_ENGINE_RENDER_SYSTEM_H

#include "../system.h"

namespace engine {

// Draws the world each frame using the RenderView's camera and lights, with
// per-entity interpolation for smooth motion. Also ramps exposure from input.
// Application brackets render() with begin/endFrame, so this only emits draws.
class RenderSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    float exposure = 0.5f;
};


}  // namespace engine

#endif
