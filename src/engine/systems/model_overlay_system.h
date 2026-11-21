#ifndef RAYTRACER_ENGINE_SYSTEMS_MODEL_OVERLAY_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_MODEL_OVERLAY_SYSTEM_H

// Draws every stamped model's collision shapes, markers and joints as debug lines when asked (#191;
// engine/model_overlay.h). Register it before DebugDrawSystem, which draws the frame's lines.

#include "../system.h"

namespace engine {

class ModelOverlaySystem : public System {
public:
    void render(FrameContext& ctx) override;
};

}  // namespace engine

#endif
