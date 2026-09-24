#ifndef RAYTRACER_ENGINE_RESIDENCY_SYSTEM_H
#define RAYTRACER_ENGINE_RESIDENCY_SYSTEM_H

#include "../system.h"

namespace engine {

// Drives the level's ResidencyService (residency.h) from the camera each frame, before the
// renderer gathers what to draw: out-of-range items are let go, the nearest wanted ones load
// within a few milliseconds. Logs the per-client totals every few seconds under RT_RESIDENCY_LOG=1.
class ResidencySystem : public System {
public:
    void render(FrameContext& ctx) override;

private:
    double logClock_ = 0.0;
};

}  // namespace engine

#endif
