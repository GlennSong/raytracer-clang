#ifndef RAYTRACER_ENGINE_CLOUD_MAP_SYSTEM_H
#define RAYTRACER_ENGINE_CLOUD_MAP_SYSTEM_H

#include "../system.h"
#include "../sky/cloud_map.h"
#include <future>

namespace engine {

// THE CLOUDS' PLACES AND DRIFT (ADR-0386). After DayNightSystem has set the weather's cloud knobs on the view
// (coverage, the type range), each frame:
// - the drift: the clouds carried along the weather's wind (VolumetricCloudParams::driftX/Z; RT_WIND_TIME pins it,
//   as the renderer's sway: the cloud lab's honest A/B) -- the renderer used to integrate it, out of the engine's
//   reach, and the map has to be laid out in the same air;
// - the types from clouds.lua onto the params, and the deck's span from the types the weather spreads over;
// - the WEATHER MAP (sky/cloud_map.h) round the camera in the air, rebaked on a worker when the camera has drifted
//   `rebuild_m` through the air or the weather has moved on, and handed to the renderer (setCloudMap).
class CloudMapSystem : public System {
public:
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    // the last map handed over (tests, the lab)
    const CloudMapInputs& builtFrom() const { return built_; }
    bool hasMap() const { return haveMap_; }

private:
    void startBake(const CloudMapInputs& in);

    double driftX_ = 0.0, driftZ_ = 0.0;
    std::future<CloudMapImage> pending_;
    bool baking_ = false;
    bool haveMap_ = false;
    CloudMapInputs built_;
    double sinceBake_ = 1e9;   // seconds since the last bake started
    TextureHandle tex_{};
    // the map before: kept a few frames after the new one is handed over (a frame in flight may still read it)
    TextureHandle oldTex_{};
    int oldTexFrames_ = 0;
};

}  // namespace engine

#endif
