#ifndef RAYTRACER_ENGINE_PLANET_LAB_SYSTEM_H
#define RAYTRACER_ENGINE_PLANET_LAB_SYSTEM_H

#include "../system.h"
#include "../world.h"

namespace engine {

// A live procedural-planet editor (procedural-planet-plan). Draws a "Planet Lab"
// section in the debug overlay (backtick) with sliders for the rocky/gas-giant
// generators; "Generate" spawns a planet in front of the camera and each change +
// "Regenerate" rebuilds and re-uploads its mesh in place (leak-free via the
// AssetManager). Without RT_ENABLE_IMGUI it is an inert System, so it stays
// registered in every build. It owns the one entity it spawns.
class PlanetLabSystem : public System {
public:
    void render(FrameContext& ctx) override;

private:
    void rebuild(FrameContext& ctx);   // (re)generate + upload; spawn on first call
    void removePlanet(FrameContext& ctx);

    // Shared
    int   mode_ = 0;          // 0 = rocky, 1 = gas giant
    int   seed_ = 1;
    float radius_ = 20.0f;
    int   faceRes_ = 64;
    bool  spin_ = true;

    // Rocky
    int   rockyPreset_ = 0;   // 0 Earth, 1 Mars, 2 Moon
    float seaLevel_ = 0.02f;
    bool  hasOcean_ = true;
    float continentAmp_ = 1.0f;
    float mountainAmp_ = 0.7f;
    float craterAmp_ = 0.05f;
    float relief_ = 0.035f;

    // Gas giant
    int   gasPreset_ = 0;     // 0 Jupiter, 1 Neptune
    float bandFreq_ = 13.0f;
    float flowWarp_ = 0.4f;
    float detailAmp_ = 0.18f;
    int   stormCount_ = 3;

    Entity planet_{};
    bool   haveEntity_ = false;
};

}  // namespace engine

#endif
