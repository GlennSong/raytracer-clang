#ifndef RAYTRACER_ENGINE_SYSTEMS_BEACON_LIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_BEACON_LIGHT_SYSTEM_H

#include "../system.h"
#include "../asset_manager.h"
#include "../procgen/city/building_records.h"
#include <cstddef>
#include <vector>

namespace engine {

// BEACON LIGHTS, the far and near tiers (skyscrapers v2 M4; Glenn's LOD ask,
// 2026-09-14). The grammar leaves a "beacon" attach point per aviation lamp
// and the loader records them on the building records; this system runs
// two things off that list every frame:
//
//   FAR   one InstanceGroup of camera-facing quads (one draw for the whole
//         city), each with a baked radial glow texture whose alpha shapes
//         the sprite (RenderMaterial::FLAG_ALPHA_FROM_MAP), sized to grow
//         gently with distance so a lamp on a 200 m tower still reads from
//         across the river. Each sprite's flash is folded into its scale
//         (an instance group has one opacity), and it fades OUT inside
//         NEAR_FADE1 m, where the grown bulb and haze spheres take over
//         (their fade lives in DayNightSystem::applyBeaconBlink).
//   NEAR  the nearest MAX_LIGHTS lamps within LIGHT_M become red point
//         lights, flashing with their sprite, so the roof, the parapet and
//         the HVAC casings pick up the red. Staged into
//         lighting.beaconPoints; RenderSystem merges them under the shader's
//         light cap.
//
// The flash phase comes from beaconCellPhase on the lamp's 24 m cell, the
// same hash the loader gives the grown beacon chunks, so sprite, spheres,
// housing and light all blink together. The clock and the night exposure
// adaptation are DayNightSystem's, published on the lighting state.
class BeaconLightSystem : public System {
public:
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    static constexpr Real SPRITE_FAR = 1800.0;   // sprites beyond this are skipped
    static constexpr int GLOW_TEX = 64;          // the baked glow texture's side
    // The tier distances come from the level's citysim block (CitySimConfig:
    // lightSpriteIn, lightSphereOut, lightRadius, lightRange, lightCount);
    // these are the defaults when a level has none.
    static constexpr Real DEFAULT_SPRITE_IN = 20.0;
    static constexpr Real DEFAULT_LIGHT_M = 60.0;
    static constexpr Real DEFAULT_LIGHT_RANGE = 10.0;
    static constexpr int DEFAULT_LIGHT_COUNT = 6;

    std::size_t lampCount() const { return lamps_.size(); }
    std::size_t streetLampCount() const { return streetLamps_.size(); }

private:
    struct Lamp {
        Vec3 pos;
        float period = 2.0f;
        float phase = 0.0f;
    };
    void gather(const CityBuildings& cb);
    void ensureGroup(FrameContext& ctx);
    // The STREET LAMPS' far tier rides here too: past the level's
    // lampGlowDistance (where their glow shells and poles stop drawing) every
    // bulb is a warm, steady sprite in a second instance group, out to
    // SPRITE_FAR — the grid of lights to the horizon a night city has.
    void ensureLampGroup(FrameContext& ctx);
    std::vector<Vec3> streetLamps_;
    std::size_t lampHeadsSeen_ = static_cast<std::size_t>(-1);
    Entity lampGroup_;
    bool haveLampGroup_ = false;

    std::vector<Lamp> lamps_;
    std::size_t recordsSeen_ = static_cast<std::size_t>(-1);
    Entity group_;
    bool haveGroup_ = false;
    MeshHandle quad_;
    TextureHandle glow_;
    std::vector<std::pair<Real, std::size_t>> order_;   // (distance², lamp) scratch
};

}  // namespace engine

#endif
