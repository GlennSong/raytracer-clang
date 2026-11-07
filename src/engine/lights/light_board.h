#ifndef RAYTRACER_ENGINE_LIGHTS_LIGHT_BOARD_H
#define RAYTRACER_ENGINE_LIGHTS_LIGHT_BOARD_H

// WHERE THE LIGHTS ARE (ADR-0362). A system that gives off light publishes CANDIDATES -- of a kind (light_kinds.h),
// at a point, aimed -- on its own channel of the board, replacing the channel at its own cadence (a fixed step, a
// frame). It never picks: LightSystem picks from every channel (light_picker.h) and writes the frame's lights here
// for RenderSystem.

#include "../../renderer/renderer.h"   // PointLight, SpotLight
#include "../../rt_math.h"
#include <array>
#include <cstdint>
#include <vector>

namespace engine {

struct LightCandidate {
    int kind = -1;                 // lightBook().kinds; -1: skipped
    Vec3 position;
    Vec3 direction{0, 0, 1};       // a spot's aim
    float scale = 1.0f;            // times the kind's intensity (a flash, a level's beacon reach)
    float range = 0.0f;            // the falloff window, metres; 0: the kind's
    uint32_t group = 0;            // candidates sharing a group are picked whole (a car's two lamps); 0: alone
    bool mine = false;             // the player's own: picked first, never faded
};

// The producers, one channel each (static emitters -- street lamps, station lamps -- are read by LightSystem itself).
enum class LightChannel : int { Vehicles, Tool, Rooms, Cabs, Beacons, Count };

struct LightBoard {
    std::array<std::vector<LightCandidate>, static_cast<std::size_t>(LightChannel::Count)> channels;
    std::vector<LightCandidate>& channel(LightChannel c) { return channels[static_cast<std::size_t>(c)]; }
    const std::vector<LightCandidate>& channel(LightChannel c) const { return channels[static_cast<std::size_t>(c)]; }
    // THE FRAME'S LIGHTS, picked (LightSystem, each frame; RenderSystem adds them to the level's own)
    std::vector<PointLight> points;
    std::vector<SpotLight> spots;
};

}  // namespace engine

#endif
