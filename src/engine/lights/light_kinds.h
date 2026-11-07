#ifndef RAYTRACER_ENGINE_LIGHTS_LIGHT_KINDS_H
#define RAYTRACER_ENGINE_LIGHTS_LIGHT_KINDS_H

// THE LIGHTS AS DATA (ADR-0362): assets/scripts/lights.lua's kinds and pools. Every real light is a kind; a kind is
// picked from a pool's budget by the one light system (light_picker.h). Read once (ensureLightsLoaded); without the
// file one plain white point kind stands -- there is no C++ copy of the catalogue.

#include "../../rt_math.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

struct LightKind {
    std::string name;
    bool spot = false;
    Vec3 color{1, 1, 1};
    float intensity = 1.0f;
    float candela = 0.0f;               // the real luminous intensity (cd); 0: none given (ADR-0369)
    float range = 25.0f;
    float inner = 0.3f, outer = 0.5f;   // a spot's cone (radians)
    Vec3 offset{0, 0, 0};               // from the emitter's point
    bool dusk = false;                  // rides the dusk ramp
    int pool = -1;                      // into LightBook::pools; -1: every one published is lit
};

struct LightPool {
    std::string name;
    int count = 0;          // at most this many (groups, when `groups`)
    float within = 0.0f;    // metres from the camera; 0: no limit
    bool fade = false;      // fade toward the set's edge
    bool groups = false;    // pick the emitters' groups whole
    // the budget on a renderer of many lights (clustered): `many` set, these replace the three above
    bool hasMany = false;
    int manyCount = 0;
    float manyWithin = 0.0f;
    bool manyFade = false;
};

struct LightBook {
    std::vector<LightKind> kinds;
    std::vector<LightPool> pools;
    float emissionScale = 1.0f;   // the glows' old emission into engine units under the physical sky (ADR-0369)
    float cityGlowNits = 0.0f;    // the city's glow on the sky, low toward downtown (cd/m^2; ADR-0369)
    float moonNits = 2500.0f;     // the full moon's disc above the air (cd/m^2)
    // THE GLOWS (ADR-0369): a glowing surface's class and its real luminance (cd/m^2) under the physical sky
    struct Glow { std::string name; float nits = 0.0f; };
    std::vector<Glow> glows;
    float glowNits(const std::string& name) const;   // 0: no such class
    int kind(const std::string& name) const;   // its index, -1 if none
    int pool(const std::string& name) const;
};

// The book in use: lights.lua's once read, else the one plain kind ("default").
const LightBook& lightBook();
void setLightBook(const LightBook& book);   // the scripting layer, a test
// A kind's index by name, looked up once per call site: `static const int k = lightKindId("headlight");`. -1 (and a
// warning, once) when lights.lua has no such kind: the emitter's lights are then skipped.
int lightKindId(const char* name);
// A glowing surface's real luminance (cd/m^2) by class (lights.lua glows); 0 when lights.lua has none (it keeps its old
// emission scaled by emission_scale).
float lightGlowNits(const char* name);

}  // namespace engine

#endif
