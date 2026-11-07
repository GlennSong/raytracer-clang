#ifndef RAYTRACER_ENGINE_LIGHTS_LIGHT_PICKER_H
#define RAYTRACER_ENGINE_LIGHTS_LIGHT_PICKER_H

// THE ONE PICKER (ADR-0362): which published lights are real this frame. Per pool (light_kinds.h): the player's own
// first, then the nearest within the pool's reach, up to its count (whole groups when it picks groups); the set fades
// out toward its edge -- the reach, or the farthest light picked when the count binds, the edge that moves as the
// camera does -- so a light joins and leaves without a pop. A kind with no pool is always lit. Pure: the tests hold it.

#include "light_board.h"
#include "light_kinds.h"
#include <vector>

namespace engine {

// `pools`: the book's, as the level has set them. `dusk`: the dusk ramp (0 day .. 1 night) for kinds that ride it.
// Appends to `points` / `spots` in the candidates' order; a light that comes out dark (scale, fade, dusk) is left out.
// `cap` (the renderer's lightCapacity): more picked than that and only the most important stay -- the player's own,
// then by lightImportance -- so a 32-light renderer never cuts the list in an order of its own.
void pickLights(const LightBook& book, const std::vector<LightPool>& pools, const std::vector<LightCandidate>& cands,
                const Vec3& eye, float dusk, std::vector<PointLight>& points, std::vector<SpotLight>& spots,
                int cap = 1 << 30);

// How much a light matters where the camera is: its illuminance there (intensity / d^2, d at least 1 m), zero past
// its reach.
float lightImportance(float intensity, float range, float d);

// The edge fade alone: 1 inside, easing (smoothstep) to 0 across the last quarter of `edge`; `edge` <= 0: 1.
float lightEdgeFade(float d, float edge);

}  // namespace engine

#endif
