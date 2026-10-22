#ifndef RAYTRACER_ENGINE_PROCGEN_GROUND_CLUTTER_H
#define RAYTRACER_ENGINE_PROCGEN_GROUND_CLUTTER_H

// GROUND CLUTTER (ADR-0212; Glenn's terrain plan, step 5): the small things that break up the ground near the player --
// stones on bare earth, scree and rock, sticks and twigs under the trees. They are planted like grass (GrassSystem
// streams them round the camera from a "grassLayers" entry with "clutter"), one small mesh a clump, from the shape kit:
//   stones  a few angular stones, a hand to a fist across, bedded a third into the ground (stylized_rock's pebbles)
//   twigs   a fallen stick or two lying on the ground, crooked, with a side shoot -- bark in the vertex colour
// A clutter mesh stands on y = 0 with its origin at its middle; it is not tilted to the slope, so it is bedded
// deep enough not to float on the slopes clutter lies on.

#include "../../renderer/renderer.h"   // RenderMesh

#include <cstdint>
#include <string>

namespace engine {

enum class ClutterKind : uint8_t { Stones, Twigs };
bool clutterKindFromName(const std::string& n, ClutterKind& out);

RenderMesh groundClutter(ClutterKind kind, uint32_t seed, double size);

}  // namespace engine

#endif
