#ifndef RAYTRACER_ENGINE_BOARD_MESHES_H
#define RAYTRACER_ENGINE_BOARD_MESHES_H

// THE BOARDS a body rides (the player's, ADR-0233; the citysim's children skating round the block): blocks from the
// mesh library, built along +z with their wheels on y = 0. One maker each, shared -- never a private copy.

#include "../renderer/renderer.h"

namespace engine {

// A SKATEBOARD: a deck, kicked-up tail and nose, two trucks, four wheels.
RenderMesh buildSkateboard();
// A KICK SCOOTER: a low deck, the stem up from the front, the bar across.
RenderMesh buildScooter();

}  // namespace engine

#endif
