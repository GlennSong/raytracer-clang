#ifndef RAYTRACER_ENGINE_OCCLUSION_H
#define RAYTRACER_ENGINE_OCCLUSION_H

// OCCLUSION CULLING against a recent frame's depth (Renderer::occlusionDepth). The renderer
// reduces its depth buffer to 16 px tiles, each holding the FARTHEST depth drawn there, and reads it
// back a few frames later. A box whose nearest point is farther than every tile it covers was
// hidden behind what was drawn there (a hill, a building), and is not drawn.
//
// Conservative by construction: anything in doubt is visible -- a box crossing the near plane,
// one covering a huge part of the screen, or a camera that has moved or turned since that depth
// was drawn (usable() says when the snapshot still stands for this view). Reverse-Z: a larger
// depth is nearer.

#include "../rt_math.h"
#include "../renderer/renderer.h"

namespace engine {

// Whether `o` may be used for a camera at `eye` looking along `forward` (unit): close enough to
// where it was drawn that what it hid is still hidden.
bool occlusionUsable(const OcclusionDepth& o, const Vec3& eye, const Vec3& forward);

// True when the world box [lo, hi] was hidden in `o`. `slack` grows the box first (metres): the
// camera's movement since `o` was drawn, so parallax cannot uncover it.
bool occludedBox(const OcclusionDepth& o, const Vec3& lo, const Vec3& hi, double slack = 0.0);

}  // namespace engine

#endif
