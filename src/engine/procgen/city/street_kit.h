#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_STREET_KIT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_STREET_KIT_H

#include "polygon.h"
#include "../../../renderer/renderer.h"   // RenderMesh

namespace engine {

// The street / intersection kit (ADR-0038, city Phase 2): a reusable toolkit of
// parametric street-furniture and intersection-decoration emitters. The road
// graph already hands us every junction's position and arm directions, so an
// intersection is a *decoration* problem on a known junction — not a layout
// problem (no WFC needed). These ops dress a junction the city builder gives
// them. All meshes are world-space and vertex-coloured, so they bake into the
// city's shared road/prop meshes like the rest of the furniture.

// Round the convex corners of a footprint with a constant-radius fillet (an arc
// of `segs` chords per corner). A block apron IS the sidewalk, and its corners
// ARE the street-intersection corners, so rounding them gives the rounded curb
// returns a real corner has (a vehicle can't turn a knife-edge). Reflex/very
// shallow/very sharp corners are left as-is; the radius is clamped to the
// adjacent edge lengths so short edges don't self-overlap.
Poly2 roundPolygonCorners(const Poly2& poly, Real radius, int segs = 4);

// A painted stop bar across the approaching lane: a thick white line set just
// inside the crosswalk on an intersection approach, spanning the half of the
// carriageway that approaching traffic uses (drive-on-the-right). `center` sits
// on the road centreline at the bar; `dir` is travel toward the node.
void emitStopBar(RenderMesh& out, const Vec2& center, const Vec2& dir,
                 Real roadW, Real y, const Vec3& col);

// A traffic-signal assembly on a street corner: a pole rising from `base`, a
// horizontal mast arm reaching out over the carriageway along `faceDir`, a
// three-lamp signal head hung from the arm end, and a pedestrian push-button
// box + walk-signal on the pole. `faceDir` (world XZ) points from the corner
// across the approach the signal governs. `grade` is the ground height at base.
void emitTrafficSignal(RenderMesh& out, const Vec3& base, const Vec2& faceDir);

// The traffic-signal assembly built once at the origin facing +Z, for use as an
// instance prototype (ADR-0041): place copies with a translate + yaw transform
// instead of baking the geometry at every corner.
RenderMesh trafficSignalProto();

}  // namespace engine

#endif
