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

// Tunable shape of a street lamp (the Lua streetfurniture.lamp recipe sets
// these; the C++ city uses the defaults). ADR-0042: parts are parametric so the
// same builder serves the engine and the script.
struct LampParams {
    Real height = 4.6;
    Real poleRadius = 0.07;
    Vec3 headSize{0.5, 0.2, 0.32};
    Vec3 poleColor{0.16, 0.16, 0.18};
    Vec3 headColor{1.0, 0.88, 0.55};   // bright: reads as a lit head
};

// A street lamp (thin pole + a bright head box) built at `base`, vertex-coloured.
void emitStreetLamp(RenderMesh& out, const Vec3& base, const LampParams& p = {});
// The same lamp built once at the origin, for use as an instance prototype
// (ADR-0041): place copies with a translate transform.
RenderMesh streetLamp(const LampParams& p = {});

// Tunable shape of a traffic signal.
struct SignalParams {
    Real poleHeight = 5.6;
    Real armLength = 4.2;
    Real armHeight = 5.3;
    Vec3 poleColor{0.16, 0.16, 0.18};
    Vec3 housingColor{0.07, 0.07, 0.08};
};

// A traffic-signal assembly on a street corner: a pole rising from `base`, a
// horizontal mast arm reaching out over the carriageway along `faceDir`, a
// three-lamp signal head hung from the arm end, and a pedestrian push-button
// box + walk-signal on the pole. `faceDir` (world XZ) points from the corner
// across the approach the signal governs.
void emitTrafficSignal(RenderMesh& out, const Vec3& base, const Vec2& faceDir,
                       const SignalParams& p = {});

// The traffic-signal assembly built once at the origin facing +Z, for use as an
// instance prototype (ADR-0041): place copies with a translate + yaw transform
// instead of baking the geometry at every corner.
RenderMesh trafficSignalProto(const SignalParams& p = {});

}  // namespace engine

#endif
