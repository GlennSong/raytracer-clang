#ifndef RAYTRACER_ENGINE_MODEL_OVERLAY_H
#define RAYTRACER_ENGINE_MODEL_OVERLAY_H

// WHAT A STAMPED MODEL CARRIES THAT IS NOT DRAWN, DRAWN (#191): lines over each model instance in the world, in the
// pose it is in now, through the engine's debug lines (debug_draw.h):
//
//  - SHAPES: every collision shape of the model where it now is. Green: it stands still. Orange: it is on a node
//    that moves, so its body moves with the node. A mesh or hull shape is shown as the box round its points.
//  - MARKERS: each marker as three short axes (x red, y green, z blue; its +z is the way it faces).
//  - JOINTS: each moving node. A turn: a yellow arrow along what it turns about, a ring round it, a white spoke at
//    each limit and a yellow one where it is now. A slide: the line it runs along, end to end, and where it is now.
//
// The switches are the process's (as the model store is): the control channel's `model_overlay` and the model
// studio's boxes set them, and ModelOverlaySystem draws by them in the editor and in play alike.

#include "components.h"
#include "debug_draw.h"
#include "world.h"

namespace engine {

struct ModelOverlay {
    bool shapes = false;
    bool markers = false;
    bool joints = false;
    bool any() const { return shapes || markers || joints; }
};

// The process's switches.
ModelOverlay& modelOverlay();

// One instance's lines into `out`, by `what`. Nothing when the entity is not a model instance.
void drawModelOverlay(World& world, Entity instance, const ModelOverlay& what, DebugDraw& out);

}  // namespace engine

#endif
