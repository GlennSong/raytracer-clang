#ifndef RAYTRACER_VISION_SPIKE_H
#define RAYTRACER_VISION_SPIKE_H

#include <CompositorServices/CompositorServices.h>

#ifdef __cplusplus
extern "C" {
#endif

// Drives the CompositorServices render loop until the layer is invalidated.
//
// BLOCKS for the lifetime of the immersive space — call it on a dedicated
// thread, never on the main actor.
//
// `layerRenderer` comes straight from the SwiftUI `CompositorLayer` closure.
// No bridging shim is needed: layer_renderer.h declares the type with
// `CP_OBJECT_DECL(cp_layer_renderer, LayerRenderer)`, which makes
// `cp_layer_renderer_t` an Objective-C object whose Swift name is
// `LayerRenderer` — Swift's value and this parameter are the same object.
void rt_vision_spike_run(cp_layer_renderer_t layerRenderer);

// Spatial input bridge. The SwiftUI host forwards CompositorLayer spatial
// events (gaze/selection ray + pinch phase) here; the engine consumes them
// through its XR backend queue. Safe from any thread; a no-op while the
// engine is not running. phase: 0=began 1=moved 2=ended 3=cancelled.
// Ray origin/direction are in the immersive space's tracking-origin frame.
void rt_vision_xr_pinch(int phase,
                        double ox, double oy, double oz,
                        double dx, double dy, double dz);

// Level selection for the NEXT rt_vision_spike_run (the menu sets it before
// reopening the immersive space). `name` is a level shortname resolved as
// assets/levels/<name>.json inside the bundle. Copies the string.
void rt_vision_set_level(const char* name);

#ifdef __cplusplus
}
#endif

#endif  // RAYTRACER_VISION_SPIKE_H
