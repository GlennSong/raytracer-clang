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

// World scale for the NEXT rt_vision_spike_run: world units traversed per
// real meter (1 = life-size, 5 = feel five times taller). The menu sets it
// before entering a scene.
void rt_vision_set_world_scale(double scale);

// Engine settings applied at the NEXT boot (menu-set): written into the
// engine's Settings store before systems start, so anything data-driven —
// daynight.timeOfDay, daynight.paused, clouds.*, exposure — is reachable
// from the shell without engine changes. Copies the key.
void rt_vision_set_pref_double(const char* key, double value);
void rt_vision_set_pref_bool(const char* key, int value);

#ifdef __cplusplus
}
#endif

#endif  // RAYTRACER_VISION_SPIKE_H
