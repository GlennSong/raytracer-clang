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

#ifdef __cplusplus
}
#endif

#endif  // RAYTRACER_VISION_SPIKE_H
