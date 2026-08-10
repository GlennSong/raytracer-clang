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

// Engine settings bridge. Values land in a mutex-guarded store; while the
// engine is RUNNING they are drained on the engine thread at the top of the
// next frame and applied straight through to the renderer (live sliders),
// and at boot the whole store is written into Settings before systems start
// — so the same call works from the menu and from the in-scene settings
// panel. Copies the key.
void rt_vision_set_pref_double(const char* key, double value);
void rt_vision_set_pref_bool(const char* key, int value);

// Current value of a pref as the engine sees it (seeded from the renderer
// after boot, updated by the setters). Falls back when the key was never
// set — e.g. the settings panel opened before any scene has booted.
double rt_vision_get_pref_double(const char* key, double fallback);
int rt_vision_get_pref_bool(const char* key, int fallback);

// Persist the current render settings to the app's writable settings.json
// (Documents). Runs on the engine thread at the next frame boundary; a
// no-op while no scene is running.
void rt_vision_save_settings(void);

// Reset the render knobs (bloom/ssao/ssr/tonemap/grade) to engine defaults,
// live. Does not touch the saved file until the next save.
void rt_vision_reset_render_prefs(void);

#ifdef __cplusplus
}
#endif

#endif  // RAYTRACER_VISION_SPIKE_H
