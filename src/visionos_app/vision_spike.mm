// visionOS host — the C++ half of the app.
//
// A SwiftUI ImmersiveSpace hands over a cp_layer_renderer_t; everything from
// there is C++/ObjC++. This file boots the engine against it and ticks it once
// per compositor frame. That is all it does.
//
// It notably does NOT contain the CompositorServices frame loop. Drawable
// acquisition, the device anchor, present and end-of-submission all live in
// CompositorSurface inside renderer/metal/metal_renderer.mm, behind the
// PresentationSurface seam — so the visionOS build runs the SAME Metal pass
// graph as the macOS viewer rather than a parallel copy of it. This loop is
// consequently the same shape it would be on any platform.
//
// A device anchor is NOT optional, which cost some time to learn. The original
// spike skipped ARKit on the theory that head-locked output was fine for a
// proof of plumbing; the compositor logged, once per frame:
//
//   "Presenting a drawable without a device anchor.
//    On device this drawable won't be presented."
//
// The simulator drew it anyway, so it looked like it worked while being broken
// on hardware. World tracking now runs inside CompositorSurface and every
// drawable carries a pose queried for its PRESENTATION time.
//
// Current limitations, deliberate and tracked:
//  - MONOSCOPIC. The pass graph runs once and composites into view 0. It is
//    head-tracked and correctly projected, but the eyes do not disagree, so
//    there is no parallax. Per-eye rendering restructures endFrame and is its
//    own change; until then this must not be described as stereo.
//  - The scene sits at the tracking origin established at launch, not on a
//    detected floor plane. Floor anchoring needs PlaneDetectionProvider.

#import "vision_spike.h"

#import <ARKit/ARKit.h>
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <simd/simd.h>

#include "../engine/application.h"
#include "../engine/asset_root.h"
#include "../game/arena_state.h"
#include "../log.h"
#include "../renderer/hosted_window.h"

#include "../engine/xr/xr_backend.h"

#include <atomic>
#include <fstream>
#include <unistd.h>
#include <memory>
#include <mutex>
#include <string>

namespace {

// Spatial-input handoff: the SwiftUI host pushes pinch events from the main
// actor; the engine's XR backend (set while rt_vision_spike_run is alive)
// consumes them. Events arriving while the engine is down are dropped.
std::atomic<engine::XrBackend*> gXrInput{nullptr};

// The level the NEXT rt_vision_spike_run boots (menu-selected). Guarded by a
// mutex because the menu writes from the main actor.
std::mutex gLevelMutex;
std::string gLevelName = "arena";

// World units per real meter for the next boot (menu-set; also scales the
// spatial-event rays so gameplay targeting matches the render composition).
// Default matches the menu's "Life size" (UI 1.0 x baseline 2.0): the levels
// are authored larger than strict metric — see baselineScale in
// VisionSpikeApp.swift, the single place to tune it.
std::atomic<double> gWorldScale{2.0};

// Boots the engine: points asset resolution at the bundle, loads arena.json,
// and pushes the same ArenaState the desktop viewer runs.
//
//
// Returns null if anything fails, and says why. This stage is deliberately
// verbose — it is the first time the engine has run on this platform at all, and
// "nothing appeared" is a useless symptom to debug from.
std::unique_ptr<engine::Application> bootEngine(cp_layer_renderer_t layerRenderer) {
    // A sandboxed bundle has no useful working directory, so every relative
    // asset path has to resolve against the bundle instead (engine/asset_root.h).
    // iOS-style bundles are flat, so resources sit at the bundle root.
    const std::string root = [[NSBundle mainBundle] bundlePath].UTF8String;
    engine::setAssetRoot(root);
    NSLog(@"[vision] asset root: %s", root.c_str());

    // Some levels write relative cache directories (terrain/coast bakes).
    // The sandbox working directory is NOT writable — creating "cache" threw
    // an uncaught filesystem_error and killed the app. Park the CWD in the
    // app's temp dir so every relative write lands somewhere legal.
    chdir(NSTemporaryDirectory().UTF8String);

    std::string levelName;
    {
        std::lock_guard<std::mutex> lock(gLevelMutex);
        levelName = gLevelName;
    }
    std::string levelPath =
        engine::assetPath("assets/levels/" + levelName + ".json");
    if (!std::ifstream(levelPath).good()) {
        NSLog(@"[vision] level '%s' not found — falling back to arena",
              levelName.c_str());
        levelPath = engine::assetPath("assets/levels/arena.json");
    }
    if (!std::ifstream(levelPath).good()) {
        NSLog(@"[vision] FATAL: level not found at %s — is assets/ in the bundle?",
              levelPath.c_str());
        return nullptr;
    }

    auto app = std::make_unique<engine::Application>();

    // HostedWindow, not a platform window: the compositor owns presentation and
    // there is no OS window to speak of. This is the same implementation the Qt
    // editor uses for "the host owns the surface, the engine gets told the size".
    auto window = std::make_unique<engine::HostedWindow>();
    // Nominal until the first drawable reports the real per-eye size; systems
    // that compute an aspect ratio need something non-degenerate before then.
    window->setSizes(1024, 1024, 1024, 1024);

    // The opaque native handle the renderer binds to (ADR-0001). On macOS this
    // is an NSWindow*/NSView*; here it is the compositor's layer renderer, which
    // MetalRenderer::initialize unwraps to build a CompositorSurface and to take
    // the MTLDevice the compositor already chose.
    window->setNativeHandle((__bridge void*)layerRenderer);

    // Settings live in the bundle, which is READ-ONLY. Pointing Application at a
    // path it cannot write would fail on exit, so send it somewhere writable.
    NSString* docs = NSSearchPathForDirectoriesInDomains(
        NSDocumentDirectory, NSUserDomainMask, YES).firstObject;
    const std::string settingsPath =
        std::string(docs.UTF8String) + "/settings.json";

    // Size is nominal — the compositor decides the real per-eye dimensions. It
    // only has to be non-zero so nothing divides by it.
    // Null audio, deliberately. Opening a real device on visionOS deadlocks
    // AURemoteIO unless the app has configured and activated an AVAudioSession
    // first — CoreAudio then aborts the process on its RPC timeout ("Initialize:
    // RPC timeout. Apparently deadlocked."), which is a HANG, so Auto's
    // open-failed fallback never gets a chance to run. Sounds still "play" into
    // the null backend, so gameplay that waits on audio does not stall. Doing
    // the AVAudioSession setup properly is its own piece of work.
    engine::Application::Config cfg{1024, 1024, "Raytracer visionOS", settingsPath};
    cfg.audio = engine::AudioBackendMode::Null;
    if (!app->initialize(cfg, std::move(window))) {
        NSLog(@"[vision] FATAL: Application::initialize failed");
        return nullptr;
    }

    app->renderer().xrWorldScale = static_cast<float>(gWorldScale.load());
    app->settings().setString("cameraMode", "fly");
    // No editor on this platform, so the usual play/edit factory pair collapses
    // to just play; ArenaState's "back to editor" factory is intentionally null.
    app->pushState(std::make_unique<ArenaState>(app->windowRef(), app->renderer(),
                                                levelPath, nullptr));
    app->begin();

    NSLog(@"[vision] engine booted — %zu entities from %s",
          static_cast<size_t>(app->world().entityCount()), levelPath.c_str());
    return app;
}

// TEMPORARY DIAGNOSTIC: the smallest legal CompositorServices frame loop —
// no engine, no assets, just anchor + magenta/depth clear + present. If THIS
// shows magenta, the compositor contract is satisfied and the engine's frame
// structure is what breaks on device; if it stays black, our loop shape
// deviates from the platform contract and content was never the issue.
#define RT_VISION_MINIMAL_PROBE 0

#if RT_VISION_MINIMAL_PROBE
static void runMinimalProbe(cp_layer_renderer_t layerRenderer) {
    id<MTLDevice> device = cp_layer_renderer_get_device(layerRenderer);
    id<MTLCommandQueue> queue = [device newCommandQueue];

    ar_session_t session = nil;
    ar_world_tracking_provider_t tracking = nil;
    if (ar_world_tracking_provider_is_supported()) {
        ar_world_tracking_configuration_t cfg = ar_world_tracking_configuration_create();
        tracking = ar_world_tracking_provider_create(cfg);
        ar_data_providers_t providers = ar_data_providers_create();
        ar_data_providers_add_data_provider(providers, tracking);
        session = ar_session_create();
        ar_session_run(session, providers);
    }

    uint64_t n = 0;
    uint64_t anchoredCount = 0;
    while (cp_layer_renderer_get_state(layerRenderer)
           != cp_layer_renderer_state_invalidated) {
        @autoreleasepool {
            if (cp_layer_renderer_get_state(layerRenderer)
                == cp_layer_renderer_state_paused) {
                cp_layer_renderer_wait_until_running(layerRenderer);
                continue;
            }
            cp_frame_t frame = cp_layer_renderer_query_next_frame(layerRenderer);
            if (!frame) continue;
            cp_frame_timing_t timing = cp_frame_predict_timing(frame);
            if (!timing) continue;
            cp_frame_start_update(frame);
            cp_frame_end_update(frame);
            cp_time_wait_until(cp_frame_timing_get_optimal_input_time(timing));

            // visionOS 26: a frame carries an ARRAY of drawables (built_in =
            // the displays, capture = recording). The deprecated singular
            // cp_frame_query_drawable ran the whole protocol without error but
            // its drawable never reached the displays — every "black view with
            // healthy logs" hour traces back to this. Query the array, render
            // and present EVERY drawable. Empty array = cancelled frame:
            // discard, no submission.
            cp_drawable_array_t drawables = cp_frame_query_drawables(frame);
            size_t drawableCount = cp_drawable_array_get_count(drawables);
            if (drawableCount == 0) continue;
            cp_frame_start_submission(frame);

            id<MTLCommandBuffer> cmd = [queue commandBuffer];
            for (size_t d = 0; d < drawableCount; d++) {
                cp_drawable_t drawable = cp_drawable_array_get_drawable(drawables, d);
                cp_frame_timing_t finalTiming = cp_drawable_get_frame_timing(drawable);
                ar_device_anchor_t anchor = ar_device_anchor_create();
                bool anchored = tracking
                    && ar_world_tracking_provider_query_device_anchor_at_timestamp(
                           tracking,
                           cp_time_to_cf_time_interval(
                               cp_frame_timing_get_presentation_time(finalTiming)),
                           anchor) == ar_device_anchor_query_status_success;
                if (anchored) { cp_drawable_set_device_anchor(drawable, anchor); anchoredCount++; }
                if (n == 0)
                    NSLog(@"[vision] drawable %zu target=%d (0=builtIn 1=capture)",
                          d, (int)cp_drawable_get_target(drawable));

                size_t texCount = cp_drawable_get_texture_count(drawable);
                for (size_t i = 0; i < texCount; i++) {
                    id<MTLTexture> color = cp_drawable_get_color_texture(drawable, i);
                    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
                    rp.colorAttachments[0].texture = color;
                    rp.colorAttachments[0].loadAction = MTLLoadActionClear;
                    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
                    rp.colorAttachments[0].clearColor = MTLClearColorMake(1.0, 0.0, 1.0, 1.0);
                    rp.depthAttachment.texture = cp_drawable_get_depth_texture(drawable, i);
                    rp.depthAttachment.loadAction = MTLLoadActionClear;
                    rp.depthAttachment.storeAction = MTLStoreActionStore;
                    rp.depthAttachment.clearDepth = 0.0;
                    // .layered delivers both eyes as slices of one array texture;
                    // a clear pass touches only slice 0 unless the pass is told to
                    // cover the whole array.
                    if (color.textureType == MTLTextureType2DArray)
                        rp.renderTargetArrayLength = color.arrayLength;
                    id<MTLRenderCommandEncoder> enc =
                        [cmd renderCommandEncoderWithDescriptor:rp];
                    [enc endEncoding];
                }
                cp_drawable_encode_present(drawable, cmd);
            }
            [cmd commit];
            cp_frame_end_submission(frame);
            if (++n % 90 == 0)
                NSLog(@"[vision] MINIMAL probe frame %llu (anchored %llu, drawables %zu)",
                      n, anchoredCount, drawableCount);
        }
    }
    NSLog(@"[vision] MINIMAL probe: layer invalidated, exiting");
}
#endif

}  // namespace

void rt_vision_spike_run(cp_layer_renderer_t layerRenderer) {
#if RT_VISION_MINIMAL_PROBE
    NSLog(@"[vision] MINIMAL PROBE build — engine bypassed, expect solid magenta");
    runMinimalProbe(layerRenderer);
    return;
#endif
    @autoreleasepool {
        NSLog(@"[vision] device: %@", cp_layer_renderer_get_device(layerRenderer).name);

        // The engine runs on this thread, ticked once per compositor frame —
        // the same per-frame decomposition the Qt editor and the Emscripten
        // build already drive (Application::begin/runFrame). CompositorServices
        // is simply a third host; no engine change was needed to invert the loop.
        std::unique_ptr<engine::Application> app = bootEngine(layerRenderer);
        if (!app) {
            NSLog(@"[vision] FATAL: engine failed to boot");
            return;
        }

        // The CompositorServices frame lifecycle (query_next_frame,
        // start/end_update, start/end_submission, drawable acquisition, device
        // anchor, present) is NOT here — it lives in CompositorSurface inside
        // metal_renderer.mm, reached through MetalRenderer::beginFrame/endFrame.
        // That is the point of the PresentationSurface seam: this loop is the
        // same shape it would be on any platform.
        // Publish the XR input funnel now that the engine (and its backend)
        // exist; the Swift host's spatial events start flowing into gameplay.
        gXrInput.store(app->renderer().xrBackend());

        uint64_t frameCounter = 0;
        while (cp_layer_renderer_get_state(layerRenderer) != cp_layer_renderer_state_invalidated) {
            @autoreleasepool {
                app->runFrame();

                // Proof of life at ~1 Hz: a frozen entity count means the sim
                // stalled, which is otherwise indistinguishable from a black eye
                // buffer.
                if (++frameCounter % 90 == 0) {
                    NSLog(@"[vision] frame %llu - %zu entities", frameCounter,
                          static_cast<size_t>(app->world().entityCount()));
                }
            }
        }
        gXrInput.store(nullptr);
        NSLog(@"[vision] layer invalidated - shutting down");
        app->end();
    }
}

void rt_vision_xr_pinch(int phase,
                        double ox, double oy, double oz,
                        double dx, double dy, double dz) {
    engine::XrBackend* backend = gXrInput.load();
    if (!backend) return;
    engine::XrInputEvent event;
    switch (phase) {
        case 0: event.kind = engine::XrInputEvent::Kind::PinchBegan; break;
        case 1: event.kind = engine::XrInputEvent::Kind::PinchMoved; break;
        case 2: event.kind = engine::XrInputEvent::Kind::PinchEnded; break;
        default: event.kind = engine::XrInputEvent::Kind::PinchCancelled; break;
    }
    // Ray origins are real-world meters from the tracking origin; scale them
    // into world units so targeting matches the scaled render composition.
    // Directions are unit-free and pass through.
    const double s = gWorldScale.load();
    event.rayOrigin = engine::Vec3(ox * s, oy * s, oz * s);
    event.rayDir = engine::Vec3(dx, dy, dz);
    backend->pushInput(event);
}

void rt_vision_set_world_scale(double scale) {
    if (scale < 0.05 || scale > 100.0) return;
    gWorldScale.store(scale);
    NSLog(@"[vision] world scale: %.2f", scale);
}

void rt_vision_set_level(const char* name) {
    if (!name || !*name) return;
    std::lock_guard<std::mutex> lock(gLevelMutex);
    gLevelName = name;
    NSLog(@"[vision] next level: %s", name);
}
