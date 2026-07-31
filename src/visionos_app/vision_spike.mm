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

#include <fstream>
#include <memory>

namespace {

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

    const std::string levelPath = engine::assetPath("assets/levels/arena.json");
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

}  // namespace

void rt_vision_spike_run(cp_layer_renderer_t layerRenderer) {
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
        NSLog(@"[vision] layer invalidated - shutting down");
        app->end();
    }
}
