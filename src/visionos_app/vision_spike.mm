// visionOS plumbing spike — the C++/Metal half.
//
// Goal: prove the seam, nothing else. A SwiftUI ImmersiveSpace hands us a
// cp_layer_renderer_t; from there everything is C++/ObjC++ — pull the MTLDevice,
// drive the CompositorServices frame loop, render one WORLD-SPACE triangle per
// eye. World-space (not screen-space) on purpose: the eyes must disagree about
// where the triangle projects, which is what actually demonstrates stereo. A
// screen-space triangle would look identical on a monocular pipeline and prove
// nothing.
//
// This is a SPIKE. It deliberately does not touch the engine's Renderer seam,
// load assets, or use ARKit — see src/renderer/metal/AGENTS.md for how the real
// backend is expected to be shaped. Nothing here should be mistaken for the
// finished VisionRenderer.
//
// A device anchor is NOT optional. The first cut of this spike skipped ARKit on
// the theory that a head-locked triangle was good enough to prove plumbing. The
// simulator log disagreed, once per frame:
//
//   "Presenting a drawable without a device anchor.
//    On device this drawable won't be presented."
//
// The simulator still drew it, so it looked like it worked; on real hardware
// nothing would have appeared at all. So world tracking is set up below and the
// anchor is attached to every drawable. That is a requirement of presenting,
// not a feature — which is also why it belongs in the spike rather than being
// deferred to M0.
//
// Known spike limitations (deliberate, not oversights):
//  - The triangle is placed relative to the world origin the tracking session
//    establishes at launch, NOT to a detected floor plane. Real floor anchoring
//    needs PlaneDetectionProvider and is M0's job.
//  - Shader source is embedded rather than read from shaders/. The engine reads
//    its .metal files relative to the working directory, which does not survive
//    an app bundle — that is the asset-root work, tracked separately.

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

// Vertex layout shared with the shader below.
//
// Plain float[3], deliberately NOT simd_float3. simd_float3 is a float4 in
// disguise — 16-byte aligned, 16 bytes wide — while MSL's packed_float3 is 12.
// Declaring these as simd_float3 makes this struct 32 bytes while the shader
// strides 24 through it, so every vertex past the first reads garbage: the
// triangle renders as a huge black wedge with one correct corner. (simd has no
// simd_packed_float3 to reach for — packed variants exist only for 2 and 4 —
// so a C array is the type with the layout guarantee we need.)
//
// The static_asserts make that mismatch a build failure instead of something
// you diagnose off a screenshot.
struct SpikeVertex {
    float position[3];
    float color[3];
};

static_assert(sizeof(SpikeVertex) == 24,
              "SpikeVertex must match `packed_float3 position; packed_float3 color;` "
              "in kSpikeShaderSource — 2 x 12 bytes, no padding");
static_assert(offsetof(SpikeVertex, color) == 12,
              "colour must start at byte 12 to match the shader's packed layout");

// A triangle 1.5 m in front of the origin, half a metre tall. Close enough that
// the per-eye parallax is obvious, far enough to be comfortable to look at.
constexpr SpikeVertex kTriangle[3] = {
    {{ 0.00f,  0.25f, -1.5f}, {1.0f, 0.2f, 0.2f}},
    {{-0.25f, -0.25f, -1.5f}, {0.2f, 1.0f, 0.2f}},
    {{ 0.25f, -0.25f, -1.5f}, {0.2f, 0.2f, 1.0f}},
};

NSString* const kSpikeShaderSource = @R"metal(
#include <metal_stdlib>
using namespace metal;

struct SpikeVertex {
    packed_float3 position;
    packed_float3 color;
};

struct VertexOut {
    float4 position [[position]];
    float3 color;
};

vertex VertexOut spikeVertex(uint vid [[vertex_id]],
                             const device SpikeVertex* verts [[buffer(0)]],
                             constant float4x4& viewProjection [[buffer(1)]]) {
    VertexOut out;
    out.position = viewProjection * float4(float3(verts[vid].position), 1.0);
    out.color = float3(verts[vid].color);
    return out;
}

fragment float4 spikeFragment(VertexOut in [[stage_in]]) {
    return float4(in.color, 1.0);
}
)metal";

// Boots the engine inside the bundle: points the asset root at the bundle,
// loads arena.json, and pushes the same ArenaState the desktop viewer runs.
//
// Returns null if anything fails, and says why. This stage is deliberately
// verbose — it is the first time the engine has run on this platform at all, and
// "nothing appeared" is a useless symptom to debug from.
std::unique_ptr<engine::Application> bootEngine() {
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

struct SpikeState {
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    id<MTLRenderPipelineState> pipeline = nil;
    id<MTLDepthStencilState> depthState = nil;
    id<MTLBuffer> vertexBuffer = nil;

    // World tracking, the source of the device (head) pose.
    ar_session_t arSession = nullptr;
    ar_world_tracking_provider_t worldTracking = nullptr;
};

// Starts an ARKit world-tracking session. Returns false if the platform cannot
// provide one — in which case there is no point rendering, since the compositor
// refuses to present a drawable that carries no device anchor.
bool startWorldTracking(SpikeState& state) {
    if (!ar_world_tracking_provider_is_supported()) {
        NSLog(@"[vision-spike] world tracking unsupported on this platform");
        return false;
    }

    ar_world_tracking_configuration_t config = ar_world_tracking_configuration_create();
    state.worldTracking = ar_world_tracking_provider_create(config);

    ar_data_providers_t providers = ar_data_providers_create();
    ar_data_providers_add_data_provider(providers, state.worldTracking);

    state.arSession = ar_session_create();
    ar_session_run(state.arSession, providers);

    NSLog(@"[vision-spike] world tracking session started");
    return true;
}

// Queries the head pose for the moment this frame will actually be shown, and
// attaches it to the drawable.
//
// The timestamp matters: querying "now" would anchor the frame to a pose that is
// one frame stale by the time the compositor displays it, which reads as
// swimming. Presentation time is what the pose must be valid for.
//
// Returns false when tracking has no pose to give (startup, or tracking loss),
// which is a skip-this-frame condition rather than an error.
bool attachDeviceAnchor(SpikeState& state, cp_drawable_t drawable) {
    cp_frame_timing_t timing = cp_drawable_get_frame_timing(drawable);
    if (!timing) return false;

    const CFTimeInterval presentationTime =
        cp_time_to_cf_time_interval(cp_frame_timing_get_presentation_time(timing));

    ar_device_anchor_t anchor = ar_device_anchor_create();
    const ar_device_anchor_query_status_t status =
        ar_world_tracking_provider_query_device_anchor_at_timestamp(
            state.worldTracking, presentationTime, anchor);

    if (status != ar_device_anchor_query_status_success) return false;

    cp_drawable_set_device_anchor(drawable, anchor);
    return true;
}

// Builds the pipeline against the drawable's actual formats. We cannot know
// these until the first drawable arrives — the layer configuration chooses them
// — so pipeline creation is deferred rather than guessed at startup.
bool buildPipeline(SpikeState& state, cp_drawable_t drawable) {
    NSError* error = nil;
    id<MTLLibrary> library = [state.device newLibraryWithSource:kSpikeShaderSource
                                                        options:nil
                                                          error:&error];
    if (!library) {
        NSLog(@"[vision-spike] shader compile failed: %@", error);
        return false;
    }

    id<MTLTexture> color = cp_drawable_get_color_texture(drawable, 0);
    id<MTLTexture> depth = cp_drawable_get_depth_texture(drawable, 0);

    MTLRenderPipelineDescriptor* desc = [[MTLRenderPipelineDescriptor alloc] init];
    desc.vertexFunction = [library newFunctionWithName:@"spikeVertex"];
    desc.fragmentFunction = [library newFunctionWithName:@"spikeFragment"];
    desc.colorAttachments[0].pixelFormat = color.pixelFormat;
    if (depth) desc.depthAttachmentPixelFormat = depth.pixelFormat;
    // A layered drawable presents both eyes as slices of one texture array, so
    // the pipeline has to be told to expect more than one layer.
    if (color.textureType == MTLTextureType2DArray) {
        desc.inputPrimitiveTopology = MTLPrimitiveTopologyClassTriangle;
    }

    state.pipeline = [state.device newRenderPipelineStateWithDescriptor:desc error:&error];
    if (!state.pipeline) {
        NSLog(@"[vision-spike] pipeline creation failed: %@", error);
        return false;
    }

    // CompositorServices supports reverse-Z only (near = 1, far = 0), which
    // happens to match the engine's existing convention: clear to 0, test
    // Greater. See drawable.h cp_drawable_set_depth_range.
    MTLDepthStencilDescriptor* ds = [[MTLDepthStencilDescriptor alloc] init];
    ds.depthCompareFunction = MTLCompareFunctionGreater;
    ds.depthWriteEnabled = YES;
    state.depthState = [state.device newDepthStencilStateWithDescriptor:ds];

    return true;
}

// Renders every view the drawable exposes. Written to be layout-agnostic: the
// view's texture map says which texture, which array slice and which viewport
// this eye occupies, so the same code path serves `dedicated` (two textures),
// `shared` (one wide texture, two viewports) and `layered` (one array, two
// slices) without branching on the layout enum.
void renderDrawable(SpikeState& state, cp_drawable_t drawable,
                    id<MTLCommandBuffer> commandBuffer) {
    const size_t viewCount = cp_drawable_get_view_count(drawable);

    // The device's pose in world space, set by attachDeviceAnchor before we got
    // here. Falling back to identity would silently render a head-locked scene,
    // so treat a missing anchor as the caller's bug rather than absorbing it.
    ar_device_anchor_t anchor = cp_drawable_get_device_anchor(drawable);
    const simd_float4x4 deviceTransform =
        ar_device_anchor_get_origin_from_anchor_transform(anchor);

    for (size_t viewIndex = 0; viewIndex < viewCount; ++viewIndex) {
        cp_view_t view = cp_drawable_get_view(drawable, viewIndex);
        cp_view_texture_map_t map = cp_view_get_view_texture_map(view);

        const size_t textureIndex = cp_view_texture_map_get_texture_index(map);
        const size_t sliceIndex = cp_view_texture_map_get_slice_index(map);
        MTLViewport viewport = cp_view_texture_map_get_viewport(map);

        id<MTLTexture> color = cp_drawable_get_color_texture(drawable, textureIndex);
        id<MTLTexture> depth = cp_drawable_get_depth_texture(drawable, textureIndex);

        MTLRenderPassDescriptor* pass = [MTLRenderPassDescriptor renderPassDescriptor];
        pass.colorAttachments[0].texture = color;
        pass.colorAttachments[0].slice = sliceIndex;
        pass.colorAttachments[0].loadAction = MTLLoadActionClear;
        pass.colorAttachments[0].storeAction = MTLStoreActionStore;
        pass.colorAttachments[0].clearColor = MTLClearColorMake(0.05, 0.06, 0.10, 1.0);
        if (depth) {
            pass.depthAttachment.texture = depth;
            pass.depthAttachment.slice = sliceIndex;
            pass.depthAttachment.loadAction = MTLLoadActionClear;
            pass.depthAttachment.storeAction = MTLStoreActionStore;
            pass.depthAttachment.clearDepth = 0.0;   // reverse-Z far
        }

        id<MTLRenderCommandEncoder> encoder =
            [commandBuffer renderCommandEncoderWithDescriptor:pass];
        [encoder setViewport:viewport];
        [encoder setRenderPipelineState:state.pipeline];
        [encoder setDepthStencilState:state.depthState];

        // cp_view_get_transform is the eye's pose RELATIVE TO the device, so the
        // world-space eye pose is device * view, and the view matrix is its
        // inverse. Getting this composition backwards is the classic way to end
        // up with a stereo image that looks plausible but has the eyes swapped.
        const simd_float4x4 eyeToWorld =
            simd_mul(deviceTransform, cp_view_get_transform(view));
        const simd_float4x4 viewMatrix = simd_inverse(eyeToWorld);

        // Ask the compositor for the projection rather than building one from
        // tangents: the per-eye frustum is asymmetric and off-axis, and
        // cp_view_get_tangents is deprecated as of visionOS 2.0 in favour of
        // this. right_up_back = standard Metal NDC (+Y up).
        const simd_float4x4 projection = cp_drawable_compute_projection(
            drawable, cp_axis_direction_convention_right_up_back, viewIndex);

        const simd_float4x4 viewProjection = simd_mul(projection, viewMatrix);

        // One-shot transform dump. The triangle came up far larger than a 0.5 m
        // object at 1.5 m should look, and guessing at matrix conventions is how
        // you end up with a stack of compensating transposes — so print the
        // actual numbers once and read them.
        static bool dumped = false;
        if (!dumped && viewIndex == 0) {
            dumped = true;
            auto row = [](simd_float4x4 m, int r) {
                return [NSString stringWithFormat:@"[%7.3f %7.3f %7.3f %7.3f]",
                        m.columns[0][r], m.columns[1][r], m.columns[2][r], m.columns[3][r]];
            };
            NSLog(@"[vision-spike] device xform:\n%@\n%@\n%@\n%@",
                  row(deviceTransform,0), row(deviceTransform,1),
                  row(deviceTransform,2), row(deviceTransform,3));
            NSLog(@"[vision-spike] view matrix:\n%@\n%@\n%@\n%@",
                  row(viewMatrix,0), row(viewMatrix,1), row(viewMatrix,2), row(viewMatrix,3));
            NSLog(@"[vision-spike] projection:\n%@\n%@\n%@\n%@",
                  row(projection,0), row(projection,1), row(projection,2), row(projection,3));
            const simd_float4 v0 = simd_mul(viewProjection,
                                            simd_make_float4(0.0f, 0.25f, -1.5f, 1.0f));
            NSLog(@"[vision-spike] apex clip=(%.3f %.3f %.3f %.3f) ndc=(%.3f %.3f %.3f)",
                  v0.x, v0.y, v0.z, v0.w, v0.x/v0.w, v0.y/v0.w, v0.z/v0.w);
            MTLViewport vp = viewport;
            NSLog(@"[vision-spike] viewport %.0fx%.0f  depth[%.2f..%.2f]",
                  vp.width, vp.height, vp.znear, vp.zfar);
        }

        [encoder setVertexBuffer:state.vertexBuffer offset:0 atIndex:0];
        [encoder setVertexBytes:&viewProjection length:sizeof(viewProjection) atIndex:1];
        [encoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [encoder endEncoding];
    }
}

}  // namespace

void rt_vision_spike_run(cp_layer_renderer_t layerRenderer) {
    @autoreleasepool {
        SpikeState state;
        // The compositor owns the device — we do not create one. This is the
        // whole reason the seam works: Swift hands over the layer renderer and
        // every Metal object below hangs off the device it already chose.
        state.device = cp_layer_renderer_get_device(layerRenderer);
        state.queue = [state.device newCommandQueue];
        state.vertexBuffer = [state.device newBufferWithBytes:kTriangle
                                                       length:sizeof(kTriangle)
                                                      options:MTLResourceStorageModeShared];
        NSLog(@"[vision-spike] device: %@", state.device.name);

        if (!startWorldTracking(state)) return;

        // The engine runs on this thread, ticked once per compositor frame. This
        // is the same per-frame decomposition the Qt editor and the Emscripten
        // build already drive (Application::begin/runFrame), so CompositorServices
        // is simply a third host — no engine change was needed to invert the loop.
        std::unique_ptr<engine::Application> app = bootEngine();
        if (!app) {
            NSLog(@"[vision] engine failed to boot — rendering the spike triangle only");
        }

        bool pipelineReady = false;
        bool loggedFirstPresent = false;
        uint64_t frameCounter = 0;

        while (true) {
            @autoreleasepool {
                switch (cp_layer_renderer_get_state(layerRenderer)) {
                    case cp_layer_renderer_state_paused:
                        // Blocks until the user brings the space back.
                        cp_layer_renderer_wait_until_running(layerRenderer);
                        continue;
                    case cp_layer_renderer_state_invalidated:
                        NSLog(@"[vision-spike] layer invalidated — exiting loop");
                        return;
                    case cp_layer_renderer_state_running:
                        break;
                }

                cp_frame_t frame = cp_layer_renderer_query_next_frame(layerRenderer);
                if (!frame) continue;

                cp_frame_timing_t timing = cp_frame_predict_timing(frame);
                if (!timing) continue;

                cp_frame_start_update(frame);
                if (app) {
                    // One full engine frame: events, update, fixed physics steps,
                    // and its own render (NullRenderer for now, so it queues draw
                    // calls and discards them — the geometry is real, the output
                    // is not yet).
                    app->runFrame();

                    // Proof of life at ~1 Hz: entity count moves as the level
                    // settles, and a frozen number means the sim has stalled.
                    if (++frameCounter % 90 == 0) {
                        NSLog(@"[vision] frame %llu — %zu entities",
                              frameCounter,
                              static_cast<size_t>(app->world().entityCount()));
                    }
                }
                cp_frame_end_update(frame);

                // Sleep until the compositor's optimal input latch point, so the
                // pose we render against is as late — and therefore as accurate
                // — as possible.
                cp_time_wait_until(cp_frame_timing_get_optimal_input_time(timing));

                cp_frame_start_submission(frame);
                cp_drawable_t drawable = cp_frame_query_drawable(frame);
                if (!drawable) {
                    cp_frame_end_submission(frame);
                    continue;
                }

                if (!pipelineReady) {
                    if (!buildPipeline(state, drawable)) {
                        cp_frame_end_submission(frame);
                        return;   // unrecoverable; don't spin on a broken pipeline
                    }
                    pipelineReady = true;
                }

                // Without this the compositor drops the frame on real hardware.
                if (!attachDeviceAnchor(state, drawable)) {
                    cp_frame_end_submission(frame);
                    continue;   // no pose yet — skip rather than draw a wrong one
                }

                if (!loggedFirstPresent) {
                    NSLog(@"[vision-spike] first anchored present: %zu views, layout ok",
                          cp_drawable_get_view_count(drawable));
                    loggedFirstPresent = true;
                }

                id<MTLCommandBuffer> commandBuffer = [state.queue commandBuffer];
                renderDrawable(state, drawable, commandBuffer);
                cp_drawable_encode_present(drawable, commandBuffer);
                [commandBuffer commit];

                cp_frame_end_submission(frame);
            }
        }
    }
}
