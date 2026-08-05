#ifdef __APPLE__

#import "metal_renderer.h"
#import <TargetConditionals.h>
#import <Metal/Metal.h>
#import <QuartzCore/CABase.h>   // CACurrentMediaTime (wind sway clock)
#import <simd/simd.h>

// Presentation differs by platform, and so do the frameworks that carry it.
// These guards are for AVAILABILITY only — CAMetalLayer/AppKit simply do not
// exist in the visionOS SDK, and CompositorServices does not exist on macOS.
// Behavioural differences stay behind PresentationSurface (AGENTS.md, Platform
// Abstraction); nothing below threads a platform conditional through the pass
// graph.
#if TARGET_OS_OSX
#import <QuartzCore/CAMetalLayer.h>
#import <AppKit/AppKit.h>
#elif TARGET_OS_VISION
#import <CompositorServices/CompositorServices.h>
#import <ARKit/ARKit.h>
#endif
#include "../../slot_map.h"
#include "../../engine/asset_root.h"
#include "../../engine/xr/xr_backend.h"
#include "../cube_faces.h"
#include <mutex>
#include <vector>
#include <algorithm>
#include <cstdlib>

// Headless frame capture (declaration only — the implementation is compiled
// into model_importer.cpp alongside the glTF importer's stb usage).
#include "../../../third_party/tinygltf/stb_image_write.h"

// GPU-shared struct layouts (CameraUniforms, LightUniforms, …) — one header
// included by both this file and the MSL source (ADR-0017 Phase 0).
#include "../../../shaders/metal/shader_types.h"

#ifdef RT_ENABLE_IMGUI
#include "imgui.h"
#include "backends/imgui_impl_metal.h"
#endif

namespace engine {

struct GPUMesh {
    id<MTLBuffer> vertexBuffer;
    id<MTLBuffer> indexBuffer;
    uint32_t indexCount;
    int materialIndex;
    BoundingSphere bounds;
};

static constexpr uint32_t MAX_INSTANCES = 4096;

// Shadow casters for every cascade share one buffer encoded into a single
// command buffer, so their instance data must all coexist until commit (a
// cascade can't reuse the region an earlier cascade's draws still reference) —
// size it for the summed caster count across all cascades, not just one.
static constexpr uint32_t SHADOW_MAX_INSTANCES = 16384;

// Foliage (alpha-cut) instances are rendered twice in the main pass — depth
// prepass then lit — from one buffer filled once, so both reads see the same
// transforms. Sized for the frustum-visible foliage instances.
static constexpr uint32_t FOLIAGE_MAX_INSTANCES = 8192;

// Dynamic per-frame GPU buffers (instance transforms) are ring-buffered this
// many deep so the CPU writing frame N+1 never stomps data the GPU is still
// reading for frame N. nextDrawable caps the CPU at ~maximumDrawableCount (3)
// frames ahead, so a 3-deep ring is always free by the time it's reused — no
// fence needed.
static constexpr int MAX_FRAMES_IN_FLIGHT = 3;

// Where the composited frame goes, and how it reaches the display.
//
// This is the ONLY part of the Metal backend that knows what it is presenting
// to. The pass graph does not: the scene, shadow and post passes all render to
// textures this backend owns, and the drawable is touched in exactly four
// places — the composite target, the lens-warp target, the frame-dump source,
// and present. Everything else is surface-agnostic, which is why visionOS can
// reuse this renderer instead of growing a parallel one (AGENTS.md: "Use the
// technology you already have").
//
// macOS presents through a CAMetalLayer. visionOS has no layer and no window at
// all — CompositorServices hands out drawables — so it implements this instead.
struct PresentationSurface {
    virtual ~PresentationSurface() = default;

    // Acquire this frame's target. False means "skip the frame": a layer can
    // fail to vend a drawable, and a compositor can be paused.
    virtual bool acquire() = 0;

    // Where the composite pass writes. Valid only between acquire() and
    // present(); nil if acquire() failed.
    virtual id<MTLTexture> colorTarget() const = 0;

    // Hand the finished frame to the display. Runs BEFORE the command buffer is
    // committed, so implementations encode rather than submit.
    // sceneDepth is the frame's rendered depth buffer (reverse-Z). Surfaces
    // that reproject (CompositorServices) hand it to the compositor; window
    // surfaces ignore it.
    virtual void present(id<MTLCommandBuffer> commandBuffer,
                         id<MTLTexture> sceneDepth) = 0;

    // The target's true pixel dimensions. The engine cannot infer these: on
    // visionOS there is no window to measure, and the compositor picks the
    // per-eye size itself. Returns false if it is not yet knowable.
    virtual bool drawableSize(int& width, int& height) const = 0;

    // The colour target's pixel format. Must be known at initialize() time,
    // BEFORE any drawable exists, because the composite and lens pipelines are
    // built against it — a pipeline whose attachment format disagrees with the
    // texture it renders into is invalid in Metal.
    virtual MTLPixelFormat colorPixelFormat() const = 0;

    // Whether the display transform is applied AFTER the shader — by the
    // hardware on write (sRGB formats) or by the presentation stack reading
    // LINEAR values (float formats; the visionOS compositor applies the
    // display transform itself). Either way the composite pass must NOT encode
    // in-shader, so the transform is applied exactly once. Shader-encoding
    // into a float target reads as washed out: the compositor treats the
    // already-encoded values as linear.
    bool targetEncodesSRGB() const {
        const MTLPixelFormat f = colorPixelFormat();
        return f == MTLPixelFormatBGRA8Unorm_sRGB || f == MTLPixelFormatRGBA8Unorm_sRGB
            || f == MTLPixelFormatRGBA16Float;
    }

    // XR head-tracked camera. When the surface tracks a headset, this returns
    // the eye pose in WORLD space (tracking pose composed with the surface's
    // locomotion base) and the compositor's own projection for the composited
    // view — the engine camera then follows the user's head instead of the
    // game camera. baseHint seeds the locomotion base the first time tracking
    // is live (the game camera's position, dropped to the floor, so the user
    // stands where the level intended). Surfaces that don't track return
    // false and the engine camera is used as-is.
    virtual bool xrView(simd_float3 /*baseHint*/, simd_float4x4& /*worldFromEye*/,
                        simd_float4x4& /*projection*/) {
        return false;
    }

    // True once xrView has a live tracked pose. Lets the renderer re-derive
    // the camera each frame from the fresh pose without disturbing surfaces
    // (macOS) that don't track.
    virtual bool xrTracking() const { return false; }

    // Number of XR views to render this frame. 0 = not an XR surface (or
    // tracking not live): endFrame runs its single mono pass, untouched.
    virtual int xrViewCount() const { return 0; }

    // Camera for view v: eye pose in world space (tracking pose composed with
    // the locomotion base) and the compositor's projection for that view.
    virtual bool xrViewCamera(int /*v*/, simd_float3 /*baseHint*/,
                              simd_float4x4& /*worldFromEye*/,
                              simd_float4x4& /*projection*/) { return false; }

    // Where view v's content goes: color/depth textures, array slice, and the
    // viewport within them. The texture map is the contract — never assume
    // view v lands in slice v or covers the full texture. (__strong on the
    // out-params: ARC defaults id& parameters to __autoreleasing, which
    // cannot bind to strong locals at the call site.)
    virtual bool xrViewTarget(int /*v*/, id<MTLTexture> __strong& /*color*/,
                              id<MTLTexture> __strong& /*depth*/,
                              NSUInteger& /*slice*/,
                              MTLViewport& /*viewport*/) { return false; }

    // World units per real meter for head-tracked composition (see
    // Renderer::xrWorldScale). No-op for non-tracking surfaces.
    virtual void setXrWorldScale(float /*scale*/) {}

    // Runs AFTER the command buffer is committed, for surfaces that bracket a
    // frame rather than just handing over a texture.
    //
    // A CAMetalLayer has nothing to do here — presentDrawable: was simply
    // another encoded command, and once committed the layer is done. Nothing on
    // macOS overrides this.
    //
    // CompositorServices does bracket the frame, with start_submission /
    // end_submission around the GPU work. Those are not bookkeeping: per
    // frame.h, "Compositor uses the time difference to improve its predictions
    // for when to start the frame submission process." Closing the frame inside
    // present() instead would time the interval EXCLUDING commit, so the
    // compositor would mis-schedule the next frame — a latency and dropped-frame
    // problem that looks fine in the simulator and shows up on device.
    //
    // Hence a hook rather than a preprocessor branch: platform differences
    // belong behind this seam, not threaded through endFrame (AGENTS.md,
    // Platform Abstraction).
    virtual void frameSubmitted() {}

    // Backing-store size changed (window resize). Compositor-driven surfaces
    // choose their own size, so this is a no-op there.
    virtual void resize(int /*width*/, int /*height*/) {}
};

#if TARGET_OS_OSX
// The desktop surface: a CAMetalLayer attached to the host's NSView.
struct LayerSurface final : PresentationSurface {
    CAMetalLayer* layer = nil;
    NSWindow* window = nil;              // nil when a host owns the window
    id<CAMetalDrawable> drawable = nil;

    bool acquire() override {
        drawable = [layer nextDrawable];
        return drawable != nil;
    }
    id<MTLTexture> colorTarget() const override { return drawable.texture; }
    void present(id<MTLCommandBuffer> commandBuffer, id<MTLTexture>) override {
        if (drawable) [commandBuffer presentDrawable:drawable];
        drawable = nil;
    }
    void resize(int width, int height) override {
        if (window)   // hosted mode: keep the scale set at initialize
            layer.contentsScale = window.backingScaleFactor;
        layer.drawableSize = CGSizeMake(width, height);
    }
    MTLPixelFormat colorPixelFormat() const override { return layer.pixelFormat; }
    bool drawableSize(int& width, int& height) const override {
        width  = static_cast<int>(layer.drawableSize.width);
        height = static_cast<int>(layer.drawableSize.height);
        return width > 0 && height > 0;
    }
};
#endif  // TARGET_OS_OSX

#if TARGET_OS_VISION
// The immersive surface: CompositorServices vends drawables; there is no layer,
// no window, and no swapchain we own.
//
// MONOSCOPIC FOR NOW. A drawable exposes one view per eye, but this composites
// the single rendered image into view 0 only. It is head-tracked and correctly
// projected, but the eyes do not disagree, so there is no parallax — it reads as
// a flat image floating in space. Real per-eye rendering means running the pass
// graph per view, which restructures endFrame, and that is its own change.
// Calling this "stereo" before then would be exactly the smoke and mirrors the
// Engineering Ethos rules out.
struct CompositorSurface final : PresentationSurface {
    cp_layer_renderer_t layerRenderer = nullptr;
    ar_session_t arSession = nullptr;   // owns the session: under ARC a local
                                        // would be released on return, which
                                        // STOPS the provider — device anchors
                                        // then fail and no frame presents
    ar_world_tracking_provider_t worldTracking = nullptr;
    ar_hand_tracking_provider_t handTracking = nullptr;

    cp_frame_t frame = nullptr;
    cp_drawable_array_t drawables = nullptr;  // all targets for this frame
    cp_drawable_t drawable = nullptr;         // the built_in (display) drawable

    // Starts world tracking. Without a device anchor on every drawable the
    // compositor refuses to present on real hardware ("Presenting a drawable
    // without a device anchor. On device this drawable won't be presented.") —
    // the simulator draws it anyway, which makes this an easy thing to get
    // wrong and only discover on the headset.
    bool startTracking() {
        if (!ar_world_tracking_provider_is_supported()) return false;
        ar_world_tracking_configuration_t config = ar_world_tracking_configuration_create();
        worldTracking = ar_world_tracking_provider_create(config);
        ar_data_providers_t providers = ar_data_providers_create();
        ar_data_providers_add_data_provider(providers, worldTracking);
        // Hand skeletons (27 joints per hand). The first run prompts the user
        // for permission; until granted the anchors just report untracked —
        // world tracking is unaffected either way.
        if (ar_hand_tracking_provider_is_supported()) {
            handTracking = ar_hand_tracking_provider_create(
                ar_hand_tracking_configuration_create());
            ar_data_providers_add_data_provider(providers, handTracking);
            NSLog(@"[xr] hand tracking provider added");
        }
        arSession = ar_session_create();
        // Name the failure instead of guessing at it: if the provider never
        // reaches running, every anchor query fails and the device presents
        // nothing — the handler's error is the only place the OS says why.
        ar_session_set_data_provider_state_change_handler(
            arSession, dispatch_get_global_queue(QOS_CLASS_DEFAULT, 0),
            ^(ar_data_providers_t, ar_data_provider_state_t newState,
              ar_error_t error, ar_data_provider_t) {
                NSLog(@"[vision] AR provider state -> %d%s", (int)newState,
                      newState == ar_data_provider_state_running ? " (running)" : "");
                if (error) {
                    CFErrorRef cf = ar_error_copy_cf_error(error);
                    NSLog(@"[vision] AR provider ERROR: %@", (__bridge NSError*)cf);
                    if (cf) CFRelease(cf);
                }
            });
        ar_session_run(arSession, providers);
        return true;
    }

    bool acquire() override {
        switch (cp_layer_renderer_get_state(layerRenderer)) {
            case cp_layer_renderer_state_paused:
                cp_layer_renderer_wait_until_running(layerRenderer);
                return false;
            case cp_layer_renderer_state_invalidated:
                return false;
            case cp_layer_renderer_state_running:
                break;
        }

        frame = cp_layer_renderer_query_next_frame(layerRenderer);
        if (!frame) return false;

        cp_frame_timing_t timing = cp_frame_predict_timing(frame);
        if (!timing) { frame = nullptr; return false; }

        cp_frame_start_update(frame);
        cp_frame_end_update(frame);

        // Latch input as late as the compositor allows, so the pose we render
        // against is the freshest one available.
        cp_time_wait_until(cp_frame_timing_get_optimal_input_time(timing));

        // visionOS 26 drawable contract: a frame carries an ARRAY of drawables
        // (built_in = the displays, capture = recording). The pre-26 singular
        // cp_frame_query_drawable is deprecated and its drawable never reaches
        // the displays on the 26 runtime — the whole protocol runs cleanly and
        // the view stays black, which cost a full night to trace. Query the
        // array BEFORE start_submission (the 26 ordering); an empty array is a
        // cancelled frame that must be discarded without submission.
        drawables = cp_frame_query_drawables(frame);
        if (cp_drawable_array_get_count(drawables) == 0) {
            drawables = nullptr;
            frame = nullptr;
            return false;
        }
        cp_frame_start_submission(frame);

        // The pass graph composites into the built_in (display) drawable;
        // present() copies that image into any other targets.
        size_t count = cp_drawable_array_get_count(drawables);
        drawable = cp_drawable_array_get_drawable(drawables, 0);
        for (size_t d = 0; d < count; d++) {
            cp_drawable_t dr = cp_drawable_array_get_drawable(drawables, d);
            if (cp_drawable_get_target(dr) == cp_drawable_target_built_in) {
                drawable = dr;
                break;
            }
        }

        // Pose for the moment this frame is actually SHOWN — each drawable's
        // own timing. Sampling "now" instead would anchor to a pose already a
        // frame stale at display time, which reads as the world swimming
        // against head motion. Every drawable needs its anchor; an unanchored
        // one is silently never displayed on device.
        for (size_t d = 0; d < count; d++) {
            cp_drawable_t dr = cp_drawable_array_get_drawable(drawables, d);
            cp_frame_timing_t finalTiming = cp_drawable_get_frame_timing(dr);
            ar_device_anchor_t anchor = ar_device_anchor_create();
            if (ar_world_tracking_provider_query_device_anchor_at_timestamp(
                    worldTracking,
                    cp_time_to_cf_time_interval(
                        cp_frame_timing_get_presentation_time(finalTiming)),
                    anchor) == ar_device_anchor_query_status_success) {
                cp_drawable_set_device_anchor(dr, anchor);
                if (dr == drawable) {
                    // Capture the head pose + BOTH views' geometry for
                    // xrView() and the XR backend. The pose is STICKY — a
                    // frame with a failed anchor query keeps rendering from
                    // the last good pose instead of snapping back to the game
                    // camera.
                    originFromDevice = ar_anchor_get_origin_from_anchor_transform(anchor);
                    trackedViewCount =
                        (int)MIN((size_t)2, cp_drawable_get_view_count(dr));
                    for (int v = 0; v < trackedViewCount; v++) {
                        deviceFromEye[v] =
                            cp_view_get_transform(cp_drawable_get_view(dr, v));
                        eyeProjection[v] = cp_drawable_compute_projection(
                            dr, cp_axis_direction_convention_right_up_back, v);
                    }
                    id<MTLTexture> tex0 = cp_drawable_get_color_texture(dr, 0);
                    trackedViewWidth = (int)tex0.width;
                    trackedViewHeight = (int)tex0.height;
                    trackedPoseValid = true;
                    if (anchorFails > 0 || !anchorEverSucceeded) {
                        NSLog(@"[vision] device anchor OK (after %d failures)", anchorFails);
                        anchorEverSucceeded = true;
                        anchorFails = 0;
                    }
                }
            } else if (dr == drawable) {
                if ((anchorFails++ % 90) == 0)
                    NSLog(@"[vision] device anchor query FAILED (%d so far)", anchorFails);
            }
        }
        return true;
    }

    id<MTLTexture> colorTarget() const override {
        return cp_drawable_get_color_texture(drawable, 0);
    }

    bool drawableSize(int& width, int& height) const override {
        if (!drawable) return false;
        id<MTLTexture> tex = cp_drawable_get_color_texture(drawable, 0);
        width  = static_cast<int>(tex.width);
        height = static_cast<int>(tex.height);
        return width > 0 && height > 0;
    }

    // From the layer CONFIGURATION, not a drawable: pipelines are built during
    // initialize(), long before the first frame is queried.
    MTLPixelFormat colorPixelFormat() const override {
        return cp_layer_renderer_configuration_get_color_format(
            cp_layer_renderer_get_configuration(layerRenderer));
    }

    void present(id<MTLCommandBuffer> commandBuffer,
                 id<MTLTexture> /*sceneDepth*/) override {
        if (!drawable || !drawables) return;
        // Color AND depth for every view are written by endFrame's per-view
        // passes (composite + depth blit through xrViewTarget's texture map).
        // All that remains here is presenting every drawable target.
        size_t count = cp_drawable_array_get_count(drawables);
        for (size_t d = 0; d < count; d++) {
            cp_drawable_t dr = cp_drawable_array_get_drawable(drawables, d);
            if (!loggedLayout) {
                NSLog(@"[vision] drawable %zu target=%d views=%zu textures=%zu "
                      @"tex0 %lux%lu (slices %lu)",
                      d, (int)cp_drawable_get_target(dr),
                      cp_drawable_get_view_count(dr),
                      cp_drawable_get_texture_count(dr),
                      static_cast<unsigned long>(cp_drawable_get_color_texture(dr, 0).width),
                      static_cast<unsigned long>(cp_drawable_get_color_texture(dr, 0).height),
                      static_cast<unsigned long>(cp_drawable_get_color_texture(dr, 0).arrayLength));
            }
            cp_drawable_encode_present(dr, commandBuffer);
        }
        loggedLayout = true;
    }
    bool loggedLayout = false;
    int anchorFails = 0;
    bool anchorEverSucceeded = false;

    // Head-tracked camera state for xrView() and the XR backend adapter.
    // Sticky: a frame with a failed anchor query keeps the last good pose.
    bool trackedPoseValid = false;
    simd_float4x4 originFromDevice;      // tracking origin -> headset
    int trackedViewCount = 0;
    simd_float4x4 deviceFromEye[2];      // headset -> each eye
    simd_float4x4 eyeProjection[2];      // compositor projection per eye
    int trackedViewWidth = 0, trackedViewHeight = 0;
    bool xrBaseValid = false;
    simd_float3 xrBase = {0, 0, 0};   // locomotion base: tracking origin in world
    simd_float3 baseHintPrev = {0, 0, 0};  // last game-camera position seen
    float worldScale = 1.0f;   // world units per real meter (Renderer::xrWorldScale)

    // Scale a rigid ORIGIN-space transform's translation by worldScale —
    // rotation (and therefore IPD *direction*) is untouched; the distances
    // the head and eyes travel are what make the user feel larger.
    simd_float4x4 scaledOriginTransform(simd_float4x4 m) const {
        m.columns[3].x *= worldScale;
        m.columns[3].y *= worldScale;
        m.columns[3].z *= worldScale;
        return m;
    }

    void setXrWorldScale(float scale) override {
        worldScale = (scale > 0.01f) ? scale : 1.0f;
    }

    bool xrTracking() const override { return trackedPoseValid; }

    int xrViewCount() const override {
        return (drawable && trackedPoseValid) ? trackedViewCount : 0;
    }

    bool xrViewCamera(int v, simd_float3 baseHint, simd_float4x4& worldFromEye,
                      simd_float4x4& projection) override {
        if (!trackedPoseValid || v < 0 || v >= trackedViewCount) return false;
        if (!xrBaseValid) {
            // Stand where the level's player stood, feet on the ground. In
            // play mode the game camera IS the player's eye, ~1.6m above
            // whatever it stands on — so ground ≈ hint.y - 1.6, which also
            // works on terrain and elevated city decks where the old
            // "Y = 0" seed buried the user under the world. (Arena floor is
            // at 0 with an eye ~1.6, so this changes nothing there.)
            xrBase = simd_make_float3(baseHint.x, baseHint.y - 1.6f, baseHint.z);
            baseHintPrev = baseHint;
            xrBaseValid = true;
        } else {
            // FOLLOW the game camera by deltas: when gameplay moves the
            // camera (teleport, vehicles, elevators), the tracking origin
            // moves with it — otherwise a teleport moves the character but
            // the user stays put, which reads as "I pinched and the world
            // glitched but I didn't go anywhere". Deltas (not absolutes)
            // keep the user's real head motion free, and re-applying a zero
            // delta is harmless, so no per-frame gating is needed.
            xrBase.x += baseHint.x - baseHintPrev.x;
            xrBase.y += baseHint.y - baseHintPrev.y;
            xrBase.z += baseHint.z - baseHintPrev.z;
            baseHintPrev = baseHint;
        }
        simd_float4x4 base = matrix_identity_float4x4;
        base.columns[3] = simd_make_float4(xrBase.x, xrBase.y, xrBase.z, 1.0f);
        worldFromEye = simd_mul(base, scaledOriginTransform(
            simd_mul(originFromDevice, deviceFromEye[v])));
        projection = eyeProjection[v];
        return true;
    }

    bool xrView(simd_float3 baseHint, simd_float4x4& worldFromEye,
                simd_float4x4& projection) override {
        return xrViewCamera(0, baseHint, worldFromEye, projection);
    }

    bool xrViewTarget(int v, id<MTLTexture> __strong& color,
                      id<MTLTexture> __strong& depth,
                      NSUInteger& slice, MTLViewport& viewport) override {
        if (!drawable || v < 0
            || v >= (int)cp_drawable_get_view_count(drawable)) return false;
        cp_view_t view = cp_drawable_get_view(drawable, v);
        cp_view_texture_map_t map = cp_view_get_view_texture_map(view);
        size_t texIndex = cp_view_texture_map_get_texture_index(map);
        slice = cp_view_texture_map_get_slice_index(map);
        viewport = cp_view_texture_map_get_viewport(map);
        color = cp_drawable_get_color_texture(drawable, texIndex);
        depth = cp_drawable_get_depth_texture(drawable, texIndex);
        return color != nil;
    }

    void frameSubmitted() override {
        if (frame) cp_frame_end_submission(frame);
        frame = nullptr;
        drawable = nullptr;
        drawables = nullptr;
    }
};

// simd -> engine Mat4 (transpose + widen): the inverse of toSimd below. Lives
// here because only the XR backend crosses this boundary in this direction.
static Mat4 fromSimd(simd_float4x4 m) {
    Mat4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = static_cast<Real>(m.columns[j][i]);
    return r;
}

// The engine-facing XR backend (engine/xr/xr_backend.h) for CompositorServices.
// Owned by MetalRenderer::Impl, wraps the CompositorSurface's tracking session.
// beginFrame answers with a PREDICTED head pose (the render still late-latches
// against the drawable's own timing); input is a thread-safe queue the Swift
// host pushes spatial events into (Phase 3).
struct CompositorXrBackend final : engine::XrBackend {
    CompositorSurface* surface = nullptr;
    uint64_t frameCounter = 0;
    std::mutex inputMutex;
    std::vector<engine::XrInputEvent> inputQueue;
    // Reused hand-anchor scratch objects for get_latest_anchors.
    ar_hand_anchor_t handAnchorLeft = nullptr;
    ar_hand_anchor_t handAnchorRight = nullptr;

    // Fill one XrHand from an ARKit hand anchor, mapping the runtime's named
    // joints onto the engine's canonical XrHandJointId order. Joint transforms
    // land in ORIGIN space with the world scale already applied.
    void fillHand(engine::XrHand& out, ar_hand_anchor_t anchor) {
        out.tracked = anchor && ar_hand_anchor_is_tracked(anchor);
        if (!out.tracked) return;
        static const ar_hand_skeleton_joint_name_t kJointNames[
            engine::XR_HAND_JOINT_COUNT] = {
            ar_hand_skeleton_joint_name_wrist,
            ar_hand_skeleton_joint_name_forearm_wrist,
            ar_hand_skeleton_joint_name_forearm_arm,
            ar_hand_skeleton_joint_name_thumb_knuckle,
            ar_hand_skeleton_joint_name_thumb_intermediate_base,
            ar_hand_skeleton_joint_name_thumb_intermediate_tip,
            ar_hand_skeleton_joint_name_thumb_tip,
            ar_hand_skeleton_joint_name_index_finger_metacarpal,
            ar_hand_skeleton_joint_name_index_finger_knuckle,
            ar_hand_skeleton_joint_name_index_finger_intermediate_base,
            ar_hand_skeleton_joint_name_index_finger_intermediate_tip,
            ar_hand_skeleton_joint_name_index_finger_tip,
            ar_hand_skeleton_joint_name_middle_finger_metacarpal,
            ar_hand_skeleton_joint_name_middle_finger_knuckle,
            ar_hand_skeleton_joint_name_middle_finger_intermediate_base,
            ar_hand_skeleton_joint_name_middle_finger_intermediate_tip,
            ar_hand_skeleton_joint_name_middle_finger_tip,
            ar_hand_skeleton_joint_name_ring_finger_metacarpal,
            ar_hand_skeleton_joint_name_ring_finger_knuckle,
            ar_hand_skeleton_joint_name_ring_finger_intermediate_base,
            ar_hand_skeleton_joint_name_ring_finger_intermediate_tip,
            ar_hand_skeleton_joint_name_ring_finger_tip,
            ar_hand_skeleton_joint_name_little_finger_metacarpal,
            ar_hand_skeleton_joint_name_little_finger_knuckle,
            ar_hand_skeleton_joint_name_little_finger_intermediate_base,
            ar_hand_skeleton_joint_name_little_finger_intermediate_tip,
            ar_hand_skeleton_joint_name_little_finger_tip,
        };
        ar_hand_skeleton_t skeleton = ar_hand_anchor_get_hand_skeleton(anchor);
        simd_float4x4 originFromAnchor =
            ar_hand_anchor_get_origin_from_anchor_transform(anchor);
        for (int j = 0; j < engine::XR_HAND_JOINT_COUNT; j++) {
            ar_skeleton_joint_t joint =
                ar_hand_skeleton_get_joint_named(skeleton, kJointNames[j]);
            if (!joint) { out.joints[j].tracked = false; continue; }
            out.joints[j].tracked = ar_skeleton_joint_is_tracked(joint);
            out.joints[j].originFromJoint =
                fromSimd(surface->scaledOriginTransform(simd_mul(
                    originFromAnchor,
                    ar_skeleton_joint_get_anchor_from_joint_transform(joint))));
        }
    }

    bool active() const override { return surface && surface->trackedPoseValid; }

    bool beginFrame(engine::XrState& out) override {
        out.active = false;
        if (!surface || !surface->worldTracking || !surface->trackedPoseValid)
            return false;

        // Predict roughly a frame ahead; if the query misses (tracking
        // momentarily paused), fall back to the sticky pose so the view
        // degrades to "held still", never to garbage.
        simd_float4x4 head = surface->originFromDevice;
        ar_device_anchor_t anchor = ar_device_anchor_create();
        if (ar_world_tracking_provider_query_device_anchor_at_timestamp(
                surface->worldTracking, CACurrentMediaTime() + 0.033, anchor)
            == ar_device_anchor_query_status_success) {
            head = ar_anchor_get_origin_from_anchor_transform(anchor);
        }

        out.frameIndex = ++frameCounter;
        // World scale applies at the source so all XrState consumers see
        // world-metric values (matching what the render path composes).
        out.originFromHead = fromSimd(surface->scaledOriginTransform(head));
        // The locomotion base: where the tracking origin sits in the world.
        // Gameplay composes world-space rays from it (teleport targeting).
        out.originBaseValid = surface->xrBaseValid;
        out.originBase = Vec3(surface->xrBase.x, surface->xrBase.y,
                              surface->xrBase.z);
        out.viewCount = surface->trackedViewCount;
        for (int v = 0; v < out.viewCount; v++) {
            // ORIGIN-space eye pose for the predicted head (scaled as one
            // rigid transform so the IPD scales with the stride).
            out.views[v].originFromEye = fromSimd(surface->scaledOriginTransform(
                simd_mul(head, surface->deviceFromEye[v])));
            out.views[v].projection = fromSimd(surface->eyeProjection[v]);
            out.views[v].targetIndex = v;
            out.views[v].width = surface->trackedViewWidth;
            out.views[v].height = surface->trackedViewHeight;
        }
        // Hand skeletons: latest tracked pose for both hands. Reports
        // untracked until the user grants the hands permission.
        out.hands[0].tracked = false;
        out.hands[1].tracked = false;
        if (surface->handTracking) {
            if (!handAnchorLeft) handAnchorLeft = ar_hand_anchor_create();
            if (!handAnchorRight) handAnchorRight = ar_hand_anchor_create();
            if (ar_hand_tracking_provider_get_latest_anchors(
                    surface->handTracking, handAnchorLeft, handAnchorRight)) {
                fillHand(out.hands[0], handAnchorLeft);
                fillHand(out.hands[1], handAnchorRight);
            }
        }

        out.active = true;

        if ((frameCounter % 90) == 0) {
            NSLog(@"[xr] predict #%llu head(%.2f %.2f %.2f) fwd(%.2f %.2f %.2f) views %d hands L%d R%d",
                  (unsigned long long)out.frameIndex,
                  out.originFromHead.m[0][3], out.originFromHead.m[1][3],
                  out.originFromHead.m[2][3],
                  -out.originFromHead.m[0][2], -out.originFromHead.m[1][2],
                  -out.originFromHead.m[2][2], out.viewCount,
                  out.hands[0].tracked ? 1 : 0, out.hands[1].tracked ? 1 : 0);
        }
        return true;
    }

    void pushInput(const engine::XrInputEvent& event) override {
        std::lock_guard<std::mutex> lock(inputMutex);
        inputQueue.push_back(event);
    }

    void pollInput(std::vector<engine::XrInputEvent>& out) override {
        std::lock_guard<std::mutex> lock(inputMutex);
        out.insert(out.end(), inputQueue.begin(), inputQueue.end());
        inputQueue.clear();
    }
};
#endif  // TARGET_OS_VISION

static simd_float4x4 toSimd(const Mat4& m) {
    simd_float4x4 result;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            result.columns[j][i] = static_cast<float>(m.m[i][j]);
    return result;
}

static simd_float4x4 inverseTranspose(simd_float4x4 m) {
    return simd_transpose(simd_inverse(m));
}

// Threadgroup count covering `threads` (ceiling division). All compute
// dispatches go through dispatchThreadgroups: — dispatchThreads: (non-uniform
// threadgroups) is unsupported on visionOS-class GPU families, so kernels must
// bounds-check against the real grid size.
static MTLSize threadgroupsCovering(MTLSize threads, MTLSize group) {
    return MTLSizeMake((threads.width  + group.width  - 1) / group.width,
                       (threads.height + group.height - 1) / group.height,
                       (threads.depth  + group.depth  - 1) / group.depth);
}

struct MetalRenderer::Impl {
    id<MTLDevice> device;
    id<MTLCommandQueue> commandQueue;
    id<MTLRenderPipelineState> opaquePipeline;
    id<MTLRenderPipelineState> terrainPipeline;   // CDLOD vertex morph (ADR-0036)
    id<MTLRenderPipelineState> transparentPipeline;
    id<MTLRenderPipelineState> opaqueInstancedPipeline;
    id<MTLRenderPipelineState> transparentInstancedPipeline;
    id<MTLRenderPipelineState> foliageDepthPipeline;   // alpha-cut depth prepass (perf)
    id<MTLRenderPipelineState> foliageLitPipeline;     // foliage lit w/ early depth test
    id<MTLDepthStencilState> depthStateOpaque;        // reverse-Z: Greater, write
    id<MTLDepthStencilState> depthStateFoliageLit;    // reverse-Z: Equal, no write (prepass)
    id<MTLDepthStencilState> depthStateTransparent;   // reverse-Z: Greater, no write
    id<MTLDepthStencilState> depthStateWireOverlay;   // reverse-Z: GreaterEqual, no write
    id<MTLDepthStencilState> depthStateOverlay;       // Always pass, no write (debug gizmos on top)
    id<MTLDepthStencilState> depthStateOpaqueForwardZ; // probe bake: Less, write
    id<MTLBuffer> instanceBuffers[MAX_FRAMES_IN_FLIGHT];  // ring; see frameIndex
    id<MTLBuffer> shadowInstanceBuffers[MAX_FRAMES_IN_FLIGHT];  // ring; shadow caster models
    id<MTLBuffer> foliageInstanceBuffers[MAX_FRAMES_IN_FLIGHT]; // ring; foliage prepass+lit
    int frameIndex = 0;                                   // advances each beginFrame
    uint64_t frameCount = 0;                              // monotonic; drives SSAO jitter
    // How this frame reaches the display. The pass graph never looks at it —
    // see PresentationSurface above.
    std::unique_ptr<PresentationSurface> surface;
#if TARGET_OS_VISION
    // Engine-facing XR backend (engine/xr/): wraps the surface's tracking.
    // Points INTO `surface` — declared after it so it destructs first.
    std::unique_ptr<CompositorXrBackend> xrAdapter;
#endif
    id<MTLTexture> depthTexture;

    SlotMap<GPUMesh, MeshTag> meshes;
    SlotMap<id<MTLTexture>, TextureTag> textures;
    id<MTLSamplerState> linearWrapSampler;
    id<MTLTexture> defaultWhiteTexture;

    CameraUniforms cameraUniforms;
    LightUniforms lightUniforms;
    id<MTLBuffer> lightBuffer;

    // Skybox
    id<MTLRenderPipelineState> skyboxPipeline;
    id<MTLDepthStencilState> skyboxDepthState;

    // Environment (ADR-0016): an equirectangular HDR map drives the skybox and,
    // via the probe bake, IBL. nil => procedural sky.
    bool skyCloudsEnabled = true;  // mirrors SceneLighting::sky.cloudsEnabled
    id<MTLTexture> environmentTexture;     // equirect RGBA16Float source, nil if procedural
    id<MTLTexture> environmentCubemap;     // equirect baked to a cube (ADR-0016), nil if procedural
    id<MTLTexture> defaultCubemap;         // 1×1 dummy cube for the procedural binding
    id<MTLRenderPipelineState> equirectBakePipeline;  // equirect → cube face
    id<MTLSamplerState> equirectSampler;   // linear, wrap-U / clamp-V
    static constexpr int ENV_CUBEMAP_SIZE = 1024;
    void bakeEnvironmentCubemap();         // (re)bake environmentCubemap from environmentTexture

    // IBL cubes derived from the environment cube (ADR-0017 Phase 3):
    // GGX-prefiltered radiance (mips linear in roughness) + cosine-convolved
    // irradiance. nil in procedural mode (the shader evaluates the analytic
    // sky instead).
    id<MTLTexture> envPrefilteredCube;
    id<MTLTexture> envIrradianceCube;
    id<MTLComputePipelineState> irradiancePipeline;
    id<MTLSamplerState> mipClampSampler;   // linear min/mag/MIP — level() lookups
    static constexpr int ENV_PREFILTER_SIZE = 128;
    static constexpr int ENV_PREFILTER_MIPS = 5;   // roughness 0..1 → mip 0..4
    static constexpr int ENV_IRRADIANCE_SIZE = 32;
    void bakeEnvironmentIBL();             // (re)build both from environmentCubemap
    // Hardware ground truth for the bake: read faces back and compare against
    // CPU-sampled equirect under eight orientation hypotheses, then hardware-
    // sample the cube along known directions — together they pin down whether
    // a mismatch lives in the bake or in the sampler convention.
    id<MTLComputePipelineState> cubeValidatePipeline;
    id<MTLRenderPipelineState> dirDebugPipeline;
    void validateBakedCube(id<MTLTexture> cube);

    // Post-processing: offscreen HDR target + G-buffer normals + composite pass
    id<MTLTexture> sceneColorTexture;
    id<MTLTexture> viewNormalTexture;
    id<MTLRenderPipelineState> compositePipeline;
    id<MTLSamplerState> linearClampSampler;

    // Planetary atmosphere glow (procedural-planet-plan P3): a fullscreen additive
    // pass over the HDR scene, driven by setAtmosphere(). `atmosphere` holds the
    // scene-set params (centre/radii/coeffs); camera + sun are filled per frame.
    id<MTLRenderPipelineState> atmospherePipeline;
    AtmosphereRenderParams atmosphere;

    // Screen-space reflections (SSR)
    id<MTLTexture> ssrTexture;         // RGBA16Float — SSR result (rgb=color, a=confidence)
    id<MTLTexture> ssrBlurTemp;        // RGBA16Float — ping-pong for bilateral blur
    id<MTLComputePipelineState> ssrPipeline;
    id<MTLComputePipelineState> ssrBlurHPipeline;
    id<MTLComputePipelineState> ssrBlurVPipeline;

    // Bloom
    static constexpr int BLOOM_MIP_COUNT = 5;
    id<MTLTexture> bloomMips[BLOOM_MIP_COUNT];     // RGBA16Float downsample chain
    id<MTLTexture> bloomUpsampleMips[BLOOM_MIP_COUNT]; // upsample chain (reuses mip 0 slot for final)
    id<MTLComputePipelineState> bloomDownsamplePipeline;
    id<MTLComputePipelineState> bloomUpsamplePipeline;

    // Lens effects (virtual-camera plan Phase 4): final image-space warp
    // (distortion + CA + vignette) and depth-of-field gather, driven by the
    // active camera's LensParams via setCamera.
    id<MTLRenderPipelineState> lensWarpPipeline;
    id<MTLTexture> postLDRTexture;     // BGRA8Unorm — composite target when the warp runs
    id<MTLComputePipelineState> dofPipeline;
    id<MTLTexture> dofTexture;         // RGBA16Float — DOF gather output
    LensParams currentLens;

    // Screen-space ambient occlusion (SSAO/GTAO)
    id<MTLTexture> aoTexture;          // R16Float — half-res AO result
    id<MTLTexture> aoBlurTemp;         // R16Float — half-res blur ping-pong / temporal out
    id<MTLTexture> aoHistory;          // R16Float — half-res previous frame's resolved AO
    id<MTLComputePipelineState> aoPipeline;
    id<MTLComputePipelineState> aoBlurHPipeline;
    id<MTLComputePipelineState> aoBlurVPipeline;
    id<MTLComputePipelineState> aoTemporalPipeline;   // reprojected history blend
    simd_float4x4 aoCurrViewProjection; // this frame's VP (inverse drives recon)
    simd_float4x4 aoPrevViewProjection; // last frame's VP (reproject into history)
    bool aoHistoryValid;                // false until the first resolve / after resize

    // Reflection probes (cubemap-based IBL)
    id<MTLTexture> probeCubemapArray;      // texturecube_array, RGBA16Float
    id<MTLTexture> brdfLUT;                // 256×256 RG16Float
    id<MTLBuffer> probeBuffer;             // GPUReflectionProbe[]
    ProbeUniforms probeUniforms = {};
    id<MTLComputePipelineState> brdfLUTPipeline;
    id<MTLComputePipelineState> prefilterPipeline;
    static constexpr int PROBE_CUBEMAP_SIZE = 256;
    static constexpr int PROBE_MIP_LEVELS = 6;  // mip 0..5
    static constexpr int MAX_PROBES = 8;
    int probeCount = 0;
    bool probesBaked = false;
    bool probesPendingBake = false;
    std::vector<ReflectionProbe> pendingProbes;

    // Shadow mapping
    id<MTLTexture> shadowMap;
    id<MTLRenderPipelineState> shadowPipeline;
    id<MTLRenderPipelineState> shadowInstancedPipeline;
    id<MTLRenderPipelineState> terrainShadowPipeline;   // CDLOD morphing caster
    id<MTLDepthStencilState> shadowDepthState;
    id<MTLSamplerState> shadowSampler;
    int shadowMapSize = 2048;
    bool shadowEnabled = false;
    ShadowUniforms shadowUniforms;
    // Per-cascade light VP (for the shadow pass) and world bounds (for culling
    // casters per cascade). activeCascadeCount cascades are live this frame.
    simd_float4x4 cascadeVP[RT_MAX_CASCADES];
    Vec3 cascadeCenter[RT_MAX_CASCADES];
    Real cascadeRadius[RT_MAX_CASCADES] = {0};
    int activeCascadeCount = 0;
    // Rasterization-side depth bias for the shadow pass encoder, from
    // ShadowConfig::bias (the lookup-side controls ride in shadowUniforms).
    float shadowDepthBias = 0.005f;

    // Headless visual verification: RT_FRAME_DUMP=<path.png> writes one
    // composited frame to disk (~frame 90, after probes bake and physics
    // settle) — eyes on the renderer without window capture permissions.
    const char* frameDumpPath = nullptr;
    int frameDumpCounter = 0;

    // Cached once per frame from surface->colorTarget(), so the composite, lens
    // and frame-dump stages agree on the target even though they run apart.
    id<MTLTexture> currentColorTarget;
    CameraState lastCameraState;      // for the per-frame XR re-derive
    bool hasLastCamera = false;
    // Locomotion-base hint from XrCameraSystem (the GAMEPLAY camera, pre
    // head-overwrite). Preferred over lastCameraState for base following —
    // once the shared camera follows the head, using it as the hint would
    // feed head motion straight back into the base.
    simd_float3 xrBaseHint = {0, 0, 0};
    bool xrBaseHintValid = false;
    id<MTLCommandBuffer> currentCommandBuffer;
    id<MTLRenderCommandEncoder> currentEncoder;
    MTLRenderPassDescriptor* currentPassDesc;   // scene pass (HDR offscreen)
    MTLRenderPassDescriptor* compositePassDesc; // composite pass (LDR drawable)
    bool imguiInitialized = false;

    int framebufferWidth = 0;
    int framebufferHeight = 0;

    struct DrawCall {
        MeshHandle meshHandle;
        Mat4 transform;
        RenderMaterial material;
        float distanceToCamera;
    };

    std::vector<DrawCall> opaqueDrawCalls;
    std::vector<DrawCall> transparentDrawCalls;
    std::vector<DrawCall> overlayDrawCalls;   // FLAG_OVERLAY gizmos, issued last

    // CDLOD terrain nodes (ADR-0036): drawn with the morph pipeline after opaque.
    struct TerrainDrawCall {
        MeshHandle meshHandle;
        RenderMaterial material;
        float morphStart;
        float morphEnd;
    };
    std::vector<TerrainDrawCall> terrainDrawCalls;

    Vec3 currentCameraPos;
    RenderStats lastStats;

    static void bakeProbes(Impl* impl, const std::vector<ReflectionProbe>& probes);
};

MetalRenderer::MetalRenderer() : impl(std::make_unique<Impl>()) {}
MetalRenderer::~MetalRenderer() { shutdown(); }

bool MetalRenderer::initialize(void* windowHandle, int width, int height) {
    // The handle is opaque (ADR-0001) and its concrete type is per-platform:
    // the GLFW runtime passes an NSWindow*, the Qt editor passes the NSView* of
    // its viewport widget, and visionOS passes the cp_layer_renderer_t handed
    // to it by the SwiftUI immersive space. Unwrapping it is the one place that
    // has to know which platform it is on; everything after this point does not.
#if TARGET_OS_VISION
    // The compositor already owns a device — never create a second one, or every
    // resource below would belong to the wrong GPU object.
    cp_layer_renderer_t layerRenderer = (__bridge cp_layer_renderer_t)windowHandle;
    if (!layerRenderer) return false;

    impl->device = cp_layer_renderer_get_device(layerRenderer);
    if (!impl->device) return false;
    impl->commandQueue = [impl->device newCommandQueue];

    auto compositorSurface = std::make_unique<CompositorSurface>();
    compositorSurface->layerRenderer = layerRenderer;
    if (!compositorSurface->startTracking()) {
        NSLog(@"[MetalRenderer] world tracking unavailable — cannot present");
        return false;
    }
    impl->xrAdapter = std::make_unique<CompositorXrBackend>();
    impl->xrAdapter->surface = compositorSurface.get();
    impl->surface = std::move(compositorSurface);
#else
    NSObject* handleObj = (__bridge NSObject*)windowHandle;
    NSWindow* nsWindow = nil;
    NSView* hostView = nil;
    if ([handleObj isKindOfClass:[NSWindow class]]) {
        nsWindow = (NSWindow*)handleObj;
        hostView = nsWindow.contentView;
    } else if ([handleObj isKindOfClass:[NSView class]]) {
        hostView = (NSView*)handleObj;
        nsWindow = hostView.window;   // may be nil before the host shows it
    } else {
        return false;
    }

    impl->device = MTLCreateSystemDefaultDevice();
    if (!impl->device) return false;

    impl->commandQueue = [impl->device newCommandQueue];

    auto layerSurface = std::make_unique<LayerSurface>();
    layerSurface->layer = [CAMetalLayer layer];
    layerSurface->layer.device = impl->device;
    layerSurface->layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    // framebufferOnly drawables can't be blitted from; relax it only when a
    // frame dump was requested (RT_FRAME_DUMP=<path.png>).
    impl->frameDumpPath = std::getenv("RT_FRAME_DUMP");
    layerSurface->layer.framebufferOnly = impl->frameDumpPath ? NO : YES;
    layerSurface->layer.contentsScale =
        nsWindow ? nsWindow.backingScaleFactor : 2.0;   // retina default

    hostView.wantsLayer = YES;
    hostView.layer = layerSurface->layer;
    layerSurface->window = nsWindow;
    impl->surface = std::move(layerSurface);
#endif  // TARGET_OS_VISION

    // Debug knobs, read on EVERY platform. These lived in the macOS branch until
    // visionOS arrived, which silently made every headless debug view
    // unreachable there — exactly when they were most needed for diagnosing a
    // platform-specific rendering difference.
    //   1=AO 2=SSR 3=depth 4=normals 5=shadow 6=albedo 7=facing 8=cascades
    impl->frameDumpPath = std::getenv("RT_FRAME_DUMP");
    if (const char* w = std::getenv("RT_WIREFRAME")) wireframe = std::atoi(w);
    if (const char* d = std::getenv("RT_DEBUG_VIEW")) debugView = std::atoi(d);

    // Load shaders. Runtime newLibraryWithSource has no include paths, so the
    // modules are concatenated in dependency order; #line directives keep
    // compile diagnostics pointing at the right file (ADR-0017 Phase 0).
    //
    // THE ORDER IS LOAD-BEARING. Concatenation is textual with no forward
    // declarations, so every module must follow the ones it calls into. This
    // list is therefore both the build order and the dependency documentation:
    // a mistake surfaces as a compile error naming the right file via #line.
    // Subsetting it is also how a backend drops effects it never runs (these
    // compile at runtime on every launch, so a headset build should not be
    // compiling SSR/DOF/lens).
    NSError* error = nil;
    NSArray<NSString*>* shaderFiles = @[
        @"shaders/metal/shader_types.h",   // GPU structs shared with C++
        @"shaders/metal/common.metal",     // layouts, Fresnel, noise primitives

        // --- surface library --- (each material owns its albedo *and* relief;
        // surfaces.metal dispatches, so it comes last)
        @"shaders/metal/surfaces_facade.metal", // brick/concrete/…/wood
        @"shaders/metal/surface_road.metal",    // after facade: calls surfAsphalt
        @"shaders/metal/surface_water.metal",
        @"shaders/metal/surface_terrain.metal",
        @"shaders/metal/surfaces.metal",        // applySurface + applySurfaceRelief

        // --- lighting ---
        @"shaders/metal/environment.metal",// sky/HDR providers, IBL precompute
        @"shaders/metal/shadows.metal",    // shadow pass + PCF lookup
        @"shaders/metal/lighting_env.metal",     // IBL sampling + reflection probes
        @"shaders/metal/lighting_brdf.metal",    // GGX terms + evaluateLighting
        @"shaders/metal/lighting_surface.metal", // shadeSurface (needs surfaces.metal)
        @"shaders/metal/lighting_entry.metal",   // vertex/fragment entry points

        // --- post stack --- (post_common first: SSR, AO *and* composite use it)
        @"shaders/metal/post_common.metal",   // unproject, linearize, bilateral
        @"shaders/metal/post_ssr.metal",      // ray march + separable blur
        @"shaders/metal/post_ao.metal",       // GTAO + blurs + temporal reproject
        @"shaders/metal/post_bloom.metal",    // downsample/upsample pyramid
        @"shaders/metal/post_composite.metal",// tone map + grade → drawable
        @"shaders/metal/post_lens.metal",     // DOF gather + distortion/CA/vignette

        @"shaders/metal/atmosphere.metal", // planetary atmosphere glow (P3)
    ];
    NSMutableString* shaderSource = [NSMutableString string];
    for (NSString* path in shaderFiles) {
        // Resolve through the asset root so the same list works from the repo
        // root (root unset -> identity) and from inside an app bundle, whose
        // working directory is not the repo. See engine/asset_root.h.
        NSString* resolved = [NSString
            stringWithUTF8String:engine::assetPath([path UTF8String]).c_str()];
        NSString* chunk = [NSString stringWithContentsOfFile:resolved
                                                    encoding:NSUTF8StringEncoding
                                                       error:&error];
        if (!chunk) {
            NSLog(@"Failed to load shader %@: %@", resolved, error);
            return false;
        }
        [shaderSource appendFormat:@"#line 1 \"%@\"\n%@\n",
                                   [path lastPathComponent], chunk];
    }

    MTLCompileOptions* options = [[MTLCompileOptions alloc] init];
    id<MTLLibrary> library = [impl->device newLibraryWithSource:shaderSource
                                                        options:options
                                                          error:&error];
    if (!library) {
        NSLog(@"Shader compile error: %@", error);
        return false;
    }

    id<MTLFunction> vertexFunc = [library newFunctionWithName:@"vertexMain"];
    id<MTLFunction> fragmentFunc = [library newFunctionWithName:@"fragmentMain"];
    id<MTLFunction> vertexInstancedFunc = [library newFunctionWithName:@"vertexMainInstanced"];
    id<MTLFunction> fragmentInstancedFunc = [library newFunctionWithName:@"fragmentMainInstanced"];

    // Opaque pipeline (MRT: color + view-space normals)
    MTLRenderPipelineDescriptor* pipelineDesc = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineDesc.vertexFunction = vertexFunc;
    pipelineDesc.fragmentFunction = fragmentFunc;
    pipelineDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA16Float;
    pipelineDesc.colorAttachments[1].pixelFormat = MTLPixelFormatRGBA8Unorm;
    pipelineDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;

    impl->opaquePipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                        error:&error];
    if (!impl->opaquePipeline) {
        NSLog(@"Pipeline error: %@", error);
        return false;
    }

    // CDLOD terrain pipeline (ADR-0036): the morph vertex shader feeding the shared
    // lit fragment, same MRT/depth as opaque (drawn in the opaque pass). Optional —
    // a missing function just disables CDLOD draws (drawTerrain no-ops).
    id<MTLFunction> terrainVertexFunc = [library newFunctionWithName:@"terrainVertexMain"];
    if (terrainVertexFunc) {
        pipelineDesc.vertexFunction = terrainVertexFunc;
        pipelineDesc.fragmentFunction = fragmentFunc;
        impl->terrainPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                             error:&error];
        if (!impl->terrainPipeline) NSLog(@"Terrain pipeline error: %@", error);
        pipelineDesc.vertexFunction = vertexFunc;   // restore for the pipelines below
    }

    // Opaque instanced pipeline
    pipelineDesc.vertexFunction = vertexInstancedFunc;
    pipelineDesc.fragmentFunction = fragmentInstancedFunc;
    impl->opaqueInstancedPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                                 error:&error];
    if (!impl->opaqueInstancedPipeline) {
        NSLog(@"Instanced pipeline error: %@", error);
        return false;
    }

    // Foliage depth-prepass + lit pipelines (perf — alpha-cut overdraw). Both
    // reuse vertexMainInstanced (same depth as the lit pass, so the Equal test in
    // the lit stage is exact). The lit fragment carries [[early_fragment_tests]].
    // A missing function just leaves the pointers nil and the renderer falls back
    // to the legacy single-pass foliage path (depthPrepassEnabled has no effect).
    {
        id<MTLFunction> foliageDepthFunc =
            [library newFunctionWithName:@"fragmentFoliageDepthInstanced"];
        id<MTLFunction> foliageLitFunc =
            [library newFunctionWithName:@"fragmentMainInstancedFoliage"];
        if (foliageDepthFunc && foliageLitFunc) {
            // Lit variant: same MRT, normal color writes.
            pipelineDesc.vertexFunction = vertexInstancedFunc;
            pipelineDesc.fragmentFunction = foliageLitFunc;
            impl->foliageLitPipeline =
                [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc error:&error];
            if (!impl->foliageLitPipeline) NSLog(@"Foliage lit pipeline error: %@", error);

            // Depth-only variant: void fragment, disable color writes on both MRT
            // attachments. Restore the write mask afterward for the pipelines below.
            pipelineDesc.fragmentFunction = foliageDepthFunc;
            pipelineDesc.colorAttachments[0].writeMask = MTLColorWriteMaskNone;
            pipelineDesc.colorAttachments[1].writeMask = MTLColorWriteMaskNone;
            impl->foliageDepthPipeline =
                [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc error:&error];
            if (!impl->foliageDepthPipeline) NSLog(@"Foliage depth pipeline error: %@", error);
            pipelineDesc.colorAttachments[0].writeMask = MTLColorWriteMaskAll;
            pipelineDesc.colorAttachments[1].writeMask = MTLColorWriteMaskAll;
        }
    }

    // Transparent pipeline (alpha blending)
    pipelineDesc.vertexFunction = vertexFunc;
    pipelineDesc.fragmentFunction = fragmentFunc;
    pipelineDesc.colorAttachments[0].blendingEnabled = YES;
    pipelineDesc.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    pipelineDesc.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    pipelineDesc.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
    pipelineDesc.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;

    impl->transparentPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                             error:&error];

    // Transparent instanced pipeline
    pipelineDesc.vertexFunction = vertexInstancedFunc;
    pipelineDesc.fragmentFunction = fragmentInstancedFunc;
    impl->transparentInstancedPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                                      error:&error];

    // Skybox pipeline
    {
        id<MTLFunction> skyVert = [library newFunctionWithName:@"vertexSkybox"];
        id<MTLFunction> skyFrag = [library newFunctionWithName:@"fragmentSkybox"];
        MTLRenderPipelineDescriptor* skyDesc = [[MTLRenderPipelineDescriptor alloc] init];
        skyDesc.vertexFunction = skyVert;
        skyDesc.fragmentFunction = skyFrag;
        skyDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA16Float;
        skyDesc.colorAttachments[1].pixelFormat = MTLPixelFormatRGBA8Unorm;
        skyDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
        impl->skyboxPipeline = [impl->device newRenderPipelineStateWithDescriptor:skyDesc
                                                                            error:&error];
        if (!impl->skyboxPipeline) NSLog(@"Skybox pipeline error: %@", error);

        MTLDepthStencilDescriptor* skyDepthDesc = [[MTLDepthStencilDescriptor alloc] init];
        skyDepthDesc.depthCompareFunction = MTLCompareFunctionAlways;
        skyDepthDesc.depthWriteEnabled = NO;
        impl->skyboxDepthState = [impl->device newDepthStencilStateWithDescriptor:skyDepthDesc];

        // Equirect → cubemap bake pipeline: vertexSkybox + a raw-radiance frag,
        // single color attachment, no depth/normal MRT (renders to one cube face).
        id<MTLFunction> bakeFrag = [library newFunctionWithName:@"fragmentEquirectBake"];
        if (bakeFrag) {
            MTLRenderPipelineDescriptor* bakeDesc = [[MTLRenderPipelineDescriptor alloc] init];
            bakeDesc.vertexFunction = skyVert;
            bakeDesc.fragmentFunction = bakeFrag;
            bakeDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA16Float;
            impl->equirectBakePipeline = [impl->device newRenderPipelineStateWithDescriptor:bakeDesc
                                                                                      error:&error];
            if (!impl->equirectBakePipeline) NSLog(@"Equirect bake pipeline error: %@", error);
        }
    }

    // Composite pipeline (fullscreen triangle, reads HDR scene texture, writes to drawable)
    {
        id<MTLFunction> compVert = [library newFunctionWithName:@"vertexComposite"];
        id<MTLFunction> compFrag = [library newFunctionWithName:@"fragmentComposite"];
        MTLRenderPipelineDescriptor* compDesc = [[MTLRenderPipelineDescriptor alloc] init];
        compDesc.vertexFunction = compVert;
        compDesc.fragmentFunction = compFrag;
        compDesc.colorAttachments[0].pixelFormat = impl->surface->colorPixelFormat();
        // No depth attachment for the composite pass
        impl->compositePipeline = [impl->device newRenderPipelineStateWithDescriptor:compDesc
                                                                               error:&error];
        if (!impl->compositePipeline) NSLog(@"Composite pipeline error: %@", error);
    }

    // Atmosphere-glow pipeline (procedural-planet-plan P3): a fullscreen triangle
    // that raymarches the atmosphere shell and ADDITIVELY blends the in-scattered
    // light over the HDR scene target (One+One), so the limb halo blooms in post.
    // No depth attachment — it draws everywhere the shell is hit.
    {
        id<MTLFunction> atmVert = [library newFunctionWithName:@"vertexAtmosphere"];
        id<MTLFunction> atmFrag = [library newFunctionWithName:@"fragmentAtmosphereGlow"];
        MTLRenderPipelineDescriptor* atmDesc = [[MTLRenderPipelineDescriptor alloc] init];
        atmDesc.vertexFunction = atmVert;
        atmDesc.fragmentFunction = atmFrag;
        atmDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA16Float;   // the HDR scene
        atmDesc.colorAttachments[0].blendingEnabled = YES;
        atmDesc.colorAttachments[0].rgbBlendOperation = MTLBlendOperationAdd;
        atmDesc.colorAttachments[0].alphaBlendOperation = MTLBlendOperationAdd;
        atmDesc.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorOne;
        atmDesc.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOne;
        atmDesc.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorZero;
        atmDesc.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
        impl->atmospherePipeline = [impl->device newRenderPipelineStateWithDescriptor:atmDesc
                                                                                error:&error];
        if (!impl->atmospherePipeline) NSLog(@"Atmosphere pipeline error: %@", error);
    }

    // Lens-warp pipeline (virtual-camera plan Phase 4): fullscreen resample of
    // the composited LDR image (distortion + CA + vignette), writes the drawable.
    {
        id<MTLFunction> lensVert = [library newFunctionWithName:@"vertexComposite"];
        id<MTLFunction> lensFrag = [library newFunctionWithName:@"fragmentLensWarp"];
        if (lensVert && lensFrag) {
            MTLRenderPipelineDescriptor* lensDesc = [[MTLRenderPipelineDescriptor alloc] init];
            lensDesc.vertexFunction = lensVert;
            lensDesc.fragmentFunction = lensFrag;
            lensDesc.colorAttachments[0].pixelFormat = impl->surface->colorPixelFormat();
            // No depth attachment, same as the composite pass
            impl->lensWarpPipeline = [impl->device newRenderPipelineStateWithDescriptor:lensDesc
                                                                                   error:&error];
            if (!impl->lensWarpPipeline) NSLog(@"Lens warp pipeline error: %@", error);
        }
    }

    // Linear clamp sampler (for post-processing texture reads)
    {
        MTLSamplerDescriptor* sampDesc = [[MTLSamplerDescriptor alloc] init];
        sampDesc.minFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.magFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.sAddressMode = MTLSamplerAddressModeClampToEdge;
        sampDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
        impl->linearClampSampler = [impl->device newSamplerStateWithDescriptor:sampDesc];
    }

    // Mip-filtering clamp sampler for IBL level() lookups (prefiltered env,
    // probe roughness mips). Without a mip filter, level() clamps to mip 0 —
    // which is why probe roughness blur silently never worked.
    {
        MTLSamplerDescriptor* sampDesc = [[MTLSamplerDescriptor alloc] init];
        sampDesc.minFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.magFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.mipFilter = MTLSamplerMipFilterLinear;
        sampDesc.sAddressMode = MTLSamplerAddressModeClampToEdge;
        sampDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
        impl->mipClampSampler = [impl->device newSamplerStateWithDescriptor:sampDesc];
    }

    // Linear wrap sampler (for material textures)
    {
        MTLSamplerDescriptor* sampDesc = [[MTLSamplerDescriptor alloc] init];
        sampDesc.minFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.magFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.mipFilter = MTLSamplerMipFilterLinear;
        sampDesc.sAddressMode = MTLSamplerAddressModeRepeat;
        sampDesc.tAddressMode = MTLSamplerAddressModeRepeat;
        impl->linearWrapSampler = [impl->device newSamplerStateWithDescriptor:sampDesc];
    }

    // Equirectangular environment sampler: wrap horizontally (longitude is
    // periodic), clamp vertically (poles), linear filtering (ADR-0016).
    {
        MTLSamplerDescriptor* sampDesc = [[MTLSamplerDescriptor alloc] init];
        sampDesc.minFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.magFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.sAddressMode = MTLSamplerAddressModeRepeat;
        sampDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
        impl->equirectSampler = [impl->device newSamplerStateWithDescriptor:sampDesc];
    }

    // 1x1 white default texture (bound when no material texture is set)
    {
        MTLTextureDescriptor* desc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                         width:1 height:1 mipmapped:NO];
        desc.usage = MTLTextureUsageShaderRead;
        impl->defaultWhiteTexture = [impl->device newTextureWithDescriptor:desc];
        uint8_t white[] = {255, 255, 255, 255};
        [impl->defaultWhiteTexture replaceRegion:MTLRegionMake2D(0, 0, 1, 1)
                                     mipmapLevel:0
                                       withBytes:white
                                     bytesPerRow:4];
    }

    // Compute pipelines for IBL (reflection probes)
    {
        id<MTLFunction> brdfFunc = [library newFunctionWithName:@"integrateBRDF"];
        if (brdfFunc) {
            impl->brdfLUTPipeline = [impl->device newComputePipelineStateWithFunction:brdfFunc
                                                                                error:&error];
            if (!impl->brdfLUTPipeline) NSLog(@"BRDF LUT pipeline error: %@", error);
        }
        id<MTLFunction> prefilterFunc = [library newFunctionWithName:@"prefilterEnvMap"];
        if (prefilterFunc) {
            impl->prefilterPipeline = [impl->device newComputePipelineStateWithFunction:prefilterFunc
                                                                                  error:&error];
            if (!impl->prefilterPipeline) NSLog(@"Prefilter pipeline error: %@", error);
        }
        id<MTLFunction> irradianceFunc = [library newFunctionWithName:@"convolveIrradiance"];
        if (irradianceFunc) {
            impl->irradiancePipeline = [impl->device newComputePipelineStateWithFunction:irradianceFunc
                                                                                   error:&error];
            if (!impl->irradiancePipeline) NSLog(@"Irradiance pipeline error: %@", error);
        }
        id<MTLFunction> validateFunc = [library newFunctionWithName:@"sampleCubeForValidation"];
        if (validateFunc) {
            impl->cubeValidatePipeline = [impl->device newComputePipelineStateWithFunction:validateFunc
                                                                                     error:&error];
            if (!impl->cubeValidatePipeline) NSLog(@"Cube validation pipeline error: %@", error);
        }
        id<MTLFunction> skyVertForDebug = [library newFunctionWithName:@"vertexSkybox"];
        id<MTLFunction> dirDebugFunc = [library newFunctionWithName:@"fragmentDirectionDebug"];
        if (skyVertForDebug && dirDebugFunc) {
            MTLRenderPipelineDescriptor* dd = [[MTLRenderPipelineDescriptor alloc] init];
            dd.vertexFunction = skyVertForDebug;
            dd.fragmentFunction = dirDebugFunc;
            dd.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA16Float;
            impl->dirDebugPipeline = [impl->device newRenderPipelineStateWithDescriptor:dd
                                                                                  error:&error];
            if (!impl->dirDebugPipeline) NSLog(@"Direction debug pipeline error: %@", error);
        }
    }

    // SSR compute pipelines
    {
        id<MTLFunction> ssrFunc = [library newFunctionWithName:@"ssrRayMarch"];
        if (ssrFunc) {
            impl->ssrPipeline = [impl->device newComputePipelineStateWithFunction:ssrFunc
                                                                            error:&error];
            if (!impl->ssrPipeline) NSLog(@"SSR pipeline error: %@", error);
        }
        id<MTLFunction> blurHFunc = [library newFunctionWithName:@"ssrBlurH"];
        if (blurHFunc) {
            impl->ssrBlurHPipeline = [impl->device newComputePipelineStateWithFunction:blurHFunc
                                                                                 error:&error];
            if (!impl->ssrBlurHPipeline) NSLog(@"SSR blur H pipeline error: %@", error);
        }
        id<MTLFunction> blurVFunc = [library newFunctionWithName:@"ssrBlurV"];
        if (blurVFunc) {
            impl->ssrBlurVPipeline = [impl->device newComputePipelineStateWithFunction:blurVFunc
                                                                                 error:&error];
            if (!impl->ssrBlurVPipeline) NSLog(@"SSR blur V pipeline error: %@", error);
        }
    }

    // SSAO compute pipelines
    {
        id<MTLFunction> aoFunc = [library newFunctionWithName:@"gtaoCompute"];
        if (aoFunc) {
            impl->aoPipeline = [impl->device newComputePipelineStateWithFunction:aoFunc
                                                                           error:&error];
            if (!impl->aoPipeline) NSLog(@"GTAO pipeline error: %@", error);
        }
        id<MTLFunction> aoBlurHFunc = [library newFunctionWithName:@"aoBlurH"];
        if (aoBlurHFunc) {
            impl->aoBlurHPipeline = [impl->device newComputePipelineStateWithFunction:aoBlurHFunc
                                                                                error:&error];
            if (!impl->aoBlurHPipeline) NSLog(@"AO blur H pipeline error: %@", error);
        }
        id<MTLFunction> aoBlurVFunc = [library newFunctionWithName:@"aoBlurV"];
        if (aoBlurVFunc) {
            impl->aoBlurVPipeline = [impl->device newComputePipelineStateWithFunction:aoBlurVFunc
                                                                                error:&error];
            if (!impl->aoBlurVPipeline) NSLog(@"AO blur V pipeline error: %@", error);
        }
        id<MTLFunction> aoTemporalFunc = [library newFunctionWithName:@"aoTemporal"];
        if (aoTemporalFunc) {
            impl->aoTemporalPipeline = [impl->device newComputePipelineStateWithFunction:aoTemporalFunc
                                                                                  error:&error];
            if (!impl->aoTemporalPipeline) NSLog(@"AO temporal pipeline error: %@", error);
        }
    }

    // Bloom compute pipelines
    {
        id<MTLFunction> downFunc = [library newFunctionWithName:@"bloomDownsample"];
        if (downFunc) {
            impl->bloomDownsamplePipeline = [impl->device newComputePipelineStateWithFunction:downFunc
                                                                                         error:&error];
            if (!impl->bloomDownsamplePipeline) NSLog(@"Bloom downsample pipeline error: %@", error);
        }
        id<MTLFunction> upFunc = [library newFunctionWithName:@"bloomUpsample"];
        if (upFunc) {
            impl->bloomUpsamplePipeline = [impl->device newComputePipelineStateWithFunction:upFunc
                                                                                       error:&error];
            if (!impl->bloomUpsamplePipeline) NSLog(@"Bloom upsample pipeline error: %@", error);
        }
    }

    // DOF gather compute pipeline (virtual-camera plan Phase 4)
    {
        id<MTLFunction> dofFunc = [library newFunctionWithName:@"dofGather"];
        if (dofFunc) {
            impl->dofPipeline = [impl->device newComputePipelineStateWithFunction:dofFunc
                                                                            error:&error];
            if (!impl->dofPipeline) NSLog(@"DOF pipeline error: %@", error);
        }
    }

    // Generate BRDF integration LUT (one-time, 256×256)
    {
        MTLTextureDescriptor* brdfDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRG16Float
                                         width:256
                                        height:256
                                     mipmapped:NO];
        brdfDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
        brdfDesc.storageMode = MTLStorageModePrivate;
        impl->brdfLUT = [impl->device newTextureWithDescriptor:brdfDesc];

        if (impl->brdfLUTPipeline) {
            id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];
            id<MTLComputeCommandEncoder> enc = [cmdBuf computeCommandEncoder];
            [enc setComputePipelineState:impl->brdfLUTPipeline];
            [enc setTexture:impl->brdfLUT atIndex:0];
            MTLSize grid = MTLSizeMake(256, 256, 1);   // threads, not threadgroups
            MTLSize group = MTLSizeMake(16, 16, 1);
            [enc dispatchThreadgroups:threadgroupsCovering(grid, group)
                threadsPerThreadgroup:group];
            [enc endEncoding];
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
        }
    }

    // Probe buffer (GPU-side probe metadata)
    impl->probeBuffer = [impl->device newBufferWithLength:Impl::MAX_PROBES * sizeof(GPUReflectionProbe)
                                                  options:MTLResourceStorageModeShared];

    // Dummy 1×1 cubemap array for when no probes are baked (shader requires valid binding)
    {
        MTLTextureDescriptor* dummyCubeDesc = [[MTLTextureDescriptor alloc] init];
        dummyCubeDesc.textureType = MTLTextureTypeCubeArray;
        dummyCubeDesc.pixelFormat = MTLPixelFormatRGBA16Float;
        dummyCubeDesc.width = 1;
        dummyCubeDesc.height = 1;
        dummyCubeDesc.arrayLength = 1;
        dummyCubeDesc.mipmapLevelCount = 1;
        dummyCubeDesc.usage = MTLTextureUsageShaderRead;
        dummyCubeDesc.storageMode = MTLStorageModePrivate;
        impl->probeCubemapArray = [impl->device newTextureWithDescriptor:dummyCubeDesc];
    }

    // Dummy 1×1 cubemap for the procedural-sky binding (env cube is sampled only
    // when env.mode == 1, but a valid cube must always be bound to the skybox).
    {
        MTLTextureDescriptor* dc = [[MTLTextureDescriptor alloc] init];
        dc.textureType = MTLTextureTypeCube;
        dc.pixelFormat = MTLPixelFormatRGBA16Float;
        dc.width = 1;
        dc.height = 1;
        dc.mipmapLevelCount = 1;
        dc.usage = MTLTextureUsageShaderRead;
        dc.storageMode = MTLStorageModePrivate;
        impl->defaultCubemap = [impl->device newTextureWithDescriptor:dc];
    }

    // Instance data buffer
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; i++) {
        impl->instanceBuffers[i] = [impl->device newBufferWithLength:MAX_INSTANCES * sizeof(GPUInstanceData)
                                                            options:MTLResourceStorageModeShared];
        impl->shadowInstanceBuffers[i] =
            [impl->device newBufferWithLength:SHADOW_MAX_INSTANCES * sizeof(GPUInstanceData)
                                     options:MTLResourceStorageModeShared];
        impl->foliageInstanceBuffers[i] =
            [impl->device newBufferWithLength:FOLIAGE_MAX_INSTANCES * sizeof(GPUInstanceData)
                                     options:MTLResourceStorageModeShared];
    }

    // Light uniform buffer (exceeds setBytes 4KB limit with 32 lights)
    impl->lightBuffer = [impl->device newBufferWithLength:sizeof(LightUniforms)
                                                  options:MTLResourceStorageModeShared];

    // Shadow map: a depth-texture array, one slice per shadow cascade.
    {
        MTLTextureDescriptor* shadowDesc = [[MTLTextureDescriptor alloc] init];
        shadowDesc.textureType = MTLTextureType2DArray;
        shadowDesc.pixelFormat = MTLPixelFormatDepth32Float;
        shadowDesc.width = impl->shadowMapSize;
        shadowDesc.height = impl->shadowMapSize;
        shadowDesc.arrayLength = RT_MAX_CASCADES;
        shadowDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        shadowDesc.storageMode = MTLStorageModePrivate;
        impl->shadowMap = [impl->device newTextureWithDescriptor:shadowDesc];
    }

    // Shadow depth-only pipelines
    {
        id<MTLFunction> shadowVertFunc = [library newFunctionWithName:@"vertexShadow"];
        id<MTLFunction> shadowVertInstFunc = [library newFunctionWithName:@"vertexShadowInstanced"];

        MTLRenderPipelineDescriptor* shadowPipeDesc = [[MTLRenderPipelineDescriptor alloc] init];
        shadowPipeDesc.vertexFunction = shadowVertFunc;
        shadowPipeDesc.fragmentFunction = nil;
        shadowPipeDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
        // No color attachment for shadow pass

        impl->shadowPipeline = [impl->device newRenderPipelineStateWithDescriptor:shadowPipeDesc
                                                                            error:&error];
        if (!impl->shadowPipeline) {
            NSLog(@"Shadow pipeline error: %@", error);
        }

        shadowPipeDesc.vertexFunction = shadowVertInstFunc;
        impl->shadowInstancedPipeline = [impl->device newRenderPipelineStateWithDescriptor:shadowPipeDesc
                                                                                     error:&error];
        if (!impl->shadowInstancedPipeline) {
            NSLog(@"Shadow instanced pipeline error: %@", error);
        }

        // CDLOD terrain shadow caster (ADR-0036): morphs the caster like the
        // receiver so sun-facing slopes don't self-shadow. Absent function ->
        // terrain just won't cast (the loop checks the pipeline).
        id<MTLFunction> terrainShadowFunc = [library newFunctionWithName:@"terrainVertexShadow"];
        if (terrainShadowFunc) {
            shadowPipeDesc.vertexFunction = terrainShadowFunc;
            impl->terrainShadowPipeline =
                [impl->device newRenderPipelineStateWithDescriptor:shadowPipeDesc error:&error];
            if (!impl->terrainShadowPipeline)
                NSLog(@"Terrain shadow pipeline error: %@", error);
        }
    }

    // Shadow depth state
    {
        MTLDepthStencilDescriptor* shadowDepthDesc = [[MTLDepthStencilDescriptor alloc] init];
        shadowDepthDesc.depthCompareFunction = MTLCompareFunctionLess;
        shadowDepthDesc.depthWriteEnabled = YES;
        impl->shadowDepthState = [impl->device newDepthStencilStateWithDescriptor:shadowDepthDesc];
    }

    // Shadow comparison sampler
    {
        MTLSamplerDescriptor* samplerDesc = [[MTLSamplerDescriptor alloc] init];
        samplerDesc.compareFunction = MTLCompareFunctionLessEqual;
        samplerDesc.minFilter = MTLSamplerMinMagFilterLinear;
        samplerDesc.magFilter = MTLSamplerMinMagFilterLinear;
        samplerDesc.mipFilter = MTLSamplerMipFilterNotMipmapped;
        samplerDesc.sAddressMode = MTLSamplerAddressModeClampToEdge;
        samplerDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
        impl->shadowSampler = [impl->device newSamplerStateWithDescriptor:samplerDesc];
    }

    impl->shadowUniforms = {};
    impl->shadowUniforms.normalBias = 0.02f;
    impl->shadowUniforms.pcfRadius = 1.0f;
    impl->shadowUniforms.shadowMapSize = impl->shadowMapSize;
    impl->shadowUniforms.shadowStrength = 1.0f;
    impl->shadowUniforms.ambientStrength = 0.5f;

    // Depth states. The screen pass renders reverse-Z (ADR-0034 Phase 0): depth
    // clears to 0 (far) and nearer fragments have a Greater depth, so the test
    // is Greater (opaque/transparent) / GreaterEqual (overlay).
    MTLDepthStencilDescriptor* depthDesc = [[MTLDepthStencilDescriptor alloc] init];
    depthDesc.depthCompareFunction = MTLCompareFunctionGreater;
    depthDesc.depthWriteEnabled = YES;
    impl->depthStateOpaque = [impl->device newDepthStencilStateWithDescriptor:depthDesc];

    depthDesc.depthWriteEnabled = NO;
    impl->depthStateTransparent = [impl->device newDepthStencilStateWithDescriptor:depthDesc];

    // Foliage lit pass (depth prepass on): only the fragment whose depth equals
    // the prepass-written nearest leaf passes; no write (the prepass owns depth).
    depthDesc.depthCompareFunction = MTLCompareFunctionEqual;
    depthDesc.depthWriteEnabled = NO;
    impl->depthStateFoliageLit = [impl->device newDepthStencilStateWithDescriptor:depthDesc];
    depthDesc.depthCompareFunction = MTLCompareFunctionGreater;  // restore for the states below

    // Wireframe overlay: draw edges that pass an equal-or-nearer depth test
    // without writing depth, so lines sit on the visible surface they belong to.
    depthDesc.depthCompareFunction = MTLCompareFunctionGreaterEqual;
    depthDesc.depthWriteEnabled = NO;
    impl->depthStateWireOverlay = [impl->device newDepthStencilStateWithDescriptor:depthDesc];

    // Debug-gizmo overlay: always pass, never write — FLAG_OVERLAY draws sit on
    // top of the world geometry drawn before them in the pass.
    depthDesc.depthCompareFunction = MTLCompareFunctionAlways;
    depthDesc.depthWriteEnabled = NO;
    impl->depthStateOverlay = [impl->device newDepthStencilStateWithDescriptor:depthDesc];
    depthDesc.depthCompareFunction = MTLCompareFunctionGreaterEqual;   // restore for below

    // Forward-Z opaque state for the reflection-probe bake only: that pass owns
    // a self-contained depth buffer (clearDepth 1.0, DontCare store) with a
    // forward [0,1] cube-face projection and never feeds the reverse-Z screen
    // passes, so it keeps the conventional Less test.
    depthDesc.depthCompareFunction = MTLCompareFunctionLess;
    depthDesc.depthWriteEnabled = YES;
    impl->depthStateOpaqueForwardZ = [impl->device newDepthStencilStateWithDescriptor:depthDesc];

    resize(width, height);
    return true;
}

void MetalRenderer::shutdown() {
    impl->meshes.clear();
    impl->device = nil;
}

void MetalRenderer::resize(int width, int height) {
    impl->framebufferWidth = width;
    impl->framebufferHeight = height;
    impl->surface->resize(width, height);

    MTLTextureDescriptor* depthDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                     width:width
                                    height:height
                                 mipmapped:NO];
    depthDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    depthDesc.storageMode = MTLStorageModePrivate;
    impl->depthTexture = [impl->device newTextureWithDescriptor:depthDesc];

    // Offscreen HDR scene color target (Phase 0C)
    MTLTextureDescriptor* sceneColorDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:width
                                    height:height
                                 mipmapped:NO];
    sceneColorDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    sceneColorDesc.storageMode = MTLStorageModePrivate;
    impl->sceneColorTexture = [impl->device newTextureWithDescriptor:sceneColorDesc];

    // View-space normal G-buffer for SSR
    MTLTextureDescriptor* normalDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:width
                                    height:height
                                 mipmapped:NO];
    normalDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    normalDesc.storageMode = MTLStorageModePrivate;
    impl->viewNormalTexture = [impl->device newTextureWithDescriptor:normalDesc];

    // SSR textures (half resolution for performance)
    int halfW = std::max(width / 2, 1);
    int halfH = std::max(height / 2, 1);
    MTLTextureDescriptor* ssrDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:halfW
                                    height:halfH
                                 mipmapped:NO];
    ssrDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
    ssrDesc.storageMode = MTLStorageModePrivate;
    impl->ssrTexture = [impl->device newTextureWithDescriptor:ssrDesc];
    impl->ssrBlurTemp = [impl->device newTextureWithDescriptor:ssrDesc];

    // AO textures (half resolution — SSAO is the dominant frame cost at full res;
    // ¼ the pixels across all four AO passes, upsampled bilinearly in the
    // composite. The AO kernels map their half-res coords to the full-res
    // depth/normal G-buffer.)
    MTLTextureDescriptor* aoDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatR16Float
                                     width:halfW
                                    height:halfH
                                 mipmapped:NO];
    aoDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
    aoDesc.storageMode = MTLStorageModePrivate;
    impl->aoTexture = [impl->device newTextureWithDescriptor:aoDesc];
    impl->aoBlurTemp = [impl->device newTextureWithDescriptor:aoDesc];
    impl->aoHistory = [impl->device newTextureWithDescriptor:aoDesc];
    impl->aoHistoryValid = false;   // contents undefined until the first resolve

    // Bloom mip chain (progressive half-res)
    {
        int mipW = halfW;
        int mipH = halfH;
        for (int m = 0; m < MetalRenderer::Impl::BLOOM_MIP_COUNT; m++) {
            MTLTextureDescriptor* bloomDesc = [MTLTextureDescriptor
                texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                             width:std::max(mipW, 1)
                                            height:std::max(mipH, 1)
                                         mipmapped:NO];
            bloomDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
            bloomDesc.storageMode = MTLStorageModePrivate;
            impl->bloomMips[m] = [impl->device newTextureWithDescriptor:bloomDesc];
            impl->bloomUpsampleMips[m] = [impl->device newTextureWithDescriptor:bloomDesc];
            mipW = std::max(mipW / 2, 1);
            mipH = std::max(mipH / 2, 1);
        }
    }

    // Lens effects (virtual-camera plan Phase 4): composite target for frames
    // where the lens-warp pass owns the drawable, + DOF gather output.
    //
    // postLDRTexture deliberately MATCHES THE SURFACE FORMAT. That makes the
    // display transform land exactly once whether or not the lens pass runs:
    // on a linear-storage target the composite encodes and the lens pass copies
    // encoded bytes through; on an sRGB target the composite stays linear, the
    // hardware encodes on write, sampling decodes back to linear for the lens
    // pass, and the hardware encodes once more on the final write — a net single
    // encode either way. Hardcoding BGRA8Unorm here would double-encode on
    // visionOS the moment lens effects were switched on.
    {
        MTLTextureDescriptor* postDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:impl->surface->colorPixelFormat()
                                         width:width
                                        height:height
                                     mipmapped:NO];
        postDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        postDesc.storageMode = MTLStorageModePrivate;
        impl->postLDRTexture = [impl->device newTextureWithDescriptor:postDesc];

        MTLTextureDescriptor* dofDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                         width:width
                                        height:height
                                     mipmapped:NO];
        dofDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
        dofDesc.storageMode = MTLStorageModePrivate;
        impl->dofTexture = [impl->device newTextureWithDescriptor:dofDesc];
    }
}

MeshHandle MetalRenderer::uploadMesh(const RenderMesh& mesh) {
    GPUMesh gpuMesh;
    gpuMesh.materialIndex = mesh.materialIndex;

    // Layout must match `struct Vertex` in shaders/metal/common.metal.
    struct GPUVertex {
        float position[3];
        float normal[3];
        float tangent[3];
        float texcoord[2];
        float color[3];
    };

    std::vector<GPUVertex> gpuVertices(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        gpuVertices[i] = {
            {static_cast<float>(mesh.vertices[i].position.x),
             static_cast<float>(mesh.vertices[i].position.y),
             static_cast<float>(mesh.vertices[i].position.z)},
            {static_cast<float>(mesh.vertices[i].normal.x),
             static_cast<float>(mesh.vertices[i].normal.y),
             static_cast<float>(mesh.vertices[i].normal.z)},
            {static_cast<float>(mesh.vertices[i].tangent.x),
             static_cast<float>(mesh.vertices[i].tangent.y),
             static_cast<float>(mesh.vertices[i].tangent.z)},
            {mesh.vertices[i].u, mesh.vertices[i].v},
            {static_cast<float>(mesh.vertices[i].color.x),
             static_cast<float>(mesh.vertices[i].color.y),
             static_cast<float>(mesh.vertices[i].color.z)}
        };
    }

    gpuMesh.vertexBuffer = [impl->device newBufferWithBytes:gpuVertices.data()
                                                     length:gpuVertices.size() * sizeof(GPUVertex)
                                                    options:MTLResourceStorageModeShared];

    gpuMesh.indexBuffer = [impl->device newBufferWithBytes:mesh.indices.data()
                                                    length:mesh.indices.size() * sizeof(uint32_t)
                                                   options:MTLResourceStorageModeShared];

    gpuMesh.indexCount = static_cast<uint32_t>(mesh.indices.size());
    gpuMesh.bounds = computeBoundingSphere(mesh.vertices.data(), mesh.vertices.size());

    return impl->meshes.insert(gpuMesh);
}

void MetalRenderer::removeMesh(MeshHandle handle) {
    impl->meshes.erase(handle);
}

TextureHandle MetalRenderer::uploadTexture(int width, int height, int channels,
                                            const uint8_t* data) {
    MTLTextureDescriptor* desc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:width height:height mipmapped:YES];
    desc.usage = MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModeShared;

    id<MTLTexture> texture = [impl->device newTextureWithDescriptor:desc];

    if (channels == 4) {
        [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
                   mipmapLevel:0
                     withBytes:data
                   bytesPerRow:width * 4];
    } else {
        std::vector<uint8_t> rgba(width * height * 4);
        for (int i = 0; i < width * height; i++) {
            rgba[i * 4 + 0] = (channels > 0) ? data[i * channels + 0] : 255;
            rgba[i * 4 + 1] = (channels > 1) ? data[i * channels + 1] : 255;
            rgba[i * 4 + 2] = (channels > 2) ? data[i * channels + 2] : 255;
            rgba[i * 4 + 3] = (channels > 3) ? data[i * channels + 3] : 255;
        }
        [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
                   mipmapLevel:0
                     withBytes:rgba.data()
                   bytesPerRow:width * 4];
    }

    // Generate mipmaps
    id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmdBuf blitCommandEncoder];
    [blit generateMipmapsForTexture:texture];
    [blit endEncoding];
    [cmdBuf commit];
    [cmdBuf waitUntilCompleted];

    return impl->textures.insert(texture);
}

TextureHandle MetalRenderer::uploadTextureHDR(int width, int height, int channels,
                                              const float* data) {
    if (!data || width <= 0 || height <= 0) return TextureHandle{};

    // Mipmapped so the lit pass can sample blurred levels for HDR image-based
    // lighting (roughness-scaled specular + a high mip approximating irradiance).
    MTLTextureDescriptor* desc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:width height:height mipmapped:YES];
    desc.usage = MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModeShared;
    id<MTLTexture> texture = [impl->device newTextureWithDescriptor:desc];

    // Expand to RGBA half-float; Metal has no 3-channel float texture format.
    const int n = width * height;
    std::vector<__fp16> rgba(static_cast<size_t>(n) * 4);
    for (int i = 0; i < n; i++) {
        const float* src = data + static_cast<size_t>(i) * channels;
        rgba[i * 4 + 0] = static_cast<__fp16>(channels > 0 ? src[0] : 0.0f);
        rgba[i * 4 + 1] = static_cast<__fp16>(channels > 1 ? src[1] : 0.0f);
        rgba[i * 4 + 2] = static_cast<__fp16>(channels > 2 ? src[2] : 0.0f);
        rgba[i * 4 + 3] = static_cast<__fp16>(channels > 3 ? src[3] : 1.0f);
    }
    [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
               mipmapLevel:0
                 withBytes:rgba.data()
               bytesPerRow:static_cast<NSUInteger>(width) * 4 * sizeof(__fp16)];

    // Build the mip chain for IBL roughness/irradiance lookups.
    if (texture.mipmapLevelCount > 1) {
        id<MTLCommandBuffer> cmd = [impl->commandQueue commandBuffer];
        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit generateMipmapsForTexture:texture];
        [blit endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];
    }

    return impl->textures.insert(texture);
}

void MetalRenderer::removeTexture(TextureHandle handle) {
    impl->textures.erase(handle);
}

// Bake the equirect HDR into a mipmapped cubemap once at load, so the skybox,
// composite, and probe bake do a cheap cube lookup instead of per-sample equirect
// math (ADR-0016). Renders 6 faces with the probe bake's per-face cameras (the
// verified cube convention), blits each into a cube slice, then builds the mips.
void MetalRenderer::Impl::bakeEnvironmentCubemap() {
    if (!environmentTexture || !equirectBakePipeline) {
        environmentCubemap = nil;
        return;
    }
    const int size = ENV_CUBEMAP_SIZE;
    NSUInteger mipCount = 1;
    for (int s = size; s > 1; s >>= 1) mipCount++;

    MTLTextureDescriptor* cubeDesc = [[MTLTextureDescriptor alloc] init];
    cubeDesc.textureType = MTLTextureTypeCube;
    cubeDesc.pixelFormat = MTLPixelFormatRGBA16Float;
    cubeDesc.width = size;
    cubeDesc.height = size;
    cubeDesc.mipmapLevelCount = mipCount;
    cubeDesc.usage = MTLTextureUsageShaderRead;
    cubeDesc.storageMode = MTLStorageModePrivate;
    id<MTLTexture> cube = [device newTextureWithDescriptor:cubeDesc];

    // Per-face temp render target, blitted into the cube (mirrors the probe bake).
    MTLTextureDescriptor* faceDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:size height:size mipmapped:NO];
    faceDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    faceDesc.storageMode = MTLStorageModePrivate;

    for (int face = 0; face < 6; face++) {
        id<MTLTexture> faceColor = [device newTextureWithDescriptor:faceDesc];

        // Face cameras from the unit-tested convention helper (ADR-0017
        // Phase 3) — the previous lookAt-only table produced mirrored faces.
        CameraUniforms cam = {};
        cam.viewProjection = toSimd(cubeFaceViewProjection(face, Vec3(0, 0, 0), 0.1, 10.0));
        cam.invViewProjection = simd_inverse(cam.viewProjection);
        cam.cameraPosition = simd_make_float3(0.0f, 0.0f, 0.0f);  // env is position-independent

        id<MTLCommandBuffer> cmd = [commandQueue commandBuffer];
        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = faceColor;
        rp.colorAttachments[0].loadAction = MTLLoadActionClear;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);

        id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
        [enc setRenderPipelineState:equirectBakePipeline];
        [enc setVertexBytes:&cam length:sizeof(CameraUniforms) atIndex:1];
        [enc setFragmentTexture:environmentTexture atIndex:0];
        [enc setFragmentSamplerState:equirectSampler atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];

        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit copyFromTexture:faceColor
                  sourceSlice:0 sourceLevel:0
                 sourceOrigin:MTLOriginMake(0, 0, 0)
                   sourceSize:MTLSizeMake(size, size, 1)
                    toTexture:cube
             destinationSlice:face destinationLevel:0
            destinationOrigin:MTLOriginMake(0, 0, 0)];
        [blit endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];
    }

    // Mip chain so rough/distant lookups are cheap.
    {
        id<MTLCommandBuffer> cmd = [commandQueue commandBuffer];
        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit generateMipmapsForTexture:cube];
        [blit endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];
    }

    validateBakedCube(cube);
    environmentCubemap = cube;
}

void MetalRenderer::Impl::validateBakedCube(id<MTLTexture> cube) {
    const int size = ENV_CUBEMAP_SIZE;
    const int ew = static_cast<int>(environmentTexture.width);
    const int eh = static_cast<int>(environmentTexture.height);

    // CPU copy of the equirect source (uploaded with shared storage).
    std::vector<__fp16> equirect(static_cast<size_t>(ew) * eh * 4);
    [environmentTexture getBytes:equirect.data()
                     bytesPerRow:static_cast<NSUInteger>(ew) * 4 * sizeof(__fp16)
                      fromRegion:MTLRegionMake2D(0, 0, ew, eh)
                     mipmapLevel:0];

    auto equirectLum = [&](const Vec3& d) -> double {
        double u = std::atan2(d.z, d.x) / (2.0 * M_PI) + 0.5;
        double v = std::acos(std::clamp(d.y, Real(-1), Real(1))) / M_PI;
        int x = std::clamp(static_cast<int>(u * ew), 0, ew - 1);
        int y = std::clamp(static_cast<int>(v * eh), 0, eh - 1);
        const __fp16* p = &equirect[(static_cast<size_t>(y) * ew + x) * 4];
        return 0.2126 * static_cast<float>(p[0]) + 0.7152 * static_cast<float>(p[1])
             + 0.0722 * static_cast<float>(p[2]);
    };

    // Compare at low frequency: a deep cube mip vs jitter-averaged equirect,
    // so filtering differences don't drown the orientation signal. Test all
    // eight axis-aligned orientations (flips and transposes).
    const int MIP = 4;
    const int faceSize = size >> MIP;
    MTLTextureDescriptor* stageDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:faceSize height:faceSize mipmapped:NO];
    stageDesc.storageMode = MTLStorageModeShared;
    id<MTLTexture> staging = [device newTextureWithDescriptor:stageDesc];
    std::vector<__fp16> faceData(static_cast<size_t>(faceSize) * faceSize * 4);

    // Jitter-averaged equirect luminance around a face texel's footprint.
    auto expectedLum = [&](int face, double u, double v) -> double {
        double sum = 0.0;
        const int J = 4;
        for (int jy = 0; jy < J; jy++)
            for (int jx = 0; jx < J; jx++) {
                double uu = u + ((jx + 0.5) / J - 0.5) / faceSize;
                double vv = v + ((jy + 0.5) / J - 0.5) / faceSize;
                sum += equirectLum(cubeFaceDirection(face, uu, vv));
            }
        return sum / (J * J);
    };

    static const char* HYP[8] = {"identity", "u-flip", "v-flip", "uv-flip (180)",
                                 "transpose", "rot90", "rot270", "anti-transpose"};
    bool allOk = true;
    for (int face = 0; face < 6; face++) {
        id<MTLCommandBuffer> cmd = [commandQueue commandBuffer];
        id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
        [blit copyFromTexture:cube
                  sourceSlice:static_cast<NSUInteger>(face) sourceLevel:MIP
                 sourceOrigin:MTLOriginMake(0, 0, 0)
                   sourceSize:MTLSizeMake(faceSize, faceSize, 1)
                    toTexture:staging
             destinationSlice:0 destinationLevel:0
            destinationOrigin:MTLOriginMake(0, 0, 0)];
        [blit endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];
        [staging getBytes:faceData.data()
              bytesPerRow:static_cast<NSUInteger>(faceSize) * 4 * sizeof(__fp16)
               fromRegion:MTLRegionMake2D(0, 0, faceSize, faceSize)
              mipmapLevel:0];

        double err[8] = {};
        for (int ty = 0; ty < faceSize; ty += 4) {
            for (int tx = 0; tx < faceSize; tx += 4) {
                double u = (tx + 0.5) / faceSize;
                double v = (ty + 0.5) / faceSize;
                const __fp16* p = &faceData[(static_cast<size_t>(ty) * faceSize + tx) * 4];
                double baked = 0.2126 * static_cast<float>(p[0])
                             + 0.7152 * static_cast<float>(p[1])
                             + 0.0722 * static_cast<float>(p[2]);
                for (int h = 0; h < 8; h++) {
                    double uu = u, vv = v;
                    if (h & 4) std::swap(uu, vv);
                    if (h & 1) uu = 1.0 - uu;
                    if (h & 2) vv = 1.0 - vv;
                    double expected = expectedLum(face, uu, vv);
                    err[h] += std::abs(baked - expected) / (expected + 0.05);
                }
            }
        }
        int best = 0;
        for (int h = 1; h < 8; h++)
            if (err[h] < err[best]) best = h;
        NSLog(@"[ENV BAKE] face %d: best=%s  errs id=%.2f uf=%.2f vf=%.2f 180=%.2f tr=%.2f r90=%.2f r270=%.2f at=%.2f",
              face, HYP[best], err[0], err[1], err[2], err[3], err[4], err[5], err[6], err[7]);
        if (best != 0) allOk = false;
    }
    if (allOk) {
        NSLog(@"[ENV BAKE] all 6 cube faces match the sampling convention");
    }

    // RT_DUMP_ENV=<dir>: write tone-mapped PNGs of each face (mip 2) and the
    // equirect source for direct visual comparison.
    if (const char* dumpDir = std::getenv("RT_DUMP_ENV")) {
        auto tonemap8 = [](float x) -> unsigned char {
            float t = x / (1.0f + x);
            float g = std::pow(std::max(t, 0.0f), 1.0f / 2.2f);
            return static_cast<unsigned char>(std::min(g, 1.0f) * 255.0f + 0.5f);
        };
        const int DM = 2;                  // mip 2 → 256² faces
        const int ds = size >> DM;
        MTLTextureDescriptor* dDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                         width:ds height:ds mipmapped:NO];
        dDesc.storageMode = MTLStorageModeShared;
        id<MTLTexture> dStage = [device newTextureWithDescriptor:dDesc];
        std::vector<__fp16> dData(static_cast<size_t>(ds) * ds * 4);
        std::vector<unsigned char> png(static_cast<size_t>(ds) * ds * 3);
        for (int face = 0; face < 6; face++) {
            id<MTLCommandBuffer> cmd = [commandQueue commandBuffer];
            id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
            [blit copyFromTexture:cube
                      sourceSlice:static_cast<NSUInteger>(face) sourceLevel:DM
                     sourceOrigin:MTLOriginMake(0, 0, 0)
                       sourceSize:MTLSizeMake(ds, ds, 1)
                        toTexture:dStage
                 destinationSlice:0 destinationLevel:0
                destinationOrigin:MTLOriginMake(0, 0, 0)];
            [blit endEncoding];
            [cmd commit];
            [cmd waitUntilCompleted];
            [dStage getBytes:dData.data()
                 bytesPerRow:static_cast<NSUInteger>(ds) * 4 * sizeof(__fp16)
                  fromRegion:MTLRegionMake2D(0, 0, ds, ds)
                 mipmapLevel:0];
            for (size_t i = 0, j = 0; i < dData.size(); i += 4, j += 3) {
                png[j]     = tonemap8(dData[i]);
                png[j + 1] = tonemap8(dData[i + 1]);
                png[j + 2] = tonemap8(dData[i + 2]);
            }
            char path[512];
            snprintf(path, sizeof(path), "%s/cube_face_%d.png", dumpDir, face);
            stbi_write_png(path, ds, ds, 3, png.data(), ds * 3);
        }
        // Equirect source, downsampled 2× for a manageable PNG.
        const int qw = ew / 2, qh = eh / 2;
        std::vector<unsigned char> eq(static_cast<size_t>(qw) * qh * 3);
        for (int y = 0; y < qh; y++)
            for (int x = 0; x < qw; x++) {
                const __fp16* p = &equirect[(static_cast<size_t>(y) * 2 * ew + x * 2) * 4];
                size_t j = (static_cast<size_t>(y) * qw + x) * 3;
                eq[j]     = tonemap8(p[0]);
                eq[j + 1] = tonemap8(p[1]);
                eq[j + 2] = tonemap8(p[2]);
            }
        char path[512];
        snprintf(path, sizeof(path), "%s/equirect.png", dumpDir);
        stbi_write_png(path, qw, qh, 3, eq.data(), qw * 3);
        NSLog(@"[ENV DUMP] wrote cube faces + equirect to %s", dumpDir);
    }

    // Link probe: GPU direction reconstruction. Renders each face writing the
    // reconstructed direction as color; compares numerically against the
    // convention table. Isolates rasterizer/invVP from equirect sampling.
    if (dirDebugPipeline) {
        const int DS = 32;
        MTLTextureDescriptor* dd = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                         width:DS height:DS mipmapped:NO];
        dd.usage = MTLTextureUsageRenderTarget;
        dd.storageMode = MTLStorageModeShared;
        id<MTLTexture> dirTex = [device newTextureWithDescriptor:dd];
        std::vector<__fp16> dirData(static_cast<size_t>(DS) * DS * 4);
        for (int face = 0; face < 6; face++) {
            CameraUniforms cam = {};
            cam.viewProjection = toSimd(cubeFaceViewProjection(face, Vec3(0, 0, 0), 0.1, 10.0));
            cam.invViewProjection = simd_inverse(cam.viewProjection);
            cam.cameraPosition = simd_make_float3(0, 0, 0);

            id<MTLCommandBuffer> cmd = [commandQueue commandBuffer];
            MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
            rp.colorAttachments[0].texture = dirTex;
            rp.colorAttachments[0].loadAction = MTLLoadActionClear;
            rp.colorAttachments[0].storeAction = MTLStoreActionStore;
            id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:rp];
            [enc setRenderPipelineState:dirDebugPipeline];
            [enc setVertexBytes:&cam length:sizeof(CameraUniforms) atIndex:1];
            [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
            [enc endEncoding];
            [cmd commit];
            [cmd waitUntilCompleted];
            [dirTex getBytes:dirData.data()
                 bytesPerRow:static_cast<NSUInteger>(DS) * 4 * sizeof(__fp16)
                  fromRegion:MTLRegionMake2D(0, 0, DS, DS)
                 mipmapLevel:0];

            double maxAngErr = 0.0;
            for (int ty = 0; ty < DS; ty++)
                for (int tx = 0; tx < DS; tx++) {
                    const __fp16* p = &dirData[(static_cast<size_t>(ty) * DS + tx) * 4];
                    Vec3 gpuDir(static_cast<float>(p[0]) * 2.0f - 1.0f,
                                static_cast<float>(p[1]) * 2.0f - 1.0f,
                                static_cast<float>(p[2]) * 2.0f - 1.0f);
                    gpuDir = normalize(gpuDir);
                    Vec3 want = cubeFaceDirection(face, (tx + 0.5) / DS, (ty + 0.5) / DS);
                    double d = std::clamp(dot(gpuDir, want), Real(-1), Real(1));
                    maxAngErr = std::max(maxAngErr, std::acos(d) * 180.0 / M_PI);
                }
            NSLog(@"[ENV DIR] face %d: max GPU-vs-expected direction error %.2f deg",
                  face, maxAngErr);
        }
    }

    // Second link: the hardware sampler. Sample the cube along directions from
    // the convention table and compare against the (jitter-averaged) equirect.
    if (cubeValidatePipeline) {
        const int GRID = 8;
        const int N = 6 * GRID * GRID;
        std::vector<simd_float4> dirs(N);
        int idx = 0;
        for (int face = 0; face < 6; face++)
            for (int gy = 0; gy < GRID; gy++)
                for (int gx = 0; gx < GRID; gx++) {
                    Vec3 d = cubeFaceDirection(face, (gx + 0.5) / GRID,
                                               (gy + 0.5) / GRID);
                    dirs[idx++] = simd_make_float4(static_cast<float>(d.x),
                                                   static_cast<float>(d.y),
                                                   static_cast<float>(d.z), 0);
                }
        id<MTLBuffer> dirBuf = [device newBufferWithBytes:dirs.data()
                                                   length:N * sizeof(simd_float4)
                                                  options:MTLResourceStorageModeShared];
        id<MTLBuffer> resBuf = [device newBufferWithLength:N * sizeof(simd_float4)
                                                   options:MTLResourceStorageModeShared];
        id<MTLCommandBuffer> cmd = [commandQueue commandBuffer];
        id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
        [enc setComputePipelineState:cubeValidatePipeline];
        [enc setTexture:cube atIndex:0];
        [enc setBuffer:dirBuf offset:0 atIndex:0];
        [enc setBuffer:resBuf offset:0 atIndex:1];
        [enc setSamplerState:mipClampSampler atIndex:0];
        uint32_t sampleCount = static_cast<uint32_t>(N);
        [enc setBytes:&sampleCount length:sizeof(sampleCount) atIndex:2];
        MTLSize validateGroup = MTLSizeMake(64, 1, 1);
        [enc dispatchThreadgroups:threadgroupsCovering(MTLSizeMake(N, 1, 1), validateGroup)
            threadsPerThreadgroup:validateGroup];
        [enc endEncoding];
        [cmd commit];
        [cmd waitUntilCompleted];

        const simd_float4* res = static_cast<const simd_float4*>([resBuf contents]);
        idx = 0;
        for (int face = 0; face < 6; face++) {
            double errSum = 0.0;
            for (int g = 0; g < GRID * GRID; g++, idx++) {
                double sampled = 0.2126 * res[idx].x + 0.7152 * res[idx].y
                               + 0.0722 * res[idx].z;
                double u = (g % GRID + 0.5) / GRID;
                double v = (g / GRID + 0.5) / GRID;
                double expected = expectedLum(face, u, v);
                errSum += std::abs(sampled - expected) / (expected + 0.05);
            }
            NSLog(@"[ENV SAMPLE] face %d hardware-sample mean rel err %.3f",
                  face, errSum / (GRID * GRID));
        }
    }
}

// Build the IBL cubes from the baked environment cube (ADR-0017 Phase 3):
// GGX-prefilter each mip (roughness = mip / maxMip) and cosine-convolve a
// small irradiance cube. One command buffer; the kernels only read the source
// cube, so no inter-dispatch barriers are needed.
void MetalRenderer::Impl::bakeEnvironmentIBL() {
    if (!environmentCubemap || !prefilterPipeline || !irradiancePipeline) {
        envPrefilteredCube = nil;
        envIrradianceCube = nil;
        return;
    }

    MTLTextureDescriptor* preDesc = [[MTLTextureDescriptor alloc] init];
    preDesc.textureType = MTLTextureTypeCube;
    preDesc.pixelFormat = MTLPixelFormatRGBA16Float;
    preDesc.width = ENV_PREFILTER_SIZE;
    preDesc.height = ENV_PREFILTER_SIZE;
    preDesc.mipmapLevelCount = ENV_PREFILTER_MIPS;
    preDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
    preDesc.storageMode = MTLStorageModePrivate;
    id<MTLTexture> prefiltered = [device newTextureWithDescriptor:preDesc];

    MTLTextureDescriptor* irrDesc = [[MTLTextureDescriptor alloc] init];
    irrDesc.textureType = MTLTextureTypeCube;
    irrDesc.pixelFormat = MTLPixelFormatRGBA16Float;
    irrDesc.width = ENV_IRRADIANCE_SIZE;
    irrDesc.height = ENV_IRRADIANCE_SIZE;
    irrDesc.mipmapLevelCount = 1;
    irrDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
    irrDesc.storageMode = MTLStorageModePrivate;
    id<MTLTexture> irradiance = [device newTextureWithDescriptor:irrDesc];

    id<MTLCommandBuffer> cmd = [commandQueue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
    MTLSize group = MTLSizeMake(8, 8, 1);

    [enc setComputePipelineState:prefilterPipeline];
    [enc setTexture:environmentCubemap atIndex:0];
    [enc setSamplerState:mipClampSampler atIndex:0];
    for (int mip = 0; mip < ENV_PREFILTER_MIPS; mip++) {
        id<MTLTexture> mipView =
            [prefiltered newTextureViewWithPixelFormat:MTLPixelFormatRGBA16Float
                                           textureType:MTLTextureTypeCube
                                                levels:NSMakeRange(mip, 1)
                                                slices:NSMakeRange(0, 6)];
        float roughness = static_cast<float>(mip) / (ENV_PREFILTER_MIPS - 1);
        int mipSize = ENV_PREFILTER_SIZE >> mip;
        MTLSize grid = MTLSizeMake(mipSize, mipSize, 1);
        [enc setTexture:mipView atIndex:1];
        [enc setBytes:&roughness length:sizeof(roughness) atIndex:0];
        for (int face = 0; face < 6; face++) {
            [enc setBytes:&face length:sizeof(face) atIndex:1];
            [enc dispatchThreadgroups:threadgroupsCovering(grid, group)
                threadsPerThreadgroup:group];
        }
    }

    [enc setComputePipelineState:irradiancePipeline];
    [enc setTexture:environmentCubemap atIndex:0];
    [enc setTexture:irradiance atIndex:1];
    [enc setSamplerState:mipClampSampler atIndex:0];
    MTLSize irrGrid = MTLSizeMake(ENV_IRRADIANCE_SIZE, ENV_IRRADIANCE_SIZE, 1);
    for (int face = 0; face < 6; face++) {
        [enc setBytes:&face length:sizeof(face) atIndex:0];
        [enc dispatchThreadgroups:threadgroupsCovering(irrGrid, group)
            threadsPerThreadgroup:group];
    }

    [enc endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    // Diagnostic scan: non-finite texels mean a kernel bug; an extreme max
    // luminance means the source sun is aliasing into fireflies.
    {
        std::vector<__fp16> texels;
        for (int mip = 0; mip < ENV_PREFILTER_MIPS; mip++) {
            int mipSize = ENV_PREFILTER_SIZE >> mip;
            MTLTextureDescriptor* stageDesc = [MTLTextureDescriptor
                texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                             width:mipSize height:mipSize mipmapped:NO];
            stageDesc.storageMode = MTLStorageModeShared;
            id<MTLTexture> staging = [device newTextureWithDescriptor:stageDesc];
            texels.resize(static_cast<size_t>(mipSize) * mipSize * 4);
            int badCount = 0;
            float maxLum = 0;
            for (int face = 0; face < 6; face++) {
                id<MTLCommandBuffer> scanCmd = [commandQueue commandBuffer];
                id<MTLBlitCommandEncoder> blit = [scanCmd blitCommandEncoder];
                [blit copyFromTexture:prefiltered
                          sourceSlice:static_cast<NSUInteger>(face) sourceLevel:static_cast<NSUInteger>(mip)
                         sourceOrigin:MTLOriginMake(0, 0, 0)
                           sourceSize:MTLSizeMake(mipSize, mipSize, 1)
                            toTexture:staging
                     destinationSlice:0 destinationLevel:0
                    destinationOrigin:MTLOriginMake(0, 0, 0)];
                [blit endEncoding];
                [scanCmd commit];
                [scanCmd waitUntilCompleted];
                [staging getBytes:texels.data()
                      bytesPerRow:static_cast<NSUInteger>(mipSize) * 4 * sizeof(__fp16)
                       fromRegion:MTLRegionMake2D(0, 0, mipSize, mipSize)
                      mipmapLevel:0];
                for (size_t i = 0; i < texels.size(); i += 4) {
                    float r = texels[i], g = texels[i + 1], b = texels[i + 2];
                    if (!std::isfinite(r) || !std::isfinite(g) || !std::isfinite(b)) {
                        badCount++;
                        continue;
                    }
                    maxLum = std::max(maxLum, 0.2126f * r + 0.7152f * g + 0.0722f * b);
                }
            }
            if (badCount > 0)
                NSLog(@"[ENV IBL] prefiltered mip %d: %d non-finite texels!", mip, badCount);
            else
                NSLog(@"[ENV IBL] prefiltered mip %d ok, max luminance %.1f", mip, maxLum);
        }
    }

    envPrefilteredCube = prefiltered;
    envIrradianceCube = irradiance;
}

void MetalRenderer::setEnvironmentMap(TextureHandle equirect) {
    auto* tex = impl->textures.get(equirect);
    impl->environmentTexture = tex ? *tex : nil;
    impl->bakeEnvironmentCubemap();   // builds environmentCubemap (or clears it)
    impl->bakeEnvironmentIBL();       // prefiltered + irradiance cubes (or clears)

    // If probes are already baked against the old environment, re-bake so IBL
    // tracks the new map (ADR-0016: the bake renders the skybox into the cubes).
    if (!impl->pendingProbes.empty()) {
        impl->probesBaked = false;
        impl->probesPendingBake = true;
    }
}

BoundingSphere MetalRenderer::getMeshBounds(MeshHandle handle) const {
    const GPUMesh* mesh = impl->meshes.get(handle);
    if (!mesh) return {};
    return mesh->bounds;
}

RenderStats MetalRenderer::getRenderStats() const {
    return impl->lastStats;
}

void MetalRenderer::beginFrame() {
    impl->opaqueDrawCalls.clear();
    impl->transparentDrawCalls.clear();
    impl->overlayDrawCalls.clear();
    impl->terrainDrawCalls.clear();
    // Advance the dynamic-buffer ring so this frame writes a slot the GPU isn't
    // still reading for an in-flight earlier frame (fixes instance tearing —
    // trees popping to a neighbor's transform for a frame).
    impl->frameIndex = (impl->frameIndex + 1) % MAX_FRAMES_IN_FLIGHT;
    impl->frameCount++;

    // Acquire the drawable and build the pass descriptor up front so the debug
    // UI's new-frame (which needs the descriptor's formats) can run before
    // systems emit ImGui widgets in their render() hooks. endFrame() reuses it.
    impl->surface->setXrWorldScale(xrWorldScale);
    impl->currentColorTarget = impl->surface->acquire() ? impl->surface->colorTarget() : nil;

    // Keep the offscreen targets the same size as the thing we present to.
    //
    // The composite pass reads depth with depthTex.read(in.position.xy) — i.e.
    // in TARGET pixel coordinates — so if the depth texture is smaller than the
    // colour target, every fragment past its edge reads out of bounds, gets 0,
    // and is classified as reverse-Z background. The symptom is a hard-edged
    // black region exactly at the depth texture's width, which reads as missing
    // geometry rather than as a sizing bug.
    //
    // Desktop never hit this because the window drives both. visionOS has no
    // window: HostedWindow reports a nominal size and the compositor picks the
    // real per-eye dimensions, so the two only agree if we ask.
    if (impl->currentColorTarget) {
        int targetWidth = 0, targetHeight = 0;
        if (impl->surface->drawableSize(targetWidth, targetHeight) &&
            (targetWidth != impl->framebufferWidth ||
             targetHeight != impl->framebufferHeight)) {
            resize(targetWidth, targetHeight);
        }
    }

    // Head-tracked surfaces: re-derive the camera from the pose acquire() just
    // captured. The game only calls setCamera when ITS camera changes; a
    // static game camera must not freeze the user's head. Non-tracking
    // surfaces (macOS) never enter here.
    if (impl->hasLastCamera && impl->surface->xrTracking())
        setCamera(impl->lastCameraState);

    impl->currentPassDesc = nil;
    impl->compositePassDesc = nil;
    if (impl->currentColorTarget) {
        // Main scene pass renders to offscreen HDR texture (not the drawable).
        // Tone mapping + gamma happens in a separate composite pass.
        MTLRenderPassDescriptor* passDesc = [MTLRenderPassDescriptor renderPassDescriptor];
        passDesc.colorAttachments[0].texture = impl->sceneColorTexture;
        passDesc.colorAttachments[0].loadAction = MTLLoadActionClear;
        passDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
        passDesc.colorAttachments[0].clearColor = MTLClearColorMake(0.0, 0.0, 0.0, 1.0);
        passDesc.colorAttachments[1].texture = impl->viewNormalTexture;
        passDesc.colorAttachments[1].loadAction = MTLLoadActionClear;
        passDesc.colorAttachments[1].storeAction = MTLStoreActionStore;
        passDesc.colorAttachments[1].clearColor = MTLClearColorMake(0.5, 0.5, 1.0, 0.0);
        passDesc.depthAttachment.texture = impl->depthTexture;
        passDesc.depthAttachment.loadAction = MTLLoadActionClear;
        passDesc.depthAttachment.storeAction = MTLStoreActionStore;
        passDesc.depthAttachment.clearDepth = 0.0;   // reverse-Z: far plane = 0
        impl->currentPassDesc = passDesc;

        // Composite pass renders to the drawable (BGRA8Unorm, no depth).
        MTLRenderPassDescriptor* compDesc = [MTLRenderPassDescriptor renderPassDescriptor];
        compDesc.colorAttachments[0].texture = impl->currentColorTarget;
        compDesc.colorAttachments[0].loadAction = MTLLoadActionDontCare;
        compDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
        impl->compositePassDesc = compDesc;
    }

#ifdef RT_ENABLE_IMGUI
    if (impl->imguiInitialized && impl->compositePassDesc) {
        ImGui_ImplMetal_NewFrame(impl->compositePassDesc);
        // ImGui_ImplGlfw_NewFrame() is run by Window in pollEvents.
        ImGui::NewFrame();
    }
#endif
}

XrBackend* MetalRenderer::xrBackend() {
#if TARGET_OS_VISION
    return impl->xrAdapter.get();
#else
    return nullptr;
#endif
}

void MetalRenderer::setXrBaseHint(const Vec3& worldPosition) {
    impl->xrBaseHint = simd_make_float3(static_cast<float>(worldPosition.x),
                                        static_cast<float>(worldPosition.y),
                                        static_cast<float>(worldPosition.z));
    impl->xrBaseHintValid = true;
}

void MetalRenderer::setCamera(const CameraState& camera) {
    impl->lastCameraState = camera;
    impl->hasLastCamera = true;
    float fovRad = static_cast<float>(degreesToRadians(camera.fovDegrees));

    // View and projection are both built engine-side (Mat4) and transposed into
    // Metal's column-major layout here; no matrix math lives in the backend.
    simd_float4x4 view = toSimd(Mat4::lookAt(camera.position, camera.target, camera.up));
    Mat4 projMat = (camera.projection == CameraProjection::Orthographic)
        ? Mat4::orthographic(camera.orthoHeight, camera.aspectRatio,
                             camera.nearPlane, camera.farPlane)
        : Mat4::perspective(fovRad, camera.aspectRatio,
                            camera.nearPlane, camera.farPlane);
    // Reverse-Z (ADR-0034 Phase 0): near->1, far->0 so float depth precision is
    // near-uniform across the wide far plane. The screen pass clears depth to 0
    // and tests Greater; every depth consumer (the post shader's sky test +
    // linear-depth reconstruction) is keyed to this. Inverses below derive from
    // this matrix, so SSR/SSAO/temporal-AO unprojection stay consistent.
    projMat = Mat4::reverseZ() * projMat;
    simd_float4x4 proj = toSimd(projMat);

    simd_float4x4 invView = toSimd(Mat4::lookAt(camera.position, camera.target,
                                                camera.up).inverse());
    simd_float4x4 invProj = toSimd(projMat.inverse());
    simd_float3 camPos = {static_cast<float>(camera.position.x),
                          static_cast<float>(camera.position.y),
                          static_cast<float>(camera.position.z)};

    // Head-tracked override (visionOS): when the surface tracks a headset,
    // render from the user's eye — pose from tracking, projection from the
    // compositor — instead of the game camera. The game camera still supplies
    // the locomotion base (where in the world the user stands) via baseHint.
    // Both projections are reverse-Z, so every depth consumer downstream is
    // unaffected by the swap.
    {
        simd_float4x4 worldFromEye, xrProj;
        simd_float3 baseHint = impl->xrBaseHintValid ? impl->xrBaseHint : camPos;
        if (impl->surface && impl->surface->xrView(baseHint, worldFromEye, xrProj)) {
            view = simd_inverse(worldFromEye);
            invView = worldFromEye;
            proj = xrProj;
            invProj = simd_inverse(xrProj);
            camPos = simd_make_float3(worldFromEye.columns[3].x,
                                      worldFromEye.columns[3].y,
                                      worldFromEye.columns[3].z);
            // Heartbeat stage 2/3 ("override"): proves the render camera is
            // LIVE. The forward vector must change as the user turns their
            // head — if it doesn't while "[xr] predict" moves, the sticky
            // capture in acquire() froze; if BOTH move but the display
            // doesn't, the fault is downstream (see the composite stage).
#if TARGET_OS_VISION
            static int xrCamBeat = 0;
            if ((xrCamBeat++ % 90) == 0) {
                simd_float4 fwd = worldFromEye.columns[2];
                NSLog(@"[xr] override #%llu pos(%.2f %.2f %.2f) fwd(%.2f %.2f %.2f)",
                      (unsigned long long)(impl->xrAdapter ? impl->xrAdapter->frameCounter : 0),
                      camPos.x, camPos.y, camPos.z, -fwd.x, -fwd.y, -fwd.z);
            }
#endif
        }
    }

    simd_float4x4 vp = simd_mul(proj, view);
    impl->cameraUniforms.viewProjection = vp;
    impl->cameraUniforms.view = view;
    impl->cameraUniforms.cameraPosition = camPos;
    impl->cameraUniforms._camPad0 = 0;

    // Inverse matrices for screen-space effects (SSR, SSAO)
    impl->cameraUniforms.invViewProjection = simd_mul(invView, invProj);
    // Roll the view-projection history for temporal AO: this frame's VP (matching
    // the invViewProjection above) becomes "current"; last frame's becomes "prev".
    impl->aoPrevViewProjection = impl->aoCurrViewProjection;
    impl->aoCurrViewProjection = vp;
    impl->cameraUniforms.projection = proj;
    impl->cameraUniforms.invProjection = invProj;
    impl->cameraUniforms.screenSize = {static_cast<float>(impl->framebufferWidth),
                                       static_cast<float>(impl->framebufferHeight)};
    impl->cameraUniforms.nearPlane = static_cast<float>(camera.nearPlane);
    impl->cameraUniforms.farPlane = static_cast<float>(camera.farPlane);
    impl->currentCameraPos = Vec3(camPos.x, camPos.y, camPos.z);
    impl->currentLens = camera.lens;   // drives the lens-warp + DOF passes

    // Wind for FLAG_WIND foliage (self-timed off the wall clock — purely
    // cosmetic vertex sway, no engine plumbing needed). Tunable defaults.
    static const double windStart = CACurrentMediaTime();
    impl->cameraUniforms.windDir = {0.85f, 0.0f, 0.30f};
    impl->cameraUniforms.windTime = static_cast<float>(CACurrentMediaTime() - windStart);
    impl->cameraUniforms.windAmplitude = 0.12f;
    impl->cameraUniforms.windFrequency = 1.6f;
    impl->cameraUniforms.windHeight = 2.5f;
    impl->cameraUniforms._windPad = 0.0f;
}

static simd_float3 toSimd3(const Vec3& v) {
    return {static_cast<float>(v.x), static_cast<float>(v.y),
            static_cast<float>(v.z)};
}

void MetalRenderer::setLights(const SceneLighting& lighting) {
    auto& lu = impl->lightUniforms;
    int idx = 0;
    constexpr int MAX_LIGHTS = 32;

    // Throttled state line: the one place every platform's lighting funnels
    // through, so a headset/simulator console can answer "what light state is
    // the GPU actually being handed" without a debugger attached.
    static int lightBeat = 0;
    if ((lightBeat++ % 300) == 0) {
        NSLog(@"[lights] sunI=%.2f dir(%.2f %.2f %.2f) zen(%.2f %.2f %.2f) "
              @"hor(%.2f %.2f %.2f) ambient=%.2f exposure=%.2f debugView=%d",
              lighting.sun.intensity,
              lighting.sun.direction.x, lighting.sun.direction.y,
              lighting.sun.direction.z,
              lighting.sky.zenithColor.x, lighting.sky.zenithColor.y,
              lighting.sky.zenithColor.z,
              lighting.sky.horizonColor.x, lighting.sky.horizonColor.y,
              lighting.sky.horizonColor.z,
              lighting.ambientMultiplier, lighting.exposure, debugView);
    }

    // Per-level cascade-fit overrides (0 = unset, keep the settings-driven value):
    // a large CDLOD world needs a far longer shadow range than the 150 m default.
    if (lighting.shadow.distance > 0.0f) shadowParams.distance = lighting.shadow.distance;
    if (lighting.shadow.cascadeCount > 0)
        shadowParams.cascadeCount = lighting.shadow.cascadeCount;

    // Directional light (sun)
    impl->shadowEnabled = false;
    if (idx < MAX_LIGHTS) {
        auto& g = lu.lights[idx];
        g.position = {};
        g.intensity = lighting.sun.intensity;
        Vec3 sunDir = normalize(lighting.sun.direction);
        g.direction = toSimd3(sunDir);
        g.innerCosAngle = 0;
        g.color = toSimd3(lighting.sun.color);
        g.outerCosAngle = 0;

        if (lighting.sun.castsShadow && lighting.shadow.enabled) {
            // Cascaded shadow maps (ADR-0017 Phase 5): split the camera view
            // frustum into shadowParams.cascadeCount ranges out to
            // shadowParams.distance and fit a texel-snapped ortho light box to
            // each. Near cascades are small (crisp); far ones cover distance —
            // no coverage "slice" when high up, and far better near-field
            // resolution than one fixed camera-sized box.
            int cascadeCount = std::max(1, std::min(shadowParams.cascadeCount,
                                                    (int)RT_MAX_CASCADES));
            Vec3 up = (std::abs(sunDir.y) > 0.99) ? Vec3(0, 0, 1) : Vec3(0, 1, 0);

            Real camNear = static_cast<Real>(impl->cameraUniforms.nearPlane);
            Real camFar  = static_cast<Real>(impl->cameraUniforms.farPlane);
            Real shadowDist = std::min(static_cast<Real>(shadowParams.distance), camFar);
            shadowDist = std::max(shadowDist, camNear * 2.0);
            Real lambda = shadowParams.splitLambda;

            // World-space corners of the full view frustum. The camera uses
            // reverse-Z (ADR-0034 Phase 0), so in NDC the near plane is at z=1 and
            // the far plane at z=0 — the opposite of the forward [0,1] convention.
            auto worldCorner = [&](float nx, float ny, float nz) -> Vec3 {
                simd_float4 c = simd_mul(impl->cameraUniforms.invViewProjection,
                                         simd_make_float4(nx, ny, nz, 1.0f));
                return Vec3(c.x / c.w, c.y / c.w, c.z / c.w);
            };
            Vec3 nearC[4] = { worldCorner(-1,-1,1), worldCorner(1,-1,1),
                              worldCorner(1,1,1),   worldCorner(-1,1,1) };
            Vec3 farC[4]  = { worldCorner(-1,-1,0), worldCorner(1,-1,0),
                              worldCorner(1,1,0),   worldCorner(-1,1,0) };

            float splitArr[RT_MAX_CASCADES] = {0};
            Real prevFar = camNear;
            for (int c = 0; c < cascadeCount; c++) {
                Real f = Real(c + 1) / Real(cascadeCount);
                Real uni = camNear + (shadowDist - camNear) * f;
                Real lg  = camNear * std::pow(shadowDist / camNear, f);
                Real zFar = lambda * lg + (1.0 - lambda) * uni;
                Real zNear = prevFar;
                prevFar = zFar;

                // View depth is affine along each frustum edge, so interpolate the
                // full-frustum corners by the cascade's near/far depth fractions.
                Real fN = (zNear - camNear) / (camFar - camNear);
                Real fF = (zFar  - camNear) / (camFar - camNear);
                Vec3 corners[8];
                for (int k = 0; k < 4; k++) {
                    corners[k]     = nearC[k] + (farC[k] - nearC[k]) * fN;
                    corners[k + 4] = nearC[k] + (farC[k] - nearC[k]) * fF;
                }
                // Camera-centered cascade fit: center on the camera eye, not the
                // frustum-slice centroid, so the covered region depends only on
                // position — not look direction — and shadows no longer pop as you
                // turn. The radius (distance to the slice's far corners) is
                // orientation-independent for a rigid frustum, so the sphere still
                // encloses the whole view cone out to this split. Costs resolution
                // vs a tight frustum fit (half the sphere is behind the camera), the
                // accepted trade for stability.
                Vec3 center = impl->currentCameraPos;
                Real radius = 0.01;   // floor avoids a zero-size ortho / snap div-by-0
                for (auto& p : corners) radius = std::max(radius, (p - center).length());
                radius = std::ceil(radius * 16.0) / 16.0;   // quantize to limit jitter

                // Pull the light back past the sphere so tall casters above the
                // cascade (trees) still register in the depth map.
                Real pullback = radius + 50.0;
                Real texelWorld = (radius * 2.0) / impl->shadowMapSize;

                Mat4 lightView = Mat4::lookAt(center + sunDir * pullback, center, up);
                Vec3 centerLS = lightView.transformPoint(center);
                centerLS.x = std::round(centerLS.x / texelWorld) * texelWorld;
                centerLS.y = std::round(centerLS.y / texelWorld) * texelWorld;
                Vec3 snapped = lightView.inverse().transformPoint(centerLS);

                lightView = Mat4::lookAt(snapped + sunDir * pullback, snapped, up);
                Mat4 lightProj = Mat4::orthographic(radius * 2.0, 1.0, 0.1,
                                                    pullback + radius);
                Mat4 lightVP = lightProj * lightView;

                impl->cascadeVP[c] = toSimd(lightVP);
                impl->shadowUniforms.cascadeViewProjection[c] = toSimd(lightVP);
                impl->cascadeCenter[c] = snapped;
                impl->cascadeRadius[c] = radius;
                splitArr[c] = static_cast<float>(zFar);
            }

            impl->shadowUniforms.cascadeSplit =
                simd_make_float4(splitArr[0], splitArr[1], splitArr[2], splitArr[3]);
            impl->shadowUniforms.cascadeCount = cascadeCount;
            impl->activeCascadeCount = cascadeCount;

            // The cascade matrices live in shadowUniforms (sun-only); GPULight
            // just flags the sun as a shadow caster (shadowMapIndex >= 0).
            g.lightViewProjection = impl->shadowUniforms.cascadeViewProjection[0];
            impl->shadowEnabled = true;
            g.shadowMapIndex = 0;
            impl->shadowDepthBias = lighting.shadow.bias;
            impl->shadowUniforms.normalBias = lighting.shadow.normalBias;
            impl->shadowUniforms.pcfRadius = lighting.shadow.pcfRadius;
            impl->shadowUniforms.shadowMapSize = impl->shadowMapSize;
        } else {
            g.lightViewProjection = simd_matrix_from_rows(
                simd_make_float4(1,0,0,0), simd_make_float4(0,1,0,0),
                simd_make_float4(0,0,1,0), simd_make_float4(0,0,0,1));
            g.shadowMapIndex = -1;
        }

        g.type = LightType_Directional;
        g.range = 0;
        g._pad[0] = 0;
        idx++;
    }

    // Point lights
    for (size_t i = 0; i < lighting.pointLights.size() && idx < MAX_LIGHTS; i++, idx++) {
        auto& g = lu.lights[idx];
        const auto& pl = lighting.pointLights[i];
        g.position = toSimd3(pl.position);
        g.intensity = pl.intensity;
        g.direction = {};
        g.innerCosAngle = 0;
        g.color = toSimd3(pl.color);
        g.outerCosAngle = 0;
        g.lightViewProjection = simd_matrix_from_rows(
            simd_make_float4(1,0,0,0), simd_make_float4(0,1,0,0),
            simd_make_float4(0,0,1,0), simd_make_float4(0,0,0,1));
        g.type = LightType_Point;
        g.shadowMapIndex = -1;
        g.range = pl.range;
        g._pad[0] = 0;
    }

    // Spot lights
    for (size_t i = 0; i < lighting.spotLights.size() && idx < MAX_LIGHTS; i++, idx++) {
        auto& g = lu.lights[idx];
        const auto& sl = lighting.spotLights[i];
        g.position = toSimd3(sl.position);
        g.intensity = sl.intensity;
        g.direction = toSimd3(normalize(sl.direction));
        g.innerCosAngle = std::cos(sl.innerConeAngle);
        g.color = toSimd3(sl.color);
        g.outerCosAngle = std::cos(sl.outerConeAngle);
        g.lightViewProjection = simd_matrix_from_rows(
            simd_make_float4(1,0,0,0), simd_make_float4(0,1,0,0),
            simd_make_float4(0,0,1,0), simd_make_float4(0,0,0,1));
        g.type = LightType_Spot;
        g.shadowMapIndex = -1;
        g.range = sl.range;
        g._pad[0] = 0;
    }

    lu.lightCount = idx;
    lu.exposure = lighting.exposure;
    lu.ambientMultiplier = lighting.ambientMultiplier;
    lu._pad[0] = 0;

    // Artistic shadow response (ADR-0017 Phase 2)
    impl->shadowUniforms.shadowTint = toSimd3(lighting.shadowArtistic.tint);
    impl->shadowUniforms.shadowStrength = lighting.shadowArtistic.strength;
    impl->shadowUniforms.ambientStrength = lighting.shadowArtistic.ambientStrength;

    // Procedural sky (ADR-0016): the analytic skybox and IBL fallback read these.
    const ProceduralSky& sky = lighting.sky;
    lu.skySunDir = toSimd3(normalize(sky.sunDirection));
    lu.skySunIntensity = sky.sunDiscIntensity;
    lu.skySunColor = toSimd3(sky.sunColor);
    lu.skyZenith = toSimd3(sky.zenithColor);
    lu.skyHorizon = toSimd3(sky.horizonColor);
    lu.skyGround = toSimd3(sky.groundColor);
    lu._skp0 = lu._skp1 = lu._skp2 = lu._skp3 = 0;
    lu.skyCloudCoverage = sky.cloudCoverage;
    lu.skyCloudDensity = sky.cloudDensity;
    lu.skyCloudScale = sky.cloudScale;
    lu.skyCloudTime = sky.cloudTime;
    lu.ambientTint = toSimd3(lighting.ambientTint);
    lu._grad0 = 0;

    // Aerial-perspective fog (mirrors the offline tracer): the lit pass fades
    // distant geometry toward fogColor. density 0 / disabled = off.
    lu.fogColor = toSimd3(lighting.fog.color);
    lu.fogDensity = lighting.fog.enabled ? lighting.fog.density : 0.0f;
    lu.fogHeightFalloff = lighting.fog.enabled ? lighting.fog.heightFalloff : 0.0f;

    impl->skyCloudsEnabled = sky.cloudsEnabled;

    memcpy([impl->lightBuffer contents], &lu, sizeof(LightUniforms));
}

void MetalRenderer::setAtmosphere(const AtmosphereRenderParams& atmosphere) {
    impl->atmosphere = atmosphere;
}

void MetalRenderer::setReflectionProbes(const std::vector<ReflectionProbe>& probes) {
    if (probes.empty()) {
        impl->probeUniforms.probeCount = 0;
        impl->probeCount = 0;
        return;
    }

    // Store probes and mark for baking on the next endFrame() when draw calls exist
    impl->pendingProbes = probes;
    impl->probesPendingBake = true;
    impl->probesBaked = false;
}

// Internal: actually bake probes (called from endFrame when draw calls are available).
void MetalRenderer::Impl::bakeProbes(Impl* impl, const std::vector<ReflectionProbe>& probes) {
    int count = std::min(static_cast<int>(probes.size()), static_cast<int>(8));
    impl->probeCount = count;

    // Create the real cubemap array if we are still holding the 1×1
    // shader-binding dummy from init, or the probe count outgrew it. (Blitting
    // 256×256 faces into the dummy is an out-of-bounds copy — the Metal debug
    // layer asserts on it, and without validation the probes silently sample
    // the dummy.)
    if (!impl->probeCubemapArray
        || impl->probeCubemapArray.width != static_cast<NSUInteger>(Impl::PROBE_CUBEMAP_SIZE)
        || impl->probeCubemapArray.arrayLength < static_cast<NSUInteger>(count)) {
        int size = Impl::PROBE_CUBEMAP_SIZE;
        MTLTextureDescriptor* cubeDesc = [[MTLTextureDescriptor alloc] init];
        cubeDesc.textureType = MTLTextureTypeCubeArray;
        cubeDesc.pixelFormat = MTLPixelFormatRGBA16Float;
        cubeDesc.width = size;
        cubeDesc.height = size;
        cubeDesc.arrayLength = count;
        cubeDesc.mipmapLevelCount = Impl::PROBE_MIP_LEVELS;
        cubeDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
        cubeDesc.storageMode = MTLStorageModePrivate;
        impl->probeCubemapArray = [impl->device newTextureWithDescriptor:cubeDesc];
    }

    // Upload probe metadata
    auto* gpuProbes = static_cast<GPUReflectionProbe*>([impl->probeBuffer contents]);
    for (int i = 0; i < count; i++) {
        gpuProbes[i].position = toSimd3(probes[i].position);
        gpuProbes[i].influenceRadius = probes[i].influenceRadius;
        gpuProbes[i].boxMin = toSimd3(probes[i].boxMin);
        gpuProbes[i]._pad0 = 0;
        gpuProbes[i].boxMax = toSimd3(probes[i].boxMax);
        gpuProbes[i].probeIndex = i;
    }

    impl->probeUniforms.probeCount = count;
    impl->probeUniforms.maxMipLevel = Impl::PROBE_MIP_LEVELS - 1;
    impl->probeUniforms._pad[0] = impl->probeUniforms._pad[1] = 0;

    // Bake cubemaps: render 6 faces per probe using existing scene geometry
    if (!impl->probesBaked) {
        int size = Impl::PROBE_CUBEMAP_SIZE;

        // Temporary per-face render target for baking (we'll blit to the array)
        MTLTextureDescriptor* faceDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                         width:size
                                        height:size
                                     mipmapped:NO];
        faceDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        faceDesc.storageMode = MTLStorageModePrivate;

        MTLTextureDescriptor* depthFaceDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                         width:size
                                        height:size
                                     mipmapped:NO];
        depthFaceDesc.usage = MTLTextureUsageRenderTarget;
        depthFaceDesc.storageMode = MTLStorageModePrivate;
        id<MTLTexture> faceDepth = [impl->device newTextureWithDescriptor:depthFaceDesc];

        // Dummy normal attachment (pipelines now output MRT)
        MTLTextureDescriptor* faceNormalDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                         width:size
                                        height:size
                                     mipmapped:NO];
        faceNormalDesc.usage = MTLTextureUsageRenderTarget;
        faceNormalDesc.storageMode = MTLStorageModePrivate;
        id<MTLTexture> faceNormal = [impl->device newTextureWithDescriptor:faceNormalDesc];

        for (int pi = 0; pi < count; pi++) {
            id<MTLTexture> faceColor = [impl->device newTextureWithDescriptor:faceDesc];

            for (int face = 0; face < 6; face++) {
                Vec3 eye = probes[pi].position;
                // Face cameras from the unit-tested convention helper
                // (ADR-0017 Phase 3). The mirrored projection flips winding,
                // so this pass culls with counterclockwise front faces.
                CubeFaceBasis basis = cubeFaceBasis(face);
                Mat4 viewMat = Mat4::lookAt(eye, eye + basis.forward, basis.up);

                CameraUniforms faceCam = {};
                faceCam.viewProjection = toSimd(cubeFaceViewProjection(face, eye, 0.1, 200.0));
                faceCam.view = toSimd(viewMat);
                faceCam.cameraPosition = toSimd3(eye);

                // Render scene to face
                id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];

                MTLRenderPassDescriptor* rpDesc = [MTLRenderPassDescriptor renderPassDescriptor];
                rpDesc.colorAttachments[0].texture = faceColor;
                rpDesc.colorAttachments[0].loadAction = MTLLoadActionClear;
                rpDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
                rpDesc.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
                rpDesc.colorAttachments[1].texture = faceNormal;
                rpDesc.colorAttachments[1].loadAction = MTLLoadActionDontCare;
                rpDesc.colorAttachments[1].storeAction = MTLStoreActionDontCare;
                rpDesc.depthAttachment.texture = faceDepth;
                rpDesc.depthAttachment.loadAction = MTLLoadActionClear;
                rpDesc.depthAttachment.storeAction = MTLStoreActionDontCare;
                rpDesc.depthAttachment.clearDepth = 1.0;

                id<MTLRenderCommandEncoder> enc = [cmdBuf renderCommandEncoderWithDescriptor:rpDesc];
                // Counterclockwise: the X-mirrored face projection flips the
                // winding of the scene's (clockwise-authored) triangles.
                [enc setFrontFacingWinding:MTLWindingCounterClockwise];
                [enc setCullMode:MTLCullModeBack];
                [enc setDepthStencilState:impl->depthStateOpaqueForwardZ];

                // Bind shadow resources (lights won't have shadows in probes, but shader expects bindings)
                [enc setFragmentTexture:impl->shadowMap atIndex:0];
                [enc setFragmentSamplerState:impl->shadowSampler atIndex:0];
                // strength/ambientStrength 0: probes bake unshadowed, as before
                ShadowUniforms noShadow = {};   // cascadeCount 0 -> no shadow
                noShadow.pcfRadius = 1.0f;
                noShadow.shadowMapSize = impl->shadowMapSize;
                [enc setFragmentBytes:&noShadow length:sizeof(ShadowUniforms) atIndex:5];

                // Bind empty probe data (no recursion)
                ProbeUniforms noProbes = {0, 0, {0, 0}};
                [enc setFragmentBytes:&noProbes length:sizeof(ProbeUniforms) atIndex:6];
                [enc setFragmentBuffer:impl->probeBuffer offset:0 atIndex:7];
                // Bind dummy textures for probe slots (shader expects them)
                [enc setFragmentTexture:impl->brdfLUT atIndex:2];
                [enc setFragmentSamplerState:impl->mipClampSampler atIndex:1];
                // IBL bindings (procedural mode during the bake — geometry in
                // reflection probes isn't re-lit by the env cubes; just bind so
                // the shader's texture(8/9)/buffer(8) slots are satisfied).
                EnvUniforms bakeEnv = {0, 0, 0, {0}};
                [enc setFragmentBytes:&bakeEnv length:sizeof(bakeEnv) atIndex:8];
                [enc setFragmentTexture:impl->defaultCubemap atIndex:8];
                [enc setFragmentTexture:impl->defaultCubemap atIndex:9];

                // Draw skybox (HDR equirect or procedural — see ADR-0016). Baking
                // it into the cube faces is what makes IBL track the environment.
                if (impl->skyboxPipeline) {
                    [enc setRenderPipelineState:impl->skyboxPipeline];
                    [enc setDepthStencilState:impl->skyboxDepthState];
                    [enc setVertexBytes:&faceCam length:sizeof(CameraUniforms) atIndex:1];
                    [enc setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
                    // cloudsEnabled = 0: clouds are never baked into probes.
                    EnvUniforms envU = {impl->environmentCubemap ? 1 : 0, 0, 0, {0}};
                    [enc setFragmentBytes:&envU length:sizeof(envU) atIndex:5];
                    [enc setFragmentTexture:(impl->environmentCubemap ? impl->environmentCubemap
                                                                       : impl->defaultCubemap)
                                    atIndex:0];
                    [enc setFragmentSamplerState:impl->equirectSampler atIndex:0];
                    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];

                    // Restore the shadow bindings the skybox clobbered
                    // (texture/sampler 0 and fragment buffer 5) before the
                    // lit geometry draws — same shared-slot hazard as the
                    // main pass.
                    [enc setFragmentTexture:impl->shadowMap atIndex:0];
                    [enc setFragmentSamplerState:impl->shadowSampler atIndex:0];
                    [enc setFragmentBytes:&noShadow length:sizeof(ShadowUniforms) atIndex:5];
                }

                // Draw opaque scene geometry
                [enc setDepthStencilState:impl->depthStateOpaqueForwardZ];
                for (auto& dc : impl->opaqueDrawCalls) {
                    const GPUMesh* mesh = impl->meshes.get(dc.meshHandle);
                    if (!mesh) continue;

                    ModelUniforms modelU;
                    modelU.model = toSimd(dc.transform);
                    modelU.normalMatrix = inverseTranspose(modelU.model);

                    MaterialUniforms matU;
                    matU.albedo = {static_cast<float>(dc.material.albedo.x),
                                   static_cast<float>(dc.material.albedo.y),
                                   static_cast<float>(dc.material.albedo.z)};
                    matU.metallic = dc.material.metallic;
                    matU.roughness = dc.material.roughness;
                    matU.opacity = dc.material.opacity;
                    matU.flags = static_cast<float>(dc.material.flags);
                    matU.textureFlags = 0;
                    matU.emission = {static_cast<float>(dc.material.emission.x),
                                     static_cast<float>(dc.material.emission.y),
                                     static_cast<float>(dc.material.emission.z)};

                    [enc setRenderPipelineState:impl->opaquePipeline];
                    [enc setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
                    [enc setVertexBytes:&faceCam length:sizeof(CameraUniforms) atIndex:1];
                    [enc setVertexBytes:&modelU length:sizeof(ModelUniforms) atIndex:2];
                    [enc setFragmentBytes:&faceCam length:sizeof(CameraUniforms) atIndex:1];
                    [enc setFragmentBytes:&matU length:sizeof(MaterialUniforms) atIndex:3];
                    [enc setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:3];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:4];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:5];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:6];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:7];
                    [enc setFragmentSamplerState:impl->linearWrapSampler atIndex:2];
                    [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                    indexCount:mesh->indexCount
                                     indexType:MTLIndexTypeUInt32
                                   indexBuffer:mesh->indexBuffer
                             indexBufferOffset:0];
                }

                [enc endEncoding];

                // Blit face to cubemap array (mip 0)
                id<MTLBlitCommandEncoder> blit = [cmdBuf blitCommandEncoder];
                [blit copyFromTexture:faceColor
                          sourceSlice:0 sourceLevel:0
                         sourceOrigin:MTLOriginMake(0, 0, 0)
                           sourceSize:MTLSizeMake(size, size, 1)
                            toTexture:impl->probeCubemapArray
                     destinationSlice:pi * 6 + face destinationLevel:0
                    destinationOrigin:MTLOriginMake(0, 0, 0)];
                [blit endEncoding];

                [cmdBuf commit];
                [cmdBuf waitUntilCompleted];
            }

            // Generate mipmaps for roughness blur (box filter for now — GGX prefilter
            // upgrade is a TODO; this gives reasonable roughness blur)
            {
                id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];
                id<MTLBlitCommandEncoder> blit = [cmdBuf blitCommandEncoder];
                // Generate mipmaps for this probe's slices in the cubemap array
                // Metal's generateMipmaps works on the entire texture
                [blit generateMipmapsForTexture:impl->probeCubemapArray];
                [blit endEncoding];
                [cmdBuf commit];
                [cmdBuf waitUntilCompleted];
            }
        }

        impl->probesBaked = true;
    }
}

void MetalRenderer::drawMesh(MeshHandle handle, const Mat4& transform,
                              const RenderMaterial& material) {
    Vec3 meshCenter = transform.transformPoint(Vec3(0, 0, 0));
    Vec3 diff = meshCenter - impl->currentCameraPos;
    float dist = static_cast<float>(diff.lengthSquared());

    Impl::DrawCall dc;
    dc.meshHandle = handle;
    dc.transform = transform;
    dc.material = material;
    dc.distanceToCamera = dist;

    if (material.flags & RenderMaterial::FLAG_OVERLAY) {
        // Debug gizmos must be issued LAST: they pass depth but never write it,
        // so anything drawn after them in the opaque pass — and the terrain /
        // foliage passes, which run later — painted straight over them
        // (device: "the visualization doesn't show for the agents").
        impl->overlayDrawCalls.push_back(dc);
    } else if (material.opacity < 1.0f) {
        impl->transparentDrawCalls.push_back(dc);
    } else {
        impl->opaqueDrawCalls.push_back(dc);
    }
}

void MetalRenderer::drawTerrain(MeshHandle handle, const RenderMaterial& material,
                                float morphStart, float morphEnd) {
    // No terrain pipeline (shader function absent) -> fall back to a plain draw so
    // the node still renders, just without the morph.
    if (!impl->terrainPipeline) {
        drawMesh(handle, Mat4(), material);
        return;
    }
    impl->terrainDrawCalls.push_back({handle, material, morphStart, morphEnd});
}

// Point the camera uniforms at one XR view's matrices. Used by endFrame's
// per-view loop; everything not overwritten here (wind, lens, near/far,
// screenSize) keeps the values setCamera computed earlier this frame.
static void applyXrCameraMatrices(CameraUniforms& u, Vec3& camPosOut,
                                  simd_float4x4 worldFromEye,
                                  simd_float4x4 proj) {
    simd_float4x4 view = simd_inverse(worldFromEye);
    u.view = view;
    u.projection = proj;
    u.viewProjection = simd_mul(proj, view);
    u.invProjection = simd_inverse(proj);
    u.invViewProjection = simd_mul(worldFromEye, u.invProjection);
    u.cameraPosition =
        simd_make_float3(worldFromEye.columns[3].x, worldFromEye.columns[3].y,
                         worldFromEye.columns[3].z);
    camPosOut = Vec3(worldFromEye.columns[3].x, worldFromEye.columns[3].y,
                     worldFromEye.columns[3].z);
}

void MetalRenderer::endFrame() {
    if (!impl->currentColorTarget || !impl->currentPassDesc) return;

    // Bake reflection probes on first frame when draw calls exist
    if (impl->probesPendingBake && !impl->opaqueDrawCalls.empty()) {
        Impl::bakeProbes(impl.get(), impl->pendingProbes);
        impl->probesPendingBake = false;
        impl->pendingProbes.clear();
    }

    impl->currentCommandBuffer = [impl->commandQueue commandBuffer];

    // Wireframe line colour: the lit fragment returns this flat colour when .w > 0.5.
    // Mode 1 draws all geometry as lines, so enable it for the whole frame; mode 2
    // keeps it off for the fills and toggles it on only for the overlay pass below.
    impl->cameraUniforms.wireColor = simd_make_float4(
        static_cast<float>(wireframeColor.x), static_cast<float>(wireframeColor.y),
        static_cast<float>(wireframeColor.z), wireframe == 1 ? 1.0f : 0.0f);

    // --- Shadow pass: one depth-array slice per cascade ---
    if (impl->shadowEnabled && impl->shadowPipeline) {
        // Batch identical-mesh casters into instanced shadow draws. The color
        // pass already instances the forest; the shadow pass used to redraw each
        // instance individually per cascade (the draw-call explosion). Sort once
        // by mesh so runs are contiguous — the color pass re-sorts by distance
        // later, so this doesn't disturb it. Shadow casters use a dedicated
        // instance buffer (the color pass's is filled afterward; sharing would
        // alias, since both are read at GPU-execution time, after all CPU writes).
        std::stable_sort(impl->opaqueDrawCalls.begin(), impl->opaqueDrawCalls.end(),
                         [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                             return a.meshHandle < b.meshHandle;
                         });
        id<MTLBuffer> shadowInstBuffer = impl->shadowInstanceBuffers[impl->frameIndex];
        GPUInstanceData* shadowInstBuf =
            static_cast<GPUInstanceData*>([shadowInstBuffer contents]);
        uint32_t shadowInstOffset = 0;  // accumulates across all cascades

        for (int c = 0; c < impl->activeCascadeCount; c++) {
            MTLRenderPassDescriptor* shadowPassDesc = [MTLRenderPassDescriptor renderPassDescriptor];
            shadowPassDesc.depthAttachment.texture = impl->shadowMap;
            shadowPassDesc.depthAttachment.slice = c;
            shadowPassDesc.depthAttachment.loadAction = MTLLoadActionClear;
            shadowPassDesc.depthAttachment.storeAction = MTLStoreActionStore;
            shadowPassDesc.depthAttachment.clearDepth = 1.0;

            id<MTLRenderCommandEncoder> shadowEncoder = [impl->currentCommandBuffer
                renderCommandEncoderWithDescriptor:shadowPassDesc];
            [shadowEncoder setFrontFacingWinding:MTLWindingClockwise];
            [shadowEncoder setCullMode:MTLCullModeBack];
            [shadowEncoder setDepthStencilState:impl->shadowDepthState];
            [shadowEncoder setDepthBias:impl->shadowDepthBias slopeScale:1.5 clamp:0.01];

            CameraUniforms cascadeCam = {};
            cascadeCam.viewProjection = impl->cascadeVP[c];

            // Conservative caster cull: a sphere reject around the cascade,
            // inflated by its own radius so the near cascade skips the distant
            // forest yet still catches tall casters that shadow into it.
            Vec3 cc = impl->cascadeCenter[c];
            Real cullR = impl->cascadeRadius[c] * 2.0;

            // One instanced draw per mesh: compact the run's casters that pass
            // the cascade cull into a contiguous instance range, then draw them
            // in a single call. Single-mesh casters (props, hero objects) go
            // through the same path with instanceCount 1 — the instanced shadow
            // shader reads instances[0].model, so no separate non-instanced path
            // is needed. Falls back to nothing only if the instanced pipeline is
            // absent (creation logged a failure at init).
            if (impl->shadowInstancedPipeline) {
                for (size_t bi = 0; bi < impl->opaqueDrawCalls.size(); ) {
                    MeshHandle batchMesh = impl->opaqueDrawCalls[bi].meshHandle;
                    size_t batchStart = bi;
                    while (bi < impl->opaqueDrawCalls.size() &&
                           impl->opaqueDrawCalls[bi].meshHandle == batchMesh)
                        bi++;

                    const GPUMesh* mesh = impl->meshes.get(batchMesh);
                    if (!mesh) continue;
                    BoundingSphere b = getMeshBounds(batchMesh);

                    uint32_t runStart = shadowInstOffset;
                    uint32_t runCount = 0;
                    for (size_t j = batchStart; j < bi; j++) {
                        if (shadowInstOffset >= SHADOW_MAX_INSTANCES) break;
                        // Debug-gizmo overlays (FLAG_OVERLAY) never cast shadows.
                        if (int(impl->opaqueDrawCalls[j].material.flags) &
                            RenderMaterial::FLAG_OVERLAY) continue;
                        const Mat4& m = impl->opaqueDrawCalls[j].transform;
                        Vec3 wc = m.transformPoint(b.center);
                        Real maxScale = std::max(
                            {Vec3(m.m[0][0], m.m[1][0], m.m[2][0]).length(),
                             Vec3(m.m[0][1], m.m[1][1], m.m[2][1]).length(),
                             Vec3(m.m[0][2], m.m[1][2], m.m[2][2]).length()});
                        if ((wc - cc).length() > cullR + b.radius * maxScale) continue;
                        shadowInstBuf[shadowInstOffset].model = toSimd(m);
                        shadowInstOffset++;
                        runCount++;
                    }
                    if (runCount == 0) continue;

                    [shadowEncoder setRenderPipelineState:impl->shadowInstancedPipeline];
                    [shadowEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
                    [shadowEncoder setVertexBytes:&cascadeCam
                                           length:sizeof(CameraUniforms) atIndex:1];
                    [shadowEncoder setVertexBuffer:shadowInstBuffer
                                            offset:runStart * sizeof(GPUInstanceData)
                                           atIndex:2];
                    [shadowEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                              indexCount:mesh->indexCount
                                               indexType:MTLIndexTypeUInt32
                                             indexBuffer:mesh->indexBuffer
                                       indexBufferOffset:0
                                           instanceCount:runCount];
                }
            }

            // CDLOD terrain casters (ADR-0036): render the selected nodes into the
            // cascade so ridges shadow their own far slopes (and the map isn't empty
            // in terrain-only scenes). The morphing caster pipeline applies the SAME
            // vertex morph as the receiver (by the real camera distance, passed in
            // cascadeCam.cameraPosition), so caster and surface line up — otherwise
            // lit, sun-facing slopes self-shadow. Nodes are world-space. Same cull.
            if (impl->terrainShadowPipeline) {
                CameraUniforms terrainCascadeCam = cascadeCam;
                terrainCascadeCam.cameraPosition = toSimd3(impl->currentCameraPos);
                [shadowEncoder setRenderPipelineState:impl->terrainShadowPipeline];
                for (const auto& tdc : impl->terrainDrawCalls) {
                    const GPUMesh* mesh = impl->meshes.get(tdc.meshHandle);
                    if (!mesh) continue;
                    BoundingSphere b = getMeshBounds(tdc.meshHandle);
                    if ((b.center - cc).length() > cullR + b.radius) continue;

                    TerrainUniforms tu = {tdc.morphStart, tdc.morphEnd, {0.0f, 0.0f}};
                    [shadowEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
                    [shadowEncoder setVertexBytes:&terrainCascadeCam
                                           length:sizeof(CameraUniforms) atIndex:1];
                    [shadowEncoder setVertexBytes:&tu
                                           length:sizeof(TerrainUniforms) atIndex:2];
                    [shadowEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                              indexCount:mesh->indexCount
                                               indexType:MTLIndexTypeUInt32
                                             indexBuffer:mesh->indexBuffer
                                       indexBufferOffset:0];
                }
            }
            [shadowEncoder endEncoding];
        }
    }

    // --- Per-view render loop -------------------------------------------
    // XR surfaces render the entire view-dependent pipeline (scene color,
    // atmosphere, post, composite) once per eye, each pass reading the
    // camera uniforms pointed at that eye. Desktop and the mono path run
    // exactly one iteration with untouched camera state. View-independent
    // work (probe bake, shadow cascades) stays above this line. The body
    // keeps its original indentation — it predates the loop.
    const int xrViewsToRender = impl->surface->xrViewCount();
    const int renderPassCount = xrViewsToRender > 0 ? xrViewsToRender : 1;
    for (int viewPass = 0; viewPass < renderPassCount; viewPass++) {
    if (xrViewsToRender > 0) {
        simd_float4x4 worldFromEye, xrProj;
        simd_float3 hint = impl->xrBaseHintValid
            ? impl->xrBaseHint
            : simd_make_float3(static_cast<float>(impl->lastCameraState.position.x),
                               static_cast<float>(impl->lastCameraState.position.y),
                               static_cast<float>(impl->lastCameraState.position.z));
        if (impl->surface->xrViewCamera(viewPass, hint, worldFromEye, xrProj))
            applyXrCameraMatrices(impl->cameraUniforms, impl->currentCameraPos,
                                  worldFromEye, xrProj);
    }

    // --- Main color pass ---
    impl->currentEncoder = [impl->currentCommandBuffer
        renderCommandEncoderWithDescriptor:impl->currentPassDesc];

    [impl->currentEncoder setFrontFacingWinding:MTLWindingClockwise];
    [impl->currentEncoder setCullMode:MTLCullModeBack];

    // Bind shadow resources for the entire main pass. activeShadowU outlives
    // the skybox draw below, which clobbers these bindings and needs them
    // restored (texture/sampler 0 and fragment buffer 5 are shared slots).
    impl->shadowUniforms.debugShadow = debugView;  // lit shader branches: 5=shadow factor, 6=albedo
    ShadowUniforms activeShadowU = impl->shadowUniforms;
    if (!impl->shadowEnabled) {       // cascadeCount 0 -> computeShadow returns lit
        activeShadowU = {};
        activeShadowU.pcfRadius = 1.0f;
        activeShadowU.shadowMapSize = impl->shadowMapSize;
        activeShadowU.debugShadow = debugView;
    }
    [impl->currentEncoder setFragmentTexture:impl->shadowMap atIndex:0];
    [impl->currentEncoder setFragmentSamplerState:impl->shadowSampler atIndex:0];
    [impl->currentEncoder setFragmentBytes:&activeShadowU
                                    length:sizeof(ShadowUniforms) atIndex:5];

    // Bind reflection probe resources for the entire main pass
    ProbeUniforms activeProbeUniforms = impl->probeUniforms;
    if (!reflectionProbesEnabled) {
        activeProbeUniforms.probeCount = 0;
    }
    [impl->currentEncoder setFragmentBytes:&activeProbeUniforms
                                    length:sizeof(ProbeUniforms) atIndex:6];
    [impl->currentEncoder setFragmentBuffer:impl->probeBuffer offset:0 atIndex:7];
    if (impl->probeCubemapArray) {
        [impl->currentEncoder setFragmentTexture:impl->probeCubemapArray atIndex:1];
    }
    [impl->currentEncoder setFragmentTexture:impl->brdfLUT atIndex:2];
    [impl->currentEncoder setFragmentSamplerState:impl->mipClampSampler atIndex:1];

    // Image-based lighting (ADR-0017 Phase 3): the prefiltered radiance and
    // irradiance cubes drive ambient/specular when an HDR is bound; in
    // procedural mode the shader evaluates the analytic sky instead.
    {
        bool hasIBL = impl->envPrefilteredCube && impl->envIrradianceCube
                      && environmentMapEnabled;
        EnvUniforms litEnv = {hasIBL ? 1 : 0, 0, Impl::ENV_PREFILTER_MIPS - 1, {0}};
        [impl->currentEncoder setFragmentBytes:&litEnv length:sizeof(litEnv) atIndex:8];
        [impl->currentEncoder setFragmentTexture:(hasIBL ? impl->envPrefilteredCube
                                                         : impl->defaultCubemap)
                                         atIndex:8];
        [impl->currentEncoder setFragmentTexture:(hasIBL ? impl->envIrradianceCube
                                                         : impl->defaultCubemap)
                                         atIndex:9];
    }

    // Draw skybox first (behind everything, no depth write)
    if (impl->skyboxPipeline) {
        [impl->currentEncoder setRenderPipelineState:impl->skyboxPipeline];
        [impl->currentEncoder setDepthStencilState:impl->skyboxDepthState];
        [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                      length:sizeof(CameraUniforms) atIndex:1];
        [impl->currentEncoder setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
        id<MTLTexture> envCube = environmentMapEnabled ? impl->environmentCubemap : nil;
        EnvUniforms envU = {envCube ? 1 : 0,
                            impl->skyCloudsEnabled ? 1 : 0, 0, {0}};
        [impl->currentEncoder setFragmentBytes:&envU length:sizeof(envU) atIndex:5];
        [impl->currentEncoder setFragmentTexture:(envCube ? envCube
                                                          : impl->defaultCubemap)
                                         atIndex:0];
        [impl->currentEncoder setFragmentSamplerState:impl->equirectSampler atIndex:0];
        [impl->currentEncoder drawPrimitives:MTLPrimitiveTypeTriangle
                                 vertexStart:0 vertexCount:3];

        // The skybox just rebound texture(0)/sampler(0) to the environment
        // cubemap + a non-comparison sampler, AND fragment buffer(5) to its
        // EnvUniforms — the same slot the lit shaders read ShadowUniforms
        // from. Restore all three, unconditionally: a stale buffer(5) left
        // the lit pass reading env data as shadow params (shadowMapSize 0 →
        // infinite texel size, NaN PCF offsets), which broke shadows entirely.
        [impl->currentEncoder setFragmentTexture:impl->shadowMap atIndex:0];
        [impl->currentEncoder setFragmentSamplerState:impl->shadowSampler atIndex:0];
        [impl->currentEncoder setFragmentBytes:&activeShadowU
                                        length:sizeof(ShadowUniforms) atIndex:5];
    }

    RenderStats stats;

    auto computeTextureFlags = [&](const RenderMaterial& mat) -> uint32_t {
        uint32_t tf = 0;
        if (mat.albedoMap.valid())            tf |= 1u;
        if (mat.metallicRoughnessMap.valid()) tf |= 2u;
        if (mat.normalMap.valid())            tf |= 4u;
        if (mat.aoMap.valid())                tf |= 8u;
        if (mat.emissiveMap.valid())          tf |= 16u;
        return tf;
    };

    auto bindMaterialTextures = [&](const RenderMaterial& mat, uint32_t) {
        auto resolve = [&](TextureHandle h) -> id<MTLTexture> {
            if (!h.valid()) return impl->defaultWhiteTexture;
            auto* t = impl->textures.get(h);
            return t ? *t : impl->defaultWhiteTexture;
        };
        [impl->currentEncoder setFragmentTexture:resolve(mat.albedoMap) atIndex:3];
        [impl->currentEncoder setFragmentTexture:resolve(mat.metallicRoughnessMap) atIndex:4];
        [impl->currentEncoder setFragmentTexture:resolve(mat.normalMap) atIndex:5];
        [impl->currentEncoder setFragmentTexture:resolve(mat.aoMap) atIndex:6];
        [impl->currentEncoder setFragmentTexture:resolve(mat.emissiveMap) atIndex:7];
        [impl->currentEncoder setFragmentSamplerState:impl->linearWrapSampler atIndex:2];
    };

    auto fillInstanceData = [&](const Impl::DrawCall& dc) -> GPUInstanceData {
        GPUInstanceData inst;
        inst.model = toSimd(dc.transform);
        inst.normalMatrix = inverseTranspose(inst.model);
        inst.albedo = {static_cast<float>(dc.material.albedo.x),
                       static_cast<float>(dc.material.albedo.y),
                       static_cast<float>(dc.material.albedo.z), 0};
        inst.metallic = dc.material.metallic;
        inst.roughness = dc.material.roughness;
        inst.opacity = dc.material.opacity;
        inst.flags = static_cast<float>(dc.material.flags);
        inst.emission = {static_cast<float>(dc.material.emission.x),
                         static_cast<float>(dc.material.emission.y),
                         static_cast<float>(dc.material.emission.z), 0};
        inst.textureFlags = computeTextureFlags(dc.material);
        inst._instPad[0] = inst._instPad[1] = inst._instPad[2] = 0;
        return inst;
    };

    auto issueSingleDraw = [&](const Impl::DrawCall& dc) {
        const GPUMesh* mesh = impl->meshes.get(dc.meshHandle);
        if (!mesh) return;

        ModelUniforms modelUniforms;
        modelUniforms.model = toSimd(dc.transform);
        modelUniforms.normalMatrix = inverseTranspose(modelUniforms.model);

        uint32_t tf = computeTextureFlags(dc.material);
        MaterialUniforms matUniforms;
        matUniforms.albedo = {static_cast<float>(dc.material.albedo.x),
                              static_cast<float>(dc.material.albedo.y),
                              static_cast<float>(dc.material.albedo.z)};
        matUniforms.metallic = dc.material.metallic;
        matUniforms.roughness = dc.material.roughness;
        matUniforms.opacity = dc.material.opacity;
        matUniforms.flags = static_cast<float>(dc.material.flags);
        matUniforms.textureFlags = tf;
        matUniforms.emission = {static_cast<float>(dc.material.emission.x),
                                static_cast<float>(dc.material.emission.y),
                                static_cast<float>(dc.material.emission.z)};

        bindMaterialTextures(dc.material, tf);

        [impl->currentEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
        [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                      length:sizeof(CameraUniforms) atIndex:1];
        [impl->currentEncoder setVertexBytes:&modelUniforms
                                      length:sizeof(ModelUniforms) atIndex:2];
        [impl->currentEncoder setFragmentBytes:&impl->cameraUniforms
                                        length:sizeof(CameraUniforms) atIndex:1];
        [impl->currentEncoder setFragmentBytes:&matUniforms
                                        length:sizeof(MaterialUniforms) atIndex:3];
        [impl->currentEncoder setFragmentBuffer:impl->lightBuffer
                                        offset:0 atIndex:4];

        [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                         indexCount:mesh->indexCount
                                          indexType:MTLIndexTypeUInt32
                                        indexBuffer:mesh->indexBuffer
                                  indexBufferOffset:0];
        stats.drawCalls++;
        stats.totalInstances++;
        stats.trianglesDrawn += mesh->indexCount / 3;
    };

    // ONE running offset for the whole frame: every pass encodes into the
    // same per-frame instance buffer, and the GPU reads it only when the
    // command buffer executes — a pass that restarted at 0 CLOBBERED the
    // instances of the passes before it (device: the debug wedges rendered
    // as big flickering blocks — those were the overlay pass overwriting
    // the opaque pass's instance transforms).
    uint32_t instanceOffset = 0;
    auto issuePass = [&](std::vector<Impl::DrawCall>& drawCalls,
                         id<MTLRenderPipelineState> singlePipeline,
                         id<MTLRenderPipelineState> instancedPipeline,
                         id<MTLDepthStencilState> depthState,
                         bool skipFoliage) {
        if (drawCalls.empty()) return;

        [impl->currentEncoder setDepthStencilState:depthState];

        // Stable sort by mesh handle to group identical meshes while preserving
        // depth order within each group.
        std::stable_sort(drawCalls.begin(), drawCalls.end(),
                         [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                             return a.meshHandle < b.meshHandle;
                         });

        id<MTLBuffer> instanceBuffer = impl->instanceBuffers[impl->frameIndex];
        GPUInstanceData* instanceBuf =
            static_cast<GPUInstanceData*>([instanceBuffer contents]);

        size_t i = 0;
        while (i < drawCalls.size()) {
            size_t batchStart = i;
            MeshHandle batchMesh = drawCalls[i].meshHandle;
            while (i < drawCalls.size() && drawCalls[i].meshHandle == batchMesh)
                i++;
            uint32_t batchSize = static_cast<uint32_t>(i - batchStart);

            const GPUMesh* mesh = impl->meshes.get(batchMesh);
            if (!mesh) continue;

            // Debug-gizmo overlay batches ignore depth so they draw on top of the
            // world; everything else uses the pass's depth state.
            [impl->currentEncoder setDepthStencilState:
                (int(drawCalls[batchStart].material.flags) & RenderMaterial::FLAG_OVERLAY)
                    ? impl->depthStateOverlay : depthState];

            // FLAG_TWO_SIDED batches draw both faces. Cull mode is encoder state,
            // not per-instance, so unlike the shading flags this cannot ride the
            // instance buffer — it is set per BATCH. Safe because batches are
            // grouped by mesh and a mesh carries one material, and cheap: a
            // handful of state changes per frame rather than per triangle.
            [impl->currentEncoder setCullMode:
                (int(drawCalls[batchStart].material.flags) & RenderMaterial::FLAG_TWO_SIDED)
                    ? MTLCullModeNone : MTLCullModeBack];

            // Alpha-cut foliage is drawn by the depth-prepass path (issueFoliage)
            // when enabled — skip it here so it isn't also drawn single-pass.
            if (skipFoliage &&
                (int(drawCalls[batchStart].material.flags) & RenderMaterial::FLAG_ALPHA_TEST))
                continue;

            if (batchSize == 1) {
                [impl->currentEncoder setRenderPipelineState:singlePipeline];
                issueSingleDraw(drawCalls[batchStart]);
                continue;
            }

            if (instanceOffset + batchSize > MAX_INSTANCES) {
                [impl->currentEncoder setRenderPipelineState:singlePipeline];
                for (size_t j = batchStart; j < batchStart + batchSize; j++)
                    issueSingleDraw(drawCalls[j]);
                continue;
            }

            for (size_t j = batchStart; j < batchStart + batchSize; j++)
                instanceBuf[instanceOffset + (j - batchStart)] =
                    fillInstanceData(drawCalls[j]);

            [impl->currentEncoder setRenderPipelineState:instancedPipeline];
            [impl->currentEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
            [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                          length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setVertexBuffer:instanceBuffer
                                           offset:instanceOffset * sizeof(GPUInstanceData)
                                          atIndex:2];
            [impl->currentEncoder setFragmentBytes:&impl->cameraUniforms
                                            length:sizeof(CameraUniforms) atIndex:1];
            // LightUniforms exceeds the 4KB setBytes limit (32 lights + sky) —
            // bind the persistent light buffer, as the single-draw path does.
            [impl->currentEncoder setFragmentBuffer:impl->lightBuffer
                                             offset:0 atIndex:4];

            bindMaterialTextures(drawCalls[batchStart].material,
                                computeTextureFlags(drawCalls[batchStart].material));

            [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                             indexCount:mesh->indexCount
                                              indexType:MTLIndexTypeUInt32
                                            indexBuffer:mesh->indexBuffer
                                      indexBufferOffset:0
                                          instanceCount:batchSize];
            stats.drawCalls++;
            stats.instancedDrawCalls++;
            stats.totalInstances += batchSize;
            stats.trianglesDrawn += (mesh->indexCount / 3) * batchSize;
            instanceOffset += batchSize;
        }
    };

    // Foliage depth prepass + lit (perf). Called after solids + terrain have
    // populated depth/color, so leaves occlude correctly and the shaded
    // background shows through their alpha-cut holes. opaqueDrawCalls is already
    // mesh-sorted (issuePass sorted it), so foliage batches are contiguous. All
    // foliage goes through the instanced path (a lone tree is instanceCount 1).
    auto issueFoliage = [&]() {
        if (!impl->foliageDepthPipeline || !impl->foliageLitPipeline) return;

        id<MTLBuffer> fBuf = impl->foliageInstanceBuffers[impl->frameIndex];
        GPUInstanceData* fData = static_cast<GPUInstanceData*>([fBuf contents]);
        uint32_t fOff = 0;

        struct FoliageBatch { const GPUMesh* mesh; RenderMaterial material;
                              uint32_t offset; uint32_t count; };
        std::vector<FoliageBatch> batches;
        auto& dcs = impl->opaqueDrawCalls;

        // Phase 1 — depth-only prepass: write the nearest leaf depth per pixel
        // (Greater + write), alpha-cut so silhouette holes don't write depth.
        [impl->currentEncoder setRenderPipelineState:impl->foliageDepthPipeline];
        [impl->currentEncoder setDepthStencilState:impl->depthStateOpaque];
        size_t i = 0;
        while (i < dcs.size()) {
            MeshHandle bm = dcs[i].meshHandle;
            size_t start = i;
            while (i < dcs.size() && dcs[i].meshHandle == bm) i++;
            if (!(int(dcs[start].material.flags) & RenderMaterial::FLAG_ALPHA_TEST)) continue;

            const GPUMesh* mesh = impl->meshes.get(bm);
            if (!mesh) continue;
            if (fOff >= FOLIAGE_MAX_INSTANCES) break;

            uint32_t offset = fOff;
            uint32_t count = 0;
            for (size_t j = start; j < i && fOff < FOLIAGE_MAX_INSTANCES; j++) {
                fData[fOff++] = fillInstanceData(dcs[j]);
                count++;
            }
            batches.push_back({mesh, dcs[start].material, offset, count});

            [impl->currentEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
            [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                          length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setVertexBuffer:fBuf
                                           offset:offset * sizeof(GPUInstanceData) atIndex:2];
            bindMaterialTextures(dcs[start].material, 0);  // albedo (tex 3) + sampler 2 for the cut
            [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                             indexCount:mesh->indexCount
                                              indexType:MTLIndexTypeUInt32
                                            indexBuffer:mesh->indexBuffer
                                      indexBufferOffset:0
                                          instanceCount:count];
        }

        if (batches.empty()) return;

        // Phase 2 — lit: Equal/no-write + early depth tests, so only the front-most
        // leaf per pixel is shaded. Shadow/probe/IBL bindings from the top of the
        // main pass are still live; bind camera + lights + material per batch.
        [impl->currentEncoder setRenderPipelineState:impl->foliageLitPipeline];
        [impl->currentEncoder setDepthStencilState:impl->depthStateFoliageLit];
        for (const auto& b : batches) {
            [impl->currentEncoder setVertexBuffer:b.mesh->vertexBuffer offset:0 atIndex:0];
            [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                          length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setVertexBuffer:fBuf
                                           offset:b.offset * sizeof(GPUInstanceData) atIndex:2];
            [impl->currentEncoder setFragmentBytes:&impl->cameraUniforms
                                            length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
            bindMaterialTextures(b.material, computeTextureFlags(b.material));
            [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                             indexCount:b.mesh->indexCount
                                              indexType:MTLIndexTypeUInt32
                                            indexBuffer:b.mesh->indexBuffer
                                      indexBufferOffset:0
                                          instanceCount:b.count];
            stats.drawCalls++;
            stats.instancedDrawCalls++;
            stats.totalInstances += b.count;
            stats.trianglesDrawn += (b.mesh->indexCount / 3) * b.count;
        }
    };

    stats.entitiesSubmitted = static_cast<uint32_t>(
        impl->opaqueDrawCalls.size() + impl->transparentDrawCalls.size());

    // Wireframe-only (mode 1): draw the geometry passes as lines. The skybox
    // already drew filled above; overlay (mode 2) re-draws lines after the fills.
    [impl->currentEncoder setTriangleFillMode:
        (wireframe == 1 ? MTLTriangleFillModeLines : MTLTriangleFillModeFill)];

    // Sort each pass by distance first, then issuePass stable-sorts by mesh.
    std::sort(impl->opaqueDrawCalls.begin(), impl->opaqueDrawCalls.end(),
              [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                  return a.distanceToCamera < b.distanceToCamera;
              });

    issuePass(impl->opaqueDrawCalls, impl->opaquePipeline,
              impl->opaqueInstancedPipeline, impl->depthStateOpaque,
              /*skipFoliage=*/depthPrepassEnabled);

    // CDLOD terrain nodes (ADR-0036). Opaque, so drawn here with the morph pipeline,
    // reusing the shadow/probe/IBL/light bindings already set on this encoder. Each
    // node is a world-space mesh (no model matrix); the vertex shader morphs it by
    // camera distance using the per-node band in TerrainUniforms.
    if (impl->terrainPipeline && !impl->terrainDrawCalls.empty()) {
        [impl->currentEncoder setRenderPipelineState:impl->terrainPipeline];
        [impl->currentEncoder setDepthStencilState:impl->depthStateOpaque];
        for (const auto& tdc : impl->terrainDrawCalls) {
            const GPUMesh* mesh = impl->meshes.get(tdc.meshHandle);
            if (!mesh) continue;

            uint32_t tf = computeTextureFlags(tdc.material);
            MaterialUniforms matU;
            matU.albedo = {static_cast<float>(tdc.material.albedo.x),
                           static_cast<float>(tdc.material.albedo.y),
                           static_cast<float>(tdc.material.albedo.z)};
            matU.metallic = tdc.material.metallic;
            matU.roughness = tdc.material.roughness;
            matU.opacity = tdc.material.opacity;
            matU.flags = static_cast<float>(tdc.material.flags);
            matU.textureFlags = tf;
            matU.emission = {static_cast<float>(tdc.material.emission.x),
                             static_cast<float>(tdc.material.emission.y),
                             static_cast<float>(tdc.material.emission.z)};
            bindMaterialTextures(tdc.material, tf);

            TerrainUniforms tu = {tdc.morphStart, tdc.morphEnd, {0.0f, 0.0f}};
            [impl->currentEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
            [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                          length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setVertexBytes:&tu
                                          length:sizeof(TerrainUniforms) atIndex:2];
            [impl->currentEncoder setFragmentBytes:&impl->cameraUniforms
                                            length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setFragmentBytes:&matU
                                            length:sizeof(MaterialUniforms) atIndex:3];
            [impl->currentEncoder setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
            [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                             indexCount:mesh->indexCount
                                              indexType:MTLIndexTypeUInt32
                                            indexBuffer:mesh->indexBuffer
                                      indexBufferOffset:0];
            stats.drawCalls++;
            stats.trianglesDrawn += mesh->indexCount / 3;
        }
    }

    // Alpha-cut foliage: depth prepass + early-Z lit pass (kills leaf overdraw).
    // Drawn here, after solids + terrain, so leaves occlude against them and the
    // shaded background shows through alpha holes. Off -> drawn in issuePass above.
    if (depthPrepassEnabled) issueFoliage();

    std::sort(impl->transparentDrawCalls.begin(), impl->transparentDrawCalls.end(),
              [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                  return a.distanceToCamera > b.distanceToCamera;
              });

    issuePass(impl->transparentDrawCalls, impl->transparentPipeline,
              impl->transparentInstancedPipeline, impl->depthStateTransparent,
              /*skipFoliage=*/false);

    // Debug-gizmo overlays LAST (device: agent widgets weren't drawn over the
    // world): they pass depth without writing it, so they only stay visible if
    // nothing renders after them. Sorted far-to-near like transparents so
    // nested gizmos layer sanely.
    std::sort(impl->overlayDrawCalls.begin(), impl->overlayDrawCalls.end(),
              [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                  return a.distanceToCamera > b.distanceToCamera;
              });
    issuePass(impl->overlayDrawCalls, impl->opaquePipeline,
              impl->opaqueInstancedPipeline, impl->depthStateOverlay,
              /*skipFoliage=*/false);

    // Wireframe overlay (mode 2): re-draw opaque geometry as lines on top of the
    // shaded image. Non-instanced (issueSingleDraw uses model bytes, not the
    // shared instance buffer, so it can't clobber the fills' instance data), and
    // a LessEqual/no-write depth state so lines sit on their own visible surface.
    if (wireframe == 2) {
        impl->cameraUniforms.wireColor.w = 1.0f;     // overlay lines draw the flat colour
        [impl->currentEncoder setRenderPipelineState:impl->opaquePipeline];
        [impl->currentEncoder setDepthStencilState:impl->depthStateWireOverlay];
        [impl->currentEncoder setTriangleFillMode:MTLTriangleFillModeLines];
        for (auto& dc : impl->opaqueDrawCalls)
            issueSingleDraw(dc);
        [impl->currentEncoder setTriangleFillMode:MTLTriangleFillModeFill];
        impl->cameraUniforms.wireColor.w = 0.0f;
    }

    impl->lastStats = stats;

    [impl->currentEncoder endEncoding];

    // --- Planetary atmosphere glow (procedural-planet-plan P3) ---
    // A fullscreen triangle that raymarches the atmosphere shell and ADDITIVELY
    // blends the in-scattered light into the HDR scene, after all geometry and
    // before post — so the limb halo blooms. Camera + sun come from this frame's
    // uniforms; centre/radii/coeffs from setAtmosphere().
    if (impl->atmosphere.enabled && impl->atmospherePipeline && impl->sceneColorTexture) {
        const AtmosphereRenderParams& ap = impl->atmosphere;
        const GPULight& sun = impl->lightUniforms.lights[0];
        AtmosphereUniforms au;
        au.invViewProjection = impl->cameraUniforms.invViewProjection;
        simd_float3 camPos = impl->cameraUniforms.cameraPosition;
        au.cameraPosition = simd_make_float4(camPos.x, camPos.y, camPos.z, 0.0f);
        au.sunDirection = simd_make_float4(sun.direction.x, sun.direction.y, sun.direction.z, 0.0f);
        au.planetCenter = simd_make_float4((float)ap.planetCenter.x, (float)ap.planetCenter.y,
                                           (float)ap.planetCenter.z, 0.0f);
        au.sunColor = simd_make_float4(sun.color.x, sun.color.y, sun.color.z, ap.sunIntensity);
        au.rayleighCoeff = simd_make_float4((float)ap.rayleighCoeff.x, (float)ap.rayleighCoeff.y,
                                            (float)ap.rayleighCoeff.z, 0.0f);
        au.radii = simd_make_float4(ap.planetRadius, ap.atmosphereRadius,
                                    ap.rayleighScaleHeight, ap.mieScaleHeight);
        au.mie = simd_make_float4(ap.mieCoeff, ap.mieG,
                                  (float)ap.viewSamples, (float)ap.lightSamples);

        MTLRenderPassDescriptor* atmPass = [MTLRenderPassDescriptor renderPassDescriptor];
        atmPass.colorAttachments[0].texture = impl->sceneColorTexture;
        atmPass.colorAttachments[0].loadAction = MTLLoadActionLoad;    // keep the scene
        atmPass.colorAttachments[0].storeAction = MTLStoreActionStore;
        id<MTLRenderCommandEncoder> atmEnc =
            [impl->currentCommandBuffer renderCommandEncoderWithDescriptor:atmPass];
        [atmEnc setRenderPipelineState:impl->atmospherePipeline];
        [atmEnc setFragmentBytes:&au length:sizeof(au) atIndex:0];
        [atmEnc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [atmEnc endEncoding];
    }

    // Lens effects (virtual-camera plan Phase 4). Both passes are skipped
    // entirely — zero GPU cost — when toggled off or visually inert (no
    // aberrations / pinhole aperture). Debug views show raw buffers, so they
    // bypass both as well.
    const LensParams& lens = impl->currentLens;
    bool lensWarpActive = lensEffectsEnabled && lens.hasAberrations()
                       && impl->lensWarpPipeline && impl->postLDRTexture
                       && debugView == 0;
    bool dofActive = dofEnabled && lens.apertureDiameter() > 0.0
                  && lens.sensorHeight > 0.0
                  && impl->dofPipeline && impl->dofTexture
                  && debugView == 0;

    // --- Post-processing compute passes (single encoder) ---
    // Batching SSAO, SSR, and bloom into one compute encoder eliminates
    // per-encoder CPU overhead (~15 encoder create/destroy → 1).
    bool needsCompute = (impl->aoPipeline && impl->aoTexture && ssaoEnabled)
                     || (impl->ssrPipeline && impl->ssrTexture
                         && (ssrEnabled || debugView == 2))
                     || (bloomEnabled && impl->bloomDownsamplePipeline
                         && impl->bloomUpsamplePipeline && impl->bloomMips[0])
                     || dofActive;

    // When temporal AO runs, the resolved result lands in aoBlurTemp (so the
    // current/history reads don't alias the output); the composite samples that
    // instead of aoTexture, and we blit it into aoHistory for next frame.
    bool aoResolvedInBlurTemp = false;

    if (needsCompute) {
        id<MTLComputeCommandEncoder> enc = [impl->currentCommandBuffer computeCommandEncoder];
        MTLSize group = MTLSizeMake(8, 8, 1);

        // --- SSAO (half resolution; see aoTexture allocation) ---
        if (impl->aoPipeline && impl->aoTexture && ssaoEnabled) {
            MTLSize aoGrid = MTLSizeMake(impl->aoTexture.width, impl->aoTexture.height, 1);

            [enc setComputePipelineState:impl->aoPipeline];
            [enc setTexture:impl->depthTexture atIndex:0];
            [enc setTexture:impl->aoTexture atIndex:1];
            [enc setTexture:impl->viewNormalTexture atIndex:2];   // real normals (no depth recon)
            [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
            // Per-frame rotation by the golden angle: successive frames sample
            // well-distributed directions the temporal resolve averages, so a low
            // direction count reads banding-free once it converges (~10 frames).
            float aoFrameRotation =
                static_cast<float>(impl->frameCount & 1023u) * 2.39996323f;
            SSAOUniforms aoP = {
                ssaoParams.radius, ssaoParams.intensity, ssaoParams.bias,
                ssaoParams.directions, ssaoParams.steps, aoFrameRotation, {}
            };
            [enc setBytes:&aoP length:sizeof(aoP) atIndex:1];
            [enc dispatchThreadgroups:threadgroupsCovering(aoGrid, group)
                threadsPerThreadgroup:group];

            if (impl->aoBlurHPipeline && impl->aoBlurVPipeline && impl->aoBlurTemp) {
                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->aoBlurHPipeline];
                [enc setTexture:impl->aoTexture atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->aoBlurTemp atIndex:2];
                [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
                [enc dispatchThreadgroups:threadgroupsCovering(aoGrid, group)
                    threadsPerThreadgroup:group];

                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->aoBlurVPipeline];
                [enc setTexture:impl->aoBlurTemp atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->aoTexture atIndex:2];
                [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
                [enc dispatchThreadgroups:threadgroupsCovering(aoGrid, group)
                    threadsPerThreadgroup:group];
            }

            // --- Temporal resolve: blend in reprojected history AO ---
            // Stabilizes foliage flicker (G-buffer aliasing). Reads the current
            // (blurred) AO from aoTexture + history from aoHistory, writes the
            // resolved AO to aoBlurTemp.
            if (impl->aoTemporalPipeline && impl->aoHistory && impl->aoBlurTemp) {
                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->aoTemporalPipeline];
                [enc setTexture:impl->aoTexture atIndex:0];      // current (blurred)
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->aoHistory atIndex:2];      // last frame's resolved
                [enc setTexture:impl->aoBlurTemp atIndex:3];     // resolved out
                [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
                AOTemporalUniforms tP = {};
                tP.prevViewProjection = impl->aoPrevViewProjection;
                // Temporal history is single-view; with per-eye rendering the
                // history would belong to the other eye. Disable blending (not
                // the pass) until history goes per-view.
                tP.alpha = (impl->aoHistoryValid && xrViewsToRender <= 1)
                    ? ssaoParams.temporal : 0.0f;
                [enc setBytes:&tP length:sizeof(tP) atIndex:1];
                [enc dispatchThreadgroups:threadgroupsCovering(aoGrid, group)
                    threadsPerThreadgroup:group];
                aoResolvedInBlurTemp = true;
                impl->aoHistoryValid = true;
            }
        }

        // --- SSR ---
        // Run SSR when enabled, or whenever the SSR debug view is active so its
        // per-pixel diagnostic (see ssrRayMarch) renders even with SSR toggled off.
        bool ssrDebug = (debugView == 2);
        if (impl->ssrPipeline && impl->ssrTexture && (ssrEnabled || ssrDebug)) {
            int halfW = std::max(impl->framebufferWidth / 2, 1);
            int halfH = std::max(impl->framebufferHeight / 2, 1);
            MTLSize ssrGrid = MTLSizeMake(halfW, halfH, 1);

            [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
            [enc setComputePipelineState:impl->ssrPipeline];
            [enc setTexture:impl->sceneColorTexture atIndex:0];
            [enc setTexture:impl->depthTexture atIndex:1];
            [enc setTexture:impl->ssrTexture atIndex:2];
            [enc setTexture:impl->viewNormalTexture atIndex:3];
            [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
            SSRUniforms ssrP = {
                ssrParams.maxRayDist, ssrParams.thickness, ssrParams.thicknessFar,
                ssrParams.stride, ssrParams.blendStrength, ssrParams.maxRoughness,
                ssrDebug ? 1.0f : 0.0f, 0.0f
            };
            [enc setBytes:&ssrP length:sizeof(ssrP) atIndex:1];
            [enc dispatchThreadgroups:threadgroupsCovering(ssrGrid, group)
                threadsPerThreadgroup:group];

            // Skip the bilateral blur in debug mode — it would smear the flat
            // color codes across surface edges and muddy the diagnostic.
            if (!ssrDebug && impl->ssrBlurHPipeline && impl->ssrBlurVPipeline && impl->ssrBlurTemp) {
                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->ssrBlurHPipeline];
                [enc setTexture:impl->ssrTexture atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->ssrBlurTemp atIndex:2];
                [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
                [enc dispatchThreadgroups:threadgroupsCovering(ssrGrid, group)
                    threadsPerThreadgroup:group];

                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->ssrBlurVPipeline];
                [enc setTexture:impl->ssrBlurTemp atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->ssrTexture atIndex:2];
                [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
                [enc dispatchThreadgroups:threadgroupsCovering(ssrGrid, group)
                    threadsPerThreadgroup:group];
            }
        }

        // --- Bloom ---
        if (bloomEnabled && impl->bloomDownsamplePipeline && impl->bloomUpsamplePipeline
            && impl->bloomMips[0]) {
            [enc memoryBarrierWithScope:MTLBarrierScopeTextures];

            // Downsample chain: scene → mip0 → mip1 → ... → mip[N-1]
            for (int m = 0; m < MetalRenderer::Impl::BLOOM_MIP_COUNT; m++) {
                if (m > 0) [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                id<MTLTexture> src = (m == 0) ? impl->sceneColorTexture : impl->bloomMips[m - 1];
                id<MTLTexture> dst = impl->bloomMips[m];

                BloomUniforms bp = {
                    bloomParams.threshold, bloomParams.knee, bloomParams.intensity,
                    static_cast<int32_t>(src.width), static_cast<int32_t>(src.height), {}
                };

                [enc setComputePipelineState:impl->bloomDownsamplePipeline];
                [enc setTexture:src atIndex:0];
                [enc setTexture:dst atIndex:1];
                [enc setBytes:&bp length:sizeof(bp) atIndex:0];
                MTLSize grid = MTLSizeMake(dst.width, dst.height, 1);
                [enc dispatchThreadgroups:threadgroupsCovering(grid, group)
                    threadsPerThreadgroup:group];
            }

            // Upsample chain: read directly from downsample mips (no blit copy needed)
            int last = MetalRenderer::Impl::BLOOM_MIP_COUNT - 1;
            for (int m = last; m >= 0; m--) {
                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                id<MTLTexture> src = (m == last)
                    ? impl->bloomMips[last]           // smallest downsample mip as seed
                    : impl->bloomUpsampleMips[m + 1]; // previously upsampled result
                id<MTLTexture> higher = impl->bloomMips[m];
                id<MTLTexture> dst = impl->bloomUpsampleMips[m];

                BloomUniforms bp = {
                    bloomParams.threshold, bloomParams.knee, bloomParams.intensity,
                    static_cast<int32_t>(src.width), static_cast<int32_t>(src.height), {}
                };

                [enc setComputePipelineState:impl->bloomUpsamplePipeline];
                [enc setTexture:src atIndex:0];
                [enc setTexture:higher atIndex:1];
                [enc setTexture:dst atIndex:2];
                [enc setBytes:&bp length:sizeof(bp) atIndex:0];
                MTLSize grid = MTLSizeMake(dst.width, dst.height, 1);
                [enc dispatchThreadgroups:threadgroupsCovering(grid, group)
                    threadsPerThreadgroup:group];
            }
        }

        // --- Depth of field (virtual-camera plan Phase 4) ---
        // CoC-driven gather from the HDR scene; the composite pass then reads
        // dofTexture in place of sceneColorTexture. Runs after SSR/bloom so
        // both keep sampling the sharp scene.
        if (dofActive) {
            [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
            [enc setComputePipelineState:impl->dofPipeline];
            [enc setTexture:impl->sceneColorTexture atIndex:0];
            [enc setTexture:impl->depthTexture atIndex:1];
            [enc setTexture:impl->dofTexture atIndex:2];
            [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
            DOFUniforms dofP = {
                static_cast<float>(lens.focusDistance),
                static_cast<float>(lens.focalLength / 1000.0),    // mm → meters
                static_cast<float>(lens.apertureDiameter()),
                // Sensor-plane meters → pixels (sensorHeight is mm)
                static_cast<float>(impl->framebufferHeight * 1000.0 / lens.sensorHeight),
                16.0f,   // max CoC radius in pixels — matches the 24-tap budget
                {}
            };
            [enc setBytes:&dofP length:sizeof(dofP) atIndex:1];
            MTLSize dofGrid = MTLSizeMake(impl->framebufferWidth,
                                          impl->framebufferHeight, 1);
            [enc dispatchThreadgroups:threadgroupsCovering(dofGrid, group)
                threadsPerThreadgroup:group];
        }

        [enc endEncoding];

        // Save this frame's resolved AO as next frame's temporal history.
        if (aoResolvedInBlurTemp && impl->aoHistory && xrViewsToRender <= 1) {
            id<MTLBlitCommandEncoder> aoBlit = [impl->currentCommandBuffer blitCommandEncoder];
            [aoBlit copyFromTexture:impl->aoBlurTemp toTexture:impl->aoHistory];
            [aoBlit endEncoding];
        }
    }

    // --- Composite pass: tone map HDR scene to LDR drawable ---
    if (impl->compositePipeline && impl->compositePassDesc) {
        // When the lens warp runs, composite renders into an intermediate of
        // the same BGRA8Unorm format and the warp pass owns the drawable.
        if (lensWarpActive) {
            impl->compositePassDesc.colorAttachments[0].texture = impl->postLDRTexture;
        }
        // XR: this view's content goes where the drawable's texture map says
        // (texture index + array slice + viewport) — never "view v = slice v".
        MTLRenderPassDescriptor* compositeDesc = impl->compositePassDesc;
        bool haveXrTarget = false;
        MTLViewport xrViewport = {};
        NSUInteger xrSlice = 0;
        id<MTLTexture> xrDepthDst = nil;
        if (xrViewsToRender > 0) {
            id<MTLTexture> xrColor = nil, xrDepth = nil;
            if (impl->surface->xrViewTarget(viewPass, xrColor, xrDepth,
                                            xrSlice, xrViewport) && xrColor) {
                compositeDesc = [MTLRenderPassDescriptor renderPassDescriptor];
                compositeDesc.colorAttachments[0].texture = xrColor;
                compositeDesc.colorAttachments[0].slice = xrSlice;
                compositeDesc.colorAttachments[0].loadAction = MTLLoadActionDontCare;
                compositeDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
                haveXrTarget = true;
                xrDepthDst = xrDepth;
#if TARGET_OS_VISION
                static int xrCompBeat = 0;
                if ((xrCompBeat++ % 180) == 0) {
                    NSLog(@"[xr] composite view=%d texSlice=%lu viewport(%.0f,%.0f %'.0fx%.0f)",
                          viewPass, (unsigned long)xrSlice,
                          xrViewport.originX, xrViewport.originY,
                          xrViewport.width, xrViewport.height);
                }
#endif
            }
        }
        id<MTLRenderCommandEncoder> compEncoder = [impl->currentCommandBuffer
            renderCommandEncoderWithDescriptor:compositeDesc];
        if (haveXrTarget) [compEncoder setViewport:xrViewport];
        [compEncoder setRenderPipelineState:impl->compositePipeline];
        [compEncoder setFragmentTexture:(dofActive ? impl->dofTexture
                                                   : impl->sceneColorTexture)
                                atIndex:0];
        [compEncoder setFragmentTexture:impl->ssrTexture atIndex:1];
        [compEncoder setFragmentTexture:(aoResolvedInBlurTemp ? impl->aoBlurTemp
                                                              : impl->aoTexture)
                                atIndex:2];
        [compEncoder setFragmentTexture:impl->depthTexture atIndex:3];
        [compEncoder setFragmentTexture:impl->viewNormalTexture atIndex:4];
        if (bloomEnabled && impl->bloomUpsampleMips[0]) {
            [compEncoder setFragmentTexture:impl->bloomUpsampleMips[0] atIndex:5];
        }
        [compEncoder setFragmentBytes:&impl->cameraUniforms
                               length:sizeof(CameraUniforms) atIndex:0];
        CompositeUniforms compositeParams;
        // Who owns the display transfer function this frame. Derived from the
        // ACTUAL target format rather than assumed, because it differs by
        // platform: macOS renders into a linear-storage BGRA8Unorm drawable, so
        // the shader encodes; visionOS is handed a *_sRGB drawable it cannot
        // opt out of, so the hardware does and the shader must not.
        compositeParams.targetEncodesSRGB = impl->surface->targetEncodesSRGB() ? 1 : 0;
        compositeParams.ssaoEnabled = ssaoEnabled ? 1 : 0;
        compositeParams.ssrEnabled = ssrEnabled ? 1 : 0;
        compositeParams.debugView = debugView;
        compositeParams.ssrBlendStrength = ssrParams.blendStrength;
        compositeParams.bloomEnabled = bloomEnabled ? 1 : 0;
        compositeParams.bloomIntensity = bloomParams.intensity;
        id<MTLTexture> compEnvCube = environmentMapEnabled ? impl->environmentCubemap : nil;
        compositeParams.envMode = compEnvCube ? 1 : 0;
        compositeParams.aoFloor = ssaoParams.aoFloor;
        compositeParams.tonemapOp = tonemapOperator;
        compositeParams.gradeContrast = gradeParams.contrast;
        compositeParams.gradeSaturation = gradeParams.saturation;
        [compEncoder setFragmentBytes:&compositeParams
                               length:sizeof(compositeParams) atIndex:1];
        // Sky for composite sky pixels: day/night procedural (+clouds) or, when an
        // HDR map is bound, the baked environment cubemap — matching the skybox/IBL.
        // (Cube orientation is unit-tested; the old equirect workaround is gone.)
        [compEncoder setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
        [compEncoder setFragmentTexture:(compEnvCube ? compEnvCube
                                                     : impl->defaultCubemap)
                                atIndex:6];
        [compEncoder setFragmentSamplerState:impl->linearClampSampler atIndex:0];
        [compEncoder setFragmentSamplerState:impl->equirectSampler atIndex:1];
        [compEncoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];

        // --- Lens-warp pass (virtual-camera plan Phase 4) ---
        // The LAST image-space pass: distortion + chromatic aberration +
        // vignette resampling of the composited image into the drawable. The
        // debug UI draws after it, into whichever encoder owns the drawable,
        // so ImGui/HUD stay undistorted.
        id<MTLRenderCommandEncoder> uiEncoder = compEncoder;
        if (lensWarpActive) {
            [compEncoder endEncoding];

            MTLRenderPassDescriptor* lensPassDesc = [MTLRenderPassDescriptor renderPassDescriptor];
            lensPassDesc.colorAttachments[0].texture = impl->currentColorTarget;
            lensPassDesc.colorAttachments[0].loadAction = MTLLoadActionDontCare;
            lensPassDesc.colorAttachments[0].storeAction = MTLStoreActionStore;

            id<MTLRenderCommandEncoder> lensEncoder = [impl->currentCommandBuffer
                renderCommandEncoderWithDescriptor:lensPassDesc];
            [lensEncoder setRenderPipelineState:impl->lensWarpPipeline];
            [lensEncoder setFragmentTexture:impl->postLDRTexture atIndex:0];
            LensPostUniforms lensP = {
                static_cast<float>(lens.distortionK1),
                static_cast<float>(lens.distortionK2),
                static_cast<float>(lens.chromaticAberration),
                static_cast<float>(lens.vignette),
                impl->framebufferHeight > 0
                    ? static_cast<float>(impl->framebufferWidth) / impl->framebufferHeight
                    : 1.0f,
                {}
            };
            [lensEncoder setFragmentBytes:&lensP length:sizeof(lensP) atIndex:0];
            [lensEncoder setFragmentSamplerState:impl->linearClampSampler atIndex:0];
            [lensEncoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
            uiEncoder = lensEncoder;
        }

        // Debug UI (ADR-0011) renders on top of the tone-mapped image. One
        // view only under XR — the HUD is not stereo-correct content.
#ifdef RT_ENABLE_IMGUI
        if (impl->imguiInitialized && viewPass == 0) {
            ImGui::Render();
            ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(),
                                           impl->currentCommandBuffer,
                                           uiEncoder);
        }
#endif

        [uiEncoder endEncoding];

        // XR: the compositor reprojects using the drawable's DEPTH, per view.
        // The scene depth buffer holds THIS view's depth (rendered with this
        // view's compositor projection, reverse-Z — bit-compatible), so copy
        // it to the view's mapped depth slice while it is still this view's.
        if (haveXrTarget && xrDepthDst && impl->depthTexture
            && xrDepthDst.pixelFormat == impl->depthTexture.pixelFormat
            && xrDepthDst.width == impl->depthTexture.width
            && xrDepthDst.height == impl->depthTexture.height) {
            id<MTLBlitCommandEncoder> dblit =
                [impl->currentCommandBuffer blitCommandEncoder];
            [dblit copyFromTexture:impl->depthTexture sourceSlice:0 sourceLevel:0
                      sourceOrigin:MTLOriginMake(0, 0, 0)
                        sourceSize:MTLSizeMake(impl->depthTexture.width,
                                               impl->depthTexture.height, 1)
                         toTexture:xrDepthDst destinationSlice:xrSlice
                  destinationLevel:0
                 destinationOrigin:MTLOriginMake(0, 0, 0)];
            [dblit endEncoding];
        }
    }
    }  // --- end per-view render loop ---

    // Headless frame capture (RT_FRAME_DUMP) — copy the composited drawable
    // before present, then write it out once the GPU finishes.
    id<MTLTexture> dumpStaging = nil;
    bool dumpThisFrame = impl->frameDumpPath && ++impl->frameDumpCounter == 90;
    if (dumpThisFrame) {
        id<MTLTexture> drawableTex = impl->currentColorTarget;
        MTLTextureDescriptor* d = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:drawableTex.pixelFormat
                                         width:drawableTex.width
                                        height:drawableTex.height
                                     mipmapped:NO];
        d.storageMode = MTLStorageModeShared;
        dumpStaging = [impl->device newTextureWithDescriptor:d];
        id<MTLBlitCommandEncoder> blit = [impl->currentCommandBuffer blitCommandEncoder];
        [blit copyFromTexture:drawableTex
                  sourceSlice:0 sourceLevel:0
                 sourceOrigin:MTLOriginMake(0, 0, 0)
                   sourceSize:MTLSizeMake(drawableTex.width, drawableTex.height, 1)
                    toTexture:dumpStaging
             destinationSlice:0 destinationLevel:0
            destinationOrigin:MTLOriginMake(0, 0, 0)];
        [blit endEncoding];
    }

    impl->surface->present(impl->currentCommandBuffer, impl->depthTexture);
    [impl->currentCommandBuffer commit];
    // Before any waitUntilCompleted below: a compositor-driven surface wants to
    // close its frame as soon as the work is submitted, not after we have
    // finished reading it back for a frame dump.
    impl->surface->frameSubmitted();

    if (dumpThisFrame) {
        [impl->currentCommandBuffer waitUntilCompleted];
        const int w = static_cast<int>(dumpStaging.width);
        const int h = static_cast<int>(dumpStaging.height);
        std::vector<uint8_t> pixels(static_cast<size_t>(w) * h * 4);
        [dumpStaging getBytes:pixels.data()
                  bytesPerRow:static_cast<NSUInteger>(w) * 4
                   fromRegion:MTLRegionMake2D(0, 0, w, h)
                  mipmapLevel:0];
        for (size_t i = 0; i < pixels.size(); i += 4)
            std::swap(pixels[i], pixels[i + 2]);   // BGRA → RGBA
        stbi_write_png(impl->frameDumpPath, w, h, 4, pixels.data(), w * 4);
        NSLog(@"[FRAME DUMP] wrote %s (%dx%d)", impl->frameDumpPath, w, h);
        impl->frameDumpPath = nullptr;
    }
}

void MetalRenderer::initDebugUi(void* /*windowHandle*/) {
#ifdef RT_ENABLE_IMGUI
    // Create the ImGui context and the Metal backend. Runs before
    // Window::initDebugUi, which attaches the GLFW backend to this context.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;   // don't write imgui.ini
    ImGui::StyleColorsDark();
    ImGui_ImplMetal_Init(impl->device);
    impl->imguiInitialized = true;
#endif
}

void MetalRenderer::shutdownDebugUi() {
#ifdef RT_ENABLE_IMGUI
    if (!impl->imguiInitialized) return;
    ImGui_ImplMetal_Shutdown();
    ImGui::DestroyContext();   // Window::shutdownDebugUi (GLFW) ran first
    impl->imguiInitialized = false;
#endif
}

std::unique_ptr<Renderer> Renderer::create() {
    return std::make_unique<MetalRenderer>();
}

}  // namespace engine

#endif // __APPLE__
