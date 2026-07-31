// visionOS plumbing spike — the Swift half.
//
// This is the ONLY Swift in the project, and it exists because visionOS gives
// no choice: an immersive app's entry point must be a SwiftUI `App` declaring an
// `ImmersiveSpace`. There is no C++ `main()` to hook. Everything past the
// CompositorLayer closure is C++/ObjC++ (see vision_spike.mm).
//
// Keep this file thin. Anything that is not "stand up the immersive space and
// hand the layer renderer to the engine" belongs on the C++ side, where the rest
// of the engine already lives.

import SwiftUI
import CompositorServices

/// Chooses the drawable formats and stereo layout before the layer comes up.
/// Preferring `.layered` (both eyes as slices of one texture array) is what
/// later allows a single draw per object with vertex amplification instead of
/// two passes; the renderer in vision_spike.mm is written to handle whichever
/// layout it is actually given.
struct SpikeLayerConfiguration: CompositorLayerConfiguration {
    func makeConfiguration(capabilities: LayerRenderer.Capabilities,
                           configuration: inout LayerRenderer.Configuration) {
        configuration.depthFormat = .depth32Float
        // MUST be an sRGB format. CompositorServices rejects .bgra8Unorm with
        // "Layer configuration not supported" (code -4) and the immersive space
        // never comes up — the compositor requires correctly tagged colour so it
        // can blend the layer with passthrough and system content.
        //
        // KNOWN CONSEQUENCE: the frame is currently gamma-encoded TWICE. The
        // composite pass emits display-encoded values (post_composite.metal
        // applies pow(c, 1/2.2), and the ACES/AgX tone mappers fold the encode
        // in), and then the hardware encodes again on write to this target. That
        // is why visionOS looks washed out relative to macOS, whose CAMetalLayer
        // uses linear-storage MTLPixelFormatBGRA8Unorm.
        //
        // The fix is NOT to change this format — the platform will not allow it.
        // It is to separate tone mapping from display encoding in the composite
        // pass so the transfer function is applied exactly once, by whichever of
        // shader or hardware owns it on that target.
        configuration.colorFormat = .bgra8Unorm_srgb

        let foveationEnabled = capabilities.supportsFoveation
        configuration.isFoveationEnabled = foveationEnabled

        let options: LayerRenderer.Capabilities.SupportedLayoutsOptions =
            foveationEnabled ? [.foveationEnabled] : []
        let supported = capabilities.supportedLayouts(options: options)
        configuration.layout = supported.contains(.layered) ? .layered : .dedicated
    }
}

@main
struct VisionSpikeApp: App {
    var body: some Scene {
        ImmersiveSpace {
            CompositorLayer(configuration: SpikeLayerConfiguration()) { layerRenderer in
                // rt_vision_spike_run blocks for the lifetime of the space, so it
                // must not run on the main actor — the compositor would starve.
                Thread.detachNewThread {
                    Thread.current.name = "rt.vision.render"
                    rt_vision_spike_run(layerRenderer)
                }
            }
        }
        .immersionStyle(selection: .constant(.full), in: .full)
    }
}
