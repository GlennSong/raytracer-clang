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
        // Ask the DEVICE what it can display instead of assuming. The 2.4
        // simulator accepted .bgra8Unorm_srgb and displayed it; on hardware the
        // layer came up and ran the whole frame protocol with that format, but
        // nothing ever reached the display. Apple's immersive-Metal template
        // uses .rgba16Float (linear EDR) — prefer that when offered, and log
        // the supported list so the truth is in the console.
        let formats = capabilities.supportedColorFormats
        print("[vision] supported color formats: \(formats.map { $0.rawValue })")
        if formats.contains(.rgba16Float) {
            configuration.colorFormat = .rgba16Float
        } else if let first = formats.first {
            configuration.colorFormat = first
        } else {
            configuration.colorFormat = .bgra8Unorm_srgb
        }
        print("[vision] chose color format: \(configuration.colorFormat.rawValue)")

        // Foveation OFF and .dedicated layout, deliberately. The renderer
        // composites a plain full-resolution image per eye; it does not yet
        // apply the compositor's rasterization rate map (foveation) or render
        // to array slices (.layered). Enabling either without implementing it
        // puts pixels where the compositor doesn't read them — on device that
        // showed as a black view while the engine ran at full rate. Both come
        // back with per-eye stereo (Task 3), which is where amplification and
        // the rate map get implemented for real.
        configuration.isFoveationEnabled = false
        configuration.layout = .dedicated
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
