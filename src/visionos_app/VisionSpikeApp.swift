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

        // Match the layout/foveation choices of Apple's Metal template — the
        // configuration proven to display on this headset. (.dedicated with
        // foveation off also runs the full frame protocol but was never
        // composited to the displays during the window-first probe rounds.)
        // Foveation OFF while the renderer does not attach the drawable's
        // rasterization rate map to its passes: writing a foveated (warped)
        // texture without the map is the last untested difference from the
        // template's working path. Foveation returns with Task 3, attached
        // properly.
        configuration.isFoveationEnabled = false
        let supportedLayouts = capabilities.supportedLayouts(options: [])
        configuration.layout = supportedLayouts.contains(.layered) ? .layered : .dedicated
        print("[vision] foveation off, layered \(supportedLayouts.contains(.layered))")
    }
}

/// Launcher window. The app used to be immersive-space-only, auto-opened via
/// CPSceneSessionRoleImmersiveSpaceApplication — on visionOS 26.3 that path
/// opens the space and runs the layer (frames accepted, anchors attached) but
/// never composites it to the displays: black void, hands breaking through.
/// Apple's own Metal template — window first, user-triggered
/// openImmersiveSpace — displays fine on the same headset, so the engine now
/// enters the arena the way the platform actually exercises.
struct LaunchView: View {
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace
    @State private var inArena = false

    var body: some View {
        VStack(spacing: 16) {
            Text("Raytracer")
                .font(.title)
            Button(inArena ? "Leave Arena" : "Enter Arena") {
                Task { @MainActor in
                    if inArena {
                        await dismissImmersiveSpace()
                        inArena = false
                    } else if await openImmersiveSpace(id: "arena") == .opened {
                        inArena = true
                    }
                }
            }
        }
        .padding(32)
    }
}

@main
struct VisionSpikeApp: App {
    var body: some Scene {
        WindowGroup {
            LaunchView()
        }
        .defaultSize(width: 360, height: 240)

        ImmersiveSpace(id: "arena") {
            CompositorLayer(configuration: SpikeLayerConfiguration()) { layerRenderer in
                // TEMPORARY DIAGNOSTIC: route to the ported Apple-template
                // renderer (rotating cube on dark green) instead of the
                // engine. Green cube visible -> the app shell and project are
                // fine and the delta is the engine's frame structure. Black ->
                // the app/project configuration is at fault; no render code is.
                let useTemplateRenderer = false
                if useTemplateRenderer {
                    TemplateRenderer.start(layerRenderer)
                } else {
                    // rt_vision_spike_run blocks for the lifetime of the space,
                    // so it must not run on the main actor — the compositor
                    // would starve.
                    Thread.detachNewThread {
                        Thread.current.name = "rt.vision.render"
                        rt_vision_spike_run(layerRenderer)
                    }
                }
            }
        }
        .immersionStyle(selection: .constant(.full), in: .full)
    }
}
