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

/// Shell state shared between the launcher window and the immersive space's
/// spatial-event handler. The handler runs where SwiftUI environment actions
/// don't reach, so the window deposits its open/dismiss actions here — the
/// MetalCheck template's AppModel pattern.
@MainActor
final class XrShellModel: ObservableObject {
    static let shared = XrShellModel()
    @Published var inArena = false
    var dismissSpace: (() async -> Void)?
}

/// Launcher window AND in-game menu: pick a scene, enter it, and return here
/// with a long pinch (~1s) from inside the arena.
struct LaunchView: View {
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace
    @ObservedObject private var shell = XrShellModel.shared
    @State private var selectedLevel = "arena"

    /// Level shortnames from the bundled assets/levels/*.json (the sidecar
    /// .cameras.json files are not levels).
    static let levels: [String] = {
        guard let root = Bundle.main.resourceURL?
            .appendingPathComponent("assets/levels") else { return ["arena"] }
        let files = (try? FileManager.default
            .contentsOfDirectory(at: root, includingPropertiesForKeys: nil)) ?? []
        let names = files
            .filter { $0.pathExtension == "json" }
            .map { $0.deletingPathExtension().lastPathComponent }
            .filter { !$0.hasSuffix(".cameras") && !$0.hasSuffix(".json") }
        return names.isEmpty ? ["arena"] : names.sorted()
    }()

    var body: some View {
        VStack(spacing: 16) {
            Text("Raytracer")
                .font(.title)
            Picker("Scene", selection: $selectedLevel) {
                ForEach(Self.levels, id: \.self) { Text($0).tag($0) }
            }
            .pickerStyle(.wheel)
            .frame(height: 120)
            .disabled(shell.inArena)
            Button(shell.inArena ? "Leave \(selectedLevel)" : "Enter \(selectedLevel)") {
                Task { @MainActor in
                    if shell.inArena {
                        await dismissImmersiveSpace()
                        shell.inArena = false
                    } else {
                        rt_vision_set_level(selectedLevel)
                        if await openImmersiveSpace(id: "arena") == .opened {
                            shell.inArena = true
                        }
                    }
                }
            }
            Text("In the scene: quick pinch teleports to where you look;\nhold a pinch ~1s to come back to this menu.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding(32)
        .onAppear {
            shell.dismissSpace = { [dismissImmersiveSpace] in
                await dismissImmersiveSpace()
                XrShellModel.shared.inArena = false
            }
        }
    }
}

/// Forwards CompositorLayer spatial events to the engine (gaze ray + pinch
/// phases) and owns the long-pinch shell gesture: hold ~1s to leave the scene
/// for the menu. Runs on the main actor.
@MainActor
final class SpatialEventRelay {
    static let shared = SpatialEventRelay()
    private var activePinchStart: [SpatialEventCollection.Event.ID: Date] = [:]
    private var menuTriggered = false
    private let menuHoldSeconds: TimeInterval = 1.0

    func handle(_ events: SpatialEventCollection) {
        for event in events {
            let ray = event.selectionRay
            let origin = ray?.origin ?? .zero
            let direction = ray?.direction ?? .init(x: 0, y: 0, z: -1)
            switch event.phase {
            case .active:
                if activePinchStart[event.id] == nil {
                    activePinchStart[event.id] = Date()
                    menuTriggered = false
                    rt_vision_xr_pinch(0, origin.x, origin.y, origin.z,
                                       direction.x, direction.y, direction.z)
                } else {
                    rt_vision_xr_pinch(1, origin.x, origin.y, origin.z,
                                       direction.x, direction.y, direction.z)
                    if !menuTriggered,
                       let start = activePinchStart[event.id],
                       Date().timeIntervalSince(start) >= menuHoldSeconds {
                        menuTriggered = true
                        Task { await XrShellModel.shared.dismissSpace?() }
                    }
                }
            case .ended:
                activePinchStart[event.id] = nil
                rt_vision_xr_pinch(2, origin.x, origin.y, origin.z,
                                   direction.x, direction.y, direction.z)
            case .cancelled:
                activePinchStart[event.id] = nil
                rt_vision_xr_pinch(3, origin.x, origin.y, origin.z,
                                   direction.x, direction.y, direction.z)
            @unknown default:
                break
            }
        }
    }
}

@main
struct VisionSpikeApp: App {
    var body: some Scene {
        WindowGroup {
            LaunchView()
        }
        .defaultSize(width: 400, height: 340)

        ImmersiveSpace(id: "arena") {
            CompositorLayer(configuration: SpikeLayerConfiguration()) { layerRenderer in
                layerRenderer.onSpatialEvent = { events in
                    Task { @MainActor in
                        SpatialEventRelay.shared.handle(events)
                    }
                }
                // rt_vision_spike_run blocks for the lifetime of the space,
                // so it must not run on the main actor — the compositor
                // would starve.
                Thread.detachNewThread {
                    Thread.current.name = "rt.vision.render"
                    rt_vision_spike_run(layerRenderer)
                }
            }
        }
        .immersionStyle(selection: .constant(.full), in: .full)
    }
}
