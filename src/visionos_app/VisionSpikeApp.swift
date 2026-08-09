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
        // template's working path. Foveation returns once every pass that
        // targets a drawable texture attaches that rate map.
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
    /// Immersion style for the NEXT space open: .full for VR scenes, .mixed
    /// for the AR sandbox (passthrough). Must agree with the engine's
    /// rt_vision_set_passthrough flag — the style controls the compositor's
    /// blending, the flag controls the alpha the renderer presents.
    /// VERIFY: ImmersionStyle stored-property + $binding pattern is from
    /// Apple's immersive Metal template (AppModel.immersionStyle).
    @Published var immersion: ImmersionStyle = .full
    var dismissSpace: (() async -> Void)?
}

/// Launcher window AND in-game menu: pick a scene, enter it, and return here
/// with a long pinch (~1s) from inside the arena.
struct LaunchView: View {
    @Environment(\.openImmersiveSpace) private var openImmersiveSpace
    @Environment(\.dismissImmersiveSpace) private var dismissImmersiveSpace
    @ObservedObject private var shell = XrShellModel.shared
    @State private var selectedLevel = "arena"
    @State private var worldScale = 1.0
    @State private var timeOfDay = "Noon"
    private static let scales: [Double] = [1, 2, 5, 10, 25]
    /// Engine day/night cycle: [0,1) with 0.5 = noon. "Cycling" resumes the
    /// authored ~50s/day cycle; everything else freezes the sun.
    private static let times: [(String, Double, Bool)] = [
        ("Morning", 0.35, true), ("Noon", 0.5, true),
        ("Sunset", 0.72, true), ("Night", 0.95, true),
        ("Cycling", 0.35, false),
    ]
    /// The engine's levels are authored larger than strict metric (the arena
    /// car is several units tall) — at 1 world-unit-per-meter a person reads
    /// knee-high. This baseline calibrates "Life size" to feel life-size in
    /// the arena; the picker's giant modes multiply on top. TUNE BY FEEL:
    /// stand next to the arena car — it should read as a real car.
    private static let baselineScale = 2.0

    /// The scene-picker entry that is not a level: mixed-immersion
    /// passthrough of the user's real room. Phase A shows the selected VR
    /// content over passthrough; the dedicated sandbox state lands next.
    static let sandboxEntry = "AR sandbox"

    /// Level shortnames from the bundled assets/levels/*.json (the sidecar
    /// .cameras.json files are not levels), plus the AR sandbox entry.
    static let levels: [String] = {
        guard let root = Bundle.main.resourceURL?
            .appendingPathComponent("assets/levels") else { return [sandboxEntry, "arena"] }
        let files = (try? FileManager.default
            .contentsOfDirectory(at: root, includingPropertiesForKeys: nil)) ?? []
        let names = files
            .filter { $0.pathExtension == "json" }
            .map { $0.deletingPathExtension().lastPathComponent }
            .filter { !$0.hasSuffix(".cameras") && !$0.hasSuffix(".json") }
        return [sandboxEntry] + (names.isEmpty ? ["arena"] : names.sorted())
    }()

    @State private var showSettings = false

    var body: some View {
        VStack(spacing: 16) {
            HStack {
                // Balance the trailing gear so the title stays centered.
                Image(systemName: "gearshape").opacity(0)
                Spacer()
                Text("Raytracer")
                    .font(.title)
                Spacer()
                Button {
                    showSettings = true
                } label: {
                    Image(systemName: "gearshape")
                }
                .buttonStyle(.borderless)
                .accessibilityLabel("Render settings")
            }
            Picker("Scene", selection: $selectedLevel) {
                ForEach(Self.levels, id: \.self) { Text($0).tag($0) }
            }
            .pickerStyle(.menu)
            .disabled(shell.inArena)
            Picker("Player scale", selection: $worldScale) {
                ForEach(Self.scales, id: \.self) {
                    Text($0 == 1 ? "Life size" : "\(Int($0))× giant").tag($0)
                }
            }
            .pickerStyle(.menu)
            // The sandbox is always life-size (room colliders assume it).
            .disabled(shell.inArena || selectedLevel == Self.sandboxEntry)
            Picker("Time of day", selection: $timeOfDay) {
                ForEach(Self.times, id: \.0) { Text($0.0).tag($0.0) }
            }
            .pickerStyle(.menu)
            .disabled(shell.inArena)
            Button(shell.inArena ? "Leave \(selectedLevel)" : "Enter \(selectedLevel)") {
                Task { @MainActor in
                    if shell.inArena {
                        await dismissImmersiveSpace()
                        shell.inArena = false
                    } else {
                        await enterScene()
                    }
                }
            }
            Text("In the scene: pinch and hold to aim (a ring marks the spot),\nrelease to teleport there. Hold ~1.2s to come back to this menu.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding(32)
        .sheet(isPresented: $showSettings) {
            RenderSettingsView()
        }
        .onAppear {
            shell.dismissSpace = { [dismissImmersiveSpace] in
                await dismissImmersiveSpace()
                XrShellModel.shared.inArena = false
            }
            // Headless verification path: the simulator's synthetic taps never
            // reach visionOS window UI, so scripted runs can't press the button.
            //   xcrun simctl launch <udid> com.glennsong.raytracer.visionspike \
            //       --auto-enter [--level=building_lab]
            for arg in ProcessInfo.processInfo.arguments {
                if arg.hasPrefix("--level=") {
                    let name = String(arg.dropFirst("--level=".count))
                    if Self.levels.contains(name) { selectedLevel = name }
                }
            }
            if ProcessInfo.processInfo.arguments.contains("--auto-enter"), !shell.inArena {
                Task { @MainActor in await enterScene() }
            }
        }
    }

    /// The button's "enter" half, shared with the --auto-enter launch argument.
    private func enterScene() async {
        let sandbox = selectedLevel == Self.sandboxEntry
        shell.immersion = sandbox ? .mixed : .full
        rt_vision_set_passthrough(sandbox ? 1 : 0)
        // "sandbox" is the engine's reserved name for SandboxState (no level
        // file). It is always life-size: sandbox content is authored in real
        // metres and the room colliders assume scale 1 — the giant modes
        // would put the real floor at the wrong height.
        rt_vision_set_level(sandbox ? "sandbox" : selectedLevel)
        rt_vision_set_world_scale(sandbox ? 1.0 : worldScale * Self.baselineScale)
        if let t = Self.times.first(where: { $0.0 == timeOfDay }) {
            rt_vision_set_pref_double("daynight.timeOfDay", t.1)
            rt_vision_set_pref_bool("daynight.paused", t.2 ? 1 : 0)
        }
        if await openImmersiveSpace(id: "arena") == .opened {
            shell.inArena = true
        }
    }
}

/// Render settings panel (gear icon in the launcher window). Every control
/// applies LIVE — the engine drains pref changes once per frame — and "Save"
/// persists them to the app's writable settings.json so they survive
/// relaunches. Values are read back from the engine on open, so the panel
/// shows the effective state (saved settings or engine defaults), not its
/// own guesses. Opening it before entering a scene also works: the values
/// queue and apply at the next boot.
struct RenderSettingsView: View {
    @Environment(\.dismiss) private var dismiss

    // Engine defaults (renderer.h param structs) — used as getter fallbacks
    // before a scene has booted, and to restore the UI after Reset.
    private static let defaults = Defaults()
    struct Defaults {
        let bloomOn = true, ssaoOn = true, ssrOn = true
        let bloomIntensity = 0.3, bloomThreshold = 1.0
        let ssaoIntensity = 0.8
        let ssrStrength = 0.5
        let contrast = 1.0, saturation = 1.0
        let tonemap = 0.0
    }

    @State private var bloomOn = Self.defaults.bloomOn
    @State private var bloomIntensity = Self.defaults.bloomIntensity
    @State private var bloomThreshold = Self.defaults.bloomThreshold
    @State private var ssaoOn = Self.defaults.ssaoOn
    @State private var ssaoIntensity = Self.defaults.ssaoIntensity
    @State private var ssrOn = Self.defaults.ssrOn
    @State private var ssrStrength = Self.defaults.ssrStrength
    @State private var contrast = Self.defaults.contrast
    @State private var saturation = Self.defaults.saturation
    @State private var tonemap = Self.defaults.tonemap
    @State private var saved = false

    var body: some View {
        NavigationStack {
            Form {
                Section("Bloom") {
                    Toggle("Enabled", isOn: $bloomOn)
                        .onChange(of: bloomOn) { _, v in
                            rt_vision_set_pref_bool("bloom.enabled", v ? 1 : 0)
                        }
                    slider("Intensity", $bloomIntensity, 0...0.5, "bloom.intensity")
                    slider("Threshold", $bloomThreshold, 0...3, "bloom.threshold")
                }
                Section("Ambient occlusion") {
                    Toggle("Enabled", isOn: $ssaoOn)
                        .onChange(of: ssaoOn) { _, v in
                            rt_vision_set_pref_bool("ssao.enabled", v ? 1 : 0)
                        }
                    slider("Intensity", $ssaoIntensity, 0...3, "ssao.intensity")
                }
                Section("Reflections") {
                    Toggle("Enabled", isOn: $ssrOn)
                        .onChange(of: ssrOn) { _, v in
                            rt_vision_set_pref_bool("ssr.enabled", v ? 1 : 0)
                        }
                    slider("Strength", $ssrStrength, 0...1, "ssr.blendStrength")
                }
                Section("Look") {
                    Picker("View transform", selection: $tonemap) {
                        Text("ACES").tag(0.0)
                        Text("AgX").tag(1.0)
                    }
                    .onChange(of: tonemap) { _, v in
                        rt_vision_set_pref_double("tonemap.op", v)
                    }
                    slider("Contrast", $contrast, 0.5...1.5, "grade.contrast")
                    slider("Saturation", $saturation, 0...2, "grade.saturation")
                }
                Section {
                    Button(saved ? "Saved ✓" : "Save to device") {
                        rt_vision_save_settings()
                        saved = true
                    }
                    Button("Reset to defaults", role: .destructive) {
                        rt_vision_reset_render_prefs()
                        let d = Self.defaults
                        bloomOn = d.bloomOn; bloomIntensity = d.bloomIntensity
                        bloomThreshold = d.bloomThreshold
                        ssaoOn = d.ssaoOn; ssaoIntensity = d.ssaoIntensity
                        ssrOn = d.ssrOn; ssrStrength = d.ssrStrength
                        contrast = d.contrast; saturation = d.saturation
                        tonemap = d.tonemap
                        saved = false
                    }
                } footer: {
                    Text("Changes apply immediately. Save keeps them for future launches.")
                }
            }
            .navigationTitle("Render settings")
            .toolbar {
                ToolbarItem(placement: .confirmationAction) {
                    Button("Done") { dismiss() }
                }
            }
        }
        .frame(minWidth: 380, minHeight: 520)
        .onAppear { load() }
    }

    /// Slider row that pushes its value to the engine as it moves.
    private func slider(_ label: String, _ value: Binding<Double>,
                        _ range: ClosedRange<Double>, _ key: String) -> some View {
        VStack(alignment: .leading, spacing: 4) {
            HStack {
                Text(label)
                Spacer()
                Text(String(format: "%.2f", value.wrappedValue))
                    .foregroundStyle(.secondary)
                    .monospacedDigit()
            }
            Slider(value: value, in: range) { _ in }
                .onChange(of: value.wrappedValue) { _, v in
                    rt_vision_set_pref_double(key, v)
                    saved = false
                }
        }
    }

    private func load() {
        let d = Self.defaults
        bloomOn = rt_vision_get_pref_bool("bloom.enabled", d.bloomOn ? 1 : 0) != 0
        bloomIntensity = rt_vision_get_pref_double("bloom.intensity", d.bloomIntensity)
        bloomThreshold = rt_vision_get_pref_double("bloom.threshold", d.bloomThreshold)
        ssaoOn = rt_vision_get_pref_bool("ssao.enabled", d.ssaoOn ? 1 : 0) != 0
        ssaoIntensity = rt_vision_get_pref_double("ssao.intensity", d.ssaoIntensity)
        ssrOn = rt_vision_get_pref_bool("ssr.enabled", d.ssrOn ? 1 : 0) != 0
        ssrStrength = rt_vision_get_pref_double("ssr.blendStrength", d.ssrStrength)
        contrast = rt_vision_get_pref_double("grade.contrast", d.contrast)
        saturation = rt_vision_get_pref_double("grade.saturation", d.saturation)
        tonemap = rt_vision_get_pref_double("tonemap.op", d.tonemap)
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
    private let menuHoldSeconds: TimeInterval = 1.2

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
    @ObservedObject private var shell = XrShellModel.shared
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
        // Mixed = passthrough (AR sandbox), full = VR scenes. The selection is
        // set by the launcher BEFORE the space opens; changing it mid-session
        // is not supported. VERIFY: selection-binding form of immersionStyle
        // with both styles offered, per Apple's immersive Metal template.
        .immersionStyle(selection: $shell.immersion, in: .mixed, .full)
    }
}
