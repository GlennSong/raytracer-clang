# Building for Apple Vision Pro (visionOS)

This guide covers setting up a Mac to build, run, and deploy the engine's
visionOS app — in the simulator (no Apple account needed) and on a physical
Vision Pro. It is written to be reproducible from a clean machine; nothing in
it is specific to any particular Mac, headset, or Apple ID.

The one-paragraph architecture summary: there is **no separate visionOS
renderer**. The same `MetalRenderer` that drives the macOS viewer renders the
headset; only *presentation* differs, behind the `PresentationSurface` seam in
`src/renderer/metal/metal_renderer.mm` (`LayerSurface` for a macOS window,
`CompositorSurface` for CompositorServices). The visionOS host app
(`src/visionos_app/`) is a small SwiftUI shell: a launcher window plus an
immersive space that hands the engine a `cp_layer_renderer_t` as an opaque
native handle and ticks it once per compositor frame. Per-eye stereo, head
tracking, hand tracking, and spatial input all flow through the engine's XR
module (`src/engine/xr/`). For the display contract the device actually
enforces (and the debugging playbook for "black view, clean logs"), read
[`src/visionos_app/AGENTS.md`](../src/visionos_app/AGENTS.md).

---

## Prerequisites

- **Xcode 26 or newer** with the **visionOS SDK** installed (Xcode ▸ Settings ▸
  Components, or `xcodebuild -downloadPlatform visionOS`). Confirm with
  `xcodebuild -showsdks` — you want `xros` and `xrsimulator` entries.
- **CMake ≥ 3.16**.
- The repo's submodules:

  ```bash
  git submodule update --init --recursive
  ```

- For the **simulator**: nothing else. No Apple account, no signing.
- For a **physical Vision Pro**: any Apple ID (a free one works — see the
  signing section for the 7-day caveat) and a headset with **visionOS 26.0 or
  newer** (the deployment target pinned in CMakeLists.txt; keep the headset's
  OS at or above it, and at or below your Xcode SDK).

## How the build is put together

`cmake -DCMAKE_SYSTEM_NAME=visionOS -G Xcode` cross-compiles the whole engine
and generates an Xcode project with a **`VisionApp`** target. Simulator and
device are separate configures into separate build directories (they are
different sysroots), and neither shares a directory with the macOS `build/`.

Things the CMake does for you that are worth knowing about:

- **Engine + submodules cross-compile.** Jolt, Lua, and miniaudio build for
  visionOS. Three platform warts are handled in CMakeLists.txt: Jolt's
  `-pthread` interface flag is re-scoped so it never reaches `swiftc`
  (which rejects it), Lua builds with `LUA_USE_IOS` (sandboxed platforms have
  no `system()`), and miniaudio's single C file compiles as Objective-C on
  embedded Apple platforms because its Core Audio path touches AVAudioSession.
- **Assets and shaders are copied INTO the app bundle** (`assets/`,
  `shaders/`), and the app points the engine's asset root at the bundle at
  startup (`src/engine/asset_root.h`). A sandboxed bundle has no useful
  working directory, so nothing resolves relative to CWD on device.
- **Metal shaders compile at runtime from source** — there is no prebuilt
  `default.metallib`. CI's `tools/check-metal-shaders.sh` offline-compiles the
  same concatenation the runtime uses, so shader breakage is caught at PR time
  rather than first launch.
- **`Info.plist` is generated from `src/visionos_app/Info.plist.in`**, which
  carries the privacy usage descriptions visionOS requires
  (`NSWorldSensingUsageDescription`, `NSHandsTrackingUsageDescription` —
  ARKit kills the app at provider start if these are missing). The template
  is registered as a CMake configure dependency, so editing it correctly
  regenerates the bundle plist on the next build.
- **The app writes nothing into its bundle.** Settings persist to the app's
  Documents directory (`settings.json`); audio runs on the null backend
  deliberately (see AGENTS.md for why opening a real device would hang).
- The bundle identifier defaults to a repo-specific value set in
  CMakeLists.txt (`MACOSX_BUNDLE_GUI_IDENTIFIER`). Change it there if you fork
  the project — bundle ids are global to App Store Connect provisioning.

---

## Building for the simulator (no signing required)

```bash
cmake -S . -B build-visionos-sim -G Xcode \
      -DCMAKE_SYSTEM_NAME=visionOS -DCMAKE_OSX_SYSROOT=xrsimulator
cmake --build build-visionos-sim --target VisionApp
```

Boot a simulator and install (device names/UDIDs come from
`xcrun simctl list devices`):

```bash
xcrun simctl boot "Apple Vision Pro"
xcrun simctl install booted build-visionos-sim/Debug-xrsimulator/VisionApp.app
xcrun simctl launch booted <bundle-id>
```

The app opens its launcher window: pick a scene, world scale, and time of day,
then **Enter**. In the simulator you drive the camera with WASD + drag.

Headless/scripted runs (useful for automation — simulated taps do not reach
visionOS window UI, so there are launch arguments instead):

```bash
xcrun simctl launch booted <bundle-id> --auto-enter --level=building_lab
```

Engine debug views work through simctl's environment prefix:

```bash
SIMCTL_CHILD_RT_DEBUG_VIEW=3 xcrun simctl launch booted <bundle-id> --auto-enter
```

(1=AO 2=SSR 3=depth 4=normals 5=shadow 6=albedo 7=facing 8=cascades.)

**Know what the simulator cannot tell you.** It draws frames the real
compositor would discard: it accepts garbage depth, wrong gamma (it skips the
device's display transform, so everything renders about one gamma step dark —
this is a known simulator lie, not an engine bug), and it reports one view
where the device renders two, so stereo and per-eye bugs are invisible.
Treat "works in the simulator" as necessary, never sufficient. The full list
of simulator blind spots is in `src/visionos_app/AGENTS.md`.

---

## Device signing setup (one-time)

1. **Sign in to Xcode** with your Apple ID (Xcode ▸ Settings ▸ Accounts). A
   free account works; its provisioning profiles expire after 7 days, so
   expect to rebuild/reinstall weekly. Paid accounts don't have that limit.
2. **Find your team ID.** It is the `OU` field of your development
   certificate:

   ```bash
   security find-certificate -c "Apple Development" -p | openssl x509 -noout -subject
   ```

   ⚠️ The ten characters printed *in parentheses* by
   `security find-identity -v -p codesigning` are a **certificate id, not the
   team id** — using them as the team fails with "No Account for Team" in both
   Xcode and `xcodebuild`, which reads exactly like a signed-out account and
   is a very effective wild-goose chase.
3. **Enable Developer Mode on the headset**: Settings ▸ Privacy & Security ▸
   Developer Mode, then restart it.
4. **Pair with Xcode**: with Mac and headset on the same network, the device
   appears under Xcode ▸ Window ▸ Devices and Simulators; confirm the prompt
   in the headset. `xcrun devicectl list devices` should then show it.
5. **After the first install, trust your developer profile on the headset**:
   Settings ▸ General ▸ VPN & Device Management. Until then, launches fail
   with `FBSOpenApplicationErrorDomain error 3`.

## Building and deploying to the device

Configure with your team id, then build — `-allowProvisioningUpdates` lets
`xcodebuild` create/refresh the provisioning profile from the CLI, no Xcode
GUI needed:

```bash
cmake -S . -B build-visionos-dev -G Xcode \
      -DCMAKE_SYSTEM_NAME=visionOS -DCMAKE_OSX_SYSROOT=xros \
      -DRT_VISIONOS_TEAM_ID=<your-team-id>
xcodebuild -project build-visionos-dev/raytracer.xcodeproj -scheme VisionApp \
  -configuration Debug -destination 'generic/platform=visionOS' \
  -allowProvisioningUpdates build
```

Install and launch (get `<device-udid>` from `xcrun devicectl list devices`):

```bash
xcrun devicectl device install app --device <device-udid> \
  build-visionos-dev/Debug-xros/VisionApp.app
xcrun devicectl device process launch --device <device-udid> <bundle-id>
```

You can equally open `build-visionos-dev/raytracer.xcodeproj` in Xcode and
hit Run — that is the only way to see os_log output (compositor complaints,
frame-drop reasons), which `devicectl --console` does not capture.

Device tooling behaviors that look like failures but aren't:

- `devicectl` commands **queue while the headset is in standby** and complete
  when it is worn and unlocked. A hang is not an error. Developer launches on
  a locked device fail with error 12040 ("device is locked").
- Taking the headset off SIGKILLs console-attached processes and pauses the
  world-tracking provider — anchor errors at session edges are noise.
- A network timeout installing usually just means the headset is asleep; wake
  it and retry.

## First launch on device

Launch from the Home View. The app opens window-first (an immersive-only
launch never composites on current visionOS — see the display contract), with
pickers for scene, world scale, and frozen time-of-day. **Enter** opens the
immersive space. On first run visionOS shows the hand-tracking permission
dialog — allow it to get the tracked hand skeletons and pinch input.

In the headset:

- **Gaze + quick pinch** (release under ~0.8 s): teleport along your gaze ray
  (a ring marks the spot while you hold).
- **Long pinch** (~1.2 s): dismiss the space back to the launcher menu.
- **Gear icon** on the launcher window: live render settings (bloom, ambient
  occlusion, reflections, tone map, color grade) with sliders that apply at
  90 fps; **Save to device** persists them to the app's Documents.
- **Gamepads** pair to the headset over Bluetooth (PS5/Xbox/Switch via
  GCController) and drive locomotion directly.

## Troubleshooting

| Symptom | Likely cause / fix |
| --- | --- |
| App killed at launch citing `NS...UsageDescription` | The installed bundle's Info.plist predates the usage keys — rebuild (the plist template is a configure dependency; a stale `build-*/CMakeFiles/VisionApp.dir/Info.plist` can also be deleted to force regeneration). |
| Black view, healthy logs | Work the display-contract checklist in `src/visionos_app/AGENTS.md` — drawable queries, device anchors, depth, gamma, scene structure. |
| "No Account for Team" from xcodebuild | You passed a certificate id as the team id — see signing step 2. |
| `invalid reuse after initialization failure` build error | Corrupted Swift module cache: `rm -rf ~/Library/Developer/Xcode/DerivedData/ModuleCache.noindex`. |
| Simulator wedged (`simctl` hangs, zombie app) | A suspended app under a dead debugserver: kill the debugserver, then `simctl shutdown` / `boot` the sim. |
| Everything renders dark in the simulator | Known simulator gamma lie (no display transform); verify on device. |
| Assets missing on device only | The bundle ships only what exists at configure time — gitignored downloads (e.g. sample glTF models) must be present in `assets/` before building; the CMake warns when a referenced one is absent. |

## Related reading

- [`src/visionos_app/AGENTS.md`](../src/visionos_app/AGENTS.md) — the
  visionOS display contract, device bring-up, and debugging playbook.
- [`docs/decisions.md`](decisions.md) — the ADRs behind the single-renderer
  architecture and the XR module.
- [`docs/web-build.md`](web-build.md) — the same engine compiled for the
  browser; a useful contrast in how far the `Renderer` seam stretches.
