# `src/visionos_app/` — Agent Guide

The visionOS host: a SwiftUI immersive space that boots the engine and ticks it
once per compositor frame. Two files do the work — `VisionSpikeApp.swift` (the
entry point visionOS requires) and `vision_spike.mm` (everything after).

> The renderer is NOT here. visionOS runs the same `MetalRenderer` as the macOS
> viewer; only presentation differs, and that lives in `CompositorSurface` behind
> the `PresentationSurface` seam in `../renderer/metal/metal_renderer.mm`. There
> is deliberately no second Metal backend — see the root `AGENTS.md`,
> "Use the technology you already have".

## Building

Simulator (no signing, no account needed):

```bash
cmake -S . -B build-visionos-sim -G Xcode \
      -DCMAKE_SYSTEM_NAME=visionOS -DCMAKE_OSX_SYSROOT=xrsimulator
cmake --build build-visionos-sim --target VisionApp
xcrun simctl boot "Apple Vision Pro"
xcrun simctl install booted build-visionos-sim/Debug-xrsimulator/VisionApp.app
xcrun simctl launch booted com.glennsong.raytracer.visionspike
```

Device — needs a signing team (see below):

```bash
cmake -S . -B build-visionos -G Xcode \
      -DCMAKE_SYSTEM_NAME=visionOS -DCMAKE_OSX_SYSROOT=xros \
      -DRT_VISIONOS_TEAM_ID=<your-team-id>
cmake --build build-visionos --target VisionApp
```

Both cross-compile, so they need their own build directory — never the macOS
`build/`.

## Device bring-up (one-time, and NOT scriptable)

Every step below needs a human: they involve an Apple ID, a passcode, or the
headset's own UI.

1. **Sign in to Xcode.** Xcode > Settings > Accounts > add your Apple ID. Until
   this is done, `security find-identity -v -p codesigning` reports
   *"0 valid identities found"* and no device build can be installed. A free
   account is enough for development (7-day provisioning).
2. **Get the team id.** Xcode > Settings > Accounts > Manage Certificates, or
   the ten characters in parentheses from `security find-identity`. Pass it as
   `RT_VISIONOS_TEAM_ID`.
3. **Enable Developer Mode on the Vision Pro.** Settings > Privacy & Security >
   Developer Mode, then restart the headset.
4. **Pair.** With both on the same network, the headset appears under
   Xcode > Window > Devices and Simulators. Confirm the pairing prompt in the
   headset. `xcrun devicectl list devices` should then show it.

## Reading the logs

The app's own lines are prefixed `[vision]`. The compositor's complaints are
worth watching for — they are specific and they are usually right:

```bash
xcrun simctl spawn booted log show --last 1m --predicate 'process == "VisionApp"'
```

`RT_DEBUG_VIEW` and `RT_WIREFRAME` work here (1=AO 2=SSR 3=depth 4=normals
5=shadow 6=albedo 7=facing 8=cascades); pass them through simctl with the
`SIMCTL_CHILD_` prefix:

```bash
SIMCTL_CHILD_RT_DEBUG_VIEW=6 xcrun simctl launch booted com.glennsong.raytracer.visionspike
```

The albedo view is what identified the target-sizing bug — geometry stopped at
exactly the depth texture's width. Reach for these before guessing.

## What the simulator cannot tell you

- **Stereo.** `cp_drawable_get_view_count` returns **1** in the simulator and 2
  on device. Per-eye rendering and parallax are unverifiable here, by
  construction.
- **Frame budget.** Simulator timings say nothing about an M2 at 90 Hz.
- **Whether an unanchored frame displays.** The simulator draws drawables with
  no device anchor; the device drops them silently.

That last one has already caused one wrong conclusion in this project. Treat
"it works in the simulator" as necessary, never sufficient.

## Conventions that bite

- The bundle is READ-ONLY. Assets resolve through `engine::assetPath` against
  the bundle root (`engine/asset_root.h`); anything the engine writes —
  `settings.json` — must go to the Documents directory instead.
- Audio runs on the NULL backend deliberately. Opening a real device deadlocks
  AURemoteIO unless an AVAudioSession is configured and activated first, and
  CoreAudio then aborts the process on its RPC timeout, so the usual
  open-failed fallback never runs.
- The CompositorServices frame protocol has no mid-frame escape: once a frame is
  taken it must go `start_submission` → `query_drawable` → `present` →
  `end_submission`. Each shortcut has its own "BUG IN CLIENT" message.
