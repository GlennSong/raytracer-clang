# `src/visionos_app/` — Agent Guide

The visionOS host: a SwiftUI launcher window plus an immersive space that boots
the engine and ticks it once per compositor frame. `VisionSpikeApp.swift` is the
entry point visionOS requires; `vision_spike.mm` is everything after;
`TemplateRenderer.swift` is a kept diagnostic (see "Debugging playbook").

> The renderer is NOT here. visionOS runs the same `MetalRenderer` as the macOS
> viewer; only presentation differs, and that lives in `CompositorSurface` behind
> the `PresentationSurface` seam in `../renderer/metal/metal_renderer.mm`. There
> is deliberately no second Metal backend — see the root `AGENTS.md`,
> "Use the technology you already have".

## How the device actually displays a frame (the visionOS 26 contract)

Everything in this section was learned by presenting frames that ran a clean
90 fps protocol and displayed NOTHING. The simulator accepts all of these
mistakes and draws anyway; the device silently discards. When the view is black
but the logs are healthy, start here.

1. **Query the drawable ARRAY.** `cp_frame_query_drawable` (singular) is
   deprecated as of visionOS 26 — it still "works" but its drawable never
   reaches the displays. Use `cp_frame_query_drawables` → iterate; each
   drawable has a target (`built_in` = the displays, `capture` = recording).
   Render into `built_in`, anchor and present EVERY drawable. An empty array is
   a cancelled frame: discard it without `start_submission`. Query order on 26:
   drawables BEFORE `start_submission` (the WWDC23 order was the reverse).
2. **Every drawable needs a device anchor,** queried for that drawable's own
   presentation time from a RUNNING world-tracking provider. Unanchored
   drawables are dropped with a per-frame log line. Two traps behind "provider
   not running": the `ar_session_t` must be OWNED (a local under ARC is
   released on return, which stops the provider), and the provider pauses when
   the headset comes off.
3. **Depth is load-bearing.** The compositor reprojects the layer using the
   drawable's depth texture. Uninitialized depth = garbage reprojection.
   Everything at reverse-Z far (0.0 = infinity) = visible tile artifacts, and a
   frame with nothing nearer is dropped entirely — that is a BLACK VIEW with
   perfect logs. Until per-eye rendering writes real scene depth, the bridge
   clears drawable depth to a virtual plane ~2 m ahead computed from
   `cp_drawable_compute_projection`.
4. **Float color targets are consumed LINEAR.** With `rgba16Float` the
   compositor applies the display transform itself; shader-side gamma encoding
   on top reads as washed out. `PresentationSurface::targetEncodesSRGB()`
   accounts for this — the composite must apply the transfer function exactly
   once, and for float targets that means not at all.
5. **The scene structure matters.** An immersive-space-only app auto-opened via
   `CPSceneSessionRoleImmersiveSpaceApplication` runs its layer without ever
   compositing it on visionOS 26.3 (black void, hands breaking through). Launch
   window-first and open the space with a user-triggered `openImmersiveSpace`,
   the way Apple's Metal template does.
6. **Foveation and `.layered` are opt-in obligations.** Enabling foveation
   means every pass targeting drawable textures must attach the drawable's
   rasterization rate map; `.layered` means `renderTargetArrayLength` and
   vertex amplification. Configure only what the render path actually
   implements. Both are currently OFF pending per-eye rendering.

## Current state: the monoscopic bridge (and its known artifacts)

The pass graph runs once from the engine camera and composites into eye 0 of
the `built_in` drawable; `present()` mirrors that image into the other eye
slice/texture and any extra targets, and clears depth to the 2 m plane. The
consequences are known and expected until Task 3 (per-eye rendering):

- **Head-locked view.** The engine ignores the head pose, and each frame is
  re-anchored at the current pose, so the image sits dead ahead no matter where
  you look. Fix = feed the device-anchor pose into the engine camera.
- **Per-eye misalignment.** Both eyes get the IDENTICAL image but the
  compositor reprojects each eye separately against the 2 m plane; close one
  eye and the other's view is visibly shifted. Fix = render each eye with its
  own `cp_view` transform + `cp_drawable_compute_projection`.
- **No parallax, billboard feel.** All content depth-flattens to 2 m.

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
cmake -S . -B build-visionos-dev -G Xcode \
      -DCMAKE_SYSTEM_NAME=visionOS -DCMAKE_OSX_SYSROOT=xros \
      -DRT_VISIONOS_TEAM_ID=<your-team-id>
cmake --build build-visionos-dev --target VisionApp
```

Both cross-compile, so they need their own build directory — never the macOS
`build/`. Two build-system warts: the generated `Info.plist` is NOT refreshed
when `Info.plist.in` changes (delete
`build-*/CMakeFiles/VisionApp.dir/Info.plist` and re-run cmake), and `.metal`
sources are not compiled into a `default.metallib` (compile shaders at runtime
from source, as both the engine and `TemplateRenderer` do).

## Device bring-up (one-time, and NOT scriptable)

1. **Sign in to Xcode.** Xcode > Settings > Accounts > add your Apple ID. Until
   this is done, `security find-identity -v -p codesigning` reports
   *"0 valid identities found"*. A free account is enough (7-day provisioning —
   rebuild/reinstall weekly).
2. **Get the team id — it is the certificate's OU field, NOT the parenthetical
   in the certificate name.**

   ```bash
   security find-certificate -c "Apple Development" -p | openssl x509 -noout -subject
   ```

   The `OU=XXXXXXXXXX` value is the team. The ten characters in parentheses
   printed by `security find-identity` are a certificate id; using them as
   `DEVELOPMENT_TEAM` fails with *"No Account for Team"* in both Xcode and
   `xcodebuild`, which reads like a signed-out account and cost a full
   provisioning wild-goose chase. With the right team id,
   `xcodebuild -allowProvisioningUpdates` signs and provisions from the CLI —
   no Xcode GUI needed.
3. **Enable Developer Mode on the Vision Pro.** Settings > Privacy & Security >
   Developer Mode, then restart the headset.
4. **Pair.** With both on the same network, the headset appears under
   Xcode > Window > Devices and Simulators. Confirm the prompt in the headset.
   `xcrun devicectl list devices` should then show it.
5. **Trust the developer profile ON the headset** after the first install:
   Settings > General > VPN & Device Management > trust the Apple Development
   entry. Until then launches fail with `FBSOpenApplicationErrorDomain error 3`.

## Device tooling truths

- `devicectl` commands QUEUE while the headset is in standby — a hang is not an
  error, it completes when the device is worn/unlocked. Developer launches on a
  locked device fail with error 12040 ("device is locked").
- Taking the headset off SIGKILLs console-attached (`--console`) processes and
  pauses the world-tracking provider; the resulting anchor errors at session
  edges are noise.
- NEVER leave a retrying `--terminate-existing` launcher running in the
  background: it will kill every session the user starts from Xcode and look
  like "the app won't launch".
- The unanchored-drawable and compositor complaints go to os_log, which
  `devicectl --console` does NOT capture — only Xcode's console (or Console.app)
  shows them. A "clean" devicectl log proves nothing about presentation.
- A wedged simulator (`simctl` hangs, zombie app) can be a SIGSTOPped app under
  a dead Xcode debugserver: kill the debugserver, then `simctl shutdown`/`boot`.

## Debugging playbook (what actually converged)

When the device shows black but logs are healthy, differential-test against a
known-good render path instead of theorizing:

1. `TemplateRenderer.swift` (kept in-tree, one flag away in
   `VisionSpikeApp.swift`) is a port of Apple's "Immersive Space Renderer:
   Metal" template — plural drawables, rate maps, amplification, real depth. If
   it displays and the engine path doesn't, the delta is in the engine's frame
   structure; if it doesn't, the delta is app/project level.
2. `RT_VISION_MINIMAL_PROBE` in `vision_spike.mm` replaces the engine with bare
   anchored clears — the smallest legal frame. Note its lesson: clears at
   infinite depth are dropped; a probe needs near-depth content to be visible.
3. Creating the actual Xcode template project on the same headset splits
   device/OS problems from app problems in one run.

## Reading the logs

The app's own lines are prefixed `[vision]`. The compositor's complaints are
specific and usually right — but see above: they only appear in Xcode/Console.

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

- **Whether anything DISPLAYS.** The simulator draws deprecated-API drawables,
  unanchored drawables, garbage-depth frames, and wrong-gamma output without
  complaint. Every black-view root cause above passed the simulator.
- **Stereo.** `cp_drawable_get_view_count` returns **1** in the simulator and 2
  on device. Per-eye rendering and parallax are unverifiable here.
- **Frame budget.** Simulator timings say nothing about an M2 at 90 Hz.

Treat "it works in the simulator" as necessary, never sufficient.

## Conventions that bite

- The bundle is READ-ONLY. Assets resolve through `engine::assetPath` against
  the bundle root (`engine/asset_root.h`); anything the engine writes —
  `settings.json` — must go to the Documents directory instead.
- Audio runs on the NULL backend deliberately. Opening a real device deadlocks
  AURemoteIO unless an AVAudioSession is configured and activated first, and
  CoreAudio then aborts the process on its RPC timeout, so the usual
  open-failed fallback never runs.
- The CompositorServices frame protocol still has no mid-frame escape once
  `start_submission` is called — but on visionOS 26 an EMPTY drawable array
  BEFORE submission is the one legal bail-out (the frame is cancelled).
- Deployment target is pinned in CMake (currently 26.0) — keep it ≤ the
  headset's installed OS, and never let it silently track the SDK.
