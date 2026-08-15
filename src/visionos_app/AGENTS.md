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

## Current state: stereo, head-tracked, unverified by eye

The monoscopic bridge is GONE. `endFrame` runs the view-dependent pass graph
once per view, each pass reading camera uniforms pointed at that eye; each
view's colour and depth land where that view's texture map says (texture index
+ array slice + viewport — never "view v = slice v"); the engine camera follows
the device-anchor pose; and `present()` only presents. If you are reading a
comment or a doc that says otherwise, it predates
`57a4366 xr: per-eye rendering through the drawable's texture maps`.

What is genuinely unresolved is **verification**, and it is worth being blunt
about why. `cp_drawable_get_view_count` returns 1 in the simulator, so the
two-view path only ever executes on a device. Its characteristic failures —
eyes swapped, an offset dropped, separation not tracking world scale — do not
announce themselves: they read as vague discomfort, or as nothing at all to
anyone who does not fuse stereo. "Put it on and look" is not a test.

So the eye math does not live in this backend. It lives in
`engine/xr/xr_view_math.h`, in pure engine types, and `tests/test_xr_stereo.cpp`
pins it numerically on any host — separation equals IPD x world scale, the
separation axis follows head yaw, a point ahead projects right-of-centre in the
left eye (the swapped-eye check), disparity falls off inversely with distance.
Those tests are mutation-checked: breaking the scale, the composition order, or
the eye offset each makes them fail.

On device, `[xr] stereo:` reports the same quantities per run — view count,
both eye positions, separation, and the implied IPD. A healthy headset line
reads `views=2` with an ipd near 0.06 m. `views=1 MONO` means the simulator.
Read the numbers; do not trust the sensation.

## Room surfaces (ADR-0078)

Two more ARKit providers ride the same `ar_session_t` as world/hand tracking:
plane detection (semantic flat surfaces — floor/wall/table/… with outlines)
and scene reconstruction (the triangle mesh of everything else, as chunk
anchors). Callbacks land on a serial queue, geometry is copied out (it does
not outlive the callback) into `engine::XrSurfaceUpdate`s and pushed through
`XrSurfaceStore`; `XrSurfaceSystem` drains, bookkeeps via the host-tested
`XrSurfaceLedger`, and draws — chunks as classification-tinted meshes, planes
as outlines. The drawing is OFF by default (`RT_XR_SURFACES=1` shows it at
boot, or the settings toggle live); ingest, colliders, occlusion and shadow
catching always run regardless.

Read health from the census line, not the render:
`[xr] surfaces: total=… floor=1 wall=3 … tris=… markers=… floorY=-0.02m` —
counts should climb as you look around, the classes should match the actual
room, and `floorY` (ORIGIN space, metres) should sit near 0 when standing on
the floor where the app launched. In the simulator both providers log
`unsupported` and zero surfaces is CORRECT, not a failure.

### The placement demo (and what it measures)

Plane dimensions print as they settle (`[xr] plane table 1.20m x 0.75m …`,
re-logged when refinement moves a dimension > 5 cm) and each plane draws its
measured extent as a dimmed rectangle, since the engine has no 3D text.

A QUICK pinch (same < 0.8 s window as teleport) while gazing at a detected
plane within ~3 real metres drops a 10 cm magenta cube on it — and consumes
the pinch, so the gesture does not also teleport; that interception is why
XrSurfaceSystem registers before PlayerSystem (arena_state.cpp) and why the
ordering must not be "tidied". Gazing past every plane leaves teleport alone.

Markers are stored in the PLANE'S anchor space, so they are the anchoring
probe: as ARKit refines or re-poses a plane, its cubes ride along. Watch a
cube on your desk while walking the room and returning — it should sit still
to within roughly a centimetre. If the runtime removes a plane (merges it),
its markers freeze at their last world pose and the log says so — evidence
never silently vanishes. Census carries `markers=N`.

First-build caveat: the plane/mesh C symbols in `startSurfaceProviders` and
the conversion helpers above `CompositorSurface` are marked VERIFY — they
follow the provider conventions this file already uses, but were authored
where visionOS code cannot compile. Check spellings against the SDK's ARKit
headers on first build; every uncertain symbol is confined to that one block.

## The AR sandbox and the XR interaction stack (ADR-0079)

The "AR sandbox" scene (`sandbox_state.cpp`, mixed immersion) is the proving
ground for the engine's XR INTERACTION STACK — which is deliberately NOT in
this directory. Pure cores live in `engine/xr/` (`xr_gestures`, `xr_touch`,
`xr_palette`, `xr_grasp` — host-tested, no SDK types), physics primitives in
`PhysicsWorld` (Hand/Grip layers, grip-spring/hinge/slider constraints,
`activeContacts`), and `HandInteractionSystem` composes them. This backend
only fills `XrState` (hand skeletons queried at PREDICTED presentation time —
the `query_anchors_at_timestamp` call is a VERIFY symbol with a
`get_latest_anchors` fallback); an OpenXR backend would reuse everything else
unchanged.

Holding is one 6-DOF spring per hand (hook by pinch near the oriented
surface, or by CLOSURE — fingertips on opposing sides); the held object stays
dynamic. Releases are pinch-open or grip-memory opening; a gentle release
(< 0.25 m/s) near a support takes the placement-assist ghost's pose (pref
`xr.placeAssist`). Tracking dropouts coast on `XrHandMemory` for ~150ms.

Device checks for a grasp build: hook logs say
`[xr] hook R crate (closure, 3 contacts)`; a crate grabbed upside down hangs
as grabbed and DRAGS along the real table while carried; a fist (no pinch)
takes a pillar; opening the hand mid-swing throws with real momentum; the
chest's lid hinges while the chest is held in the other hand; the bolt slides
6cm and springs back; a toss between hands logs `[xr] toss L->R crate`.

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
- Audio opens the REAL backend only after `vision_spike.mm` configures and
  activates an AVAudioSession (VERIFY on first build); if either call fails it
  stays on the null backend. Never open a device without the session: AURemoteIO
  deadlocks and CoreAudio aborts the process on its RPC timeout, so the usual
  open-failed fallback never runs.
- The CompositorServices frame protocol still has no mid-frame escape once
  `start_submission` is called — but on visionOS 26 an EMPTY drawable array
  BEFORE submission is the one legal bail-out (the frame is cancelled).
- Deployment target is pinned in CMake (currently 26.0) — keep it ≤ the
  headset's installed OS, and never let it silently track the SDK.
