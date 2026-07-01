# Vision Pro Port — Plan

Status: **plan / not started.** What it takes to run the engine on Apple Vision
Pro, phased from "flat window in shared space" (cheap, reuses the renderer
unchanged) to "fully immersive stereo" (the real VR experience). Companion to
`docs/compute-shaders-plan.md`'s cross-API notes; promote committed choices to
ADRs in `docs/decisions.md`.

## Why this is tractable: the seams already exist

The expensive half — the GPU code — is already portable. Metal is identical on
macOS/iOS/visionOS: the whole `MetalRenderer::Impl`, all six `.metal` shaders,
and the runtime `newLibraryWithSource:` compile carry over unchanged. And the
engine's host seams were built for exactly this kind of embedding:

- **`Application` host-driven mode** (`application.h`): `begin()` /
  `runFrame()` / `end()`, added for the Qt editor. A visionOS host drives
  frames the same way — no engine-loop changes.
- **`HostedWindow`** (`hosted_window.h`): platform-free `Window` impl where the
  host owns the real window, supplies the native handle, and forwards events
  through the `inject*` API. Unit-tested headlessly. This *is* the visionOS
  window; we don't write a new `Window`, we write a host that feeds this one.
- **Opaque render handle** (ADR-0001): `metal_renderer.mm` already accepts
  either an `NSWindow*` or a view (`initialize`, ~line 259). Needs one more
  accepted shape (a `CAMetalLayer*` / `UIView*`), not a redesign.
- **Gamepad already ports**: `gamepad_gc.mm` imports only GameController +
  Foundation (ADR-0013) — compiles for visionOS as-is. The IOKit/GLFW
  fallback path is macOS-only and simply isn't built.

What does **not** carry over: GLFW (desktop-only — but engine_core doesn't link
it; only the viewer host does), AppKit (`NSWindow`/`backingScaleFactor` uses in
`metal_renderer.mm`), ImGui's GLFW backend, and the **single-view assumption**
— one `CameraState` in `RenderView`, one `setCamera` (`render_system.cpp:50`),
one `CAMetalLayer` drawable, one present (`endFrame`).

## The two visionOS modes (this fork drives the phasing)

1. **Shared space / windowed**: the app renders into a `CAMetalLayer` on a flat
   panel floating in the room. Mono, no head tracking — *single-view*, so the
   existing renderer runs unchanged. This is "the iPad port shown on a
   headset."
2. **Fully immersive**: no `CAMetalLayer` at all. A **CompositorServices**
   `cp_layer_renderer` hands you a drawable (a 2-slice texture array, one per
   eye) plus per-eye view/projection derived from **ARKit** head pose; you
   render stereo and submit on the compositor's schedule. This forces the
   stereo rework and is the only mode that's actually VR.

Phase A ships mode 1 and de-risks all platform plumbing; Phases B/C build
mode 2 on top of it.

---

## Phase A — visionOS app, shared space (renderer unchanged)

Goal: the viewer's game running on device/simulator as a flat floating window.
Everything here is also 90% of an iPhone/iPad port.

- **A1 — Build engine_core for visionOS.** engine_core is C++17,
  windowing-free by design (CMake keeps GLFW host-side). Options: CMake Xcode
  generator or an Xcode app target that consumes engine_core as a static lib.
  Dependency audit: Lua (pure C — fine), stb/tinygltf/json (header-only —
  fine), Jolt (claims iOS-family support — **verify** on visionOS, else
  `RT_ENABLE_PHYSICS=OFF` for the first light), ImGui **off**
  (`RT_ENABLE_IMGUI=OFF`; its GLFW backend can't exist here).
- **A2 — Asset/shaders path resolution.** The Metal backend reads the six
  `.metal` files from CWD-relative paths at init (`metal_renderer.mm` ~301) and
  levels/scripts load relative too. App bundles have no useful CWD: add a
  resource-root (set once by the host from `NSBundle.mainBundle.resourcePath`)
  that all disk reads prefix. Small, engine-wide, benefits the desktop too
  (fixes "must run from repo root").
- **A3 — AppKit → UIKit split in the Metal backend.** Guard the
  `<AppKit/AppKit.h>` import and the `NSWindow`/`hostView` handle-probing with
  `#if TARGET_OS_OSX`; add a branch accepting a `CAMetalLayer*` directly and a
  host-supplied `contentsScale` (no `backingScaleFactor` off-desktop). The
  render code below `initialize` doesn't change.
- **A4 — The visionOS host app.** SwiftUI `WindowGroup` → a
  `UIViewRepresentable` whose `UIView` is `CAMetalLayer`-backed → a small
  Obj-C++ shim that: creates `HostedWindow`, `setNativeHandle(layer)`, builds
  `Application` with the game states (as `viewer_main.cpp` does), calls
  `begin()`, then drives `runFrame()` from a `CADisplayLink`. Touch/gesture
  events forward through `HostedWindow::inject*`.
- **A5 — Input for a touch/eye platform.** First light: **require a gamepad**
  (GCController path works today; visionOS pairs PS/Xbox controllers). Then a
  minimal touch scheme (virtual sticks) via the inject API. Keyboard bindings
  in `InputMap` stay for Mac; no engine changes, just new event sources.

Deliverable: the arena level playable on a Vision Pro (simulator first) as a
flat window, gamepad-driven. Zero renderer changes beyond A3.

## Phase B — Fully immersive stereo (two-pass first, correctness over speed)

Goal: true VR — head-tracked stereo in an `ImmersiveSpace`. Ships unoptimized
(two render passes per frame); Phase C makes it fast.

- **B1 — CompositorServices frame loop.** An `ImmersiveSpace` +
  `CompositorLayer` host: per frame, `queryNextFrame` → wait optimal input time
  → ARKit `worldTracking` head pose → call the engine's `runFrame()` → submit.
  Replaces `CADisplayLink`; `Application`'s host-driven mode absorbs this — the
  compositor is just a stricter Qt.
- **B2 — Externally-provided view/projection.** Today `CameraState` carries
  fov/aspect and the backend *builds* the matrix (`setCamera`,
  `metal_renderer.mm:1632`). The headset *dictates* per-eye view transforms
  and asymmetric (tangent-based) projections. Add a matrix-provided mode:
  `CameraState` (or a stereo sibling on `RenderView`) optionally carries
  explicit per-eye view + projection; gameplay still positions the *head*
  (the existing camera system output becomes the head pose the eye offsets
  compose with). `render_system.cpp:50` is the single call site to extend.
- **B3 — Presentation-target seam in `endFrame`.** Abstract "where the frame
  goes": today `[metalLayer nextDrawable]` → composite → `presentDrawable`
  (`metal_renderer.mm:1594, 3109`). Immersive mode instead targets one slice of
  the compositor drawable's texture array per eye and never touches
  `CAMetalLayer`. First implementation: run the frame graph **twice** (once per
  eye/slice) — main pass + post per eye; shadow cascades and probe bake are
  view-independent and run **once** (a real win the two-pass structure should
  exploit from day one).
- **B4 — VR correctness switches.** Disable the lens-sim post in immersive
  mode: chromatic aberration, vignette, distortion, and screen-space DOF are
  discomfort-inducing lies in a headset (the device has real optics). The
  `LensParams` defaults are already inert — immersive mode pins them. Audit
  **world scale**: the headset renders 1:1 physical meters; if arena/city units
  aren't meters, everything reads giant or tiny.
- **B5 — Input.** Gamepad (works from A5). Hand/pinch via ARKit hand tracking
  is a later, gameplay-design-shaped addition — not required to ship B.

Deliverable: head-tracked stereo walkthrough of a level, ~half the frame
budget wasted on duplicated encode — acceptable for validation, not shipping.

## Phase C — Stereo performance

- **C1 — Vertex amplification (single-pass stereo).** One encoded pass emits
  both eyes: camera uniforms become `viewProjection[2]`, vertex shaders
  (`lighting.metal` etc.) index by `[[amplification_id]]`, render targets
  become 2-slice arrays with `[[render_target_array_index]]`. Halves CPU
  encode and most bandwidth vs. B3's two passes.
- **C2 — Foveated rendering.** Bind the compositor-provided
  rasterization-rate map so peripheral pixels shade cheaper. Near-free
  quality/perf lever on this platform; the post stack must respect the rate
  map when sampling.
- **C3 — Per-eye post budget.** SSAO/SSR/bloom/DOF double in immersive mode.
  Levers: share one AO/SSR between eyes (reprojected), drop half-res further,
  or cut SSR in VR. Decide from device profiles, not in advance.
- **C4 — Frame pacing.** 90 Hz is a hard floor (dropped frames = nausea, not
  stutter). Wire `RenderStats` into the compositor's timing feedback; add a
  dynamic-resolution lever if needed.

## Risks / open questions

| Risk | Note |
|---|---|
| Jolt on visionOS | Probably fine (iOS-family support) — verify early in A1; physics-off fallback exists. |
| CI can't cover any of this | Like the Metal backend today: macOS/Xcode-only, needs on-device validation. Keep everything below the host seam headless-testable (HostedWindow already is). |
| Simulator ≠ device | The visionOS simulator renders mono at desktop perf; B/C validation needs hardware. |
| `double` in engine math | Fine for the renderer (backend already converts to simd floats); only matters if the compute-shader plan's path tracer lands. |
| App review / distribution | A raytracer viewer is uncontroversial; only matters if shipping beyond TestFlight. |

## Effort shape

A is mechanical (build + bundle paths + one `#if` split + a thin Swift host) —
the same work unlocks iPhone/iPad. B is the real engineering: two focused
seams (camera-in, drawable-out) plus the compositor loop. C is contained
renderer/shader work with clear Apple-documented patterns. Nothing requires
restructuring the engine — ADR-0001's opaque handle, the `HostedWindow`
inject model, and the host-driven frame loop were the load-bearing decisions,
and they hold.
