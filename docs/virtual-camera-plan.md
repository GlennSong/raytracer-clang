# Virtual Camera System — Plan

Placeable camera entities in the world, switchable viewports, and a physically
parameterized lens model (focal length, aperture/DOF, distortion, chromatic
aberration) — usable both as live viewports in the interactive viewer and as a
"virtual filming" rig. The end-state vision includes driving a camera's pose
from an external device (e.g. an iPhone running an ARKit app), so the design
keeps a seam for externally-sourced poses from day one.

This is a planning document; each phase should land with its own ADR(s) in
`docs/decisions.md` where it makes a real architectural choice. Roadmap
cross-reference: Tier 3.4.

---

## Goals

1. **Cameras are ECS entities.** A camera is a `Transform` + a camera component
   in the existing sparse-set `World` (ADR-0006) — placeable, inspectable,
   destroyable like any other game object. No parallel "camera manager"
   bookkeeping that can drift from the world.
2. **Switchable viewports.** The viewer can render from the editor camera
   (existing fly/orbit controllers) or from any placed camera entity, switching
   at runtime. Placed cameras are stationary: they render from wherever their
   `Transform` says, with no controller input.
3. **Real-camera controls.** Lens parameters expressed the way a filmmaker
   would set them — focal length + sensor size (not just raw FOV), f-stop,
   focus distance — with derived effects: depth of field, barrel/pincushion
   distortion, chromatic aberration, vignette.
4. **Placement usability.** Placing a camera "up high, angled down" must be
   easy: fly the editor camera to the desired vantage point, press a key, and a
   camera entity is spawned with that exact pose. Refinement happens in an
   ImGui inspector.
5. **Two render paths, one parameterization.** The same lens parameters feed
   the offline path tracer (physically: thin-lens ray sampling) and the
   realtime Metal viewer (approximately: post-process passes). High-quality
   "virtual filming" stills come from handing a placed camera to the tracer.
6. **(Late) external pose driving.** A camera entity's pose can be written by
   an external source — ultimately an iPhone streaming its real-world ARKit
   pose — through a narrow, testable seam.

## Non-goals (for now)

- Simultaneous multi-viewport / split-screen rendering. One active view at a
  time; the architecture (camera = entity, view selection = system) leaves the
  door open.
- Camera animation / dolly paths. Stationary cameras first; a moving camera is
  "just" an entity whose Transform is animated, so this composes later.
- Physically-based exposure/film simulation (ISO, shutter speed, motion blur).
  Worth a follow-up once the lens model exists; bloom + the existing exposure
  control cover the near term.

---

## Architecture

### New components (`src/engine/components.h` or `src/engine/camera/`)

```cpp
// A placeable camera. Pose comes from the entity's Transform (forward = -Z of
// the orientation, matching the engine's right-handed convention, ADR-0009).
struct SceneCamera {
    LensParams lens;            // see below
    std::string name;           // for the camera list UI ("Cam A", "Crane 2")
    bool showGizmo = true;      // draw frustum/body gizmo when not active
};

// Physical lens parameterization. FOV is *derived*, not stored.
struct LensParams {
    Real focalLength   = 50.0;  // mm
    Real sensorHeight  = 24.0;  // mm (full-frame 36x24 default)
    Real fStop         = 8.0;   // aperture, f/N
    Real focusDistance = 10.0;  // meters, world units
    Real nearPlane = 0.1, farPlane = 1000.0;

    // Image-forming aberrations (0 = ideal lens)
    Real distortionK1 = 0.0, distortionK2 = 0.0;  // Brown radial terms
    Real chromaticAberration = 0.0;               // lateral CA strength
    Real vignette = 0.0;

    Real verticalFovDegrees() const;   // 2*atan(sensorH / (2*focal))
    Real apertureDiameter() const;     // focalLength / fStop, in meters
};
```

Notes:
- `LensParams` lives engine-side with pure-math derivations → unit-testable
  headlessly (`make test`), per ADR-0001/0009. No renderer types.
- `SceneCamera` + `Transform` is the whole truth. Aim = orient the Transform;
  a `lookAt(Vec3 target)` helper on the component or a free function makes the
  "point it at the courtyard" workflow one call.
- Existing `CameraState` grows the lens fields (or carries a `LensParams`
  member) so `Renderer::setCamera` is the single seam through which the
  backend learns about lens effects. `fovDegrees` stays for the editor camera;
  scene cameras fill it from `verticalFovDegrees()`.

### View ownership: extend `CameraSystem` into the view selector

`CameraSystem` already owns "what the view is" and publishes to
`ctx.view.camera`. Rather than adding a competing system, it gains a notion of
**view source**:

```cpp
enum class ViewSource { Editor, SceneCamera };
// CameraSystem members:
ViewSource source = ViewSource::Editor;
Entity activeCamera;   // valid when source == SceneCamera
```

- `update()`: if `Editor`, current behavior (fly/orbit controllers). If
  `SceneCamera`, build `CameraState` from the entity's `Transform` +
  `SceneCamera` (position = transform.position, target = position + forward,
  up = orientation's +Y) and publish. Controller input is ignored — the
  camera is stationary by design.
- Generation-checked handles mean a destroyed active camera is detected by
  `world.alive()`; fall back to `Editor` gracefully.
- Switching: a `view_next_camera` / `view_prev_camera` action cycles
  `Editor → cam1 → cam2 → … → Editor`; `view_editor_camera` (Esc-like) always
  returns home. Bindings registered alongside the existing camera bindings.
- Placement: a `place_camera` action spawns an entity at the *current editor
  view pose* (fly camera eye + yaw/pitch as a Quat). This solves the "put it
  high up and angle it down" problem with zero new UI: fly there, frame the
  shot in first person, press the key — the new camera sees exactly what you
  saw.

### Camera gizmos

Placed cameras should be visible in the world when not looked through: a small
camera-body mesh (via the existing `Renderable` path — a `MeshBuilder`-style
box+lens primitive is enough) and, later, frustum lines. The gizmo is attached
as a normal `Renderable` so it needs no renderer changes; a follow-up can move
it to a debug-line path. The gizmo for the *active* camera is skipped (you're
inside it).

### ImGui camera panel (`RT_ENABLE_IMGUI`, ADR-0011)

A `CameraPanelSystem` (or a section in `DebugOverlaySystem`) provides:
- List of all `SceneCamera` entities (`world.each<Transform, SceneCamera>`),
  with name, **Look through**, **Snap to my view**, **Delete**.
- Inspector for the selected camera: position/rotation fields, look-at target,
  and lens controls — focal length (with the familiar 24/35/50/85 stops),
  f-stop, focus distance, distortion, CA, vignette.
- "Place camera here" button (same code path as the keybind).

### Persistence (level JSON)

`LevelLoader` learns a `"camera"` entity type so placements survive sessions:

```json
{ "type": "camera", "name": "Crane",
  "position": [4, 12, -3], "orientation": {"axis": [1,0,0], "angleDeg": -35},
  "lens": { "focalLength": 35, "fStop": 2.8, "focusDistance": 8.0 } }
```

Save-back (writing edited cameras to the level file, or a sidecar
`*.cameras.json`) comes with the panel work — placing cameras interactively is
pointless if they evaporate on restart.

### Lens effects: two implementations, one parameter set

**Offline path tracer (`src/camera.h`)** — the physically honest path:
- Thin-lens model in `generateRay`: sample a disk of radius
  `apertureDiameter()/2`, focus on the plane at `focusDistance` → true depth
  of field. This is the classic extension; the tracer is already
  multi-sampled, so it costs nothing extra.
- Radial distortion: warp the (u,v) → direction mapping by the Brown model.
- Chromatic aberration: per-channel focal scale — trace R/G/B with slightly
  different ray mappings (3 rays per sample at the highest quality, or one ray
  with channel-weighted accumulation as the cheap mode).
- The tracer's `Camera` constructor gains a `LensParams` overload; the
  existing pinhole behavior is `fStop = ∞` (aperture 0), so current output is
  unchanged by default. Deterministic sampling (seeded, per ADR-0014's
  reproducibility spirit).

**Realtime viewer (Metal backend)** — the perceptual approximation:
- The backend already has a post-process stack (SSAO, SSR, bloom) and a depth
  buffer, so the hooks exist:
  1. **DOF pass:** circle-of-confusion from depth + `focusDistance` +
     `apertureDiameter`, gather blur (single-pass scatter-as-gather is fine at
     this scale).
  2. **Lens-warp pass (final):** one full-screen pass doing radial distortion
     + lateral chromatic aberration (three distorted UV lookups) + vignette.
     These three share a pass because they're all "resample the final image".
- Engine-side, everything arrives via `CameraState`; the Metal shader work and
  pass wiring stay sealed in the backend (ADR-0001). Each effect gets a
  Renderer toggle like the existing `bloomEnabled`, surfaced in the debug
  overlay. Needs macOS verification like the rest of the Metal work.

### External pose seam (the iPhone milestone)

Late-stage, but it shapes one early decision: pose *writes* to a camera entity
must be a plain `Transform` update, owned by a system — which the design above
already guarantees (CameraSystem reads the Transform fresh every frame).

When the time comes:
- A `PoseSource` seam (engine-side interface: `bool poll(Pose& out)`), a
  `CameraPuppetSystem` that applies a `PoseSource` to a tagged camera entity
  in `update()`, with smoothing/prediction.
- A first concrete source that listens on UDP for a trivial datagram
  (timestamp, position, quaternion). **Note:** sockets are outside the
  "standard library only" rule — this needs its own ADR (platform seam like
  ADR-0001/0013, POSIX + BSD sockets are available everywhere we build).
- The iPhone side is a small ARKit app streaming `ARFrame` camera transforms;
  coordinate-system mapping (ARKit gravity-aligned, meters → world) lives in
  the puppet system where it's testable with canned data.
- A mock `PoseSource` replaying a recorded path makes the whole chain testable
  headlessly long before any phone app exists — and doubles as a camera-path
  animation preview.

---

## Phases

### Phase 1 — Camera entities + viewport switching (the core)
- `SceneCamera` + `LensParams` components (FOV derivation only; aberration
  fields present but unused).
- `CameraSystem` view-source selection, cycling actions, fallback on dead
  entity, `place_camera` spawning at the editor pose.
- Camera-body gizmo `Renderable`, hidden for the active camera.
- Tests: FOV/aperture derivations, view-state-from-Transform, switch/cycle
  logic, dead-camera fallback, place-at-pose correctness. All headless.

**Exit criteria:** fly somewhere, place a camera, keep flying, switch to it
and see the frozen framing; place several and cycle through them.

### Phase 2 — Editing UX + persistence
- ImGui camera panel: list, look-through, snap-to-view, delete, name; lens +
  transform inspector; look-at helper.
- `LevelLoader` `"camera"` entities; save-back of edited cameras.
- Tests: loader round-trip for camera JSON.

### Phase 3 — Physical lens in the offline tracer
- Thin-lens DOF, radial distortion, per-channel CA in `Camera::generateRay`;
  pinhole remains the default.
- A path (CLI flag or small tool) to render the offline tracer *from a placed
  camera's parameters* — first real "virtual filming" output.
- Tests: ray statistics (aperture=0 ⇒ identical to pinhole; focus-plane points
  converge; distortion displaces corners as predicted).

### Phase 4 — Realtime lens effects in the viewer
- DOF pass (depth-based CoC) and final lens-warp pass (distortion + CA +
  vignette) in the Metal backend, driven by the active camera's `LensParams`
  through `CameraState`.
- Debug-overlay toggles + parameter sliders; macOS verification pass.

### Phase 5 — Filming aids (polish, optional ordering)
- Framing overlays: rule-of-thirds grid, aspect-ratio letterboxing (2.39:1
  etc.), safe areas — pure ImGui drawing, cheap.
- Focal-length presets, focus-pull helper (click an object → focusDistance).

### Phase 6 — External pose driving (iPhone)
- `PoseSource` seam + `CameraPuppetSystem` + mock/replay source (headless
  tests, and immediately useful for previz).
- UDP listener source (with the sockets ADR), then the ARKit companion app.

Phases 3 and 4 are independent of each other (both depend on Phase 1; Phase 2
is UX and can interleave). Phase 6 only needs Phase 1.

---

## Risks / open questions

- **Realtime DOF quality.** A single-pass gather DOF can look mushy at wide
  apertures; acceptable for previz, and the offline tracer is the ground
  truth. Revisit (scatter bokeh, near/far split) only if filming use demands.
- **`CameraState` growth.** It's the renderer seam; adding `LensParams` keeps
  one struct flowing through `setCamera` rather than a second side channel.
  If it keeps accreting, split view (pose/projection) from lens (post) — flag
  at Phase 4.
- **Who owns "active camera" persistence?** Settings already persists camera
  pose/mode; the active view source should persist the same way (entity
  *names*, not handles, since handles don't survive a reload).
- **Gizmo rendering path.** Using `Renderable` means gizmos appear in
  reflections/shadows. Fine initially; a debug-draw layer is the clean fix
  and is independently useful.
