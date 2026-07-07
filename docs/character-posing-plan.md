# Character Posing & Comics Pipeline — Plan

The goal, in one sentence: bring a rigged 3D character into the engine, pose
it per panel on a shot timeline, and batch-render high-res stills for comics —
using the machinery the engine already has wherever it exists.

Shaped by an interview with the owner (2026-07-07):

- **Look:** undecided / per-project mix → the pipeline stays *style-agnostic*.
  A panel renders through the offline path tracer (cinematic, lens effects) or
  the realtime viewer (the current stylized look) from the same posed scene;
  a toon/NPR pass is a later, optional renderer feature, never a data-model
  assumption.
- **Characters:** all three sources, in this order — simple mannequins first
  (prove the workflow), Blender/bought rigs via glTF (real content), and
  eventually procedurally generated characters (the engine's ethos; the
  citysim walker is the seed). Consequence: ONE engine skeleton/skin
  representation that every source fills — import is a producer, procgen is a
  producer, the posing/timeline/render stack consumes.
- **Timeline:** mostly static poses. Comics are stills: the timeline organizes
  *panels* (pose + camera + lens per shot), not interpolation. But the data
  model is keyframes from day one — a pose is a one-key clip — so real
  animation playback grows out of it instead of being bolted on.
- **Output:** high-res stills per panel (PNG). Page layout/lettering stays in
  external tools; no in-engine 2D page composer.

## What already exists (don't rebuild it)

| Need | Existing tech |
|---|---|
| Joint hierarchy math | `Transform`, `worldMatrix` parent chains, slerp (ADR-0006) |
| Keyframe curves | `curve.h` (Catmull-Rom, arc length), `EditableCurve`, `AnimationPath`, ADR-0031 `AnimCurve` plan |
| Gizmo manipulation | ImGuizmo (vendored, drives editor transforms today) |
| Timeline / curve UI | **ImSequencer + ImCurveEdit ship in the vendored ImGuizmo repo** |
| Undo, inspector, save/load | `UndoStack`, property registry, LevelWriter round-trip patterns |
| A posable biped, today | `citysim::buildPersonMesh(swing, outfit)` — limbs already pitch about hip/shoulder pivots |
| glTF parsing (incl. skins/animations) | tinygltf (vendored; engine consumes meshes only so far) |
| The panel renderer | the offline path tracer + `LensParams` (DOF, vignette) — stills are its home turf |
| High-level sequencing | `docs/virtual-camera-plan.md` shot/camera work |

## Phases

### P0 — Skeleton substrate + the mannequin (headless)
`engine::Skeleton`: joints (name, parent index, local TRS, inverse bind
matrix), `Pose` = per-joint local TRS overrides, `worldJointMatrices(skeleton,
pose)`. A procedural **mannequin builder** produces a Skeleton + one rigid
mesh part per bone (MeshBuilder capsules/boxes; proportions can borrow the
citysim person so it reads human). Rigid per-bone attachment — no skin weights
yet — every part follows its joint. Pure data, unit-tested (`make test`
tier). *Proves posing end to end with zero import work.*

### P1 — Pose editing in the editor
Joint picking + ImGuizmo rotate/translate on bones, undo-integrated. `Pose`
assets save/load as JSON (LevelWriter pattern) → a pose library panel. A
`SkeletonComponent` + system stamps posed joint matrices into the render
transforms each frame; DebugDraw (`ctx.debug`) overlays the bone lines —
that's what it was built for.

### P2 — Skinned characters (glTF import + CPU skinning)
Extend `ModelImporter` to read glTF skins (joints, weights, inverse binds) and
fill the same `Skeleton`. **CPU skinning first**: posed vertices go through
the ordinary upload path, so characters render in the viewer *and* the offline
path tracer with one implementation and zero per-backend shader work. GPU
skinning is a later perf optimization behind the same component (revisit
trigger: crowd-scale characters or profiler evidence — ADR-0068 exists for a
reason). Blender / bought / Mixamo-converted rigs become usable here.

### P3 — Panels and the still renderer
A `Panel` = camera pose + `LensParams` + pose assignments (+ level). A shot
list document (JSON) and a batch renderer: every panel → path-traced PNG at
print resolution (the `raytracer` binary already takes a scene; this is a
driver around it), with a viewer-capture path for the realtime look. A
contact-sheet output for review. This is the smallest phase — and the payoff.

### P4 — Timeline growth (animation proper)
glTF animation-channel import → clips over the `AnimCurve` substrate; a
sampler/player system (poses interpolate: lerp + slerp per joint); the
ImSequencer-based timeline panel with scrubbing; two-bone IK helpers (grab a
wrist/ankle, the limb solves) — pull IK earlier if FK-only posing proves
tedious in P1. In-engine playback also unlocks cutscenes/animatics later.

## Build vs. buy

In-house through P4: skeleton + sampling sits squarely on the existing curve/
math kernel, and CPU skinning keeps offline-tracer parity trivial. The one
credible library is **ozz-animation** (MIT, the "Jolt of animation").
**ADR trigger to vendor it:** animation *blending* trees, retargeting between
rigs, or production-grade IK — not before. Character *content* is authored or
bought (Blender/asset stores), never an in-engine authoring tool; procedural
character *generation* is its own future research track (the walker + the
tree-skeleton work both feed it).

## Non-goals (for now)
In-engine page layout/lettering; NPR/toon rendering (style decision deferred);
GPU skinning; muscle/cloth/physics-driven secondary motion; facial rigs/blend
shapes (worth a look at P2 import time — glTF morph targets — but not load-
bearing for mannequin comics).
