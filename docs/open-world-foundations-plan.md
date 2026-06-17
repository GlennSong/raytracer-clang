# Open-world foundations — coordinate precision, rendering & streaming

**Status:** Design plan (plan-first; no code yet). Target picked: **infinite
procedural terrain** (Minecraft-like — walk forever, deterministic chunks). Pairs
with ADR-0027 / `world-system-plan.md` (the *content* model: fields + recipes +
deterministic tiles). This doc owns the part those don't: the *coordinate system,
floating-point precision, and rendering* that an open world demands. New decisions
land as ADR-0034.

---

## 0. Why now — the symptoms that forced this

Recent work on distant terrain surfaced a cluster of failures that are all the
same underlying problem — *the engine assumes a small world centred on the
origin*:

- **Terrain composited as sky.** The composite/SSR/SSAO classify a pixel as
  background with a fixed `depth >= 0.999`. With `near=0.1` that NDC value maps to
  only ~99 m, so all terrain past ~99 m is painted over with the skybox (the
  "circular cut" — the 99 m ground disc). Hyperbolic depth + tiny near plane.
- **Frustum culling misjudges terrain.** The LOD terrain is one giant mesh (plus
  concentric rings) centred at the origin, so its bounding sphere is a poor proxy;
  the point-sphere test rejects/keeps it wrongly as the camera moves.
- **LOD ring seams crack** (already logged in TECH_DEBT) — concentric rings are
  not a real streaming scheme.
- **Latent: vertex/camera jitter** once the player is more than a few km out
  (float32 precision), and **z-fighting** in the far field.

None of these are independent bugs; they are the small-world assumption leaking.
Fixing them piecemeal is the "huge mess" we want to avoid.

## 1. The root cause: float precision degrades with distance from the origin

A 32-bit float carries ~7 significant digits, so *absolute* precision scales with
magnitude:

| Distance from origin | float32 resolution |
|---|---|
| 1 km | ~0.06 mm |
| 100 km | ~8 mm |
| 10,000 km | ~1 m |
| 12.5M blocks (Minecraft "Far Lands") | meters — visibly broken |

Every open-world technique below exists to keep the numbers the **GPU and physics**
actually consume *small* (near zero), regardless of where the player "is" in the
world. Our CPU side already helps us here: `Real = double`, and `Vec3`/`Mat4` are
double; we only narrow to float at GPU upload (`toSimd`). Doubles are sub-mm
precise far beyond Minecraft scale, so the *simulation* frame can stay exact — the
danger is purely (a) what we hand the GPU as float, and (b) Jolt, which is float
internally.

## 2. The coordinate model (decision)

For *infinite, planet-free* terrain we do **not** need a bespoke 64-bit integer
"universe" layer (that's the space-sim / Star Citizen tier). The robust,
proportionate model:

1. **World simulation positions stay `double`** (already true). Authoritative,
   exact at Minecraft-and-beyond scale.
2. **Chunks are addressed by integer tile coordinates** (`i64` x,z), so the map is
   unbounded and every chunk regenerates identically from `(tileCoord, worldSeed)`
   (ADR-0002 / ADR-0027 §5).
3. **Render camera-relative** (§3): the GPU only ever sees positions relative to
   the camera, so float narrowing happens on *small* numbers.
4. **Floating origin rebasing** (§4) keeps the active simulation — including Jolt
   — near zero so float-internal subsystems never drift.

This is the Minecraft/KSP lineage (doubles + floating origin + camera-relative),
not the No Man's Sky lineage (doubles/ints + cube-sphere planets + scattering). If
we ever go planetary, §7 notes the upgrade path.

## 3. Camera-relative rendering (Phase 1)

**Idea.** Before building the view-projection and uploading transforms, translate
the world by `-cameraPosition`, done in `double`, *then* narrow to float. The
camera sits at `(0,0,0)` for the GPU; nearby geometry has small coordinates, so
vertex precision is excellent no matter the absolute world position.

**Where it lands (we have the seam).** `CameraState` → `Renderer::setCamera` /
`drawMesh(transform, …)`. Concretely:
- View matrix is built with the camera at the origin (rotation only; translation
  folded out).
- Each draw's model matrix has `cameraPosition` subtracted from its translation
  (in double) before `toSimd`.
- Instance transforms (the forest / scatter path) get the same offset.
- Skybox is unaffected (direction-only).

**Cost.** One double subtraction per object/instance at submit; no new buffers.
Contained to the render submit path + the matrix helpers. **Metal-unverifiable on
Linux** — verified offline (the tracer already works in world space) + by the user
in the viewer.

## 4. Floating origin / world rebasing (Phase 3)

**Idea.** When the player crosses a threshold from the current *render origin*
(e.g. 4 km), shift **everything** — entity `Transform`s, the streaming anchor, and
the Jolt body positions — by the player's offset, and reset the player near zero.
The player keeps walking "forward forever"; in absolute float terms they never
leave the high-precision zone.

**Why it's still needed if §3 exists.** Camera-relative rendering fixes *rendering*
precision, but Jolt simulates in float in *world* space. Without rebasing, physics
jitters far out. Rebasing keeps the simulated set near zero. (Double world coords
mean the *authoritative* position is always exact; rebasing is about the float
subsystems and bounded accumulation error.)

**Care points.**
- Rebase on a fixed-update boundary, atomically across ECS + physics + streaming.
- Velocities/relative state are invariant under a pure translation — only
  positions shift.
- Determinism: chunk generation keys off integer tile coords, not the shifting
  render origin, so rebasing never changes generated content.

## 5. Depth & sky — reverse-Z + robust background classification (Phase 0)

The immediate render break and the latent z-fighting are one fix:

- **Reverse-Z.** Map near→1, far→0 with a float depth buffer (`Depth32Float`,
  which we already use). Float concentrates precision near 0; reverse-Z puts the
  *far* field there, giving near-uniform precision from 0.1 m to 100 km. Lets us
  open the far plane wide without the precision collapse that caused the 99 m cut.
  - Touches: `Mat4::perspective` (reverse mapping), depth clear (0 not 1), depth
    compare (`Greater`/`GreaterEqual`), skybox z, and the depth-reconstruction
    math in SSR/SSAO.
- **Robust sky test.** Replace every `depth >= 0.999` with a test against the
  *cleared* far value, not a magic NDC constant. Under reverse-Z, background =
  `depth <= 0` (the clear), which is exact and scene-independent. (Until reverse-Z
  lands, the stop-gap is a *linearized* test — `linearDepth >= 0.999 * far` —
  which is correct for any near/far ratio.)
- **Re-enable frustum culling** (currently bypassed by a temporary diagnostic),
  once chunk bounds are tight enough to test correctly (§6).

This is the smallest change that makes `range.json` render, and it's worth doing
regardless of how far up the ladder we go.

## 6. Terrain: chunked + streamed, tight bounds (Phase 2)

Replace the single origin-centred tile + concentric LOD rings with a **grid/
quadtree of terrain chunks around the camera**, each:
- generated deterministically from `(tileCoord, worldSeed)` (ADR-0027 §5), cached,
  and freed when far;
- a *small* mesh with its **own tight bounding box** (so frustum culling is
  correct — kills the origin-sphere misjudgement);
- carrying its own LOD; neighbours differ by at most one level, stitched with
  skirts/edge-stitching (retires the "rings crack at seams" debt).

Candidate schemes (decide in the ADR when we build): **geometry clipmaps**
(camera-centred concentric grids, GPU-friendly) or **CDLOD / chunked quadtree**
(per-chunk LOD with morphing). Both are camera-relative by nature (§3) and tile
deterministically (ADR-0027). The streaming manager is the one ADR-0027 §7
anticipated; this doc adds that it must produce **camera-relative, tightly-bounded**
chunks and cooperate with rebasing (§4).

## 7. Skybox & "turn it off"

Keep the skybox — it *is* the sky/atmosphere; removing it just yields a flat clear
colour. The "let it render as it is" instinct is really three needs the skybox was
masking, each addressed above: **robust sky/geometry classification** (§5),
**terrain that reaches the horizon** (§6), and **atmospheric blending** of distant
terrain into the sky (the aerial-perspective fog already added). *Planetary upgrade
path (out of scope now):* curved horizon + real atmospheric scattering + a
cube-sphere terrain mapping; this is the No Man's Sky tier and would extend, not
replace, §2–§6.

## 8. Phasing

| Phase | Scope | Unblocks | Risk |
|---|---|---|---|
| **0** | Reverse-Z + robust sky test; re-enable culling; revert diagnostics | The current broken render; far-field z-fighting | Small, Metal-only (user-verified) |
| **1** | Camera-relative rendering | Vertex/camera jitter; precision to tens of km | Medium, render submit seam |
| **2** | Chunked/streamed deterministic terrain w/ tight bounds + LOD stitching | Correct culling; endless terrain; retires ring/seam debt | Large; pairs with ADR-0027 streaming |
| **3** | Floating-origin rebasing (ECS + Jolt + streaming) | Truly infinite navigation; bounded float error | Large; cross-subsystem atomicity |

Phases 0 and 1 are foundation for any scale and are independently shippable. Phase
2 is the architectural centrepiece (and the natural place to start building the
ADR-0027 streaming manager for real). Phase 3 is what makes "infinite" literal.

## 9. Open questions (resolve as phases start)

- **Clipmaps vs CDLOD** for the chunk LOD (Phase 2 ADR).
- **Rebase threshold & cadence** — distance trigger vs time; how to hide the shift
  (it must be imperceptible).
- **Jolt origin-shift API** — does shifting all body transforms in one step play
  well with the broadphase, or do we recreate/translate the world? (Phase 3 spike.)
- **Offline tracer parity** — the tracer renders in absolute world space and has no
  far clip, so it's the precision *oracle*; camera-relative/reverse-Z are
  realtime-only and validated against it.
- **Reverse-Z blast radius** — every depth consumer (SSR, SSAO, temporal AO, debug
  views) must flip together; enumerate before touching (Phase 0).

## 10. Decisions to record

- **ADR-0034** (this plan): *Open worlds are double world coords + camera-relative
  rendering + floating-origin rebasing + reverse-Z; terrain streams as
  deterministic, tightly-bounded chunks.* Status **Pending** until Phase 0 lands.
- A later ADR fixes the Phase 2 LOD scheme once clipmaps-vs-CDLOD is chosen.
