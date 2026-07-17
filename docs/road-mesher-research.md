<!-- Research output, 2026-07-17. Commissioned after the user drove metropolis
and reported: "you have a lot of degenerate surfaces on the road. I can see a ton
of triangles emitting from a single point. Those don't look like ribbons at all...
We would need a mesher to build all quads and maybe even be able to build quads at
different resolutions so that things like roads and sidewalks could be decimated
easier. I think we need to research a better meshing system because without that we
can't build anything with detail."

MEASURED on a 4x4 grid city on hills (tests/test_mesh_quality.cpp):
  tris=17874 verts=51958  -> 2.91 verts/tri  (unshared soup = 3.00; shared grid ~0.5)
  distinct positions=6947 -> 7.5x duplication; nothing is connected to anything
  worst apex fan = 457 triangles sharing ONE point
  slivers (aspect>20) = 12306 = 68.8%; worst aspect 297339
  degenerate (zero-area) triangles present  <- and checkWeld asserts ==0, passing on FLAT fixtures only

This is a RESEARCH DOCUMENT, not an approved plan. No code has been written against it. -->

# Design Recommendation: a Swept-Lattice Road Mesher

## 0. The diagnosis in one paragraph

The user's framing is right but understates the case. The engine is not *missing* a road parameterisation — it **already has one and throws it away twice**:

1. `UnionSpine` (`road_mesh.h:229-253`) is literally a swept-surface spec: `points` (centreline), `yAbs` (per-station absolute deck Y), `hw` (per-station half-width), `crossSlope` (per-station superelevation), `klass`. That is centreline × station-varying cross-section. `weldSolid` takes that exact 3-D authoring, **flattens it to 2-D**, boolean-unions the outlines, ear-clips the union, and then *reconstructs* the height per vertex with a nearest-spine winner-take-all field (`heightOf`). The measured 164%-grade faces on the 45° diagonals of a 4-way pad are not a bug in `heightOf` — they are the signature of reconstructing information the input already carried exactly. Sweeping never needs to reconstruct.

2. `surfRoadMarkings` (`shaders/metal/common.metal:192-239`) is **already written against (s, t)**: `mu` is the lateral coordinate (0..1 sidewalk, 1..3 carriageway, 2 = centreline, >3.5 = divider strip), `mv` is arc length / metres past the junction mouth. The shader is a swept-surface shader. The deck sheet is emitted with `emitTri` (`road_mesh.cpp:1755-1757`), which hardcodes `Vertex(p, normal, tan, 0, 0)` → `mu = 0` → the `mu < 0.98` branch → **sidewalk concrete at world-planar UV across the entire carriageway.** The shader is not wrong; the geometry never told it where it was.

So the recommendation is not "adopt quads because quads are nice". It is: **stop discarding the parameterisation the authoring and the shader both already speak.** Everything else — the 68.8% slivers, the 457-fan, the degenerate triangles, the missing LOD structure — falls out of that one decision, because a lattice's aspect ratio and valence are set *by construction* rather than by whatever ear-clipping happened to carve.

---

## 1. Mesh representation

### 1.1 What NOT to build

- **Not a half-edge / winged-edge structure.** Junctions are non-manifold by construction (pad + ribbon + sidewalk share an edge); half-edge cannot represent that, and it has no per-corner attribute story anyway. It would be work that ends in re-splitting exactly the seams it was built to close.
- **Not a general connectivity mesh.** `RenderMesh` is the engine-wide interchange type at ~20 `acquireMesh` sites plus Metal upload (`metal_renderer.mm:972-1016`), physics (`physics_world.cpp:345-368`), and the path tracer (`level_scene.cpp:151-175`). It cannot move, and it should not.
- **Not a decimator.** Nothing in this design ever simplifies a mesh. Coarser levels are *subsets of index buffers over one vertex buffer*.

### 1.2 What to build

For a swept surface, **connectivity is index arithmetic — you do not need to store it.** A patch of `(R+1) × (P+1)` vertices has vertex `(i, j)` at `i*(P+1)+j` and quad `(i,j)` = `{(i,j), (i,j+1), (i+1,j+1), (i+1,j)}`. That is the entire adjacency structure. This is exactly how `terrain_lod.cpp:177-187` and `MeshBuilder::gridIndices` already work, and how `sdf.cpp:146-205` works. **The precedent for a shared-vertex indexed lattice is proven end-to-end in this engine through upload, shadows and colliders.** What does *not* transfer from CDLOD is its machinery (regular even grid + analytic height field); what transfers is the indexing.

The new emitter:

```cpp
// mesh_builder.h — the one primitive the engine is missing.
// Emits (R+1)*(P+1) SHARED vertices and 2*R*P triangles. Row i = one station
// ring; column j = one profile point. Connectivity is the index arithmetic.
// Winding is fixed ONCE for the whole lattice (front = clockwise, engine
// convention) — NOT per-face. This is the point sdf.cpp:150-159 warns about.
struct LatticeSpec {
    int rings, profilePts;
    const Vertex* verts;   // row-major, caller fills position/normal/tangent/uv/color
};
void MeshBuilder::emitLattice(RenderMesh&, const LatticeSpec&);
```

### 1.3 How flat shading survives — the key structural point

**You only need sharing along one axis. Split freely along the other.**

- **Longitudinal (s, ring axis): fully shared.** This is where every LOD gain lives (drop every other ring). Consequence: consecutive quads along the road no longer carry independent face normals — the road shades *smooth along its length*. For a carriageway that is a visual **improvement**, and it is the deliberate choice `vehicle_mesh.cpp:129,169` already makes ("shared loft verts → SMOOTH shading"). Flag: it is still a visual change to every road in the world.
- **Lateral (t, profile axis): duplicate columns at every crease and every attribute seam.** The profile array simply lists the same XZ twice with different normal/uv/color. Curb top gets column `j` (road-side, normal = curb-face `nIn`) and column `j+1` (slab-side, normal = `nUp`) at the same position. The lattice stays perfectly regular; you just have a zero-width band.

This resolves the whole "share position vs. split attributes" tension **without inventing a corner/loop array**. Within a band the normal is constant across `t` by construction (it *is* a swept profile), so **flat shading of curb faces, parapets and fascia is preserved for free** — no duplication needed, because both bounding columns already carry the same face normal. The only place attributes must split is at band boundaries, and there they split by an extra column, which costs `R+1` vertices per seam and nothing structurally.

Net effect on the measured numbers: `V/T` goes from **2.70** to roughly `(R+1)(P+1) / 2RP ≈ 0.55` for a typical `P` of 10-16 — i.e. past the raw dedup bound of 4.23×, because the lattice never creates the duplicates in the first place rather than removing them afterwards.

### 1.4 Attributes

| Channel | Content | Note |
|---|---|---|
| `position` | swept from `points`/`yAbs`/`hw`/`crossSlope` | exact, never reconstructed |
| `normal` | per-band face normal (flat) or ring-averaged miter (bends) | see 1.3 |
| `tangent` | **free on roads** (verified: `surfRoadMarkings` never reads it) → reserve for the CDLOD morph target | see §4 |
| `u` = `mu` | lateral: profile column → the shader's existing 0..3 / >3.5 encoding | **no shader change needed** |
| `v` = `mv` | arc length, or metres past the mouth (crosswalk gate) | **no shader change needed** |
| `color` | per-band (asphalt / concrete / fascia / soffit) | preserved by column split |

**No `Vertex` change is required.** `Vertex` (`renderer.h:23-38`) carries exactly one `u,v` pair and `tangent` is already hijacked by CDLOD — so the prior-art recommendation of CityEngine-style *three* UV sets **cannot be adopted without adding 8 bytes to every vertex in the engine.** Do not do it. `mu`/`mv` already encode `(t, s)`; the world-planar `wu/wv` the shader tiles by are derived in the fragment shader, not stored. This is a real reuse win and it should be protected.

---

## 2. The body: quad strip, arc-length × lateral

### 2.1 Rings (the `s` axis)

Replace `stripSegs` (`road_mesh.cpp:334-348`, per-span, `ceil(maxSag/conformTol)`, arbitrary integer) and the `deckSag`/`refineDeck` recursion (`road_mesh.cpp:1767-1808`) with **one chain-global arc-length sampler**:

- Walk the whole chain by arc length. Place rings densely where curvature or terrain sag demands (keep `conformTol`/`conformStep` as the *placement heuristic* — the logic is sound, only its scope is wrong).
- **Ring index must be chain-global**, not per-span, or `i % 2^L` is meaningless across span boundaries.
- Round `R` up to a multiple of `2^(numLods-1)`.
- **Run the sampler exactly once, at LOD0.** Every coarser level is a subset. Re-running it per level yields unrelated vertex sets and guaranteed cracks.

Non-uniform spacing is fine; nesting only requires that the coarse set be a *subset*.

`refineDeck` and `deckSag` **delete entirely.** They exist only to make an ear-clipped sheet follow a profile it was never told about. A lattice conforms because rings are placed by arc length. That is a net −60 LOC, not an addition.

### 2.2 Profile (the `t` axis)

One `RoadProfile` per `RoadClass`, from the kit. Example, a 2-lane local street:

```
j:  0      1        2      3          4   ...  5          6      7        8
    toe    outer    slab   curb-top   curb-top   edge-line  lane... centre ... (mirror)
    ground face     top    (slab side)(road side) 
mu: -L     -L'      -L''   -0         1.0        1.02       ...     2.0     ...
```

Duplicated columns at `j=3/4` (the curb crease: `nUp` vs `nIn`, concrete vs asphalt, `mu` jumping 0 → 1). This directly replaces the three disconnected quads at `road_mesh.cpp:381-383` with three *bands of one lattice*.

**What falls out for free:**
- **Lanes**: interior columns at lane boundaries. The shader already computes `lat = mu - 2.0` and paints edge lines at `|lat| = 0.86` — that hardcodes 1 lane each way. With per-column lane indices you can eventually feed lane-local `mu` and get multilane dividers from the parameterisation instead of the `mu > 3.5` overlay-strip hack. **Defer** — it is a 4-shader change (Metal + WGSL + Vulkan + `scene.cpp`). Keep the overlay strip for now.
- **Kerbs**: profile columns with a per-station height. A dropped kerb at a crossing = set `curbHeight = 0` over a station range. Data, not geometry surgery.
- **Barriers / parapets / median**: profile columns with a per-station height. **This is the fix for `blocked=11`** — the parapet across every ramp gore. Today it is trimmed geometrically ("trimmed to stop at junction mouths", `WeldSolidParams::barriers`); as a profile column it is a per-station scalar that goes to zero across the gore. The probe's `rideLo=0.35 / rideHi=1.80` bumper band then finds nothing.
- **Viaduct fascia + soffit**: the profile *wraps* — it continues past the deck edge down the fascia and back under the soffit. Then `p.viaductSideColor` / `p.viaductBottomColor` are per-band colours, not a per-triangle `authoredAt(centroid)` test (`road_mesh.cpp:1762-1764`). The `authoredAt` centroid test disappears, because a swept spine *knows* whether it is authored — it has `yAbs`.
- **Superelevation**: `crossSlope` is already per-point in `UnionSpine`. In a lattice it is `y += lateralOffset * crossSlope[i]` when placing column `j` of ring `i`. Native.

### 2.3 UV

Emit `mu` from the column's profile spec, `mv` from the ring's arc length. **That is the entire fix for the "carriageway painted as sidewalk concrete" defect** — no shader change, no new channel.

---

## 3. The junction — the crux, not hand-waved

### 3.1 The constraint is a theorem, and it is narrow

Any quad mesh of a simply-connected polygonal domain has an **even number of boundary edges**. (Bishop, Lemma 4.1: `4Q = 2E − B` ⟹ `B = 2(E − 2Q)`.) This is necessary for *every* algorithm — templates, paving, Blossom-Quad, all of them. It is also **sufficient** (Bern & Eppstein: any simple polygon with even boundary has a linear-size all-quad mesh, all angles ≤ 120°).

So the junction's ledger is:

```
B = Σ K_i   (arm mouth divisions — PINNED, a contract with the body)
  + Σ C_i   (kerb fillet segments — FREE)
```

and you need `B` even. **The kerb arcs are your slack variable.** Nobody can see whether a fillet has 3 or 4 segments; bumping one by 1 fixes parity, always. This is exactly Mitchell's sum-even constraint `ΣAx − 2y = 0` from Sandia's interval-assignment work, and you get the *easy* version because your `K_i` are given rather than solved for. **One parity bit per junction. Do not import an integer program.**

**Never fix parity on an arm.** That is the seam you are protecting.

### 3.2 Why not the alternatives

- **Midpoint/rosette and Catmull-Clark do not match the arms — they *double* the boundary.** A `K`-span arm mouth comes out as `2K` spans. They need no parity check precisely *because* `2n` is always even: they "solve" the constraint by refusing to honour the prescribed boundary, which is the one property you cannot give up. A fine rosette also plants a valence-`B` pole dead centre of the intersection (4 arms, `K=4`, 4 kerb arcs of 3 → `B=28` → **valence-28 pole exactly where the player looks**). This is strictly worse than today's fan.
- **Paving** is nonrobust by its own community's assessment and is absurd machinery for a ~30-element patch you build thousands of.
- **Blossom-Quad** *does* honour the arms exactly, and its parity gate provably collapses to the same even-`B` condition. Keep it as the explicit "I give up" path — but do not lead with it: it produces unstructured valence that fights the lane-aligned `mu` bands, and you lose the flow that makes a road read as a road.

### 3.3 The recommendation: per-degree mapped templates

| N | Template | Singularities |
|---|---|---|
| 2 | quad strip (bend / continuation / taper / merge) | **none** |
| 4, equal K on opposite arms | pure `K × K` grid | **none** (`4Q=2E−B` checks: `Q=K²`, `E=2K(K+1)`, `B=4K` ✓) |
| 3 (T) | Cubit tri-primitive: 3 mapped blocks around one valence-3 centre, dims `i=(a+b−c)/2, j=(b+c−a)/2, k=(c+a−b)/2`, requires `a+b ≥ c+2` | one valence-3 |
| ≥5, or very unequal K | polyhedron cage: map the **2N-gon cage** (arm-mouth / kerb alternating) to N blocks around one valence-N centre | one valence-N |
| pathological | Blossom-Quad fallback | unstructured |

Apply the centroid trick **only to the coarse 2N-gon cage**, never to the fine boundary. That is one valence-2N centre instead of a valence-28 pole — and for the dominant city case (4-way, equal K) it is **zero** extraordinary vertices.

You get the medial-axis benefit (Tam & Armstrong) for free: **your input is a `RoadGraph`, not an opaque polygon. You do not need to compute a spine — the graph is the spine.**

### 3.4 Heights in the junction — this is what kills the 164% grade

This is the half the meshing literature does not cover and RoadRunner's docs call the hard part ("a space where multiple surfaces compete for influence over the junction's final surface representation").

- Each arm's **mouth ring is fixed** — it is literally the body's ring 0 (or ring R). Exact, by construction.
- Pick one junction centre elevation `Yc` = a weighted average of the arm-mouth centre Ys (weight by class/width, so an arterial dominates a local).
- Each mapped block's interior heights are a **Coons / bilinear blend** between its two bounding arm-mouth edges, the kerb arc, and `Yc`.

The blend is `C0` across block boundaries by construction (shared vertices, shared edge) and **exact at every arm mouth**. The medial-axis step *disappears because the patch interpolates instead of choosing a winner.* Nearest-spine (`heightOf`) is winner-take-all; a Coons patch is not. That is the causal fix for the measured 0.12–0.28 m drops on the 45° diagonals.

### 3.5 Geometric quality ≠ topological quality — budget for it

All-quad topology does not make a well-shaped junction. Roads meeting at 30° produce badly skewed blocks no matter the connectivity, and Bern & Eppstein's bound bites on slip lanes and acute forks (a rectangle of aspect `A` needs `Ω(A)` quads even with `n=4` — information-theoretic, not an implementation failure). **You will need a Laplacian / angle-based smoothing pass on the junction interior with arm-mouth vertices pinned.** Cubit's tri-primitive smooths by default for exactly this reason. Do not skip it and do not pretend it is free.

### 3.6 Ordering constraint (this may be the bigger refactor)

**Interval assignment must run before any geometry is emitted.** An interval error propagates globally (a division entering a quad exits the opposite edge and continues to the boundary) and **cannot be patched afterwards.** If the weld currently computes kerb tessellation downstream of arm division, that ordering has to change. Flag: I did not trace where `cornerRadius` filleting sits relative to arm capping in `weldSolid`, so I cannot size this.

---

## 4. LOD — and the property that makes it easy

### 4.1 The structure

One vertex buffer at LOD0 per chain. **N index buffers**, stride `2^L` over rings. That is it. Coarsening = "keep rings where `i % 2^L == 0`" — the same even/odd rule `terrain_lod.cpp:144-173` already uses.

### 4.2 Seams — the junction patch is the LOD firewall

This is the strongest property of the design and it should drive the argument:

- Ring `0` and ring `R` are in **every** subset (`R` is a multiple of `2^(numLods-1)`). So a chain's **mouth rings are pinned at every LOD.**
- A chain's mouths are the junction patch's arm boundaries.
- Therefore **every chain can sit at an independent LOD level with exactly zero seam**, and the junction patch never changes.

No stitching. No skirts (which would be visible on a 0.25 m-lift ribbon anyway). **Morphing is optional** — it only buys you popping suppression on interior rings, and it can come in a later stage. When you want it: copy `terrain_lod.cpp:144-173` (odd ring `i` targets the average of `i−1` and `i+1`, same column `j`), bake into `Vertex::tangent` (**verified free on roads** — `surfRoadMarkings` reads only `mu/mv/wu/wv`, never tangent), and draw via the existing `drawTerrain(handle, material, morphStart, morphEnd)` which already has a matching morphing shadow-caster (`metal_renderer.mm:777, 2317-2331`).

**Two things I could not verify and you must check before relying on morph:**
1. Whether the `drawTerrain` pipeline can dispatch surface id 11 (`RoadMarkings`) — surface dispatch is in the fragment shader (`common.metal:308`), so it probably can, but the terrain vertex shader may not forward `u/v`.
2. `terrain_lod.h` says "only the height differs on a regular grid" — the shader at `lighting.metal:580-584` may lerp only Y. **A road curves in XZ**, so an odd ring on a bend must slide toward the coarse chord laterally too. You need a full-`Vec3` morph. Check `mix(pos, morphTarget, k)` actually does that before assuming `drawTerrain` is a drop-in.

### 4.3 Recommend LONGITUDINAL LOD ONLY, at first

Do not decimate the lateral profile in stage one. If arms drop `K`, the junction's interval assignment has to re-solve at every level (and `K_i` would need to be divisible by `2^(numLods-1)`, which doubles lateral quads on every ribbon in the world). Rings alone capture most of the win because the body dominates road area. Lateral collapse (curb → flat strip) is also the riskier half visually: the morph interpolates position but the curb's **normals flip from vertical to horizontal**, which reads as popping despite the morph.

**Junction patches do not decimate at all.** They are small and bounded. If they become a budget problem at metro scale, distance-cull or HLOD them — do not make them multi-resolution.

**Collision stays pinned at LOD0**, always. A morphing collider is a physics bug factory. Terrain already handles this with a leaf-level collider window (`terrain_lod_system.h`); mirror it.

---

## 5. Boolean union, grade separation, corridor deck

### 5.1 The union largely goes away — and that is the scariest part of this design

`weldSolid` unions ribbons to solve three problems:
1. **Overlap / z-fight** where arms meet ("two ribbons z-fighting"). — **Solved structurally**: the body runs mouth-to-mouth, the junction patch covers the rest. No overlap by construction. Nothing to union.
2. **Block interiors must be holes.** — **Vanishes**: blocks are simply not covered. Nothing to hole.
3. **Weld gaps between junctions.** — **Vanishes**: the graph says which chains meet where.

I checked the consumers: `weldSolid` produces only a `RenderMesh` plus `pierBasesOut`. **The union polygon is used only internally, for triangulation.** The lot/vegetation passes consume `pierBases` (`level_loader.cpp:3101`), not the block outlines. That is a genuine relief and it means the union can go without dragging `city_lots` with it.

**But be honest about what `weldSolid` actually is.** It is 2460 lines of `road_mesh.cpp`'s load-bearing centre and it currently owns: the deck sheet, the soffit, the fascia, sidewalks along every boundary loop, curb returns / `cornerRadius` fillets, junction pad discs, freeway barriers, piers (**and it slides them clear of streets below — it is "the only honest source" for `pierBases`**), crosswalk `mv` baking, grade-separation clearance, roundabout annuli, and the `deckSag` refinement. Replacing it is **not a refactor — it is a rewrite of the road mesher, with a long period of two meshers coexisting.** There is also a public Lua binding (`procgen_bindings.cpp:1876`) that is a compatibility surface.

### 5.2 Grade separation moves from geometry to the graph

Today, `WeldSolidParams::clearance = 5.0` is a **geometric backstop for a topological fact**: two carriageways overlapping in plan weld only if their decks are within 5 m. In a graph-first mesher the decision moves up: a plan-overlap either *is* a graph node (→ junction patch) or *is not* (→ two bodies pass, one on `yAbs`, no interaction).

That is cleaner and it matches the ADR-0055/0056 direction. **The honest cost: bugs in the graph stop being silently unioned away and start being visible interpenetration.** `clearance` is currently absorbing graph errors you cannot see. Expect to find some. Recommend keeping a *diagnostic* clearance check (assert in the drive probe / a bake warning) even after the geometric one is gone.

### 5.3 The corridor deck is the *easiest* case, not the hardest

`UnionSpine {points, yAbs, hw, crossSlope, klass}` maps onto a swept lattice **natively and losslessly**:

- `points` → ring centres
- `yAbs` → ring height, **used directly, never reconstructed** (today it is flattened, unioned, and re-derived by nearest-spine — the corridor is where `heightOf` is most wrong, hence `steps=86, worst 3.20 m`)
- `hw` → profile lateral scale (the aux-lane / gore flare of one-mesher P5 is a per-station width; the lattice widens natively)
- `crossSlope` → `y += lateralOffset * crossSlope[i]`
- `klass` → which profile from the kit
- barriers → profile columns with per-station height → **`blocked=11` fixed as data**

Freeways also have **no sidewalks and no crosswalks**, so the profile is short. And there is already a dedicated drive test with published numbers (`tests/test_drive_freeway.cpp`, `blocked == 0` at :87 and :109).

**Therefore: the corridor deck is stage 1.** It is the most isolated, the most measurably broken, and the one whose authoring already *is* the target representation.

---

## 6. Staged migration, with a stop-and-ship at each stage

Every stage below ships on its own and leaves the tree green. The two meshers coexist throughout — that is not a smell, it is the price.

---

### Stage 0 — Baseline the defect (days)

**Do**: extend `tests/drive_probe.h` with a companion **mesh-quality report**: triangle count, `V/T`, unique positions at 0.1 mm, aspect-ratio histogram, max fan valence (triangles sharing one vertex), degenerate count. Land it as a test that *records* today's numbers.

**Ship**: nothing user-visible.

**Gate**: reproduces the measured baseline — 17874 tris, worst fan 457, 68.8% aspect>20, worst aspect 297339, degenerates > 0 on a 4×4 grid city on hills; `V/T = 2.70`, dedup 4.23× on a 5×5 street grid.

**Why first**: every later stage's claim is a delta against this. Without it you are asserting improvement.

---

### Stage 1 — `emitLattice` + swept corridor deck (the big one)

**Do**:
- `MeshBuilder::emitLattice` + `LatticeSpec` (`mesh_builder.h`/`.cpp`). Winding fixed **once per lattice**, not per face — this is the specific trap `sdf.cpp:150-159` documents and the reason `emitTri`'s per-face `dot(cross(c−a,b−a), normal)` test (`mesh_builder.cpp:424`) cannot survive into shared topology.
- Chain-global arc-length ring sampler (replaces `stripSegs`; keep the `conformTol` heuristic as placement).
- `RoadProfile` for `RoadClass::Freeway`: median box, parapets, verges, lanes, fascia, soffit. Per-station barrier heights.
- Route `corridorDeckSpines` + `corridorRampSpines` through it. **Leave `weldSolid` in place for everything else.**
- `pierBases` still comes from the same pier placer — do not touch it.

**Ship**: freeways are drivable and correctly textured (asphalt + centreline, not concrete). This is visible in one screenshot.

**Gate**:
- `test_drive_freeway`: `blocked == 0` (the gore parapet), `steps == 0`, `worstGrade <= designGrade + eps`. Currently `blocked=11, steps=86, worst 3.20 m`.
- Mesh-quality report on the corridor: `V/T <= 0.7`, aspect>20 fraction `< 5%`, zero degenerates, max fan valence `<= 6`.
- `test_weld_deck_surface` still green for the paths still on `weldSolid`.

**Cost, honestly**: this is where the profile-kit abstraction gets designed under fire, and it is the stage most likely to overrun. `deckSag`/`refineDeck` deletes here, which is a real simplification, but the fascia/soffit/barrier/pier interactions in `weldSolid` are dense.

---

### Stage 2 — Junction patch, N=2 and N=4-equal-K only

**Do**:
- Interval assignment as a **separate phase before emission**: `K_i` pinned from arm profiles, `C_i` on kerb arcs, one parity bit. Hard assert `(ΣK_i + ΣC_i) % 2 == 0` in the weld.
- N=2 → quad strip (bends, tapers, ramp merges — this covers most of the corridor's remaining joints).
- N=4-equal-K → `K×K` grid, Coons-blended heights, arm mouths pinned, `Yc` = width-weighted average.
- Laplacian smoothing on the interior, mouths pinned.
- **Everything else still falls through to the existing earcut fan** (`road_mesh.cpp:2310`).

**Ship**: the 4-way cross — the most-looked-at object in a city — is flat and singularity-free.

**Gate**:
- `tests/test_junction_surface.cpp`: all-quad assertion (every emitted face is one of a lattice's two triangles), max valence `<= 6`, parity invariant holds.
- Drive probe over the 4-way cross on <3% terrain: `worstGrade <= 0.08` (currently **164%**), `worstStep <= 0.02 m` (currently 0.12–0.28 m).

---

### Stage 3 — Street body + sidewalk in one profile; retire `weldSolid` for streets

**Do**:
- `RoadProfile` for `Local`/`Collector`/`Arterial` including kerb and sidewalk as duplicated columns (replaces `road_mesh.cpp:381-383`).
- Bodies run mouth-to-mouth between junction patches. `mu`/`mv` from the parameterisation. Crosswalk `mv` = metres past the mouth, which the junction patch now *knows* exactly.
- N=3 tri-primitive + N≥5 polyhedron cage.
- `weldSolid` retained only for the Lua binding and any level still authored against it.

**Ship**: the whole city. This is the "we can build detail now" moment.

**Gate**:
- Full metropolis drive probe over all nav centrelines: `holes == 0`, `blocked == 0`, `worstStep <= 0.05`, `worstGrade <= maxGrade`.
- Mesh report vs Stage 0 baseline: slivers `< 5%` (from 68.8%), zero degenerates, max fan `<= 6` (from 457), `V/T <= 0.7` (from 2.70).
- A render shot of a 4-way with sidewalks, curbs, zebra and centreline all in the right place.

---

### Stage 4 — Longitudinal LOD

**Do**: `R` rounded to `2^(numLods-1)`, N index buffers over one vertex buffer, per-chain level selection by camera distance. Collider pinned at LOD0. Morph target in `tangent` **only after** verifying the two `drawTerrain` unknowns in §4.2.

**Ship**: the distant city gets cheap.

**Gate**: a test asserting LOD0 index buffer reproduces the Stage 3 mesh bit-for-bit; a test asserting mouth rings appear in every level's index set; frame-time delta on the metropolis flyover.

---

### Stage 5 — optional

Lateral LOD; Blossom-Quad fallback (needs edge-swap / vertex-duplication escape hatches — an even triangle count alone does **not** imply a perfect matching exists); lane-local `mu` to retire the `mu > 3.5` divider-strip overlay (4 shader implementations).

---

## 7. What this does NOT fix

Stated plainly, because the temptation to oversell this is high.

1. **It does not touch buildings.** `shape_grammar.cpp` has **102** of the ~180 `emitTri`/`emitQuad` sites across 2411 LOC and stays unshared soup. `city_lots.cpp` (21 sites) too. This is a road mesher, not a mesh architecture.
2. **LOD0 triangle count probably goes UP.** A full `P`-point profile on every ring — including dead-flat straights that today collapse to one quad — plus power-of-two ring rounding, costs memory and LOD0 triangles for the *privilege* of being decimatable. **The win only materialises if most road area is actually distant.** Budget this before Stage 4, not after. A 6 m connector rounded to 16 rings for a 4-level hierarchy is pure waste; you will need per-chain `numLods` derived from length.
3. **It does not fix the road/terrain contact.** The lattice makes the road's own surface right; the seam where it meets flattened terrain is a separate problem (skirt/fillet). Note terrain flatten-owned vertices already don't morph (`terrain_lod.cpp:149-150`), so road rings over flattened ground must be pinned non-morphing too, or the 0.25 m lift will breathe.
4. **It does not give you CSG.** A plaza with a fountain cut out, an irregular pedestrian island, a roundabout annulus — still earcut. Keep `triangulateWithHoles`. If you ever need *real* booleans, use Manifold (which already solves property discontinuity at boolean intersections and tracks `faceID`); do not roll your own. You are not at that gate.
5. **The physics win is unverified.** `MeshCollider` drops all attributes and would take 4.2× fewer vertices into Jolt's BVH — but `JPH::MeshShapeSettings` may internally weld or index-optimize the list it is handed (`physics_world.cpp:363`). I did not read Jolt to check. Do not put this in a commit message until you measure it.
6. **UV semantics live in four places.** `common.metal` says the RoadMarkings logic is mirrored in WGSL, Vulkan and `scene.cpp`. **I verified only the Metal one.** Stages 1–3 need no shader change (that is the design's biggest gift), but Stage 5's lane-local `mu` would touch all four.
7. **The path tracer's image changes.** `level_scene.cpp:151-175` explodes meshes into per-corner Triangles and is already face-varying, so the representation maps naturally — but longitudinal normal sharing (§1.3) changes the *raytraced* image, not just the viewer.
8. **`weldMesh` does not go away, and its bug is worth fixing anyway.** `weldMesh` (`road_mesh.cpp:260`, anonymous namespace, **called at exactly one site**, :827) ANDs `u/v` into its match key while the pad emits `uv=(0,0)` via `emitTri` and the ribbon emits real UVs via `emitQuadUV` — so **the pad/ribbon seam it was written to fuse cannot weld.** (This is a static read of the code, not a runtime measurement; confirm cheaply by logging `in.vertices.size()` vs `out.vertices.size()`.) In the new design, **make one emitter own both the junction patch and the adjacent body mouth ring, so they share the mouth by index and no weld is needed at all.** If you cannot, `weldMesh` stays as the seam glue — and its 1 mm absolute `posTol` is a real risk at 2 km metro scale where float drift between independently-computed corners may exceed it. Do **not** "fix" it by dropping `u/v` or `normal` from the key: `u/v` is load-bearing (it is `mu`/`mv`) and `normal` is what keeps curb tops from merging with curb faces.
9. **It does not fix a latent crack I suspect but did not reproduce.** `refineDeck` (`road_mesh.cpp:1779-1808`) claims crack-freedom because `edgeSags` is stateless. But `depth` is per-triangle and `deckSagMaxDepth = 8` is shared: two triangles reaching a shared edge at *different* depths could disagree if one has saturated the cap. The claim holds only while neither side saturates. This code deletes in Stage 1, so the question may be moot — but do not cite the comment as proof of correctness in the meantime.
10. **It buys nothing for HLOD, occlusion, or the distant-city mass boxes.** Different tier of the plan.

---

## 8. The single line to take away

The road mesher does not need a mesh library, a half-edge, a decimator, or a boolean engine. It needs to **sweep the swept surface it is already handed** (`UnionSpine`) into the parameterisation the shader is already written against (`mu`, `mv`), as a shared-ring lattice whose connectivity is index arithmetic — and to build junctions as **mapped blocks that interpolate elevation instead of choosing a winner**, with parity paid on the kerb.

Everything the user asked for — quads, subdividability, decimation, detail — is a consequence. The cost is a rewrite of `weldSolid`, a period with two meshers, and a triangle-count regression at LOD0 that only pays back at distance.