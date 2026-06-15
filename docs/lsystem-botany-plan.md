# Procedural botany — L-system realism plan

A plan for growing the L-system from "hotdog-finger trees" into a generator
capable of believable **trees, vines, and flowers** (and, structurally, a lot
more — see *Reach* below). The botanical model follows Prusinkiewicz &
Lindenmayer, *The Algorithmic Beauty of Plants* (ABoP) — the standard reference
for L-system plant modeling — adapted to our value-type substrate (ADR-0021).

This is a planning doc, not a decision record. Two decisions it implies — the
**mesh-skinning approach** and **parametric L-system semantics** — should be
ratified as ADRs before the corresponding phase starts (flagged inline).

Relevant code today: `src/engine/procgen/lsystem.{h,cpp}`,
`src/engine/procgen/sdf.{h,cpp}`, `src/engine/mesh_builder.{h,cpp}`,
`src/engine/level_loader.cpp` (tree loading), `docs/lsystem-grammar.md`
(author reference).

---

## 1. Why the trees look wrong

The current trees fail realism on four axes. Critically, **two of the four are
not engine limitations** — the machinery exists and is simply unused or
under-parameterized. The plan must not rebuild what's already there.

| Symptom | Root cause | Already in engine? |
|---|---|---|
| Branches too thick / don't taper | `taper` (per-segment) defaults to `1.0` = off; thickness is a fixed multiplier, not derived from the subtree it carries | Partial — `taper` + `radiusTaper` exist (`lsystem.h:37-40`), but can't express the pipe model |
| Flat, fan-shaped, not 3D around the trunk | Grammars use only `+`/`-` (yaw); no roll between successive branches | **Yes** — full 3D turtle, pitch `&`/`^` + roll `/`/`\` already implemented (`lsystem.cpp:79-84`); just unused, and no phyllotaxis |
| No leaves | `leafRadius` defaults to `0.0`; when on, leaves are SDF spheres (wrong primitive) | **Yes** — `L` symbol + `turtleLeaves()` (position+heading) exist (`lsystem.cpp:75-78,131-145`) |
| Blobby, untextured, thin twigs vanish | SDF/Surface-Nets skinning can't represent branches thinner than ~1.5 grid cells and emits no UVs/tangents | N/A — see §4 |

**Step zero (no engineering):** enable continuous `taper`, set a `leafRadius`,
and rewrite the stock grammars to interleave roll/pitch with golden-angle
spacing. This validates direction immediately and de-risks the rest. It will
*not* reach realism — that needs §3 and §4 — but it proves the gap is real and
the levers work.

---

## 2. What ABoP gives us (the model we're adopting)

ABoP's value is a small, composable vocabulary that spans every plant we care
about. We pull these constructs, in rough order of realism-per-unit-effort:

1. **Parametric L-systems** — modules carry parameters: `F(len,rad)`, `+(angle)`,
   `!(width)`. Magnitudes live in the string, not in one global `TurtleParams`.
   This is the enabling change; most of the others ride on it. (ABoP ch. 1.10.)
2. **The pipe model of tree form** — branch diameter from the cross-section it
   supports: `r_parent^n = Σ r_child^n`, `n ≈ 2–3` (da Vinci / Murray's law).
   The single biggest fix for taper. Requires a **bottom-up pass** over the
   branch tree — impossible in a pure forward turtle walk. (ABoP ch. 2.)
3. **Phyllotaxis** — successive organs roll by the golden angle (~137.5°) around
   the parent axis, spiralling instead of stacking coplanar. *This* is the "3D
   around the trunk" fix, and it governs leaf, petal, and floret layout alike.
   (ABoP ch. 4 — the spiral/lattice phyllotaxis chapter.)
4. **Tropisms** — a per-step heading bias: gravitropism (branches bend up/down),
   phototropism (toward light). Turns one grammar into upright poplar vs. droopy
   willow vs. climbing vine. (ABoP ch. 2.6, the tropism vector `T`.)
5. **Stochastic + context-sensitive productions** — weighted random rules
   (we *have* this, `lsystem.cpp:21`) plus context (`a < b > c`) for signals
   that propagate along the plant (apical dominance, flowering waves). (ABoP
   ch. 1.7–1.8.)
6. **Standard module alphabet** — `!` set width, `'` set/scale color, `%` cut
   (prune the rest of the branch), `~` random orientation, `{ . }` polygon
   capture for leaves/petals, `$` roll-to-horizontal. (ABoP ch. 1.6, the turtle
   appendix.) Adopting these verbatim means ABoP's published grammars for
   specific species largely *just work* as data.

These five-plus constructs are enough for trees, bushes, vines, ferns, grasses,
and flowers — the variety the user is after — from one interpreter.

---

## 3. Grammar & turtle work (priority-ordered)

### 3.1 Parametric modules *(needs ADR — semantics)*
Extend the rewriter and turtle from `char` to **module = symbol + parameter
list**. `expand()` parses `F(2,0.3)` as one module; productions can reference
formal parameters and use arithmetic/conditions (`F(s) : s>1 -> F(s/2)...`).
This is a parser and data-model change to `lsystem.{h,cpp}`; it subsumes the
"per-rule trunk vs. branch taper" item already noted in `lsystem-grammar.md`.

**Decision to ratify:** how far to take parametric semantics — full ABoP
arithmetic + conditionals, or a restricted subset (params passed through, no
expressions). Recommend the restricted subset first; it covers width/length/
angle carrying without an expression evaluator.

### 3.2 Pipe-model radius pass
After expansion, build the branch **tree** (not just a flat segment list as
`turtleSegments()` does today), then walk it bottom-up assigning radius from
accumulated child cross-section. Replaces the forward `taper`/`radiusTaper`
heuristic for the trunk; the heuristics stay as cheap fallbacks. This is the
real fix for "too thick / no taper."

### 3.3 Phyllotaxis + jitter
Auto-roll by the golden angle on each successive `[` (configurable: spiral,
distichous, decussate, whorled). Add **angle jitter** — draw turn angles from
`angle ± noise` using the seeded RNG we already thread through `expand()`,
extended into the turtle walk. Together these kill the flat-fan and the
mechanical regularity.

### 3.4 Tropisms
Add a turtle-level bias vector applied per `F`: bend the heading toward a target
direction by an amount scaled by branch flexibility. One parameter set yields
upright/weeping/climbing forms — and is the backbone of **vines** (strong
phototropism + low rigidity + a `twine` bias around supports).

### 3.5 Curved + organic internodes (rotation-minimizing frame, surface bumps)
The current twigs are straight tubes because each internode is one straight
capsule. Two additions make branches read organic:
- **Curvature:** smooth the per-branch node polyline (Catmull-Rom) and carry a
  **parallel-transport (rotation-minimizing) frame** instead of the accumulated
  `Mat4` orientation, which drifts in roll. Gives gentle natural curvature and —
  crucial for §4 — twist-free bark UVs. Tropisms (§3.4) supply the *directional*
  bend (gravity droop / phototropism); this section supplies the *smooth* bend.
  (ABoP renders straight internodes; this is our real-time upgrade.)
- **Surface bumps:** displace each bark **ring radius** with low-frequency noise
  (fork swell, bark irregularity, taper wobble) in the cylinder skinner (§4.1) —
  cheap, and it removes the machined-tube look. (ADR-0028 §4.)

### 3.6 Remaining ABoP alphabet
`!`, `'`, `%`, `~`, `$`, and polygon capture `{ . }` (§4.2). Once these land,
published ABoP grammars become copy-paste level data.

---

## 4. Mesh skinning — beyond SDF *(needs ADR)*

ADR-0021 commitment #3 makes SDF our geometric-modeling path. That stands for
**CSG/organic fusion**, but SDF is the wrong default for *branches*:

- **Thin twigs are unrepresentable.** `buildTurtleMeshSdf` floors capsule radius
  at `1.5 × cell`; twigs are far thinner than the trunk, so fine branches either
  vanish or balloon. Capturing them needs an enormous grid — cost is
  **O(resolution³)**. You cannot have a fine canopy and a sane grid at once.
- **No UVs or tangents.** Surface Nets emits gradient normals only, so no bark
  texture and no normal mapping — hence the clay look (today masked by the
  height-based `bakeHeightColor` vertex tint, `level_loader.cpp:534`).
- **Blobby.** Smooth-min rounds away bark ridges and crisp forks.

### 4.1 Generalized cylinders for branches (recommended primary path)
The classic approach (Weber & Penn 1995; the basis of SpeedTree and Blender's
sapling add-on): **sweep a ring of vertices along each branch polyline**.
- ring radius from the pipe model (§3.2),
- **UVs flowing along length × around circumference** → real bark + tangents for
  normal maps,
- ring frame from the RMF (§3.5) so the texture doesn't spiral,
- cost **O(branches × ring-verts)** — twigs are free, no volumetric grid.

This is a **Mesh generator** in the ADR-0021 sense (produces the `Mesh` value
type via the mesh builder), so it does *not* contradict the SDF commitment — it
sits alongside it. Branch junctions can simply overlap at first (cheap, hidden
under bark/foliage); proper junction stitching is later polish.

**Hybrid (the sweet spot):** SDF smooth-union for the **lower trunk / root
flare / burls** where fusion genuinely matters, generalized cylinders above.
Keeps the existing SDF investment exactly where it shines.

**Decision to ratify (ADR):** adopt generalized cylinders as the branch-skinning
path and define the SDF/cylinder boundary (the hybrid split). This is a real
addition to the modeling story in ADR-0021 and deserves a record.

### 4.2 Leaves, petals, fruit — cards, not spheres
Foliage is **flat with an alpha-cut silhouette**; SDF spheres are fundamentally
wrong. We already emit oriented leaf placements (`turtleLeaves()` → position +
heading). Render them as **textured quads** (a couple of cross-quads per cluster
for volume), oriented to the heading with phyllotaxis roll + slight random tilt.
ABoP's polygon-capture syntax `{ . . . }` generalizes this to **arbitrary
leaf/petal outlines** — the same mechanism builds flower petals and broad
leaves. Needs **alpha-test/cutout** material support (the `RenderMaterial` has
`albedoMap` + `opacity`; confirm the shader does alpha-test).

---

## 5. Phasing

Each phase is independently shippable and visibly better than the last.

- **Phase 0 — Free wins (no engine code).** Turn on `taper`/`leafRadius`;
  rewrite stock grammars with roll/pitch + manual golden-angle spacing. Proves
  the levers.
- ✅ **Phase 1 — Generalized-cylinder skinner** (§4.1) with pipe-model radii
  (§3.2). Done: `src/engine/procgen/tree.{h,cpp}` (`growTree`) sweeps tapered
  bark rings (UVs + tangents) and solves radii bottom-up by the pipe model.
- ✅ **Phase 2 — Leaf cards + alpha-test material** (§4.2). Done: double-sided
  alpha-cut leaf cards + `RenderMaterial::FLAG_ALPHA_TEST` (Metal-shader
  discard). Procedural bark/leaf textures generated CPU-side in `tree.cpp`.
- ✅ **Phase 3a — Parametric L-system** (§3.1). Done: `ParametricLSystem` in
  `lsystem.{h,cpp}` — modules with parameters, expression-valued successors.
  *Remaining:* RMF bark curvature (§3.5) for twist-free UVs.
- **Also landed:** a collidable `shape:"tree"` level entity (static triangle
  `MeshCollider` from the bark) — a real object to bounce off / shoot at.
- **Phase 4 — Tropisms, phyllotaxis as grammar, jitter, full alphabet**
  (§3.3, §3.4, §3.6). Phyllotaxis + angle jitter are in; tropisms and the wider
  alphabet (vines, flowers, ferns) remain. Where the *variety* lives.

The two ADRs flagged in §7 (branch skinning; parametric semantics) are now
worth writing retroactively, since the implementations exist to describe.

Phases 1–2 alone move the trees from "hotdog fingers" to "recognizable tree
with foliage." 3–4 are what make a *variety* of believable species.

---

### 3.7 Recipe composition via attach points (the sakura example)
Realistic plants are *composed*, not generated by one grammar. The mechanism
(ADR-0028 §2): a recipe exposes **named attach points (sockets)** — flagged
skeleton nodes (ADR-0026) — that a child recipe populates, all in free Lua (L2).
The leaf-card loop in `flora.tree` (`lsystem.leaves` → translate/orient a card at
each apex) is the embryo of this; generalize it.

Worked example — **sakura**:
1. `flora/sakura.lua` runs a branch L-system; terminal nodes are flagged as
   `"blossom"` attach points.
2. `flora/sakura_blossom.lua` makes a blossom variant (a tiny grammar or
   parametric petals like `flora.flower`); the tree recipe **loops over the
   blossom attach points and populates** them with per-instance jitter/scale,
   thinned by a noise field.
3. `scatter/sakura_grove.lua` scatters sakura instances over a region (ADR-0027)
   — the "base"/grove is just a scatter recipe with sakura as the species.

Same mechanism at three scales (blossom-on-branch, tree-in-grove, later
prop-on-building); child seeds derive from `parentSeed + attachIndex` (ADR-0002).

## 6. Reach — "and maybe much more?"

The ABoP toolkit isn't tree-specific; once the constructs above exist they
compose into far more, all on the existing `(params, seed) -> Mesh/Frame`
substrate (ADR-0021):

- **Vines & climbers** — tropism (§3.4) toward supports + low rigidity.
- **Flowers** — phyllotaxis (§3.3) places florets/petals; polygon capture
  (§4.2) shapes petals; parametric modules size them by age.
- **Grasses, ferns, shrubs, corals, kelp** — same interpreter, different
  grammars/params.
- **Growth over time** — L-system iteration count is an age axis; ties into the
  "temporal generators" track (ADR-0021 commitment #4) for *animated* growth.
- **Authoring as data** — these grammars are already level JSON
  (`lsystem-grammar.md`) and will become Lua/`flora` recipes (ADR-0023, -0025),
  and eventually node-graph data (ADR-0021 Phase C) — no consumer changes.

## 7. Roadmap & decisions

- Slots under **Tier 4 → Phase B** of `docs/ROADMAP.md` (the L-system is one of
  the "three generators on one substrate").
- Two ADRs to write before building: **branch mesh skinning** (generalized
  cylinders + SDF hybrid, §4.1) and **parametric L-system semantics** (§3.1).
- Update `docs/lsystem-grammar.md` as the alphabet grows; its "Planned
  extensions" list is this plan's Phase 0/2/3 in miniature.
