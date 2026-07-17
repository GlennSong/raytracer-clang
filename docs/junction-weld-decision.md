I have enough verified grounding. The key contradiction between the briefs is resolved: `weldSolid` already wraps sidewalks robustly (the band at `road_mesh.cpp:1652` rides the *unified boundary loop*, "wraps junction corners as one piece" — it does NOT pile up), so the `boolthenquad` brief's claim that weldSolid strokes un-unioned sidewalk rails is wrong (that's the older `buildSideRails` analytic path at `:359/:804`, not weldSolid). The measured 16% double-cover is a **lattice-only** failure, confirmed at `road_net.cpp:336-341`.

---

# DECISION: Road-junction welding re-architecture

## 1. Direction — Option B, hybrid-scoped to the junction pad

**Keep the swept-lattice bodies. Replace every per-degree `junctionPatch` template with ONE indirect quad-meshing routine over the FULL-ring junction boundary.** Do not swap the whole mesher (rejects A and C).

**Why B converges where the templates don't, and why it beats A/C:**

- The failure is *localized*. `buildRoadNetLattice` (`road_net.cpp:281`) never branches on degree — it sweeps one clean body per chain and calls exactly one fill entry, `junctionPatch(arms[v])` (`road_net.cpp:359`). The entire per-degree zoo is inside one 58-line function (`road_lattice.cpp:410-468`): `N==3 → junctionPatchT` (embedded-branch Coons, hand-tuned far/near-verge sign work, `:367-407`), `N==4 → coonsPatch` grid (`:442-449`), `N>=5 → centroid fan` "spoke soup" (`:451-467`, comment at `:451` literally calls it a "Stopgap"). Each degree is a distinct code path — this is the divergence, and it is deletable in one place.
- The bodies are already the target quality (V/T~0.5, terrain-conforming, arc-length UV, real curb/lane geometry via `streetProfile`). B keeps 100% of that and rewrites only the small junction fraction of the footprint.

**A vs B is genuinely close** — both `polyquad` and `boolthenquad` correctly converge on the *same* core algorithm (triangulate the junction hole, combine tris into quads). The deciding factor is **which base already ships the quality you want to keep**:
- Option A (fix weldSolid) inherits robustness + free sidewalk-wrap, but weldSolid's *bodies* are earcut fans with nearest-spine height that **steps everywhere, not just at junctions** (`sample()`'s `bestD2` argmin at `road_mesh.cpp:1490-1526`). Fixing A means rebuilding whole-surface body quality the lattice **already has**, and re-touching height code coupled to `oSpine` identity, `authoredAt` elevated-deck detection, and superelevation.
- Option C (SDF/dual-contour, `unionRoadbed` `road_mesh.cpp:2141`) is complete but drape-only, 0.4 m grid stair-steps, no authored decks/barriers/grade-sep — reaching weld parity is the largest hidden cost (~400-800 LOC) and trades one quality ceiling for another.

B is the smallest diff to the convergent endpoint. The research consensus (indirect quad meshing) is applied to the base that already has good bodies.

## 2. The single robust routine at its core

**Constrained ear-clip triangulation → greedy triangle-pairing into quad-dominant cells** — the "indirect" family (`polyquad` finding; Blossom-Quad/Q-Morph are the same family with a costlier matcher).

- **Front-end already ships:** `triangulateWithHoles(outer, holes)` (`triangulate.h:21`) is a mapbox/earcut port that connects only existing boundary vertices — it **never adds Steiner points on the boundary**. This is what honors arm-mouth K *by construction*: the pinned mouth segments survive triangulation verbatim, and the combine step only ever deletes *interior* diagonals. One routine, every degree, boundary matched — no per-config logic.
- **Boundary-matching story (HARD-1):** build the closed junction loop from each arm's **full profile ring** (not the carriageway slice) joined by kerb-return fillet arcs between adjacent arms. Because the mouth vertices ARE the body's end-ring (`ring0/ringN` exposed at `road_lattice.cpp:114-119`), the seam is bit-identical → no T-junctions.
- **Parity (for the optional all-quad upgrade):** Bishop's lemma — an all-quad fill needs an even boundary-edge count. The free fillet arcs absorb this: pin Σ(arm K), then bump one fillet's subdivision by ±1. This is a 1-DOF knob, not a special case. **Only needed if you take Stage 4.** Greedy pairing (quad-dominant) needs no parity control.
- **Ship greedy first** (~150-300 LOC: weld `triangulateWithHoles`' Vec2 triples to recover tri-tri adjacency, greedily merge pairs weighted by quad angle-quality, smooth interior). All-quad Blossom (Edmonds min-cost matching) is a ~600-1200 LOC optional upgrade with the same framework; Blossom V is non-vendorable (non-commercial license) — **do not plan on it.**

## 3. Sidewalks / kerb-returns — they fall out, but only after the boundary fix

The root cause of the 85%-at-junctions double-cover is a **boundary-construction bug, not the fill algorithm**: `mouth()` (`road_net.cpp:336-341`) slices carriageway columns `[2, 2+cw)` and drops the sidewalk/curb columns. So each body sweeps its own raised sidewalk to the pad edge while the pad covers carriageway only — the two sidewalk surfaces overlap at every corner. `trimSpine` trims each body by carriageway half-width only (`rad = width*0.5`, `road_net.cpp:288`).

**Fix:** make the arm mouth the **full ring** — columns `[0, cw+4)`: `sidewalk(mu=-0.6) → curb(mu=-0.05) → lanes(mu=1..3) → curb → sidewalk` (layout confirmed at `road_lattice.cpp:216-228`). The junction then *owns* the sidewalk annulus, and the fillet arcs between arms are the kerb returns wrapping the corners — one boundary, every corner, any degree. This is exactly the model weldSolid already proves works (`road_mesh.cpp:1652` outer band on the unified loop) and the `sidewalk` brief's "junction owns the annulus." `curbReturnFillet` (`road_mesh.cpp:20`, a fixed-radius tangent arc, capped via `tMax`) already computes the corner arc but is currently wired only into the old analytic path (`:542`), never into `junctionPatch` — wire it there.

## 4. UV and interpolated height

Both fall out of **one small harmonic (Laplace) solve** over the interior triangulation nodes, with the boundary ring as Dirichlet data:

- **Height (HARD-3):** the mouth ring vertices already carry correct 3D heights (curb lip at `curbHeight`, carriageway at grade, all draped to ground). Solve ∇²h=0 interior with those as boundary → smooth blend of all arm heights, no nearest-spine step, degree-agnostic. A few Gauss-Seidel sweeps on a ≤few-hundred-node system; hand-rolled, no Eigen. `coonsPatch` already does transfinite interpolation for the equal-width 4-way (`road_lattice.cpp:314-320`) — this generalizes it to any boundary.
- **UV (HARD-2):** interpolate `mu` with the *same* solve — `streetProfile` already stamps per-column `mu` (`-0.6`/`-0.05` sidewalk, `2+f` carriageway) onto the ring vertices. Harmonic `mu` across the interior → the marking shader paints asphalt on lanes and concrete on the wrapped sidewalk. `mv` (arc length) has no single direction on a pad; accept unmarked asphalt interiors (real intersections mostly are) rather than threading dashes across.

`triangulateWithHoles` returns Vec2 triples in the ground plane, so triangulate the **projected** footprint, then lift each welded vertex by the harmonic height. Reuse the corner-snap epsilon (`road_lattice.cpp:434-439`) for the vertex weld that recovers adjacency.

## 5. Keep / Delete / New (file:line)

**KEEP (untouched):**
- All body sweeping: `sweepRoadLattice`, `streetProfile` (`road_lattice.cpp:212`), `ring0/ringN` seam exposure (`:114-119`), `emitLattice` winding/degenerate-drop (`mesh_builder.cpp`).
- `coonsPatch` (`road_lattice.cpp:295-345`) — reuse as the interior transfinite fill for the clean equal-width 4-way; it already satisfies K-match + UV + height interp.
- `curbReturnFillet` (`road_mesh.cpp:20-52`) — wire into the new boundary builder.
- `triangulateWithHoles` (`triangulate.h`), `weldSolid` (`road_mesh.cpp:1275`) as the **fallback/default** until the lattice reaches whole-city parity.

**DELETE:**
- `junctionPatchT` (`road_lattice.cpp:367-407`, ~40 LOC).
- The `N>=5` centroid fan stopgap (`:451-467`) and the T dispatch (`:424`).
- The carriageway-only `mouth()` slice logic (`road_net.cpp:336-341`) — replaced by full-ring.

**NEW (~250-450 LOC):**
- Full-ring `mouth()` + arm-sidewalk trim so bodies stop at the pad (extend `trimSpine`'s effect to sidewalk columns, or a `GapWindow` on the sweep).
- Junction boundary builder: full arm rings + `curbReturnFillet` kerb returns between CCW-adjacent arms → one closed loop.
- Indirect fill: `triangulateWithHoles` → vertex weld → tri-adjacency → greedy quad pairing → interior Laplacian smooth.
- Harmonic height+mu solve (Dirichlet from ring).

## 6. Staged migration (each shippable, gated by surface-scan overlap→0 / under→0 / degenerate=0 AND driving)

The lattice is already opt-in (`net.latticeStreets || RT_LATTICE_STREETS`, `road_net.cpp:476`), so every stage ships behind that flag without touching the default weldSolid path.

- **Stage 1 (smallest thing that measurably drops the 16%):** Widen `mouth()` to the full ring + trim the arm sidewalk to the pad. Route **all** N≥3 through the existing centroid-fan-with-interpolated-height on the full ring (temporarily; it's watertight and already height-interpolates). This makes the pad cover the sidewalk annulus so arm sidewalks stop double-covering. **Gate:** `RT_LATTICE_DEBUG` surface-scan overlap drops from 16% toward the ~2.4% graph-level residual; under→0 (no new holes); degenerate=0. Drive the 168-node city (`tests/drive_probe.h`). *This is a boundary fix, independent of the mesher — it banks the 85% win before any quad work.*
- **Stage 2:** Replace the fan with the indirect routine (`triangulateWithHoles` + greedy pairing) on the same boundary; wire `curbReturnFillet` as the between-arm kerb returns. **Gate:** quad fraction up, overlap still 0, drive-probe clean. Keep `coonsPatch` for the clean equal-width 4-way (measured better than generic pairing there).
- **Stage 3:** Swap the interim distance-weighted interior height for the harmonic solve; add `mu` interpolation. **Gate:** drive — no height steps at any junction; markings render on lanes, concrete on wrapped corners.
- **Stage 4 (optional, likely skip):** Greedy → Blossom min-cost matching for strict all-quad, *only if* measured quad quality demands it. Honest cost: 600-1200 LOC, must be written from scratch (Blossom V non-vendorable), conditional success (needs even parity + a perfect matching to exist). Recommend staying quad-dominant.
- **Later:** flip lattice to default and delete weldSolid — gated on whole-city parity + driving, not on this junction work.

## 7. What it does NOT fix / honest risks

- **The ~15% mid-chain slice of the 16% (~2.4% of footprint) is a GRAPH bug, not a mesher bug** — collinear/duplicate graph edges yield two stacked bodies (`polygonUnion` explicitly does not dedup collinear edges, `road_offset.h:48-49`). **No junction fill fixes this.** If the acceptance gate demands overlap→0 *absolute*, you need a separate planarize/merge-collinear pass on `RoadGraph` upstream. Scope it as an independent task; don't let it block the junction work.
- **Acute forks stay ugly.** No method makes a good quad from a bad boundary. The win is that indirect *confines* the acuteness to a couple of fillet triangles at the fork instead of propagating a singularity chain inward (as paving would). Weight pairing by angle-deviation; verify forks explicitly with the drive-probe before declaring parity.
- **Quad-dominant ≠ all-quad.** Greedy leaves stray triangles near forks/odd-parity. Fine unless a downstream Catmull-Clark/subdivision invariant needs pure quads (then Stage 4). Do NOT split leftover *boundary* triangles — that breaks the pinned K.
- **Lane dashes won't thread through the intersection interior.** `mu` interpolation gets concrete-vs-asphalt right; `mv` continuity across a pad is underdetermined. Accept unmarked interiors.
- **Vertex-weld epsilon is load-bearing.** `triangulateWithHoles` returns Vec2 triples, not indices — recovering tri-adjacency needs a float weld whose epsilon matches the corner-snap (`road_lattice.cpp:434-439`), or pairing silently fails at shared fillet corners.
- **Earcut sliver quality** feeds the pairer poor raw material near forks; a ~50-100 LOC Lawson-flip (constrained-Delaunay) pass on the earcut output before pairing is a cheap quality insurance if forks look bad.
- **Harmonic solve averages inconsistent corners.** If two arms disagree sharply in height at a shared kerb corner, the solve blends visibly — keep the corner-snap height-averaging before the solve.

**Bottom line:** B-hybrid buys guaranteed convergence (one routine, every degree, boundary honored by construction) at **quad-dominant** quality now, reusing the front-end (`triangulateWithHoles`) and the fillet primitive (`curbReturnFillet`) you already ship. You give up Option B-pure's theoretical all-quad aligned lattice, and you never write another per-degree template. Stage 1 alone — a pure boundary fix, no new mesher — banks the 85% double-cover win.
