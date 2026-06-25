# Road corners, self-intersection, and texture-based markings — research + plan

Status: **proposal / investigation** · Date: 2026-06-24 · Relates to ADR-0044 (junction
trim), ADR-0048 (SDF roadbed + stroking), ADR-0049 (editable RoadNet), ADR-0050 (path tool)

This captures three coupled problems surfaced while editing roads in the viewport, the
literature behind each, and a staged plan. It is grounded in the code the editable road
actually runs.

## Where the editable road is meshed today

`buildRoadNetMesh` (`road_net.cpp:118`) → **`buildRoadMesh`** (`road_mesh.cpp:11`), i.e.
the **ADR-0044 per-edge-ribbon + junction-trim** path. NOT the ADR-0048 SDF roadbed
(`unionRoadbed`), which exists and is unused by the editable path. Consequences:

- Carriageway = per-edge ribbons with **mitered chain joins** (`road_mesh.cpp:~459`).
  Inner bends overlap coplanarly (harmless) but **fold** once the turn radius drops below
  the half-width — exactly what dragging a curve node can force.
- Curb returns = a **variable-radius arc** (radius lerps `rA→rB`, `road_mesh.cpp:238`),
  **rejected when `|sin θ| < 0.25`** (`:226`) → falls back to a straight chamfer. That is
  why corners look right near 90° and degrade at acute/obtuse angles.
- Lane markings + crosswalks = **thin draped quad strips** (`paintLine` `:317`,
  `crosswalkBand` `:105`), seated per-vertex on `heightAt`. When the road tessellation is
  coarser than the terrain (or vice-versa) the strips don't lie flush → the **spikes /
  splinters** over hills. Thin marking geometry also aliases.
- Road vertices **do not set UVs** (`u=v=0`); `Vertex` has `u,v` fields (`renderer.h:25`).

---

## Problem 1 — Ribbon self-intersection / folding

**Root cause.** Offsetting a centerline by a half-width is *path stroking*; when the
centerline curvature radius < half-width, the inner offset crosses itself. The miter path
has no defense (the ADR-0048 SDF path does).

**Literature.**
- Robust offsetting of self-intersecting open paths flattens overlaps instead of folding —
  the property we want. Practical libraries: **Clipper2** (Angus Johnson) open-path
  `ClipperOffset`; **CGAL 2D Straight Skeleton & Polygon Offsetting** (exact curb-return
  geometry via the straight skeleton / "motorcycle graph"). Both are **external deps**,
  which collides with the project's "standard library only" rule.
- **Signed-distance-field union** (Inigo Quilez / gmshaders SDF): the distance to a
  polyline is a Minkowski sum, `min` over spines is the union, and meshing `{sdf<0}` with
  disjoint marching-squares cells is fold-proof *by construction*. **Already implemented
  in-house** as `unionRoadbed`/`unionRibbons` (ADR-0048) — no new dependency.

**Plan.**
1. *Prevention (solver side), cheap + high-value.* Treat folding as a generation
   constraint, as you intuited: enforce a **minimum centerline radius ≥ half-width + curb
   margin** on the editable curve. Detect at regen by walking the offset and flagging where
   the inner rail spacing goes non-positive (a fold). On a flagged node either (a) clamp the
   node/tangent to the legal radius, or (b) auto-insert a **turning bulb** — the hairpin
   disc pad already does this for degree-2 bends (`road_mesh.cpp:285`); generalize its
   trigger to fire on the radius test rather than a fixed deflection.
2. *Robust fallback (mesh side).* Where a fold is unavoidable (dense junctions), route that
   locality — or the whole carriageway — through the **SDF roadbed**, which cannot fold.

## Problem 2 — Junction corners / curbs at any angle

**Root cause.** The curb-return arc isn't a true circle and is abandoned at sharp/shallow
angles, so the rounded city-street look you liked only survives near right angles.

**Literature.** Real curb returns (NACTO *Corner Radii*; Minneapolis SDG 3.7C) are
**simple / two- / three-centered circular arcs** of a fixed radius — typically 3–4.5 m,
as low as ~0.6 m in tight urban corners. The defining property is **tangency to both kerb
lines**, which a fixed-radius fillet gives at *every* angle.

**Plan.**
1. Replace the variable-radius arc with a **true fixed-radius fillet tangent to both kerb
   lines**. For interior angle θ between two arms: center on the angle bisector at
   `d = r / sin(θ/2)`, tangent points at `t = r / tan(θ/2)` back along each arm. **Clamp r
   so `t ≤ available arm length`** — the arc shrinks gracefully on acute corners instead of
   being rejected. Drop the `|sin θ| < 0.25` guard. Works acute *and* obtuse.
2. The sidewalk wraps a **concentric arc** offset outward by `sidewalkWidth` (the corner
   loop is already structured this way at `:256` — feed it the true arc). This makes
   corners read right **with or without sidewalks**.
3. *Subsumes-everything alternative.* The SDF roadbed yields rounded curbs at all angles
   for free (Minkowski sum with a disc), solving Problems 1 and 2 together — at the cost of
   grid-resolution crispness on straights and a larger change.

## Problem 3 — Markings/crosswalks as a shader, not geometry

**Root cause.** Marking geometry is brittle over terrain (spikes/splinters), aliases when
thin, and needs more triangles for each variant (double/broken yellow).

**Literature / industry.** The standard is a **UV-remapped road material / decals** (UE &
Unity "smart road material"): markings are a fragment-shader function of road-local UV
(`u` = distance along, `v` = across), so they ride the asphalt surface and **conform to
terrain for free**, with cheap variants (solid / double-yellow / broken, lane count,
dashes) and crisp edges via `smoothstep`/`fwidth`.

**Feasibility in this engine.** Good, with one real piece of plumbing:
- `Vertex` already has `u,v`; the path tracer already samples per-hit `rec.u/rec.v`
  (`scene.cpp:559,623`). So UVs reach the shader — roads just need to **bake** them:
  `u` = arc-length along the chain (`traceChains` already gives chains), `v` = signed
  lateral in `[-1,1]` across the carriageway. Write them while stroking the ribbon.
- The procedural surface library `applySurface(id, base, worldPos, n)` (`scene.cpp:235` +
  `common.metal`) is **world-position based and does not take UV today**. Add a
  `RoadMarkings` surface id and **extend `applySurface` to also receive `(u,v)` + a few
  params** (lane count, centerline style, dash period). It then composites white/yellow
  paint over the asphalt albedo procedurally. Per-road params can pack into material
  `flags`/a field; `v` already encodes lateral position.
- Crosswalk = a zebra via `fract` across the band where `u` is within `crosswalkDepth` of a
  junction mouth — also shader-side, also conforms for free.
- One implementation in `common.metal` + `scene.cpp` covers both the offline tracer and the
  viewer (shared surface library). Removes the entire spike/splinter bug class and frees
  geometry.

---

**Landed.** (Problem 3) `RoadMarkings` surface — the editable road's carriageway bakes
road-local UV (u = lateral, encoded 1=left / 2=centre / 3=right so non-carriageway u=0 is
excluded; v = arc-length), and `applySurface` (scene.cpp + common.metal) composites a
double-yellow centreline + white edge lines procedurally instead of stroking stripe
geometry. Conforms to terrain for free, no z-fight, crisp at any distance; the whole
spike/splinter class is gone. The geometric `paintLine` pass is gated behind
`shaderMarkings` (off for the procgen union path, which keeps its baked markings). v1 is
2-lane (centreline + edges); multi-lane dashed dividers want a per-road lane-count param
(v2 — the arc-length `v` is already baked for the dashes). (Problem 2) `curbReturnFillet`
— a true fixed-radius arc tangent to both
kerbs, robust at any angle. (Problem 1) `fairHermite` — the editable road's spline is
sampled with its curvature capped to a minimum radius (half-width + sidewalk + margin) by
blending toward the chord, so an over-tight bend can't fold the ribbon or its sidewalk.
Note on the journey: a per-vertex corner fillet and a tangent-magnitude scale were both
tried and rejected (the first can't reach the target radius once it fires; the second
trades apex curvature for endpoint hooks) — chord-blend is the one that provably converges.

## Recommended order (each independently shippable)

1. **Markings → shader** (Problem 3). Biggest visual win, kills the worst bugs, mostly
   additive (bake UV + a marking surface), independent of the corner work.
2. **True fixed-radius curb fillet** (Problem 2.1). Localized change to `road_mesh.cpp`;
   restores + hardens the rounded-corner look at all angles, with/without sidewalks.
3. **Folding prevention** (Problem 1.1): curvature clamp + generalized turning bulb.
4. **Optional, larger:** evaluate moving the carriageway onto the **SDF roadbed**, which
   subsumes 1+2 with one fold-proof, all-angle-rounded path — weigh against the look change
   and effort once 1–3 are in.

**Dependency note.** Clipper2 / CGAL would solve 1–2 cleanly but are external deps,
disallowed here; the in-house SDF roadbed is the dependency-free equivalent and is already
written.

## Sources
- Parish & Müller, *Procedural Modeling of Cities* (CityEngine origin).
- *StreetGen: in-base city-scale procedural generation of streets* — arXiv:1801.05741.
- CGAL, *2D Straight Skeleton and Polygon Offsetting* — doc.cgal.org.
- Angus Johnson, *Clipper2* offsetting (open-path self-intersection handling).
- NACTO *Corner Radii*; Minneapolis Street Design Guide 3.7C *Curb-return radii*.
- Inigo Quilez / GM Shaders, *2D signed distance fields* (rounded joins, smooth-min).
- UE & Unity road-marking materials (UV remap / decals): Epic "smart material for roads",
  Unity road-texture UV remapping.

---

## Round 2 — junction joins, drivable profile, terrain conforming

Three more issues from driving the analytic roads: junction joins overlap in various
configs; roads mirror every terrain bump (undrivable); terrain pokes through the road
("green poke-through") because the road never carves the ground.

### A. Junction joins (overlap at knots)
**Root cause.** The junction pad fanned triangles from the node centre V to the boundary
ring, assuming the ring is **star-convex from V**. T-junctions, mixed-width arms and notched
rings violate that, so the fan **self-overlaps**.
**Fix (landed).** `triangulatePolygon` — ear-clip the ring boundary directly; robust for any
simple ring, no centre assumption. **Remaining:** (1) setback/trim consistency — the ribbon
pull-back uses the *clamped* setback while the mouth geometry assumed the *unclamped* value,
so the ribbon-pad seam can gap or overlap; share one value. (2) near-parallel arms
(`denom~0`) clamp more carefully (spoked/extreme hubs are an acceptable-"decent" edge case).

### B. Drivable vertical profile (hills / valleys / switchbacks)
**Root cause.** Every centerline sample sits on **raw** terrain (`heightAt`), so the road
mirrors every bump — a roller-coaster, not a road. **Literature** (corridor model; WSDOT/
AASHTO vertical alignment; cubic-smoothing-spline grade extraction): give the road a
**smoothed, grade-limited vertical profile** `y(s)` (3-point moving average / clamped slope,
desirable max grade ~6%) and build the surface on `y(s)`, not raw ground.
**Plan.** Pure `roadProfile(sampleHeights, arcLengths, maxGrade)` -> smoothed, grade-limited
profile. Testable. Road surface and the conform target (C) both use it.

### C. Terrain conforming (carve ground to road; kill green poke-through)
**Root cause.** The editable road **drapes** but **never carves**; the terrain mesh is built
independently (`terrain.cpp`), so where the road dips below ground — or coarse road quads
miss a bump — green pokes through. Generated cities carve via Lua `terrain.conform` ->
`TerrainFlatten` regions folded into the terrain (`applyFlatten`); editor roads have **no
such path**. **Literature.** Cut-and-fill corridor grading: subgrade = `y(s)`, **cut** where
land is higher, **fill** where lower, **feather** the shoulder back over a falloff (the
embankment) — exactly `makeFlattenRamp(a,b,yA,yB,halfWidth,falloff)` / `applyFlatten`, which
already exist in `terrain.h`.
**Plan.** (1) `roadConformRegions(net, profile, shoulder, falloff)` -> a flatten ramp per
edge at `y(s)` plus junction pads (pure, testable). (2) Build the road surface on the **same**
`y(s)` so road and terrain agree exactly. (3) Editor integration: on regen, inject the regions
into the level terrain params and rebuild the terrain mesh/collider (heavier, editor-coupled).

**Order:** A (landed: ear-clip) -> A-seam -> B (profile) -> C (footprints) -> editor terrain
rebuild. B and C share `y(s)`, so they land together.
