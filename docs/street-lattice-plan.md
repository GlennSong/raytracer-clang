# Street-mesher rewrite — swept lattice + Coons junctions (Stage 3)

The endgame of the swept-lattice mesher: move **streets** off the earcut union
mesher and delete `weldSolid`. Design context: `docs/road-mesher-research.md`
§2 (body), §3 (junction), §5 (what the union was doing). Freeway is already on
the lattice (`sweepCorridor`); the Coons junction primitive (`coonsPatch`) is
built and tested. This plan is the street integration.

## What we're replacing

`buildRoadNetMesh(net)` (`road_net.cpp:246`) does graph build → constraints →
grade-sep elevation stamping → **`return weldSolid(weldChainSpines(g), wp)`**
(`:417`). That one call is the whole street surface. `weldSolid` (793 lines of
`road_mesh.cpp`) owns: the deck sheet, sidewalks along every boundary loop, curb
returns / `cornerRadius` fillets, per-node junction pad discs (`padCenters`,
deg≥3), crosswalk `mv` baking, freeway barriers, grade-sep clearance, roundabout
annuli, piers + `pierBasesOut`, and the `deckSag` refinement.

Only two outputs are consumed downstream: the `RenderMesh` and `pierBases`
(lot/vegetation passes). The union polygon is internal to triangulation — so it
can go without dragging `city_lots` along (design §5.1).

## The two new pieces

### 1. Street body profile (Glenn's "geometry to conform" point)

The current ribbon is ~2 triangles across its width — "long rectangular
triangles, no geo to conform." The lattice fixes this **only if the profile has
enough lateral columns and rings are placed densely enough that quads are
~square**. A `StreetProfile` (extends the freeway kit in `road_lattice`):

- columns: outer-sidewalk, curb-top (split for the crease), verge, one per lane
  boundary, centre, mirror. ~`2*lanes + sidewalk/curb columns`.
- ring spacing: `min(ringStep, laneWidth)` so a lane-width column × ring quad is
  roughly square — the interior vertices terrain can drape onto.
- draped height: `yAbs` empty → `ground(x,z)` per ring (streets drape). This is
  where the conform finally has vertices to work with.
- `mu` per column for the markings (sidewalk `<1`, carriageway `1..3`); `mv` =
  arc length, set to the crosswalk sentinel except near a mouth.
- curb as split columns (road-side normal vs slab-side), like the freeway fascia.

### 2. RoadGraph node → Coons patch (the seam-free integration)

The one hard contract: **the junction patch and each incident body must SHARE
the mouth ring by index**, or they T-junction. So:

- Each chain body ends at a node with a **mouth ring** = its ring 0/R (K+1
  cross-section points with heights).
- At a node, order incident arms by bearing. Build the boundary curves from the
  arm mouths; the kerb corners between adjacent arms are the fillet (parity slack,
  design §3.1 — pay it on the kerb, never an arm).
- N=4 equal-K → `coonsPatch` (opposite arms = the u/v sides). N=2 → the body just
  runs through (a bend, no patch). N=3 / N≥5 → the cage templates (design §3.3) —
  **defer to a follow-up; start with N=2 and N=4-equal-K**, everything else keeps
  a simple centroid-fan pad as a stopgap (honest, logged).
- Height: each arm mouth is the body's exact ring; the patch interior Coons-
  blends them → no medial-axis step (already proven by `coonsPatch`).
- Emit the patch and the body mouth from ONE place so they share vertices — no
  `weldMesh` needed (and `weldMesh`'s uv-in-key bug, `road_mesh.cpp:260`, retires
  with it).

## Staged, each green + committed + gated

- **3a — StreetProfile + swept street body, isolation.** Profile + a
  `sweepStreetBody(chain, ground)`; not wired. Tests: V/T<0.7, fan≤6, slivers<5%,
  a lane drivable (drive probe holes/steps/blocked 0), sidewalk/curb bands
  present, quads ~square (aspect < ~4 on flat). *(mirrors freeway 1b/1c.)*
- **3b — node→patch adapter, isolation.** `junctionPatchForNode(arms)` → boundary
  curves → `coonsPatch`, sharing mouth rings. Test on a synthetic 4-way + T:
  all-quad, mouths matched exactly, no step across arms at different heights.
- **3c — wire `buildRoadNetMesh` to the lattice.** Bodies mouth-to-mouth + patches
  at nodes; keep the grade-sep elevation stamping (already on nodes as `elev`);
  keep `triangulateWithHoles` for roundabout annuli / plaza cutouts; keep the
  pier placer for `pierBases`. **Gate: `test_junction_surface` steep 8→0**, plus
  the whole-metropolis drive probe == 0, plus Glenn drives it.
- **3d — delete `weldSolid`** (and `weldMesh`, `deckSag`/`refineDeck`, the pad-disc
  path) once nothing calls it. Update the Lua binding `procgen_bindings.cpp:1876`
  (compat surface). Re-run every mesh-quality + drive gate on the whole city.

## Risks (design §5, be honest)

- **Grade-sep moves geometry→graph.** `clearance=5.0` currently unions away graph
  bugs you can't see; without it they become visible interpenetration. Keep a
  *diagnostic* clearance assert in the drive probe (§5.2). Expect to find some.
- **`weldMesh` seam bug** (`road_mesh.cpp:827`, uv in match key) means the
  pad↔ribbon seam never actually welds today — confirm by logging in/out vert
  counts before trusting any "it welded before" intuition.
- **Interval assignment before geometry** (design §3.6): arm K's are pinned; if
  the current code tessellates kerbs downstream of arm capping, that ordering
  flips. Untraced — size it in 3b.
- **Two meshers coexist across 3a–3c.** That is the price; 3d ends it. No standing
  flag — freeway + streets both on the lattice before `weldSolid` dies.

## What this does NOT fix (record, don't oversell)

- Buildings (`shape_grammar.cpp`, 102 emitTri sites) stay soup — separate tier.
- Freeway markings (lane-local UV, task #71) — deferred, unrelated to streets.
- Ramp foot → street weld (#64) — the street lattice makes the *target* street a
  clean mouth, which helps, but the ramp-to-street join is its own step.
- LOD (design Stage 4) — the lattice makes it *possible*; not built here.

## Gate summary (the ratchet)

`test_junction_surface` steep→0 · `test_mesh_quality` on the whole city (V/T<0.7,
slivers<5%, fan≤24, 0 degenerate) · drive probe over the entire metropolis nav
graph (holes/steps/blocked/grade all 0) · **Glenn drives metropolis end to end.**
