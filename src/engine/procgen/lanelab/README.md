# lanelab — the lane-atomic road generator (ADR-0083, ADR-0085)

**What it is.** A road generator that ships beside the swept lattice rather than replacing it:
its own static library, linked into `engine_core` when `RT_ENABLE_LANELAB` is on (the default),
with `lanelab_tool` and `lanelab_tests` as its headless hosts. A level picks a generator per
entity — `shape:"road"` for the lattice, `shape:"lanelab"` for this one. It
takes spline roads with a class (the engine's `RoadGraph` shape, lane layout as `RoadSpec`
bands), expands them internally into constant-width lane ribbons, unions the ribbons into the
pavement, runs one constrained Delaunay over every lane outline, gives every deck vertex the
height of its owning lane's spine by projection (with nearest-partner blending at seams), and
conforms a heightfield around the result. Ramps are anchored on a host's outer lane and
compose their own auxiliary and deceleration lanes; pockets are lanes born out of a neighbour;
junction connectors are generated. Nobody authors a lane.

**Why it exists.** `docs/decisions.md` ADR-0083 — the context (what the lattice does, why the
union family was demoted, what the earlier `proto/roadlab` was), the decision, the two points
the owner signed off (Clipper2 + CDT as pinned submodules; two road generators in the tree), and
ADR-0085 for how a level selects one and what the generator owes the rest of the engine. `docs/capabilities.md`
lists every road capability the engine already has and why lanelab does or does not use it.

**Files** (one header + one `.cpp` per module):

| Module | Role |
|---|---|
| `geom2d` | the only TU that includes Clipper2/CDT: booleans, offsets, closing, point-in-polygon, constrained triangulation |
| `road_graph_spec` | the graph JSON → classes (as `RoadSpec` bands), rules, terrain spec, resolved spines, ramp anchors, pockets, connectors |
| `terrain_recipe` | flat / procedural / stored-grid terrain as an engine `HeightField`; the grid the conform edits |
| `polyline_ops` | frames, projection, crossings, point-in-ring, a segment grid for nearest-distance queries |
| `lane_expand` | roads → lanes from the band slots; ramps recomposed from their anchors; pockets; connectors; derived adjacency |
| `vertical_profile` | terrain smoothing under 80 % of the class grade; node + crossing consistency (iterated, viaducts excluded); ramp hold rules; floor tents |
| `deck_height` | own/deck height fields; nearest-partner blend to the partner's *deck*, recursive through the rank chain |
| `pavement` | footprints, closing (capped fillets where same-level footprints meet), layers, the CDT arrangement, level clustering, welding, mesh checks |
| `terrain_conform` | one owner per grid node (lanes, then road envelopes), skirt + 1:2 slopes, sequential; terrain-vs-deck census |
| `lanelab` | the pipeline, stats, invariants |
| `deck_mesh` / `lanelab_export` | engine `RenderMesh` via `MeshBuilder` (lane-local UVs), GLB / plan SVG / stats JSON |
| `level_import` | a shipped level's recipe + terrain → a lanelab graph; perimeter loop → two-carriageway freeway with diamond ramps onto chosen streets |

**Run it.**

```
cmake --build build-viewer --target lanelab_tool lanelab_tests
./build-viewer/lanelab_tests                                              # 9 cases: geometry seam + the four scenes' invariants
./build-viewer/lanelab_tool build assets/lanelab/ring_city.json --out out/lanelab/ring_city     # summary, PASS/FAIL invariants, .glb/.svg/stats.json
./build-viewer/lanelab_tool from-level assets/levels/metro_v2_test.json --out out/lanelab/metro --diamonds 6
./build-viewer/lanelab_tool build out/lanelab/metro/<name>_lanelab.json --out out/lanelab/metro
```

Open the `.glb` in Blender (File → Import → glTF) to look at it; the `.svg` is the plan.

**Scenes** (`assets/lanelab/`): `grid_city` (flat: blocks, arterials, a boulevard, two freeway
carriageways with a median and two ramps), `valley_viaduct` (a viaduct over a valley, a local road
beneath, a two-lane ramp that dovetails and crosses the valley on its own piers), `hill_junction`
(a divided arterial with a turn pocket and a slip lane over rolling ground, two generated
connectors, an island), `ring_city` (a two-carriageway ring at a design height over two hills,
a grid inside, arterials under it, diamonds at north and east — 56 lanes, one welded deck, ~4 s).

**Invariants** (`lanelab_tool build` prints them; `lanelab_tests` asserts them): decks welded (no
cracks, no non-manifold edges, closed boundary loops); grades within class limits; ramps long
enough for their climb; ramp ends within 2 cm of their hosts' heights; anchored ramp ends within
5 cm of the host lane's centreline; junction heights consistent; multilane roads have derived
adjacency; no terrain above any deck by more than 1 cm.

**Known reds and limits.** `hill_junction` has 3 of 14 032 terrain samples 17 cm above the deck
(under investigation; the Python prototype had 0). Conform is grid nudging, not `TerrainFlatten`
regions. Markings are geometry strips in the lab; the engine paints them from UV. No Lua, no
nav, no level loading — those are the integration ADR's job if the gate is met.

**Is it drivable? Yes, behind an option.** `-DRT_ENABLE_LANELAB=ON` links the lab into `engine_core`
and enables `shape:"lanelab"` in the level loader (`loadLaneLabEntity`, next to `loadRoadEntity`):
the graph is built at load, one entity per material mesh, each with a static `MeshCollider` from the
SAME triangles (the Playable Scenes rule); paint strips are visual only. The authored block round-trips
as a document entity: `{"lanelab": {"graph": "assets/lanelab/ring_city.json"}}` or the inline spec.
`assets/lanelab/levels/ring.json` is the lab level (two sedans and the player on the north gore).
Headless acceptance shots go through the control channel (ADR-0078):
`tools/lanelab_shots.py assets/lanelab/levels/ring.json out/shots --cams out/lanelab/ring_city/stats.json --near`
launches the viewer once in play mode (physics live) and frames every exported gore camera.
OFF (the default) leaves `engine_core` lanelab-free, so the lab stays a separate app until ADR-0083 graduates.

**Blocks and lots: through the engine's own pass.** Alongside the meshes, the loader hook leaves a
`RoadEntity` twin of the lab network in the world (`road_twin.{h,cpp}`: corner-only nodes, class and
paved width per edge, deck heights as absolute elevations on freeway/ramp nodes, the carriageways as
`plan.freewayPlans` for the lot pass's right-of-way band, the STREET subgraph planarised so every
crossing and T is a shared node). A lab level with a `citysim` block (`buildLots: true`) then grows
blocks, lots and grammar buildings exactly as a generated road does, graded on the lab's conformed
terrain (published as the level ground when the level has none). An edge's `lots_range: [s0, s1]` (stations) limits
where the city may build along it — beyond it the street is right-of-way (the twin marks those sub-edges
`baked`, which the lot pass skips as block/lot frontage while keeping the clearance band). The importer
sets it on landing streets (lots stop at the outer frontage road); ring city's arterials carry it by hand. Diagnostics: `lanelab_tool twin
<graph.json>` prints the twin's junction census, the faces `extractBlocks` walks and whether each
survives the sidewalk inset (the failure that once emptied every grid block: a node every 8 m on a
straight edge collapses the miter inset); `RT_LOT_PLAN_SVG=<path>` writes the parceller's blocks,
lots and built plans; `RT_LANELAB_TWIN_SVG=<path>` writes the twin graph with junction dots.

## After the first drive (2026-09-04)

Glenn drove metro v2's freeway in the engine: ramps on and off work as expected. His list, and where
each stands:

1. **Junction boxes — done.** Lane paint stops inside any face a crossing street shares at the same
   level (`deck_mesh.cpp`: `inBox`, then the merged per-street spans so a divided cross road is one
   junction). Every street approach gets a zebra crosswalk (0.5 m stripes on a 1 m pitch across the
   paved width, 1–4 m before the box) and a stop bar where the crossing road's rank is at least its own.
2. **Lots clear of ramps — done, verifying.** The twin's freeway and ramp edges feed the lot pass as
   the routed right-of-way (per-class keep, under-deck re-zoning), and their edge WIDTH carries the
   earthwork (paved + 2 × conform_w), which the building-clearance check reads. Declaring ramps as
   freeway class was tried and rejected: it re-zoned every block inside the frontage roads.
3. **Clearance — done.** Importer clearance 6.5 → 8 m over crossing streets (about 6.8 m under the
   girder); the climb limit went to 18 m so the higher freeway keeps its four diamonds.
4. **Parapets and undercarriage — done.** A 0.9 m parapet on the OUTER edge of the shoulder beside every
   deck boundary edge owned by a freeway or ramp lane that runs along the lane and stands more than
   1.5 m above ground (the first cut stood it inside the lane: a ramp had 2.3 m between walls and cars stuck); a 1.4 × 1.6 m
   box girder under every elevated freeway/ramp lane. Concrete, so collidable.
5. Ramps are long but work; no change asked.

Still open from the same drive: the metro's east-frontage faces that parcel no lots; the lab levels'
low sun; 3 min of build at level load for the metro.

## Blocks that vanished (2026-09-05)

Glenn: the lot outlines in the lane-lab metro followed the OLD metro road graph, not the new one, and
lots crossed streets. He was right about the mechanism being in the code, not the data. `lanelab_tool
twin <graph> --margin 5 [--at x y]` reproduces it headless: the lot pass's first step insets each block
face by the sidewalk with a miter offset that REJECTS the whole face when any edge is shorter than the
offset can turn around. Every dead face counts as open ground for the rim synthesiser, which lays its
rectangles along the face's boundary streets and straight across the streets inside — those rectangles
were the "old" outlines. Metro at the 5 m margin: 30 of 88 faces dead. The twin manufactured the
short edges: a vertex every couple of metres on curved streets (0.25 m simplification), a crossing node
inserted beside a corner, a street end overshooting its T by less than the node tolerance, two junctions
almost coincident. The twin now simplifies at 1.5 m (widths padded by the same), drops corners within
6 m of a junction, removes dead-end stubs under 8 m and contracts junction pairs under 8 m: 3 of 88
faces dead, all with genuine 6.5–7 m edges. The chords are also pinned straight for the engine's
sampler (a node with no tangent and two neighbours gets a Catmull-Rom tangent and bows). Gate:
`lanelab_road_twin_faces_of_a_curved_junction_survive_the_metro_sidewalk_inset`. Measurement tool:
`tools/lanelab_overlap.py` (blocks/lots/buildings against road ribbons, from a city map or a plan dump).
Ramps: the lot pass excludes ramp class as block frontage, so an at-grade ramp foot left its face whole
and lots were parcelled across the ramp lane. The twin now splits every ramp at the 1.5 m mark and emits
the at-grade runs as street edges (face boundaries), the elevated runs as ramps (right-of-way). Metro
lots on streets: 131 → 40; buildings on streets: 0 throughout. `tools/lanelab_lots_over_roads.py` draws
the parceller's blocks, lots and buildings over the lab's pavement plan (with zooms) — the picture that
made this obvious.
Two follow-ups on the ramp split: an at-grade ramp run beyond its host street's `lots_range` (the outer
feet, past the freeway) is baked like the street there, else the rim synthesiser lines it with lots in
open country; and a "gore ribbon" down the wedge between ramp and street was tried and gated off
(`kGoreRibbons`): its chunk endpoints merged into street nodes and the short-edge cleanup contracted
real junctions (metro lost 49 of 143 blocks). The ~38 lots still in ramp-foot gores need the wedge as a
face of its own. The lab levels now also publish a `LevelRoadGraph` from the twin when the level has
none, so `citymap <path> all`, street furniture and the citysim nav work on lane-lab streets.

## Broken blocks, validated in code (2026-09-05)

`lanelab_tool twin <graph> --margin <sidewalk>` now reports BROKEN BLOCKS: every block foot — the face
inset by the margin and pushed clear of the sampled ribbons exactly as `pushPolyClearOfRoads` does —
that overlaps the BUILT paved surface (every lane, freeways and ramps included), with the roads it
crosses. Ring city 0 of 16, curve_blocks 0 of 4; metro 66 of 100 (6,222 m²): every foot over 300 m² has
a RAMP inside it, the rest are junction fillets under 100 m². Gate:
`lanelab_twin_leaves_no_block_foot_on_built_pavement_in_the_clean_scenes`.

The structural fix — a LOT TWIN with every deck run relabelled as a street so it bounds faces, while the
class-faithful twin feeds the unified graph and the right-of-way — is written (`roadTwin(..., forLots)`,
`--lot-twin`) but PARKED (`kLotTwin` in the loader): on the metro it produced two merged faces of 163k
and 54k m² spanning dozens of streets. Suspect: ramps' gore ends dangle beside the carriageway
centreline instead of meeting it, so the carriageway boundary is never closed there. Until it is fixed,
lots still parcel across the elevated ramp runs inside blocks.

Maps: with a lab level running, `citymap <path.svg> all` over the control channel (or
`tools/lanelab_shots.py LEVEL OUT --cmd "citymap <abs path> all"`) writes the engine's layered map;
`tools/lanelab_svg2png.py map.svg map.png 2400 --layers=roads,curbs,blocks,lots` rasterises the block
plan alone (layer styles inherited from the `<g>` groups, legend dropped). Spurs: `lanelab_tool twin`
counts dead-end spurs per face; line ends now snap onto curved roads out to the simplification
tolerance and leftover dead ends are bridged to the nearest street within 20 m (metro: 44 → 20 dead
ends, 8 spurs on 6 faces remain).

## One representation (2026-09-05)

Glenn: two representations will cause problems; there should be one source of truth, and the prototype
should commit to the new one. Adopted. The lane lab's PAVEMENT is the source of truth for the city:

- **City blocks are the holes in the paved surface** (`block_audit.h` `sceneBlocks`: lanes + shoulders
  unioned, holes ≥ 2000 m², simplified at 1.5 m so kerb-return fillets do not reverse under the pass's
  inset). No graph, no face walk. The loader hands them to the engine's `growLotBuildings(blocks, …)`
  entry point with NO clearance graph — the blocks are exact to the kerb, so nothing can land on the road.
- **The twin is a derived graph, not a RoadEntity.** `roadTwin` still produces the class-faithful road
  graph, but only to publish the level's `LevelRoadGraph` for the citysim nav, street furniture and the
  map writer, the way `deck_mesh` derives triangles for rendering. Nothing extracts blocks from it.
- **The audit runs the real pass headless.** `lanelab_tool blocks <graph> [--citysim level.json] --out dir`
  builds the scene, runs `growLotBuildings` on its holes, writes `blocks.svg` (pavement, blocks tinted
  red where broken, lots, built plans) and prints every broken block with its spike area, area on
  pavement, convexity and the roads under it. Gate: `lanelab_the_city_block_pass_yields_well_formed_blocks_
  on_the_clean_scenes` (grid, ring, curve, hill) — the first version of this gate failed on three of them
  while the older twin tests passed, which is what a real gate is for. `--graph-blocks` keeps the old
  face-extraction path for comparison only.

## Walls (2026-09-06)

Glenn hit "a very thin vertical edge in the middle of a freeway lane", then "actual bits of walls", walls
across exit gores, and gaps and missing end caps between wall pieces. One cause and one missing tool:

- A weld crack is an edge used by one triangle, so it was a deck *boundary* to the parapet rule and to the
  slab's side faces: a 0.9 m wall or a height step down the middle of a lane. `deck_mesh.cpp` now tells a
  deck edge from an internal seam by testing a point just beyond the edge for SAME-LEVEL pavement (a
  street under the viaduct is pavement in plan but not this deck). Seams get no wall and no side face; the
  build prints their count and first positions (`lanelab: parapets on N deck edges; M internal seams …`).
  The same test opens the exit gores: the freeway's outer edge beside the departing ramp is a seam.
- `sweepWall(mesh, pts, z, offOuter, offInner, height, color, closed)`: the one wall builder — eligible
  boundary edges are chained into runs, corners mitred (clamped 2x), open runs capped, per-point offsets so
  a run can pass from a freeway shoulder to a ramp shoulder. Any future wall (retaining, kerb, barrier)
  goes through it.

## From the second drive (2026-09-06)

Glenn's list and where each stands:

1. **Holes in the carriageway with the wall running past** — fixed. Invariant added first: every lane's
   footprint must be covered by deck triangles (`every lane's footprint is covered …`, flags lanes under
   60 %); it named three metro lanes (c16.f0, c135.f0, c150.b0) at 6 % coverage. Cause: the cover pass
   in `buildSurfaces` pre-filtered CDT centroids against `bounds(footprints[li][0])`, the bounding box of
   the footprint's *first* polygon only. A lane whose closing pieces left a detached 3 m² sliver as
   polygon 0 was tested against the sliver's box, so its whole 390 m² strip failed the box test, never
   reached `contains`, and every triangle there had an empty cover list and was dropped. The parapet
   pass keyed on the lane's footprint edge, not on the triangles, so the wall ran on past the hole.
   `bounds(const PolySet&)` now spans every polygon; the same `[0]` box was in three places in
   `deck_mesh.cpp` (seam test, girders, parapet eligibility) and is gone there too. Metro: PASS, no lane
   below threshold; +609 triangles; nothing else in the invariant report moved.
2. **Ramps are long; they should land on the nearest suitable road and dovetail into it** — design work,
   not started: an interchange planner picking the nearest collector-or-better within the ramp's climb,
   ending as an added lane with a turn pocket. The lane machinery exists; the planner does not.
3. **Freeway too low for its undercarriage** — measured now, not assumed. New invariant `grade-separated
   pairs clear their structure`: for every lane pair that is separated everywhere it overlaps, the smallest
   deck-to-deck gap must be ≥ `under_clearance` (5 m) + slab + `structure_depth` (1.6 m girders); the
   detail names the pair, gap, both heights and the point. It found four things the old pass missed:
   - the pass tested the centreline crossing point only, and a floor tent peaks at its station and falls at
     design grade, so the deck over the far kerb of a 20 m road sat 0.5 m low. Floor points now carry a
     **half-span** (`floorPts` is x, y, z, half-span) over which the hold is flat, covering the lower road's
     paved band plus the upper road's own width along the upper road. Holding three separate points did
     not work: the max of two cones 14 m apart sags 0.17 m between them.
   - the segment test missed a road whose END lies on the other's interior (the ring's freeway arcs meet at a
     node straight over the arterial; the arc that starts there was found, the one ending a hair short of the
     line was not, so one side of every such node stayed low). Ends within the two roads' paved half-widths
     now count; a shared node still does not.
   - **ramps crossing streets below bridge height were nobody's**: `crossingConsistency` excluded ramps, the
     clearance pass skipped anything under `bridge_h`, so exit ramps sat a metre over local streets (that
     was the undercarriage collision). Ramps now take part in crossing consistency as the side that never
     moves: the street rises or dips to meet a ramp it crosses at grade, exactly as streets already met
     each other. The line between "at grade" and "structure" for a ramp is `ramp_level_dz` (1.5 m, the
     same height the twin uses to call a ramp run a street), not `bridge_h`: with the 4 m line the two
     rules fought — a frontage road was hauled 3 m up to meet an on-ramp, the ramp was then lifted to
     clear it, the road fell back, twelve times over — and each action now moves away from the line.
   - lifted decks must re-agree at their nodes: the consistency loop runs again after each lift round, and
     the node check names its worst node.
   A ramp's floor tent is capped by its class's maximum grade from either anchor, so a hold the ramp cannot
   afford is met as far as the grade allows and the invariant names the remainder — no cliff (the first
   version produced 580 % grades). Ring: 0 of 96 pairs short. Metro: 14 of 126 short (was 50), and every
   one is a ramp that cannot afford the lift between its anchors (fr_a|d1_in_off, fr_b|d3_in_on,
   c65|d0_in_off, fr_b|d3_in_off — the ramp planner's job) or the ramp-landing dovetail where the ramp
   deck still sits ~1 m over its landing street (c6|d0_in_on — the d0 step). Side effects of the round on
   metro: weld cracks 79 → 34, terrain-above-deck samples 2115 → 1105, node mismatch 0.4 cm; one local
   street (c11) now climbs 15 % to meet an at-grade ramp — the planner would land the ramp on it instead.
4. **Piers in the road** — a pier's footprint is tested against every other road's pavement, slides up to
   half a bay along the span, or the bay goes without. Piers in the median for dual carriageways: next.
5. **Medians in skinny openings** — a hole in the asphalt that a 4 m opening removes entirely (and under
   2000 m²) is a kerbed grass island via the median layer; anything with a body is land, a block.
6. **Squared-off land that became no block** — the parceller's miter inset rejected whole holes over one
   short kerb edge (ring: its two largest, 61k and 57k m²). The lab now insets holes with Clipper and hands
   the parceller a zero margin. Ring 24 of 24 holes → blocks; metro 105 of 105 (1739 lots, 0 on pavement).
   `lanelab_tool blocks` reports any hole that yields no block, with the reason.


## The raised slab in the 4-way (2026-09-07)

Glenn stood at metro (−534, 160) below the c67/c70/c199/c196 junction and saw a slab standing a metre
proud inside the intersection, concrete sides showing, ground visible beneath. The lane decks there were
clean — `--probe` showed all four roads meeting at 21.53 m and every lane's deck agreeing — and the
new step detector found no deck step within 60 m. It was the **sidewalk layer**: `layerMeshes` gave each
layer vertex the height of the nearest road **spine**'s profile. At a corner between a wide arterial and a
narrow local climbing at 4 %, a vertex beside the arterial's kerb is nearer the local's spine, so it took
the local's height from 20 m up the hill. Layers now follow the pavement they border: the nearest
**lane**'s deck (`DeckHeight::layerHeight`, shared by the mesher and the check).

Validation, so this class of malformed geometry is caught in code rather than found by driving:

- `driving surface has no steps (> kerb, < bridge_h)` — `surfaceSteps()` walks every deck boundary
  edge and every sidewalk/shoulder/median ring, samples 0.35 m outside, and flags an edge standing more
  than kerb height above or below the pavement there (under bridge height, so viaducts are not steps).
  Fails on pavement steps; lips over bare ground are reported as information only, because an
  abutment's transition cell (ground graded to the deck one node, natural the next, where the deck
  passes bridge height) looks identical to an ungraded embankment edge. Metro after the layer rule: 3 step
  edges citywide (shoulder slivers 0.3–0.5 m low at ramp gores), none within 60 m of Glenn's spot; the
  five lab scenes: 0. `lanelab_tool build … --probe x y r`
  prints the roads' profiles, every lane's own and deck height, deck ownership, separated pairs and the
  steps within r of a point — the first thing to run when something looks wrong at a place.
- `pads:` in the block audit (`lanelab_tool blocks`, exit 2; the block-pass test) — building pads and
  block terraces are TerrainFlattens with priority over roads. Road-graph levels keep them off the
  carriageway with roadClear; a lanelab level had no road graph there, so pads (plan + 2.2 m apron,
  5 m feather) and terraces lifted pavement and sidewalk wholesale (metro: 393 flattens over 60 700 m²;
  ring: 19 over 4 960 m²). The loader now builds a lanelab level's pads with `clipPadsToBlocks` and its
  terraces with `lanelabTerraces`: footprints inset by the feather (2 m) and clipped to the block inset by
  the same, so plane plus ramp end at the block line. `padsOnPavement` checks exactly those flattens
  against conform's own grid nodes marked pavement or sidewalk, for the raw pads (the bug's size) and the
  built ones (must be 0). One derivation for loader and audit: `lotPadFlatten` in city_lots.

## The block at the end of the on-ramp, and the fast mesher (2026-09-07)

**The block** was a pier of on-ramp d0_in_on: a 2.6 m concrete column from 6.8 m to 11.7 m, topped from the
ramp's centreline PROFILE (12.66 m there) while the deck actually built at that point was 8.90 m — the
lane's partner blend had pulled it 3.8 m down toward local street c6, whose lanes were within the 20 m
blend radius. So the column stood 2.8 m proud of the surface. Found with `--probe` (own 12.66, deck 8.90,
pier z1 11.66 at one point) after the map, the lot listing and a fresh-GLB scan had ruled out buildings,
pads, places and layer triangles. Two fixes and two invariants:

- piers (and the structure test that places them) use `deckRoad`, the height the road actually gets, never
  the raw profile — `piers stay under their decks` (241 metro piers, all under the slab);
- the partner blend fades to nothing between `same_level_dz` and `bridge_h` of vertical separation, but
  keeps full pull within 3 m of the seam (a slip lane two metres off its cross street on the hill must still
  land on it exactly: the first version without the seam rule opened a crack there) — `decks follow their
  profiles` flags any lane pulled more than 2.5 m from its own profile.

That second invariant now names the real defect underneath: the importer routed d0_in_on (from c8 to the
freeway) straight down c6's corridor for ~60 m at 1–4 m above it, so their footprints overlap; the blend
had been welding the ramp onto the street (a hidden 3.8 m dip), and without it the ramp's edge stands a
level above c6's lanes (`driving surface` lists ~40 step edges there; metro cracks 34 → 25 otherwise). That
is the ramp planner's job — a ramp must not share a street's corridor unless it becomes that street.

**The mesher.** `buildSurfaces` was 89 % of a metro level start (168–183 s of 190). It is now parallel over
all hardware threads (Glenn signed off: "make buildSurfaces fast"): a coarse grid over lane bounding boxes
replaces the per-triangle scan of every lane, the cover pass runs in contiguous chunks whose results are
concatenated in chunk order (byte-identical output: same 514 461 triangles, same deck), and the deck heights
of every distinct (vertex, owner) pair are computed in parallel before the sequential weld, which assigns
ids in first-use order as before. Metro: `cdt 0.5 s, cover 12.7 s, heights 0.3 s (24 threads), weld 0.2 s,
decks 0.9 s` — surfaces 168 → 14.7 s, the lanelab build 190 → 36 s. `LANELAB_THREADS` overrides the count.
Next on that clock: profiles 13.5 s (twelve agreement rounds each re-finding every crossing) and the cover
pass's `H.own` projection onto long polylines; a build cache would take the rest to seconds.


## The level bundle: build once, load from disk (2026-09-07, ADR-0084)

A lanelab level no longer rebuilds its city on every start. `engine::bundle` (`src/engine/bundle/`) is a
general one-file cache — `cache/levels/<key>/level.bundle` + `manifest.json` — of named, 64-byte-aligned,
memory-mapped sections; the lab is its first **producer** (`city_producer.{h,cpp}`): per lanelab entity it
writes `city/e<n>/cell/<cx>_<cz>/mesh/<material>` (float32 packed meshes split by the level's render cell),
`city/e<n>/ground`, `roads/{twin,nav,row}`, `blocks/holes` (un-inset; the sidewalk inset happens at load),
`cells`, `report`. The loader (`loadLaneLabEntity`) obtains the level's bundle once — from disk when the key
matches, else baked now — and **always instantiates from bundle sections**, so a cold level and a cached one
are the same geometry to the byte.

- `rt_bake assets/lanelab/levels/ring.json [--out <root>] [--glb] [--glb-split cell] [--force] [--threads N] [--check]`
  bakes from the command line (run from the repo root); `rt_bake --inspect <dir>` prints a bundle's identity,
  producers, inputs and largest sections. The editor's **Bake level cache** button runs the same bake on a
  worker thread with a status-bar progress bar and the Console raised, then reloads.
- Key = graph JSON bytes + terrain grid file bytes + `citysim.renderCell` + `sizeof(Real)` + format +
  `kLanelabBuildTag` (`city_producer.cpp`). **Bump the tag whenever the lab's output changes.**
- `RT_NOCACHE=1` builds in memory and never touches disk; `RT_BUNDLE_WRITE=0` reads but does not write;
  `RT_BUNDLE_REQUIRE=1` refuses to build; `RT_BUNDLE_DIR` moves the root. The `bundle?` control verb reports
  the last hit/miss.
- Blender: `--glb` writes `city.glb` (one object per material, tangents and vertex colours kept),
  `--glb-split cell` writes `cells/<cx>_<cz>.glb` + `index.json` (one tile per 250 m cell; multi-select in
  File → Import → glTF 2.0). Scene extras carry the manifest, the road twin as JSON and the pavement holes.
- The invariant sweep is not part of a bake (it costs minutes on metro): `rt_bake --check` or
  `lanelab_tool build` when you want it.
- Baking metro showed where the rest of a level start went: `buildMeshes` was 143 s because three point
  queries in `deck_mesh.cpp` (the seam test behind the parapets and slab sides, the junction-box test, the
  paint classifier) scanned all 746 lanes per query and recomputed a footprint's bounding box from its
  vertices on every call. They now use `laneBoxes()` once and the `LaneGrid` the cover pass already had
  (`pavement.h`), candidates in ascending lane order so the answers are unchanged — ring bundle sections
  byte-identical before and after, the ring bake 9.5 → 5.0 s; metro's mesh stage 143 → 110 s. What is left
  of that 110 s is `H.deck` — the partner blend — evaluated per candidate in the seam test, the junction
  boxes and the paint classifier; per-lane and parallelisable, and the next thing to take off a cold bake
  (a cold metro bake is 154 s: build 42, meshes 110, pack + write 2). A warm metro load reads its city
  products in 0.36 s and instantiates them in 0.16 s.

### The lot pass as the second producer (2026-09-07, milestone B)

- `lots` (`lots_producer.{h,cpp}`) grows the last lanelab entity's blocks — the pavement holes inset by the
  level's sidewalk, on the lab's conformed grid — with the loader's own parameters (`engine/lot_grow_setup.*`
  is the one place they are derived now; `growCityLots` calls it too) and writes the `NetLotResult` under
  `lots/` (`procgen/city/lot_cache.*`: lots, plan, terraces, `parts/<PartId>`, `flat/<PartId>`).
- It reads the city's sections **out of the bundle being written** (`BundleWriter::readBack`): built this
  run or copied forward, the same bytes. No producer-to-producer side channel.
- Key: city key + `citysim` + the spawn (the enterable building) + `style_book.lua`/`archetype_book.lua`
  bytes + `kLotsBuildTag` (bump it when the grow's output changes). Applies to lab levels only (no `terrain`
  block, no shape:"road" entities).
- Loader: registers both producers before the first obtain; the lots come out of the city's bundle when its
  manifest carries the current lots key, else `obtainForLevel(inputs, "lots")`, else an in-place grow.
- Parts are stored **per render cell** (`lots/cell/<cx>_<cz>/parts/<i>`, lots format 2): the loader
  instantiates one section per Renderable and never chunks or materialises whole parts at load.
  `RT_LEVELS=lanelab_metro ./build-viewer/level_tests` times one level; the loader logs `[lots] load tail`.
- Numbers: a warm metro level loads in 1.2 s end to end (roads 0.4 s, 1433 cell parts 0.58 s), ring in 0.4 s.
  Before per-cell parts it was 3.4 s: lots read 1.07 s + 1.62 s of chunking at load.
  Ring lots 24 blocks → 385 lots, 382 buildings, 1.6 M part triangles, grown in 1.3 s, read back in
  0.11 s; metro 105 blocks → 1739 lots, 1741 buildings, 5.9 M part triangles, grown in 4.8 s, read in 0.96 s.
  Bundles: ring 256 MB, metro 1.04 GB — the packed part meshes are the bulk (metro `lots/` ≈ 730 MB, one
  wall part alone 83 MB on ring), because the grammar's parts are unwelded triangle soup. Welding or a
  per-cell part split is the obvious size lever; the read is mmap'd, so size costs disk, not load time.
- `rt_bake --prune [--yes] [--out <root>] [level.json ...]` lists the bundles under the root and marks stale every
  one that is not the newest for its level path (and not the current bundle of a listed level); `--yes`
  deletes them. A directory without a manifest is stale; `*.tmp-<pid>` directories are never touched.
- Tests: `lanelab_lots_producer_identity_follows_its_inputs_and_the_city_key`,
  `lanelab_lots_bake_reads_the_city_products_back_and_is_deterministic` (two grows byte-identical; an in-memory
  bake of city + lots reads back; the manifest key matches the identity).

## The chunk taken out of a bend (2026-09-07, importer)

Glenn stood at a metro street corner where two local streets met at 64° with crosswalks, stop bars and a
junction box whose outer edge was cut flat — "what should be a bend in the road has a chunk taken out of
it". The lab was right about what it was given: two edges ending at one point with nothing else there.
The source city (`LANELAB_IMPORT_PROBE=x,y` prints the source graph and spine ends around a point) had a
4-way crossing there, on the city's ORIGINAL perimeter. The importer redraws the perimeter as the freeway
alignment at the 220 m design radius, which fills concave notches of the outline; every perimeter chain
was then discarded, including the two that ran through this notch hundreds of metres from where the
freeway ended up. Two fixes in `level_import.cpp`:

- **Perimeter chains inside the band are streets.** The clipping loop no longer exempts loop chains: a
  perimeter chain with a run inside the frontage band keeps it (ended as a T on the frontage road like any
  other street); one the freeway actually replaces has no inside run and goes. Metro: 6 kept.
- **Two arms of one class at a dead point are one street that bends.** Wherever exactly two chain ends met
  (the third arm consumed by the redraw, a dropped stub, a landing), same-class arms are joined into one
  chain and the joint rounded (quadratic fillet, tangent length R·tan(turn/2), R 12/20/30 m for
  local/collector/arterial, never more than 45 % of the shorter leg). A class change stays a two-arm point
  and is reported as such (`N two-arm dead points: c…/c… at x,y (deg between the arms)`). Metro: 9 joined,
  0 left. Landings are never joined (their ids carry the ramps).

The regenerated `assets/lanelab/metro_v2/metropolis_sky_lanelab.json`: 189 → 186 edges, 746 → 742 lanes,
110 → 115 pavement holes, deck cracks 25 → 2, non-manifold 36 → 0. The corner is a 4-way crossing again
(four twin arms, one junction node). Open on the lab side: a degree-2 node should still not get
crosswalks — `inBox` calls any overlap with a tangent difference over ~45° a junction — so an authored
graph with a sharp two-edge bend would show the old picture; the importer no longer produces one.

## Band ramps: a diamond's ramps end on the first road they meet (2026-09-07 night)

Glenn, after the ramp over c6: the on-ramps should not cut through several city blocks; they should descend
to the city road level and intersect with the first road they meet. (A first prototype removed the landings
altogether and put slip-ramp terminals on the frontage road; that dropped the whole ring to ground level
and, worse, exposed a hole bug — a ground-level freeway ring whose frontage road never touches it has one
pavement hole, the whole city, and the parceller filled it with lots. Both fixed below; the diamonds stayed.)

The diamond keeps its landing street, underpass and elevated freeway. Only the four ramps changed
(`level_import.cpp`, the diamond emission):

- **Where a ramp ends.** Inside the ring the first road is the frontage collector: the inner off-ramp ends as
  a T on it 70 m before the landing junction (the first of 70…250 m that is 45 m clear of any street T), the
  inner on-ramp starts as a T 70 m after it. Outside, the first road is the landing street itself: both outer
  ramps meet it at ONE point 45 m beyond the freeway's edge, from opposite sides — a four-way ramp terminal
  (two Ts 40 m apart read as a stutter). Nothing is dovetailed onto a street any more; the lab takes a ramp
  with one anchor and a free end (it always did).
- **Where a ramp runs.** In the band beside its carriageway, following the loop's curvature — a polyline, not
  a bezier: a chord between points 600 m apart on the ring cut 100 m through the blocks and the carriageway.
  The spine eases from the gore lane's radial to the band centre (6.7 m off the carriageway's edge) over 80 m,
  runs parallel, and ends on a cubic into the T whose arrival direction depends on the road met: INWARD for
  the frontage road (tangential — met squarely by turning), ALONG THE LOOP for the landing street (radial —
  met squarely by continuing, shifted sideways; the quarter circle I tried first hooked the ramp away from
  the freeway and jogged back). The frontage setback is 20 m now (was 14) so a lane fits in the band.
- **How long.** As long as its climb needs: the lab's design grade for ramps (g_max / 1.5) when that lands a
  ramp under 600 m, else 7.5 % — a ramp beside a freeway climbing away at 4.8 % never catches it at 5.3 %.
  The gore height depends on where the gore lands, so the run is a fixed point (iterated to 0.5 m; a diamond is
  skipped when the freeway outruns its ramps). Two diamonds' band runs never overlap (10 m apart; their gore
  runs may meet — that is a weave lane), and gores stay clear of the arc joints by their taper runs.
- Metro: 3 diamonds (loop stations 395, 1666, 4443; 2377 skipped — its ramps would run into d0's), 12 ramps
  of 150–677 m at 1.0–7.4 %, every free length OK, deck cracks 3 (0.0 m), non-manifold 0. Blocks: 100 of 100
  holes, 0 m² on pavement, 4 strip holes dropped. Bake 151 s → `cache/levels/a143461d265af466`.

Pavement-hole fixes that came out of the prototype (`block_audit.cpp`; `kLanelabBuildTag` and `kLotsBuildTag`
→ 2026-09-07.2): `pavementHoles` takes the buildable space as the union of the pavement's holes MINUS any
pavement inside them and drops hole-shaped pieces (an annulus is verge); `blocksFromHoles` drops any inset
piece that vanishes under a further 3 m inset (a strip under 2·(sidewalk+3) m wide — a band, a median).

**Gores (2026-09-07, "missing segments on the freeway").** The wedge between a ramp curving away and the road
it left is a hole in the paved surface. `buildSurfaces` turned a hole under 2000 m² into a kerbed grass island
only when it was THIN (a 4 m opening erases it); a hole with a body was assumed to be a small city block and
left alone — which showed bare ground in the middle of the pavement. Metro had three, ~1700 m² each, all at
the new band ramps. A hole with a body is still land, EXCEPT inside a freeway or ramp corridor (nearest edge
is a ramp or rank >= 3), where there is no land: it is a gore, and gets the kerb + grass slab at deck height.
`lanelab_tool build <graph> --quick --holes` lists every hole under 2000 m², what covers it, and the edge it
lies against: metro is 25 holes, 25 covered, 0 bare. `kLanelabBuildTag` -> 2026-09-07.3.

**Where a barrier lands (2026-09-07, "those walls don't seem to belong to any of the carriageways").** The
offset assumed the class's shoulder was really beside the deck. Often it is not: a crossing road eats it, a
gore ends it, a viaduct edge never had one. The wall then stood on bare ground beside the deck (456 sampled
segments on metro) or across a travel lane (148). The placement now probes the barrier's OWN position instead
of a point 0.35 m out: same-level travel lane there means the edge is an internal boundary inside a road
corridor and gets no wall at all; nothing solid there pulls the barrier in against the deck edge (offset 0.05,
the classic bridge parapet); otherwise it stays at the shoulder's outer edge. Metro after: deck edge 456,
shoulder 7835, median 460, lane 0, bare ground 0. `--walls` prints exactly that census.

**Why grass spikes through the carriageway (2026-09-08, diagnosed, not yet fixed).** The lab meshes its
terrain as a COMPLETE grid — `buildMeshes` walks every node of `r.terrain` and calls `gridIndices` over the
whole thing — so there is terrain under every square metre of pavement. Anywhere a conformed node ends up
above the slab that covers it, its triangle pokes through the road. That is why clamping keeps reducing the
count without ever removing the artefact: `conformGrid` now clamps under lane decks (reaching past the
shoulder) and under road envelopes, and the "no terrain above any deck" invariant is down from 1743 samples
/ 2.79 m to 387 / 0.52 m, yet wedges remain. They are above the SHOULDER and MEDIAN slabs, which that
invariant never samples — it walks lanes.
The durable fix is to stop drawing terrain where pavement covers it: skip a grid quad whose four corners are
all inside surface ∪ shoulder ∪ sidewalk ∪ median, keeping partially covered cells so the joint still meets.
Then no clamp is needed for appearance and z-fighting goes with it. The cost to watch is the coverage test:
`contains()` per node over the layer PolySets is too slow at 200k nodes, so it wants the lane grid for the
carriageway and a bounded test for the layers.

## CDLOD renders the lab's ground (2026-09-08, wired, not yet enabled)

The lab bakes the source level's terrain to a 5 m grid, conforms it to the roads, and then meshed that grid
itself: no LOD, no morphing, no material blending, 399,648 triangles of it, and cut faces faceted by the grid.
CDLOD builds its nodes from `TerrainParams` + `Noise`, so it cannot take a height callback — but
`terrainHeight` reads `params.erodedBase` INSTEAD of the analytic relief, and that is a `std::function`.
`level_loader.cpp` now points it at the conformed grid when a level has BOTH a lanelab entity and a terrain
block: the city bundle is obtained early (the same one `loadLaneLabEntity` reuses), its `city/e0/ground`
section becomes the sampler, and outside the grid it falls back to the level's own terrain, blended over the
last 60 m so the edge value cannot smear across the world. The lab then skips meshing its own ground.

It renders correctly and looks like the real thing — rock and snow materials, morphing, smooth hillsides
where the 5 m facets used to be. It is NOT enabled: adding the terrain block to `lanelab_metro.json` makes
`level_census_every_floorplan_conforms_to_the_drawn_ground` fail with 706 burials of 1284 lots, worst 4.28 m.
That is a real defect, not a harness artefact — the census reads the live `TerrainLodConfig`, so it sees the
same sampler the lot pass used. The cause is ordering: the lot pass levels a graded PAD under each building
and the host is supposed to stamp those `gradeFlatten` records into the terrain, but that only happens for
`preLots` (the terrain pre-pass), and a lanelab level's lots are grown long after the terrain exists. With no
terrain the mismatch was invisible; with one, every building on a slope sits below the drawn ground. Fixing
it means growing the lab's lots in the pre-pass, or stamping their grades and rebuilding the terrain after.
The parked terrain block is metro_v2's with the erosion keys stripped (the grid already carries the eroded
shape).

**Road surfaces (2026-09-08).** The lab's roads were flat colour because the loader set albedo and
roughness and never asked for a surface. `surfaceForLaneLabMaterial` in `level_loader.cpp` maps the bundle's
material names to the engine's ANALYTIC surfaces (renderer.h: grain computed in the shader from the
world-planar UV, no maps to bake, nothing to author): asphalt and shoulder -> `Asphalt`, concrete ->
`Concrete`, sidewalk -> `Pavement`, terrain -> `TerrainGround` (micro-relief over the baked grass colour).
Median grass slabs and the two paint meshes keep `None`. This is a LOAD-time change, not a producer one, so
no tag bump and no rebake — only the binaries that instantiate need rebuilding.

Colours moved with the surfaces, because grain is invisible on a near-black albedo: `kAsphalt` 0.045 ->
0.085 (real asphalt is 0.08-0.12), `kShoulder` to match, and `kConcrete` 0.55 -> 0.80 so a barrier or pier
reads as eggshell white and matte. A GUARDRAIL is now its own material rather than sharing the concrete
mesh — it could not differ from a wall otherwise — with `kGuardrail` light cool steel, roughness 0.42 from
the producer, `Surface::CorrugatedMetal` (a W-beam is corrugated) and `metallic` 0.18 in the loader. A first
pass at 0.65 metallic rendered the rails BLACK: a metal with no environment to reflect has nothing to be.

Lane markings are still geometry (paint_white 70806 tris, paint_yellow 22222, 3.4 % of metro's 2.7 M). The
engine already has `Surface::RoadMarkings`, analytic, used by its own roads via `net.look.markings`: it paints
the double yellow, the white edges, dashed dividers and zebra bars from a road-local UV (carriageway lat
normalised to [-1,1] as mu-2, mv = arc length, metres past a junction mouth near one). Moving the lab onto it
needs the deck UV rewritten in that convention — today it is lateral METRES from the owning LANE, and the
shader wants it normalised across the parent ROAD — and it needs the shader to learn about one-way
carriageways, or every ramp gets a yellow centreline down the middle. Discrete symbols (turn arrows, stop
bars, legends) are not a function of lateral position and belong in decals or small quads the lab places from
its own turn-pocket and junction-box knowledge, not in that shader.

**Barriers are built like sidewalks (2026-09-08, Glenn: "it's like sidewalks but they're walls").** A
sidewalk is a ribbon offset from the road, unioned and clipped by Clipper, which is why it never fragments
and never twists. Barriers were assembled from the deck's boundary TRIANGLE EDGES chained by vertex index:
any failed test ended a chain (775 runs over 25 km) and every sharp joint had to be mitred by hand. Now
`parapetRuns` offsets the union of freeway and ramp footprints (`offsetSet`, which is Clipper's inflate with
round joins — the Minkowski sum with a disc, the same primitive as `closing`), takes the resulting polygon's
boundary rings as ordered curves, resamples them at 1 m and walks them. Per point: the owning deck lane from
the lane grid, its deck height, the `EdgeRole`, and so the `BarrierSpec`. The ring is split into arcs of one
spec and each is swept with the profile it asks for. Metro: 319 runs instead of 775, no folds, 98 % of
26.5 km of outline carried.
Two rules the outline needs that an edge walk did not. A MOUTH is not an edge: the outline wraps around the
end of a ramp where it meets a street, and a barrier there seals the exit, so a point standing on or beside
pavement that is not ours builds nothing. And a taper is only for a TERMINAL — an arc that stops because the
next stretch wants no barrier — not for a change of barrier kind, or every transition grows a wedge.
`BarrierSpec` now carries `offset` and `thick` beside `kind` and `h`, all authored per class per role, so how
far out a fence stands is class data rather than a constant: `"edges": { "vs_street": {"kind": "wall",
"h": 2.5, "offset": 2.45} }`. Offset 0 hugs the pavement edge, which is the only place the lab can guarantee
ground under it — pushing it to the class shoulder aimed at a surface `layerMeshes` drops in gores, which is
why rails stood in the grass.

**The edge grammar, on (2026-09-08).** `RoadClassSpec::edges` maps an `EdgeRole` to a `BarrierSpec`
({kind: none|wall|guardrail, h}), authored per class in the graph JSON under `"edges"`, e.g.
`"freeway": { "edges": { "at_grade": {"kind": "guardrail", "h": 0.75} } }`. `set` on the spec distinguishes
an authored "none" from silence, so a class can switch a default off. Defaults for freeway and ramp only:
median a 1.05 m wall whether elevated or not, city-facing a 2.5 m wall, elevated the 0.9 m parapet, at grade
a guardrail. `sweepGuardrail` sweeps a beam (through `sweepWall`, so the mitring is shared) and drops a post
every 4 m; `miterOffset` gives the post line the same bisector-scaled normal, because offsetting each point
by its own segment normal made the rail zigzag round the ring. A run breaks where the barrier kind changes,
and such an end is NOT tapered — only an end that would leave a face on pavement is. Metro edge coverage went
33 % -> 98 %, tapered ends 223 -> 57, still lane 0 / bare ground 0.

**Edge roles: the first half of an edge grammar (2026-09-08).** One geometric rule built one product — a
0.9 m parapet wherever a freeway or ramp deck stood 1.5 m clear of the ground — so streets got nothing (right,
they are the lot pass's business) and an at-grade freeway beside a frontage road also got nothing (wrong).
`EdgeRole` (deck_mesh.h) names what a freeway/ramp deck edge FACES, from data the mesher already had: `Seam`
(same-level pavement 0.35 m out: not an edge of the structure), `Median` (freeway on BOTH sides across the
median; a ramp beside the carriageway it left is a gore, not a median), `VsStreet` (an arterial, collector,
local or alley within 32 m), `Elevated`, `AtGrade`. `parapetRuns(r, &census)` fills a `ParapetCensus` of
metres per role and how much of each carries a barrier today, printed by `lanelab_tool build … --walls`:

| role | metro | walled | ring | walled |
|---|---|---|---|---|
| median | 5861 m | 52 % | 4398 m | 82 % |
| vs street | 1397 m | 0 % | 286 m | 0 % |
| elevated | 5903 m | 97 % | 4584 m | 98 % |
| at grade | 12997 m | 0 % | 1842 m | 0 % |
| all | 26280 m | 33 % | 11217 m | 72 % |

So the rule works exactly where it applies (elevated, ~98 %) and the two thirds of freeway edge it does not
reach are the interesting ones: 13 km of at-grade edge with nothing, half the median unwalled because it is
at grade, and 1.4 km facing the city. Next: an `edges` map on the class spec ({role -> kind, height}) with
defaults reproducing today's output, then guardrail and noise-wall sweeps to turn the new roles on.

**Barrier runs, not fragments (2026-09-08, "the walls now randomly taper instead of being consistently
straight").** Tapering EVERY open end turned a continuous barrier into a row of wedges, because eligibility
flips off for a few edges at a time — a seam, an edge whose deck dips under 1.5 m, a fan of triangles at a
gore — and each break started a new run with a terminal at both sides of it. Two changes: `parapetRuns` now
rejoins an end to a start within 6 m that carries on in the same direction (dot >= 0.7), refusing any join
whose gap would put a barrier on drivable road; and the taper is applied PER END, only where `capOnPavement`
says a face would otherwise stand on pavement, over 8 m rather than 12. Metro: 195 runs with 234 tapered ends
-> 124 runs with 150, still lane 0 / bare ground 0.

**Barrier ends (2026-09-07, "the freeway walls end up blocking the car").** A parapet goes on a deck boundary
edge that is elevated 1.5 m or more; where a viaduct descends, that test stops being true mid-carriageway and
the run ended in a full-height 0.9 m vertical face standing on the shoulder — a block to drive into, not a
deck edge. `sweepWall` now takes a `taper` (12 m for parapets): an OPEN run ramps its height to nothing over
the last 12 m at each end (or 40 % of a short run), and the end caps are dropped once the height reaches zero,
so a barrier ends the way a real one does. Closed loops are untouched. The two walls a driver sees between the
carriageways are NOT meant to meet: each belongs to its own deck edge across the median, and they end at
different stations because each carriageway crosses 1.5 m somewhere else.
`parapetRuns()` (deck_mesh.h) is now the single source of the runs the mesher sweeps, and
`lanelab_tool build <graph> --quick --walls` prints them with the ends that stand on same-level pavement:
metro has 171 runs, 8717 m, 217 such ends. `kLanelabBuildTag` -> 2026-09-07.4.

**Ramp carriageway (2026-09-08).** A ramp was a freeway LANE: `freewayLaneW` (3.6 m) with a 1.0 m shoulder,
which drives narrow — one lane, a barrier either side and no room to correct. It is its own road now:
`ImportOptions::rampLaneW` 4.5 m and `rampShoulder` 2.5 m (the mainline's), also on the CLI as
`--ramp-width` / `--ramp-shoulder`. The wider shoulder moves the parapet out by the same amount. Nothing else
in the ramp pipeline cares: the anchor taper is computed from the HOST's lane width, and metro rebuilds with
the same 12 ramps, lengths, grades and free lengths, cracks 2 / non-manifold 0. A two-lane ramp (`fwd: 2`)
also builds and puts both lanes outboard of the host's outer lane, but the lab has no 2-to-1 taper at the
merge, so it would merge two lanes at once — left alone.

## Earthworks that daylight (2026-09-08, "deformed terrain interfering with the road")

Driving the metro off-ramp on a hillside, the ground stood in sharp cones at the kerb and half-buried the
car. Reading the conformed grid out of the bundle beside the natural one showed why. `conformGrid` gives each
node to its NEAREST road within `conformW` + half width (~17 m) and grades it; every node beyond that keeps
its natural height. On a cross slope with two carriageways 5 m apart in level, that left bands cut to 46 m
and 41 m with untouched 49-53 m cells standing between them. Two passes were added at the end of the conform:

- **Daylight the cut.** A real earthwork runs its batter until it meets the ground. The batter cone is
  propagated outward from every conformed node by two 8-neighbour chamfer sweeps, and the natural surface is
  clamped under it: no node may stand higher than the nearest conformed node plus `slope` x distance.
  Self-limiting — far from any road the cone passes over the terrain and nothing changes. CUT ONLY: the same
  cone downward would raise the ground under a viaduct into an embankment, and a road on piers is not one.
- **No terrain above a deck.** Ownership by nearest road is what puts ground from the higher road over the
  lower road's kerb. Every node within a lane's half width + 2.5 m is now clamped under that lane's deck,
  lanes on structure excepted (as in the ownership rule). The lab's own invariant scores it: 1743 samples
  above a deck by up to 2.79 m before, 633 by up to 0.60 m after.

`kLanelabBuildTag` -> 2026-09-08.2. Note the terrain a lanelab level renders is NOT the CDLOD terrain: the
importer bakes the level's terrain function (erosion included) to a flat grid at `terrainRes` (5 m) beside the
graph, and the lab conforms and meshes that. metro_v2's CDLOD leaf is ~2.7 m, so the lab's ground is coarser
and has no LOD or material blending.

Open after this: two-way frontage roads mean ramp terminals are Ts, not merges (one-way frontage roads, the
Texas pattern, would allow slip merges); the 170 unwalled internal seams the parapet pass reports (104
before) want a look; the block audit lists 6 thin "broken" blocks along the moved frontage road.
