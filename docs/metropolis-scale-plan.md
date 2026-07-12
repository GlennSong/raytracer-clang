# Metropolis Scale Plan — streaming, LOD, freeway ribbons, conform, baking

Device findings (2026-07, first real playtest of `metropolis.json`, 2km city):
2–8 fps with the whole city on screen, ~108M triangles drawn at worst, slow
editor loads, freeway "blobs", road-terrain conform failures, buildings without
foundations. The bedrock placement bug (recipe ran twice against different
ground) and the 500k-tri scatter tree are FIXED; this plan covers the systems
that remain. Each phase has a visual/measured acceptance gate — nothing counts
as done until it's verified on device (frame dump or playtest).

Related: road-network-execution-plan.md (hierarchy phases), unified-road-plan.md
(weld engine), open-world-foundations-plan.md (HLOD nesting), ADR-0075 (grading
cascade), TECH_DEBT.md.

## 1. City rendering at scale: chunk → cull → LOD → stream

Today: `growLotBuildingsOnNets` merges ALL building geometry per PartId
(`NetLotResult::parts`, ~10 meshes for the whole 2km city). One frustum test
per mesh means the whole city passes whenever any of it is visible. Vegetation
is already chunked per 280m cell (level_loader `veg.cullCell`) — buildings and
roads follow the same shape.

- **P1.1 Building chunk grid.** Split lot geometry per (cell, PartId) at grow
  time — `growLotBuildingsOnNets` gains a `cellSize` (default ~250m, 0 = old
  behaviour); the loader spawns one Renderable per chunk with real bounds.
  Existing per-mesh frustum culling then works unchanged. Gate: frame dump at
  street level shows Draws down and triangles < 25% of today's worst case
  with no visual change.
- **P1.2 Distance policy.** Per-chunk draw distance (near: full detail;
  mid: chunk still drawn — buildings are cheap boxes at distance thanks to
  vertex-colour materials; far: swap to the existing `hlodProxy` mass-box mesh
  per chunk instead of the whole-city proxy). Requires baking the HLOD proxy
  per cell (same splitter as P1.1). Gate: 30+ fps from the 300m diagnostic
  aerial; no visible pop at the swap distance with fog masking.
- **P1.3 Road mesh chunking.** The welded deck is one giant mesh; split the
  weld output by the same cell grid (post-weld triangle binning is fine — the
  weld itself stays global so seams stay watertight). Gate: overhead map view
  draws fewer road triangles than today by >50%.
- **P1.4 Texture mipmaps.** Baked PBR sets (surface_maps) upload without mips;
  generate mip chains at upload (MTLTexture mipmapped + blit
  generateMipmaps) and switch samplers to trilinear. Gate: no shimmer on
  facades in motion at distance; bandwidth-bound scenes speed up.
- **P1.5 Streaming (GTA-style, later).** Once chunks exist, streaming is a
  residency policy on top: keep GPU buffers for chunks within R, release
  beyond R+hysteresis, load on approach (background thread; chunks are
  deterministic re-bakes or cache reads — see §4). Design questions: budget
  (bytes, not chunk count), fade-in, and whether colliders stream with render
  chunks (they should; Jolt bodies per chunk). Not started until P1.1–P1.3
  land and we re-measure — chunk culling alone may buy 60 fps at this scale.

## 2. Freeway as a single ribbon with explicit ramps

Device: freeway link polylines + colonization merges + block subdivision create
blobby self-intersecting areas near freeways. The fix is discipline, not more
patching: a freeway is ONE spline ribbon; nothing else touches it except at
authored connection points.

- **P2.1 Ribbon isolation.** Freeway links become their own RoadNet entity
  (own weld pass, no shared planarize with the street grid). Street-grid
  crossings become grade-separated by default: `layer=1` on the freeway
  (ADR-0051 already lifts bridges); no at-grade freeway junctions at all.
- **P2.2 Interchanges.** At each `interchangeSpacing` station: a diamond — two
  short Ramp-class slip roads (DesignRules::Ramp) connecting to the nearest
  arterial node within reach; skip if none within ~250m. The colonization
  seeds stay (arterials still grow from interchange neighbourhoods) but tips
  NEVER merge into freeway nodes (drop the merge exception; freeways aren't in
  the colonization node set — already true — and the seed points move to the
  ramp terminals instead of the freeway line).
- **P2.3 No frontage growth.** Faces adjacent to the freeway ribbon get an
  inset greenway margin (no edge-blocks along freeways, no lots fronting
  them) — buildings front the parallel arterial instead, like real cities.
- Gate: overhead map shows freeways as continuous clean ribbons with diamond
  ramps; zero self-intersecting deck geometry along them; drive one end to the
  other on-device.

## 3. Road–terrain conform, evidence first

Device: "roads are not conforming to terrain in many many places — complete
failure." The corridor carve exists (ADR-0044/0075) but hasn't been audited on
the eroded 2.6km terrain with the CDLOD sampler.

- **P3.1 Evidence pass.** A probe that walks every chain and reports
  max |deck − terrain| mid-corridor and at aprons (the poke-probe from the
  road study, rerun on metropolis) + frame dumps at the 10 worst sites.
  Hypotheses to check: CDLOD morph vs flatten-dilate mismatch at coarse LODs;
  smoothstep feather too narrow on steep eroded slopes; junction lowest-plane
  rule fighting the chain profile; erosion detail re-emerging inside corridors
  (erodedBase sampled with flatten applied after — verify order).
- **P3.1 RESULTS (2026-07-10, road_poke_probe metropolis mode, dilate 1.5):**
  deck verts 147,741. Poke-through is the MINOR failure (0.08% CDLOD-sampled,
  worst 0.66m; 0.63% vs exact carve, worst 2.0m). The DOMINANT failure is
  PROUD APRONS: 12,457 verts (8.4%) stand >1m above the sampled terrain
  (26,801 > 0.7m; worst 2.64m @ (2,-390)) — the "road on a plinth" cliff seen
  on device. Junction interiors poke 5.7x worse than mid-span (2.32% vs
  0.41%). The conform corridor already spans halfWidth + sidewalk + 2m, so
  the proud verts are a VERTICAL disagreement between the weld's chain
  profile and the conform's profile (they are built by parallel code paths
  that are supposed to match — road_net.cpp weld vs roadNetConformRegions),
  worst where the junction lowest-plane rule overrides one but not the other.
  P3.2 should unify them: ONE profile source (the weld's, post-junction-
  resolution) feeding both the mesh and the flatten planes.
- **P3.2 ROUND 1 (2026-07-10):** the mid-span overlap reconciliation moved INTO
  weldChainProfiles (overlapReach param) so decks agree with each other and the
  carve by construction; approaches ease back at maxGrade. Probe: >0.7m proud
  26,801 -> 14,321 (-47%), >1m 12,457 -> 5,822, CDLOD poke steady 0.09%. The
  RESIDUE clusters where junction-pad geometry (corner fillets, plaza rings)
  bulges outside every chain corridor — next: probe dumps worst-proud vertex
  locations bucketed by pad-vs-chain, then either widen chain coverage to the
  pad ring or emit pad-specific flatten footprints.
- **P3.2 ROUND 2 (2026-07-10):** forensics narrowed it decisively. The probe now
  classifies >1m-proud verts: ALL are INSIDE a carve corridor (zero coverage
  holes), and the clustered worst sites all sit 7-28 m from a junction (worst
  3.17 m @ (-24,-390)). Endpoint mean->min and self-overlap reconciliation were
  both ~inert (5,822 -> 5,807), which rules out chain-profile rules entirely:
  the offending mesh heights come from JUNCTION PAD / TURNING-DISC surfaces —
  flat pads at one plane over per-chain carve ramps (a 15 m pad on a 15% slope
  stands ~2.3 m proud at its downhill edge). NEXT: export the weld's pad/disc
  polygons as their own TerrainFlatten pads at the pad's plane (the
  "pad-specific flatten footprints" option), then re-run the probe.
- **P3.2 ROUND 3 (2026-07-10):** junction pad discs now emit their own tilted
  flatten ramps (roadNetConformRegions samples the reconciled deck plane per
  pad). CDLOD poke HALVED (0.09% -> 0.04%; exact 0.67% -> 0.56%) — the
  terrain-slicing-through-junction symptom visibly cleaned up at the evidence
  site. The >1m proud metric is FLAT (~5.8-6.0k) and its worst sites are
  byte-identical across every flatten change tried — those verts' ground is
  NOT owned by any region we've touched. Next: per-site introspection (dump
  which flatten region + which spine owns the deck vert at (-24,-390)) before
  any more fixes.
- **P3.2 ROUND 4 (2026-07-10):** two orthogonal poke sources fixed on-device.
  (a) CDLOD MORPH: coarse-LOD morph targets are blind neighbour averages, so
  flatten-clamped corridor vertices lerped back UP through the asphalt at
  distance — corridor-owned vertices (flattenCovers) no longer morph.
  (b) LOT PADS: padMeshFor triangulated corner vertices only, so big green/park
  lots were flat sheets BRIDGING over road cuts — pads now drape-subdivide
  (~7 m edges) and park height dropped under the deck lift, so pad edges tuck
  below the asphalt. REMAINING: darker olive tongues over decks at the
  evidence site are TERRAIN mesh at fine LOD (not pads, not morph). The probe
  approximates the device flatten but the loader's set also contains BUILDING
  pads + city/script flatten with their own priorities — next tool: per-site
  introspection that rebuilds the LOADER's exact flatten set and prints which
  region owns the ground at a tongue point vs the deck's owning spine.
- **P3.2 ROUND 5 (2026-07-10) — THE HONEST INSTRUMENT.** The headless probe was
  structurally blind: it sampled ground AT its test points (with a dilation
  knob) while the device interpolates BETWEEN grid corners, and it point-
  sampled 5 lateral lines while pokes live between them (user's call). New
  RT_POKE_REPORT=1 in the loader: runs at level load with the FINAL flatten
  set, the exact per-LOD grid formula, deck heights from the reconciled chain
  profiles, and a DENSE map over the whole deck area (2 m x ~1.5 m).
  BASELINE (metropolis): LOD0 9.45% of 184,688 deck samples poke (worst
  0.94 m), LOD1 8.40%, LOD2 5.96%; worst sites cluster near (-660,-810).
  Segment end caps added to roadConformRegions (outer-bend wedge coverage) —
  re-A/B against THIS report, not the old one. ALL prior probe percentages are
  understated ~8x; the -47%/-50% relative improvements likely persist but must
  be re-verified against the dense map. Iterate fixes against RT_POKE_REPORT
  until LOD0 < 0.5%, then screenshot-verify.
- **P3.2 ROUND 6 (2026-07-10):** flattenDilate step*0.75 -> step*1.45 (full cell
  diagonal) in generateLodNodeMesh + the report. Dense map: LOD2 5.96% -> 2.18%,
  LOD1 8.40% -> 6.45% (the mid-distance shimmer), LOD0 9.45% -> 8.62% BUT its
  worst site is byte-identical (0.9432 m) across the change — instrument
  suspicion: the report decomposes chains via graphToSpines(navRoadGraph) while
  the mesher uses weldChainSpines(constrainedNetGraph) (file-static), so report
  deck heights can differ from the real deck on hills. NEXT: export
  weldChainSpines/constrainedNetGraph, make the report read the mesher's exact
  profiles, THEN trust and attack the LOD0 number.
- **P3.2 ROUND 7 (2026-07-10):** the report now reads the mesher's EXACT
  decomposition (roadNetWeldSpines/roadNetConstrainedGraph exported) — LOD0
  numbers byte-identical, so 8.6% is REAL, not instrument drift. New
  classification: every >0.3m poke is INSIDE a flatten footprint, ZERO coverage
  holes. Diagnosis: adjacent LOT/BUILDING PAD planes sit above the road deck
  (uphill lots don't grade down to their street — the disabled ADR-0075 block
  cascade), their grid corners own cells beside the corridor, and those
  triangles tilt over the sidewalk. NEXT (pick by measurement): (a) road
  footprints get priority over lot pads + corner-clamp cells straddling
  priority-1 corridors; (b) clamp pad groundY near roads to deck height +
  step (frontage grading); (c) re-enable block grading behind the dense map.
- **P3.2 ROUND 8 (2026-07-11) — AUTOPSY RESULTS.** RT_POKE_SITE per-sample dump
  at the immortal site: the terrain corners sit EXACTLY on a road conform plane
  (raw == roadPlane, covered) — but that plane is ~1 m ABOVE the deck actually
  there. Owner tags show the covering regions belong to conform-chain 232 while
  the report indexes the same physical road as chain 1289 AND finds it owns
  ZERO regions: the conform's spine list and the report's spine list DIFFER in
  content/order despite calling the same functions on the same net. Also fixed
  on principle: cached pre-pass road nets now keep their NATURAL-ground sampler
  for meshing (the mesh had been re-deriving profiles over CARVED ground —
  the recipe-ran-twice sin one level deeper). NEXT: log spine counts in
  roadNetConformRegions vs the report; find why two same-input calls disagree
  (suspect hidden state or nondeterminism in netGraph/applyConstraints);
  THEN the conform planes and deck heights unify for real.
- **P3.2 ROUND 9 (2026-07-11) — CLOSED. LOD0/1/2 = 0/184,688 POKES, pixel-
  verified at both evidence sites.** The round-7 report swap to the mesher's
  decomposition had silently not applied (fingerprints: conform 341 welded
  chains vs report 1,719 per-edge spines — one spine per edge, phantom ~1 m
  profile disagreements at every junction). With the swap actually landed the
  dense map reads ZERO at all LODs, and the previously-poking sites render
  clean. The stack that got here, all real: overlap-reconciled profiles ->
  end caps -> junction pad carve -> full-diagonal dilation -> corner clamp +
  road priority -> ONE ground source for mesh + carve profiles (cached nets
  keep the natural sampler). Remaining honest caveats: the report models LOD
  meshes per level (not the runtime morph blend between them, bounded by the
  measured levels) and profile-math decks (not literal mesh verts). Next
  device pass decides if those matter.
- **DEVICE FEEDBACK ROUND (2026-07-11, post-closure):** pad skirt normals
  oriented by lot winding (half the lots arrive CW); scatter gains per-instance
  variance + bedding (rocks: independent XZ/Y scale 0.55-2.4x/0.5-1.9x, bedded
  a third of their height; trees: extra jitter, rooted 0.35 below grade);
  road boundary skirts drop to the sampled ground + 0.5 m instead of a fixed
  0.5 m (high decks no longer show terrain underneath); LOD corner clamp
  exempts corners strictly inside a building-pad footprint whose plane sits
  above the road plane (houses stay grounded, steps meet earth) — LOD0 dense
  map still ZERO with the exemption. QUEUED: rock FORMATIONS (clustered
  outcrops, not just bigger stones); residual pokes the user still sees on
  device — collect coordinates via RT_POKE_SITE.
- **P3.2 Fix by measurement**, then re-enable the disabled pieces in order:
  retaining walls (P1b) where cut depth > threshold, block grading (P2) now
  that faces are guaranteed by the enclosed-block metro. Each behind its own
  device gate.
- Gate: probe reports zero exposed skirts above threshold; on-foot drive along
  the worst previous sites shows ground meeting deck.

## 4. Bake & cache (load time) + editor progress

Loads rebuild everything: erosion (800² grid, 500k droplets), metro recipe,
weld, lots, shape-grammar buildings. Deterministic in (params, seed) — so
cache by content hash without losing procgen: the hash IS the identity, change
a knob and it rebuilds.

- **P4.1 Erosion cache** (biggest single win, simplest): bake the eroded grid
  to `cache/terrain/<hash>.bin` (hash = terrain params block + seed +
  erosion params). Load = mmap + bilinear sampler as today. Invalidation is
  automatic via hash; `--nocache` to force.
- **P4.2 City bake (design doc first).** Two candidate shapes: (a) cache the
  RoadGraph + lots as data (fast, keeps meshes rebuildable), or (b) cache
  final meshes per chunk (fastest load, feeds §1 streaming). Leaning (a) then
  (b) as the streaming format. Must round-trip the editor's regenerate: an
  edited road invalidates its chunk hashes, not the world.
- **P4.3 Loading progress.** The loader already logs stage boundaries; surface
  them as a progress bar (stages weighted by measured time: erosion, terrain
  mesh, metro, weld, lots, buildings, veg) in both editor and viewer boot.
- Gate: warm metropolis load < 15s; cold unchanged; progress bar tracks.

## 4b. FOUND 2026-07-12: the build was -O0 all along

CMAKE_BUILD_TYPE was EMPTY -> no optimization flags. Every device number in
this plan (2-8 fps, 10-60 fps after chunking, ~3.5 min loads) was a DEBUG
build. CMakeLists now defaults to RelWithDebInfo (explicit Debug still works).
Measured: metropolis load 3.5 min -> 12 s; the erosion cache (P4.1, landed the
same day, content-hash keyed, RT_NOCACHE=1 to skip) saves ~2 s at -O2 — kept
for debug builds and bigger worlds. P1.2 HLOD and P1.5 streaming should be
re-justified against optimized numbers before building them.

## 5. Building foundations (done) + look follow-ups

Plinth 0.45m landed; evaluate on-device. If slopes still read wrong, the
architect's plinth should scale with local grade (pad drop across the
footprint), and street-side entrances want a step/stoop from the grammar.

## 6. Urban ground plan: plazas, paths, stairs, fencing ("a building without the building")

**Status 2026-07-12: P6.1 + P6.2 + P6.4 SHIPPED** (recipePlaza -> Massing::Plaza ->
sculptPlaza): paver podium on the fitted plan (PartId::Path / Pavement — no new
surface needed), pad flattens like a building (placeType "civic" dodges the
park/green flatten skip), concrete skirt, stair runs at the longest low-drop
edges, iron guard fence with mouth gaps, fountain/lamps/benches/planter-trees/
flower beds on a claim registry. Financial 3.5%% / Commercial 3%% table slices.
Device-verified overhead + street level. P6.3 (paths as mini-roads) still open.

Device direction: skyscraper and beachfront lots need designed OPEN space —
concrete plazas, walking paths, decorative fencing, staircases between
elevations — not just the green pads parks get today. The framing that makes
this cheap: an open-space lot goes through the SAME pipeline as a building lot
(road → block → lot → pad → grammar), but the grammar emits ground furniture
instead of storeys. Everything below already exists: pads flatten and conform,
the shape grammar knows lot frontage/orientation, the roof-plan partitioner
(shape_grammar emitCrown) is exactly the space-divider a plaza needs, and the
baked-PBR surface pipeline can mint concrete/paver materials like it did the
HVAC kit.

- **P6.1 Lot selection + slab.** DistrictMap tags candidate lots (financial
  hub adjacency, beach frontage, or `plaza` zone roll). The lot keeps its
  flatten pad (it already grades + conforms); the "ground floor" is a concrete
  slab inset from the lot like a building footprint, with the existing plinth
  logic as the slab edge. New PartId::PlazaSlab + a paver/concrete Surface
  (Field2 recipe: expansion-joint grid + aggregate noise, same pattern as
  evalUtilityPanel).
- **P6.2 Plaza plan partition.** Reuse the roof-plan partitioner on the slab
  polygon: cells become lawn insets, planter beds (shrub clusters from the
  vegetation variants), a fountain (revolved basin + the water surface at rest,
  no sim), benches/light posts along cell seams. Deterministic per lot seed —
  same technique, different item catalogue.
- **P6.3 Paths as mini-roads.** Walking paths are Ramp-class-width ribbons on
  the EXISTING road mesher (weld profiles, conform regions, all battle-tested)
  with a paver surface and no markings — connecting slab edges to sidewalk
  entry points and, on multi-level lots, to stair landings. No new conform
  code.
- **P6.4 Stairs + fencing at grade breaks.** Where the slab (or two adjacent
  plaza cells) sits above the sidewalk/beach grade by > ~0.8m: emit a straight
  stair run (steps = height/riser, from the grammar's existing stoop logic
  generalized) and cap exposed slab edges with decorative fence segments
  (instanced post + rail, one InstanceGroup per lot). Below the threshold the
  slab just feathers with the plinth as today.
- Gate (device): a financial-district block and one beach lot read as designed
  public space at street level — slab + partition + at least one stair and
  fence run — with zero pokes (the pad/conform machinery is already proven)
  and no new per-frame cost class (all static instanced geometry).

## 7. P1.5 expanded: streaming the city (build-on-demand + cache)

Now justified: HLOD proxies keep the vista cheap, but a full 2.6km world still
RESIDES in memory, and bigger worlds (or aerial → street dives) need residency
management. Determinism is the foundation — every chunk's content is a pure
function of (level params, seed, cell coords), which the conform/bedrock work
guarantees, so a chunk can be REBUILT on demand instead of stored, and the
content-hash cache (P4.1 pattern) makes rebuild ≈ disk read after first visit.

**What streams (answering "does it include everything?"): yes — per cell:**
- Building detail meshes + their HLOD mass-box proxies (proxy tier stays
  resident always: it's tiny and owns the vista).
- Road deck chunks (P1.3 binning) + markings; the WELD stays a global
  load-time pass (it's cheap; only meshes stream).
- Vegetation/rock InstanceGroups (already per-cell) and their capsule/box
  colliders.
- Jolt bodies for everything above — colliders stream WITH their render
  chunk, one body-batch per cell, added/removed on residency change.
- NOT streamed: terrain (CDLOD already streams detail by construction), water,
  the road graph/lot data (small, resident — needed for AI/navigation later),
  audio, global systems.

**Shape:**
- **P7.1 Chunk manifest at load.** The generation pass runs as today but stops
  at DATA (graph, lots, per-cell work orders) — meshes/colliders per cell are
  built lazily. Manifest = cell → content hash.
- **P7.2 Residency controller.** Ring around the camera (R_detail ~ 700m =
  today's drawDistance, R_release = R + 25%): on enter, request cell; on exit,
  free GPU buffers + Jolt bodies. Budget in BYTES with an LRU beyond the ring.
- **P7.3 Background bake + cache.** A worker thread runs the cell work order
  (grammar emit → chunk mesh → upload staging; Jolt shapes built off-thread,
  bodies added on main). First build writes `cache/city/<hash>.bin`; later
  visits read it. Fade/pop policy: proxies are ALWAYS resident, so a late
  chunk is a soft LOD switch, never a hole.
- **P7.4 Determinism harness.** CI test: bake cell (i,j) cold, bake it again
  via cache, byte-compare; and a soak that drives the camera across the map
  asserting no chunk hash ever differs from the manifest.
- Branch note: this is a build-pipeline restructure (loader split into
  plan/bake phases + a residency system) — do it on a dedicated branch off
  this one, landing P7.1 (manifest split) first since it's independently
  useful for load time.

## Order of execution

1. P1.1–P1.4 (chunking + mips) — perf floor for everything else. **DONE**
2. P3.1 (conform evidence) — cheap, unblocks the conform rework. **DONE**
3. P2 (freeway ribbon) — needs a regen anyway; do after P1 so testing is fast.
4. P4.1 (erosion cache) anytime — independent quick win. **DONE**
5. P3.2 **DONE**; P4.2/P4.3 next.
6. §6 plazas (small, self-contained, huge look win near hubs/beach).
7. §7 streaming on its own branch (P7.1 manifest split first).
