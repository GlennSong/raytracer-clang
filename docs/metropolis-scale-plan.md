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

## 8. Freeway engineering: the corridor model (task 9, done RIGHT)

Device direction: freeways are multilane (CA-style, up to 8 lanes) with exits
where ONLY the last lane peels off through a gore and curves (clothoid) down
to a surface street; elevated sections spiral/slant down to grade; city
streets run underneath; and it must be REAL because the traffic simulation
depends on it. Borrowed civil-engineering vocabulary (the corridor model used
by Civil 3D / OpenRoads):

- **Alignment**: a stationed centreline — every point addressed by distance
  along the line. Horizontal geometry is tangent–spiral–arc–spiral–tangent:
  clothoid transitions so curvature ramps linearly (design speed sets minimum
  radius; a 40 km/h loop ramp may be R~50m, a 110 km/h mainline R>=600m).
- **Vertical profile**: separate 1D design — constant grades (freeway <=4%,
  ramp <=7%) joined by parabolic crest/sag curves. Elevated->surface = sag at
  the bottom, crest at the top; height / max-grade = minimum ramp length
  (that's WHY real ramps are long or helical).
- **Cross-section template + lane schedule**: lanes(station) — through lanes
  continuous end-to-end; lanes appear/drop only at GORES (the painted
  triangular nose). An exit = auxiliary lane -> decel length -> gore -> the
  aux lane BECOMES the ramp. Merges mirror with accel lane + ~50:1 taper.
- **Superelevation**: deck banks into curves (e ~ V^2/127R, cap ~7%) — the
  swept template rotates about the centreline per station.
- **Clearance + structures**: streets pass under when the profile holds
  ~5.1m vertical clearance over their surface; the deck becomes a viaduct on
  piers every 25-35m where deck-ground exceeds ~2m.
- **Interchange catalogue**: diamond (default), parclo/cloverleaf loops,
  trumpet, stack — each a recipe of ramp alignments between corridors.

**Per-lane driving, current state (city_sim):** NavGraph links carry `lanes`
(from width/3.5) and `laneCenter(link, lane, t)` offsets the drive point;
drivers roll a lane at trip start, KEEP the index across legs (clamped to
each leg's count), and car-following keys gaps per (link, lane). Missing for
real exits: (a) mid-link lane CHANGES (lane is fixed per trip), (b)
lane-level connectivity (router is link-level; any lane may take any turn),
(c) the geometric tie "ramp continues lane N's edge". Additions: fractional
lane offset (Real) animated toward a target lane; "seek rightmost lane
starting ~40m before a gore leg"; gore records which lane feeds the ramp.

**Phases (all proven in the FREEWAY LAB first — `freeway_lab.json`: HILLY
terrain (device: meshing vs terrain is part of the test), a small street grid
passing under, one curved partially-elevated mainline, NO buildings):**
- **P8.1 Alignment module.** Stationed horizontal alignment (clothoid corner
  fitting by curvature-driven trace), parabolic vertical profile, lane
  schedule spans. Headless unit tests (lengths, tangent continuity, curvature
  ramps, grade limits).
- **P8.2 Corridor sweep mesher + "corridor" level entity.** Sweep the
  template (median | lanes | shoulder) along the alignment with
  superelevation; per-lane-edge marking strips as geometry first (defers the
  marking-shader work honestly); flatten pads where at grade; piers where
  deck-ground > 2m. Terrain conform must hold on the hilly lab (poke map 0).
- **P8.3 Gores + diamond ramps.** Lane schedule grows an aux lane before each
  exit; ramp alignment inherits the aux lane edge at the gore, clothoids away
  and grades down to a street junction. Lab gate: geometry reads as a real
  exit (gore nose, taper) into the street grid.
- **P8.4 Lane-aware sim.** Fractional lane positions + target-lane seek; ramp
  links enterable only from the feeding lane; cars visibly move right, take
  the exit, and merge back on-ramp. Lab gate: watch a car street->on-ramp->
  mainline->off-ramp->street.
- **P8.5 Clearance solver + viaduct.** Profile constrained by crossing street
  elevations + 5.1m clearance; piers/bents emitted; streets truly run under.
- **P8.6 Loop ramps (parclo quadrant), then metro integration** behind
  corridor identity: the metro's freeway backbone becomes corridors; growth
  may not junction the mainline (grade-separated crossings only); ramps at
  interchange_spacing near arterials.

## Order of execution

1. P1.1–P1.4 (chunking + mips) — perf floor for everything else. **DONE**
2. P3.1 (conform evidence) — cheap, unblocks the conform rework. **DONE**
3. P2 (freeway ribbon) — needs a regen anyway; do after P1 so testing is fast.
4. P4.1 (erosion cache) anytime — independent quick win. **DONE**
5. P3.2 **DONE**; P4.2/P4.3 next.
6. §6 plazas (small, self-contained, huge look win near hubs/beach).
7. §7 streaming on its own branch (P7.1 manifest split first).

## §9 The corridor contract — how freeways live in the road graph

Decided with the device rounds of 2026-07-12. One sentence: **the alignment
is the document; everything else is derived.**

### 9.1 Source of truth and editability
A freeway is NOT authored as graph nodes. Its document is the CorridorDef —
control polyline, design speed/radius/spiral, vertical profile PVIs, lane
schedule, exits list. You edit THOSE (they round-trip through the editor as
a first-class entity); the drivable graph, the mesh, the flatten, the
furniture, and the piers all regenerate from them. Manipulating individual
generated nodes would let the graph drift out of agreement with the
concrete — the same bedrock rule as roads->blocks->lots.

### 9.2 One graph, 3D by elevation, classes carry the semantics
There is a single road graph. Freeway links are ordinary links whose
RoadClass is Freeway/Ramp and whose nodes carry elev-above-ground; nav
links lerp elevation, so the network is genuinely layered — same XY,
different heights, no phantom junctions (chain nodes never knot-merge; only
ramp terminals weld, explicitly). Nothing else in the sim special-cases
freeways: the router prefers them purely through classSpeed (28 vs 8 m/s),
signals skip them because no link ENTERS a junction on the mainline, and
lane counts derive from width as everywhere else. Ramp links are one-way in
stored direction. So: not a special case attached to the side — a class of
links with richer provenance.

### 9.3 Lanes per direction
CorridorDef.lanes.throughLanes is per carriageway; aux spans add the
exit/merge lanes per side with a 35 m taper. Changing throughLanes changes
deck width, marking count, nav lane count (width/3.5/2), and gore offsets
automatically — exits key off lanesAt(station, side), so the peel always
comes off the OUTERMOST lane wherever the schedule puts it. Asymmetric
carriageways (3 up, 4 down) are one refactor away: LaneSchedule already
answers per side; CorridorDef needs a second throughLanes.

### 9.4 Order of operations (metro integration, P8.6)
1. TERRAIN exists.
2. FREEWAY corridors route hub-to-hub over it (clothoid alignments, profile
   solved against terrain: at-grade where possible, viaduct over dips/city).
3. INTERCHANGE sites picked along each corridor (spacing param, must have
   reachable street-node candidates on the needed side) — each site stamps
   the 4-ramp diamond: exit + on-ramp per carriageway, PAIRED by
   construction.
4. STREET GROWTH runs with the corridor as a constraint: colonization may
   cross UNDER a viaduct span (clearance >= 5 m) or be culled at grade;
   ramp terminals are seeded as attractors so the network grows TO the
   interchanges (connector roads emerge, then fan into grids/radials/
   cul-de-sacs as the district kinds dictate).
5. LOTS: the corridor EASEMENT (deck footprint at grade + pierBases discs +
   a 12-18 m verge) is subtracted from blocks before parceling. Easement
   land is not buildable: it hosts overgrowth scatter, chain-link fencing,
   utility boxes, billboards — the "freeway margin" look — never buildings.
6. FURNITURE/SIGNAGE from the final graph.

### 9.5 Freeway-to-freeway (system interchanges)
Where two corridors cross, connect them with DIRECTIONAL RAMPS built by the
same exit machinery, except the ramp's target is a MERGE STATION on the
other corridor instead of a street node (deck-to-deck splice: leave
corridor A at its gore elevation, clothoid + profile to corridor B's merge
elevation, join as an on-ramp). Four directional ramps = a simple system
interchange; loops (cloverleaf quadrants) only where a left-turn movement
would otherwise need a tight directional ramp. Needs: ramp-on-ramp piers
(done), gore machinery on BOTH corridors (done), and a solver that picks
merge stations far enough from each corridor's other ramps (the P8.6
spacing rule reused). Deferred until one corridor + city integration is
solid.

## §9.6 The merge is a LANE EVENT, not a point splice (device round, 2026-07-12)

### What is actually wrong today (verified in code)
1. corridor_mesh.cpp:356 and level_loader.cpp:2451 both compute the ramp's
   merge offset with lanesAt(sg - 1). For an EXIT that is correct (the decel
   lane exists BEFORE the gore). For an ON-RAMP the accel lane exists AFTER
   the merge station — at sg-1 there is no aux lane, so P0 lands in the
   centre of the outermost THROUGH lane, INSIDE the deck. The whole parallel
   run of the on-ramp ribbon is therefore tucked under the deck's shoulder:
   "the ramps are below the freeway and the cars pop through the top".
2. The ramp's final nav edge connects to the carriageway chain node nearest
   the merge station IN EITHER DIRECTION (fabs(chainS[k]-sg)) and hops from
   the ramp's parallel offset to the CARRIAGEWAY CENTRELINE in one link — a
   sideways (sometimes slightly backward) lurch instead of a merge.
3. Lane changes are adjacency-correct (+/-1 only) but PURPOSELESS: a paced
   coin flip. No slow/fast semantics, no relationship to exits.

### The construction that fixes all three
**A ramp does not END at the freeway; it BECOMES the auxiliary lane.**

Geometry (one ribbon, no overlap):
- ON-RAMP: street -> climb -> arrive PARALLEL at the aux-lane offset
  (lanesAt(sg + 1), i.e. one lane OUTSIDE the through lanes) exactly at the
  merge station -> continue as the accel lane over [sg, sg + accelLen] (this
  span is already meshed by the deck's aux flare — the ramp ribbon simply
  ENDS at sg where the flare takes over; today they overlap wrongly) ->
  the flare's taper closes the lane at the far end.
- EXIT mirrored: the deck flare opens the decel lane at [sg - decel, sg];
  the ramp ribbon begins at the gore where the flare ends.
- Rule of thumb: the FLARE owns everything on the deck; the RAMP owns
  everything off it; they meet at exactly one rib with identical offsets.

Nav (the zipper):
- The ramp chain does not stop at the merge point. It continues ALONG the
  aux lane — nodes at the aux-lane offset, deck elevation — from sg to
  sg + accelLen*0.8, and ONLY THEN joins the carriageway chain node just
  DOWNSTREAM (never backward). The join edge is ~30 m long and nearly
  parallel: geometrically a zipper.
- A merging car is therefore ON the accel lane in its own nav link while
  freeway traffic passes on its left; joining the flow is the existing
  laneF glide (one lane left), indicator blinking, executed when the join
  link hands over to the carriageway link.
- EXIT mirrored: the carriageway hands over at sg - decel*0.8 to a
  decel-lane chain, then the gore, then down.

Lane semantics (slow right, fast left):
- Lane 0 sits nearest the median (one-way links centre their lane set) =
  the FAST lane; the highest index is the SLOW/EXIT lane. This matches the
  device rule: "slow lanes closer to the exit ramps, fast lanes closer to
  the median".
- Each driver gets preferredLane from its personality (speedFactor
  percentile): fast drivers prefer low indices, slow drivers high.
- ROUTE-AWARE OVERRIDE: when the route leaves via a Ramp link within
  ~350 m, preferredLane = rightmost — the car works its way back across,
  one adjacent lane at a time, indicator on, before the gore.
- Discretionary changes stop being coin flips: every paced decision moves
  ONE step toward preferredLane (or holds). Adjacency is preserved by
  construction; "random jumping" disappears because direction is now
  purposeful.

### Order of implementation (when approved)
1. lanesAt(sg +/- 1) fix by ramp kind (mesh + nav must stay identical).
2. Ramp ribbon ends at the flare boundary (kill the overlap).
3. Nav accel/decel-lane chains + forward-only join nodes.
4. preferredLane + route-aware exit seek on the existing laneF machinery.
Acceptance: a warmed lab run where an on-ramp car visibly rides the accel
lane, blinks left, glides into lane 3, works left to its preferred lane,
then works right again and exits — without ever clipping a parapet or
popping vertically.

## §10 ONE road network — reconciling corridors, ramps, and streets
(device round 2026-07-12: "The freeway should be a part of the road network
... I imagine for an onramp there's a control point on the freeway
entrance/exit and then another one connecting it to the road.")

### 10.0 The decision
Two AUTHORING documents, ONE derived graph, split MESHING by class.

- Authoring stays heterogeneous because the design data is genuinely
  different: a street is a node/edge net the mesher drapes; a corridor is an
  alignment (control polyline + profile + lane schedule). Forcing either
  into the other's document model loses information.
- But everything DERIVED must come from one place: a single per-level
  RoadGraph — nodes carry {pos, elev, elevAbsolute}, edges carry {width,
  class, oneWay, provenance} — that nav, lots, growth, furniture, editor
  picking, and junction logic ALL read. The citysim-side ExtraNavGraph
  bolt-on is deleted once this lands (it was the prototype of this graph).
- Provenance tag per edge: Street | CorridorMain(corridorId, station range)
  | CorridorRamp(rampId). Meshing dispatches on it: the street mesher
  ignores corridor provenance; the corridor sweep meshes only its own.
  Junction geometry at shared nodes is ALWAYS street-mesher-owned.

### 10.1 The RAMP is a first-class connection document
Exactly the device intuition: a ramp is TWO ANCHORS + design params.
    RampSpec {
      corridor anchor: (corridorId, station, side, kind: exit|onramp)
      street  anchor: (roadNet nodeId)         // a REAL street node, by id
      params: decel/accel length, radius, spiral, laneWidth
    }
- Geometry (clothoid + profile + aux flare) derives from the two anchors —
  never guessed from a target point again. The loader's node-snapping
  heuristics become a PROCGEN policy that emits RampSpecs (the metro picks
  the node; a human can re-pick it in the editor).
- Editor: anchor A slides ALONG the ribbon (station scrubbing); anchor B
  snaps street nodes. Both are gizmos on the corridor's document entity.
- The street anchor's node becomes a REAL junction in the street net: the
  ramp injects a short landing STUB edge into the RoadNet before meshing,
  so the junction mouth, corner blending, stop bar, crosswalk suppression,
  and signal placement all happen through the normal street machinery.
  (Today the ramp pavement just lies on top of an unsuspecting node.)

### 10.2 Elevation becomes native to the graph
- RoadGraph nodes already carry elev/elevAbsolute (P8.4); RoadNet's authored
  street nodes gain optional elevation too, replacing the integer `layer`
  hack long-term (street overpasses become real). At-grade streets default
  0/relative; corridor chains absolute; ramps absolute.
- Rule: a shared (welded) node takes the STREET's elevation — ramps land at
  street grade by construction.

### 10.3 Meshing + carving interface contract
- The corridor sweep stays the mesher for Freeway/Ramp provenance
  (elevated structure is its whole point). The street mesher owns every
  junction polygon, including ramp landings: the ramp ribbon TERMINATES at
  the junction-mouth radius the street mesher reports; the mouth pavement,
  curb returns, and markings are street work.
- ONE carve pass: corridor + ramp flatten regions merge into
  roadNetConformRegions so terrain conforming has a single owner (same
  bedrock as §3: one profile source).
- Deck COLLIDERS: elevated deck + ramps get static mesh colliders (players
  and physical cars can drive the freeway; today only ghosts can).

### 10.4 Editor integration
- Corridor doc entity carries the deck Renderable -> click-select works
  like streets (today the mesh lives on untagged runtime companions).
- Gizmos: alignment control points (XZ), profile PVIs (Y at station), exit
  stations (slide), ramp anchors (10.1). Every edit re-derives graph + mesh
  + flatten through the same regenerate path roads use.

### 10.5 Systems sweep (after the graph unifies)
- Lots/growth: corridor easement + pier footprints come from the unified
  graph (buildable mask); ramp terminals seed growth attractors.
- Pedestrians: sidewalk routing EXCLUDES Freeway/Ramp classes (walkers
  must never route along a carriageway; audit today's behaviour).
- Furniture: signals become junction-aware of ramp landings; no lamps on
  corridor provenance (class rule already holds).
- Router: already class-speed-weighted; nothing to do.

### 10.6 Metro integration (supersedes P8.6)
The metro generator stops emitting freeway-WIDTH street edges entirely:
it plans corridor alignments hub-to-hub, stamps interchange RampSpec pairs
(exit + on-ramp per carriageway, spacing + feasibility checked against the
grown street net), and the unified graph does the rest. The "freeway looks
like a fat street" era ends.

### Order of work + migration
1. 10.2 graph schema (elev on RoadNet nodes; provenance tags) — additive.
2. 10.1 RampSpec document + derived geometry, replacing ExitDef.target
   (ExitDef stays as the serialized form, gains streetNodeId).
3. 10.0 unified graph assembly in the loader; citysim reads it; DELETE
   ExtraNavGraph.
4. 10.3 junction stubs + mouth contract + single carve pass + colliders.
5. 10.4 editor pickability + gizmos.
6. 10.5 sweep, 10.6 metro.
Each step keeps the lab green (drivable end-to-end) before the next.
