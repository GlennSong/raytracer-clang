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

## 5. Building foundations (done) + look follow-ups

Plinth 0.45m landed; evaluate on-device. If slopes still read wrong, the
architect's plinth should scale with local grade (pad drop across the
footprint), and street-side entrances want a step/stoop from the grammar.

## Order of execution

1. P1.1–P1.4 (chunking + mips) — perf floor for everything else.
2. P3.1 (conform evidence) — cheap, unblocks the conform rework.
3. P2 (freeway ribbon) — needs a regen anyway; do after P1 so testing is fast.
4. P4.1 (erosion cache) anytime — independent quick win.
5. P3.2, P4.2/P4.3, P1.5 (streaming) after re-measuring.
