# World streaming — a world designed for 100 km, built for what we have

**Status:** design plan (ADR-0095). Step 1 (ADR-0094, the Vulkan allocator and upload queue) is
done; everything below builds on it. Supersedes the *scale target* of
`open-world-foundations-plan.md` (ADR-0034: a ~16 km curated place, single precision) — its
reverse-Z, partitioning and HLOD work stand; its size assumption does not.

Glenn, 2026-09-24: *"design for 100 km"*, after asking whether terrain could be baked as tiles at
degrees of detail — fine near the player, coarser away — and what would scale from 1 km to 100 km.

---

## 0. Why — what the 6 km world already told us

Measured on metro_planned (`mem?`, ADR-0094):

| On the GPU | Size | Note |
|---|---|---|
| Building part chunks | 4.9 GB | 10,181 meshes, every building's detail, resident all the time |
| Road decks | 1.4 GB | asphalt alone 779 MB, ~14 M vertices |
| CDLOD terrain | 3–4 MB | 37–53 tiles |
| Textures | 142 MB | |

The card is full with a **6 km** world, and not because of terrain. The level loads *everything*
and keeps it; a vertex costs 56 bytes whatever it is; the terrain's finest cell is tied to a stock
level count, so it grew to 11.7 m when the world grew and buried streets beside graded lots. None
of that survives a bigger world. The design rule that follows:

> **Memory is bounded by what is near the camera, never by the size of the world.**
> Disk and bake time may grow with the world; GPU and RAM may not.

## 1. Scale targets

| World | Terrain on disk (fine only where built) | Resident terrain | Everything else resident |
|---|---|---|---|
| 1 km | ~2 MB | a few MB | all of it (fits) |
| 6 km (today) | ~30 MB | a few MB | must stream (today: 6.5 GB, doesn't) |
| 100 km | a few GB | a few MB | must stream, by region |

Target machine: the 3080 (10 GB, ~1.6 GB taken by the desktop) — a budget of **~6 GB of GPU**
for the world, set from `VK_EXT_memory_budget` at run time, not hard-coded. A smaller card keeps
less detail resident; a bigger one keeps more. Nothing else changes.

## 2. Coordinates

- Simulation and world data are already **double** (`Real` is 8 bytes). Keep them double.
- **The GPU sees camera-relative float positions.** A 32-bit float at 50 km resolves ~4 mm (and
  ~1.5 cm at 64 km) — visible jitter up close. Each frame the renderer subtracts a *render origin*
  (the camera's position, snapped to a 1 km grid so tile data stays stable) before converting
  to float. Tile and chunk meshes are stored relative to their own origin, not the world's, so
  nothing baked depends on where the camera is.
- Physics (Jolt) runs in a region-relative frame the same way, rebased as the player crosses
  regions. (Jolt supports double precision; we keep its float build and rebase instead.)

## 3. Terrain: a baked height pyramid

### 3.1 What is baked
The **final ground** — natural terrain, the carved road grid, block terraces, building pads,
every flatten — sampled into a quadtree **pyramid of height tiles**:

- **Tile:** 128×128 cells = 129×129 samples. The border row/column is shared with the neighbour,
  so edges at the same level match exactly.
- **Sample:** `uint16`, quantized per tile (`h = min + q * scale`, 1 cm steps over up to 655 m of
  relief; a tile with more relief uses a coarser step, flagged). Normals are derived in the shader
  from neighbouring samples; nothing else per vertex.
- **Levels:** level 0 is the finest (0.5–1 m cells where there is built content). Each coarser
  level is **downsampled from the one below**, never recomputed, so a morph target is always the
  true parent. For 100 km: 1 m × 128 = 128 m tiles at level 0, ~10 levels to one tile per
  ~128 km.
- **Per tile:** min/max height (culling, bounds) and **geometric error** — the largest vertical
  difference between this tile drawn at its level and the level below. Tiles that bury a road are
  given infinite error (they must refine).
- **Sparse:** a child tile is only stored when the parent's error over it exceeds a tolerance
  (1–2 cm). Plains, sea and gentle hills stop at coarse levels; streets, pad edges, embankments and
  ridges go deep. This is what makes disk size scale with *content*, not area.

### 3.2 Storage
In the level bundle (ADR-0084), cell-addressed, one section per tile:
`terrain/L<level>/<tx>_<ty>`, zstd-compressed, plus a small `terrain/index` (which tiles exist,
their min/max/error). The bundle is memory-mapped, so a tile read is a page fault and a
decompress — no file parsing. For 100 km the terrain goes in per-region bundles (§6).

### 3.3 Rendering
- **One shared grid mesh** (129×129) for every tile; the vertex shader reads the tile's heights
  from a texture array page and morphs toward the parent near the level boundary (CDLOD).
  Per-tile GPU cost: 129² × 2 B ≈ 33 KB, against ~1 MB of vertices today.
- **Selection by error, not distance alone:** split a tile when its error, projected to the
  screen, exceeds ~1 px. Flat ground stays coarse up close; road edges refine wherever they are.
- **Pages:** the residency service (§4) owns the texture-array pages; a missing child draws its
  parent until it arrives (never a hole).

### 3.4 Collision, lots, editing
- The **physics heightfield** near the player is built from the same level-0 tiles — what you
  walk on is what you see.
- **Lot paths and draped dressing** sample the baked tiles, not the formula — one surface for
  everything (today `lotMeshCell` has to match the mesher by hand).
- **Editing** marks the tiles under a change dirty; the editor re-bakes those and their parents
  (milliseconds for a few tiles). The runtime formula path stays as the editor's live preview and
  as the fallback for a level with no bake.

## 4. The residency service

One engine service decides what is on the GPU. Everything streamable is a *client*:

| Client | Unit | LOD chain |
|---|---|---|
| Terrain | height tile | the pyramid (§3) |
| Buildings | cluster of a cell's buildings | full parts → shell → HLOD proxy → skyline impostor |
| Roads | road cell (the bundle's cells) | full deck → simplified deck |
| Props, vegetation | cell | instances → impostors |
| Interiors | building | loaded on approach only |

- A client registers **resources**: bounds, LOD level, byte cost, and a loader (decode from the
  bundle on a worker thread → upload through the queue, ADR-0094).
- Each frame the service ranks wanted resources by **priority** (screen-space error / size,
  distance, visibility, and "the player is heading there"), keeps the resident set **under
  budget**, issues loads, and evicts the lowest-priority residents (retired, not destroyed —
  ADR-0094). Hysteresis between load and evict distances so nothing thrashes.
- A resource that is not resident yet is **covered by its coarser parent**, which always is:
  the coarsest level of every client is small enough to keep resident for the whole world.
- Terrain's private tile cache (`TerrainLodSystem`) becomes the service's first client;
  buildings are the second and the big win (4.9 GB → the ring around the camera).

## 5. Formats that stop wasting memory

- **Vertex layouts per kind**, not one 56-byte vertex: terrain has none (§3.3); buildings and
  roads use 32 bytes (float3 position relative to the chunk origin, octahedral normal+tangent
  in 2×16 bits each, half2 UV, RGBA8 colour); a full-precision layout remains for what needs it.
  Both backends (Vulkan, Metal) and their shaders change together.
- **Road decks meshed to a tolerance** at the source (the lanes deck mesher): a vertex only where
  the surface bends by more than ~1 cm. Long straight roads collapse; curves and crests keep
  detail.

## 6. Regions — how 100 km is baked and loaded

- The world is divided into **regions** (8 km × 8 km). A region owns its bake outputs — terrain
  tiles, road cells, lots, buildings — in its own bundle, keyed by the hash of its inputs, so an
  edit re-bakes one region, and regions bake in parallel.
- Content that crosses regions (a freeway, a coastline) is baked by the region that owns its
  segment; the shared border tiles are the single place two regions must agree, and the bake
  checks it.
- A city is baked per region today it spans (metro_planned fits in one); a 100 km world is many
  cities and towns joined by regional roads (the outer loop and expressways are the pattern).
- **Simulation** follows the same partition: full simulation near the player, the coarse tier in
  loaded regions, dormancy beyond (ADR-0062's tiers), and a **hierarchical nav graph** — regions
  joined by highway links — so a trip across 100 km plans in two levels.

## 7. Budgets (3080, from the driver's budget)

| Pool | Budget | Today (metro_planned) |
|---|---|---|
| Terrain pages | 64 MB | 3–4 MB (CDLOD vertices) |
| Buildings | 2.5 GB | 4.9 GB, all resident |
| Roads | 400 MB | 1.4 GB |
| Props, vegetation, vehicles, people | 800 MB | — |
| Textures | 1 GB | 142 MB |
| Render targets, reserve | ~1 GB | — |

The service enforces the total; the split is a starting point to tune, not a contract.

## 8. Order of work

1. **Done — allocator and upload queue** (ADR-0094): 28 device blocks instead of 25,394, peak
   9.6 → 8.1 GB, load 225 → 180 s.
2. **Residency service core + terrain as its first client**, with the baked pyramid for the
   current 6 km world: bake stage, tile format in the bundle, shared-grid rendering (Vulkan and
   Metal), error-driven selection, collider from tiles. *Fixes the buried roads properly.*
3. **Buildings as the second client** — cluster by bundle cell, chain full → shell → HLOD.
4. **Vertex layouts** (32-byte building/road vertex).
5. **Road deck tolerance meshing.**
6. **Camera-relative rendering** (render origin), then physics rebasing.
7. **Regions:** per-region bundles and bakes, parallel; hierarchical nav.
8. **A 100 km test world:** several towns joined by regional roads, most of it terrain — the
   proof that memory stays flat as the world grows.

Each step lands with before/after `mem?`, load time and frame time on metro_planned, and a
level-test budget gate (mesh bytes from the asset manager, which works without a GPU) so the
numbers cannot silently regress.

## 9. What this does not change

- Levels without a bake still load and run (the formula path, today's behaviour).
- The editor keeps live terrain editing; baking is what play and ship use.
- The city design pipeline (brief → plan → lanes scene → bake) is unchanged; it gains outputs.
