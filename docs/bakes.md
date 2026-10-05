# Bakes: what they are, when they go stale, and how to recover

A **bake** is everything a level's city takes minutes to build — roads, lots, buildings, the parts drawn
per cell — computed once and saved, so the next load reads it in seconds. The design is ADR-0084
(`engine::bundle`); this page is the working guide: what happens on a bake, what can go wrong, and what
to do when it does. The short rules every agent must follow are in AGENTS.md, under "Bakes (Agent Rule)".

## What is on disk

Everything lives under `cache/` in the checkout the game runs from (each worktree has its own):

| Folder | What | Keyed by | Size |
|---|---|---|---|
| `cache/levels/<key>/` | the level's **bundle**: `level.bundle` (sections) + `manifest.json` (its table of contents and producers) | the producers' keys combined | 1 GB (metro) to 8 GB (the island) |
| `cache/terrain/<hash>.pyramid` | the baked ground mesh (terrain with every lot flatten folded in) | the ground's content, flattens included | 145–385 MB |
| `cache/terrain/<hash>.bin`, `maps_*.bin`, `weather_*.bin` | the erosion bake, the cover maps, weathered ground | the terrain settings | ~65 MB each; slow to remake |
| `cache/population/<hash>.pop` | who lives and works where | places, street graph, agents, buses | small |
| `cache/fields`, `cache/road_signs`, `cache/clouds` | textures, sign atlases, cloud fields | their own inputs | small |

Every one of these is a **cache**: deleting it is always safe; it is rebuilt on the next load.

## What happens on a bake

A bundle is built by **producers**, in order, each writing named sections:

1. `city` (lanes levels): the road network — decks, junctions, the nav graph. Tag `kLanesBuildTag` in
   `procgen/city/roads/lanes/city_producer.cpp`.
2. `lots` (lanes levels) / `citylots` (lattice levels): lots, buildings, parks, the campus, every part
   mesh, split per 160 m render cell (`citylots/cell/<cx>_<cz>/parts/<n>`). Tags `kLotsBuildTag`
   (`lots_producer.cpp`) and `kCityLotsBuildTag` (`citylots_producer.cpp`).

A producer's **key** is a hash of: its tag, the lot format version (`lotcache::kLotsFormatVersion`), the
Real width, scripting on/off, the level JSON blocks it reads (`terrain`, `water`, `citysim`, road
entities), the render cell, the spawn point, the style books (`style_book.lua`, `archetype_book.lua`) —
and, for `lots`, **the `city` key**. So the stages chain: new roads always mean new lots.

On load (`obtainForLevel`): if a bundle with every producer's current key exists, it is opened (a hit,
well under a second). Otherwise the level is baked: a producer whose key matches an older bundle is
**copied forward** from it; the rest are built. `rt_bake <level>` does the same from the command line.

Built at load from the bundle, with their own caches: the terrain pyramid (keyed by the flattens, so
moved lots mean a new pyramid) and the population (keyed by places and the street graph).

### Shared blocks (2026-10-05)

A new bundle **clones** every section that comes out byte-identical to one in the level's previous
bundle (or the bundle a producer is copied forward from) instead of writing it: `FICLONERANGE`, so the
two files share those blocks on btrfs. Sections are 4 KB-aligned in files so this works. Every clone is
first **compared byte for byte** with the source, so a clone is exactly what would have been written.
On a filesystem that cannot share blocks (tmpfs, ext4, macOS) the bytes are simply written.
`rt_bake` prints how much was shared: `shared 1.07 GB ... (100%; new on disk 3.4 MB)`.

Measured on metro_v2_test: a rebake after a one-line campus change wrote 3.4 MB of 1.07 GB. A bake is
deterministic: the same inputs give byte-identical sections.

Consequences: `du` overstates disk use (shared blocks count in every bundle that holds them); deleting
an old bundle frees only the blocks nothing newer shares; keep the newest bundle per level, since it is
what the next rebake clones from.

## Cascades: how far one change reaches

Correctness never depends on how far a change reaches — every section is recomputed on every rebake and
only identical bytes are shared. Reach only decides how much is shared.

- **Between stages** — handled by the keys: a roads change rebuilds the lots; moved lots rebuild the
  pyramid; changed places rebuild the population.
- **Inside the lots stage** — a local edit can move far:
  - *one-per-city choices*: the campus takes the first block that qualifies, the sports field and dorm
    block the first suitable ones after it, big-box stores keep 700 m apart in block order. A change
    that makes a block stop qualifying moves the campus and everything placed relative to it.
  - *dice*: each block has its own seed and each lot's dice are keyed by its index in the block, so
    adding or removing a lot reshuffles that block, not the city. Code that draws from ONE city-wide
    random stream would shift every draw after it — don't write that.
  - *grading*: lot pads flatten the terrain; the next lot's walks and planting stand on that ground.
  - *cells*: a block across two 160 m cells changes both.
  - *global code*: facade, colour or style-book changes touch every building; nothing is shared.

## Edge cases

| Case | Outcome |
|---|---|
| The old bundle is deleted after a clone | Fine: btrfs keeps the shared blocks for the new one. |
| Either bundle is modified later | Fine: copy-on-write; neither sees the other's change. Bundles are written once. |
| A bake crashes, or two run at once | Fine: each writes `<key>.tmp-<pid>` and renames into place only when complete; a source that disappears mid-bake just means that section is written. |
| Hash collision / a source that rotted on disk | Caught: the byte comparison refuses the clone and writes the true bytes (`clonesRefused`). |
| **Code changed, tag not bumped** | **NOT caught.** The old bundle keeps loading: wrong, but it looks current. |
| **The lots stage reads a file that is not in its key** | **NOT caught.** Changing that file does not rebake. |
| A bundle from another build (different engine) | Recorded in the manifest (`engine.version`), enforced only with `RT_BUNDLE_REQUIRE_ENGINE=1`; two uncommitted builds look alike. |
| A level baked inside a test runs out of memory | island_8 / island_8_nature `bad_alloc` under the suite's 30 GB ulimit: pre-bake them with `rt_bake` first. |
| The bundle format changes | `kFormatMajor` is checked hard on open; a mismatch rebuilds. Old 64-byte-aligned bundles still read. |

Owed (not built yet): `RT_BUNDLE_VERIFY` — rebuild in memory and compare every section's hash with the
cached bundle, run nightly — which would catch the two uncaught rows; and an audit of what the lots
stage reads against its key.

## When to bump what

- **`kLotsBuildTag` and `kCityLotsBuildTag`** (both, same value): any change to what the lot / city pass
  OUTPUTS — sculpting (parks, quad, sports field), recipes and massing, lot selection (campus, big box),
  grading, anything stored in a `LotBuilding` or a part mesh. Note the change in the tag's comment.
- **`kLanesBuildTag`**: any change to the roads the lanes builder produces.
- **`kLotsFormatVersion`** (and `kLotsVersion` in lot_cache.cpp) **plus a field in
  `tests/test_lot_cache.cpp`**: any change to the lot RECORD's fields.
- **No bump**: building interiors and their furniture (grown at runtime from `BuildingParams`), the
  citysim, rendering, anything computed at load rather than read from the bundle. If a change reads a
  NEW file during the lots stage, add it to the producer's `identity()` instead.

A tag bump makes every level's bundle stale; the level suite then rebakes everything it loads. Iterate
on looks with `RT_NOCACHE=1` (builds in memory, writes nothing), and bump once the change is settled.

## When it all goes butts up

Symptoms of a stale or bad bake: a change does not show; the game and a test disagree; something
appears that the code no longer makes; a load logs `[bundle] ... unreadable`. In order, least to most
drastic:

1. **Is it the bake?** Run once with `RT_NOCACHE=1` (bakes in memory, ignores and writes no cache). If
   the problem goes away, the cached bundle is stale.
2. **Rebake that level**: `build-viewer/rt_bake assets/levels/<level>.json --force`. Then ask why it was
   stale: almost always a missing tag bump or an input missing from the key — fix that too.
3. **Drop that level's caches**: its `cache/levels/<key>/` folders (the manifest's `level.path` names the
   level) and, if the ground looks wrong, the `.pyramid` files from around the same time.
4. **Wipe everything**: `rm -rf cache/levels cache/terrain cache/population` in that checkout. Safe;
   costs time: metro ~15 s, the island ~3 min, plus the erosion bake (minutes) on the first load. Then
   pre-bake island_8 and island_8_nature before running the level suite.

Never delete a `*.tmp-<pid>` folder while that process is running (a bake in flight).

## Housekeeping

Old bundles are never deleted automatically. `build-viewer/rt_bake --prune` lists the bundles under
`cache/levels` that are not the newest for their level (a dry run); `--prune --yes` deletes them; list
level files after it to keep their CURRENT bundle whatever its age. The editor's Level cache panel prunes
one level. Terrain pyramids are not covered (they do not record their level): the `rt-bake-cleanup` skill
(`~/.claude/skills/rt-bake-cleanup/prune_bakes.py`, a dry run by default) ages them out by date. Known
gap (ADR-0084): the prune keys on the level path as written, so a test's absolute-path bundle and
rt_bake's relative-path one are two "levels". Planned: the level suite pruning after itself; terrain files
that record their level.
