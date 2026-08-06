# Piedmont P8 — footprint-first city, and what the profiling taught us

Report covering commits `e755b51..9863f03` (City Planner → sim rate ladder).
Written for the next person to pick this up — including a future me.

---

## 1. Where piedmont stands

| | before P8 | now |
|---|---|---|
| arterials | space colonization (tangled, sliver junctions) | constructed: footprint → bisection |
| built lots | 2,176 | 6,127 |
| towers | capped ~16 floors (silently) | 55–62 floors downtown |
| ambient cars | 170-triangle boxes | real `mesh.car` bodies, round wheels |
| kerbside parking | stalls 7 cm over the kerb | real 2.5 m parking band, 0.0000 m overhang |
| drives city→both towns | south exit had 24 holes | clean (3.8 km / 2.6 km, 0 defects) |
| frame (downtown, 3000 cars) | ~7 fps | ~16 fps |

Both piedmont levels generate footprint-first (`"skeleton": "footprint"`).
Full suite 1145/1145, ctest 3/3.

---

## 2. Generation findings

**Space colonization is the wrong tool for arterials.** Every rule in it is
local: the kill radius stops two roads chasing the same attractor, but nothing
says "two roads shouldn't run 60 m apart," and junctions are never *placed* —
they happen where growth fronts collide, at whatever angle. That is the direct
cause of the tangles, the near-parallel pairs and the sliver wedges. It stays
behind a recipe key for old levels; the arterial tier is now constructed.

**Construction gets the guarantees for free.** Footprint polygon → recursive
near-midpoint bisection to district cells, where *the cut lines are the
arterials*. Spacing is the bisection stop size, junctions are placed stations,
and every district is bounded by arterials by construction. Measured on the
fixture: in-polygon spans median 1929 m, none under 400, worst foldback 21.4°.

**Two bugs were emergent, not local.** The south-exit drive holes and the
east-town 179° hairpin — both of which cost dedicated fix rounds — simply
stopped existing once the skeleton was constructed. They were artifacts of the
generator, not defects to repair.

**A junction registry beats a cleanup pass.** Sibling cells share boundary
chains; without a global registry of gates + cut endpoints, two branches plant
feet a few hundred metres apart and the consolidation backstop "fixes" it by
dragging junctions into 64° kinks. Snapping at construction time removed the
kinks *and* upgraded the topology to long through-avenues.

**Parcelling must follow the block boundary, not its OBB.** The old frontage
parceller laid rows along the OBB's two long sides and dropped any cell that
clipped — on trapezoidal faces most cells died, leaving one building alone
(zero survivors → blind interior bisection → landlocked mid-block buildings).
A boundary walk works on any simple polygon and fronts every lot by
construction: landlocked rejects 1102 → 3.

**Silent defaults are the expensive kind.** `LotParams::center` — the anchor
all height grading measures from — was never set by any loader since the
polycentric switch. It defaulted to the world origin while the city sits at
(900,900), so "coreness" was 0.0 city-wide and every tower capped at its
floor-band minimum. No error, no warning, just a city with no skyline.

---

## 3. Performance findings

### 3.1 Methodology (learned the hard way, twice)

**Do not compare runs at equal wall time.** A slower build drops more clock
backlog, so it reaches a *different sim hour* — and the workload varies with
the hour (rush vs midday). An early comparison showed the sim "doubling" in
cost when it had only sampled rush hour. Compare at equal sim hour, or read
the census (`active / far / moving`) before believing any delta.

**Instrument first, then look.** `RT_DUMP_STATS` (frame + per-system bill) and
`RT_GPU_TIME` (command-buffer GPU ms) answered "CPU or GPU?" in one run. The
per-system bill named `CityRenderSystem` immediately; guessing between clouds,
SSAO and population would have burned days.

**A green suite can be measuring nothing.** The `city_render` tests asserted
bake counts on a city whose agents never departed — they sat parked at home
for the whole test. Found only when a new test needed motion.

### 3.2 The structural traps

**A city-wide `InstanceGroup` cannot be frustum-culled.** One bounding sphere
per group means a group spanning the city is always submitted — all ~4,450
parked cars, every frame, in colour *and* shadow passes. Harmless at 170
triangles a box; my own fleet upgrade to 1,898-triangle bodies turned it into
~8M triangles a frame. **Any city-scale instanced content needs distance
culling at bake time.** Fixed: poses resolved once, drawn within 450 m.

**The fixed-step death spiral.** When the sim exceeds its step budget the clock
runs *more* steps to catch up, each as expensive as the last, until it hits the
8-step cap. Piedmont sat pinned at exactly 8. The cure is not a bigger cap —
it is making the step cheaper, or (for traffic) simulating coarsely.

**Work whose only consumer is the renderer belongs once per frame.** The pose
bake ran on every fixed step; only the last is ever drawn. `FrameContext` now
carries `fixedStepIndex/fixedStepCount` so a system can tell.

**Tiering saved the work but not the iteration.** ~12 full-population loops per
step each walked all 4,800 agents to `continue` on the far tier. `sizeof(Agent)`
is 432 bytes — ~2 MB of state, ~25 MB of memory traffic per step, mostly to
skip. Fixed with an active list.

### 3.3 The biggest win was a design fix, not an optimization

Spreading workplaces (gravity-biased instead of uniform over a downtown-heavy
set) and staggering schedules (five shift archetypes instead of one 1.5-hour
rush) took the frame **7 → 16 fps at the same population** — more than every
code optimization in the round combined, and the city reads better for it.

| | before | after |
|---|---|---|
| active (K/P) | 1,420 | 950 |
| moving | 3,970 | 3,030 |
| sim step | 13.7 ms | 9.7 ms |
| steps/frame | 8.0 (capped) | 3.7 |

### 3.4 Where the frame goes now (downtown, 3000/1800)

- `fixedUpdate` ≈ 57 ms/frame over ~3.7 steps — **the traffic sim dominates**
- GPU ≈ 32–48 ms depending on viewpoint, ~8.5 M triangles (buildings now)
- Scenery cull and once-per-frame bake are done; the bake is ~1 ms/step

---

## 4. Process findings

- **Stale binaries cost two rounds.** `cmake --build build -j8` (all targets)
  before ever saying "take a look" — a green test suite proves the code, the
  user experiences the binary. The specific trap: an asset (`vehicles.lua`) and
  its C++ reader evolved together, and a stale `editor_app` read the new asset
  with the old reader, drawing cars with no bodies.
- **The citysim does not run under `--edit`.** Traffic exists only in Play, so
  edit-mode captures show no fleet line at all.
- **The viewer persists its camera on exit**, so a repeated capture can silently
  frame somewhere else.
- **Verdict lines beat argument.** When "the city is using the old cars" kept
  disagreeing with my probes, the fix was a log line that states which fleet was
  instanced *and its triangle count* — an unfalsifiable claim became a number.

---

## 5. Next steps, ranked

**1. Express the V-tier budget in sim seconds, not tick counts.**
Blocks the whole rate ladder. `frameIndex_ % vTickDivisor` counts sim ticks
("~1 Hz at the 60 Hz step"), so halving the sim rate halves how often distant
agents refresh; they go stale near the bubble edge and the tier pass
over-promotes (K/P 950 → 2650, and K agents are the drawn ones). Small change,
needs its own measured round. Then enable `citysim.localHz: 30` — the ladder,
extrapolation and gates are already in and green.

**2. Isolate the GPU passes.** Never measured: clouds, SSAO, SSR individually.
Add `RT_NO_CLOUDS` / `RT_NO_SSAO` / `RT_NO_SSR` (the codebase idiom —
`RT_CPU_EROSION`, `RT_SYNC_CDLOD`) and A/B them. 32–48 ms is above the 33 ms a
30 fps frame allows, so something here must give regardless of the CPU work.

**3. Building LOD.** ~8.5 M triangles is now mostly buildings, drawn in colour
and shadow passes. The HLOD swap exists (`detailDistance`); it deserves the
same scrutiny the parked cars just got — measure what fraction of those
triangles are beyond 300 m.

**4. Hot/cold agent split (SoA).** 432 bytes per agent, of which the hot loops
need ~60–80. Multiplies with the active list; the remaining sim cost is memory
traffic, not arithmetic.

**5. Sleeping parked agents.** Agents resting at home/work still take full
ticks. A scheduled wake (they already have depart windows) instead of a
per-step poll would drop the active set further — the shift work made this more
valuable, since the population is now spread across the day.

**6. Content follow-ups.** P8-G editable footprint in the viewport; the 12×12
world (now only recipe numbers — the mechanism is scale-free); `courtMinArea`
is consumed as documentation only; ~8 % of lots still green out via heavily
clipped ring fragments; acute block corners can stay unparcelled.

---

## 6. Open questions I could not answer

- Which GPU pass dominates (see next step 2). I have refused to name one
  without measuring it.
- Whether the ghost taillights are fully resolved — the box fleet baked lamps
  into the mesh *and* drew emissive lens instances; the generated body has real
  lamp housings, but this was never confirmed against a fresh binary.
- Cloud grain in dense banks after the Perlin-Worley upgrade (likely detail
  frequency vs the bilateral upsample).
