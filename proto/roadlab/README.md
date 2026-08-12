# roadlab — a parametric road system, prototyped from scratch

**This is a prototype and it is deliberately isolated.** Nothing under
`proto/roadlab/` includes anything from `src/`, and nothing in `src/` knows this
exists. It depends only on the STL plus two headers the project already
vendors (`nlohmann/json` for the scene format, `stb_image_write` for PNG). If it
turns out to be good it can replace the engine's road code later; until then it
can be thrown away without touching a line of the game.

It is headless: it renders to PNG, prints reports and runs traffic without a
window, so it builds and runs anywhere `make` does.

```bash
make roadlab                      # build ./roadlab
make roadlab-test                 # 3747 assertions, no framework
make roadlab-shots                # render every demo to out_roadlab/

./roadlab --demo showcase --view persp --out shot.png
./roadlab --demo grades --view persp            # skewed junction, four grades
./roadlab --demo lanes --view top --focus 90 4 220
./roadlab --scene proto/roadlab/scenes/example.json --report --lint
./roadlab --city --seed 3 --sim 60 --cars 200 --peds 60 --view top
./roadlab --demo interchange --debug strips     # false-colour the cross-section
./roadlab --demo interchange --xodr out.xodr    # export OpenDRIVE
./roadlab --import out.xodr --view top --report # ...and read it back
```

---

## The one idea

**A road is not a mesh. It is a parametric field over arc length.**

Every road carries a *reference line* (the spine) parameterised by `s`, and three
independent scalar functions of `s`: elevation, superelevation and lateral
offset. Any point on or above the road surface is `(s, t, h)` — arc length,
lateral offset in metres, height above the surface.

Everything else in the system is a function of those two or three numbers:

| consumer | uses (s, t) for |
|---|---|
| the mesh | vertex positions, and the UVs it carries |
| the shader | which strip you are in, where every stripe is, how worn it is |
| the simulator | where a vehicle is, who its leader is, when its lane ends |
| the placers | where a lamp, sign, signal or parking bay goes |

One coordinate frame, four consumers. That is what stops them from disagreeing.

Plan geometry is a chain of **line / arc / clothoid** primitives rather than
Béziers. Clothoids give curvature linear in `s`, which is what real road design
uses: constant steering rate through a transition, a derivable superelevation
runoff, and no step change in lateral acceleration for an AI driver.

---

## Four orthogonal axes, no type hierarchy

There is no `Freeway : Road`. A road "type" is a combination of four independent
things, and the presets in `profile.cpp` are just named combinations:

- **Geometry** — spine, elevation, superelevation (`spine.h`)
- **Profile** — the cross-section as a function of `s` (`profile.h`)
- **Topology** — how lanes connect (`network.h`, `junction.h`)
- **Semantics** — speed, access, control, signage rules (`props.h`)

`--list` prints the presets: `street2`, `street2_park`, `alley1`, `collector4`,
`arterial4`, `arterial6_median`, `boulevard_twltl`, `freeway2/3/4`, `ramp1/2`,
`roundabout1/2`, `country2`, `bridge2`, `tunnel2`, `service1`.

## The cross-section is a stack of strips

At any `s` the road is an ordered list of strips either side of the reference
line, each with a width that is a **cubic in s**:

```
shoulder | travel | turn | bus | bike | parking | median | apron
         | curb | gutter | verge | sidewalk | barrier | slope
```

That single idea covers most of what road systems normally special-case:

- **A lane appearing** is a strip whose width ramps `0 → W`.
- **A lane dropping** is `W → 0`, over a taper length computed from the design
  speed (MUTCD `L = W·S`, or `W·S²/60` below 72 km/h) — nobody types it.
- **Shoulders, medians, parking, footways, kerbs** are strips.
- **A turn bay** is a strip that only exists near a junction.
- **A two-way left-turn lane** is a pair of `Turn` strips straddling the centre.

Topology changes only at **lane section** boundaries; inside a section only
widths vary. Continuous enough to render, discrete enough to reason about.

**Lanes pair across a road boundary by where they physically are, not by
ordinal.** Two lanes are the same lane when their centres coincide at the joint.
That sounds pedantic until a deceleration lane peels off at a diverge: the lane
count changes on one side, an ordinal zip shifts every remaining lane one place
sideways, and the mistake is completely silent — in the lane graph *and* in the
export. `pairLanesAcross()` is the single implementation both read.

**Transitions between road types are solved, not authored.** `blendSection()`
runs a Needleman–Wunsch alignment over the two strip-kind sequences: matched
strips blend width, unmatched ones taper in or out, and a lane being dropped has
its outer marking forced solid — which is how a real taper is painted. The
solver also reports the shortest length the change can legally take.

## Composition, not primitives

Nothing below needed a new type (`builders.h`):

| thing | what it actually is |
|---|---|
| roundabout | a ring split into arcs + one junction per arm |
| on-ramp | an auxiliary lane growing from zero + one lane-level link |
| off-ramp | the same, reversed |
| overpass | an elevation profile + a bridge span |
| tunnel | an elevation profile + a tunnel span |
| multi-tier stack | two roads whose elevation profiles differ. Nothing else. |

Junctions are the only genuine special case, because the surface stops being a
ribbon there. A junction trims its arms back (by moving their `[sBegin, sEnd]`
window — the geometry is never destroyed), builds a pad polygon with rounded kerb
returns, and generates a **connector road** per turning movement, so a left turn
is a real curve of real width rather than a spline hack.

### The pad surface

This was the piece I expected to be hardest, and it was. A skewed junction whose
arms arrive at different heights, grades, crossfalls and banks cannot be covered
by an averaged plane without putting a lip at every approach.

The answer is **transfinite interpolation from the boundary**. Each boundary
vertex carries the height its own arm produced — the left and right corners of an
arm differ, which is how crossfall and superelevation come through — and the
interior is **mean value coordinates** over the pad polygon. MVC reproduces the
boundary exactly, is smooth inside, and unlike the Coons patch it generalises is
defined for any simple polygon. Points outside take the nearest boundary height,
so the terrain meets the kerb without a step.

The pad is triangulated by **ear clipping** (concave outlines from acute
multi-arm junctions are fine) and refined until no edge exceeds ~2.5 m, so the
interpolated surface has somewhere to curve. The `grades` demo is a deliberately
skewed four-arm junction built to exercise exactly this; a test asserts the pad
meets every arm at that arm's own surface height across the arm's full width, and
that the interior has no steps.

## Markings are a shader, not geometry

`surface.cpp` is a CPU reference implementation of a fragment shader, written to
port to GLSL/MSL almost line for line. The mesh under it is dumb: a ribbon whose
vertices carry `(s, t)` **in metres**.

- Materials come from which strip `t` falls in — procedural asphalt (cellular
  aggregate, fbm binder, worley patches, crack filaments), concrete with
  expansion joints every 4.5 m, gravel, grass, brick.
- **Lane markings are analytic SDFs about the boundary offsets `t_i(s)`.** A
  stripe is `abs(t - t_i(s)) - w/2`. Because `t_i(s)` is the actual lane-width
  polynomial, **a stripe follows a tapering lane automatically** — no authoring,
  no marking mesh, no z-fighting, crisp at any distance. There is a test that
  asserts exactly this.
- Dashes gate on `frac(s / period)`. Doubles, solid-dashed pairs, Botts dots,
  hatching and chevrons are all the same machinery.
- Glyphs — arrows, stop bars, zebra and ladder crossings, gore chevrons, parking
  tees, bike and HOV symbols, and stroke-font text (`ONLY`, `MERGE`, `BUS`, speed
  numerals) — are more SDFs placed in `(s, t)`.
- Wear erodes paint worst in the wheel paths; wheel paths also darken, polish and
  rut the asphalt, all as functions of `t` relative to the lane centre.
- `filterWidth` (the fragment's footprint in metres) does the antialiasing, and
  past the point where a stripe is thinner than a pixel it fades the stripe
  toward its average contribution instead of letting it alias. SDFs cannot mip;
  this is the substitute.

Junction pads shade in their own planar frame — there is no meaningful arc length
inside an intersection — with the same materials and the same glyphs.

## Everything else is derived

Nothing below is authored anywhere. All of it is a rule evaluated against the
parametric road, which is what makes it correct by construction:

- **Paint** (`generateRoadPaint`) — lane-use arrows from the movements a lane
  actually feeds; `ONLY` under a dedicated bay; merge arrows where a width
  function reaches zero; chevrons in the gore where one grows from zero; stop
  bars or yield teeth from the junction's control policy; zebras from the
  crosswalk rule; tees on every parking strip; speed numerals where the design
  speed changes.
- **Props** (`generateProps`) — lamps every 30 m alternating sides, on the
  parapet where the road is a bridge and absent in tunnels; signals at signalised
  approaches; STOP or GIVE WAY from the same control policy the simulator
  arbitrates with; `LANE ENDS` exactly one taper length upstream of a taper;
  curve-warning and advisory-speed plaques carrying the number the geometry
  produced; trees on verges, meters beside parking, hydrants on footways.
- **Parking slots** — a rule over the parking strip, the same strip the shader
  paints tees on.
- **Signal phases** — a greedy colouring of the junction's crossing-conflict
  graph. There is a test asserting that no phase ever contains two movements that
  cross.

## Simulation-ready by construction

The simulator (`sim.h`) exists to prove the data model is sufficient, not to be a
finished traffic model. It adds **no new road data**:

- Vehicles are `(lane node, distance travelled)` — the same frame as the shader.
- Lanes keep an s-ordered occupant list, so leader/follower is O(1). IDM
  car-following, with the leader search walking downstream through the lane graph
  so a queue at a junction is visible to the car approaching it.
- **Lane-change legality reads the marking** — the same field that draws the
  line. A solid line is uncrossable in the sim for exactly the reason it looks
  uncrossable on screen.
- A lane whose successors are empty is a mandatory lane change. The taper the
  renderer draws and the "this lane ends" the driver reads are the same fact.
- Right of way reads the junction conflict table — the same table the signal
  phases were coloured from. Signals, yields, priority-stop, all-way stop and
  roundabout circulating priority all run off it.
- Parked cars occupy the slot rule's output — the same strip the shader paints
  bay tees on. A test asserts no parked car ends up outside a parking strip.

### Routing, and the query the two-resolution graph exists for

Vehicles plan with Dijkstra over the **lane** graph by travel time and follow the
plan through junctions. That makes the second resolution earn its keep:
`routeDemand` walks the plan to find the first road the current lane *cannot*
reach, and how far away it is becomes the urgency behind a mandatory lane change.
A driver who needs a lane will accept a tighter gap — but still will not cross a
solid line, so missing an exit is possible, which is correct.

The test for this is the honest one: on the interchange, `lanesReaching` for the
off-ramp returns a non-empty **strict subset** of the mainline's lanes, and a lane
outside that subset genuinely cannot reach the ramp. If every lane could, the
query would be meaningless.

### Pedestrians

Sidewalk strips are graph edges. The links between them are either a **corner**
(round the kerb returns, ordered by bearing so the footway actually wraps the
intersection) or a **crossing** — and the crossings come from
`junctionCrosswalks()`, the same call that generates the zebra paint, so a
pedestrian can only cross where a crossing is painted.

Deciding to cross is a commitment: they wait at the kerb until it is safe.
Safety comes from two places, neither of which is new data:

- at a signalised junction the pedestrian reads the **same phase table** the
  drivers obey — walkable exactly when no movement using that arm's carriageway
  is green. A test walks a whole cycle asserting the two agree;
- elsewhere it is gap acceptance against real vehicles, with patience shrinking
  the gap they will take so a busy kerb resolves instead of deadlocking.

And drivers give way: a vehicle closing on an occupied crossing treats it as an
obstacle. The crossing the driver stops for is the crossing the shader painted
and the pedestrian walked.

## OpenDRIVE export

`--xodr out.xodr` writes the network as OpenDRIVE 1.7. The model was built to be
convergent with the standard on purpose — reference line of line/arc/clothoid,
elevation and superelevation as cubics in s, lane sections of width polynomials,
signed lane ordinals, junctions as sets of connecting roads — so the exporter is
mostly transcription. That is the point: if it had needed a translation layer,
the model would have been drifting away from the one format the industry reads.

Coordinates line up without a transform. This prototype is Y-up with the plan in
XZ and heading from +X toward +Z; OpenDRIVE is Z-up with the plan in XY and
heading from +X toward +Y. Both rotate the first plan axis toward the second and
both put +t to the left, so `(x, z, y)` maps onto `(x, y, z)` with the same
signs — including superelevation.

Roads export over their **active window**, so a junction-trimmed arm or a split
piece becomes a road starting at `s = 0` with every piecewise function re-based
(`Poly3::shifted`), including re-deriving a clipped clothoid's start pose and end
curvature. Tests assert the geometry lengths sum to each road's declared length
and that a trimmed arm's first geometry sits at the trim pose, not the original
road start.

### Ramps have to become junctions

OpenDRIVE allows a road at most one predecessor and one successor, and requires a
`<junction>` wherever the connection is **not one-to-one**. Internally a ramp is
an `ExtraLaneLink` — one lane continuing into another, which is what a merge
physically is and which avoids inventing a junction with no pad. But a diverge
leaves the mainline with two successors, and that is exactly what the format
forbids.

So export builds a **plan** before it writes anything: a list of road windows in
which merges and diverges have been replaced by a junction whose connecting roads
are short **stubs** carved off the front or back of the branches. No new geometry
is invented — a stub is another window over a spine that already exists, the same
trick junction trimming and road splitting use. The interchange demo, which has no
authored junctions at all, exports as two junctions and four stubs.

A conformance test enumerates the whole document and asserts the invariant
directly: for every road, everything that attaches to a given end must be the one
road it names there, or must be a connecting road of the junction it names there.
That is the check that would have caught the first version of this exporter,
which wrote a file whose ramps were unreachable.

## OpenDRIVE import

`--import town.xodr` reads a `.xodr` back into a `Network`. This is the half that
meets data nobody here wrote, so it reports rather than guesses silently:
`OdrImportReport` counts roads, junctions, lane sections and lanes, and carries
notes for the things it had to approximate.

Two pieces make it work:

- **Anchored geometry.** Our spines chain: each primitive starts where the last
  one ended. A file's records each declare their own `x/y/hdg`, and real files
  disagree with the chain by millimetres to metres. `GeomPrim::anchored` keeps a
  record's declared pose instead of chaining, so the imported reference line is
  the file's, not a re-derivation of it — and the drift between the two is
  measured and reported rather than silently absorbed.
- **Adopted junctions.** An imported junction arrives with its arms already
  trimmed and its connectors already built. Re-resolving it would trim a second
  time and duplicate every connector, so `Junction::imported` routes `build()` to
  `adoptJunction`, which measures the arms where they are and derives only what
  the file does not carry: the pad polygon, the conflict table, priority and the
  signal phases. Those four steps are shared with `buildJunction` rather than
  reimplemented.

A minimal XML reader (`rl_xml.h`, ~200 lines) sits underneath. `.xodr` uses a
small, well-behaved subset — elements, attributes, self-closing tags, comments,
five entities — and parsing that is cheaper than adding a dependency to a
prototype meant to stay buildable with the STL alone.

### What the format cannot carry, and what to do about it

Three things matter to this model and have no slot in OpenDRIVE. Two of them fit
existing elements once you look:

- A lane's **material** goes in `<material surface=…>` and a kerb's **height** in
  `<height inner= outer=…>`. Without those a re-imported network comes back as
  flat asphalt, verges and all.
- A **two-way left-turn lane** is `type="bidirectional"` — the standard has the
  exact concept, and spelling it `"driving"` turns a TWLTL into two extra lanes
  the simulator will happily drive down.
- **Junction control** lives on the approaches, as `<signal>` elements — which is
  also where it lives in the world. The `type` is the standard code so any reader
  knows what the sign is; the `name` carries our own enum so a round trip does not
  have to reverse-engineer control from sign codes.

Only *ramp* and *circulatory carriageway* have nowhere to go, since OpenDRIVE has
no concept of either; they use the format's sanctioned `<userData>` extension
slot rather than being thrown away, because the difference decides who yields.

### Round trip

The first pass through the file legitimately changes the representation: a
lane-continuation merge becomes a junction with connecting roads, because the
format requires it. So the invariant worth asserting is not "identical after one
pass" but **a fixed point** — and that is the stronger claim, because it says the
reader and the writer agree on a canonical form. Every demo reaches it: export,
import, export again, and the second document is byte-identical to the first.

Alongside that the tests assert the physical claims: total centreline within
0.5%, every sampled surface point back within 0.2 m in three dimensions (a
plan-only query picks the wrong deck wherever one road runs over another),
lane-metres within 1%, arms not re-trimmed, connectors not duplicated, control
and turn classification recovered — and finally that traffic will drive on the
result, including through the junctions the exporter synthesised.

Structurally, five of the seven demos come back with an *identical* report. The
other two are the ones with ramps, and they differ exactly by the junctions the
format demanded.

### What it found

Pointing the importer at our own exporter's output surfaced a bug in
`Spine::toST` — the world-to-`(s, t)` inverse — that had been there from the
beginning. Its Newton step was `cur -= -f/fp` where it should have been
`cur += -f/fp`: the iteration walked *away* from the foot of the perpendicular,
doubling the seed's error every pass.

It never failed loudly. An earlier fix had added a residual check that rejects an
answer with a non-zero tangential component, which is exactly what a diverged
Newton produces — so the inverse map did not return wrong answers, it returned
*no* answers, on every query, and every caller took its fallback path. Terrain
stopped conforming to roads, `Network::sample` never found anything. Fixing the
sign took the test suite from 32 s to 4.6 s, because eight useless iterations per
query were being run and then thrown away.

The round trip caught it because it is the first test that asks the inverse map a
question with a known right answer, across the whole network, and checks it.

## Performance

Terrain generation was 95% of all runtime. Three fixes gave about 8x: the surface
normal now comes from grid heights the loop already computed rather than four
extra evaluations of the height field per vertex; a uniform bucket index over the
roads means "which roads are near this point" stops meaning "all of them" (which
also speeds up every re-localisation the simulator does); and bounding-box
rejects plus scratch reuse cut the per-junction work. The spine inverse uses a
coarse-to-fine seed search rather than a flat scan.

The urban demo's terrain went from 6.5 s to 1.2 s and the whole test suite from
57 s to ~32 s — and then to 4.6 s once the `toST` sign bug above was fixed and
the inverse map stopped doing eight wasted Newton iterations per query. The route-reachability query is cached on a 0.4 s timer per
vehicle — a driver does not re-ask the graph ten times a second either.

## Design lint

`--lint` runs the checks that make authoring safe (`Network::validate`):

- radius below the minimum for the design speed (`R = v²/127(e+f)` — the *same*
  formula the ramp and connector generators use to pick their speeds, so a
  generator can never emit a road its own lint rejects)
- grades over the limit for the road class
- tapers sharper than 1:12 (1:8 for turn bays, which are deliberately shorter)
- **vertical clearance between stacked roads** — the check that makes multi-tier
  authoring safe, scoped so that junction internals, split siblings and ramps
  merging into their own mainline are not reported as violations
- two roads crossing at grade with no junction declared

Every demo and every generated city seed passes it clean; the tests assert that.

---

## Layout

```
rl_math.h/.cpp     Vec2/Vec3, Poly3, noise, Gauss-Legendre. No engine deps.
rl_xml.h/.cpp      a minimal XML reader, sized for .xodr and nothing more.
spine.h/.cpp       line/arc/clothoid chains, elevation, superelevation,
                   (s,t,h) -> world and the inverse.
profile.h/.cpp     strips, lane sections, presets, the transition solver.
network.h/.cpp     roads, links, splitting, the lane graph, the design lint.
junction.h/.cpp    arm trimming, pad polygons, connectors, conflicts, phases.
builders.h/.cpp    roundabouts, ramps, lane changes, bays, overpasses, tunnels.
structure.h/.cpp   decks, piers, parapets, bores, portals, terrain conform.
surface.h/.cpp     THE SHADER: materials, SDF markings, glyphs, paint rules.
tessellate.h/.cpp  dumb ribbons carrying (s,t); junction pads; sweeps.
raster.h/.cpp      z-buffer rasteriser + PNG. Exists so you can look at it.
props.h/.cpp       rule-driven lamps, signs, signals, furniture.
sim.h/.cpp         IDM traffic, lane changes, junction arbitration, pedestrians.
odr.h/.cpp         OpenDRIVE export (with the road plan) and import.
scene.h/.cpp       JSON authoring, the demo set, the city generator.
main.cpp           the CLI.
tests.cpp          3747 assertions, invariant-focused.
```

## Known gaps

Honest list of what a prototype this size does not do yet.

- **Import has only been fed our own exporter's output.** The round trip reaches
  a fixed point on every demo, but the next real test is a third-party `.xodr`
  from a cloverleaf someone else authored. `poly3`/`paramPoly3` geometry is
  sampled into segments rather than represented exactly, and a lane with more
  than one `<width>` record keeps only the first; both are counted and reported
  rather than silently dropped.
- **Only junction control is exported as `<signal>`.** The generated signs and
  lamps are still props, not `<objects>`.
- **Hatching and chevrons have no OpenDRIVE equivalent** and export as `none`.
  A faithful export would emit them as `<object>` surfaces.
- **Very acute or self-intersecting pad outlines** fall back to a centroid fan.
  Ear clipping handles ordinary concave pads, but a pathological boundary from a
  near-parallel pair of arms is covered rather than solved.
- **Strip heights do not blend through a transition.** A cross-section change
  that also changes a kerb height steps rather than ramps.
- **No LOD, no tiling, no streaming.** The shader's distance-fade is the only LOD
  in the system. A real version would bake the procedural surface into a virtual
  texture or clipmap around the camera.
- **Pedestrian crossings are derived from the vehicle phases** rather than having
  their own phase with a clearance interval. Defensible, and it guarantees the
  two agree, but it is not what a real controller does.
- **Vehicles only yield to pedestrians on arms**, not on the connector roads
  inside a junction — a pedestrian on a crossing is invisible to a car already
  committed to a turn.
- **`saveSceneJson` writes a summary, not a reloadable scene.** Authored scenes
  round-trip through the hand-written JSON only.
- **`toST` still has no per-agent cache.** The spatial index and the
  coarse-to-fine seed made it cheap enough; a production version would also cache
  the last road per agent.
