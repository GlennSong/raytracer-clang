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
make roadlab-test                 # 2406 assertions, no framework
make roadlab-shots                # render every demo to out_roadlab/

./roadlab --demo showcase --view persp --out shot.png
./roadlab --demo grades --view persp            # skewed junction, four grades
./roadlab --demo lanes --view top --focus 90 4 220
./roadlab --scene proto/roadlab/scenes/example.json --report --lint
./roadlab --city --seed 3 --sim 60 --cars 200 --peds 60 --view top
./roadlab --demo interchange --debug strips     # false-colour the cross-section
./roadlab --demo interchange --xodr out.xodr    # export OpenDRIVE
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

## Performance

Terrain generation was 95% of all runtime. Three fixes gave about 8x: the surface
normal now comes from grid heights the loop already computed rather than four
extra evaluations of the height field per vertex; a uniform bucket index over the
roads means "which roads are near this point" stops meaning "all of them" (which
also speeds up every re-localisation the simulator does); and bounding-box
rejects plus scratch reuse cut the per-junction work. The spine inverse uses a
coarse-to-fine seed search rather than a flat scan.

The urban demo's terrain went from 6.5 s to 1.2 s and the whole test suite from
57 s to ~32 s. The route-reachability query is cached on a 0.4 s timer per
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
scene.h/.cpp       JSON authoring, the demo set, the city generator.
main.cpp           the CLI.
tests.cpp          844 assertions, invariant-focused.
```

## Known gaps

Honest list of what a prototype this size does not do yet.

- **No OpenDRIVE import.** Export works; reading real networks back in is the
  highest-value next addition, because it is what would let the junction solver
  be pointed at a real cloverleaf instead of at scenes I wrote myself.
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
