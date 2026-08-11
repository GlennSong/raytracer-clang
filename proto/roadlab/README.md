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
make roadlab-test                 # 844 assertions, no framework
make roadlab-shots                # render every demo to out_roadlab/

./roadlab --demo showcase --view persp --out shot.png
./roadlab --demo lanes --view top --focus 90 4 220
./roadlab --scene proto/roadlab/scenes/example.json --report --lint
./roadlab --city --seed 3 --sim 60 --cars 200 --peds 60 --view top
./roadlab --demo interchange --debug strips     # false-colour the cross-section
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
- Pedestrians walk the footway strips; parked cars occupy the slot rule's output.

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

- **Junction surfaces between differently-graded approaches.** The pad's height
  is an inverse-distance blend of the arms. A Coons patch is the right answer for
  a skewed junction where each arm arrives at its own grade. This is the piece I
  would prototype next; it is the one place the model is genuinely hard.
- **Concave junction pads** are triangulated as a fan from the centroid, which
  assumes near-convexity. Very acute multi-arm junctions will need a real
  triangulation.
- **Strip heights do not blend through a transition.** A cross-section change
  that also changes a kerb height steps rather than ramps.
- **`toST` is a linear scan** over the sample cache per road. Fine at prototype
  scale; a production version wants a spatial index and a per-agent cache.
- **No LOD, no tiling, no streaming.** The shader's distance-fade is the only LOD
  in the system. A real version would bake the procedural surface into a virtual
  texture or clipmap around the camera.
- **The sim has no routing.** Vehicles pick successors at random. The lane graph
  supports the real query (`lanesReaching`) but nothing drives it yet.
- **Pedestrians walk their own road's footway** and turn around at the ends; they
  do not cross at the crosswalks the paint generator emits.
- **No OpenDRIVE import/export.** The model is deliberately convergent with it,
  and this is the highest-value next addition: it would buy real-world networks
  to test the junction solver against.
