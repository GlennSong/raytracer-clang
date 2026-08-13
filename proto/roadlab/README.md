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
make roadlab-test                 # 23345 assertions, no framework
make roadlab-shots                # render every demo to out_roadlab/
make roadlab-wgsl                 # regenerate rl_paint.wgsl from rl_paint.h
make roadlab-wgsl-check           # + validate it with naga (cargo install naga-cli)
make roadlab-wgsl-conformance     # run the WGSL and diff it against the C++

./roadlab --demo showcase --view persp --out shot.png
./roadlab --demo grades --view persp            # skewed junction, four grades
./roadlab --demo lanes --view top --focus 90 4 220
./roadlab --scene proto/roadlab/scenes/example.json --report --lint
./roadlab --city --seed 3 --sim 60 --cars 200 --peds 60 --view top
./roadlab --metro --seed 11 --view top --report --lint   # bypass + interchanges
./roadlab --demo interchange --debug strips     # false-colour the cross-section
./roadlab --demo interchange --xodr out.xodr    # export OpenDRIVE
./roadlab --import out.xodr --view top --report # ...and read it back
./roadlab --fallbacks                           # census every quiet substitution
./roadlab --demo grades --view persp --terrain-amp 20   # real relief, real earthworks
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

## A metro, not a grid with a freeway next to it

`--metro` generates the layout `--city` cannot: a freeway **bypass that wraps the
city**, carrying an interchange per radial arterial, around a street network of
genuinely different-sized blocks.

The structural difference is which thing gets planned first. `--city` lays a grid
and puts a freeway beside it. A bypass has to enclose the network, so the ring is
sized first — and sized by its **design speed**, not by the city: a ring below the
minimum radius for 110 km/h is not a bypass, so a small town gets a ring standing
further out rather than a tighter one. Interchanges are then placed on it, and
each one pulls a radial arterial inward until it meets the grid.

Three things that took a correction each:

- **The ring is arcs, not a filleted polyline.** A polyline's corner radius is
  whatever fits between adjacent points, so asking for 560 m on a polygon with
  room for 400 gets 400 — and the lint then correctly reports a motorway below
  its minimum radius. Putting the vertices on the circumscribing polygon does not
  save it either: the spiral transitions need more tangent than an exact fit
  leaves. An arc spine simply is the radius it says it is.
- **Standoff is measured from the half-diagonal.** Measured from the half-width,
  the ring clears the middle of each side by the full standoff and the corners by
  almost nothing, and a radial aimed at a corner has nowhere to exist.
- **One closed carriageway, not two halves.** Every seam is a place a ramp cannot
  go. Ramps split the ring as they are added, and the pieces stay windows over
  one circular spine with absolute stations — which is what lets an interchange
  planned at a station still find its piece after the ones upstream have already
  cut the road up.

Blocks vary because the cell spacing is irregular *and* because whole rectangles
of interior street are removed to leave superblocks. Streets curve because the
lattice is warped with smooth noise — coherently, so a street curves as one line
instead of wiggling between junctions — with individual edges bowed on top and
the rest left dead straight for contrast. Every third line is an arterial and the
middle one of each axis is a divided boulevard, so roughly half the network is
four lanes or more.

A default metro is 655 roads, 37 km of centreline, 94 lane-km, 62 junctions and
five interchanges, and it lints clean. It exports to 2.2 MB of OpenDRIVE, comes
back with every metre intact, and reaches the same byte-identical fixed point the
demos do — the ramps becoming the ten junctions the format requires.

### What that exercise found

Scale broke three things that the demos never did, all of the same family: **two
places that had to agree about lanes, and did not.**

- **Lane sections linked by ordinal.** `pairLanesAcross` had already been fixed to
  pair by position at a road boundary; `linkSections`, which does the same job at
  a *section* boundary inside one road, still matched by ordinal. Insert an
  auxiliary lane mid-stack and every kind comparison from that point outward
  mismatches, so every lane outboard of it links to nothing. Traffic cannot drive
  past an on-ramp, and nothing reports it, because a lane whose successor is
  `kNoLane` is the legitimate way to say "this lane ends here".
- **Zero-length lane sections.** Replaying cross-section edits lands one edit's
  end exactly on the next one's start. The section that results is never in force
  anywhere, but the lanes either side link *to it* rather than to each other. The
  threshold that dropped them and the threshold the lane graph used to decide a
  section was worth a node had drifted apart — 1 mm against 0.25 m — so a 5 cm
  section survived the first and failed the second. They are one constant now.
- **A road window ending just past a section boundary.** Splitting for a ramp
  lands wherever the merge geometry says, not on a section edge, which leaves a
  sliver of the next section inside the window. The lane graph emits no nodes for
  a sliver, so anything resolving "the section at this end" by station found a
  section with nothing in it and linked nothing. `endSectionIndex` resolves to the
  section that actually carries nodes, and `pairLanesAcross` now takes both the
  lane ids and their positions from it — taking ids from one section and geometry
  from another matched lanes that were never in the same stack.

Two more turned up in the same place, and finished the job:

- **The taper and the steady state disagreed about which lane was new.**
  `sectionWithLanesChanged` adds an auxiliary lane at the OUTER edge of the
  travel run; the Needleman–Wunsch alignment that builds the transition between
  the two sections resolves the tie — which of four Travel strips is the surplus
  one — at the INNER end, arbitrarily. So the taper introduced a lane the steady
  section did not have in that slot: the through lanes shifted one place, the new
  lane inherited their traffic, and one real lane was starved at every
  transition. On a ring with five interchanges that is the whole carriageway. The
  alignment now re-pairs each same-kind run from its inner end so the surplus
  falls out at the outer end, matching where the lane was actually added.
- **The ramp's lane-level link resolved its sections by station.** The same
  sliver problem as above, in the one code path that had not been fixed: an
  on-ramp that renders perfectly and that no vehicle can ever use to join the
  mainline.

Together the five took freeway-to-street connectivity from 1 seed in 8 to **10 of
10, in both directions, for every street** — asserted per street rather than
sampled, because one reachable street passes while the rest of the city is
stranded.

There is a lesson in how long that took. Twice I "fixed" the alignment tie and
measured it as catastrophically worse, and reverted. Both times the measurement
was wrong, not the fix: the search followed lane successors only, and a driver
reaches an exit lane by **changing into it**. A deceleration lane has no
predecessor by construction — it grows out of the carriageway rather than
continuing anything — so a successor-only search reports that no exit is
reachable from any freeway. That says more about the search than the road. The
reachability check now follows lane changes, and with the alignment fixed the
same measurement returns 10 of 10.

## Earthworks: the ground meets the road on a batter

The prototype always conformed the terrain to the roads, but it did it with a
**fixed-width feather** — a smoothstep from the road's edge height out to natural
ground over a constant number of metres. That is the wrong shape, and the reason
is worth stating plainly: a fixed width makes the slope depend on how deep the
earthwork happens to be. A 20 cm difference becomes a lazy ramp; a 4 m difference
becomes a wall. Measured on the demos as shipped, the ground beside a road
reached **76 degrees**, and with real relief **83 degrees**.

The worst case was not even the deep ones. It was a road **END**: the conform
tested `s` against the road's window and gave up outside it, so where a road
simply stopped, the ground fell from the carriageway to natural ground in half a
metre. The inverse map reports a miss for a point off the end of a spine — there
is no perpendicular foot — and that miss was read as "this road is irrelevant
here".

What the engine does instead (ADR-0075, `TerrainFlatten::Falloff::DaylightBatter`)
is run a real earthwork: a slope at an **authored gradient** that continues until
it reaches natural ground, where it *daylights*. That idea is what came across.

Each road and junction pad claims a **band** of heights at a query point: at
distance `d` from its footprint the ground may be as high as `yEdge + cut·d` and
as low as `yEdge − fill·d`. At `d = 0` the band collapses to the road's own
surface, which is what makes the ground meet the kerb exactly. Far away it
constrains nothing. The ground is then the natural height clamped into the band.

Bands **combine by intersection**, not by the weighted average the old code used,
and that is the substantive change. An average of two surfaces is a third surface
that is neither, and its gradient is unbounded; an intersection of two batters is
a batter. Defaults are 1:1 for a cut (undisturbed material stands steeper) and
1:2 for a fill.

Two details carry most of the improvement:

**Distance is to the footprint, not across the road.** Clamping the station into
the road's window and the offset into its cross-section gives the nearest point
of a 2D region, which has ends as well as sides. The end cliff disappears without
a special case — walking off the end of a road now measures **0.56** against a
1.00 batter, where it used to measure **8.17**.

**Where the earthwork cannot daylight, that is a retaining wall.** Three
situations qualify: two roads at different levels closer together than their
batters can reconcile; a batter that hits its reach before it meets ground; and
an **abutment**, where the approach embankment claims the ground and the deck
beside it does not. roadlab does not build walls, so a step remains — and each is
counted in the fallback census rather than smoothed away, because a silently
smoothed wall is a lie about the geometry.

The test asserts the property rather than a picture: walking outward from every
road edge in every demo at three terrain amplitudes, no ground is steeper than
its authored batter *or* the hillside it sits on, whichever is greater — and
that the bound moves when the batter is re-authored, so it cannot pass by the
terrain simply being flat. Demos with no structures hold that strictly.

### Abutments, and what footings stand on

An **abutment** is where earth meets structure: the approach embankment carries
the ground at deck level, the deck itself carries nothing, and between them is a
vertical face. roadlab already builds the concrete for that (`tessellateStructures`
emits an abutment box at each end of a span), so the step is correct rather than
missing — but code that measures ground gradients has to tell it apart from a
batter that has gone wrong. It is marked on **both** sides of the transition; the
first attempt marked only the structural side, which missed the point entirely,
since the face is *between* the two and a sample on the embankment side measures
across it just as much as one on the deck side.

Fixing that turned up a separate bug worth naming: piers and abutments sized
their footings against a **default-constructed** `TerrainParams`, not the scene's
— and against the raw noise rather than the conformed ground. So raising a
scene's relief left its bridges standing on whatever height the default noise
happened to give. They now stand on `terrainHeightAt` with the scene's own
terrain, which is also the right surface: under a deck the ground is natural,
because the deck does not claim it.

A bounded residual remains: worst **2.05** against a 1.00 batter, a handful of
samples where two roads at different levels sit at the edge of a tall
embankment. Asserted at 2.5 so a regression is visible.

### The terrain grid cannot draw a wall

Where the earthwork legitimately steps — an abutment face — the ground is drawn
by a uniform grid whose cell is a few metres, so a 4.5 m step becomes a
**staircase** of grid cells rather than a face. It is a mesh-resolution problem,
not an earthwork one: the heights are right and the test agrees, but the surface
sampling them cannot represent a discontinuity. The fix is an adaptive grid that
refines near roads and across wall sites; it is not done.

## The roundabout's aprons

The arms of the `roundabout` demo stopped 26 m from the ring's reference line
while the ring's outer edge is about 9 m out — so the junction had to bridge a
17 m gap, and every "entry" rendered as a plain of tarmac with the give-way line
somewhere in the middle of it. The stand-off is now derived from the ring's own
cross-section rather than typed, leaving the few metres the kerb return needs.
Worth noting the failure mode: the lint was clean throughout, because a junction
that spans a large gap is not invalid — just wrong.

## Getting the markings onto a GPU

The engine already shades roads this way. `shaders/metal/surface_road.metal`
paints centrelines, edge lines, dividers and crosswalks analytically in the
fragment shader from a road-local UV, under `Surface::RoadMarkings` (ADR-0044),
with a `shaderMarkings` flag in `road_mesh.h`. So this is not a renderer rewrite;
it is an upgrade to both halves of a seam that exists.

What changes is one line — `road_mesh.cpp:1441`:

```cpp
gU[gi(i,j)] = 2.0 + lateral / std::max(0.5, hw);   // 1 left, 2 centre, 3 right
```

The engine's lateral coordinate is **normalised by half-width**. Markings sit at
fractions of the road, so they stretch when it widens, and a lane that appears
cannot be expressed. Here `t` is metres.

`rl_paint.h` is the half that has to run on a GPU, in a subset that compiles as
C++, MSL and GLSL unchanged, and that `tools/roadlab-wgsl.py` translates to WGSL.
Three constraints shape it, all from the GPU side: no containers, so boundaries
arrive as a flat fixed-size array something else filled in; no noise, so wear's
blotch mask is an input and each backend spends the value noise it already ships;
and float, not double. `surface.cpp` now calls it, so there is one implementation
rather than a reference and a port that drift.

The important thing this de-risks: the expensive part of a road shader is the
asphalt, not the paint. The engine already has `surfAsphalt`/`vnoise2` and
already affords it. Only the markings need porting, and they are a handful of
`abs()` and `smoothstep()` per boundary.

### Two things the bake had to be measured for

`paint_bake.h` turns a cross-section into that flat array. Testing it against the
exact cross-section — which is the whole reason to build it before touching a
shader — produced two results that changed the design.

**A ring is required on both sides of every lane-section boundary.** Slot *k* of
the baked array is only the same physical boundary at both ends of a step if the
boundary *set* is unchanged, and it changes discontinuously at a section seam: a
lane appears and every slot outboard of it shifts one place. Interpolating across
that blends two unrelated boundaries. Measured on the `lanes` demo it dragged
paint **0.78 m** sideways; with a ring pair straddling each seam the worst error
over a 2 m step is **8 mm**, against a 100–150 mm stripe. That is now a stated
requirement on whatever meshes the road.

**Bake only the painted boundaries.** A full cross-section runs to **17**
boundaries once kerbs, gutters, verges and slopes are counted, but never more
than **nine** carry paint. Baking all 17 is 136 floats per station, which settles
the vertex-attribute question against itself; nine is 18 RGBA texels, which is
nothing. So the recommendation is a **profile texture**, not vertex attributes —
a conclusion measured rather than guessed, and the opposite of where I started.
Filtering also broke the hatch styles, which fill outward to their neighbour and
so needed a boundary the shader would no longer be given; the fill extent is now
resolved at bake time and travels in a field the area styles do not otherwise
use, which removes the shader's only dependence on array ordering.

Tests assert the invariants a GPU port depends on: coverage never rises as the
filter widens (no shimmer), a dashed line averages to its duty cycle and stops
varying with `s` once sub-pixel (no crawl), the painted boundaries fit the fixed
record, and interpolated paint matches exact paint across every demo.

### The profile texture

`paint_texture.h` is where the fragment actually gets its array. Two textures,
addressed by one interpolated scalar:

| | width | rows | sampler | holds |
|---|---|---|---|---|
| profile | 4 texels | one per mesh ring | **linear** for texels 0–2, **nearest** for texel 3 | twelve lateral offsets, then a style row index |
| styles | 24 texels | one per **distinct** style set | **nearest** | style, width, gap, colour, dash pattern, wear |

The split is the design. `t` is the one field continuous in `s`, so it is the one
field a linear filter should touch; a blend of two style codes is not a third
style, it is a number that selects a branch at random.

The indirection through `styleRow` is what makes it scale. Style, width, dash and
colour change only at lane-section seams, so a row-per-station style table stores
the same 96 floats over and over. Deduplicating across the network collapses it
by the ratio of stations to distinct cross-sections: on the `urban` demo **3218
station rows fold to three style sets**, and the atlas drops from 1357.6 KiB to
**202.2 KiB**. Extrapolated to 500 km of road at the mesher's 2 m ring spacing
that is ~16 MiB, or half that as RGBA16F — a texture that scales with the number
of road *types*, not with kilometres.

Rows are not uniformly spaced, so a fragment cannot compute its row from `s`. The
mesher emits one ring per row and writes the row index into a vertex channel; the
rasteriser interpolates it. That is the same trick `road_mesh.cpp` already uses
for its road-local U, and it is why the seam ring pair matters twice over — it
keeps slot numbering stable across a step *and* gives the mesh somewhere to put
the discontinuity.

It does not remove that discontinuity; nothing can. So the confinement is
measured rather than assumed: across the 12 interior seams in the demos, the
widest band where interpolated paint disagrees with the cross-section is
**0.90 mm**, inside the 1 mm ring pair. Building the texture also exposed two
bugs in the strip it reads from — a slot live at one end of a step and padding at
the other was interpolating its offset from zero, sweeping a marking out from the
middle of the road, and `sampleInterpolated` was blending fields that a sampler
pair cannot blend.

### WGSL is generated, not shared

Metal and GLSL include `rl_paint.h` verbatim. WGSL cannot: `float x` is
`x : f32`, `T f(a)` is `fn f(a) -> T`, struct members are comma-separated, and a
pointer parameter needs an address space. So `tools/roadlab-wgsl.py` translates
it, and `rl_paint.wgsl` is checked in.

The generator's most important property is that it **refuses** anything outside
the subset rather than guessing — a guess would emit WGSL that compiles and
shades the road differently from the CPU, which is precisely the failure the
shared-source exercise exists to prevent:

```
$ tools/roadlab-wgsl.py
rl_paint.h:202 refuses translation (ternary — use rlMax/rlMin or an if)
    float energy = w > 0.0f ? w / effW : 1.0f;
```

Making that refusal cheap meant shrinking the shared subset instead of growing
the translator. `rl_paint.h` now has no ternaries and no pointer out-parameters:
every comparison goes through `rlMax`/`rlMin` (the only two functions the
generator maps rather than translates, straight onto WGSL builtins), and
`rlPaintColor` returns a small struct. Neither is worse in C++, and together they
took the translator down to something that fits in a page and fails loudly.

Two guards keep the pair honest without any toolchain. The generated file records
an FNV-1a digest of its input, and the test suite recomputes it — a header edit
nobody regenerated fails the build rather than shipping. And because a digest
only proves the file was regenerated, not that regenerating produced anything,
the same test walks the header for every `RL_FN` and `#define RL_*` and requires
each in the WGSL, checks brace balance, and greps for C that survived
translation.

### It parses, and it runs

Both of those are structural. Two stronger checks are available headlessly, and
neither needs a GPU:

`make roadlab-wgsl-check` runs the output through **naga** — the WGSL front-end
wgpu and Firefox use — and then asks it to lower the module to SPIR-V and MSL,
because the backends enforce things the parser lets through. The subject is
`rl_paint.wgsl` concatenated with `rl_paint_sampler.wgsl`, a hand-written
companion that implements the paint_texture.h contract: the bindings, the texel
arithmetic, and a fragment entry point. That is deliberate — validating the
evaluator alone proves it type-checks, not that a fragment can reach it. naga is
not vendored (`cargo install naga-cli`); without it the step says so and skips.

`make roadlab-wgsl-conformance` goes further and **executes** the shader.
`roadlab --paint-fixture` dumps 12,870 evaluator inputs with the CPU's answers —
real cross-sections from every demo, plus synthetic rows covering the styles the
demos never reach, across filter widths from 2 mm to 8 m — and a compute pipeline
replays them. wgpu falls back to Vulkan on llvmpipe, so this runs on a machine
with no graphics hardware at all.

That is the check that earns the "one evaluator" claim, because a translation can
be flawless WGSL and still paint a different road. It was tested by breaking it:
swapping `dashOn` and `dashOff` in the struct moves the worst difference to 1.0,
and changing a single constant from 0.55 to 0.56 moves it to 2.1e-2 — both far
outside the gate.

### What running it found

95.05% of cases match bit for bit and the mean difference is 3.1e-8, but 18 cases
disagreed by up to 2.7e-4 — all of them in the *periodic* styles (dashed, hatch,
chevron), and the error grew with station: 6e-8 near a road's start, 2.7e-4 at
s ≈ 386 m. Solid and double lines, which never wrap, were exact.

The cause is `rlWrap`. Written `x - period * floor(x / period)`, the product is a
number the size of `x` — 270 for that hatch — so rounding it costs an ulp *at
that scale*, about 2.3e-5, and all of it lands in a result of magnitude 0.6.
`fma` computes the product at full width and rounds once, at the scale of the
answer. rlWrap now asks for it explicitly, which is both more accurate and the
only portable way to say which expression is meant: whether a compiler contracts
a multiply-and-subtract on its own is its business, and clang already did while
the shader did not.

What that does *not* fix is the software rasteriser. llvmpipe lowers `fma` to a
multiply and an add — verified directly, it returns 0.60672 where the
correctly-rounded fused answer is 0.60670626 — so on llvmpipe the two sides still
evaluate different expressions and the 2.7e-4 remains. The runner probes for this
and picks its tolerance accordingly (2e-6 where fma is fused, 1e-3 where it is
lowered, saying which), rather than loosening the bound everywhere and hiding a
real defect on real hardware. A separate guard requires ≥90% of the corpus to
match bit for bit, which is what the tolerance cannot do: a systematic shift
would sit under any bound loose enough for float noise, but it cannot leave most
of the corpus exact.

One thing this makes plain for the engine: `f32` cannot resolve a dash edge past
a few kilometres — an ulp of 10 km is a millimetre — so a fragment should be
handed a road-local station, not a world one.

For what it is worth, naga also validates all eleven of the engine's existing
`shaders/webgpu/*.wgsl` clean.

### What is left

The Metal entry point that calls `rlEvaluateMarkings`, uploading the two
textures, the bake from a roadlab `Network` to the engine's road mesh, and
measuring the frame cost — all of which need the viewer, so they need a machine
with a GPU. The frame ledger (ADR-0077, `RT_FRAME_STATS=<csv>`,
`tools/frame-report.py`) is already the harness for it.

## The fallback census

`--fallbacks` runs every demo and five generated cities with terrain, props,
traffic and a full OpenDRIVE round trip, then prints how often each routine had
to substitute a lesser answer — next to how many times it was called.

The reason to build this is the `toST` bug above. It was invisible for the life
of the prototype not because it was subtle but because **a fallback firing looks
exactly like a fallback not being needed**. The routine returned "no answer" on
100% of queries; every caller took its documented fallback path; nothing failed.
A counter would have shown it on day one.

So every place the system quietly settles for less now increments something:

```
site                                                        fired   of calls    rate
Spine::toST miss (point is beyond the end of the spine)   1895747    4696770  40.36%
Network::sample miss (no road under the point)             958482    1073449  89.29%
Spine::toST rejected (Newton did not converge)                2431    4696770   0.05%
planRoute nothing past the floor -> shortest available          652       4556  14.31%
planRoute no reachable destination at all                      1321       4556  28.99%
pairLanesAcross miss (one-way against this direction)           349        771  45.27%
pairLanesAcross lane found no partner within tolerance          112        771  14.53%
junction pad kerb return skipped (would fold into the pad)      510          -       -
stationAtRadius hit the trim limit                               48          -       -
junction pad outline self-intersected -> convex hull              8          -       -
```

Reading it is the whole point, and most of the work was making the numbers mean
something. Two counters were lying at first: the residual guard in `toST` was
absorbing every "point is past the end of the road" query, so neither the
convergence number nor the miss number said what its name claimed. Checking the
station range before the residual splits them, and the convergence rate drops to
0.05% — all of which turn out to be points near the centre of curvature of an
8.6 m kerb-return arc, where the projection is genuinely ambiguous. Likewise
`pairLanesAcross` looked alarming at 45% until it was clear that `linkRoadToRoad`
pairs *both* directions of every link and every connector is one-way: a correct
negative, now named as a miss rather than a fallback.

### What it found

**A demo whose junction arms stood 120 m apart.** `showcase` split high-street at
a hand-typed `s = 300`, which lands at x = -120, while the road it crosses is at
x = 5. The junction still trimmed, still built a pad, still rendered as asphalt —
a 365 m² sheet spanning the gap, with an outline that doubled back on itself and
fell through to a centroid fan. The demo had shipped like that. The split station
is now *projected* from the crossing point, which cannot drift when either road
is re-authored, and `validate()` grew the rule that catches the general case: an
arm whose contact is further from the junction centre than the geometry can
account for means the arms do not meet.

**Kerb returns folding into the pad.** Between the two halves of a street running
straight through a junction there is no corner, but the construction still asked
where the two arms' outward edges intersect — and for diverging rays that lands
*behind* both of them, inside the pad. The distance test waved it through (the
point is close, just on the wrong side) and the Bézier then dived inward across
the far edge. Three junctions across the city seeds had self-intersecting
outlines because of it. A kerb return now also has to bulge away from the
junction centre.

**Roundabout entries trimmed 45 m short of the ring.** The general setback
formula is driven by the widest arm, and a circulating carriageway is wide, so an
approach got pulled back as if it were meeting a large open intersection. The
ring runs *through* a roundabout and an approach meets it at its outer edge, so
that setback is now the ring's half-width.

**Routing silently off for 38% of traffic.** `planRoute` requires a destination
at least 45% of the furthest reachable travel time away; when nothing qualified
it returned no route at all, and the vehicle drove unrouted forever. Nobody
notices, because unrouted vehicles still drive. Falling back to the furthest
*available* destination instead of to nothing took the miss rate on a connected
grid from 21% to 9%. What remains is honest — lanes that genuinely run off the
edge of the map, and the `tiers` demo, which has no junctions and therefore
nowhere to go.

The pad outline is now guaranteed simple by construction: if one still manages to
cross itself, the convex hull of the same points replaces it (carrying the
per-vertex heights), because everything downstream — ear clipping, mean value
coordinates, the terrain point-in-polygon test — is only defined on a simple
polygon. That fires 8 times across the whole corpus and the centroid fan is now
dead code.

### The tool's own blind spot

A counter registers itself the first time it is hit, so a guard that never fires
is *absent* from the table rather than showing a zero — which is the one reading
the census cannot give you directly. Tests close that: they assert the
must-stay-zero sites by name, where an unregistered site correctly reads as 0.

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
rl_paint.h         the marking evaluator, in a C++/MSL/GLSL subset. The GPU half.
rl_paint.wgsl      generated from it by tools/roadlab-wgsl.py. Do not edit.
rl_paint_sampler.wgsl  hand-written: the bindings and fetch for the profile texture.
paint_fixture.h/.cpp   evaluator inputs + CPU answers, for the WGSL comparison.
paint_bake.h/.cpp  cross-section -> the flat boundary array that shader reads.
paint_texture.h/.cpp  that array as the two textures a fragment samples.
diag.h/.cpp        the fallback census: counters for every quiet substitution.
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
tests.cpp          22694 assertions, invariant-focused.
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
- **A self-intersecting pad outline is repaired with a convex hull**, not solved.
  It fires 8 times across every demo and city seed — all roundabout entries,
  where two arms are wide slices of the same circulating carriageway — and the
  hull is within a metre or two of the intended shape, but it is a repair.
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
