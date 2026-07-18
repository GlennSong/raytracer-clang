# Road Network v3 — Total Conversion Plan

Status: PROPOSED (2026-07-18). Supersedes the System A analytic mesher
(`road_mesh.cpp` / `buildRoadNetMesh`), the System B offset/weld prototype
(ADR-0056, `road_offset` / `weldSolid`), and every road meshing doc that
precedes it. The network *data* contracts of ADR-0038 (road → block → parcel
pipeline), ADR-0048 (curves are the source of truth), ADR-0051 (2.5D graph),
ADR-0053 (crossing taxonomy), and ADR-0075 (grading cascade / SurfaceField)
are respected and carried forward. This is a **total conversion**: when the
final phase lands, the old road code is deleted, and lots, traffic sim, nav,
and pedestrian graphs are rebuilt on the new model.

Written to be readable without prior context. Technical terms appear in
asides marked *Jargon* the first time they matter.

---

## 1. Why we are doing this

The current roads look like triangle soup because they *are* triangle soup.
The investigation that preceded this plan found the exact causes:

- The default mesher outlines the whole road surface as one big flat polygon,
  then triangulates it with ear-clipping — an algorithm that produces long,
  skinny, arbitrarily-oriented triangles with no relationship to the road's
  direction of travel.
- Every triangle gets three brand-new vertices (nothing is shared), each
  stamped with a hard-coded "straight up" normal even after the vertex has
  been draped onto sloped terrain. Normals are never recomputed. On any hill,
  every facet is lit wrong, which is why the surface reads as shattered.
- Junction pads and road ribbons compute the *same* boundary points in two
  different code paths and emit them as separate vertices, so seams drift
  and crack.
- Deck, underside, walls, sidewalk, curb, and markings are emitted as
  independent passes, and each road entity is its own mesh — a city is a
  pile of overlapping meshes, not one surface.
- A nine-round debugging saga (documented in `metropolis-scale-plan.md` §3)
  proved that whenever the road's height is computed by two parallel code
  paths — one for the mesh, one for the terrain carve — they *will* disagree
  and roads will float on plinths or sink into the ground.

None of this is patchable. The mesh generation strategy itself is wrong.
This plan replaces it — and while we're at it, upgrades the data model so
the network can describe everything a real city needs: multilane roads,
turn lanes, parking lanes, optional sidewalks, freeways with on/off ramps,
bridges, elevated roads, tilt on curves, and intersections of any shape.

### What we learned that the new system must keep as law

1. **One height source.** A single elevation function per road feeds the
   render mesh, the collision mesh, the terrain carve, and the nav graph.
   Never two parallel computations of "how high is the road here."
2. **Consumers eat the graph, not the mesh.** City lots, districts, traffic
   sim, pedestrian routing, and street furniture all consume centerlines,
   widths, classes, lane counts, elevations, and block topology. The mesh is
   a leaf output. Keep the graph the single source of truth.
3. **Merges are lane events, not point splices.** An on-ramp is "the freeway
   gains a lane that later tapers away," not "two lines touch at a point."
   Splicing at points is what buried ramps under decks.
4. **Freeways are isolated ribbons.** Nothing merges into a grade-separated
   corridor except through a ramp. Letting street generators grow into
   freeway nodes is what produced the "blobs."
5. **Markings are texture, not geometry.** Marking geometry produced spikes
   and splinters; the shader-based marking approach worked. We keep and
   extend it.
6. **Terrain conforms to the road, not the reverse** (ADR-0075). Roads carry
   a smoothed, grade-limited profile; the ground is cut and filled to meet
   it through the SurfaceField grading cascade.

---

## 2. The big picture: three layers, one direction of flow

The system is three layers. Data flows strictly downward; nothing below
ever writes back upward.

```
┌────────────────────────────────────────────────────────────┐
│ LAYER 1 — NETWORK      what designers and generators touch │
│ nodes + spline edges + cross-section recipes + overrides   │
├────────────────────────────────────────────────────────────┤
│ LAYER 2 — LANES        derived: every lane as its own path │
│ lane centerlines, lane events, turn connections, signals   │
├────────────────────────────────────────────────────────────┤
│ LAYER 3 — SURFACE      derived: the actual geometry        │
│ quad-grid meshes, junction patches, decals, colliders,     │
│ terrain-carve footprints                                   │
└────────────────────────────────────────────────────────────┘
```

- **Layer 1** is small, serializable, editable, and deterministic. It is
  what the procgen generators emit and what the editor manipulates.
- **Layer 2** is recomputed from Layer 1 whenever it changes. The traffic
  sim drives on it. The pedestrian graph and street furniture read it.
- **Layer 3** is recomputed per streaming chunk, on demand, and never saved
  to disk. Same inputs → identical output, always.

Everything the old system got wrong traces back to not having this
separation. Everything downstream (lots, sim, rendering) plugs into exactly
one of these layers.

---

## 3. Layer 1 — the road network graph

### 3.1 Nodes and edges

The network is a graph: **nodes** are places where roads meet, end, or
change character; **edges** are the road segments between them.

- **Node**: a 2D position on the map, an elevation *mode* (at-grade,
  absolute height, or offset-above-terrain), a *layer* number for
  grade-separated stacking (ADR-0051), and a stable ID.
- **Edge**: references two nodes, carries a **centerline spline**, a
  **cross-section recipe** (§3.2), a **road class** (Freeway, Arterial,
  Collector, Local, Ramp — carried over from today), one-way flag,
  speed class, and a stable ID.

> *Jargon — spline:* a smooth curve defined by a handful of control points.
> We use cubic Hermite curves (each node stores a tangent direction —
> "which way the road is pointing as it passes through me"), because they
> match the existing `EditableCurve`/`RoadHandleSource` editing tools and
> are trivially draggable. Straight roads are just splines with collinear
> tangents — there is no separate "straight" type, which kills a whole
> class of old bugs.

**Stable IDs matter.** Nodes and edges get slot-map style IDs (index +
generation counter, like the engine's existing `slot_map.h`) that never
change as the graph is edited. Determinism (§9), undo (§11), and streaming
all key off these IDs.

**Arc-length parameterization.** Every edge exposes its centerline as a
function of *distance along the road*, not spline parameter. All measuring,
sampling, and event placement uses meters-from-the-start.

> *Jargon — station/offset:* civil engineers describe any point on a road
> as (station, offset) = (how many meters along the centerline, how many
> meters left or right of it). We adopt this everywhere: `s` is station,
> `t` is offset. The mesh grid, the UV coordinates, the lane positions, the
> marking patterns, and the decal placements are all expressed in (s, t).
> This single convention is what makes everything line up.

### 3.2 Cross-section recipes — how one edge describes a whole street

Instead of "width = 14," an edge carries an ordered list of **strips**,
read left to right across the road:

```
Example: a 4-lane arterial with parking and sidewalks
[ Sidewalk 2.4 | Curb 0.15 | Parking 2.4 | Lane 3.6 | Lane 3.6 |
  Median 1.2 | Lane 3.6 | Lane 3.6 | Parking 2.4 | Curb 0.15 | Sidewalk 2.4 ]

Example: a local lane, no sidewalk
[ Shoulder 0.6 | Lane 3.3 | Lane 3.3 | Shoulder 0.6 ]

Example: one direction of a freeway
[ Barrier 0.6 | Shoulder 3.0 | Lane 3.6 | Lane 3.6 | Lane 3.6 |
  Shoulder 3.6 | Barrier 0.6 ]
```

Strip types: `Lane`, `Parking`, `Sidewalk`, `Curb`, `Median`, `Shoulder`,
`Barrier`, `Gore`, `BikeLane`, `Verge` (grass strip). Each strip has a
width, a surface material, and a vertical offset (sidewalks sit 0.15 m
above the carriageway behind a curb — a real step the physics can feel).

This one structure answers most of the feature list directly:

- *Multilane roads*: more `Lane` strips.
- *Not all roads need sidewalks*: recipes without `Sidewalk` strips.
- *Parking*: a `Parking` strip.
- *Divided highways*: a `Median` strip (or two separate one-way edges for
  wide medians — both supported; the generator picks by median width).
- *Road classes* become **recipe presets**: class + lane count → default
  recipe, which the generator stamps and the editor can then override.

**Recipes change along the road via profile events.** A turn lane doesn't
exist for the whole edge — it appears 60 m before the intersection. So an
edge's cross-section is: a base recipe + a sorted list of **events** at
stations:

```
LaneAdd    (s=210, side=right, type=Lane, taper=30m)   → turn lane opens
LaneDrop   (s=850, side=right, taper=90m)              → ramp lane merges away
StripEdit  (s=0..120, strip=Sidewalk, width=3.6)       → wider sidewalk downtown
```

A taper is meshed as a smooth wedge (the strip's width animates from 0 to
full over the taper length). **This is the "merge is a lane event"
principle made concrete**: an on-ramp is a `LaneAdd` on the freeway edge
at the gore point, and the ramp edge's lane flows into that new lane in
Layer 2. Multi-turn lanes are just multiple `LaneAdd` events before a
junction, with the lane-to-turn mapping stored on the junction (§6.4).

### 3.3 Elevation: one profile to rule them all

Each edge owns exactly one **vertical profile**: a function
`height(s)` built by the elevation solver (§7) — smoothed, grade-limited,
clearance-respecting. Plus one **superelevation profile** `bank(s)`: the
tilt of the road surface on curves.

> *Jargon — superelevation:* the banking of a road on a curve so cars are
> pushed into the surface instead of sliding off. We use the standard
> highway formula (bank fraction ≈ V²/127R, capped at 6%): tighter curve or
> faster class → more tilt, interpolated smoothly in and out of the curve.

**The law from §1 applies here with no exceptions:** `height(s)` and
`bank(s)` are computed once, stored on the edge, and *sampled* by the
mesher, the collider builder, the terrain-carve emitter, the lane layer,
and the nav graph. There is no second implementation anywhere.

### 3.4 Decorations

Anything else a road can carry rides as typed key-value decorations on
edges and nodes rather than hard fields, so new features don't churn the
core structs: `speedLimit`, `districtStyle`, `weatheringAge`,
`streetlights: none|left|right|both`, `treeLawn`, `namedStreet`, etc.
Consumers query them with defaults.

---

## 4. Layer 2 — the lane graph

Derived from Layer 1 by a pure function. For every edge, for every `Lane`
strip, emit a **lane centerline** (an offset curve at that lane's `t`,
sampling the same `height(s)`/`bank(s)`). At junctions, emit **connectors**:
short splines linking an incoming lane to each outgoing lane it may turn
into, tagged left/through/right/U.

What this buys us over today's system, where the sim guesses
`lanes = width / 3.5`:

- The traffic sim drives on real lane paths with real turn topology —
  including "left two lanes must turn left" (multi-turn lanes), captured as
  junction turn maps (§6.4).
- Signals attach to connectors, not approaches-by-axis: a protected left
  arrow is just a signal group over the left-turn connectors.
- Ramp merges are `LaneAdd`-aware: the ramp's lane connector lands in the
  added freeway lane, so simulated cars merge exactly where the painted
  gore is.
- The pedestrian graph reads `Sidewalk` strips (only roads that have them)
  plus **crosswalk connectors** generated at junctions (§6.3), so pedestrians
  cross where the paint is.

The lane graph replaces `NavGraph`/`ExtraNavGraph`/`ped_graph` as the
routing substrate. It is regenerated whenever Layer 1 changes and never
serialized.

---

## 5. Layer 3 — meshing roads as quads

This is the heart of the plan. The rule: **every surface in the road
system is a grid of quads, roughly square, oriented with the flow of the
road, with every vertex shared and every normal computed from real
geometry.** No ear-clipping. No fans. No duplicated boundary points.

### 5.1 The ribbon grid

A road segment is meshed by **sweeping its cross-section along the
centerline**:

- **Rows** are stations: sample points along the road, spaced ~3.5 m apart
  (one lane-width) so quads come out square-ish. The spacing adapts:
  - On curves, rows densify so the mesh never cuts the corner by more than
    1.5 cm (chord-error bound: `Δs ≤ √(8·ε·R)` — tighter radius, closer
    rows) and never folds (row spacing also capped by radius minus
    half-width, which kills the old self-intersection bug).
  - On elevation changes, rows densify where `height(s)` curves (crest and
    sag vertical curves) so bridges arc smoothly.
- **Columns** are strip boundaries, with wide strips subdivided so no
  column exceeds ~4 m. A 3.6 m lane is one column; an 8 m plaza-sidewalk
  is two.

Each (row, column) cell is one quad. Target aspect ratio between 1:2 and
2:1 — "square-like," as requested, everywhere from parking lane to freeway.

Every vertex is placed by one function:

```
worldPos(s, t) = centerline(s)
               + right(s) * t            (rotated by bank(s))
               + up * (height(s) + stripLift(t))
```

then the whole chunk mesh gets `recomputeNormals()` (the area-weighted
smooth-normal pass that already exists in `mesh_builder.h` and that the
old road code simply never called) and tangents for normal mapping.

**UVs come free:** `u = t`, `v = s`, in meters. Because UV equals
station/offset, the marking shader (§8) knows exactly where lane lines and
dashes go with zero extra data, and the asphalt texture tiles without
stretching on curves — the parameterization is arc-length by construction.

**Curbs and sidewalk steps are real geometry**: the `Curb` strip emits a
small vertical quad column (0.15 m riser) so the profile reads correctly
up close and Jolt sees a real step. It's still part of the same grid —
its rows are the same stations.

**Undersides and edges**: elevated spans emit a matching underside grid
(same stations, offset down by deck thickness), side skirts, and barrier
strips — all quads on the same station rows, so the deck is a closed,
watertight box you can stand under.

### 5.2 One welded mesh per chunk

Roads no longer own meshes. A **chunk** (§9) collects every edge segment
and junction inside it into one indexed vertex/index buffer, welding
shared boundaries by construction (not by epsilon-matching after the
fact): a junction patch doesn't *recompute* the road's mouth vertices, it
*receives their indices*. Cracks become impossible rather than unlikely.

Material batching within the chunk: one draw for carriageway (asphalt),
one for sidewalk/curb (concrete), one for medians/verges, one decal pass
(§8.3). Vertex color carries per-vertex weathering/tint modulation.

### 5.3 Level of detail

The grid structure makes LOD trivial in a way soup never could: LOD1 drops
every other station row and merges lane columns per direction; LOD2 is a
single quad strip per edge (2 columns) with markings entirely in texture;
LOD3 renders the network into the terrain's virtual texture / a flat
ribbon pass for the far horizon. Because rows are indexed and regular,
LOD transitions can stitch (skirt or index-collapse) without cracks.

---

## 6. Intersections — any degree, all quads

Junctions are where the old system died. The new approach makes the
junction a first-class object with its own solver, built so that **the
junction's boundary is made of the roads' own grid vertices**.

### 6.1 The junction polygon

For each node with degree ≥ 2 (or degree ≥ 3 at-grade; degree-2 nodes are
just spline continuations unless classes differ):

1. **Cut back each incoming edge** to a *stopline station*: far enough
   from the node that the edges' cross-sections don't overlap, computed
   from arm angles and widths (plus space for crosswalks). The edge's
   ribbon grid simply ends at that row.
2. **Corner fillets**: between each pair of adjacent arms, insert a curb
   return — an arc (radius 3–4.5 m by class, per the existing design
   rules) connecting the right curb line of one arm to the left curb line
   of the next. The fillet is sampled at the same ~3.5 m spacing.
3. The junction boundary is now: mouth rows (the last station row of each
   arm — *shared vertices*) + fillet arcs. A closed polygon with known,
   grid-compatible vertices.

### 6.2 Filling the interior with quads

Two strategies, picked automatically:

- **Degree 3–4, sane angles (the 95% case): transfinite patches.** Pair
  opposite mouths and blend their rows across the junction (a Coons patch —
  a surface interpolated from four boundary curves). A 4-way intersection
  becomes a clean grid whose rows flow from one road into its continuation;
  a T-junction uses a half-patch plus fillet triangles' quadified fill.
  Quads stay square-ish and *aligned with traffic flow*, which is exactly
  what makes markings and weathering read correctly.
- **Any other case (5+ arms, weird angles, roundabout interiors):
  quadrangulation by midpoint subdivision.** Partition the polygon into
  convex cells, then split every cell into quads by inserting its center
  and edge midpoints — one subdivision step turns *any* polygon into
  all-quads, guaranteed (this is the Catmull-Clark trick). Then run a few
  Laplacian smoothing iterations (vertices relax toward the average of
  their neighbors, boundary pinned) so the quads even out toward square.
  This handles *any number of arms at any angles*, which is the "handle
  any number of intersections" requirement with no special cases.

> *Jargon — Coons patch:* a surface defined purely by its four boundary
> curves; every interior point is a blend of the boundaries. Cheap, stable,
> and it inherits the boundary's vertex spacing — perfect for stitching.

Junction interior heights sample a small height patch blended from the
arms' `height(s)` at their stoplines (continuous by construction — arms
already agreed on the node's elevation in the solver, §7). Normals come
from the same chunk-wide recompute.

**Roundabouts** are junction templates (per ADR-0053): circulating ring =
a closed ribbon grid (annulus of quads), arms attach with fillets exactly
as above, center island is a quadified disc.

### 6.3 What the junction solver also emits

Because the junction knows its geometry exactly, it emits — from the same
data, in the same pass:

- **Crosswalk bands** across each arm just outside the stopline (skipped
  where a `NoPedestrian` decoration or missing sidewalks say so), as decal
  rectangles in (s, t) — and matching crosswalk connectors into Layer 2.
- **Stop bars / yield lines** per approach, signal-or-sign aware.
- **Corner sidewalk continuations**: sidewalk strips wrap the fillets
  (quads along the arc) with ADA-style curb ramps at crosswalk landings.
- **Furniture anchors**: signal pole positions at corners, sign positions,
  lamp positions — consumed by the street furniture system (§12.4).

### 6.4 Turn maps

Each junction stores, per approach, which lanes feed which turns
(`lane 0 → left; lane 1 → left+through; lane 2 → through; lane 3 → right`).
Defaulted by a rule table (leftmost turns left, etc.), overridable in the
editor. This drives three things identically: painted turn arrows (§8.3),
Layer-2 connectors, and signal phase grouping. Paint, sim, and signals can
never disagree because they are the same data.

### 6.5 Grade-separated interchanges

Freeway interchanges are **compound structures, not mega-junctions**: a
diamond is two ordinary street junctions + four Ramp-class edges + four
`LaneAdd`/`LaneDrop` events on the freeway. Trumpets, cloverleafs, and
stacks are generator templates (ADR-0053's taxonomy) that expand into
plain nodes/edges/events — so the mesher never sees an "interchange," only
roads, ramps, and junctions it already knows how to mesh. Ramp gores get
`Gore` strips (the striped triangular no-drive zone) with chevron decals.

---

## 7. Elevation, bridges, and the vertical solver

A single **elevation solve** runs over the whole network (incrementally
per dirty region after edits):

1. Nodes get target heights from their mode: at-grade nodes sample
   terrain; absolute nodes (freeway decks) keep their authored/planned
   height; offset nodes float relative to terrain.
2. Edge profiles `height(s)` are solved as smoothed splines between node
   heights subject to: class grade limits (freeway ≤ 4%, ramp ≤ 7%,
   local ≤ 10%), minimum vertical-curve lengths (no kinks at crests/sags),
   and **clearance constraints** — wherever two edges cross on different
   layers, the upper must clear the lower by ≥ 5.1 m; the solver raises
   the upper profile (and its approach) or flags an unsolvable spot for
   the generator to reroute. Constraint relaxation over the graph; it
   converges because constraints are convex in the heights.
3. Each station of each edge is then classified by `height(s) − terrain(s)`:

   | condition | treatment |
   |---|---|
   | within ±0.75 m | **at-grade** — terrain is carved to the road (SurfaceField cascade, ADR-0075) |
   | 0.75–3 m above | **embankment/cut** — sloped earthworks, or retaining walls (StructureSet) where the footprint budget is tight |
   | > 3 m above, or spanning water/roads | **bridge** — deck box (top/underside/skirts), barriers, and piers placed at regular spans where ground allows; pier bases subtract from buildable land |
   | below terrain | **cut** walls, or tunnel (portal + hole via StructureSet) — tunnels are a later phase but the classification slot exists from day one |

   Transitions between treatments happen at station boundaries with
   blending rows, so an on-ramp flows at-grade → embankment → deck as one
   continuous grid.

Because the classification derives from the *same* `height(s)` used by the
mesh, the carve footprints and the deck can't disagree — the proud-apron
class of bug is structurally gone.

---

## 8. Materials and markings

### 8.1 Asphalt (and friends) as PBR

The carriageway gets a real PBR material set — albedo, normal, roughness,
AO — using the texture slots the material system already supports
(`albedoTex`/`normalTex`/`mrTex`/`aoTex`) but which roads never used.
Tiling in (s, t) meters, so texel density is uniform across the entire
city regardless of curvature. Sidewalk/curb concrete, median concrete,
and gore surfaces are their own materials in the same batched chunk mesh.
Realtime viewer first; the offline path tracer reads the same maps.

### 8.2 Line markings: procedural, in the shader

Lane lines stay shader-evaluated (the proven approach), but upgraded from
the old hue-hack to a **marking spec** the shader reads per road:

- Each edge compiles its strip recipe + events into a tiny table of line
  definitions: `{t-position, color (white/yellow), pattern (solid, dashed
  10ft/30ft, double, dot-dash), width 10–15 cm}`. Uploaded per-chunk as a
  small buffer/texture; the pixel shader draws crisp anti-aliased lines by
  comparing the fragment's (u, v) = (t, s) against the table.
- US grammar by default: dashed white between same-direction lanes, solid
  white at shoulder, double yellow center for two-way, dashed-yellow
  passing zones, solid white through turn-lane pockets, dotted lines for
  lane extensions through junctions where taper events are active.
- Dashes are phase-continuous across edges (pattern phase carried through
  nodes as accumulated s) so lines never stutter at graph seams.
- Weathering (§8.4) multiplies marking opacity, so paint fades believably.

### 8.3 Stencils: decal quads from an atlas

Discrete symbols — crosswalk bands, turn arrows, "STOP AHEAD" / "ONLY" /
"XING" pavement text, yield triangles, gore chevrons, parking Ts, bike
symbols — are **decal quads**: small (s, t)-aligned quad patches laid over
the road grid, UV-mapped into one marking atlas texture, rendered as a
decal pass with polygon offset (no z-fighting, no geometry displacement —
the mistake that caused the old spikes is not repeated). Placement is
automatic: the junction solver emits crosswalks/stop bars/arrows (§6.3),
turn maps pick arrow glyphs, `LaneDrop` events emit merge arrows,
approach rules emit "STOP AHEAD"-style text where a stop-controlled
junction follows a blind curve or crest (sight-distance test over
`height(s)` and curvature).

### 8.4 Weathering

A per-chunk weathering mask, seeded deterministically (§9): large-scale
noise for patch repairs and tone variation; **wheel-path darkening**
computed in the shader from lane geometry (two darker tracks per lane at
±0.9 m from lane center — trivially available because the shader knows
lane `t` positions from the marking table); crack/patch decals scattered
by age decoration; marking-fade multiplier. `weatheringAge` is a per-edge
decoration so a district can read old while a new subdivision reads fresh.

---

## 9. Streaming, determinism, and scale

Designed for a NYC/Tokyo-scale metropolis; degrades gracefully to a
village (a small town is just a mostly-empty chunk grid — no special
casing).

- **Layer 1 is global and resident.** The network graph is compact
  (thousands of nodes, not millions of vertices) and lives in memory for
  the whole world; it's what generators produce and saves serialize. For
  truly huge worlds it partitions into region files loaded by proximity,
  but that's an optimization, not an architecture change.
- **Layers 2–3 are chunked.** The world is a grid of chunks (256 m,
  tunable). A chunk builds when the camera/simulation needs it:
  gather edges/junctions overlapping the chunk → mesh → collide → done.
  Edges crossing a border are cut at station rows; because station
  sampling is anchored to the *edge's* start (never the chunk), both
  chunks compute identical border rows and share identical vertices.
- **Deterministic randomness everywhere.** Every stochastic choice
  (weathering seeds, pier jitter, decal scatter, generator decisions)
  draws from `hash(worldSeed, stableId, purposeTag)` — never from time or
  global RNG state. Same seed + same edits → bit-identical city, so chunks
  regenerate on demand instead of being saved, and a save file is just
  the seed + Layer 1 + edit overrides.
- **Budgets.** Chunk builds run on the job system off the main thread;
  a build is pure (inputs: graph slice, terrain sampler, seed) so it
  parallelizes trivially. Colliders (§10) build in the same job.
- **LOD ladder** per §5.3, driven by chunk distance rings.

---

## 10. Physics — the surface you see is the surface you drive

Per chunk, the *same* indexed quad grid (triangulated at upload — each
quad → two triangles, purely a GPU/Jolt formality; the authoritative
topology stays quads) feeds:

- a Jolt `MeshShape` static collider for the road surface, sidewalks
  (with their real 0.15 m curb step), medians, and gores;
- barrier/parapet colliders on bridges and freeway edges (you cannot
  drive off an elevated deck through the railing);
- pier and retaining-wall colliders from the StructureSet;
- underside colliders on decks (things exist beneath bridges).

Because collision and render sample the same `worldPos(s, t)` and the same
`height(s)`, cars and pedestrians follow the surface at any elevation with
zero visual/physical mismatch — wheels sit on the asphalt you render, on a
banked curve, on a 30 m-high stack ramp, under a deck. Vehicles and
pedestrians remain fully Jolt-simulated (per ADR-0059–0062's one-vehicle
principle); the lane graph gives them *goals*, physics carries them there.

---

## 11. Live editing

Generator-first (the network normally comes from procgen), with full
manual override. Reuses the proven editor seams: `HandleSource` /
`PathEditTool` for viewport manipulation, `ComponentRegistry.onEdited` +
the properties panel for the inspector, `UndoStack` for history.

- **Drag nodes**: move a junction; its arms' splines, the junction
  polygon, profiles, and affected chunks rebuild live. During the drag,
  affected edges remesh at half station density for interactivity; on
  release, full quality. (Dirty-set: the dragged node, its edges, their
  far junctions, and chunks they touch — nothing else rebuilds.)
- **Drag spline handles**: per-node tangent handles (the existing
  `RoadHandleSource` two-handle scheme) reshape curvature live; the
  curvature-adaptive mesher and superelevation respond automatically.
- **Inspector panel**: select an edge → class, one-way, speed, recipe
  preset, lane count, sidewalk/parking toggles, median width, weathering
  age, name; select a node → elevation mode, junction control
  (signals/stop/yield), turn-map override, roundabout toggle; select the
  network → generator recipe + seed + regenerate.
- **Topology edits**: split edge, join, extend from node, delete (with
  junction healing), draw new road (click-click-click along a path).
- **Undo, fixed properly**: every Layer-1 mutation goes through a command
  object capturing before/after graph deltas; `PathEditTool.onGrab` (which
  exists but was never wired) snapshots at drag start. This closes the
  audit's finding that node drags are currently not undoable.
- **Overrides vs regeneration**: hand-edited elements get an `authored`
  flag; re-running the generator preserves authored elements and re-flows
  the generated remainder around them. (Simple ownership model — no
  three-way merges.)
- **Serialization**: saves store Layer 1 + recipe + overrides as JSON in
  the existing `SourceSpec.recipe` pathway (load → save stays a no-op);
  geometry is never saved.

---

## 12. Rebuilding the consumers (the total conversion)

Everything downstream re-plugs into the new layers. Contracts:

1. **Blocks / lots / districts (Layer 1 + curb lines).** Planar faces of
   the at-grade network (per layer) → blocks, exactly as ADR-0038's
   pipeline expects — but the block boundary is now the mesher's *own curb
   line* (the outermost strip edge at the sidewalk's back), not an
   inset-by-half-width approximation. Lots front onto real frontage
   geometry; the drift between "where the lot thinks the road is" and
   "where the road actually is" disappears. Freeway corridors contribute
   easement footprints (deck shadow + pier bases + embankment slopes)
   subtracted from buildable land.
2. **Traffic sim (Layer 2).** Drives on lane centerlines and connectors;
   signals bind to connector groups from junction turn maps; class speeds
   come from edge decorations. Deletes `NavGraph`-from-width inference,
   `ExtraNavGraph`, and the axis-guessing signal builder.
3. **Pedestrian graph (Layer 2).** Sidewalk strip paths + crosswalk
   connectors + door-snap to the back-of-sidewalk line. Only exists where
   sidewalks exist. Excludes Freeway/Ramp by construction.
4. **Street furniture (junction solver output).** Signal poles, signs,
   lamps, hydrants placed on the anchors the junction/edge emitters
   produce (§6.3), instanced via the existing `InstanceGroup` path.
5. **Terrain (Layer 3 footprints).** At-grade carve footprints +
   embankment/wall/pier structures flow into the ADR-0075 SurfaceField
   grading cascade unchanged in spirit: terrain → roads → blocks → lots →
   pads, walls where the grade budget breaks.
6. **Generators (emit Layer 1).** Grid/radial/tensor street generators and
   the metro/freeway planner are ported to emit the new node/edge/recipe
   model (street-first ordering per `city-pipeline.md` v2 stays). The
   constraint pass (min arm angle, degree caps, roundabout promotion,
   short-edge merge) carries over nearly as-is — it operates on topology,
   which hasn't changed shape, only gained information.

---

## 13. What gets deleted

At the end of the conversion (Phase 7), these are removed:
`road_mesh.{h,cpp}` (all of it — ribbons, SDF, weld, pads),
`corridor_mesh.{h,cpp}` (freeways become ordinary edges + events),
`road_offset.{h,cpp}`, `road_crossings.{h,cpp}`, `triangulate.{h,cpp}`
(earcut has no remaining callers on roads), the `RT_SDF_ROADS` /
`RT_ANALYTIC_ROADS` env-var forks, `ExtraNavGraph`, and the old
`ped_graph` builder. `road_rules.{h,cpp}` (design constants) and
`road_constraints.{h,cpp}` (topology rules) survive as inputs to the new
system. ADR-0056's "unified system" is formally closed as superseded; the
duplicate-ADR-0066 numbering bug in `decisions.md` gets fixed in the same
commit that adds this plan's ADR.

---

## 14. Build order

Each phase is independently verifiable, keeps `make test` / `ctest` green,
and ends with a visible milestone. The new code lives in
`src/engine/procgen/roads/` from Phase 1; old code keeps running the
existing levels until Phase 7 cuts over — "break it all" happens as one
final, deliberate switch, not a long broken middle.

- **Phase 0 — Skeleton + math.** Graph structs, stable IDs, Hermite
  centerlines with arc-length tables, cross-section recipes, event lists,
  (s, t) sampling. Pure, headless, unit-tested (`tests/test_road_graph`).
- **Phase 1 — The perfect ribbon.** Single-edge quad-grid mesher:
  adaptive stations, strip columns, curbs, welded indices,
  `recomputeNormals`, (s, t) UVs. PBR asphalt + procedural line shader
  (solid/dashed/double-yellow). *Milestone: one curved, banked, 4-lane
  road with parking and sidewalks that looks production-quality.* Tests:
  quad aspect bounds, no fold at min radius, chord error, UV arc-length.
- **Phase 2 — Junctions.** Junction solver: stopline cutback, fillets,
  Coons fill, midpoint-subdivision fallback, shared mouth vertices,
  turn maps; crosswalk/stop-bar/arrow decals; "STOP AHEAD" text decal.
  *Milestone: 3/4/5/6-way junctions and a roundabout, zero cracks
  (asserted: watertight index audit), turn lanes with arrows.*
- **Phase 3 — Vertical.** Elevation solver (grades, vertical curves,
  clearances), at-grade/embankment/bridge classification, deck boxes,
  piers, barriers, superelevation, terrain-carve emission through
  SurfaceField. *Milestone: a freeway with on/off ramps (LaneAdd/Drop +
  gores + chevrons) crossing a surface street on a bridge; drive under it.*
- **Phase 4 — Chunks + determinism.** Chunked build on the job system,
  border-exact stitching, LOD ladder, deterministic seeding, Jolt
  colliders per chunk. *Milestone: fly across a large generated net with
  streaming on; bit-identical rebuild test; car drives every surface.*
- **Phase 5 — Generators.** Port grid/radial/tensor + metro planner +
  constraint pass to emit Layer 1; recipe presets per class; district-
  aware recipes. *Milestone: `metropolis.json`-scale city generates on
  the new system side by side with the old.*
- **Phase 6 — Editing.** Handles, inspector, topology ops, dirty-set
  incremental rebuild, undo (onGrab wired, graph deltas), serialization.
  *Milestone: drag a junction in a live city; everything follows; ctrl-Z.*
- **Phase 7 — The conversion.** Lane graph → traffic sim + signals;
  ped graph from sidewalks/crosswalks; lots/blocks re-plumbed to curb
  lines; street furniture on anchors; levels migrated; **old system
  deleted** (§13); ADR written; docs pruned. *Milestone: the full game —
  lots, sim, pedestrians — runs on v3 only, and the diffstat is negative.*
- **Phase 8 — Polish (ongoing).** Weathering mask + wheel-path wear,
  passing-zone logic, parking stall Ts, bike lanes, interchange templates
  (trumpet/cloverleaf/stack), tunnels, virtual-texture far LOD.

---

## 15. Open questions (answers welcome, defaults chosen)

1. **Chunk size** — defaulting to 256 m; happy to tune against streaming
   metrics in Phase 4.
2. **Median policy** — auto-split into twin one-way edges above 4 m median
   width; below that, a `Median` strip. Sane? (Affects generator + lots.)
3. **Marking atlas art** — plan assumes one authored atlas texture for
   stencils (arrows, text, chevrons). Generated placeholder glyphs first;
   want real art at some point?
4. **Tunnels** — slotted as Phase 8. If underground freeways matter sooner,
   Phase 3 grows a portal treatment and it moves up.
5. **Old levels** — `showcase.json` (the only System-B consumer) will be
   rebuilt as a v3 showcase rather than migrated. OK to retire the
   original?
