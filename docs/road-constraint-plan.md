# Road constraint system + multi-scale world layout — research + plan

Status: **proposal / investigation** · Date: 2026-06-25 · Relates to ADR-0044 (junction
trim), ADR-0046 (terrain-aware city), ADR-0048 (one road model: curve→graph), ADR-0049
(editable RoadNet), and proposes **ADR-0051–0055**.

This captures the design for turning road generation from a per-pattern stamp into a
**constraint-driven, multi-scale system** that always produces something natural-looking —
and that extends to grade separation, freeways, ramps, interchanges, winding mountain
roads, waterways, and whole-map settlement layout, *without* a new pipeline per feature.

It grew out of a concrete bug: adding many "spokes" to one editable-road node makes the
analytic junction overlap and self-destruct (the trim setback to clear a curb corner scales
~`w/sin θ` in the neighbour angle, diverges as spokes pack, then hits the
`0.45·edgeLen` clamp and the pad ring self-intersects — `road_mesh.cpp:368-443`). The fix
is not better trim math; it is to stop representing an intersection as a dimensionless point
and to **constrain the topology so the degenerate case is unrepresentable**.

---

## The one idea

> **Everything is one machine: a `class`-parameterized graph that gains a vertical/layer
> dimension, run through a single named *local-constraints* pass, where every crossing
> resolves to a *template* chosen by `(incident classes, level difference, degree, angle)`;
> and the whole thing is driven top-down by a stack of *fields* that say where places,
> water, and density are.**

Junction patch, roundabout, diamond, trumpet, cloverleaf, stack, freeway merge, and
switchback are all members of the **same template family**. Mountain roads and freeways are
not features — they are what the *same* rules produce at different class + terrain settings.
Waterways, settlements, and block density are not new pipelines — they are *fields* the
existing machine reads.

Two architectural commitments make this hold together and make it extensible:

1. **Generate-and-constrain, never global-solve.** After Parish & Müller (CityEngine,
   SIGGRAPH 2001): a *global-goals* phase proposes road geometry from a soft preference
   field (we already have this — the tensor field, `tensorRoads`); a *local-constraints*
   phase adjusts/snaps/rejects each candidate against its immediate neighbourhood. The
   natural global look *emerges* from local legality + the goal field. No step ever
   constrains all nodes simultaneously — that is what makes it scale and what dissolves the
   "constrain the whole system at once" problem.

2. **New world systems plug in as fields, rules, or templates — never as new pipeline
   stages.** This is the extensibility invariant (see "Extensibility" below). Waterways are
   the worked proof.

---

## Where we are today (grounded in the code)

- **Editable road:** JSON `shape:"road"` → `RoadNet` → `buildRoadNetMesh` (`road_net.cpp:115`)
  → `buildRoadMesh` (the ADR-0044 analytic trim+pad path). Planar, 2D, draped on terrain;
  every crossing is forced to be an at-grade intersection.
- **Procedural network:** `gridRoads` / `radialRoads` / `tensorRoads`
  (`road_network.h`) build a `RoadGraph`; `planarize` / `extractBlocks` /
  `pruneSteepEdges` / `connectComponents` are *scattered* legality fixups.
- **Vertical model exists but is shallow:** `roadProfile` (smoothed, grade-limited
  centreline) + `roadConformRegions` (terrain cut/fill). No notion of a road leaving the
  ground onto a deck.
- **Class exists** (`RoadClass {Arterial, Collector, Local}`) but barely drives anything.
- **The radial generator already meets a central roundabout, never a spike**
  (`road_network.h:63-80`) — an implicit instance of the node-promotion rule we are about to
  make explicit.

So roughly half the machine exists; it is unnamed, un-unified, and 2D.

---

## Phase 0 — Data model: planar-2D → layered 2.5D  *(ADR-0051)*

The precondition for grade separation, and therefore for over/underpasses, freeways, ramps,
and interchanges. Without an elevation/layer per node, *every* crossing must be an
intersection and none of those features can even be represented.

- **Per-node elevation + layer tag.** Two edges crossing in XY are the same junction only if
  their vertical profiles meet within clearance there; otherwise they are grade-separated and
  do **not** share a node.
- **Per-edge vertical profile, first-class.** Generalize `roadProfile` from "drape on
  terrain" to "a profile that may rise onto a deck or dip into an underpass," bounded by class
  max-grade and a min vertical clearance (~5 m) at any separation it is part of.
- **Per-edge class + carriageway mode.** Extend `RoadClass` with `Freeway` and `Ramp`.
  Carriageway mode ∈ {single ribbon, **dual carriageway** (two ribbons + median), ramp
  taper}. Class is the master knob: it sets min radius, max grade, lane count, *and* access
  policy (who may cross at-grade).

---

## Phase 1 — The named local-constraints pass + rule registry  *(ADR-0052)*

Factor the scattered fixups into one reusable phase both the generators and the editor call:

```
RoadGraph applyConstraints(RoadGraph g, const RuleSet& rules);
```

It runs the rule catalog **incrementally and locally** (each rule adjusts/snaps/rejects
against the local neighbourhood). Two entry points:

- **Generators** call it after laying down candidate streamlines (P&M local-constraints
  phase; the tensor field is the global-goals phase).
- **Editor** calls it on the neighbourhood of a dragged node, so illegal configs *snap or
  refuse live* — the degenerate hub becomes unrepresentable rather than fixed-up after the
  fact.

**Rule catalog (class-parameterized), by scope:**

| Scope | Rule | Status |
|---|---|---|
| Edge | Min curve radius by class | primitive exists (`fairHermite`) → make class-keyed |
| Edge | Max grade by class | exists (`roadProfile`, `pruneSteepEdges`) |
| Edge | Min segment length / max deflection (kink → curve or junction) | new, small |
| Node | **Min arm angle** → caps degree to `2π/θ_min`; over-packed spokes can't exist | new, highest leverage |
| Node | Max degree / required-radius → **promote to roundabout** | new (classifier) |
| Node | No acute merge → tangential fuse (ramp/merge) not knife-edge T | new |
| Network | Planarity **with level test** → at-grade split *or* grade separation | extends `planarize` |
| Network | Connectivity / dead-end policy (closure) | exists (`connectComponents`), see Phase 4 |
| Network | Block-size feedback (too big → inject street; too small → drop) | half-exists (`extractBlocks minArea`) |
| Network | Class-access grammar (local ⊀ freeway without a ramp; arterials form ring backbones) | new — the "grammar" |

**Build the min-arm-angle rule first.** It is small and it kills the original spoke bug at
the source: if no two arms may leave a node closer than `θ_min`, a node physically cannot hold
more than `2π/θ_min` arms — the pathological hub is unrepresentable, and two roads that want to
leave too close either *fuse* into one arm or the node *splits* into two a short edge apart.

---

## Phase 2 — Crossing resolver + interchange templates  *(ADR-0053)*

A single classifier decides what each crossing *becomes* from
`(classA, classB, levelDiff, degree, angle)`:

- 2 locals, at-grade, deg ≤ 4 → **junction patch** (today's analytic pad)
- many arms / large required radius → **roundabout** (ring rewrite: one super-node → a ring
  of degree-3 nodes; every node downstream is degree ≤ 3, the case the pad already nails)
- freeway × surface road, separated → **diamond** (overpass + 4 ramps + 2 surface
  intersections)
- freeway × freeway, separated, all movements → **cloverleaf** (4 loop ramps + 4 connectors)
  or **stack**
- freeway T → **trumpet**
- two roads below the acute threshold → **merge/fork**

Each output is a **parametric template macro** — an L-system production: the crossing is the
non-terminal, the template is the production, the classifier picks which fires. A template
rewrites the abstract crossing into concrete nodes + edges + ramps + vertical profiles, then
hands the result *back through Phase 1* so the ramps themselves obey radius/grade/clearance.

Build order inside this phase: patch (exists) → roundabout → diamond → trumpet →
cloverleaf/stack (hardest; needs Phase 3 solid first).

---

## Phase 3 — Vertical / grade-separation engine  *(ADR-0054)*

Extends `roadProfile` into a real vertical model:

- **Clearance solver.** At any grade-separated crossing, force ≥ clearance between the two
  decks by pushing one road's profile up (bridge) or down (cut/underpass), within
  approach-grade limits.
- **Bridge structure emitter.** Deck thickness, abutments, piers to terrain; approaches reuse
  the corridor cut/fill (`roadConformRegions`).
- **Underpass.** A profile dip + deep terrain cut with a deck overhead (we already cut/fill).
- **Waterway crossing = bridge.** A road crossing water is the *same* template as an overpass:
  the water surface is the lower "deck," the road bridges it. So waterways reuse this engine
  outright (see Extensibility).

---

## Phase 4 — Multi-scale world layout: where places, highways, and blocks come from  *(ADR-0055)*

Phases 0–3 make a road network *legal and buildable*. Phase 4 decides **what to build and
where** — the metropolitan areas, towns, beach towns, freeway arteries, and block density the
world needs — as a **top-down stack of layers, each reading fields from above and writing
fields/seeds for below.**

**Layer A — Region / suitability fields.** From the heightfield (exists) and water, derive
scalar fields: *buildability* (flat + low grade), *water-proximity*, *coastalness*,
*resource/attractor* maps. Pure data, no roads yet.

**Layer B — Settlement placement (the "important waypoints").** Weighted Poisson-disk
sampling on the suitability field places settlements, each with a **class** (metropolis /
city / town / village / beach town), a **radius/population**, and a **character** (grid
downtown / radial old-core / organic / coastal strip). Spacing rules keep metros far apart
and let villages cluster. This *is* the "where the important places are" decision — a ranked
settlement graph.

**Layer C — Inter-settlement network (the highway skeleton, and closure).** Connect
settlements with a graph (Delaunay-pruned / gravity model: big pairs get `Freeway`, smaller
get `Arterial`). Highways route along the **cost field** (avoid water/steep, or pay to
bridge). **Closure is enforced here** (see below): no highway dangles at the map edge; every
leaf is a settlement; lone settlements get a **ring road (beltway)** so the highway returns —
"highways loop to guide the player between places."

**Layer D — Intra-settlement network (grid vs radial vs mixed).** Per settlement, seed the
tensor field from its **character**: metro downtown → grid; old core → radial w/ roundabout;
coastal town → a strip following the coastline; mixed → the radial-core→grid-rim blend
`tensorRoads` already produces. The Layer-C highway enters the settlement boundary and
**degrades via the class grammar**: highway → interchange/exit (diamond, Phase 2) → arterial
→ collector → local grid. *This is the answer to "when does a road become city streets vs an
exit onto a highway": the settlement boundary, sized by the Layer-B radius.*

**Layer E — Blocks & parcels (dense vs sparse).** Planar faces of the local graph = blocks
(`extractBlocks` exists) → lots. A **density field** peaking at the settlement centre and
falling off sets each block's target size, feeding the block-size feedback rule (Phase 1):
small dense downtown blocks, large sparse suburb blocks. *This is "how blocks get decided" and
"whether they spring up dense/radial/mixed" — read from character (D) + density (E).*

### Closure: no dead ends, no map-edge stubs

A bounded map must not leak roads off its edge. Two enforcement levels:

- **Inter-settlement graph (Layer C) is closed:** min-degree ≥ 2 at every node, and **no node
  on the map boundary**. A settlement of highway-degree 1 gets a return loop / beltway. The
  map boundary acts as a **redirect field** that arcs any approaching road back inward toward
  the nearest settlement or merges it into a perimeter ring. Highways therefore *arc toward
  something* and form loops, never terminate in space.
- **Local rule (Phase 1) dead-end policy:** a stub that is not an intended cul-de-sac must
  connect onward or be pruned.

---

## Extensibility — how new world systems drop in

**The invariant:** a new world system contributes **fields**, **rules**, and/or **crossing
templates** — and *nothing else*. It never adds a pipeline stage. Concretely, the extension
points are: (1) a suitability/cost/density **field** (Layer A/E), (2) a placement **attractor**
(Layer B), (3) a **rule** in the registry (Phase 1), (4) a **crossing template** (Phase 2),
(5) a **road class** with its own radius/grade/access numbers (Phase 0).

**Worked example — waterways (not yet built).** A river + coastline plug into *four existing
hooks* and add zero pipeline:

1. **Barrier/cost field (Layer A → C routing).** Highways and streets avoid water, or pay a
   bridge cost to cross — the same cost-weighted routing as ADR-0047 earthwork weighting.
2. **Attractor field (Layer B).** Coastline raises *coastalness*, so beach towns place along
   the shore and acquire the "coastal strip" character.
3. **Crossing template (Phase 2/3).** A road × water crossing resolves to a **bridge** — the
   very same grade-separation template as an overpass, water as the lower deck.
4. **Land-use mask (Layer E).** Blocks/parcels don't spawn on water; faces clip to the
   shoreline.

So "add waterways" = one field source + one mask + reuse of the bridge template. The same
shape covers future systems: **rail** (a class + a level-crossing template), **parks/green
space** (a land-use field that suppresses roads), **zoning/districts** (a character field),
**lore landmarks** (seed points in Layer B), **trade routes/resources** (attractor fields).
If a feature can be expressed as "where is it" (a field) and "what happens when a road meets
it" (a rule or template), it extends the system without touching the core.

---

## Build order (each step shippable, lowest-risk first)

1. **[DONE]** **Min-arm-angle + roundabout-promotion rules** on the existing 2D graph — fixes
   the original spoke bug, validates the constraint-pass shape. *Shipped:
   `road_constraints.{h,cpp}` (`applyConstraints`, `nodeNeedsRoundabout`), wired into
   `buildRoadNetMesh`; `test_road_constraints`.*
2. **[PARTIAL]** **The Phase 1 pass + rule registry**; editor calls it on drag. *Shipped: the
   pass runs on every editor regen via `buildRoadNetMesh`, and terrain-conform shares it
   through `constrainedNetGraph` (mesh + ground agree). Still to do: a formal rule registry
   and a live editor warn/preview using `nodeNeedsRoundabout`.*
3. **[DONE]** **Phase 0 layered graph** + planarity-with-level-test. *Shipped: `RoadEdge.layer`,
   `planarizeLayered` (same-layer → intersection, cross-layer → overpass),
   `gradeSeparationCount`; `test_road_layers`.*
4. **[DONE]** **Phase 3 clearance + bridge**. *Shipped: `clearanceProfile` (the vertical
   solver), `bridgeDeck` (the deck slab), `RoadNet.edgeLayers` + JSON `edge_layers`, and the
   layered build (`buildLayeredRoadNetMesh`): ground roads flat, each bridge chain lifted onto
   a deck that clears the roads it crosses. Demo: `road_overpass.json`. Tests: `test_road_layers`.
   Still to do: piers/abutments under the span, and the water-as-lower-deck reuse.*
5. **Diamond template** (freeway × arterial) — the simplest interchange; needs 1–4.
6. **Dual-carriageway freeways + ramps**, then **trumpet**, then **cloverleaf/stack**.
7. **Phase 4 Layers A/B** (suitability + settlement placement), then **Layer C** (highway
   skeleton + closure), then **D/E** (character-seeded streets + density blocks).
8. **Mountain roads** — mostly tuning the existing contour coupling (`slopeAlign`) +
   hairpin/turning-bulb (`hairpinDeflection`) under the new class rules; little new code.
9. **Waterways** — the extensibility test: prove a real feature lands as fields + mask +
   bridge-template reuse, touching no pipeline stage.

## Testing

Follow the existing pattern (`test_road_net.cpp`, `test_curb_return.cpp`): pure geometric
assertions per rule, all headless — arm-angle floor honoured, clearance ≥ threshold, template
ramp radii ≥ class min, no edge folds, highway graph has no degree-1 / boundary nodes (closure),
block sizes within `[min,max]`. Templates assert ramp count + connectivity, not pixels.

## How the listed features each fall out (none is bespoke)

- **Over/underpass** = Phase 0 layer + Phase 1 planarity-with-level-test + Phase 3 deck.
- **Multilane freeway** = `Freeway` class → dual-carriageway mode + lane-count-from-width;
  its "no at-grade crossings" access rule is what *forces* interchanges to exist.
- **Exit ramp** = a `Ramp`-class connector whose endpoints are taps (edge splits) on two
  roads, with merge/diverge tapers (no-acute-merge rule) and its own min radius.
- **Cloverleaf** = the freeway×freeway template: through-roads separated (Phase 3) + 4 loop
  ramps + 4 connectors (Phase 2), each legalized by Phase 1.
- **Winding mountain road** = `Local` class (tight radius) + max-grade rule + tensor contour
  coupling (`slopeAlign`) + hairpin/turning-bulb (`hairpinDeflection`). Grade limit on steep
  ground *forces* switchbacks; the hairpin rule turns each into a clean turning bulb.
- **Waterways** = fields + mask + bridge template (Extensibility, above).
- **Where towns/highways/blocks go** = Phase 4 layers A→E.
</content>
</invoke>
