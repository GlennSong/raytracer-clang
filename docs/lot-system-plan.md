# The Lot System — a designed site, not a building on a polygon

Status: **substrate built, not yet wired into the city** (August 2026).
Supersedes `building-floorplan-plan.md` and absorbs the unbuilt half of
`building-grammar-plan.md` (P3 lot-driven composition, P4 curtain panels).
Not a patch on `city_lots.cpp` + `shape_grammar.cpp` — a replacement substrate
those two become thin consumers of.

## What is built

Every phase P0–P9 below has landed as a tested headless module. **130 test
cases across seven files**, built by `tools/lot_system_build.sh`.

| Phase | Module | What it gives you |
|---|---|---|
| P0a | `shape2.{h,cpp}` | `Shape2` — regions with holes, true circular arcs (bulge), edge tags; the `Pen` turtle |
| P0b | `shape_ops.{h,cpp}` | arc-preserving booleans, topology-changing offset, fillet/chamfer, the sampled-field fallback |
| P1 | `plan_grammar.{h,cpp}` | selectors, plan ops, the quality invariant, 13 named templates |
| P1 | `mass_stack.{h,cpp}` | tiers, per-band floor heights, profile/loft/twist, the support rule |
| P2 | `facade_plan.{h,cpp}` | the shared bay grid, openings as data, the element registry |
| P4 | `lot_program.{h,cpp}` | lot tags, the inscribed-rectangle measurement, programs, quotas |
| P3 | `site_plan.{h,cpp}` | subtractive zone allocation, boundaries, gates, furnishing |
| P5 | `material_set.{h,cpp}` | coherent palettes, district families, per-building perturbation |
| P9 | `building_recipe.{h,cpp}` | 34 recipes as data rows + `buildFromRecipe`, which composes every layer |
| P6–P8 | `lot_fixtures.{h,cpp}` | prop instance batches, interactable/hinge/sensor/seat specs, the unified `LightBudget` |

**What is NOT done**, stated plainly:

* **Nothing is wired into the shipping city yet.** `city_lots.cpp` and
  `shape_grammar.cpp` still run the old path; Piedmont looks exactly as it did.
  The substrate exists and is tested, but becoming "thin consumers of" it is
  the next body of work, and it is deliberately a separate reviewable step.
* **No mesher.** These modules produce plans, levels, placements, specs and
  palettes — the *decisions*. Turning a `Level`'s `Shape2` into walls with
  thickness, and an `ElementPlacement` into the existing emitters' geometry, is
  the bridge to `shape_grammar.cpp`.
* **No Lua yet** (§17.8). The recipes are C++ data rows with no logic in them,
  which is the shape a Lua table needs — but the binding is unwritten.
* **No Qt Building Lab** (§17.9).
* **Physics and rendering bridges are unverifiable here**: the Jolt submodule
  cannot be fetched in this environment, so `HingeSpec`/`SensorSpec` stop at the
  specification boundary, and `InstanceBatch` still needs ADR-0041 Phase 2.
* **Nothing has been seen on a screen.** There is no GPU in this environment.
  Every claim below is backed by a headless test, not by an image.

---

## In plain terms

**What we do now.** The generator picks a piece of land, then shrink-wraps a
building onto its exact shape. Whatever is left over on the lot — a garden, a
path, a car park — only appears when a lot *fails* to get a building. Gardens
are a consolation prize.

**What this proposes.** Treat a piece of land the way a real developer does.

1. **Decide what the land is for** — a house with a garden, a shop meeting the
   pavement, a tower with a public square. This decision carries its own
   numbers: how much of the land the building may cover, how far back from the
   street it sits.
2. **Divide the land into areas** — building, front garden, paths, back yard,
   parking, bin store. Each area is carved out of what's left of the previous
   one, so **two things can never end up on top of each other.** That's a
   property of how it's built, not something we check for afterwards.
3. **Furnish each area.** The building fills the building area. Trees and lawn
   fill the garden. A fence runs along the boundary — and a gate appears
   wherever a path crosses that fence, because the geometry demands one.

**Then draw the building as a real floor plan.** Walls with thickness, doors
with the arc they swing through, windows in the walls. Not a sketch — the actual
drawing the 3D builder later reads. Today the 3D code decides where windows go
while it is building geometry, so nobody can see the decision or check it. If
the plan decides instead, we can look at it on paper.

**Buildings go up as a stack of floor plans.** Each floor's plan has to be
*held up* by the one below — mostly that means sitting inside it, which gives
setbacks, roof terraces (the exposed roof is just "the floor below minus the
floor above"), and several towers rising off one shared base. Loosening how much
support is required is what lets an upper floor jut out over the ground floor,
the way a cantilevered house or a balconied tower does.

**The designs live in text files, not in code.** A file describes one kind of
building: its floor heights, which walls get which sort of window, what it's
made of. The program code only knows *how* to draw a wall or punch a window —
never *which* building it's drawing. **Adding a new architectural style should
be a new file, never a code change.** That's the test of whether we got this
right.

**Lots are cut to fit buildings, not the other way round.** A kind of building
declares the smallest rectangle it can be built on, and a piece of land only
becomes a lot if it holds one. Land that can hold nothing becomes open space on
purpose. That is what stops a skinny scrap of ground getting a tiny triangular
house, or a hundred-storey tower landing on a plot too small to carry its lifts.

**Special buildings are rationed.** If "twisted tower" is a dice roll per
building, a city gets fifty of them and none of them feels special. Instead the
city decides up front: *one* signature tower, placed on the best site. The code
that already places exactly one courthouse per city does this job.

**We can review all of it on paper.** Everything above is 2D, and the companion
tool draws it as diagrams you can open in a browser. That matters practically:
this project can't run its 3D viewer in the environment where the code gets
written, so being able to judge the design without a 3D view is the difference
between reviewing it and guessing.

### Words this document uses

| Word | Means |
|---|---|
| **lot** / **parcel** | one piece of land, the thing a single building sits on |
| **block** | the area enclosed by surrounding streets; gets cut into lots |
| **plan** / **floor plan** | the outline of a building seen from above |
| **footprint** | the ground the building actually covers |
| **setback** | how far the building is held back from a boundary |
| **coverage** | what fraction of the lot the building covers |
| **frontage** | the side facing the street |
| **party wall** | a wall shared with the building next door — no windows in it |
| **fenestration** | the arrangement of windows and doors on a wall |
| **poché** | the solid black of a wall in an architectural drawing |
| **loft** | morphing one floor plan into a different one as the building rises |
| **cantilever** | an upper floor jutting out past the one below it |
| **plate** | the floor area of one storey of a tower |
| **profile** | how the plan's *size* changes with height (a taper, a bulge) |
| **recipe** | a text file describing one kind of building |
| **selector** | a phrase in a recipe naming which walls a rule applies to |
| **boolean** | adding, subtracting or intersecting two shapes |
| **distance field** | a shape stored as "how far to the edge from here", which makes blending and merging easy |
| **arc edge** | a wall stored as a true curve rather than as many short straight pieces |
| **`Shape2`** | our name for a shape that has an outline *and* holes in it |

---

## 0. Thesis

**The lot is the unit of composition. The building is one occupant of it.**

Today the pipeline is `lot polygon → building`, and everything else on the
parcel is a consolation prize for lots that failed to build (`emitGreen`,
`sculptYard`, `sculptPark`, `sculptPlaza` — four ad-hoc furnishers reachable
only through rejection paths). The building takes the lot's own shape because
that was the cheapest way to stop it overhanging the sidewalk, not because
anyone designed it.

Invert that. A lot is **programmed**, then **partitioned into zones**, then each
zone is **furnished**. A building is what furnishes the `Building` zone. A
garden furnishes `Open`. A gate furnishes the point where `Circulation` crosses
a `Frontage` boundary. Nothing overlaps, because the partition is a
subtract-as-you-go tiling of one region — an invariant, not a rejection test.

Three consequences fall out immediately, all of which the current system fights:

* A building no longer needs to be lot-shaped. It is fitted into a *reserved
  envelope* that is smaller than the lot and knows which side faces the street.
* A 100-storey tower and a cottage differ in their **program**, which sets both
  the envelope and the open space around it — a plaza is a zone, not a
  fallback.
* Landscaping, paths, fences, gates, lighting and seating are first-class
  occupants with geometry, collision and behaviour — not decoration sprinkled
  after the fact.

---

## 1. What we keep, and what we retire

The current system is not a failure; it is a set of well-tested parts wired to a
model that ran out of room. Keep the parts, replace the model.

**Keep, largely as-is**
* The 1-D **facade splitter** (`emitFacadeRect`) and the **opening element** —
  arched heads cut into the wall, frame in the reveal, glass in the frame,
  sill/hood/voussoirs. This is the best code in the subsystem.
* The **part/material split** (`PartId` → `materialFor` → baked PBR surfaces).
* The **mesh-op vocabulary**: `latheMesh`, `emitPortico`, `emitSteeple`,
  `emitEntranceSteps`, `emitBalconyRun`, the roof-plan furniture packer.
* The **road-clearance discipline** and the terrain **pad/plinth/foundation**
  machinery — hard-won from device rounds.
* The **architect's district → weighted recipe table** idea, and the
  **landmark quota planner** (place civic anchors, never roll them).
* Determinism throughout: seeded, headless, unit-tested.

**Retire**
* `BuildingParams` as ~45 flat bools. It is the direct cause of "new
  architecture = new C++" and it cannot express a building with two different
  wings.
* `floors` + `setbackEvery` as the only vertical model.
* The plan as a bare `Poly2` with no holes, no arcs, and no edge identity.
* `growLotBuildings`'s six rejection counters as the density mechanism.
* The four ad-hoc `sculpt*` furnishers.
* The `city.cpp` / `city_lots.cpp` duplication (see §11).

---

## 2. How the pieces are layered

Follows the layering ADR-0028 already ratified, extended one layer down and one
layer out.

```
L3  Site       parcel → program → site plan → occupants          (NEW)
L2  Recipes    Lua data: lot programs, building recipes, palettes
L1  Grammars   PlanGrammar (2D) · MassStack (2.5D) · Facade+Elements (1D)
L0  Kernel     Shape2 · booleans · offset · fillet · arcs · pen · 2D SDF (NEW)
```

The rule that makes this work: **each layer's output is the next layer's input,
and every layer is pure, seeded, and headless-testable.** No layer reaches into
the renderer or the ECS; the host adapts the final `SitePlan` into entities.

---

## 3. The shape toolkit — 2D geometry we can trust (layer L0)

This is the foundation and the riskiest piece. Everything above it is
straightforward once it exists; nothing above it is possible until it does.

### 3.1 `Shape2` — a region, not a ring

```cpp
struct Arc { Real bulge = 0; };        // sagitta/halfChord, DXF convention
struct Edge2 { Vec2 a, b; Real bulge = 0; EdgeTag tag; };
struct Loop2 { std::vector<Edge2> edges; };     // closed; CCW outer, CW hole
struct Shape2 {
    Loop2 outer;
    std::vector<Loop2> holes;
};
```

Three things the current `Poly2` cannot do, all of which the brief requires:

1. **Holes** — courtyards, atria, a plaza with a planting bed, a block with a
   court. `triangulateWithHoles` (`triangulate.h`, an earcut port) already
   exists and is currently called with an empty hole list at its only site;
   `polygonUnion` (`road_offset.h`) already returns exterior loops CCW and
   interior holes CW. **The hole machinery is built and unwired.**
2. **Arcs as edges** — "round a square into a circle" is a per-vertex fillet
   that must survive as an arc, not immediately tessellate. Bulge lets a
   rounded corner stay one edge through booleans, offsets and facade splitting,
   and tessellate once, at the end, at a chord tolerance the caller picks.
3. **Edge identity (`EdgeTag`)** — `street | side | rear | court | party`.
   Today the entrance edge is recomputed by dot product every time anything
   needs it. Tagging edges once, at plan-authoring time, is what lets the
   facade grammar spend ornament on the street and keep the rear plain, and
   lets a party wall be blank by construction.

### 3.2 Operations

| Op | Notes |
|---|---|
| `unite / subtract / intersect` | Generalize `polygonUnion`'s method (split edges at crossings, classify sub-edges by midpoint containment, chain survivors). Only the *keep rule* differs per op — this is a small, well-understood extension of code we already trust at road junctions. |
| `offset(shape, d)` | A **topology-changing** offset: inward offset of a U splits into two shapes; outward offset merges. Returns `std::vector<Shape2>`. Today's `offsetPlan` returns a fixed vertex count and cannot do either. |
| `fillet(vertex, r)` / `chamfer(vertex, d)` | Per-vertex, emitting an arc edge / a straight cut. Also `filletAll(r)` with a per-vertex feasibility check. |
| `roundToCircle(shape, t)` | The brief's "square → circle": interpolate each corner's fillet radius toward the inscribed limit. Falls out of `filletAll`. |
| `simplify(shape, tol)` | Merge short edges, drop near-collinear vertices. Exists inline in `city_lots.cpp:1543`; promote it. |
| `tessellate(shape, chordTol)` | Arcs → chords, at the end. |

### 3.3 Two implementations, one interface

Exact loop algebra is fast and preserves sharp corners, but is fiddly at
degeneracies (collinear shared edges — `polygonUnion` already documents this
limitation). A **2D signed distance field** is the robust complement: rasterize
the shape to a grid, do min/max/offset arithmetically, contour with marching
squares, simplify. Offsets, fillets, and topology changes are free; sharpness
and exactness are lost.

**Decision: ship both behind one interface**, default to exact, fall back to the
field when the exact path detects a degeneracy or when the caller asks for an
organic result. This directly answers the brief's "real-time SDF" instinct —
and a 2D field is orders of magnitude cheaper than the 3D `polygonizeSdf` we
already run for rocks and trees.

### 3.4 The pen

A 2D turtle, sibling to the L-system's 3D turtle (`lsystem.cpp`), producing a
`Shape2`:

```lua
local p = pen.new()
p:move_to(0, 0):forward(18):turn(90):forward(12)
 :arc_to(x, z, bulge):turn(90):forward(18):close()
```

Both an absolute path API (`move_to/line_to/arc_to`) and a relative turtle API
(`forward/turn/left/right`), because floorplans want both: a facade is a turtle
walk, a boolean cut is absolute. This is what makes plans **authorable data**
rather than C++ literals.

### 3.5 Also needed: 3D finishing ops

The brief asks for fillet/chamfer/bevel on the mesh side too. Scope them
narrowly and honestly:

* **Edge bevel on extruded prisms** — cheap and high-value (a chamfered
  building corner, a bevelled sill). Implementable as a 2D chamfer + a
  vertical profile, not as a general mesh-kernel bevel.
* **Profile sweeps** — sweeping a 2D profile along a `Shape2` outline. This is
  what a cornice, a coping, a curb, a handrail and a fence rail all are. The
  road mesher already sweeps profiles along centrelines (`ribbonOutline`);
  generalize it to closed loops. **One op, many features.**
* **General mesh boolean / arbitrary-mesh fillet — explicitly out of scope.**
  That is a B-rep kernel. Where organic blending is genuinely needed, use the
  existing SDF path (`sdfSmoothUnion` + `polygonizeSdf`) as ADR-0022 intends.

---

## 4. Drawing a floor plan by rules (layer L1)

Rewrites `Shape2` regions. This is the brief's "start with a box, iterate each
edge, outset, grow wings, inset the back face."

```
plan := seed(shape)
      | outset(plan, selector, depth)     -- push edges out: bays, wings
      | inset(plan, selector, depth)      -- pull edges in: light wells, recesses
      | wing(plan, selector, w, d)        -- a perpendicular limb off an edge
      | court(plan, selector, w, d)       -- subtract a bite: U, L, donut
      | fillet(plan, selector, r) | chamfer(plan, selector, d)
      | unite/subtract(plan, plan)
      | clip(plan, envelope)              -- ALWAYS last: conformance guarantee
```

A **selector** names edges by tag, by orientation relative to the street, by
length, or by index — so a rule reads *"outset the two longest side edges by
2 m"* rather than hard-coding vertex numbers.

Two invariants make this safe, and both are cheap unit tests:

* **Containment.** Every op is followed by `clip` against the reserved
  envelope. A plan can never leave its zone. (Today's equivalent is the
  progressive-inset-until-clear loop plus a corner-pocket rescue — a search
  where this is a guarantee.)
* **Viability.** After each op, reject the result if any wall is shorter than a
  minimum or any wing narrower than a room. This is where "no weird triangle
  buildings" is actually enforced — as a rule on the *plan*, not a rejection of
  the *lot*.

**Named plan templates** (`bar`, `L`, `U`, `H`, `T`, `courtyard`, `flatiron`,
`cruciform`, `chamfered-square`, `curved-corner-slab`, `pinwheel`) are just
short scripts in this grammar, fitted to the envelope's frame and clipped.
Adding a template is Lua data, not C++.

### 4.1 Composed plans — the grammar actually used

The list above is thirteen *single shapes*: a rectangle with a bite taken out,
a corner rounded, a wall bowed. Each is one op on one seed, which meant the
grammar's own machinery — selectors, multi-step scripts, blended unions — was
carrying no weight, and every building on the street was recognisably a box.

The **composed** templates are the answer: a seed rewritten by a *sequence* of
ops, which is what the grammar was built for and what a list of one-shot shapes
can never reach.

| template | what it composes |
| --- | --- |
| `pentagon`, `hexagon` | a regular n-gon seed, so a plan need not start rectangular |
| `radial-wings` | polygon + a `Wing` on **every** face + a disc blended onto each tip |
| `trefoil` | three discs smooth-joined into one organic plate |
| `cross-apse` | cruciform + a true semicircular apse on one arm |
| `atrium-ring` | courtyard + fillet + an entrance notch |
| `lobed-tower` | square + `Bow` on every edge — a clover plate of four true arcs |
| `wedge-tower` | trapezoid + a buildable chamfer on its sharp end |
| `sawtooth-shed` | a shed whose rear wall is notched into structural bays |
| `courtyard-wings` | two wings off a spine, open on one side, closed around a real court on the other |

Two rules make these work at any size the caller asks for, and both are the
template taking responsibility rather than the caller knowing each one's
minimum frame:

* **A feature is spent only if the frame can pay for it.** A fillet whose arc
  would be shorter than a wall, a notch that would eat through the ring it is
  cut into, a wing narrower than a corridor with rooms either side — each is
  skipped, and the plan scales down to the simpler thing a small site actually
  gets built as. A radial-winged plan is a *large-site* plan; below about 30 m
  it becomes a lobed polygon instead of a knobbly blob whose every feature is
  smaller than a room.
* **A cap is tangent, not overlapping.** A disc blended onto a wing tip sits
  exactly on the end face, so its circle meets both side walls tangentially. A
  wider disc pushed further out re-enters the walls at a grazing angle, and a
  cusp is the one thing the plan invariant will not accept.

The blended ones (`radial-wings`, `trefoil`) can only be built by the sampled
field — a smooth join between two masses is not expressible in loop algebra —
and the field returns a *contour*: several hundred half-metre chords. That is
unusable three ways over (no wall long enough for a window, a vertex count
multiplied by the storey count, and every wall below the invariant's minimum),
so the kernel gained the inverse of tessellation: **`fitArcs`** refits a dense
polyline as the few true arcs and straight runs it came from. A 428-chord
blend of two circles comes back as **four arcs** with 0.2 % area error, and
corners survive because a fit that would round one off exceeds its tolerance
long before it gets there.

---

## 5. Stacking floors to make a building (layer L1)

Replaces `floors` + `setbackEvery`.

```cpp
struct Storey { Real height; int count; };           // a repeated floor band
struct Tier {
    Shape2 plan;
    std::vector<Storey> storeys;
    MaterialSetRef materials;                        // a tier may re-clad
    ElementList elements;                            // a tier may re-dress
};
struct MassStack { std::vector<Tier> tiers; Foundation base; };
```

Each tier's plan is produced by the plan grammar **independently**, then
constrained by the **support rule** (§17.4) — `area(above ∩ below) / area(above)
>= minSupport`, with `minSupport = 1` meaning full containment and lower values
admitting a bounded cantilever. This is
literally the brief: *"once we've built a few stories we can create a different
floorplan on top."*

It buys, at no extra cost: podium + tower, wedding-cake setbacks, twin towers on
a shared base (a tier with **two** plans — the stack holds a list per level),
terraces where a tier's roof is exposed, and a base/shaft/crown that can change
cladding at the transition (ADR-0040 Pass B, still owed).

**The foundation is its own object**, not derived from the plan. It has its own
`Shape2` (usually the plan outset by a plinth reveal, but free to differ — a
podium, a terraced base on a slope, a stepped foundation), its own material, and
it is what beds into the graded terrain. The plan rests on it. This is exactly
the brief's separation and it also fixes today's single-flat-pad-per-building
limitation on steep ground.

---

## 6. Windows, cornices and the rest of the facade (layer L1)

The facade splitter stays 1-D and stays as it is. Two changes around it.

### 6.1 The bay grid becomes a shared object

Today `emitFacadeRect` computes `bays = round(width/bayWidth)` privately, so
nothing else can know where the bays are. Extract:

```cpp
struct BayGrid { Real x0, bayWidth; int bays; };
BayGrid bayGridFor(const Edge2& e, const FacadeStyle& s);
```

Computed once per plan edge, consumed by the splitter *and* by every element
that wants to land on a bay boundary. This is a prerequisite for balconies that
line up, pilasters between bays rather than through windows, storefronts that
tile, and — later — interior partitions that don't bisect a window.

### 6.2 An element registry

The thing `building-grammar-plan.md` promised and never built. An element is a
parametric assembly plus a **selector** saying where it goes:

```lua
elements = {
  { kind = "opening",  on = "all",              style = WINDOW_SEGMENTAL },
  { kind = "opening",  on = "edge:street bay:centre floor:0", style = DOOR },
  { kind = "balcony",  on = "edge:street floor:1..",  depth = 0.9 },
  { kind = "cornice",  on = "tier:top",         profile = CORNICE_HEAVY },
  { kind = "quoin",    on = "corner:convex" },
  { kind = "awning",   on = "element:door" },              -- attaches to another element
}
```

Selectors compose over the axes the geometry already has: edge tag, floor
range, bay index, corner convexity, tier. Elements can attach to other
elements. The existing emitters (`emitPortico`, `emitBalconyRun`, `emitPorch`,
`emitBayFront`, `emitSteeple`, the roof furniture) become registered element
kinds with no change to their geometry code.

**This is the change that makes new architecture data instead of C++**, and it
retires the 45-bool struct in one move.

---

## 7. What a building is made of

The brief: *"buildings should be made out of sets of materials."*

```cpp
struct MaterialSet {
    PartId wall, trim, roof, base, accent;
    Vec3 wallColor, trimColor, roofColor, accentColor;
    OpeningStyle window;          // the coherent window for this palette
    const char* name;             // "london_stock_brick", "deco_limestone"
};
```

A set is a **coherent bundle**, drawn as a unit by district × era × height —
preserving ADR-0040's rule that cladding follows structure follows height, while
letting a tier re-clad (glass tower on a stone podium) legitimately. Sets live
in Lua (`style_book.lua` grows into a palette library), which makes "upscale
area / run-down area / industrial / Edwardian" a data axis rather than another
branch in `dress()`.

Crucially, sets are picked **per district and per lot program**, then perturbed
per building — so a street reads as one place with variation, instead of
independent dice rolls per parcel (today's "same-y but random" complaint from
both directions at once).

---

## 8. Designing the whole lot (layer L3)

The new layer, and the heart of the brief.

### 8.1 Parcel → Program

A **program** is the brief for a lot, chosen by district, parcel size, frontage
and corner condition:

```cpp
struct LotProgram {
    const char* name;              // "corner_shopfront", "office_park", "villa"
    Real coverage;                 // building footprint / lot area (0..1)
    Real frontSetback, sideSetback, rearSetback;
    FrontageKind frontage;         // Forecourt|Garden|Plaza|Patio|Direct|Yard
    OpenKind open;                 // Lawn|Garden|Court|Park|None
    int parkingStalls;             // 0 = none; drives a Parking zone
    ServiceKind service;           // Bins|Loading|None
    BuildingRecipeRef building;
    MaterialSetRef palette;
};
```

Note what moved: **setbacks and coverage are program properties, not global
constants.** Today `lotSetback` is one number per district. A villa wants 8 m
of front garden; a shopfront wants zero; a tower wants a plaza on the corner
and nothing at the rear.

#### Minimums decide eligibility; TARGETS decide grain

A program states two rectangles, and conflating them is a bug that hides for a
long time:

* the **minimum** it will accept — what `lotFitsProgram` gates on;
* the lot it is actually **built** on (`targetW` × `targetD`) — what the cutter
  aims at.

Without the second, a recursive cutter can only halve until it happens to fall
under the first, so every lot is `frontage / 2^k` and the grain is an artefact
of the block's dimensions rather than anything a program asked for. Measured
before targets existed: a 110 m downtown block produced 2100 m² tower plates
and a 150 m one produced 1040 m² — **the bigger block gave the smaller lots**,
and whether a tower had a plate wide enough for a shaped plan was luck. The
cutter now stations its cuts on whole target-width lots and divides depth into
a whole number of rows, so the same programs give 1700–2600 m² plates on every
block size from 100 m to 240 m.

Two consequences fall out of the same change. Lot depth is bounded by the row
stationing rather than by a service lane, and the last row is no longer the odd
one out. And the frontage-normal depth is measured by projection instead of off
an axis-aligned bounding box — a block at an angle to the axes is the normal
case, not a special one, and its bounding box is deeper than the block is.

#### The scale fit has to PEAK, not decay

Eligible programs are weighted by how well the land fits them. The first
version only punished programs that were too small for the parcel, which decays
every candidate by the same factor: it changes the magnitudes and never the
**order**. The highest base weight therefore won at every size, and the largest
program in a mix could never be reached at all — `glass_tower` lost to
`office_tower` on a 1600 m² parcel and on a 48 000 m² one alike, so the cutter
always aimed at the smaller plate and the skyscraper program was dead weight in
the list. The fit is now symmetric in log space (half the land, or twice it,
costs the same), so the ranking moves with the parcel: a modest plate is an
office tower, a whole block is a skyscraper site.

#### 8.1a The block a district needs

The same inversion, one level **up**. A block is not a number of metres
somebody liked; it is two rows of the lots that district's programs are built
on, plus the verge — which `blockGrainFor` derives from the programs
themselves:

| mix | derived block | driving program |
| --- | --- | --- |
| downtown | 130 × 224 m | glass tower |
| commercial | 92 × 136 m | office block |
| residential | 72 × 88 m | walkup |
| rim | 284 × 378 m | campus |

The road layer consumes this as a **ratio against the commercial mix**, so a
level's own block size still sets the city's scale and this only decides each
district's share of it. Two places take it: the patch fabric's cell size, and —
through the region-aware `consolidateJunctionSpans` overload that had been
sitting unused since it was written — the arterial skeleton's junction floor,
so downtown consolidates to fewer, bigger junctions and a neighbourhood keeps
its tight ones.

Measured on a generated city, before: financial blocks came out at ~105 m and
residential at ~140 m, so the district with the towers had the *tightest* land
in the city and the variation that did exist was noise from the skeleton rather
than intent. After: financial is the largest grain in the city, commercial the
tightest. The effect is bounded — face size in this generator is dominated by
the arterial skeleton and freeway geometry, not by the fill — but it now points
the right way, and it points that way *because of what each district builds*.

### 8.2 The site plan — zones

```cpp
enum class Zone { Building, Frontage, Circulation, Open, Parking, Service };
struct SitePlan {
    Shape2 lot;
    std::vector<std::pair<Zone, Shape2>> zones;   // disjoint, tile the lot
    Shape2 free;                                  // unallocated remainder
    std::vector<Edge2> boundaries;                // fence/hedge/wall lines
    std::vector<Vec2> accessPoints;               // where paths meet the street
};
```

Allocation is ordered and **subtractive**:

1. **Access.** Where does the lot meet the street? (Nearest sidewalk point per
   street-tagged edge — reuse `snapToSidewalk` from the places layer.)
2. **Setbacks.** Subtract the program's front/side/rear setbacks → the
   buildable envelope.
3. **Building envelope.** Reserve `coverage × lotArea` inside it, biased toward
   the rear for a garden program, toward the street for a shopfront, toward a
   corner for a tower with a plaza. **The envelope is not the plan** — it is the
   box the plan grammar is allowed to play in.
4. **Circulation.** Route paths from each access point to the building
   entrance, and a driveway to `Parking` if the program has stalls. Paths are
   swept ribbons (the same profile-sweep op as §3.5), not decals.
5. **Frontage / Open / Parking / Service** fill what remains, each as a
   `subtract` from `free`.
6. **Boundaries.** Fence/hedge/wall lines run along lot edges by program;
   **a gate is emitted where a Circulation zone crosses a boundary line.** The
   gate is placed because the geometry demands one, not because a die said so.

Every step subtracts from `free`. **Overlap is impossible by construction** —
which is the brief's "make sure they all have their place and don't collide
over one another", solved structurally rather than by rejection sampling.

### 8.3 Furnishing a zone

Each zone kind has furnishers that consume the zone's `Shape2` and emit:

```cpp
struct Furnishing {
    std::vector<RenderMesh> parts;        // by PartId, merged upstream
    std::vector<PropInstance> props;      // instanced: trees, planters, benches
    std::vector<ColliderSpec> colliders;
    std::vector<LightSpec> lights;
    std::vector<InteractableSpec> actors; // gates, seats, doors, switches
};
```

Props place against the zone's **free region** with the existing Poisson-disk
dart-throwing from `ScatterParams` (`minSpacing`, `clusterCount`, `exclude`) —
that code already does spacing, clustering and exclusion; it just needs to run
at lot scale against a `Shape2` instead of at terrain scale against a square.
Each placed prop subtracts its footprint. Same invariant, one level down.

---

## 9. Interactivity

The brief is explicit that a gate must be a real object. This is genuinely new
engine surface — worth saying plainly what does not exist today:

* **No interaction input.** There is no `interact` action in the input map.
* **No trigger volumes.** `PhysicsWorld` exposes boxes/spheres/capsules/meshes
  and `ContactEvent`s from real touches. No Jolt *sensor* bodies are wrapped.
* **No constraints at all** beyond the vehicle's `VehicleConstraint`. There is
  no hinge, no motor, no limits.
* **Props are baked, not entities.** ADR-0041 Phase 2 ("city emits props as
  instances") is accepted and unbuilt; today everything merges into one mesh
  per material class, so there is nothing to attach behaviour to.

So the interactive layer needs, in order:

1. **`InstanceGroup` emission from the city** (ADR-0041 Phase 2) — the
   prerequisite. Without it there are no per-prop entities to make interactive.
2. **`Interactable` component** + an `interact` input action + a proximity
   prompt. Proximity via a cheap spatial query first; sensor bodies later.
3. **Sensor bodies** in the Jolt wrapper (`addSensor`, overlap enter/exit
   events) — the general trigger-volume primitive, useful far beyond gates.
4. **Constraints** in the Jolt wrapper: `addHingeConstraint` with limits,
   friction and an optional motor. A gate then has two honest tiers:
   * **Tier A — scripted swing.** A kinematic body animated by a
     `ScriptBehaviour` on interact. Works with today's physics wrapper. Good
     enough for a garden gate.
   * **Tier B — real hinge.** A dynamic body on a hinge constraint with limits
     and damping. The player pushes it open; it swings back. This is what the
     brief actually asks for and it needs (4).
5. **`Seat`** — an attach point with a pose, plus a camera/controller state.
   Ties into `character-posing-plan.md`; the seat *marker* is cheap, the *pose*
   is the real work.

**Caveat, stated up front:** the Jolt submodule cannot be fetched in this
environment (`third_party/JoltPhysics` is empty), so all physics work here is
written against the documented API and verified on device later — the same
posture the register already records for the vehicle and character work.
Design and pure logic stay headless-testable; the Jolt bridge does not.

---

## 10. Lighting and the night

The brief wants lights that make the night interesting. There is a hard wall:

**`RT_MAX_LIGHTS = 32`** (`shaders/metal/shader_types.h`), forward-lit, and the
street-lamp system already spends 14 of those on a camera-relative
nearest-N selection (`render_system.cpp:61`). A city where every lot has a
porch light, two path lights and a gate lamp is thousands of lights. Adding lot
lighting without addressing this just starves the street lamps.

Three tiers, in order of cost:

1. **Emissive geometry + baked pools** for everything. Lamps, windows and sign
   faces get an emissive material; a cheap unlit "pool" decal on the ground
   under each fixture. No light budget consumed. This alone transforms the night
   look and is the same technique the vehicle lamps already use.
2. **A unified light budget.** Generalize the lamp system's nearest-N selection
   into a `LightBudget` that ranks *all* candidate lights (street, lot, window,
   vehicle) by distance × intensity and fills the 32 slots each frame. One
   mechanism instead of one per system.
3. **Clustered / tiled forward lighting** — the structural fix, its own ADR, and
   the prerequisite for genuinely many lights. Also unblocks the local-light
   shadows the register already names as owed.

Lot lighting should be designed as *fixtures with an emissive mesh and a
`LightSpec`*, so tier 1 works immediately and tiers 2–3 upgrade it without
touching the recipes.

---

## 11. Parcelling and the two pipelines

Two structural cleanups this work should carry, because it cannot avoid them.

**Variable lot sizing.** The brief wants big lots for big programs. Today
`subdivideBlock` cuts a block into roughly uniform frontage strips and the
architect picks whatever fits. Invert it: the district emits a **program mix**
(*this block wants one anchor + six shopfronts*), the parceller cuts to fit that
mix, and adjacent parcels can be **merged** for a large program (office park,
big-box + lot, campus, cathedral close). Parcel merging is a `unite` on
`Shape2` — free once L0 exists.

**One pipeline.** `city.cpp` (ADR-0038, `shape:"city"`, still bound to
`city.json` / `city_arena.json` and the offline tracer) has its own
`paramsForDistrict` and never calls the architect. `city_lots.cpp` (ADR-0066)
is the living one. The site layer must be the single producer, with both hosts
as consumers — otherwise this work doubles the divergence instead of ending it.
Concretely: `city.cpp`'s block loop calls the site layer; `paramsForDistrict`
is deleted.

---

## 12. Phasing

Each phase is shippable, headless-testable, and useful before the next lands.

| Phase | Contents | Test |
|---|---|---|
| **P0** | **2D kernel**: `Shape2`, booleans, topology-changing offset, fillet/chamfer, arcs, `simplify`, `tessellate`, the pen; 2D SDF fallback path | Property tests: area conservation, no self-intersection, hole preservation, offset round-trip, arc chord error |
| **P1** | **Plan grammar + mass stack**; `growPlanBuilding` re-founded on `MassStack`; `noop` as a weighted op (§17.5); the support rule (§17.4) | Plan-quality invariant after every op; support/overhang bounds; determinism |
| **P2** | **Bay grid + element registry**; retire `BuildingParams` bools; port every existing element | Element selectors resolve to expected bays/edges; existing buildings visually unchanged (part-count/vertex-count goldens) |
| **P3** | **Site layer**: programs, zone allocation, furnishers; retire the four `sculpt*` paths | **Zones are disjoint and tile the lot**; props never overlap; every path connects an access point to an entrance |
| **P4** | **Parcelling rework** (§17.1–17.3, 17.6): programs declare minimum rectangles, lots are cut to hold them, every lot carries its tags; rim blocks parcel coarse | No lot ships that fails its program's minimum; the six rejection counters are gone |
| **P5** | **Material sets** + palette library; ADR-0040 Pass B (tier re-cladding, crown kit) | Palette coherence per district; height→structure→cladding rule holds |
| **P6** | **Instancing** (ADR-0041 Phase 2) — props as `InstanceGroup`s | Instance counts; per-instance culling |
| **P7** | **Interactivity**: `Interactable`, interact action, sensors, hinge constraint, gates (Tier A then B), seats | Pure logic headless; Jolt bridge device-verified |
| **P8** | **Lighting**: fixtures + emissive, unified `LightBudget` | Budget selection determinism; no starvation of street lamps |
| **P9** | **Recipe library** — port the existing 55 (see below), then the long tail: cathedral, big-box, twin-tower, curved slab, stacked | Each recipe reachable, coherent, deterministic |

### The recipe port

The existing `architect.cpp` carries **55 recipe functions** (44 district
archetypes + 11 landmarks), and they all have the same five-slot shape: a floor
range, a ground-floor mode, a `dress()` cladding bundle, a handful of element
flags, a place type. **That uniformity means the port is mechanical** — and it is
the strongest evidence that these are data typed as code.

Audited in full (detail and before/after drawings in `lot-lab-findings.md`):

* **46 port directly** — table only, no new capability needed.
* **5 need several masses per tier** — `office_park`, `strip_mall`, `church`,
  `market_hall`, `hospital`. All five today emit one prism where the name
  promises a complex.
* **6 need per-band floor heights** — `hotel`, `factory`, `warehouse`,
  `capitol`, `library`, `museum`. They already fight this by overloading
  `floorHeight`/`groundHeight`.
* **8 collapse into other recipes with different numbers** — `civic_hall`/
  `civic_midtown` differ by a floor range and a coin flip; `bungalow`/
  `craftsman` by one bool. The port *reduces* 55 → ~47 while making each more
  capable.
* **2 are not buildings** — `pocket_park`, `plaza` become lot programs (§8.1).

**Crucially: nothing in the 55 needs a verb the vocabulary lacks.** That was the
question worth answering before freezing the strategy and element sets — anything
they could not express would have been a missing operation. Every gap turned out
to be expressiveness of the *container* (one mass, one floor height, one window
style), not a missing op. **This is the green light for the P0 vocabulary**, and
it is why the port should be scheduled early rather than last: it is the cheapest
possible validation of the whole design, and it is pure 2D.

Two things the port deliberately does **not** fix, both of which want addressing
in the same pass:

* `coreness` is copy-pasted across six tower recipes with a different constant
  each. That is a *height model* and belongs in the architect above the tables.
* `RecipeCtx` lets recipes read `shortSide`/`area`/`roomy`, so a recipe knows
  about parcels. This inverts: the **program** reads the lot and picks a recipe
  that fits, so a recipe never asks how big its site is.

P0 is the gate. **Nothing else should start before it is solid**, because every
later phase encodes assumptions about what the kernel can express.

### What the build actually found

Each phase was landed with tests written to the invariant, not to the
implementation, and several of those tests found real defects. The ones worth
remembering, because they are design lessons rather than typos:

* **Collinear shared edges broke the boolean.** Two shapes that share a wall
  have no proper crossing, so the shared run was never split and three
  overlapping bars united to 90 m² instead of 260. Uniting a wing onto a box
  shares an edge *by construction* — this is the common case, not a corner one.
  Fixed with a four-way fragment classification (Outside / Inside / OnSame /
  OnOpposite) and a rule per op.
* **Offsetting an arc offset its chord**, so a circle grown by 3 m came back
  the wrong size and shape. An offset arc is a *concentric* arc; edges now
  offset as curves and each bulge is recomputed from where its endpoints land.
* **An over-shrunk square survives every cheap test.** Push a 10 m square in by
  6 m and it comes back as the same square rotated 180°, which preserves
  winding *and* has plausible positive area. Only the definition catches it:
  every point of an offset by d sits |d| from the original boundary.
* **The support rule was being applied between every pair of levels**, which
  clipped a Gherkin's bulge away floor by floor and left a prism. Support is a
  *tier-to-tier* constraint; within a tier the profile curve is the design.
* **Lofting toward the union of two towers is a no-op** — two disjoint shapes
  united are still two disjoint shapes. An arch's loft target has to be the
  mass that *bridges* them.
* **`carve` kept only the largest region**, so a circulation zone split by the
  building lost either its front path or its driveway. Zones are genuinely
  multi-part.
* **xorshift32's first output is strongly correlated for nearby seeds.** The
  "one RNG per building, ask it one question" pattern this whole system is
  built on therefore gave *every* building the same first answer — forty
  buildings on a street and forty different districts all chose the identical
  palette. `Rng` now scrambles its seed. Any code doing `Rng(seed).unit()` was
  affected, not just palettes.

* **Two `engine::ParcelParams`, two `engine::BuildingRecipe`.** The new modules
  named types the old city pipeline already had, with different layouts and
  the same namespace — a textbook ODR violation. It cost nothing while the two
  systems were compiled into separate test binaries and corrupted the stack the
  moment they were linked into one: a default member initializer from the new
  `ParcelParams` (`sharedGreenMinArea = 260.0`) landed past the end of the old
  one and overwrote the caller's vector. Renamed to `BlockParcelParams` and
  `LotRecipe`. The lesson is about the *guard*, not the names: `make test` links
  every test file into one binary and is the only thing that can catch this, so
  it had been failing to link for unrelated reasons long enough for the bug to
  get in. A build target that does not run is not a guard.

The RNG one is the most important: it was invisible in every individual
module and only showed up when a test asked a question about a *population*
("does a street share a family, with a minority that does not?"). Tests that
assert on distributions, not just on single outputs, are what caught it.

---

## 12a. The bridge to the engine — what it took

The system described a building; nothing turned that description into triangles.
`lot_mesh` does, and `lot_city` runs the whole chain per block.

The decisive design choice is that both emit into the types the city path
ALREADY speaks — `BuildingMesh` parts keyed by `PartId`, `LotBuilding` records,
the LOD1 twin, the HLOD proxy. The loader, the renderer, the procedural surface
bake, the box colliders, the chunker and the editor's save/load round trip all
consume those today, so a building described the new way needs no new code
anywhere downstream, and `LotParams::lotSystem` can switch a level between the
two pipelines with everything else none the wiser. A migration that needs a
fork is a migration that never lands.

What the wiring found, all of it invisible until geometry existed:

* **The engine's winding convention is not the obvious one.** `MeshBuilder::
  emitQuad` orders indices so `cross(b - a, c - a)` points AGAINST the shading
  normal. Every mesh in the engine is built that way. Hand-wound slabs that
  disagreed were backfacing against the whole world — and it does not look like
  a winding bug, it looks like a hole in the roof.
* **A dense field contour is a latent performance bomb.** Harmless while it is
  only drawn on an SVG sheet; ruinous once a mass stack repeats it per storey
  and a mesher extrudes every edge. A 24-storey tower whose taper went through
  the sampled field carried 8 684 plan edges and meshed to **13.8 million**
  triangles. Every field result is refit as arcs on the way out now: the same
  tower is 20 000 triangles.
* **`fitArcs` was not idempotent.** Fitting a loop that already carries arcs
  re-derived each arc from its two endpoints — which is a chord — so a second
  pass silently flattened every curve. It now fits over the TESSELLATED loop, so
  a span crossing an arc has real samples to measure against, and a span
  covering exactly one input edge keeps that edge verbatim.
* **Fitting twice costs shape.** The second pass has only the first pass's
  output to measure against, and each round trip through the raster loses a
  little more. The field takes the caller's wall minimum instead, and fits once.
* **A face is not always a block.** Where the road fill fails, `extractBlocks`
  hands back a face a kilometre across; parcelling that at house grain is
  thousands of buildings and gigabytes of triangles, and it OOM-killed the first
  end-to-end run. A face far over its district's grain is now reported and left
  unbuilt.

Reviewed the same way the plans were: `tools/lot_mesh_sheet.cpp` renders every
meshed building isometrically to SVG with a painter's sort, because there is no
GPU on the machine that generates them and "it compiles and the triangle count
is plausible" is not a check that a building looks like a building. It caught
the inverted cull and the missing roofs in one glance.

Measured: 266 triangles for a cottage, ~2 000 for a walkup, ~20 000 for a
36-storey tower. The three detail levels of a glass tower are 22 546 / 756 / 10.

---

## 13. Risks

* **The 2D kernel is the crux.** Robust polygon booleans are a classic source
  of subtle bugs. Mitigations: generalize code we already trust
  (`polygonUnion`), property-based tests over random inputs, and the SDF
  fallback for degenerate cases.
* **Triangle budget.** Richer lots multiply geometry. P6 (instancing) is
  therefore not optional polish — it is load-bearing, and it is also what makes
  P7 possible.
* **Scope.** P0–P3 is the coherent core and delivers most of the visible change.
  P7–P9 are large in their own right and should be re-planned once P3 lands.
* **Device verification.** Physics and shader work cannot be verified in this
  environment. Keep the pure/impure seam sharp so the untestable surface stays
  as small as possible.
* **Determinism.** Every layer must stay seeded and reproducible; the site layer
  adds many new draws, so seed derivation needs a documented scheme (per-lot
  stream, per-zone substream) before P3.

---

## 14. Open decisions

1. **Kernel exactness.** Ship exact-only first and add the SDF path when
   degeneracies bite, or build both in P0? (Recommendation: both — the field
   path is small and de-risks the exact path.)
2. **Where recipes live.** Lua data from the start, or C++ tables ported to Lua
   at P5? (Recommendation: C++ through P2 for speed and testability, Lua from
   P3 — the site layer is where authorability pays.)
3. **Gate fidelity.** Is scripted-kinematic (Tier A) acceptable for v1, with
   real hinges deferred to a physics-constraints ADR?
4. **Interior seam.** ADR-0038 §4 deliberately scoped interiors out. The mass
   stack makes per-storey plates natural. Do we re-open Tier C now, or keep the
   seam and stay exterior-only? (This plan assumes exterior-only, as briefed.)
5. **Does this get an ADR now, or after P0 proves the kernel?** The layering,
   the retirement of `BuildingParams`, and the single-pipeline decision are all
   hard to reverse and span modules — by the AGENTS.md rule they warrant one.

---
## 15. The Lot Lab

`tools/lot_lab.cpp` draws everything above as top-down SVG sheets, so the 2D
design can be reviewed on paper — which is the only part of this subsystem that
needs no GPU to judge. It is the acceptance harness for P0.

```sh
c++ -std=c++17 -O2 tools/lot_lab.cpp \
    src/engine/procgen/city/polygon.cpp -o /tmp/lot_lab
OUT=/tmp /tmp/lot_lab 7
```

**What it has already taught us — including several corrections folded back into
the sections above — lives in `docs/lot-lab-findings.md`.** The short version:
the exact boolean does not need to be axis-aligned (which retires P0's biggest
risk); arc edges have to be a kernel type rather than bezier approximations;
lofting must interpolate distance fields, not vertices; the data/code line sits
at "code owns verbs, data owns nouns"; and anything that would stop being
special if it were common has to be a quota rather than a probability.

---

## 16. Relationship to existing docs

* **Companion:** `lot-lab-findings.md` — what the 2D lab has demonstrated, and
  the corrections it forced back into this document.
* **Supersedes** `building-floorplan-plan.md` (its `FloorPlan`/`PlanEdge` design
  is absorbed and extended into `Shape2` with holes, arcs and edge tags).
* **Absorbs** the unbuilt half of `building-grammar-plan.md` — P3 lot-driven
  composition (now §4), P4 curtain panels (now an element kind), P5 architect
  (now the program layer).
* **Completes** ADR-0040 Pass B via §5 tier re-cladding and §7 material sets.
* **Depends on** ADR-0041 Phase 2 for props and interactivity.
* **Does not re-open** ADR-0038 §4 (interiors stay Tier C) — but §5's mass stack
  is deliberately the seam that generator would fill.

---

## 17. Buildability, tags, and the architect's decision (owner review round 2)

This section answers a review pass that found several things the plan had wrong
or simply unsaid. Drawings: `buildable.svg`.

### 17.1 The inversion: lots are cut to fit buildings, not the reverse

Today the parceller cuts a rhythm and `growLotBuildings` rejects what will not
fit — six rejection counters. That is why a skinny trapezoid gets a tiny
triangular house: **nothing upstream ever asked whether a building could stand
there.** Invert it.

* A program declares its **minimum buildable rectangle** (`minW × minD`) and
  minimum area. This is a *property of the building type*, not of the lot.
* The parceller emits a lot only if it **contains that rectangle** — measured
  by the largest inscribed rectangle in the lot's own oriented frame, which is
  the same shrink-to-fit construction `RectYard` already uses to seat a house,
  promoted to a qualifying measurement.
* Land that can carry no program becomes **open space by design**, not by
  rejection. The six counters disappear; there is nothing left to reject.

`buildable.svg` panel 1 shows the wedge block cut on a rhythm — every lot fails.
Panel 2 shows the same block parcelled to the program's minimum: fewer lots,
all viable, and the residue explicitly handed to open space.

### 17.2 Height is capped by the plate, once

Lifts, cores and structure scale with the storeys they serve, so a 100-storey
tower needs a big floor plate — you cannot put one on 200 m². Today six tower
recipes each carry their own `cx.coreness * N` guess. Replace with **one rule**
above the tables:

```
maxStoreys = min(shortSide / 0.55, plateArea / 26)
```

Numbers are placeholders to be tuned; the point is that it is a *single* height
model the architect consults, not a constant copy-pasted per recipe.

### 17.3 Lot tags: what the architect actually reads

The plan named `LotContext` but never said what it carries or who consumes it.
It is the parceller's output and the architect's input:

| Tag | Source | Used for |
|---|---|---|
| `frontages`, `shape` (corner/through/mid-block/island) | edge tagging (§15.6) | corner shops, chamfers, which walls get ornament |
| `inscribedW/D`, `area` | measured at parcel time | which programs are even eligible |
| `maxStoreys` | §17.2 from the plate | rejects a tower on a small plate |
| `streetClass` (arterial / street / lane) | the road graph edge | retail wants an arterial; a lane gets service |
| `enclosed` | block topology | false on rim blocks → coarse parcels (§17.6) |
| `coreness` | distance to centre | the skyline model |
| `slope`, `padPlane` | terrain | rejects a tower on a steep site |
| `neighbours[]` | adjacency | party walls, terrace continuity, avoiding three identical shops in a row |

**How the architect decides** is then a two-stage filter, not a dice roll:

1. **Eligibility** — drop every program whose minimums the lot fails. A 12 × 14 m
   mid-block lot is simply not eligible for `glass_tower`.
2. **Weighted pick among survivors**, using the district's program mix, biased
   by tags: `shape == Corner` lifts `corner_shop` and `bank`; `streetClass ==
   Lane` lifts service programs; `enclosed == false` lifts campus programs.
3. **Quotas run first** (§15.9) — landmarks and signature massing are *placed*
   on the best eligible lot before the weighted pass sees it.

This is the piece that was missing: cornerness, size and street class are
**facts computed once by the parceller**; the architect only reads them.

### 17.4 Support, not containment — cantilevers

§15.3's "every tier is clipped to the tier below" is **too strong**, and it
would have made Fallingwater and every cantilevered green tower impossible.
Containment is not the real constraint; *support* is:

```
support = area(above ∩ below) / area(above)     >= minSupport
overhang = max distance from `above` beyond `below`'s edge   <= maxOverhang
```

Containment is simply `minSupport = 1`. A recipe opts into an overhang by
lowering `minSupport` and setting `maxOverhang`; the default stays 1 so ordinary
buildings do not start floating. **This replaces the clip rule** rather than
adding a special case to it — the honest version, per the codebase ethos.

Same mechanism, one level down, gives per-floor **balconies**: a balcony is a
small overhanging mass attached to a storey with `minSupport ≈ 0` and a
structural depth cap, placed by a selector (`on = "edge:street floor:2.."`), so
it is the same op as a cantilevered storey at a different scale.

### 17.5 Plan quality is an invariant, and the no-op belongs in the grammar

The divot riding a skyscraper's shaft for sixty storeys started as one bad
vertex in the base plan. **A defect in the base plan is amplified up the entire
stack**, which makes plan validation the highest-leverage check in the system.
So it is an invariant checked *before* a plan is accepted, never a rejection
after geometry exists:

* every wall ≥ `minEdge` (~2.2 m — shorter is not a wall)
* every interior angle ≥ `minAngle` (~42° — the knife-edge gate)
* no notch narrower than a room
* after **every** grammar op, not only at the end

And the smaller point, which the grammar was missing: **`noop` is an op.** Every
plan-grammar step should be a weighted choice *including doing nothing*, so a
face that does not grow is a designed outcome with a probability, not an
accident. This also stops the "every building has every feature" failure mode —
the same disease as everything twisting (§15.9), one level down.

### 17.6 Rim blocks parcel coarse

A block on the city edge has no far-side street, so its depth is unbounded.
That is not a special case to handle — it is a tag (`enclosed == false`) that
admits a different program set: university campus, office park, big-box retail,
works and industrial yards, all of which want parcels far larger than anything
an enclosed block can offer. The coarse grain falls out of program-driven
parcelling automatically once the eligible program list differs.

### 17.7 Mixing materials, and painting inside separately

**Mixing.** A `MaterialSet` is bound by **selector**, exactly like elements — so
a plinth can be granite under brick walls under a copper cornice, and a podium
can be stone under a glass tower. The rule that keeps it coherent is ADR-0040's:
one *family* per mass, with a legitimate second treatment at the base. Mixing is
therefore expressive but bounded, and the bound lives in data.

**Inside vs outside.** A surface gains a **side**: `PartId` becomes
`(part, Side::Exterior | Side::Interior)`. This costs almost nothing now — a
wall's inner ring already exists as geometry once walls have thickness (§15.2) —
and it is the difference between interiors being a rewrite later and being a new
material binding later. Worth doing in P2 even though interiors stay out of
scope (ADR-0038 §4).

### 17.8 The Lua recipe surface

Required by AGENTS.md's Procgen Authoring rule (ADR-0042) — recipes in Lua over
a bound C++ substrate — so this is not a new policy, only its shape for
buildings.

**A recipe is a table with escape hatches.** Declarative for the 95% case;
where a recipe needs real logic it supplies a function, which is ADR-0028's L2
layer working as designed:

```lua
return {
  name = "riverside_tower", place = "office",
  requires = { min_w = 26, min_d = 26, min_storeys = 20, enclosed = false },

  massing = function(lot, rng)              -- the escape hatch
    local p = pen.new():move_to(0,0):forward(lot.w):turn(90):forward(lot.d)
                        :turn(90):arc(18, 90):close()
    return plan.fillet(p, 3.2)
  end,

  stack = {
    { role="lobby",  storeys=1,  height=6.0, plan="podium" },
    { role="office", storeys={20,34}, height=3.9, plan="shaft",
      profile="swell", support=1.0 },
    { role="crown",  storeys=2,  height=5.2, plan="shaft",
      support=0.55, overhang=4.0 },       -- a cantilevered crown
  },

  fenestration = {
    { on="edge:street", strategy="curtain" },
    { on="edge:party",  strategy="blank" },
    balconies = { on="edge:street floor:2..", depth=1.8, every=1 },
  },
  palette = { walls="glass_steel", base="dark_granite", trim="bronze" },
}
```

**What C++ exposes** (the substrate — extends the existing `building.*` surface):

| Namespace | Provides |
|---|---|
| `pen.*` | `move_to / forward / turn / arc / line_to / close` → a plan |
| `plan.*` | `unite / subtract / intersect / offset / fillet / chamfer / outset / wing / court / clip / simplify` |
| `field.*` | `from_plan / smooth_union / offset / contour` — the organic path |
| `stack.*` | `band / loft / profile / twist / support` |
| `fen.*` | the strategy set; `openings(plan, rules)` returns the opening list |
| `elements.*` | the registry: `cornice / portico / balcony / quoin / steeple / …` |
| `lot.*` | **read-only tags** (§17.3) — how a recipe adapts to its site |
| `palette.*` | material sets, bound by selector |

**How expressive.** Anything the pen and the plan ops can describe, which the
composition test (§15.1) already showed is a wide space — and a recipe can go
further by computing its plan in Lua. The bound is the *verb* set, and the
recipe audit found that today's 55 need no verb we lack.

### 17.9 The Building Lab in the Qt editor

Most of this already exists and should be **wired, not rebuilt** (AGENTS.md:
"use the technology you already have"):

| Piece | Status |
|---|---|
| Hot-reload watching | **exists** — `ScriptWatch` was deliberately moved out of `ArenaState` so the Qt editor could drive it |
| Lua recipe → mesh | **exists** — `building.grow_plan_parts`, `assets/scripts/building_lab.lua` |
| A lab scene | **exists** — `assets/levels/building_lab.json` |
| Qt shell + engine viewport | **exists** — `src/editor_app/` |
| Reseed / re-run | partly — the viewer rewrites `opts`; needs a real control |
| **A lot with editable tags** | **new** |
| **A recipe browser + variant grid** | **new** |

So the work is: a **Lab dock** in the Qt editor with (a) the recipe file being
edited, (b) a **tag panel** whose fields are exactly §17.3's `LotContext` so you
can force `shape = Corner`, `enclosed = false`, `maxStoreys = 40` and watch the
architect respond, (c) a seed field plus **"regenerate N variants"** rendering a
grid, and (d) a lot outline you can drag to resize. Save the file → it rebuilds.

Two things to get right, both ethos points rather than features: the lab must
drive the **same** `LotContext` struct the city uses (not a parallel one), and
the variant grid must run the **same** architect pass, so what the lab shows is
what the city builds.

### 17.10 Committing to a massing discipline

A tower should either be organic all the way up or change plan deliberately —
not drift. So the *recipe* carries a **massing discipline**, and the architect
picks a recipe, never a per-storey behaviour:

* `prismatic` — one plan, repeated (most buildings)
* `profiled` — one plan, a size curve (the Gherkin)
* `staged` — plan changes at named band transitions (podium → tower → crown)
* `continuous` — lofted throughout, committed from base to crown

Mixing `profiled` and `staged` inside one band is what produces drift; the
discipline is a per-band property and the validator rejects a band that sets
both a loft target and an incompatible profile.

---

## 18. What the previous generation already did (audit)

Written after the first rendered frames, because the rooftop kit was found *by
accident* and that is not a search strategy. The question this section answers is
the one that should have been asked before `lot_mesh` was written: **what does the
shipping city pipeline already do, and which of it did the Lot System drop?**

### 18.1 The structural finding

`shape_grammar.h` exports seven symbols: `facadeColor`, `growBuilding`,
`growPlanBuilding`, `emitBox`, `emitShell`, `emitParapet` (the box form) and
`emitQuad`. Everything else in that 2 000-line file is `static`.

So the old pipeline is **one entry point, not a vocabulary**. `emitPlanParapet`,
`emitCrown`, `emitPortico`, `emitSteeple`, `emitRotunda`, `emitBayFront`,
`emitPorch`, `emitSawtoothRoof`, `emitCurtainWall`, `emitFlaredRoof`,
`growCylinder`, `growPagoda`, `emitParkingDeckRect`, `emitBalconyRun` and
`emitEntranceSteps` are all file-local. A new mesher **could not** have reused
them without extracting them first.

That reframes the migration. It is not "copy the roof code across" — it is
*promote the old emitters into a shared registry so both meshers call one
implementation*. Anything less produces a third copy, which is what
`AGENTS.md`'s single-source-of-truth rule exists to prevent.

### 18.2 The ledger

| capability | old | new | status |
| --- | --- | --- | --- |
| parapet + coping | `emitPlanParapet:1509` — upstand with inner *and* outer faces, oversailing coping, top and underside caps, mitred via `offsetPlan` | `emitSweptBand` with `project = 0` | **REGRESSED** — zero thickness, no coping, no mitre |
| rooftop kit: access bulkhead, water tanks, louvred HVAC, fan discs, **plus the cell partition that stops them overlapping** | `emitCrown:1020` | — | **DROPPED** |
| portico / entrance steps / columns | `emitPortico:1650`, `emitEntranceSteps:1628`, `emitColumn:1609` | `ElementKind::Portico/Steps` declared | **DECLARED, UNBUILT** (`default: break`) |
| porch | `emitPorch:1763` | `ElementKind::Porch` declared | **DECLARED, UNBUILT** |
| bay window | `emitBayFront:1962` | `ElementKind::BayWindow` declared | **DECLARED, UNBUILT** |
| steeple / spire | `emitSteeple:1914`, `emitSpireCrown:1822` | `ElementKind::Steeple/Spire` declared | **DECLARED, UNBUILT** |
| rotunda / dome | `emitRotunda:1694` | `ElementKind::Dome` declared | **DECLARED, UNBUILT** |
| balcony | `emitBalconyRun:1734` — a run across bays | single-bay tray | **PARTIAL** |
| sawtooth roof | `emitSawtoothRoof:1857` | `SawtoothShed` is a *plan* template; the roof stays flat | **PARTIAL** |
| flared / pagoda roof, cylinder mass | `emitFlaredRoof:933`, `growCylinder:885`, `growPagoda:961` | — | **DROPPED** |
| parking-deck facade | `emitParkingDeckRect:1794` | a `parking` recipe exists; its facade does not | **DROPPED** |
| prop instancing | `CityInstanceGroup` (`city.h:98`) | `lot_city` fills none | **DROPPED** |
| lot fixtures | — | `LotFixtures` is built and never consumed | **UNWIRED** |
| oriented collider | oriented box | `lb.yaw` always 0 | **REGRESSED** (the exact prism collider compensates) |
| multi-mass collider | — | records `levels[0].plans[0]` only | **PARTIAL** |

Of 19 declared `ElementKind`s, `lot_mesh` builds **four** — Cornice, BaseCourse,
Parapet, Balcony — plus `Opening` as a permission flag. The registry advertises
fifteen features it cannot build.

### 18.3 What this changes

1. **Phase 2 is an extraction, not a port.** Promote the file-local emitters into
   the element registry; both meshers then call one implementation.
2. **Do not hand-fix `emitSweptBand`'s mitre.** `emitPlanParapet` already solves
   it correctly on `offsetPlan`; the new side has `offsetShape`
   (`shape_ops.h:77`). Reuse an offsetter, do not write a third.
3. **The registry must not advertise what it cannot build.** Every `ElementKind`
   either gets an emitter or is deleted, so `default: break` stops being a place
   features go to disappear silently.
4. **Instancing is a budget item, not a feature request.** The old path sends
   props through `CityInstanceGroup`; the new one merges everything into
   per-`PartId` meshes. That belongs in the triangle-budget work, with numbers.

### 18.4 Status after the first migration pass

The rooftop kit is across. Its **arrangement** — what a roof carries and how the
pieces are placed so none overlaps — now lives in `roof_plant.h` and is called by
both `shape_grammar`'s `emitCrown` and the Lot System's mesher, so there is one
design and two drawings rather than two designs. The move preserves the RNG call
sequence exactly (`Rng::range(a,b)` is `a + (b-a)*unit()`, so a `unit()` callback
is equivalent), which is why it changed no building that already existed.

`offsetPlan` was extracted the same way: it was file-local in an anonymous
namespace, which is precisely why the Lot System grew its own broken mitre.

**Still unbuilt**, now reported at build time instead of vanishing into
`default: break` (counts from one `make test` run):

| kind | asked for | note |
| --- | --- | --- |
| `RoofDeck` | 60 | includes the **chimney** variant — every recipe that asks for one gets nothing |
| `Steps` | 27 | `emitEntranceSteps` exists in shape_grammar |
| `Quoin` | 21 | — |
| `Porch` | 20 | `emitPorch` exists |
| `Sign` / `Awning` | 18 each | shopfronts read as blank ground floors without them |
| `Portico` | 7 | `emitPortico` exists |
| `Pilaster` | 7 | — |

Five of the seven already have working emitters in `shape_grammar.cpp`, file-local.
The extraction pattern is established; this is the list it should be applied to.
