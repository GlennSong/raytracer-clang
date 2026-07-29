# The Lot System — a designed site, not a building on a polygon

Status: **proposal / research** (owner brief, July 2026). Supersedes
`building-floorplan-plan.md` and absorbs the unbuilt half of
`building-grammar-plan.md` (P3 lot-driven composition, P4 curtain panels).
Not a patch on `city_lots.cpp` + `shape_grammar.cpp` — a replacement substrate
those two become thin consumers of.

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

## 2. Layer architecture

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

## 3. L0 — the 2D kernel

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

## 4. L1a — the plan grammar

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

---

## 5. L1b — the mass stack

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
constrained to sit inside the tier below (`contains(below, tier)` — a real
containment test, not the current three-part `insetOk` heuristic). This is
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

## 6. L1c — facade elements

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

## 7. Material sets

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

## 8. L3 — the site layer

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
| **P1** | **Plan grammar + mass stack**; `growPlanBuilding` re-founded on `MassStack`; existing recipes ported 1:1 | Containment (plan ⊆ envelope), tier containment, wall-length minimums, determinism |
| **P2** | **Bay grid + element registry**; retire `BuildingParams` bools; port every existing element | Element selectors resolve to expected bays/edges; existing buildings visually unchanged (part-count/vertex-count goldens) |
| **P3** | **Site layer**: programs, zone allocation, furnishers; retire the four `sculpt*` paths | **Zones are disjoint and tile the lot**; props never overlap; every path connects an access point to an entrance |
| **P4** | **Parcelling rework**: program mixes, variable lot sizing, parcel merging | Block program mix satisfied; merged parcels are simple polygons |
| **P5** | **Material sets** + palette library; ADR-0040 Pass B (tier re-cladding, crown kit) | Palette coherence per district; height→structure→cladding rule holds |
| **P6** | **Instancing** (ADR-0041 Phase 2) — props as `InstanceGroup`s | Instance counts; per-instance culling |
| **P7** | **Interactivity**: `Interactable`, interact action, sensors, hinge constraint, gates (Tier A then B), seats | Pure logic headless; Jolt bridge device-verified |
| **P8** | **Lighting**: fixtures + emissive, unified `LightBudget` | Budget selection determinism; no starvation of street lamps |
| **P9** | **Recipe library** — the long tail: townhouse, duplex, mixed-use, cafe frontage, church, cathedral, town hall, office park, big-box, strip mall, factory, flatiron, twin-tower, curved slab, stacked | Each recipe reachable, coherent, deterministic |

P0 is the gate. **Nothing else should start before it is solid**, because every
later phase encodes assumptions about what the kernel can express.

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

## 15. The Lot Lab — the review loop for all of this

`tools/lot_lab.cpp` draws site plans and floorplans top-down as SVG. It builds
them out of the vocabulary above — `Region` with holes, the exact rectilinear
boolean, the sampled-field path, per-edge setbacks, the plan grammar, the
subtractive zone allocation, prop dart-throwing, gates placed where a path
crosses a boundary — and writes nine sheets: `lots.svg`, `plans.svg`,
`shapes.svg`, `curves.svg`, `compose.svg`, `blueprint.svg`, `stack.svg`,
`recipes.svg`, `corner.svg`.

```sh
c++ -std=c++17 -O1 tools/lot_lab.cpp \
    src/engine/procgen/city/polygon.cpp -o /tmp/lot_lab
OUT=/tmp /tmp/lot_lab 7          # seed 7
```

This matters more than it looks. The subsystem's core bottleneck (§13) is that
verification needs a Metal build, so anything visual gets written blind. But
**the entire 2D layer can be judged on paper** — and that is where the design
risk actually lives. The lab is the P0 acceptance harness: if a floorplan or a
site plan is wrong, you see it in a browser in seconds, with no GPU involved.

Findings from the first pass, already folded back into this document:

* A plan is legitimately **several disjoint masses** (an office park's two
  blocks). Returning only the largest region silently drops the rest — the
  kernel's plan-finish must return them all.
* `insetEdges` needs the same **miter limit** `offsetPlan` already documents:
  without it the chord joints of a rounded ring fly off and the tier outline
  grows spikes. The existing code's scar tissue is load-bearing.
* Rounding by **quadratic bezier is not a true arc** — the r=8 "circle" in
  `shapes.svg` is visibly not circular. This is the concrete argument for arc
  (bulge) edges as a first-class kernel type rather than eager tessellation.
  **Now implemented** (`Arc2`, `curves.svg`): a loop carries one bulge per edge
  in the DXF convention (`bulge = tan(sweep/4)`), so a wall is either a line or
  an exact circular arc, and tessellation happens once at the end at a chosen
  chord tolerance. Straight walls stay crisp; curves are exact. Eight mixed
  plans — bay front, stadium, apsidal hall, crescent, moderne slab, bowed
  terrace, flatiron prow, rotunda-and-wings — are one loop each.
* Ops must push off the **current mass**, not the envelope; referencing the
  envelope makes every outset a no-op once the clip runs.
* **A representation can hide a bug from the very view meant to check it.**
  The arc centre was computed on the wrong side of the chord, and it stayed
  invisible for a full round because the SVG writer emits `A` commands — the
  renderer re-derives the centre from radius plus flags, so every curve drew
  correctly. Only `tessellate`, which uses the centre, exposed it. The lesson
  for the real kernel: **the acceptance test must consume the same numbers the
  consumers do.** A drawing that bypasses a field cannot validate it — so P0's
  test suite should assert on tessellated points and on areas, not on whether
  a picture looks right.
* Corner filleting must run on the **orthogonal ring**, before rounding, and
  each tier must be rounded on its own radius. Insetting an already-rounded
  ring feeds chord joints to the miter limit and the tier outline crumples.

### 15.1 The composition test (`compose.svg`)

The question the whole design rests on: do the pieces actually compose? The
sheet runs one pipeline end to end — a pentagon, a wing pushed off every wall,
a drum joined to each wing end, a smooth-min weld, and a court subtracted from
the middle. It works, and three things came out of it:

* **The keep-rule claim (§3.2) holds.** `polygonUnion`'s method generalizes to
  subtract and intersect exactly as predicted: split every edge at its
  crossings, classify sub-edges by midpoint containment, and change only
  *which* side you keep — plus reverse the clip's survivors for subtract so the
  hole winds the other way. ~90 lines, and it handles the pentagon's 72° walls
  that the rectilinear tracer cannot touch. **The exact path does not need to
  be axis-aligned**, which retires the biggest open risk in P0.
* **Marching-squares winding is not a hole flag.** The cell walk emits loops in
  whatever direction it happened to produce, so a field result rendered as an
  unfilled hole while the identical exact result rendered solid. Classify by
  **nesting depth** instead (even = outer, odd = hole) and force the winding.
  This is a hard interop requirement: the two kernel paths must be
  interchangeable downstream, so the field path has to normalize before
  handing anything on.
* **The weld is not area-preserving.** Smooth-min *adds* material in every
  joint — 793 m² of hard union became 825 m² welded at k = 2.6, about +4%.
  Coverage and FAR must therefore be measured on the **finished** plan, not on
  the union of the pieces that made it, or a program that asks for 26% will
  quietly deliver more. The site layer should compute coverage after massing,
  not before.

### 15.2 The blueprint as the authority (`blueprint.svg`)

The move that makes the 2-D layer load-bearing rather than decorative. Today
`emitFacadeRect` computes its bays privately, in 3-D, per wall rectangle — so
nothing outside it can know where a window is, and nothing can be reviewed
before geometry exists. Promote that decision onto the plan:

> **An opening is a 1-D SPAN ALONG A WALL — the plan's business — plus a
> SILL/HEAD height, the storey's business.** Those two facts together fully
> determine the 3-D opening.

```cpp
struct Opening {
    std::size_t edge;          // which wall
    Real  s0, s1;              // span along it, metres from the wall's start
    bool  door;  int swing;    // plan symbol: leaf + swing arc
    Real  sill, head;          // the storey's half of the fact
};
struct WallProgram { WallRole role; BayGrid grid; std::vector<Opening> openings; };
```

Consequences, all demonstrated in `blueprint.svg`:

* **Walls get real thickness.** The drawn wall is `outer − inset(outer, t)`,
  and every opening is punched through the poché by boolean. Three subtracts,
  no special cases — and it exercises the same kernel everything else uses.
* **`WallRole` makes a party wall blank by construction**, not by a downstream
  check that happens to skip it. The rowhouse unit carries openings on two
  walls and literally cannot carry them on the other two.
* **The constant-module rule survives the corner.** The L-plan runs one
  program round six walls; a wide wall and a narrow one show the *same* window
  with the piers absorbing the slack, which is what ADR-0040 already asks for
  and what the current per-rectangle computation can only approximate.
* **The bay grid becomes shared** — the prerequisite §6.1 already called for.
  Balconies, pilasters, storefront mullions and (later) interior partitions can
  all register against it, because it exists as data before geometry does.

### 15.3 Blueprints going up (`stack.svg`)

**Tiers.** A `MassStack` is a list of tiers; each tier holds a *list* of plans,
not one. Two rules do all the work:

1. **Every tier is CLIPPED to the tier below** (`intersect`). Containment stops
   being a check that can fail and becomes a property of the construction —
   the same guarantee the plan-in-envelope clip gives one level down.
2. **A tier may hold several plans.** That is the whole of "towers on a
   podium": tier 0 is the podium, tier 1 is three separate footprints each
   clipped to it, and they need not survive to the same height. Different
   towers, one base, no new concept.

**Terraces fall out.** The exposed roof at any setback is `plan[i] −
plan[i+1]` — a boolean, not a special case. Balconies, railings and roof
furniture attach to that region without anything having to detect it.

**Lofting is field interpolation, not vertex interpolation.** This is the
finding worth keeping. A vertex lerp needs a correspondence between the two
plans and dies the moment they differ in vertex count — and dies completely if
the topology changes. Interpolating the two plans' *distance fields* and
contouring per level handles a square becoming a circle (different counts) and,
more importantly, **one slab splitting into two towers** (different topology),
with no special case at all. Both halves already exist in the kernel.

Three parameters make it architecture rather than a morph:

* **A profile curve** on the loft parameter, so `t = profile(h)` rather than
  `t = h`. Entasis, a fast taper near the crown, a slow one at the base. The
  engine's `Spline` (curve.h) is the natural carrier.
* **Twist**, applied to the *finished* storey plan, not to the sampling. (I
  rotated one input's sample frame first; every intermediate level then blends
  an unrotated field with a rotated one and the levels cross through each
  other. The plan is what turns.)
* **Level runs.** A straight shaft shares one plan across many storeys, so the
  stack should store runs, not a plan per floor — otherwise a 60-storey tower
  pays for 60 identical plans.

### 15.4 How 3-D would consume this

The point of all of the above is that the 3-D pass stops *deciding* and starts
*reading*. Nothing here needs new geometry code — it needs the existing code to
take its inputs from the blueprint:

| 3-D output | Comes from | Today |
|---|---|---|
| Wall panel | `planEdgeRect(plan[i], edge)` — already exists | same |
| Window / door | the storey's `Opening` list | `emitFacadeRect` recomputes bays |
| Floor slab | `triangulateWithHoles(plan[i])` | outer ring only |
| Terrace deck | `plan[i] − plan[i+1]` | `setbackEvery` uniform inset |
| Corner post | plan vertices with a real turn | same |
| Roof | the topmost plan | rect-ish plans only |

So the porting job is narrower than it looks: `emitFacadeRect` loses its bay
computation and gains an opening list; `emitPlanSlab` gains holes;
`growPlanBuilding`'s tier loop is replaced by a walk over storey plans. The
element emitters (portico, balcony, steeple, roof furniture) are untouched.

**The one real cost:** a lofted tower has a different plan on every storey, so
walls can no longer be shared up a run — which is exactly why level runs matter,
and why lofting should be opt-in per recipe rather than the default.

**Still unproven.** Arc-aware booleans. Every curved plan here is authored as
one loop or tessellated before it meets the boolean, so the arc kernel and the
boolean kernel do not yet talk to each other. A lofted *arc* plan is therefore
tessellated too. That seam is the last open question in L0.

### 15.5 Where the data/code line sits (`recipes.svg`)

The rule:

> **Code owns verbs. Data owns nouns and numbers.**

Code knows *how* to punch an opening, split a wall on a module, sweep a
cornice, loft a plan. Data says *which*, *where*, and *how big*. The test for
the line being in the right place: **if adding an architectural style requires
a C++ change, the line is wrong.**

| Layer | Code (C++, hot, tested) | Data (Lua, hot-reloadable) |
|---|---|---|
| L0 kernel | every geometric op | — (knows nothing about buildings) |
| L1 grammars | the plan grammar's ops, the wall splitter, the element emitters, the loft | which ops, in what order, with what numbers |
| L2 recipes | — | **the building designs live here** |
| L3 site | the zone allocator, the prop packer | lot programs, coverage, setbacks, palettes |

**Fenestration is the worked example.** There is a small **closed set of
strategies** in code — `punched`, `plate`, `ribbon`, `curtain`, `clerestory`,
`storefront`, `blank` — each of which knows how to glaze a wall run. Everything
else is a table:

```lua
-- assets/recipes/buildings/modern_house.lua
return {
  fenestration = {
    { on = "edge:street", strategy = "plate",  margin = 1.4, sill = 0.45, head = 2.7, door = true },
    { on = "edge:side",   strategy = "plate",  margin = 2.2, sill = 0.45, head = 2.7 },
    { on = "edge:party",  strategy = "blank" },
    corner_glazing = { wrap = 2.4, sill = 0.45, head = 2.7 },
  },
  massing = "L", palette = "warm_concrete",
}
```

`recipes.svg` is the proof: five visibly different buildings — Victorian
terrace, modern house, glass tower, factory, corner shop — from **one plan and
one code path**, differing only in that table. Adding Edwardian, brutalist or
art deco is a new file. Adding *oriel windows* is one new strategy in C++, and
then every oriel variant is data again.

Three design consequences came out of building it:

* **Fenestration must be a PLAN-level pass, not a per-wall function.** A corner
  window spans two walls and deletes the post between them — a per-wall
  function structurally cannot express it. So corners are claimed first, then
  each wall glazes the run it has left. This is the concrete reason the
  original §6.1 "bay grid per wall" is not sufficient on its own.
* **The door is orthogonal to the glazing strategy.** A plate-glass wall still
  needs an entrance. Placing the door first and letting it *split the run*
  means every strategy — present and future — gets a door without knowing
  doors exist.
* **A wall's role is data, not geometry.** `party` is why the terrace's flank
  is blank, and the glass tower's recipe overrides it to `curtain` because a
  tower has no party walls. The role is asserted by the plan, honoured by the
  table, and never inferred by the mesher.

**What this means for the existing 40 architect recipes.** They are C++
functions today (`recipeGlassTower`, `recipeBungalow`, …) that imperatively set
~45 bools. They become tables of the shape above. Worth refining my earlier
sequencing advice: keep them in **C++ through P2 for speed, but write them
data-shaped from day one** — a list of `{selector, strategy, numbers}`, not a
function body — so the move to Lua at P3 is mechanical rather than a rewrite.

**The discipline this needs.** Data-driven means a bad table must not corrupt a
city: recipes get schema-validated at load with a named fallback, and overrides
stay pure (no RNG of their own) so determinism holds. The current `styleHook`
bug — the style book applied *after* `capFloors`, so a data override can exceed
the slenderness cap — is exactly the failure mode of skipping this.

### 15.6 Context facts: corners, heights, lofts (`corner.svg`)

**Where does "is this a corner?" come from?** Not from the building, and not
derivable from its plan. It is a fact about **the parcel's place in the block** —
what lies on the far side of each of its edges. The parceller just cut the
block, so it is the only pass that knows, and it tags each edge as it emits it:

| Tag | Meaning | Set when |
|---|---|---|
| `street` | faces a carriageway | the edge lies on the block boundary |
| `rear` | faces the block core | the edge lies on the interior ring |
| `party` | shares a wall with a neighbour | the edge is shared with another parcel |
| `court` | faces an interior court | the edge bounds a block court |

Then the classification is exact and needs no heuristics:

* **corner** — two *adjacent* street edges
* **through** — two *opposite* street edges (street front, lane behind)
* **mid-block** — one
* **island** — all of them

**The diagonal corner entrance** then falls out of three layers each doing one
job: the plan grammar's `chamfer(vertex, d)` *makes* the edge; the parceller's
tag makes it **addressable**; and the fenestration table puts the door on it —
`{ on = "edge:corner_chamfer", strategy = "storefront", door = true }`. No
special case anywhere, and the same context can be answered differently by a
different recipe (panel 4 puts wrapped corner *glazing* there instead). **The
context is a fact; the response is a recipe choice.**

**Where do floor heights come from?** A building is a list of **storey bands**,
and height belongs to the band:

```lua
stack = {
  { role="retail",  storeys=1,  height=5.4, plan="podium" },
  { role="lobby",   storeys=1,  height=4.2, plan="podium" },
  { role="parking", storeys=3,  height=3.0, plan="podium" },
  { role="office",  storeys=9,  height=3.9, plan="shaft",
                    loft_to="shaft_top", twist=0.30 },
  { role="plant",   storeys=1,  height=4.6, plan="shaft"  },
  { role="crown",   storeys=2,  height=5.0, plan="crown"  },
}
```

That one tower carries five different floor-to-floor heights. `floorHeight` +
`groundHeight` can express exactly two of them — which is why every generated
tower today has one lobby height and one repeat height, forever.

**Where does the loft structure come from?** The same place: it is a *band
property*. `loft_to` names the plan to morph toward across the band, `twist` is
the total rotation over it, and a profile curve can bend the parameter. Nine
office floors resolve to nine storey plans. Bands without `loft_to` stay
prismatic and share one plan — which is the level-run optimisation, expressed
as data rather than as a special case.

### 15.7 Does this retire the 45 booleans?

Yes — and it is worth being precise, because this is the structural claim.
Every field of today's `BuildingParams` lands in one of five tables:

| Table | Absorbs (from `BuildingParams`) |
|---|---|
| **Stack** | `floors`, `floorHeight`, `groundHeight`, `groundRetail`, `setbackEvery`, `setbackFloors` |
| **Fenestration** | `bayWidth`, `windowInset`, `curtainWall`, `solidFacade`, `retailStreetOnly`, `groundBays`, `sideBays`, and all seven `window.*` fields |
| **Elements** | `baseCourse`, `stringCourse`, `pilasters`, `awning`, `quoins`, `portico`, `entranceSteps`, `dome`, `balconies`, `porch`, `chimney`, `spire`, `steeple`, `parkingDecks` |
| **Palette** | `wallColor`, `trimColor`, `wallPart`, `window.frameColor` |
| **Roof rule** | `roofStyle`, `roofPitch` (a rule on the top band) |

Three fields don't move into a table — they **disappear**: `shape`, `tiers` and
`sides`. "Cylinder" and "pagoda" stop being enum values the mesher switches on
and become *plans and stacks*, which is why the `BoxMass` workaround (a recipe
deliberately failing plan massing so a shape enum can dispatch) has nothing left
to work around.

One field is **replaced by something strictly better**: `faceDir` — a single
direction the lot pass computes and pushes into the params — becomes the
parcel's per-edge tags, which distinguish street from side from rear from party
rather than collapsing all four into one vector.

Four genuinely stay as scalars: `wallThickness`, `parapet`, `walkableGround`,
`seed`.

**What "more complete" actually means:** the number of tables does not grow when
you add architecture — only the number of *entries* does. Today every new
architectural idea costs a struct field, a branch in two grow functions, and a
test; the marginal cost of variety rises forever. After this, a new style costs
a file.

**The honest bound.** The *strategies* (`punched`, `plate`, `curtain`, …) and
the *element kinds* (portico, balcony, cornice, …) remain closed sets in C++.
That is deliberate — they are the verbs — but it means the system is only as
expressive as that vocabulary. Choosing it well is the real design work, and it
is why the recipe-porting exercise (reinterpreting the existing ~40 architect
recipes as tables) should happen *before* the vocabulary is frozen: anything the
40 cannot express is a missing verb, and that is exactly what we want to find
out early.

## 16. Relationship to existing docs

* **Supersedes** `building-floorplan-plan.md` (its `FloorPlan`/`PlanEdge` design
  is absorbed and extended into `Shape2` with holes, arcs and edge tags).
* **Absorbs** the unbuilt half of `building-grammar-plan.md` — P3 lot-driven
  composition (now §4), P4 curtain panels (now an element kind), P5 architect
  (now the program layer).
* **Completes** ADR-0040 Pass B via §5 tier re-cladding and §7 material sets.
* **Depends on** ADR-0041 Phase 2 for props and interactivity.
* **Does not re-open** ADR-0038 §4 (interiors stay Tier C) — but §5's mass stack
  is deliberately the seam that generator would fill.
