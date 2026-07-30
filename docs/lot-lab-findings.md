# Lot Lab — findings

Companion to `lot-system-plan.md`. The plan is the design; this is the evidence.

`tools/lot_lab.cpp` draws site plans and floorplans top-down as SVG, built out
of the vocabulary the plan proposes. It matters more than it looks: this
subsystem's core bottleneck is that visual verification needs a Metal build, so
anything visual gets written blind — but **the entire 2D layer can be judged on
paper**, and that is where the design risk actually lives. The lab is the P0
acceptance harness.

```sh
c++ -std=c++17 -O2 tools/lot_lab.cpp \
    src/engine/procgen/city/polygon.cpp -o /tmp/lot_lab
OUT=/tmp /tmp/lot_lab 7          # seed 7
```

Eleven sheets: `lots`, `plans`, `shapes`, `curves`, `compose`, `blueprint`,
`stack`, `recipes`, `corner`, `silhouette`, `port`.

**The three findings that changed the plan**, if you only want those:
[where data ends and code begins](#where-the-datacode-line-sits-recipessvg),
[what replaces the 45 flags](#does-this-retire-the-45-booleans),
[why special buildings must be rationed](#rarity-is-a-quota-not-a-probability),
and [the audit of the existing 55 recipes](#porting-the-existing-recipes-portsvg).

Everything below is in the order we found it out; each section names the sheet
that prompted it.

---

## First pass — the kernel


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

## The composition test (`compose.svg`)

The question the whole design rests on: do the pieces actually compose? The
sheet runs one pipeline end to end — a pentagon, a wing pushed off every wall,
a drum joined to each wing end, a smooth-min weld, and a court subtracted from
the middle. It works, and three things came out of it:

* **The keep-rule claim (the plan's §3.2) holds.** `polygonUnion`'s method generalizes to
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

## The blueprint as the authority (`blueprint.svg`)

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
* **The bay grid becomes shared** — the prerequisite the plan's §6.1 already called for.
  Balconies, pilasters, storefront mullions and (later) interior partitions can
  all register against it, because it exists as data before geometry does.

## Blueprints going up (`stack.svg`)

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

## How 3-D would consume this

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

## Where the data/code line sits (`recipes.svg`)

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
  the plan's §6.1 "bay grid per wall" is not sufficient on its own.
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

## Context facts: corners, heights, lofts (`corner.svg`)

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

## Does this retire the 45 booleans?

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

## Profile, loft and twist are different knobs (`silhouette.svg`)

I conflated two things in "Blueprints going up" above, by treating the loft as
*the* way a tower stops
being a prism. There are four independent descriptions, and separating them is
what stops every interesting building from having to twist:

| Knob | Says | Example |
|---|---|---|
| `plan` | **what** the footprint is | a rounded polygon |
| `profile` | **how big** it is at height *t* — a scalar curve | the Gherkin |
| `loft_to` | **what it becomes** — morph toward another plan | the arch |
| `twist` | **how it rotates** | Turning Torso |

**A Gherkin is one plan with a profile curve and no loft at all** — the
footprint never changes shape, only size, swelling to a bulge and closing to a
nose. A pyramid is the same machinery with a straight line. Once profile exists
as its own knob, most "interesting" towers need no loft and no twist, which is
exactly the point.

**The arch is a loft that MERGES.** Two legs at grade become one span at the
top — the mirror image of the slab-splitting-into-towers case, and the field
loft handles it for the same reason: topology change is free. A **donut** is
simply a plan that carries a hole; `Shape2` already does, so nothing else
changes. A **pen-drawn irregular plan** takes corner cuts exactly as a rectangle
does, because `chamfer` is per-vertex and never cared about the shape.

## Porting the existing recipes (`port.svg`)

An audit of all **55 recipe functions** in `architect.cpp` (44 district
archetypes + 11 landmarks), read end to end.

### They are already a table pretending to be code

Almost every one has the identical shape:

```cpp
void recipeBrickShop(BuildingRecipe& out, Hash& rng, RecipeCtx&) {
    p.floors = rng.irange(3, 6);          // 1. a floor range
    p.groundRetail = true;                // 2. ground-floor mode
    dress(p, FacadeStyle::Brick, rng);    // 3. a cladding bundle
    if (p.floors <= 3 && rng.unit() < 0.3) {   // 4. 0-6 element flags
        p.roofStyle = RoofStyle::Gable;
        p.roofPitch = rng.range(0.35, 0.5);
    }
    out.placeType = "shop";               // 5. place type + name
    out.name = "brick_shop";
}
```

Five slots, every time. **The port is therefore mechanical**, and the uniformity
is itself the argument: 55 functions with one shape are data that has been
typed as code.

```lua
brick_shop = {
  place = "shop",
  stack = { { role="retail", storeys=1, height=4.5 },
            { role="office", storeys={3,6}, height=3.2 } },
  palette = "warm_brick",
  fenestration = {
    { on="edge:street", strategy="storefront", floor=0 },
    { on="edge:street", strategy="punched", bay=3.0, sill=0.9, head=2.4 },
    { on="edge:party",  strategy="blank" },
  },
  elements = { {kind="awning", on="element:door"}, {kind="cornice", on="tier:top"} },
  roof = { style="gable", pitch={0.35,0.5}, when="storeys<=3", chance=0.3 },
}
```

### How the 55 divide

| Group | Count | Port |
|---|---|---|
| Direct — floors, cladding, a few flags | **46** | mechanical; table only |
| Need **several masses per tier** | **5** | `office_park`, `strip_mall`, `church`, `market_hall`, `hospital` |
| Need **per-band floor heights** | **6** | `hotel`, `factory`, `warehouse`, `capitol`, `library`, `museum` (they already fight this with `floorHeight`/`groundHeight`) |
| Not buildings at all | **2** | `pocket_park`, `plaza` → lot programs, not building recipes |
| Collapse into another recipe + different numbers | **8** | see below |

**Nothing in the 55 needs a verb the vocabulary lacks.** That was the real
question — anything they could not express would have been a missing strategy or
element — and the answer is that the gaps are all *expressiveness of the
container*, not missing operations. That is the green light for P0's vocabulary.

### The eight that should collapse

`civic_hall` / `civic_midtown` differ by a floor range and a coin-flip on
cladding. `bungalow` / `craftsman` differ by `floors` and one bool. Also
`office_slab` / `office_midrise`, `oldtown_house` / `oldtown_grand`,
`brick_warehouse` / `factory`, `walkup_homes` / `apartments`. As tables these
are **one recipe with two number sets** — so the port *reduces* the count from 55
to about 47 while making each more capable.

### Six improvements the port buys (`port.svg`)

Left of each pair is what today's recipe can actually produce; right is the
ported table.

1. **`office_park`** — the clearest failure. The name promises a campus; the
   recipe emits one prism, because a plan cannot be two masses. Ported: two
   blocks, an atrium notch, a link bar.
2. **`church`** — sets `floors = 0` and one tall `groundHeight` to fake a nave.
   That hack exists because a church is not a floor stack. Ported: nave + two
   aisles + a tower base, four masses, and the steeple element lands on the
   tower rather than on the roof ridge.
3. **`strip_mall`** — a square box called a strip. Ported: a long bar of units,
   an end anchor, and a canopy run along the frontage.
4. **`hotel`** — carries an office's 3.2 m floors and cannot express a podium,
   so it is an office slab with a different name. Ported: a 4.6 m lobby band on
   a retail podium, then 3.1 m room floors. Same storey count, visibly
   different silhouette — the band section shows it.
5. **`loft_conversion`** — the sharpest case. Its defining feature is *big
   industrial glazing in old masonry*, and `dress()` makes that impossible:
   picking `Brick` also picks round-arched sash with muntins. Decoupling
   cladding from fenestration is what lets the recipe be what its name says.
6. **`craftsman` vs `bungalow`** — today one rectangle and one differing bool.
   Ported, the craftsman gets an L-plan with a rear ell and a porch wrapping
   two sides; the bungalow stays a simple gable. They stop being the same
   building.

### What the port cannot fix

Two limitations survive it, and both are honest costs rather than oversights:

* **`coreness` is copy-pasted skyline logic.** Six tower recipes each contain
  their own `cx.coreness * N` lift with a different constant. That is a *height
  model*, not a per-recipe property, and it belongs in the architect above the
  tables — otherwise every new tower recipe re-invents the skyline.
* **`RecipeCtx` couples recipes to lot size.** `cx.roomy`, `cx.shortSide` and
  `cx.area` are read inside recipes to decide floors and massing, which means a
  recipe knows about parcels. In the new model that inverts: the *program*
  (§8.1) reads the lot and picks a recipe that fits, so a recipe never needs to
  ask how big its site is.

## Rarity is a quota, not a probability

The concern is right, and it is the most important thing in this section: **if
twist is a per-building roll, every building twists and the skyline turns to
mush.** A 3% chance of a twisted tower across 400 lots is twelve twisted towers,
which is not a skyline, it is a texture.

The fix is structural, and the machinery already exists. `city_lots.cpp` PASS B
does not *roll* civic buildings — it **places** them: a quota table ("one
courthouse per city, one school per residential quarter") filled by the
best-scoring eligible lot, with a relaxation pass if nothing qualifies. Signature
massing should go through exactly that planner:

* A city gets **at most one or two signature towers**, placed on the best sites
  — tallest allowance, most central, most visible — never rolled per lot.
* `signature` becomes a **gate on the expensive knobs**: twist, merging lofts,
  unusual plans. An ordinary tower may use `profile` freely (that is just good
  massing) but may not twist.
* The quota scales with the city, not with the lot count: one per city, plus
  perhaps one per secondary hub — the same shape as the `Fire` station rule
  (`1 + total / 150`).

This also answers "how do we decide if a building has a twist": **nothing
decides per building.** The planner decides *for the city* which one or two sites
carry signature massing, and those sites get a signature recipe. Everything else
draws from tables that do not contain a twist knob at all.

The general principle, worth stating once: **anything that would stop being
special if it were common must be a quota.** Landmarks already are. Signature
massing, unusual materials, and landmark plan shapes should join them, and the
architect's weighted tables should carry only the ordinary vocabulary.
