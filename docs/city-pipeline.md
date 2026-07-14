# The city build pipeline (v2)

Canonical ordering for procedural city generation, decided on-device 2026-07-13.
This **supersedes** the ad-hoc ordering scattered through metropolis-scale-plan
§10.6 / freeway-rules §12: those built streets first and bolted the freeway on
afterward, which is the root cause of the freeway crossing bad junctions, ramps
hunting for landings, and cut streets orphaning blocks. v2 inverts that.

## Two load-bearing principles

1. **Each stage's output is the next stage's fixed input. Never reach backward.**
   Terrain constrains hubs; hubs place the freeway; the freeway's ramp landings
   seed the streets; the streets define the blocks; districts + blocks define
   the lots; lots get realized; streets get decorated. The freeway is the
   *first cause the city organizes around*, not a reactive intruder.

2. **The pipeline is scale-independent.** It runs identically at 2.6 km or
   8 km; map size is a knob turned LAST, after the machine is proven at the
   current scale. A "small town downstream" is just this same build seeded at a
   far hotspot with a small budget.

Two design invariants that hold the whole thing together:
- **Districts are intent declared early and refined late** (coarse before
  streets so they steer street *style*; snapped to real streets after, so they
  steer building *type*).
- **Frontage is the law of parceling** — every lot faces a street; leftover
  block interior becomes a reachable court (plaza/park + alley), never a
  landlocked building.

## The order

**1. Terrain.** Heightfield + erosion + coastline/mountains. Derive the
buildability mask (slope / water / beach margin). Nothing builds where it says
no. The one stage with no upstream dependency.

**2. Regional hubs.** A handful of major anchor points on buildable land — the
tentpoles of the metropolis. They exist only to organize the freeway and the
districts. Few, deliberate, spread out. (At large map scale, distant hubs
become *satellite towns* the freeway connects to.)

**3. Freeway placement.** Route ONE spine near (not through) the regional hubs.
Off the beach, straight and smooth, network rules honored (no self-cross, bridge
where corridors must meet). Output: an *alignment* — centerline + vertical
profile, elevated through the dense middle, grounding at the map edges.
Confirmed policy: **one freeway is enough** for a city this size (R1.4 top-1).

**4. Corridor synthesis.** The alignment becomes real freeway geometry: deck,
median, structure (girders/bents/piers), elevated sections. Mainline only — no
ramps yet.

**5. Exits and entrances.** Stamp interchanges (feasibility-driven: only where
reachable). Each exit + on-ramp peels off the deck, clothoids down, ends at a
**ramp landing point** on the ground. These landings are the crucial output —
the fixed doors between freeway and streets. Everything after grows *toward*
them.

**6. City sizing + local hotspots.** Decide how much city to build, then scatter
a finer layer of neighborhood-seed hotspots near the corridor and hubs. Where
step 2 was tentpoles, these are individual neighborhoods.

**6b. Coarse zoning.** Give each local hotspot a district kind (downtown,
financial, residential, suburb, office park, industrial, main-street, beach
front) and paint soft regions. Happens BEFORE streets, because the district
decides how its streets look.

**7. Street growth — TWO-TIER (skeleton + template fill).** The single largest
piece of the rebuild, and the fix for malformed blocks at the root. Colonization
today is the *primary* generator, which is why blocks are garbage: it makes
irregular self-intersecting faces that can't parcel. Demote it:
- **Skeleton (colonization, sparse):** very few seeds, high influence,
  aggressive merge — lays a handful of smooth non-axis-aligned *arterials*
  giving the city its organic character line, anchored to the ramp landings
  (each landing's first road is a deliberate arterial spine, not colonization
  noise). This is what colonization is genuinely good at.
- **Fabric (template fill, NOT colonization):** arterials + freeway + coast
  bound a set of regions; each region is *filled* with a district-appropriate
  pattern template — rotated grid (downtown), loops + cul-de-sacs (suburb),
  sparse big-parcel loops (industrial), shoreline strip (beach front), radials
  (old core). Templates produce clean regular blocks *by construction*.

  Grids/radials/cul-de-sacs stop being hoped-for outcomes of colonization and
  become the fill primitives themselves.

**8. Block demarcation.** Read the finished street net as areas:
- **Enclosed blocks** — faces fully ringed by streets (what we extract today).
- **Ribbon frontage** — strips along OPEN roads not fully enclosed (main-street
  strip, edge-of-town). New: today only closed faces are found.
Inset from the road edges for sidewalk + setback.

**9. District refinement.** Snap the coarse 6b zones to real street boundaries
(a district ends at an avenue, not mid-block). This refined map drives building
selection. (Coarse @ 6b → street style; refined @ 9 → building type.)

**10. Lot subdivision (frontage-first).** Cut blocks into lots working INWARD
from the street frontage, not by area. Split the perimeter into frontage runs,
pull each lot to a district-typical depth. Every lot touches a street. Leftover
interior becomes a shared **court** (plaza / pocket park) with an alley or
walking path connecting out. Kills the current "crammed, inaccessible interior
lots" defect. Acceptance rig: the plan-only demarcation view — every lot must
front a street, or the interior is an explicitly reachable court.

**11. Lot realization (the site).** Treat each lot as a designed site chosen by
district: ground surface (concrete / exposed terrain / lawn), building envelope
+ placement, parking, driveway, yard, walkways, landscaping. Unifies today's
scattered specials (yards, plazas, park planting) into "how a lot of type X is
composed." Suburb = house set back + yard + drive; office park = low slab +
parking apron + hedges; downtown = tower + plinth to the sidewalk line. Building
geometry (shape grammar) plugs in here; skippable for the demarcation view.

**12. Street decoration.** Last, because it needs settled geometry: signals at
junctions, lamp posts along sidewalks, and finer sidewalk furniture — bike
racks, newspaper boxes, planted street trees, shrubs, painted curbs, benches.

## Build sequence (each stage verified in the plan-only sandbox first)

1. **Pipeline inversion** — corridor first; ramp landings as growth seeds with
   arterial spines. Street growth stubbed to **arterials only** so the skeleton
   is visible before the fabric exists. ← STAGE 1, in progress.
2. **Frontage parceling + courts/alleys** (step 10) — immediately visible in the
   demarcation view; fixes crammed lots.
3. **District-driven street templates** (step 7 fabric) — grids/loops/radials
   per zone.
4. **Ribbon lots along open roads** (step 8 second clause).
5. **LotPlan site design** (step 11), then **street decoration** (step 12).

## Map scale (deferred — the LAST knob)

Current: ~2.6 km coastal square. Target vision: ~8×4 km coastal strip with the
metropolis at one end and satellite towns strung along the freeway (gives the
freeway a *reason*: it connects places). Deferred until the inverted pipeline is
clean at current scale. Costs to remember when we do scale:
- **Terrain resolution must scale with size** or the ground goes blobby
  (~2.9 m/sample at 2.6 km / res 900; holding that at 8 km needs ~res 2700 →
  erosion cost grows ~with res²). Map size is mostly a *terrain* budget.
- **Rectangular domain support** — terrain gen + CDLOD assume a square. A
  coastal strip wants the long shoreline; either add rect-domain support (right
  answer) or use an 8 km square with the coast offset (cheap, wasteful).
