# City Generation — Playbook & Idea Backlog

Companion to `docs/city-generation-plan.md` (the built system) and ADR-0038. This
doc answers two questions: **how do procedural cities actually get built**, and
**which of the big ideas are possible now vs. need planning**. It exists because
the idea space exploded productively — this organizes it so we build deliberately.

---

## 1. How a city gets built procedurally (the playbook)

The canonical pipeline (CityEngine; Parish & Müller, *Procedural Modeling of
Cities*, 2001) is the one we already follow:

```
road network → blocks (graph faces) → parcels (lots) → buildings → details
        ▲                                                    │
        └──────────── fields: population, zoning, water ◀─────┘
```

Everything keys off **fields** (rasters over the map): population/density, zoning,
elevation, water/coast distance. The road network reads them; blocks fall out of
the network; parcels subdivide blocks; buildings fill parcels per zone.

### Road generation — the real approaches
This is the question "is there a whole approach to generating roads?" Yes — four,
in increasing sophistication:

1. **Template + deformation (what we have).** Lay a grid/radial template, jitter
   it, drop edges by density. Trivial, deterministic, connected. Good bootstrap;
   not organic.
2. **Extended L-system (Parish & Müller).** Grow roads as an L-system with *global
   goals* (head toward population peaks, follow the coast) and *local constraints*
   (snap to nearby intersections, avoid water, legalize angles). Produces
   highway→arterial→street hierarchies.
3. **Tensor fields (Chen et al., *Interactive Procedural Street Modeling*, 2008).**
   A tensor field encodes the *local street direction* everywhere; tracing
   hyperstreamlines along it yields coherent grids that **bend around terrain,
   rivers, and coastline**, with radial/grid/curved regions blended. This is the
   cleanest path to the "grid + radial parts intersecting to create interesting
   divisions" you described.
4. **Agent / growth simulation.** Agents extend roads toward unmet demand; the
   city accretes. Natural fit for "**simulate city growth over time**."

**Your "civil-engineering growth over time" idea** is a real, buildable hybrid:
seed near a resource (water/mountain), grow an **organic core** with agent/L-system
roads, then stamp **planned eras** (a grid district here, a radial one there) each
as its own tensor field; where eras meet, the street seams create the "interesting
divisions" — and each enclosed face is a block. Mathematically it's: a *demand/
population field* driving agent road growth + *per-era tensor fields* for planned
expansion + the existing face-extraction/parcel/building stages. No new math we
can't write; it's a multi-pass orchestration over approaches 2–4.

### The grading / civil layer (your "human-built, not draped" insight)
Real streets are **flat and engineered**, and the *terrain is cut/filled to meet
them* — not the reverse. The correct layering is:

```
road grades (engineered slopes along the network)
   → block pads graded flat to meet the streets
      → cut/fill the terrain to the pads (retaining walls, embankments)
         → steps/stairs where the grade break is too steep to ramp
```

We just started this: blocks are now flat graded pads with curb/retaining skirts
(`city.cpp`). The missing pieces are a **road-grade solver** (propagate consistent
grades along the graph so streets are flat across width and gently sloped along
length) and **steps** at steep grade breaks.

---

## 2. Idea backlog — what's possible now vs. needs planning

Every idea from the brainstorm, triaged by effort against the *current* C++
substrate (grammar + scope ops + scatter + grading + Lua).

### A. Possible now (small, build on what exists)

> **Done since this doc was written:** flat lane'd streets (road-grade solver +
> `emitFlatRoad` + painted lane lines), **steps** on tall curbs, **lamp posts**
> along verges, **zebra crosswalks** at intersections, tighter curbs (apron snaps
> to the carriageway). Remaining items below.

| Idea | Approach |
| --- | --- |
| ~~**Steps / stairwells** on steep inclines~~ ✅ | Done: a stair run descends mid-edge where a block's curb exceeds 0.7 m. |
| **Hydrants, signage, street plants** (lamp posts ✅) | Scatter prop meshes along road verges (like the street-tree/lamp pass) and onto facade attach points (ADR-0028 §2). Cheap kit-bashed meshes. |
| **Parking lots & garages** | A parking lot = a flat apron block variant with painted stalls + a few car boxes. A garage = a `solidFacade` building with open-deck floors (a parking archetype). Both are archetype/block-program additions. |
| **More architectural styles** (European/Eastern/Western) | Extend `FacadeStyle` + the archetype library with param presets (mansard roofs, sash-window rhythms, balconies, brackets). Pure parameters over the grammar. |
| **Designed parks** | A park block-program: paths (flat apron strips), tree clusters, benches/fountains (prop meshes), a pond (flat blue quad). Composition of existing primitives. |
| **Block "landscaping" phase** | A *block program* step: decide a block's content — building / plaza / park / parking / civic — by district + shape, before parcelling. A small policy layer over the current per-lot loop. |
| **Odd-block points of interest** | Detect non-rectangular blocks (OBB aspect/area, intersection wedges) and switch placement: a single hero building, a courtyard ring, or a radial fan instead of street-aligned rows. A parcel/placement branch. |
| ~~**Crosswalks / lane lines** (geometry)~~ ✅ | Done: yellow centre + white edge lane lines on every street; zebra crosswalks on intersection approaches. |

### B. Near-term (needs a design pass, then a focused build)

| Idea | What it needs |
| --- | --- |
| **Flat streets with lanes + consistent grades** | A **road-grade solver**: assign each intersection an elevation (relax toward terrain with max-slope limits), build flat crowned road cross-sections between them, with curbs to the block aprons. Extends the grading we started — the biggest "looks human-built" win. |
| **Tensor-field / agent road generation** | Replace the grid bootstrap with approach 3/4. Well-understood; a real module (`road_growth.*`) over a density field. Unlocks organic + radial + coastal layouts. |
| **Procedural textures** (brick, concrete, asphalt+markings, crosswalks) | A texture generator (à la `barkMaps`) + **world-scaled UVs** on facades/roads + both-renderer support (Metal viewer first; the offline tracer's albedo-texture path needs a check). The "make the textures real" thread. |
| **Architectural style library as data** | A Lua style schema (`styles/european.lua`, etc.) over `building.grow`, so styles are authored, not hard-coded — the Lua-authoring payoff. |
| **Signs & stoplights** | Prop meshes at intersections + signage attach points; placement is easy, *legible* sign content (text/symbols) wants the texture pipeline. |

### C. Needs significant planning (whole subsystems)

| Idea | Why it's big |
| --- | --- |
| **Water** — rivers/streams, lakes, **oceans with waves**, shorelines | A subsystem: carve rivers into terrain (SDF/erosion channels), a water surface (flat plane + animated shader; gentle rolling waves far, lapping at the shore near), a shoreline/coast field. Render is **Metal-side** (animated water shader). Ties to `world-system-plan.md` §5. **It also reframes roads** (cities seed near water) — so design water *before* the growth-sim roads. Enables **piers, bridges, coastal homes**. |
| **Bridges** | Need water/valley spans + road-over-gap geometry; gated on water + the road-grade solver. |
| **City growth simulation over time** | The ambitious era-by-era model (organic core → planned districts). Design *after* tensor/agent roads + water exist, since it orchestrates them. |
| **Building interiors** (rooms/hallways) | Deferred at Tier C (ADR-0038 §4) — a separate floor-plan generator. |

### D. Far future
Traffic/agent simulation, full LOD/HLOD + streaming for a huge city, day-night
crowds — all gated on the above and the spatial-partition work (ADR-0034).

---

## 3. Recommended sequencing

A coherent path that front-loads visible "human-built" wins and the one big
subsystem that reframes everything else:

1. ~~**Finish the street/grading layer**~~ ✅ Done: road-grade solver, flat
   lane'd streets, curbs, steps. The city reads as built.
2. **Street furniture + block programs** (A): lamp posts ✅ + crosswalks ✅ done;
   remaining — hydrants/signage, **parking**, **designed parks**, **odd-block
   POIs**, and a **block-program** step (decide building/plaza/park/parking by
   district + shape). Lots of cheap visible variety; the natural next slice.
3. **Procedural textures** (B): asphalt+markings, brick/concrete facades — the
   jump from "flat-shaded massing" to "surfaced city" (viewer-first).
4. **Water** (C): the big subsystem — rivers, coast, ocean waves, piers/bridges.
   Plan it as its own ADR; it reframes road generation.
5. **Tensor-field / agent roads + growth simulation** (B→C): once water exists to
   seed against, replace the grid with organic+planned growth over eras.
6. **Style library in Lua + more archetypes** (B): broaden Eastern/Western/
   European variety as authored data.

Items 1–2 are buildable now in the C++ substrate; 3–4 each warrant their own ADR;
5–6 are the long game. Pick per appetite — but **one slice at a time** (the
lesson from this sprint).
