# The unified road system — one design

Status: **shipped** (all six build steps done; see the per-step `[DONE]` notes below) ·
Supersedes the road-meshing decisions scattered across ADR-0044
(analytic trim + junction pad), ADR-0048 (SDF roadbed/union), ADR-0054 (deck/grade
separation). Those documented *prototypes*; this is the single system they converge into.

## Why this exists

The road code had become three disjoint meshers — analytic junction pads, the SDF roadbed,
and the bridge deck — each strong at one thing and wrong at the others, picked per case. The
result rendered "a bunch of stuff next to each other," not one network: highways *overlapped*
cities instead of connecting, joins were either sharp-but-notched (analytic) or welded-but-
blobby (SDF), markings were extra geometry, and ramps/decks were one-sided floating planes.

This doc defines **one** road system. Every feature is a part of it, not a sibling of it.

## The decision that unifies everything

The requirement for **textured markings** (double-yellow, dashed, crosswalk, …) forces a
**road-local UV** — `u` across the road (lane), `v` along it (distance). The SDF has no such
parameterization, so it can never texture lanes; a **parametric ribbon** (a width swept along
a spline) *is* a UV by construction. Parametric also wins on every other axis: it is **light**
(a few quads per segment, not a dense grid), it **conforms + smooths** (the centerline carries
the smoothed/grade-limited vertical profile), it has **real volume** (extrude the ribbon), and
it is trivially **multilane**. So:

> **The base is parametric polygonal ribbons over a spline graph. The SDF and the analytic
> junction pad are both retired and replaced by ONE join engine: in-house 2-D polygon
> offsetting + boolean union + corner fillets.** (Join-engine decision: option 1, in-house,
> no external dependency — respects the stdlib-only rule.)

## The architecture

```
                 spline path graph  (nodes + spline edges; width, class, layer, lanes)
                          │   ← the rules pass runs here (min radius, spoke→roundabout, no-acute-merge)
                          ▼
  per edge:  sweep the spline into a ribbon OUTLINE (offset ± half-width, fold-safe)
                          │
  per node:  UNION the incident ribbon outlines + FILLET the corners   ← the one join engine
                          │
            one welded 2-D road polygon (carriageway, with curb/sidewalk offset bands & holes)
                          │
        EXTRUDE to a closed solid, seated on the terrain-SMOOTHED vertical profile
                          │
        bake road-local UV  →  MARKINGS as a texture/shader (no stripe geometry)
                          ▼
                 one light, volumetric, welded, textured road mesh
```

### The single source of truth: a spline path graph
`RoadGraph` is the one model. Nodes carry position (+ elevation/layer); each edge is a
**spline** (Hermite/Bézier) with width, `RoadClass`, layer, lane count. The generators
(grid / radial / tensor) and the editable road both **produce this same graph**; the crossing
resolver connects graphs into one. There is no second representation.

### The one join engine (the core, ADR forthcoming)
A single in-house routine solves **every** join — curb returns, curved roads, grid
intersections, roundabout rings, ramp gores, "anything in between":
1. **Offset** a centerline polyline to its two side rails (± half-width), with **miter** joins
   on convex corners and **round** fillets where asked, and **fold-safe** handling when the
   curve is tighter than the width (no self-crossing).
2. **Boolean-union** the incident ribbon polygons at a node into one outline-with-holes.
3. **Fillet** the union's reflex corners to the class's curb radius (the rounded curb return).
This replaces `buildRoadMesh`'s pad **and** `unionRoadbed`'s SDF. Sharp where it should be,
rounded where a curb return wants it, welded always, exact (no grid blur), light.

### Volume + terrain
The welded 2-D outline is **extruded** into a closed solid (top surface, side walls with real
thickness, underside) — no one-sided planes. The centerline carries a **smoothed, grade-limited
vertical profile** (`roadProfile`): the road follows hills but irons out bumps, because a road
is smooth. Grade separation is the same profile lifted to clear what it crosses (clearance
solver), with piers as real solids.

### Markings as texture
The surface carries road-local UV. Markings — edge line, single/double yellow, dashed lane
divider, crosswalk, stop bar — are a **material/shader keyed on (u,v) and lane layout**, not
geometry. Lane count comes from width/class. All stripe geometry is removed.

### Rules
The constraint/design pass (DesignRules: min radius, max grade, min arm angle, max arms before
a roundabout, clearance, ramp grade — per `RoadClass`) runs on the graph **before** meshing, so
roads can't bend too tight, a spoke hub becomes a roundabout, and acute merges fuse. One rule
source (`road_rules.h`), already started.

### Connection, not overlap
The **crossing resolver** detects where edges cross and rewrites the crossing into a real,
*shared-node* interchange (at-grade intersection / roundabout / diamond / cloverleaf, by class
+ angle + level). A highway then **connects** to the city through shared nodes — one routable
network, not two meshes that happen to overlap.

## Build order (each step testable, each retires a prototype)

1. **[DONE] One spline graph + rules.** `RoadGraph`, `DesignRules` (`road_rules.h`),
   constraint pass.
2. **[DONE] The offset/join engine.** `road_offset.{h,cpp}`: miter/fold-safe polyline offset,
   2-D polygon boolean union, hole bridging, corner fillet (`roundPolygonCorners`). Pure,
   unit-tested on cross / skew / overlapping squares / "#" grid with an open block. *The core.*
3. **[DONE] Volumetric extrude + vertical profile.** `weldSolid`: closed slab (deck + side
   walls + underside, block interiors are real shafts), seated on a smoothed grade-limited
   profile (`roadProfile`) sampled from the nearest spine.
4. **[DONE] Road-local UV + marking texture shader.** `weldSolid` bakes road-local UV
   (`emitTriUV`); the `RoadMarkings` surface paints double-yellow + dashed multilane dividers +
   white edges from it. Junction-spanning triangles stay plain. No stripe geometry.
5. **[DONE] Crossing resolver.** `road_crossings.{h,cpp}` / `city.resolve`: splice crossings
   into shared nodes (at-grade weld vs grade-separated flag); generators + ramps feed one graph.
6. **[DONE] Collapse the scenes.** Deleted the prototype demos (net_grid/radial/city/
   interchange, roadbed_network, hill_roads, net_weld, road_earthwork); shipped ONE
   `showcase` scene — welded multilane grid + roundabout + a highway connected to the city
   through on/off ramps, on smoothed terrain, with textured markings and real volume.

## Non-negotiables (the acceptance bar)
- One mesher. No "analytic vs SDF" choice survives.
- Joins: welded, no notches, rounded curb returns, exact (no grid blur).
- Real volume everywhere (no one-sided planes).
- Markings are texture, not geometry.
- Light geometry (parametric, not a dense grid).
- Highways connect to cities through shared graph nodes.
- Terrain-conforming and bump-smoothing.
- Multilane; spline-graph source of truth.

## Honest constraints
- In-house polygon boolean/offset is real work and the riskiest piece; if robustness proves
  intractable in reasonable time, the fallback decision (adopt Clipper2 as a dependency) comes
  back to the user.
- The viewer can't be run in this environment (no GPU); verification is the offline path tracer
  + headless unit tests, with the user doing in-engine walk/feel checks.
</content>
