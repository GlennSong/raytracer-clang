# Building Grammar v2 — Architecture, Floorplans, and the Building Lab

Status: agreed plan (device-feedback interview, Living City / ADR-0066 era).
Supersedes and absorbs `building-floorplan-plan.md`.

## Decisions from the interview

1. **Massing**: composed primitives (union/subtract rectangles + chamfer /
   fillet / curved edges), where the LOT SHAPE drives which composition is
   chosen and how it is fitted — then the result is clipped against the inset
   lot polygon. L / T / U / H, courtyard, flatiron, and curved plans are all
   in scope; skyscrapers get their own plan recipes (interesting tower
   footprints, not extruded rectangles).
2. **Style**: multiple vocabularies, COHERENT per district — a financial
   district never grows a cottage. First new vocabulary: European old town
   (stucco/stone, tall narrow windows, hood moulds, arcaded ground floors),
   alongside the existing American masonry + modern glass.
3. **First slice**: the window/door sub-grammar — designed as a GENERAL
   element system, not a window-only special case.
4. **Performance**: quality first. The lab may emit rich geometry; LOD /
   instancing is a later pass if the city gets heavy.
5. **Tooling**: a collaborative BUILDING LAB — a small scene whose building
   recipe is a Lua file, hot-reloaded on change, so recipe edits (Claude) and
   on-screen review (device) iterate in seconds without a rebuild.

## The element system (P2 — first grammar slice)

Openings become parametric ASSEMBLIES stamped by the facade splitter:

```
struct OpeningStyle {                 // one WINDOW or DOOR design
    Head  head;                       // Flat | Segmental | RoundArch
    Real  archRise;                   // segmental: rise/span
    Frame frame;                      // depth, faceWidth, material (PartId)
    int   lightsX = 1, lightsY = 1;   // partitioned panes (mullions/muntins)
    bool  transom = false;            // separate top light
    Sill  sill;                       // profile + oversail
    Hood  hood;                       // header band | hood mould | voussoir arch
};
```

The emitter cuts the REAL opening shape into the wall (arched heads are
polygons with tessellated arcs — the wall face becomes wall-with-hole), the
frame is geometry seated in the reveal, and the glass sits inside the frame
(device: "the window should have a frame around it and then in that frame
sits the actual window"). Voussoir arches emit under the wall's masonry part
with UVs oriented along the arc, so brick visibly turns with the arch.

Generalisation (the point of doing it this way): `OpeningStyle` is one case
of an **Element** — a parametric assembly stamped into a wall/plan scope.
Cornices, quoins, balconies, storefronts, porches, and roof furniture join
the same registry, so archetype tables compose *elements*, and new
architecture is mostly new tables, not new C++. Also in this slice:

* **Quoins** — alternating corner masonry up building corners (hides the
  thin-brick edge at arrises; device feedback).
* **A real base cornice** — a projecting profiled course at the base/shaft
  transition (the current 10 cm string course reads as nothing).

## Floorplans (P3)

* `FloorPlan` = closed loop of `PlanEdge{a, b, bulge, styleTag}` (bulge =
  arc sagitta, the DXF convention — curved walls and curved corners).
* **Extrusion reuses the 1-D facade splitter**: each plan edge (arcs
  tessellated to chords) becomes a wall rectangle the existing bay/window
  machinery decorates. Roof slabs triangulate the plan; cornice profiles are
  SWEPT around the plan outline; corner posts hide miters.
* **Lot-driven composition**: analyse the inset lot polygon (area, aspect,
  corner angles, street edge) → pick a plan template (bar, L, T, U, H,
  courtyard; flatiron on acute corners; chamfered/rounded corners downtown)
  → fit its primitives to the lot frame → CLIP against the lot polygon.
  Conformance is guaranteed by the clip; design intent by the template.
* **Base / shaft / capital**: a building is a STACK of plans, each
  constrained inside the one below (`inset` + containment), with a swept
  cornice at every transition and at the parapet. Setback roofs become
  terraces (balcony/railing elements attach there). Skyscraper recipes
  compose tower plans (cruciform, chamfered square, curved-corner slab,
  stepped wedding cake) over a fuller-footprint podium.

## Curtain wall as real panels (P4)

Kill the overlay mullions: the facade becomes a mullion/transom lattice of
actual profiles; each cell is FILLED with a vision-glass pane or an opaque
spandrel panel, inset behind the lattice — every panel is a pane of metal or
glass (device feedback). Same splitter, pointed at cells instead of holes.

## The architect pass (P5)

A deterministic planning stage (not an LLM) between districts and lots:

```
district tag ("financial", "midtown", "residential", "oldtown", "industrial")
    → ArchetypeTable (weighted recipes: massing templates × vocabularies
      × element styles × materials)
    → per-lot pick (seeded), constrained by lot analysis
```

This replaces `typeFor`/`paramsFor` in city_lots and is where coherence
lives: the financial district's table simply contains no cottages. District
tags come from the existing radial zoning first; real district polygons
later. Suburbs get house/duplex/low-rise tables with their own massing
(pitched roofs are new element work, flagged as its own follow-up).

## The building lab (P1 — the tool, shipped with this plan)

* `assets/levels/building_lab.json` — a pedestal scene with ONE script
  entity running `assets/scripts/building_lab.lua`, `"watch_scripts": true`.
* `building_lab.lua` — the recipe: a `LOT` preset table (rect / L / wedge /
  flatiron / curved-corner), an archetype switch, seed — grows the building
  plus a lot-outline pad. All grammar work happens against this file.
* **Hot reload**: in play mode the state watches the level's script files;
  when one changes it re-enters through the state machine (the same clean
  reset as the editor→play loop). Desktop viewer only (the web bundle bakes
  assets; no live editing there).
* Workflow: Claude edits the recipe / grammar tables → the viewer reloads on
  save → device review → notes → next edit. The same loop will drive
  L-system and material recipes.
* Editor integration (inspector-driven regenerate) is a later convenience;
  the lab does not block on it.

## Phases

| Phase | Contents | Where it lands |
|---|---|---|
| P0 | asphalt dial-back, curb-following sidewalk scoring, wider sidewalks, rim-block overlap fix | shipped with this doc |
| P1 | building lab (level + lua + hot reload) | shipped with this doc |
| P2 | element system: window/door sub-grammar (arched heads, frames, lights), quoins, real base cornice | lab |
| P3 | floorplans: plan extrusion, lot-driven composition, base/shaft/capital stacks, swept cornices | lab → city |
| P4 | curtain wall as real panels | lab → city |
| P5 | architect pass + European old-town vocabulary; suburbs (pitched roofs) | city |
