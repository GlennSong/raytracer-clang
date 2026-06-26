# Road network — execution plan (cities → highways, on real splines)

**Status:** Active plan · supersedes the earlier draft of this file. Captures the corrected mental
model and the next steps, with an honest diagnosis of why density/structure/drivability haven't
shown up yet.

## The corrected mental model

A **hotspot is a CITY, not a highway seed.** I had this backwards — growing arteries between hotspots
first. The right order is:

1. **Cities first.** Each hotspot is a city. A city grows *dense*: an organic skeleton that then
   develops **structure** (its enclosed areas subdivide into blocks → grid/radial emerge as the
   structured infill, not as a stamped pattern).
2. **Highways second.** A separate pass connects the finished cities across the terrain by routing
   the *flattest* path city→city→city, fit to a spline.

Underneath both, two invariants that have been missing:
- **Roads are spline curves you can see and edit** — persistent graph of nodes + tangents, not baked
  polylines. (Answers "where are the nodes, tangents, curves? why can't I interact?")
- **Roads are drivable** — perfectly flat across their width, sloping only on a gentle grade, with the
  **land cut/filled to meet them**. A car can't drive a bumpy road. (Answers "patchy / doesn't conform
  to inclines / not solid".)

## Honest diagnosis — why each complaint is true

| Complaint | Root cause |
|---|---|
| "Where are the nodes/tangents/curves? Can't interact." | Generated roads are baked sampled polylines → meshes. No persistent curve object. `EditableCurve` (Bézier + handles, ADR-0050) exists but only for hand-authored roads, never wired to generated ones. |
| "No density. You keep saying you'll do it." | Space colonization alone makes a **tree** (stubs), not blocks. Density + structure come from **subdividing the enclosed faces** — the step I kept deferring. So you've only ever seen the sparse skeleton. |
| "Patchy, not solid, doesn't conform to inclines." | Roads are leveled **per-chain, independently**, so junctions disagree and pieces fragment; and the meshes are thin draped ribbons, not solid bodies. The terrain-conform helps but the underlying grade isn't network-consistent. |
| "Broken up by terrain in odd ways." | Same fragmentation: each road segment makes its own decision about height, so the network can't read as one cohesive surface. |

## The plan

Sequenced to put the two things you most want to see — **density** and **visible/editable splines** —
first. Each phase names the visible deliverable (verified by screenshot AND by walking it in-world).

### Phase 0 — Make the spline real, visible, and editable  *(the substrate)*
- Promote the road network to a persistent **graph of spline edges**: each edge a Hermite/Bézier with
  **nodes + in/out tangents** (reuse `EditableCurve` math). The graph is the source of truth; meshes
  derive from it. This is the "graph as a view of curves" keystone (ADR-0048) that was never built.
- **Debug overlay**: draw node dots, tangent handles, and the curve, toggleable — so the splines are
  literally visible.
- Wire the existing **`PathEditTool`** (ADR-0050) so dragging a node/tangent regenerates the road.
- *Deliverable:* you see and grab the spline nodes/tangents on a generated road.

### Phase 1 — Cities: organic skeleton → structured blocks → DENSITY  *(your #1 frustration)*
- Hotspot = city center (flat site, `findFlatSites`).
- Per city, two stages:
  1. **Organic skeleton** — space colonization fills the city footprint with a connected organic web
     (the city's arterials/collectors).
  2. **Structured infill** — extract the skeleton's enclosed **faces** (`extractBlocks`) and
     **recursively subdivide** each into blocks with new streets (OBB longest-axis splits; the same
     machinery `parcel.cpp` uses for lots). A regular **grid emerges** where a face is rectangular; a
     **radial** near the center if the character field says so. Structure is an *outcome*, not a stamp.
- *Deliverable:* a recognizably dense city per hotspot — blocks, hierarchy (arterial → collector →
  local), real road density. This is the thing you haven't seen.

### Phase 2 — Drivable roads + terrain that conforms  *(flat, solid, cohesive)*
- **Network-consistent grade**: solve ONE height per graph node — seed from terrain, then relax
  (Laplacian smooth) under **per-edge grade limits** until every node has a height and every edge is
  within grade. The whole network now shares a coherent vertical profile → **junctions agree, nothing
  fragments.**
- **Cut/fill the land to that grade** (`conform`): the terrain moves to meet the road — cuts through
  rises, fills dips, with embankments — so the deck is **perfectly flat across its width** and slopes
  only on the solved gentle grade.
- **Solid road bodies**: real width + thickness + curbs + embankment, meshed continuously (no
  patches/holes), collider built in the same pass (per AGENTS "Playable Scenes").
- *Deliverable:* you can drive a city's roads — flat, connected, sitting in the earth.

### Phase 3 — Highways: connect the cities across the terrain
- After cities exist, choose inter-city connections (Delaunay/gravity between city centers).
- For each connection, route the **flattest path** with a least-cost search over a **buildability cost
  field** (slope + water + grade) — this is exactly `terrain.route`/`routeRoad` (A*), which finds the
  minimum-cost line: **straight where the land is flat, curving to follow contours / around steep
  ground, gentle climbs over passes.** (Space colonization is an option for the multi-city *topology*;
  least-cost is the right tool for the *flatness heuristic* you described.)
- **Fit a spline** through the routed points; cut/fill as in Phase 2; higher class (wider, lower
  max-grade, grade-separated where it crosses city roads → ramps).
- *Deliverable:* highways linking the cities, flat and drivable, draped/cut naturally through the hills.

### Phase 4 — Blocks → lots → buildings
- The Phase 1 blocks feed `parcel.cpp` → lots → shape-grammar buildings (mostly existing).

## Sequencing & what I'll do first
1. **Phase 1 density** (subdivision into blocks) and **Phase 0 spline overlay** — the two you keep
   asking for. I'll land a dense, structured city you can see the splines of.
2. **Phase 2** drivable/cohesive roads (network grade + cut/fill + solid bodies).
3. **Phase 3** highways, then **Phase 4** buildings.

## Non-negotiables (acceptance bar)
- Splines are visible and editable (nodes + tangents).
- Cities are dense and show structure (blocks), not stubs.
- Roads are flat and drivable; the land conforms to them; they read as one cohesive solid surface.
- Verified by walking/driving in-world, not just an overhead fly camera.
