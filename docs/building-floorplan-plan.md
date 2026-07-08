# Polygonal Floorplan Buildings — Design Plan

Status: proposal (device feedback round, Living City / ADR-0066 era)
Owner: shape grammar (`src/engine/procgen/city/shape_grammar.*`) + lots
(`city_lots.*`)

## The ask

Today every grown building is a box (or cylinder): `scopeFromFootprint` takes
the lot polygon's **oriented bounding box** and the split grammar fills that
rectangle. Two consequences the device rounds keep surfacing:

* interesting-looking "compound" buildings only happen **by accident**, when
  two OBB masses overlap;
* an L-shaped or wedge-shaped lot either fails the fill-ratio gate (no
  building) or would overhang its lot if we let the OBB through.

The goal: buildings whose massing comes from a real **floorplan** — a
polygonal (optionally curved-edged) outline — extruded into an architectural
base and floors, with the same PBR walls, window openings, cornices and
balconies the box grammar already produces. Stacked plans (a tower rising off
a podium) should compose from the same primitive.

## Core idea: the floorplan becomes the scope

Introduce a `FloorPlan` as a first-class grammar input alongside `Scope`:

```
struct PlanEdge {
    Vec2 a, b;          // outline edge in plan space (CCW)
    Real bulge = 0;     // 0 = straight; ±sagitta/halfChord = circular arc
                        // (the DXF/polyline convention — one scalar per edge
                        //  gives every curved-wall case we care about)
    uint8_t style = 0;  // facade role: 0 street, 1 side, 2 rear, 3 court
};
struct FloorPlan {
    std::vector<PlanEdge> edges;   // one closed CCW loop (holes: phase 3)
    Real y0 = 0, height = 0;       // vertical extent of this plan's mass
};
```

Key design choice: **the facade grammar stays 1-D**. All the box grammar's
facade intelligence (bays, window insets, ground retail, pilasters, string
courses) operates per-wall on a `(width, height)` rectangle — it never cares
that the wall's neighbours meet it at 90°. So the extrusion step walks the
plan outline, and for each `PlanEdge` builds exactly the wall rectangle the
existing splitter already knows how to decorate:

1. `wallScope(edge)` → a `Scope` whose `axis[0]` is the edge direction,
   `axis[2]` the outward normal, `size = (edgeLen, floorsHeight, wallThick)`.
2. run the **existing** facade ops (bays/windows/door placement) inside that
   scope — zero new facade code, full PBR part output (Brick/Glass/Trim...).
3. arc edges (`bulge != 0`) tessellate into chord segments ~2.5 m long first;
   each chord gets a thin wall scope. Windows land on chords, which is how
   real curtain-wall curves are built anyway. The cylinder building already
   proves the look works.
4. roof/floor slabs: triangulate the plan polygon (`triangulatePolygon` from
   road_mesh is reusable) for the roof deck, parapet ring runs the outline,
   cornice/string courses become outline *offsets* (`inset()` from polygon.h)
   extruded as thin ledges — this is what makes a polygonal mass read as one
   designed building instead of a prism.

Corners need one rule: at each outline vertex, shorten both wall scopes by
half the wall thickness and drop a corner post (a quoin/pilaster part) —
hides the miter, and masonry corners get their trim for free.

## Where floorplans come from

Phase-ordered, each shippable alone:

* **P1 — lot-fitted plans.** In `city_lots.cpp`, stop OBB-boxing the site:
  simplify the inset lot polygon (drop sub-2 m edges), take it directly as
  the plan. The fill-ratio and slenderness gates move from "reject the lot"
  to "simplify the plan". L/T/U lots produce L/T/U buildings that fit
  exactly. Deterministic from the same lot rng.
* **P2 — authored massing vocabulary.** A tiny op set that *composes* plans
  before extrusion: `rectPlan`, `lPlan`, `uPlan`, `chamfer(corner, d)`,
  `roundCorner(corner, r)` (bulge edges), `courtyard()` (phase 3, needs
  holes). Zoning picks from the vocabulary: downtown = chamfered corners
  (classic flatiron on acute lots — which our skewed junctions now produce!),
  residential = L/U around a garden.
* **P3 — stacked plans (podium + tower).** `std::vector<FloorPlan>` per
  building, each plan constrained to sit inside the one below
  (`inset(below, setback)` ∩ plan). The existing `setbackEvery` already does
  this for boxes; generalizing it to plans gives wedding-cake towers, and
  the roof of each tier below the next plan becomes a **terrace** — which is
  where balcony/railing parts and roof props attach.

## Balconies & window openings

* Window openings: unchanged — the facade splitter already recesses window
  scopes; on plan walls it inherits that per wall rectangle.
* Balconies: a new facade op at the bay level — project the window scope
  outward `0.9 m`, emit slab (Concrete part) + railing (Trim/Metal part).
  Gate by style tag: street-facing edges of residential plans, floors ≥ 1.
  The `PlanEdge::style` tag exists precisely so the grammar can spend
  ornament on the street side and keep rear walls plain (real buildings do).

## What stays the same

* Part/material system: everything still lands in `PartId` buckets →
  `materialFor` → baked PBR surface maps. No renderer work.
* Determinism: plans derive from (lot polygon, seed) only.
* Tests: pure geometry — plan-in-lot containment, wall-scope lengths sum to
  perimeter, arc tessellation chord error, stacked-plan setback containment;
  all headless, same style as test_city_lots.

## Suggested order

1. `FloorPlan` + `extrudePlan()` producing walls/roof/slab parts (straight
   edges only), unit-tested; cylinder path rewritten on it as proof.
2. P1 lot-fitted plans behind a `LotParams` flag; device look.
3. Bulge edges + corner posts + cornice offsets.
4. P2 vocabulary + zoning hookup; P3 stacked plans; balconies.
