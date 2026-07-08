# Road intersection weld & conform — an in-depth study

Device feedback flagged three things: (1) some intersections don't weld together
well, (2) terrain pokes over the road in places, and (3) a request to study
whether the analytical intersection approach is the right one, with a testing
arena and possibly graph-construction rules forbidding sharp corners.

This is the study. It is backed by two headless diagnostics
(`tools/diagnostics/`) that measure the real geometry — no eyeballing renders.

**Bottom line up front:** the analytical union is *robust* and is **not** the
problem, and acute angles are **not** the cause of the weld gaps. The gaps come
from one specific bug in the deck-surface fill (an over-eager coverage skip).
The poke-through is two separate, smaller issues, one already fixed. No new
angle constraints are needed for welding.

---

## How the intersection is actually built

Worth stating precisely, because it changes the diagnosis. A junction is **not**
one welded solid. It is assembled in two independent layers:

1. **The curb / sidewalk boundary** — each arm is offset into a ribbon polygon
   (`ribbonOutline`, mitered + clamped), a junction gets a **pad disc** of radius
   = *widest incident arm's half-width × 1.02*, and `polygonUnion` boolean-unions
   ribbons + pad into the outer boundary the curb and sidewalk band ride.
2. **The deck surface** — filled *separately* by per-arm **strips** (short quads
   along each centreline out to the mitered rail) plus, at junctions, the arms
   overlapping across the pad. Overlapping coplanar strips are the same asphalt,
   so they "read as one surface." A per-spine sub-mm height bias
   (`0.0005·(1+spine%6)`) keeps overlaps from z-fighting.

So "welding" (the boundary) and "coverage" (the deck surface) are two different
mechanisms. A hole in the *surface* is not a failure of the *union*.

---

## Diagnostic 1 — junction weld quality (`road_weld_probe`)

For every junction it forms the local ribbons+pad, unions them (the curb the
mesher draws), rasterizes a disc, and measures **GAP** = curb-enclosed ground the
actual deck mesh fails to cover. It also reports the min approach angle and the
`polygonUnion` outer-loop count (1 = clean; >1 = the boolean fragmented).

### The analytical union is robust

Synthetic star junctions (`road_weld_probe arena`), 3–5 arms, angles from 20° to
100°, **all weld cleanly**: gapFrac 0.000, a single union loop, every case. The
naive edge-split/boundary-chase `polygonUnion` handles acute multi-arm junctions
in isolation without fragmenting.

### The city sweep — where the gaps really are

`road_weld_probe city` over the living-city net (31 junctions):

```
# junctions=31  gappy(>2%)=2  union-fragmented=0  acute(<35deg)=1  total-gap=19.8 m^2
node  minAngle  gapFrac   gapArea  ...
34    89.1      0.102     16.40    (111,60) gap@(107,61) r=3.5..6.2
33    79.8      0.021      3.28    ( 96,66) gap@(106,62) r=10.7..11.3
...all others  0.000
# gap rate by min-approach-angle bucket:
   30-40 deg: n=2  gappy=0    40-60 deg: n=7  gappy=0
   60-90 deg: n=21 gappy=1    90-181:   n=1  gappy=0
```

Three facts that redirect the whole investigation:

- **Zero union fragmentation.** `polygonUnion` returns one clean loop at every
  real junction. The boolean is not failing.
- **Gaps do not correlate with acute angles.** The *single* acute junction (34°)
  is perfect; the *worst* gap is at a near-right-angle **89°**. The angle
  buckets show no trend.
- **The gaps cluster.** Nodes 33 and 34 are ~16 m apart and *both* report their
  gap at the same spot, ~(106,62) — a wedge **between** two junctions joined by a
  short arm. 29 of 31 junctions are gap-free.

### Ground truth (`road_weld_probe focus`)

The white-box dump of that region (deck triangles filled grey, centrelines
yellow, roadway-expected-but-bare cells red) shows a real crescent hole in the
carriageway between the two junctions — 262 bare cells ≈ 16.4 m². It is genuine
deck uncoverage, confirmed against an *independent* "expected paved" definition
(within a centreline's half-width, or within a pad), not an artifact of the
probe's local model.

### Root cause — proven

The deck-strip loop **skips** a quad when it is fully inside a *lower-index
spine's corridor* (`distToSpine < halfWidth`) or inside a junction pad, on the
assumption that surface covers it. But:

- The pad is only a **union-boundary** contributor — it is *never triangulated as
  deck surface*. (Filling pads as deck in an experiment did **not** close node
  34's hole — its pad, sized to a 7 m street, is only ~3.6 m and doesn't even
  reach the r=3.6–6.2 m gap.)
- The corridor test is a *centreline ± half-width* region, but a lower-index
  spine that **curves** (or whose own quads were themselves skipped) does not
  actually surface all of that region. A quad defers to asphalt that isn't there.

Disabling the skip entirely is decisive:

```
skip ON  : total-gap = 19.6 m^2,  node 34 = 260 bare cells
skip OFF : total-gap =  0.1 m^2,  node 34 =   0 bare cells
```

The skip — added to fight junction z-fighting — is the sole cause of the visible
weld gaps.

### Recommended fix (not yet applied)

Make coverage *sound* rather than corridor-guessed. Cheapest and lowest-risk:
**drop the corridor skip and keep the per-spine height bias**, which already
prevents z-fighting (overlaps become sub-mm-separated coplanar asphalt, exactly
as the original design comment intends). Cost: ~750 extra triangles citywide;
markings are already suppressed on overlapping junction quads. A heavier
alternative — triangulate the `polygonUnion` interior as the deck instead of
overlapping strips — is "more correct" but loses the per-road UV the lane
markings ride, so it is not recommended.

---

## Diagnostic 2 — terrain poke-through (`road_poke_probe`)

Grows the net on the living-city terrain, computes the conform, and compares
every deck vertex to the carved terrain — exact, and with a flatten dilation
approximating a coarse CDLOD tile.

```
# deck verts=20208
EXACT carved terrain : poke verts=44 (0.22%)  worst=0.55 m @ (136,108)
CDLOD (dilate=2.0)   : poke verts= 0 (0.00%)
poke rate by distance-to-junction:  <8m 0.04% | 8-20m 0.44% | >20m 0.08%
poke rate by cross-slope:  gentle 0.06% | moderate 0.22% | steep 0.30%
```

Two separate causes:

1. **Coarse-LOD undersampling** — a distant CDLOD tile whose grid step exceeds
   the road corridor could interpolate natural ground across the carriageway.
   The half-cell **flatten dilation** already added to `generateLodNodeMesh`
   removes it entirely (CDLOD poke → 0). Confirmed here.
2. **Residual conform error** — 0.22% of deck verts, worst 0.55 m, concentrated
   in the **8–20 m junction-approach zone on cross-slopes**, independent of LOD.
   The deck's smoothed, grade-limited, junction-reconciled profile and the carve
   diverge there, and the fixed −0.22 m carve depression is not enough on a
   side-slope near a reconciled junction. This is the poke still visible up close
   on the finest LOD.

### Recommended fix (not yet applied)

Deepen the carve adaptively where the deck sits on a cross-slope near a junction:
scale the depression by the local terrain gradient (e.g. −0.22 − k·slope), and/or
widen the full-strength flatten band a little further past the sidewalk before
its falloff starts. Small, local change to `roadNetConformRegions`.

---

## Diagnostic 3 — is analytical the right approach? do we need angle rules?

**Analytical is the right approach.** The union is robust across every angle and
arm-count the arena throws at it, and it fragments at *zero* real junctions. The
failures are in the *surface fill* and the *conform*, both ordinary bugs with
local fixes — not a fundamental limitation of computing intersections
analytically. A tessellation/CSG rewrite would be a large amount of work aimed at
a problem the evidence says we don't have.

**Angle constraints are not needed for welding.** The one acute junction in the
city welds perfectly; the worst gap is at 89°. The existing `pruneAcuteArms`
constraint (drop the shorter arm of any junction pair tighter than ~20°) has
already removed genuinely pathological corners, and it is worth keeping for
*realism* — real street grids rarely have sub-20° forks — but it does nothing for
the weld gaps and should not be tightened in the hope that it would.

If anything, the graph-construction rule the data *does* suggest is a **minimum
arm length between junctions**: every real gap was a short arm between two
junctions ~16 m apart. `mergeShortEdges` already folds crossings closer than
~14 m; nudging that threshold up (or ensuring a junction's pad can't overlap a
neighbour's) would remove the *configuration* that exposes the skip bug — but
fixing the skip bug is the real fix, and makes the length rule unnecessary.

---

## Summary

| Symptom | Real cause | Status | Fix |
|---|---|---|---|
| Intersections don't weld | Deck-strip **coverage skip** defers to unsurfaced corridors | **Proven** (19.6→0.1 m²) | Drop corridor skip, keep height bias |
| Terrain pokes through (distance) | Coarse-LOD undersampling | **Already fixed** | Flatten dilation in CDLOD mesher |
| Terrain pokes through (up close) | Conform deck/carve divergence on approach cross-slopes | Characterized (0.22%, ≤0.55 m) | Slope-adaptive carve depth |
| "Maybe analytical is wrong / add angle rules" | Neither — union is robust, angles uncorrelated | Ruled out | Keep analytical; keep (don't tighten) `pruneAcuteArms` |

The diagnostics that produced every number here live in `tools/diagnostics/` and
should be re-run after any change to the road mesh or conform to keep the city's
network honest.
