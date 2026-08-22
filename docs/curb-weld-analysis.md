# Curb welds — why the kerb returns fail

Status: measured 2026-08-21 on `metro_v2_test.json`, headless, with
`tools/diagnostics/curb_weld_probe.cpp`. Device report that started it:

> there are places where it simply fails and we have overlapping curbs or
> they're not properly dovetailed and rounded

Both halves are real and both are measurable. The cause is not the one it looks
like: the corner geometry is mostly *right*, and gets destroyed on the way to the
band by a 1 cm cleanup pass that manufactures the very artifacts it exists to
remove — which a mitre offset with no fold check then amplifies into metres.

## What the probe measures

`curb_weld_probe` grows a shipped level's road network exactly as the loader
does, meshes it with the real mesher, and then measures the band the mesher
**actually emitted** — not a reconstruction of what it should have emitted.
That distinction matters: the curb/sidewalk band is not a per-junction object,
it is swept along the closed boundary loops of the whole asphalt union
(`road_net.cpp`, roads-v2 S5), so a probe that rebuilt the loops itself would be
measuring its own copy of the algorithm. Instead the mesher hands out its own
working state through `CurbBandAudit` (same contract as the existing
`chainTriEndsOut` diagnostic out-param), and the probe reads the emitted
triangles back out of the `RenderMesh` by their band colours.

Three properties, each checked against the graph's junctions:

| Property | How it is measured |
|---|---|
| **Rounded** | Per kerb corner: total turn, the arc length it is spread over, the implied radius, and the worst single-vertex turn. A real return spreads ~90° over an arc; a mitre dumps it on one vertex. |
| **Single** | Band-top triangles rasterized at 0.25 m in a disc around each junction; cells covered more than once are folded band. Cross-checked against summed triangle area vs. covered area — two independent measures that agree. |
| **Continuous** | Walk the kerb line, sample outward, require band-top coverage except where a mouth gap deliberately suppresses it. |

Plus **band reach**: the farthest any band-top vertex sits from the kerb line it
belongs to. A correct band reaches exactly `sidewalkWidth`. This is the metric
that localizes the spikes.

## The measurement

216 junctions, 187 band loops, 35 115 band-top triangles:

```
  mitred (a corner where ONE vertex eats > 30 deg) : 216  (100.0%)
  kerb returns actually rounded to >= 1.5 m        : 0    (0.0%)
  overlapping band (> 0.5 m2 double-covered)       : 216  (100.0%), 6246.1 m2 total
  band missing beside the kerb (> 1 m)             : 114  (52.8%), 307.8 m total
  band SPIKES past 5.2 m (1.5x the 3.5 m walk)     : 79   (36.6%)
  worst single-vertex kerb turn                    : 169.7 deg
  band reach from the kerb  median 4.63 m / worst 8.00 m (want 3.50 m)
```

Not a tail. **Every** junction in the level reads as mitred *as the band
receives it*, and the average junction stacks ~29 m² of sidewalk on top of
itself, up to 9 layers deep. (The 0% rounded line is measured on the kerb line
the band is actually handed. Strip the hairline spurs first and 63.7% of those
corners turn out to be real arcs — see Cause 1, which is why the ordering of
these two facts matters.)

It reproduces without a level at all — `curb_weld_probe --arena` on synthetic
stars:

```
arms   spread   reach    worstTurn corners  rounded  ovl     triArea   covered
4      90       4.95     101.1     8        4        40.8    1829.5    1719.0
4      75       7.32     124.0     4        1        35.8    1785.2    1721.6
4      60       4.95     120.0     4        1        0.0     1664.0    1660.5
3      30       9.50     30.9      1        1        35.2    1083.6    1021.2
```

A plain **90° four-way** — the most ordinary junction there is — folds 40.8 m²
of sidewalk over itself and reaches 4.95 m instead of 3.50 m.

## Cause 1 (primary) — hairline reversal spurs, manufactured by the snap itself

Dumping the kerb line vertex by vertex at (137.88, 205.30) shows what the band
is actually handed. One corner of one block loop:

```
     x          z       edge-len   turn
  137.810   193.990      4.349     123.2
  137.800   193.990      0.010    -130.5   <- 1 cm edge, kerb reverses
  139.040   195.440      1.908     -20.8
  140.450   196.210      1.607     -25.0   <- these four ARE the rounded
  142.030   196.310      1.583     -21.7      corner: ~20 deg per segment,
  143.780   195.740      1.840    -117.0      radius ~4.7 m
  143.770   195.730      0.014     109.4   <- 1.4 cm edge, kerb reverses
  144.270   195.490      0.555      -0.5
```

The arc in the middle is real — `junctionPatch`'s own pad fillet
(`tests/test_road_lattice.cpp: junction_pad_corner_fillets_round_the_kerb`)
survives the union and reaches the band. What ruins it is the **1 cm reversal
spur on each side of it**.

Those spurs are manufactured by the cleanup that is supposed to remove them
(`road_net.cpp`, just before the band sweep):

```cpp
const Vec2 q(std::round(p.x * 100.0) / 100.0,      // snap grid: 1 cm
             std::round(p.y * 100.0) / 100.0);
if (c.empty() || (q - c.back()).length() > 0.005) c.push_back(q);   // dedupe: 0.5 cm
```

**The snap quantum (1 cm) is coarser than the dedupe epsilon (0.5 cm).** Two
union vertices that were a hair apart snap to grid points exactly 0.010 apart —
above the 0.005 threshold — so the pair survives as an edge with no meaningful
direction, and the kerb reverses across it. The de-spike pass that follows only
drops a vertex when `dot(e0, e1) / (l0 * l1) < -0.985`, a turn steeper than about
**170°**; these reversals measure 90–166° and sail through.

Across the level: **539 hairline spurs over 187 loops**, worst reversal 153.4°.
The turn distribution shows the tail that survives:

```
  144-162 deg :     19
  162-180 deg :      9   <- the de-spike filter cuts at ~170 deg
```

Measure the same corners on a **de-spurred** copy of the kerb line and the
geometry is largely fine: of 562 junction corners, **358 (63.7%) are properly
rounded arcs**. The corner geometry mostly works; the spurs are what the band
chokes on.

## Cause 2 (amplifier) — the band's mitre is unbounded and never checked for folds

`sweepCurbSidewalkBand` (`road_lattice.cpp`) offsets each loop vertex outward
along the corner bisector:

```cpp
const bool opens = (e0.x * e1.y - e0.y * e1.x) > 0.0;
if (!opens) {                                   // PINCH
    const Vec2 q = loop[i] + mm * (sidewalkWidth / std::max(0.2, cosH));
```

`sidewalkWidth / cosH` is the textbook mitre offset and it diverges as the corner
sharpens. The `max(0.2, ...)` clamp bounds it at **5× the sidewalk width** —
17.5 m at a 3.5 m walk. Hand it a 130° reversal spur and it emits a multi-metre
spike into the intersection; nothing afterwards tests whether the offset polyline
crossed itself, so the spike folds over its own band and its neighbour's.

Measured: band reach median **4.63 m** and worst **8.00 m** against a 3.50 m
sidewalk, 36.6% of junctions spiking past 5.25 m, and 6 246 m² of double-covered
band stacking up to 9 layers deep.

**Correction to an earlier reading of this number.** The 4.95 m floor that shows
up in every clean arena case is `3.5 × √2` — and it is CORRECT, not a defect. A
3.5 m band around a right-angle corner genuinely reaches 4.95 m at the apex,
because the outer corner is the intersection of the two offset lines and each
line is still exactly 3.5 m from its own kerb edge. The `reach` metric measures
distance to the nearest kerb SEGMENT, and at an apex the nearest point is the
shared endpoint, so it over-reads by 1/cos(half-angle) at every corner by
construction. What is wrong is only the RUNAWAY: the `max(0.2, cosH)` clamp let
that same intersection sit 5× the band's width away, and nothing checked whether
the resulting quad had turned inside out.

Seen directly (`--focus 137.88 205.30 30 --svg /tmp/curb.svg`; docs carry no
PNGs, `.gitignore` drops them, so regenerate it): each of the four block kerb
loops around that junction ends in a long triangular spike shooting into the
intersection and crossing its neighbour's, while the corner itself turns on a
single vertex. The spikes are the band's outer edge; the fold is where two of
them cross.

## Cause 3 (secondary) — the authored corner radius drives nothing

`curbReturnFillet()` in `road_mesh.h` is a true fixed-radius arc tangent to both
kerb lines at any angle, which shrinks to fit a tight corner. It has four
passing unit tests (`tests/test_curb_return.cpp`). **It has no callers** — only
its definition and its tests reference it.

`RoadLook::cornerRadius` — commented "rounded kerb-return radius (m)", default
3.0 m, in the editor properties panel and serialized into every level — is read
by exactly two functions: `roadNetFromJson` and `roadNetToJson`. It is
round-tripped and never used. The rounding that does happen is `junctionPatch`'s
internal pad fillet, which has no access to `RoadLook` and picks its own radius
(measured ~4.7 m against an authored 3.0 m).

So the knob is dead, the general-purpose fillet is dead code, and the 36% of
junction corners that are *not* rounded even after de-spurring have nothing to
round them.

## What the algorithm is missing

In rough order of leverage:

(1) through (4) are DONE — see "What the fixes did" above. (5) is not.

1. ~~**Make the dedupe epsilon at least the snap quantum.**~~ DONE. Snapping to 1 cm and
   then merging at 0.5 cm cannot converge — the snap creates 1 cm pairs by
   construction. Merging at, say, 2 cm after a 1 cm snap kills the 539 spurs at
   their source, and it is a one-line change.
2. ~~**De-spike by spur SHAPE, not vertex angle.**~~ DONE. The `-0.985` cut only catches
   reversals past ~170°; the ones that do the damage are 90–166°. A spur is
   better identified by the area its removal changes (a sliver of near-zero
   area) or by a short edge with a large reversal — both catch the real cases
   without touching a legitimate sharp corner.
3. ~~**A bounded join in the band offset**~~ DONE (mitre bounded at 2×, arc past it), so a corner
   produces a rounded outer edge at the offset radius instead of a mitre point
   that is 1.41× at 90° and 5× at the clamp.
4. ~~**A self-intersection removal pass on the offset polyline**~~ DONE as the
   trim relaxation, which unfolds 40% of the folded quads — the standard
   second half of polygon offsetting, and the only thing that can *guarantee*
   the band never covers ground twice whatever the generator hands it.
5. **Wire `look.cornerRadius` to a real fillet** — either by calling
   `curbReturnFillet` when the band loops are assembled, or by passing the look
   through to `junctionPatch` so the pad fillet honours the authored radius.
   Until then the editor's corner-radius control is a placebo.

(1) and (2) fix the kerb line the band is handed. (3) and (4) make the band
robust to whatever line it is handed — worth doing regardless, since a mitre that
reaches 1.41× at an ordinary right angle is wrong on its own. (5) is what makes
the authored radius mean something.

## What the fixes did

Four changes landed, in `road_net.cpp`'s band cleanup and `road_lattice.cpp`'s
band sweep:

1. **Merge tolerance ≥ snap quantum** — 2 cm against a 1 cm snap, so no
   adjacent-grid-point pair can survive.
2. **De-spur by shape** — a sub-decimetre edge goes at any angle; a reversal
   past 120° goes if it encloses under 0.05 m².
3. **Mitre bound + arc fallback** — the mitre is kept where it is valid (it is
   the correct answer, see the correction above) and bounded at 2× the band
   width; past that the corner is swept on an arc of the band's own radius.
4. **Trim** — the second half of polygon offsetting. A quad is valid only while
   its outer edge runs the same way as its kerb edge; when a corner reaches
   further than the neighbouring kerb segment is long, the outer edge reverses
   and the slab folds back over its neighbour. A relaxation pass pulls the
   longer corner in by exactly the amount the constraint needs.

Measured on `metro_v2_test`, same probe, before → after:

| | before | after |
|---|---|---|
| hairline spurs | 539 | **0** |
| double-covered band | 6 246.1 m² | **883.9 m²** (−86%) |
| folded band quads | 770 / 16 423 | 462 / 16 423 |
| kerb returns rounded, as the band receives them | 0 (0%) | 65 (30.1%) |
| corners where one vertex eats the swing | 216 (100%) | 185 (85.6%) |
| kerb with no band beside it | — | 105.9 m of 15 618 m (0.7%) |
| kerb whose band is under half width | — | 600.4 m (3.8%) |

On the synthetic stars, where there is no generator noise to argue about:

```
              before                    after
arms spread   reach  ovl                reach  ovl    triArea  covered
3    90       4.95   22.2               4.95   0.9    1347.9   1345.9
4    90       4.95   40.8               4.95   1.3    1721.7   1718.8
4    75       7.32   34.5               5.55   9.9    1621.6   1607.4
5    72       4.95   57.8               4.95   0.8    1484.1   1485.2
3    30       9.50   35.2               4.99   29.4    976.5    928.3
```

An ordinary 90° four-way went from 40.8 m² of folded sidewalk to 1.3, and its
summed triangle area now matches the ground it covers (1 721.7 vs 1 718.8) —
i.e. the band lies down once. The needle case (two arms 30° apart) is the one
that still folds badly.

### What it cost, and what is left

The trim narrows the band where the geometry cannot support full width, which is
what a real kerb does — 3.8% of kerb near junctions now carries a sidewalk under
half width. That was a deliberate trade against the alternative: simply DELETING
folded quads was tried and measured (overlap 884 → 628 m², but bare kerb 106 →
633 m). Bare ground beside a kerb reads worse than a bounded overlap, so the
folded quads are still emitted; the trim just unfolds most of them first.

Two things remain, and neither is reachable from a per-loop corner rule:

* **462 quads still fold** (2.8%, down from 4.7%). These sit on kerb edges so
  short that a 3.5 m band cannot exist there at all; the trim floor (35% of the
  band width) stops before it deletes the sidewalk. Lowering the floor to 15%
  was measured and is worse overall (overlap 884 → 1 038 m²).
* **The remaining 884 m² is largely CROSS-LOOP**: one block's band reaching into
  the next block's. No per-loop offset rule can see that; it needs the bands
  unioned against each other, or the kerb lines to stop producing needle
  corners in the first place.

## Fix 5 — wiring the authored corner radius, and what it taught

`RoadLook::cornerRadius` is now threaded `buildRoadNetMesh` → `buildRoadNetLattice`
→ `junctionPatch`, where the corner between two arms becomes a TRUE circular arc
of that radius, tangent to both verges (`curbReturnFillet`'s first production
caller since it was written). What was there before had no radius at all: a
quadratic Bézier whose control point is the verge-line intersection, so its shape
fell out of wherever the arm mouths happened to land — measured at ~4.7 m against
an authored 3.0.

**It has to go on the ASPHALT boundary, not the sidewalk's copy of it.** Rounding
the band loop was tried first, since that is where the band is swept. It reads
better by every corner metric (mitred corners 216 → 0, worst single-vertex turn
169.7° → 25.7°, holes 106 m → 0.4 m) and is still WRONG: the kerb line stops
following the road edge. 587 m of kerb ended up with no asphalt on its road side
— bare ground between the carriageway and the kerb. Rounding the pad instead
keeps the two in lockstep by construction (detached: 0.5 m), because the pad IS
the asphalt and the band is swept along the union that contains it.

**But it is shipped OFF, behind `RT_KERB_RETURN` plus a `radius > 1.05 ×
sidewalk` guard**, because at this level's authored numbers it makes the band
worse:

| metro_v2_test | folded quads | stacked band |
|---|---|---|
| no rounding, 3.5 m walk | 462 | 883.9 m² |
| r = 6 m, 3.5 m walk | 832 | 1 083.7 m² |
| r = 6 m, 2.0 m walk | 464 | 253.9 m² |
| no rounding, 2.0 m walk | 272 | 229.9 m² |

It also fails two junction-quality tests at 3.5 m (`zoo_skew_tee_mixed_widths`,
`merge_probe_drives_ramp_onto_mainline_on_a_real_metro`), confirmed by bisection.

The reason is geometric, not a bug: **the sidewalk sits on the CONCAVE side of a
kerb return**, so the band is an INWARD offset of that arc — and a curve cannot
be offset inward by more than its own radius of curvature. A 3.5 m walk around a
3.0 m return is degenerate before it is drawn. Real streets keep 1.5–2.5 m walks
against 3–9 m returns, and the table shows the same thing: at 2.0 m the folding
collapses by 4× and the rounding costs almost nothing.

**The sidewalk width is the dominant variable, ahead of the rounding.** 3.5 m
came from a device request ("sidewalks should be wider"); it is also what drives
most of the remaining fold. Narrowing to ~2.0–2.5 m improves the band roughly
4× on its own, with or without kerb returns, and is what would let the corner
radius finally be turned on.

## Reproducing

```sh
cmake --build build --target curb_weld_probe
./build/curb_weld_probe assets/levels/metro_v2_test.json --top 20 --csv /tmp/curb.csv
./build/curb_weld_probe --arena                     # synthetic stars, no level
./build/curb_weld_probe assets/levels/metro_v2_test.json \
    --focus 137.88 205.30 30 --svg /tmp/curb.svg    # look at one junction
```

Every finding carries world (x, z), so the worst junctions can be driven to or
aimed at directly in the editor.
