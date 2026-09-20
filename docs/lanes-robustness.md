# How far does the lane builder go? (2026-09-20)

**Trigger.** Glenn: "we should really build more road networks + freeways + onramps and really vet
the system ... metro_test_v2 looks good. But I think there's more testing to be done to see how
robust the lane building system is."

Ten scenes through `lanes_tool build`, in two groups: the five graphs **authored by hand** for the
lab, and five **imported from a shipped level's road recipe** (`lanes_tool from-level`, the path a
real city would take). Every number below is from that run; nothing is estimated.

## The scoreboard

Authored graphs — the lane geometry engine on input made for it:

| scene | triangles | non-manifold | cracks | junction mismatch before → after | terrain above deck |
|---|---|---|---|---|---|
| hill_junction | 8,540 | 0 | 0 | 35.8 cm → **0.0** | 1 of 14,032 (5.9 cm) |
| curve_blocks | 11,994 | 0 | 0 | 0.0 → **0.0** | — |
| grid_city | 22,973 | 0 | 0 | 0.0 → **0.0** | — |
| valley_viaduct | 10,528 | 0 | 0 | 0.0 → **0.0** | 0 of 16,084 |
| ring_city | 66,321 | 0 | 0 | 77.6 cm → **0.0** | 0 of 106,334 |

All invariants PASS on all five, except hill_junction's one 5.9 cm terrain sample (the failure
`lanes_tests` already carries).

Imported from a level recipe — the same engine on input the importer produced:

| scene | triangles | non-manifold | cracks | mismatch before → after | terrain above deck (worst) |
|---|---|---|---|---|---|
| **metro_v2** (the tuned one) | 518,048 | **0** | 2 | 156.8 → **0.0** | 387 of 817,421 (0.5 m) |
| freeway_lab | 86,255 | 104 | 40 | 392 → **318** | 1,482 of 145,022 (**9.7 m**) |
| living_city | 42,235 | 13 | 23 | 222 → **410** | 1,303 of 73,836 (1.7 m) |
| hillcity | 275,470 | 278 | 95 | 331 → **385** | 6,040 of 475,281 (**11.0 m**) |
| coast_city | 21,269 | 0 | 0 | 198 → 173 | 250 of 33,822 (0.9 m) |
| metro_hills, piedmont_mini | — | — | — | — | no perimeter ring found: no freeway at all |

coast_city looks clean only because there is almost nothing there — see below.

## What the importer actually kept

| level | streets kept | dropped | landings built (of those considered) | ramps |
|---|---|---|---|---|
| metro_v2 | — (reference) | — | 3 diamonds | 12 |
| hillcity | 42 | **47** | 2 of ~25 | 8 |
| freeway_lab | 5 | **38** | 1 of ~14 | 4 |
| living_city | 7 | **27** | 0 of 3 | 0 |
| coast_city | **0** | 5 | 0 of 2 | 0 |

The importer discards most of a real city, and on two levels it finds no ring to build a freeway on.

## The five failure modes, with the evidence

1. **The redrawn freeway ring does not hold its radius.** It is specified `R >= 220 m` and reports
   what it actually achieved: hillcity **min radius 9 m**, freeway_lab 10 m, living_city 10 m,
   coast_city 12 m, with long "tight (< 150 m)" station lists on each. Everything downstream is
   built on that alignment, so this is the root of most of the rest.
2. **Ramps that cannot make their climb are built anyway.** freeway_lab's diamond reports
   `d0_in_off 88.2% > 8%` and `d0_out_off 94.3% > 8%`. Those are not ramps, they are walls. The
   companion failure, "decks follow their profiles", shows partners pulling lanes **12–18 m across
   levels** to reach them.
3. **The junction-height agree loop does not converge — and sometimes diverges.** On authored
   graphs it lands on 0.0 cm. On hillcity it goes 331 → **385 cm**, on living_city 222 → **410 cm**:
   the correction makes it worse. A loop that can move away from its fixed point is the thing to
   instrument first.
4. **Decks stop being welded.** Cracks and non-manifold edges appear only on imported graphs
   (hillcity 278 non-manifold / 95 cracks), and metro itself carries 2 cracks.
5. **Grade separation stops clearing.** freeway_lab: 33 of 53 separated lane pairs short of
   clearance, worst by 6.6 m. living_city: **36 of 36**. These are crossings drawn as if they
   passed over one another while the gap is ~1.3 m.

## The reading

**The lane geometry engine is solid; the importer is what is not robust.** Given a graph authored
for it, the builder welds decks, converges junction heights to zero, keeps grades legal and clears
its structures — on five different scenes including a viaduct and a ring. Given a graph the importer
derived from a real city recipe, it fails in ways that all trace back to the *input*: a ring with
9 m curves, landings at 2°, ramps asked to climb 15 m in 159 m of street.

metro_v2_test is not evidence that the system generalises. It is the level the importer was tuned
on, and it is the only imported graph that comes out clean.

## What to do about it, in order

1. **Make this corpus a gate.** The table above is a script's worth of work
   (`lanes_tool from-level` + `build`, parse the invariants). Robustness then has a number, and the
   next change to the importer either improves it or does not.
2. **Fix the ring redraw** (failure 1). It is the root: alignment feeds ramp feasibility, which
   feeds clearance and the profile pull.
3. **Make landing selection honest** (failure 2). Today it both over-rejects (hillcity: 23 of 25
   candidates skipped) and under-rejects (freeway_lab: a landing accepted whose ramps then need
   94%). A candidate should be scored on whether its ramps can actually be built, once.
4. **Instrument the agree loop** (failure 3) before touching it — print per-iteration mismatch and
   find out where it turns around.
5. Then re-run the corpus and see what is left of failures 4 and 5, which may be consequences
   rather than causes.

Artefacts of this run (graphs, stats, SVG plans) are under `/tmp/claude-road/lanes/`; nothing was
added to `assets/`.
