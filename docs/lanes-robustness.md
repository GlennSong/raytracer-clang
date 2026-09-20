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

## It is not only the freeway machinery

`metro_hills` and `piedmont_mini` are the control for that claim: neither has a perimeter ring, so
**no freeway, no landings, no ramps are built at all** — just streets converted to lanes. metro_hills
still fails six invariants, including grades (`c2 12.8% > 8%`, an arterial over its own class limit)
and 9 of 9 grade-separated pairs short of clearance. Plain street conversion is not clean either.

Following that one down found a specific mechanism, and it is a disagreement between three passes
about what "the same level" means:

| pass | rule | where |
|---|---|---|
| crossing consistency | pulls two roads together when they are within `bridge_h` (4.0 m) | `vertical_profile.cpp:115` |
| the clearance lift | raises the upper road only when they are **already** more than `bridge_h` apart (`ramp_level_dz`, 1.5 m, for ramps) | `lanes.cpp:78` |
| the clearance invariant | judges lane pairs "purely grade-separated (never same-level anywhere they meet)" — a third rule, on lanes rather than edges | `lanes.cpp:318` |

A crossing sitting 1–4 m apart therefore falls between all three: consistency did not bring it to
zero, the lift declines it as "a level crossing the street meets", and the invariant still calls it
a grade separation and demands 7.2 m. metro_hills' nine failures all sit at ~1.1 m
(`z 5.37/6.48`). The reference city never shows this because it lifts properly: metro's log says
`9 grade-separated crossings lifted to clear their structure`, and **metro_hills' log never mentions
a lift at all.**

## The reading

**The lane geometry engine is solid; the importer is what is not robust** — and where the engine
itself is weak, it is at thresholds that an authored scene never lands on. Given a graph authored
for it, the builder welds decks, converges junction heights to zero, keeps grades legal and clears
its structures — on five different scenes including a viaduct and a ring. Given a graph the importer
derived from a real city recipe, it fails in ways that all trace back to the *input*: a ring with
9 m curves, landings at 2°, ramps asked to climb 15 m in 159 m of street.

metro_v2_test is not evidence that the system generalises. It is the level the importer was tuned
on, and it is the only imported graph that comes out clean.

## The gate, and the scenes that were missing (2026-09-20, same day)

`tools/lanes_corpus.py` runs every scene in `tools/lanes_corpus.json` — importing first where the
scene comes from a level recipe — parses what `lanes_tool` already prints, and reports one table.
`--check` compares against `tools/lanes_corpus_baseline.json` and exits non-zero if a count grew or
an invariant that passed now fails; `--update` records a run as the baseline. Counts must not grow
at all; the continuous numbers carry a small tolerance, because a mesher is not bit-stable across
compilers and a gate that cries over 0.3 cm gets switched off.

The authored scenes are also wired into `lanes_tests`, which is the fast gate; it now carries two
known failures on purpose — hill_junction's 5.9 cm terrain sample, and the `steep_climb` open-end
defect below. Both are defects, not tolerances, and both go green when they are fixed.

Running the first corpus showed what it did **not** contain, so four scenes were authored for the
holes (all pass, which is the point — they are controls now, and they will catch the day they stop):

| new scene | what had never been tested | outcome |
|---|---|---|
| `freeway_cross` | two **freeway**-class roads crossing, one carried over the other. ring_city carries a freeway over *arterials*, which is not the same test | passes: 24 piers, clearance held, no steps |
| `freeway_cross_relief` | the same crossing with relief under it — the carried road must reach its hold and come back down inside its profile window | passes: 36 piers, agree loop 95.4 → 0.6 cm |
| `merge_taper` | a **2 → 1 ramp merge**. The lab builds two-lane ramps and has a dovetail station for the second lane, and no scene used it | passes: 14 lanes, one-lane control beside a 30 m and a 90 m dovetail |
| `steep_climb` | **sustained grade**: ground rising ~6% under a road capped at 4%, a hill on top, a crossing on the slope, a ramp climbing off it | passes — and see below |

Three things fell out of authoring them:

* **Two freeways with no hold between them meet at grade.** The agree loop pulls them together
  (182.6 → 0.6 cm) and builds one flat intersection of two 3-lane freeways. Nothing objects. The
  only thing that ever separates two high-rank roads is an author writing a `floor`, or the
  importer's diamonds.
* **A hold that is too low for the ground is built anyway.** The first cut of `freeway_cross` put
  its hold at 8 m over ground that is ~6 m up; the result was a crossing with a 2.0 m gap and
  36 of 36 clearance failures reported. It does not lift the structure to make its own clearance —
  the same shape of behaviour as the importer's 88% ramps: **infeasible input is built and
  reported, not resolved.**
* **The profile solver prefers structure to earthwork, and does not pin an open end to the ground.**
  `steep_climb` answers 114 m of relief with **1040 m of bridge** (55% of the road) and only 48k m³
  of cut — and it begins **65 m in the air**: deck 74 m where the ground is 8 m, because the profile
  picks a level that suits the middle of the road and holds it. The far end behaves (3.7 m into the
  hill, a cut). Every invariant passes; the road is a kilometre of viaduct over a valley floor it
  never comes down to. `lanes_tests` now pins this
  (`lanes_a_freeway_climbs_sustained_relief_within_its_grade`) and fails on the start gap until a
  profile pins its open ends.

## What to do about it, in order

1. ~~**Make this corpus a gate.**~~ Done: `tools/lanes_corpus.py` over `tools/lanes_corpus.json`,
   16 scenes, `--check` against a recorded baseline.
2. ~~**Author the scenes that were missing.**~~ Done: the four above, also wired into `lanes_tests`
   so the fast suite carries them.
3. ~~**Make the three level rules one rule.**~~ **Done** (2026-09-20). The dead zone was not a
   threshold disagreement at all — it was a disagreement about what counts as a CROSSING. A street
   ending a metre off another road's centreline (inside its paved band, outside `endpointTol`) was
   a node to nobody and a crossing to nobody: `nodeConsistency` only recognises a T within 0.5 m of
   the centreline, and `crossingConsistency` only looked at centreline intersections. The clearance
   invariant then saw overlapping lanes with no same-level area and asked for 7.8 m. Now the
   crossing pass uses the same inclusive rule the clearance lift uses (an end under the other's
   paved band counts), skips only genuinely SHARED nodes, and leaves `nodeConsistency` the cases
   within its own tolerance. A second finding fell out: the agree loop was capped at 12 iterations
   and each pass only partially resolves, so a real city stalled short — metro_hills read 35 cm at
   12, 19 at 30, 5.5 at 60. The cap is 120 with the existing 0.5 cm early exit, so the lab scenes
   are unaffected and unchanged in cost. **metro_hills, which builds no freeway at all: clearance
   failures 9 of 9 → 1 of 1, surface steps 146 → 1, junction solve 312 → 0.5 cm.**
4. ~~**Pin a profile's open ends to the ground.**~~ **Done** (2026-09-20). `gradeLimit` is the max
   of two monotone envelopes — it fills and never cuts — so a road that cannot follow its terrain
   floated. An end nothing meets is now pinned to the ground there, and nothing may sit higher than
   the grade allows from it (a cone of slope `gd`, so the result is still grade-legal; a crossing's
   hold within reach of an open end loses to it). `steep_climb` went from **starting 65 m in the
   air with 1040 m of viaduct** to starting at the ground with none. The rule the test now holds is
   the honest one: an open end may be CUT into the ground — that is buildable — but must never
   float. What it exposed in exchange: the terrain conform does not fully clear a 53 m cutting
   (~80 samples above the deck, step edges at the lip), which is the scene's remaining pair of
   failures and a fair question about what a builder with no switchback and no tunnel should do
   when the ground out-climbs the road.
5. **Fix the ring redraw**: it is specified `R >= 220 m` and delivers 9–12 m on every real city.
   Alignment feeds ramp feasibility, which feeds clearance.
6. **Score landings on whether their ramps can actually be built**, once — today it both
   over-rejects (23 of 25 skipped on hillcity) and under-rejects (a landing whose ramps come out at
   94%).
7. **Instrument the agree loop** before touching it: on two cities it moves away from its fixed
   point (331 → 385 cm, 222 → 410 cm).

Artefacts of a run are under `/tmp/lanes-corpus/`; nothing was added to `assets/levels/`.
