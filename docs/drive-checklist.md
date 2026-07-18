# Metropolis drive checklist — Roads v2 acceptance (S1–S8)

Launch: `./build/viewer metropolis --play` (bare names resolve now).
`Esc` → edit mode, backtick → debug overlay.

Each item names the feature it validates. Items marked **KNOWN EDGE** are
documented open edges — noting *how bad* they look is the useful data there.
Report by item number ("B3 fails at the second on-ramp").

## A. Streets, up close (the one mesher: S4 pads, S5 curbs/sidewalks, S6 switch)

- [ ] **A1 — Junction surfaces.** Drive through a dozen intersections of mixed
  size (4-way, T, the odd 5-arm). The pad should be a flat, continuous
  surface — no bumps or steps crossing it, no fan of slivers from a centre
  point, no criss-crossed polygons. This is the S4 quad-pad rebuild.
- [ ] **A2 — Curbs and corners.** Every junction corner should carry a rounded
  curb return; sidewalks should read as closed bands around each block, with
  the curb between them and the asphalt. No floating ribbons, no sidewalk
  strips detached from their road. (S5 ownership: asphalt, curb, sidewalk
  never overlap.)
- [ ] **A3 — Terrain conform on hills.** Find sloped streets: the road must
  sit ON the carved ground — no asphalt submerged under terrain, no ground
  poking through the surface, no vertical walls of road edge. (The
  one-profile ride: mesh and carve share heights.)
- [ ] **A4 — Materials.** Asphalt on carriageways, concrete only on
  sidewalks; centrelines run mid-road; crosswalk zebras only at junctions.
- [ ] **A5 — KNOWN EDGE.** Crosswalk windows are junction-trim based, not
  skew-aware: at very acute forks the zebra/pad can look rough (bounded
  partial fill by design). Note the worst one you find.

## B. The freeway (S3 bake, S6a viaduct kit)

- [ ] **B1 — Mainline end to end.** Drive the full elevated spine both
  directions: no holes, no steps, nothing blocking the lanes. Parapets run
  the edges but must be OPEN across every gore — you can physically merge.
- [ ] **B2 — Every ramp.** Take each on/off ramp: continuous surface street →
  landing → climb → deck and the reverse. The landing must meet the street
  at a real junction — no wedge gaps, no dropped ground at the ramp foot.
- [ ] **B3 — Under the deck.** Drive the streets below: piers never stand in
  a carriageway, real headroom under the deck, ground continuous beneath.
- [ ] **B4 — Vegetation vs the corridor.** Look for trees through the deck or
  through ramps. Under the viaduct is fine (intended); THROUGH structure is
  a defect. (Unverified from headless runs — this one needs your eyes.)
- [ ] **B5 — THE PROMISE: freeway is editable like a road.** `Esc` to edit,
  click the freeway mainline: nodes should show handles AT DECK HEIGHT.
  Click a ramp: same. Drag a node — it moves like any street node. Then
  Regenerate the road entity: the freeway must survive (the regen re-bakes).
  This is the thing asked for six times; it is only done when you see it.

## C. Traffic (S7: IDM + senses)

- [ ] **C1 — No weave in town.** Watch/follow cars at street speed: they hold
  their lane line steadily. On the freeway at speed, a slight human drift is
  intended. (Fixed this session — the old gate was a no-op.)
- [ ] **C2 — Following.** Queues form smoothly behind slow cars and at reds —
  no accordion slamming, no rear-end pile-ups. (IDM.)
- [ ] **C3 — Junction conflict.** Park at a busy 4-way for a few minutes:
  crossing and turning cars brake for each other and nobody drives through
  another car at speed. **KNOWN EDGE:** occasional low-speed nose-to-nose
  shuffling at junction mouths is the documented kinematic-lane limitation
  (fix = the physical steering tier). Contact at real speed is a defect;
  creep-speed awkwardness is the known edge — note how often you see it.
- [ ] **C4 — Signals.** Exactly ONE signal head per approach at 4-way
  junctions; NO signal poles at T-junctions at all (they're uncontrolled by
  design); heads face their approaching traffic. (Fixed this session.)
- [ ] **C5 — Freeway flow.** Cars use the ramps, merge at gores without
  stopping dead on the mainline, and freeway traffic moves at freeway speed.
- [ ] **C6 — One car, one day.** Tail a single car: it pulls out, drives its
  route, waits at reds, turns, and arrives without teleporting.

## D. Pedestrians (S8: band-model walking + kerb discipline)

- [ ] **D1 — Sidewalks only.** Mid-block, walkers stay on sidewalks — never
  down the middle of a road, never on the freeway, ramps, or any road
  without sidewalks.
- [ ] **D2 — Crossings.** At unsignalled junctions walkers WAIT at the kerb
  while cars are coming and cross in gaps (after a long wait they assert,
  but never step under a car). At signalled junctions they hold on red.
- [ ] **D3 — Corners.** Walkers round bends and corners staying on the
  walkway — no dipping into the carriageway on curves.

## E. The whole city (the road graph as foundation)

- [ ] **E1 — Blocks and lots.** Buildings sit on lots, lots respect roads (no
  building in a carriageway); under-freeway blocks are parking/utility/open
  space, not towers through the deck.
- [ ] **E2 — No regressions.** Terrain shaping, ocean, fog, building look —
  everything not road-related should look as it did before Roads v2.

## Scoring

A/B items are the mesher and graph — defects there are top priority.
C3/C5 calibrate the S7 gate. B5 is the acceptance for the whole S3 arc.
B4 and A5 decide two open tasks (#65, crosswalk geometry).
