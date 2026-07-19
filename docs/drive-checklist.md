# Drive checklist — Roads v2.1 acceptance (R1–R6)

Launch: `./build/viewer metropolis --play`, then repeat the hill items on
`./build/viewer hillcity --play` (the standing hills testbed).
`Esc` → edit mode, backtick → debug overlay, `K` → spectate/follow cars.

Each item names the feature it validates. Items marked **KNOWN EDGE** are
documented open edges — noting *how bad* they look is the useful data.
Report by item number ("B3 fails at the second on-ramp").

Everything below was rebuilt or added since your last drive: the corridor
renderer is DELETED (the baked graph is the freeway, meshed by the one
street mesher), signals run conflict phases, blocks are graded and
NYC-scaled, and the nearest cars to you are now real physics vehicles.

## A. Streets, up close (one mesher; R2 junction zoo quality)

- [ ] **A1 — Junction surfaces.** A dozen mixed intersections (4-way, T,
  acute forks, the odd 5-arm): flat continuous pads, no bumps/steps, no
  sliver fans, no stacked surfaces. Acute forks now carry wedge trims and
  rails instead of floating ribbons (your A1 finding — the zoo gates it).
- [ ] **A2 — Curbs and corners.** Rounded curb returns at corners, closed
  sidewalk bands, curb ends CAPPED (no open square tube mouths — your A2).
- [ ] **A3 — Hills: conform + walls.** On hillcity especially: roads sit ON
  carved ground; deep hill cuts now carry concrete RETAINING WALLS with a
  bench crown instead of raw near-vertical dirt (new, R6). Shallow cuts
  stay grassy on purpose. Note any raw steep face without a wall, or a
  wall standing somewhere silly.
- [ ] **A4 — Markings.** Centrelines mid-road, zebras only at junctions —
  and NEW: a white STOP BAR plus a per-lane TURN ARROW on every signalled
  approach, none of it outside the asphalt.
- [ ] **A5 — Parking (NEW, R6).** Local streets carry marked parallel bays
  mid-block, roughly half filled with parked cars. Driving past them must
  not clip them; arriving drivers pull into free bays to rest and pull
  out again later. Bays never crowd a junction mouth.

## B. The freeway (R1 unification + R6 structure)

- [ ] **B1 — Mainline end to end.** Both directions: no holes, steps, or
  blockers; parapets run the edges, OPEN across every gore (merges work —
  the merge probe drives every gore in CI now).
- [ ] **B2 — Every ramp.** Street → landing → climb → deck and reverse.
  Landings are real junctions (the 3-mesher seam is gone — one mesher).
- [ ] **B3 — Under the deck.** Piers are now PORTAL BENTS (two columns + a
  cap beam) that dodge carriageways; real headroom; and NEW: box GIRDERS
  hang under the deck edges, railing POSTS top the parapets every ~3 m.
- [ ] **B4 — Vegetation vs the corridor.** Trees under the viaduct fine;
  through structure is a defect.
- [ ] **B5 — Editable like a road.** `Esc`, click the mainline: sparse
  handles AT DECK HEIGHT (~a dozen per ramp, not hundreds — your B5).
  Drag a gore node, Regenerate: the freeway rebuilds around your edit and
  stays drivable (this is gated headlessly now, but see it once yourself).

## C. Traffic (R3 signals + R5 physical cars)

- [ ] **C1 — Physical cars (NEW, R5).** The ~12 cars nearest you are REAL
  physics vehicles — suspension, braking dive, corner roll. Watch close
  cars: no vibration (the old tilt jitter is filtered), no flips, no
  visible teleports. **KNOWN EDGE:** a car wedged by traffic may snap
  back to its lane after ~1.5 s — count how often you actually see it.
- [ ] **C2 — Corner behaviour.** Cars now BRAKE INTO corners (~10 km/h for
  a 90° turn) and take wide feasible arcs — no more whipping around
  junctions at cruise. Following queues stay smooth (IDM).
- [ ] **C3 — Signals: conflict phases.** Park at a busy 4-way: crossing
  streams NEVER share green (the all-four-green you found is structurally
  dead); each junction runs its own offset clock; a WALK window closes
  each cycle.
- [ ] **C4 — Signal furniture.** One head per approach, none at
  T-junctions, heads face their traffic.
- [ ] **C5 — Freeway flow.** Ramps used, gore merges without dead stops,
  freeway speed on the mainline.
- [ ] **C6 — Population (NEW, R6).** Density-based now: metropolis runs
  ~360 cars + ~200 walkers (was 80/30). Streets should read alive; note
  if any district still feels empty — the knob is per lane-km.

## D. Pedestrians (S8 kerb discipline + R3 WALK)

- [ ] **D1 — Sidewalks only;** never mid-carriageway, never on freeway.
- [ ] **D2 — Crossings.** Kerb-wait and gap-accept at unsignalled
  junctions; at signalled ones they go with their parallel green/WALK and
  never walk under a moving car.
- [ ] **D3 — Corners.** Walkers round corners on the walkway.

## E. The whole city (R4 ground truth)

- [ ] **E1 — Blocks: NYC bones.** Long rectangular blocks (~70×250 m) in
  the gridded districts, not squares. Buildings on lots, lots off roads.
- [ ] **E2 — No pits, walkable ground.** Walk off any road into a block:
  interiors are graded to their road ring — no inescapable bowls, no
  sidewalk cliff faces; you can step up onto every sidewalk.
- [ ] **E3 — Parks contained + solid.** Parks/greens stop at the road (no
  grass exploding into carriageways); fences and tree trunks are SOLID —
  you collide, you don't ghost through.
- [ ] **E4 — No regressions.** Terrain, ocean, fog, buildings — everything
  non-road as before.

## Scoring

A/B items are the mesher and graph — defects there are top priority.
C1/C2 accept the R5 physical tier (its wedge/snap rate is the number to
calibrate). A3 walls, A5 parking, A4 arrows, B3 girders/posts, and C6
density are the new R6 layer — for each, "looks right / looks wrong
where" is enough. E-items accept R4. Your findings remain the final gate
for the whole v2.1 round.
