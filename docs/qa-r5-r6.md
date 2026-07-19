# QA pass — R5/R6 session changes

Companion to `drive-checklist.md` (the feature acceptance list). This one is
**change-driven**: for each thing that changed this session, what to look at,
what failure looks like, and where the change could have broken something
that used to work. Report by item ("Q3 — cars lean too long cresting ramps").

Launch: `./build/viewer metropolis --play` and `./build/viewer hillcity --play`.
`K` follows cars, backtick is the debug overlay, `Esc` is edit mode.

---

## Q1. Physical car tier (R5 — the ~12 cars nearest you are real Jolt bodies)

- [ ] **Suspension is visible.** Follow close traffic: braking dive at reds,
  roll in corners, settle over bumps. If close cars still look like sliding
  boxes, the tier isn't engaging.
- [ ] **No flips, ever.** A car on its side/roof anywhere is a hard fail.
- [ ] **Count the snaps.** A car wedged by traffic teleports back to its
  lane after ~1.5 s — that's the designed fuse. Seeing it *occasionally* is
  expected; seeing it every minute near you is a fail. The count is the
  calibration number I need.
- [ ] **No grinding.** A car pinned against a parked car, pole, or another
  car, wheels spinning for seconds — fail (the fuse should rescue in ~1.5 s).
- [ ] **Hand-off is invisible.** Sprint/drive away from a group of cars,
  then return: no popping, no duplicated cars, no car frozen mid-road where
  the tier released it.
- [ ] **Commandeering still works.** Enter a car (the ADR-0062 promotion)
  in dense traffic, drive it, exit. The possession tier and the player
  promotion share machinery — watch for a car that won't release, or a
  ghost car left behind.
- [ ] **You still collide with ambient cars.** Walk into slow traffic
  farther from the pack: the kinematic proxies must still push you.

## Q2. Traffic behaviour retune (affects ALL cars, both tiers)

The brains changed so physical cars could follow the plan: acceleration
6 → 4 m/s², cars brake into corners (~10 km/h for a 90°), and junction
arcs widened to a real ~5 m turning radius.

- [ ] **Corners look human.** Cars slow before the turn, arc wide, speed
  out. They should NOT swing into the oncoming lane, clip the inside curb,
  or hit corner signal poles mid-arc.
- [ ] **U-turns stay tight.** At dead ends, turning cars must not sweep a
  huge loop across the whole road (that specific bug was fixed — verify).
- [ ] **No new gridlock.** Sit at the busiest 4-way you can find for 3–5
  minutes. Slower launches + slower corners lower throughput; queues must
  still clear each green, not grow without bound.
- [ ] **Pedestrian safety held.** Cars now approach walkers slower; kerb
  gap-acceptance should look the same or better. A walker under a bumper
  is a hard fail.

## Q3. Car pose filter (the vibration fix)

- [ ] **No jitter.** Cars on graded/bumpy streets hold a steady body —
  the old per-frame tilt shiver should be gone everywhere.
- [ ] **No sluggish tilt either.** The filter has a ~0.7 s constant:
  a car cresting a ramp or hitting a grade change should settle its pitch
  quickly and naturally, not float nose-up like a boat. If tilt visibly
  lags the road, say where.

## Q4. Density population (metropolis 362 cars / 198 walkers, was 80/30)

- [ ] **Perf.** Downtown at street level during rush: note the frame rate.
  This is 4.5× the cars and 6.6× the walkers — the biggest perf risk of
  the session.
- [ ] **Distribution.** Streets should read alive city-wide; note any
  district that's dead or any street that's wall-to-wall saturated.
- [ ] **Signals under load.** Busy junctions with real queues: cycles
  still clear them.

## Q5. Retaining walls (R6 — hillcity is the showcase)

- [ ] **Deep cuts are walled.** Where a road cuts into a hill steeply, a
  concrete wall with a flat bench crown fronts the face. Raw near-vertical
  dirt faces taller than ~4 m should no longer exist next to roads.
- [ ] **No walls around lawns.** The first version fenced in flat graded
  blocks — that's fixed, but verify: a wall standing beside ground that's
  LEVEL with the road is a bug.
- [ ] **Wall ends.** Runs end abruptly at full height (no taper yet —
  known). Note the worst-looking end you find.
- [ ] **Solid.** Walk into one: it should stop you (it has a collider).
- [ ] **Seams.** Where the bench crown meets the hillside: gaps or
  z-fighting flicker.
- [ ] **Look.** The concrete reads very dark in its own shadow in my
  headless frames — judge whether it needs material work.

## Q6. Curbside parking (R6)

- [ ] **Bays exist and read right.** Local streets: white bay outlines
  mid-block, roughly half holding parked cars, aligned with the road, not
  floating or sunk, never within ~20 m of a junction.
- [ ] **Clearance.** Watch traffic pass a parked row: ambient cars keep a
  visible gap (parked streets narrow their drivable width). Scraping = fail.
- [ ] **The physical tier vs parked rows.** This was the hardest
  interaction: near-you cars passing parked cars are the most likely place
  to see grinding/snaps. Watch it specifically.
- [ ] **The life cycle.** Tail a car to its destination on a local street:
  it should come to rest IN a bay (not on the grass) and later pull out.
  Verge-resting remains correct on streets without bays.
- [ ] **Collision.** You can't walk or drive through a parked car.
- [ ] **Walkers ignore them correctly.** No pedestrians pathing through
  parked cars or standing inside them.

## Q7. Stop bars + turn arrows (R6)

- [ ] **Coverage.** Every SIGNALLED approach: one white stop bar + one
  arrow per lane. T-junctions (unsignalled): none.
- [ ] **Correctness.** Arrows match the turns that actually exist at that
  junction; arrows sit on lane centres (including on parked-up streets
  where lanes shift inward).
- [ ] **Paint quality.** Flat on the asphalt — no floating, no z-fighting
  shimmer, nothing on sidewalks, nothing poking through the crosswalk zebra.

## Q8. Girders + railing posts (R6 — the freeway's underside and edges)

- [ ] **From below.** Under every elevated span: two box girders hang
  under the deck edges. They should stop where the road returns to grade.
- [ ] **From the deck.** Railing posts top the parapets every ~3 m; the
  runs must GAP at gore merges exactly like the parapets do — a post
  standing in a merge opening is a fail.
- [ ] **At flares.** Where the deck widens at gores, posts follow the
  flared edge — no posts floating inboard or hanging off the edge.

## Q9. Regression sweep (things these changes touch indirectly)

- [ ] **Editor regenerate.** `Esc`, regenerate a road: walls, bays,
  markings, girders, posts all rebuild consistently with the new graph.
- [ ] **Signals/WALK unchanged.** Conflict phases and WALK windows behave
  as before (no all-green, offsets staggered).
- [ ] **Freeway drive.** Mainline + every ramp end to end — the mesher
  changes (girders/posts/gaps) must not have reopened holes, steps, or
  blockers anywhere.
- [ ] **hillcity full pass.** It exercises walls + grades + bents at once;
  anything weird there, screenshot it.

## Priorities

Q1/Q2 first (they change how every car behaves — the core of the round),
then Q6 (the riskiest interaction), then Q5 on hillcity. Q4's frame rate
decides whether the density default survives. Everything else is polish
calibration.
