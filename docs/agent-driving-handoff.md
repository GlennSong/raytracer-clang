# Hand-over: agent-oriented driving (2026-11-16)

For a fresh session picking this work up. Read this, then the documents it points to. It says where things
stand, what is next, and the owner's standing rules.

## The goal

The new driving is oriented around the residents who live in the world. A car moves because a person is
driving it. Driving is a skill the person's brain uses. Detail follows distance from the player: near is a
full agent (later with a physics body and visible crashes), out of sight is simplified with a dice roll, far
is a person due somewhere at a time. The old driving inside `src/apps/citysim/city_sim.cpp` is to be deleted
once the new one carries river_town; until then leave it running and do not patch it.

## Read first

1. `AGENTS.md`: the repo's rules. All apply.
2. `docs/agent-brain-design.md`: the agreed design. Section 9 has a "Built" block, section 12 the owner's
   decisions, section 14 where building corrected the design.
3. `docs/decisions.md`, ADR-0439 to the last one (0443 if present): what each sitting built.
4. `docs/roads-orientation.md`: the one road model and its query functions.
5. `docs/driving-model-design.md`: the new driving's own design (tasks A to E).
6. GitHub issues #285 (joining, zipper, lane changes, crashes by distance), #286 (the lab's failing limits),
   #290 (one-agent lab), #291 and #292 (road model leftovers).

## Where the code is

| What | Where |
|---|---|
| The road model (one object per level; ask it, never derive from geometry) | `src/engine/procgen/city/roads/` (`road_model.h`, `road_model_parts.h`) |
| The new driving library | `src/apps/driving/` (front door `driving.h`) |
| The brain library (must not include `city_sim.h`) | `src/apps/citysim/brain/` |
| States, events, transitions (data) | `assets/scripts/brain.lua` |
| Ways of travelling (data) | `assets/scripts/travel.lua` |
| Glue between brain, citysim and driving | `src/apps/citysim/city_brain.*`, `city_brain_system.*` |
| The test level and its markers | `assets/levels/brain_lab.json`, `assets/levels/markers/traffic_lab.json` |
| Tests | `tests/test_brain_*.cpp`, `tests/test_brain_lab.cpp`, `tests/brain_fakes.*` |
| Unattended run with pictures | `tools/brain_lab_probe.py` |

## What works (committed, on `claude/island8-batch`)

- One road model per level, made in one place; kerb parking spaces and "from a door to the road" are
  questions it answers (ADR-0439).
- The brain: a 120-byte record per person (intent, a plan of up to 8 legs, one state), a machine that names
  no state, one event queue, near and far detail (ADR-0440).
- On `brain_lab`, one resident makes real trips between markers: walk to the car, pull out, drive on the
  new driving, park in a held kerb space, walk. A skateboard hop is one leg. `goto` while driving redirects
  the drive (ADR-0441, 0442).

- On `brain_lab`, six residents in three households share cars (whoever reaches one first takes it); the
  player or a director can take a car, parked or driven, and take a resident over; a person with no plan
  tries again (ADR-0443). Not built: the player giving back a car they took with the key. Check `git log`
  for a commit naming ADR-0443; if it is absent, that work is uncommitted in the working tree.

## What is next, in the agreed order

1. **Two agents meeting properly.** Design first, then build: lane changes; joining from a ramp by zipper
   (the joining car slows and takes turns, issue #285); the junction layer reworked so the lab's four limits
   pass (#286); crashes near the player as physics bodies. The roads now state what the junction layer used
   to guess (through pairs, facing approaches, turns, stop lines, merge marks), so it can be simpler. This
   is the largest piece. Break it into sittings of one or two items.
2. **Middle and far detail for brain agents**: the clock queue, raising people mid-leg, feeding the GPU
   crowd shader (`crowd.comp`) the "leg, start time, end time" record.
3. **A bus driver on a shift**, then the bus as a way of travelling (the transit states exist in
   `brain.lua` and refuse entry as "not built yet").
4. **River town on it**, intent by intent, and the old driving deleted.

Small things owed:
- real pull-out and parking (a sideways slide today; needs lane changes);
- the follow camera sits in the car roof;
- a `goto` in the tick the car reports "parked" leaves the car in the lane;
- the lot pass should record each door's street (costs a full re-bake; parked);
- boards and scooters dealt to the population by life stage.

## Decisions waiting on the owner

- The car's fixed 2 minutes at the start and 3 at the end of a trip are spent sitting in the parked car and
  decide when the car is chosen. Recommended: about 20 and 30 seconds for kerb parking.
- Stop 25 cm earlier so the bumper is short of the stop bar's near edge (moves the pinned lab run).
- The old driving: recommended to leave it until river_town runs on the new one, then delete.

## The owner's standing rules

Work and git
- ONE working branch, `claude/island8-batch`, in `.claude/worktrees/buildings`. No new branches, no stash,
  no merge to main or pull request unless asked.
- Never touch these modified files of his: `assets/levels/arena.json`, `car_drive.json`, `cdlod.json`,
  `forest.json`, `metro_planned.json`, `river_valley.json`; nor the untracked `metro_rivers`,
  `island_8_beach`, `zz_mtn_window` files; nor `imgui.ini`. Commit selectively, never `git add -A`.
- Do not discard uncommitted edits (`git checkout --`) without his word. Do not delete files you did not
  create.
- Stop only your own processes, by the PID you recorded. He runs `viewer` too. No wait loops on process
  names.
- Never run `run_tests` or `level_tests` without `RT_TEST_FILTER`.
- Give him commands as `cmake --build build-viewer --target X`.

Staying on the plot
- When he names the one goal, tangents and edge cases become GitHub issues, not work.
- Fix findings hardest first. Two functions that answer one question must agree and be merged into one.
- Only three levels matter for testing: traffic_lab (and brain_lab on its roads), river_town, agent_lab.
  Other levels may be stale. The target level is island_8_nature, too big to test on.

Code
- Kinds are data (Lua or data files); C++ never branches on a kind's name. No Lua per agent per tick.
- Use the existing systems; one per job. Ask the road model; never build a second view of the roads.
- No `auto` except lambdas. Computed numbers are compared through `approxEqual` and its companions in
  `src/rt_math.h` with named tolerances.
- The world drives on the left. Never say left or right for a road's sides: kerb side, centre-line side.
- Outside sources get a `CREDITS.md` line.
- Docs in plain language: say what a thing does before naming it, gloss borrowed terms.

Tests and claims
- A test must be able to fail: literal expected values, and each rule shown to bite by breaking it once.
- The pinned lab run (`RT_TEST_FILTER=level_traffic_lab_v2_drives ./build-viewer/level_tests`) must end at
  state hash `0x7aade29530b4fdd1`. It fails four known checks (report passed, loop samples, over-longest,
  the deadlock row); that is issue #286 and expected. Never edit a pin silently: report old, new and why.
- Verify a helper's claims against the tree before repeating them. Look at pictures yourself; a chase
  camera misleads (a car at its stop line looks past it), so take one from above.

Helpers and cost
- One helper at a time. One or two items per sitting. State the size before starting.
- Check the work, then commit; report the same numbers each time.

## Checking a change

```
cd /home/glennsong/Projects/raytracer-clang/.claude/worktrees/buildings
cmake --build build-viewer
./build-viewer/brain_tests
./build-viewer/driving_tests
./build-viewer/lanes_tests
RT_TEST_FILTER=level_brain_lab ./build-viewer/level_tests
RT_TEST_FILTER=level_traffic_lab_first_cars ./build-viewer/level_tests
RT_TEST_FILTER=level_traffic_lab_v2_drives ./build-viewer/level_tests
./build-viewer/rt_bake --status assets/levels/traffic_lab.json
```

`run_tests` filters that cover this area: `trip_plan`, `goal`, `role`, `_lua`, `parking`, `schedule`,
`tolerance`, `lane_graph`, `control`, `nav`, `route`, `sign`, `street`, `map`, `signal`.

## Watching it

```
cd /home/glennsong/Projects/raytracer-clang/.claude/worktrees/buildings
cmake --build build-viewer --target viewer
./build-viewer/viewer assets/levels/brain_lab.json --play
```

Six residents in three households (ADR-0443): ana and ben share a car at `home`; cat, dan (a board) and eve
share one at `west`; fay (a scooter, no car) lives at `shop`. Their day tables run from the start, and a
household's members leave within seconds of each other, so they contend for the car. `<who>` is a name or a
number; a verb given none acts on the followed resident (the one the camera is on).

Console:
- `residents?` everybody in a few words; `brain? ana` one resident in full; `markers?`
- `brain follow` the camera on the next resident; `brain follow ben`; `brain follow off`
- `brain day all off` stops every day table (then send people yourself); `brain day ana on`
- `goto ana work`, then a second later `goto ben ramp`: the contended car (ben gets it; ana walks)
- `brain trait ana has_board on`
- the player and the car: walk up to a car with its driver in it, look at the driver's seat (Tab moves the
  focus) and press E; `brain? <who>` then says "taken by the player" and the driver walks on
- a director: `possess walker <x> <z>` (the nearest walker to that point), `walk_to <x> <z>`, `board`,
  `alight`, `release`
- `sim speed 8`

Looking at a resident or a household's car (the middle of the view) shows its line at the foot of the screen.
`python3 tools/brain_lab_probe.py --scenario multi` plays the contended car and the player taking it, and
takes pictures into `~/Pictures/Raytracer/brain/` (names beginning `multi_`).
