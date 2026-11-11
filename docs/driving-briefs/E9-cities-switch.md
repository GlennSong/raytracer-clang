# E9 — Cities move to the new model; the old one is deleted

**Read first:** the plan's build order, phase 9. Design 14 (drawbacks 3 and 8). Brief A2's table. The skill
`rt-bake-cleanup` for rebuilding and pruning bakes. `docs/ci-plan.md`.

## The goal

Each city runs on the new driving model with its roads rebuilt, its measurements passing and its tests green. Then
the old driving code and its tests are removed.

This is mostly careful process, not design. It is graded hard because it touches every city and deletes about 3,400
lines from `city_sim.cpp`.

## The order

One city at a time: **metro, then river_town, then the island.** Do not start the next until the one before is
signed off by Glenn.

For each city:

1. **Rebuild its roads** with connectors, crossing points, the control plan, signs and paint. This is a lanes
   rebuild and takes hours; get Glenn's OK first, and keep heavy builds on disk, not in `/tmp`.
2. **Read the builder's refusal report.** Every refused turn is a place cars can't go. Sort them: fix the planner's
   intent, fix the builder, or accept and note. Don't loosen a check to clear the list.
3. **Check the builder's guarantees** (design 5.3) pass for the whole city.
4. **Turn on `citysim.driving = "v2"`** for that level.
5. **Run the measurements** for a full day of city time. Compare with the lab's. Deadlocks: none. Gridlock: counted,
   target none. Parking: enough in every district.
6. **Run the city's own tests,** then the level suite. Each of the roughly 85 old driving tests has an entry in the
   list from phase 2: rewritten for the new model, or to be retired.
7. **Cost:** frame time and memory against the old model in the same place, written down.
8. **Glenn drives and walks it.** A list of what he sees goes into the issue.
9. Offer the bake prune.

## Deleting the old model

Only after all three cities are signed off:

- Remove the per-level switch; the new model is the only one.
- Remove the old model's code, using A2's table as the list of what belongs to it: the driving fields on `Agent`, the
  driver states in `Agent::State`, the lane map built to match links to lanes, the pull-out easing, the ghost and
  tether, the timers that unstick cars (`holdTimer` and the like), the far tier's guessed-speed stepping.
- Remove the retired tests. Keep none "just in case".
- The interface from A2 stays, now with one thing behind it.
- Update `docs/` (the "How the engine works" pages on the city sim), `docs/capabilities.md`, and the ADRs that the
  new model replaces (mark them superseded; don't delete them).

## Traps

- **Check before regenerating.** Run `git status` on every output path before a bulk rebuild; an untracked or
  changed file there is Glenn's work.
- **Don't re-seed a failing level** to make it pass. A seed that breaks is a bug in the generator.
- **The island is large:** the lane graph's memory, the junction plans' load time and the middle-distance cost were
  all measured in the lab only. Measure them here before switching, and stop if they don't fit.
- **Things that quietly depended on the old model:** buses' stops, cabs' pickups, the map's traffic picture, sounds
  by car state, the census. A2's table is the checklist.
- **Push CI against nightly:** heavy levels are on the nightly list. Don't move them.

## Done when

- All three cities run the new model with the plan's measurements table passing.
- The old code and tests are gone; a search for the old fields finds nothing.
- The level suite and nightly are green.
- The plan's baseline comparison (166 junction stops, 78 wide swings, 3 spins, in the lab) is written up against the
  final numbers.

## Show Glenn

Each city, in the editor, at rush hour. And the before-and-after numbers.
