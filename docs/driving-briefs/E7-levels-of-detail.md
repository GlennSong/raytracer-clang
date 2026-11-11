# E7 — Middle distance, out of sight, arriving in an area

**Read first:** design 12, 3.1 (levels are the host's decision; today's tiers), 3.3, section 18 answers 1 and 7.
`city_sim.h` lines 60–135 (tiers, `rememberUntil`, `pinned`) and `scheduleSnapshot`. The plan's part 4.

## The goal

The same people on the same trips under the same rules at every distance, at a cost the island can bear. A car
never jumps, and nobody is lost or doubled, when the level changes.

## Decisions already made

**Middle distance is the near model looked at less often.** Same state, same rules, same personality (Glenn: yes).
What is dropped comes from the rule set's `detail` table: attention slips, would-like-to lane changes, the horn, and
the facts only those use. Decide twice a second, move ten times a second. Today's far tier (a guessed speed, lights
ignored) is not used under the new model.

**The city sim says which level; the driving library carries it out** and may answer "not yet" for a car part-way
through a lane change or inside a junction.

**Going out of sight** (allowed anywhere once not inside a junction): the trip ends in the driving library; the
person's schedule carries on from how far along the route the car was, in time.

**Where an out-of-sight traveller is** comes from one shared function: route, time under way, and the table of how
long each section takes → a section and a distance along. **The far-away crowd drawing uses the same function.**
Otherwise a car seen moving in the distance pops to somewhere else when it becomes real.

**Arriving in an area (out of sight → middle):**

1. The function gives the place the schedule says.
2. If that is on a connector, move back to the lane before it.
3. Look for a spot at or behind that place with a safe gap ahead and behind at the speed of the cars there. Search
   back up to a distance from the `detail` table.
4. Found: start a trip there, at the speed of the car ahead (or the limit), in a lane that leads to the next turn.
5. Not found: the person stays out of sight; try again when their schedule has moved them on. They are not dropped.

**Queues at lights are not placed; they form.** People arriving are placed in a ring *outside* what can be seen
(wake distance greater than view distance by a margin), so they drive in real time for some seconds before they can
be seen, and queues form by the rules. This replaces "run a few seconds of simulation before the area is shown":
the lights are a function of the one clock, so an area can't be run ahead on its own. When the player moves faster
than the margin allows (flying), placement is spread over several ticks and less tidy queues are accepted.

**Parked cars** (Glenn: "close to where the people are"): when an area wakes, each person there who owns a car and is
not travelling has it placed in the nearest free space to where they are, taken in order of person id. When an area
goes out of sight, parked cars of people who are *also* out of sight are forgotten; they are placed afresh next time.
Cars of people being followed, or seen lately, stay.

**How long sections take.** One table: section × hour of day. It starts at length ÷ limit. Each simulated car that
leaves a section adds its time to a running average. Updates are gathered and applied in a fixed order (brief C12).
Times are in driving seconds; the schedule converts when the city clock runs faster than real time.

**The arrival event** carries how long the trip took, for the person's own memory (design 3.3).

## Traps

- **`rememberUntil` and `pinned`** already keep some people exact after the player leaves. They stay at middle
  distance; nothing new is needed, but test that a followed person is never put out of sight.
- **A space taken twice:** two people waking in the same tick near one free space. Order by person id and take
  spaces one at a time.
- **A person out of sight who "arrives":** their car isn't placed anywhere until someone looks.
- **A trip started part-way** needs its remembered facts started sensibly: not "stopped at the line" for a line it
  hasn't reached; no room grants.
- **The table and sameness:** what the table has learned depends on where the player has been. Tests fix the
  player's path. Two play-throughs will differ slightly; that is accepted (design 14, drawback 10).

## Done when

1. A thousand cycles of an area going out of sight and coming back: the number of people, and of cars owned, is
   unchanged; nobody appears twice.
2. No car is ever placed overlapping another, on a connector or inside a junction.
3. Near to middle and back: position, speed and route are unchanged across the change.
4. The place the far-away crowd drew a traveller and the place it is put when it becomes real differ by less than a
   car's length along its route.
5. From the lab's hill viewpoint, middle-distance traffic for ten minutes: the stop-start count and the jump count
   in the measurements are zero, and queues stand at red lights.
6. Cost per 1,000 drivers at middle distance, recorded, with a note on whether the island's numbers fit on the CPU.

## Show Glenn

The island from a hilltop at rush hour, then driving down into it: nothing pops, queues are where they should be, and
the same cars seen from above are the ones met on the street.
