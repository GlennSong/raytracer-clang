# A2 — Separate everything that reads the old car state

**Read first:** design 3 and 3.1. `src/apps/citysim/city_sim.h` lines 46–390 (the `Agent` struct) and 388–450
(`SimVehicle`). ADR-0395 to ADR-0397 in `docs/decisions.md`.

## The goal

Today, drawing, the physics tier, buses, cabs, sound, the map and possession all read a car's position and state
straight out of `Agent` fields. The new model keeps that state somewhere else. Before it can drive anything, every
reader has to ask through one small interface that either model can answer. **This task changes no behaviour.**

## What is there now

A rough count of lines that read the driving fields (`link`, lane, `param`, `speed`, `heading`), by file:
`city_sim.cpp` 237, `city_render.cpp` 39, `city_physics.cpp` 13, `city_walkers.cpp` 10, `city_player_transit.cpp` 10,
`city_possess.cpp` 9. Sound, the map tool, spectating and the bus code read some too. The count is a starting point,
not the list.

## Decisions already made

1. **Two halves, two commits.** First the list, changing nothing. Then the interface.
2. **The list** goes in the ADR as a table: file, function, which fields it reads, what it uses them for, and whether
   it only reads or also writes. Readers outside `city_sim.cpp` are the ones this task converts. Readers inside
   `city_sim.cpp` that belong to the old model itself are left alone; they are deleted with it in E9.
3. **The interface is a read-only "car view"**, asked of `CitySim` by vehicle:

   | It answers | Notes |
   |---|---|
   | Position, direction and height, at the start and end of the current tick | Drawing shows cars one tick behind, blended between the two (`tickFromPos`). The view must give both, or the blend. |
   | Speed and acceleration | |
   | Lamps: braking, indicating which side, reversing | Today's indicators are read from how the road bends. Keep that inside the old model's answer. |
   | Front wheel angle | For drawing. |
   | What it's doing, as a short text | Replaces reads of `Agent::State` for driving. Walking states are untouched. |
   | Size and class | From the vehicle. |
   | Who is driving | An agent index or none. |

4. **The old model answers the view by reading the same fields the reader used to read**, so every number is
   identical. The pull-out easing (`pullOffset`, `pullYawOffset`, `pullS`) stays inside the old model's answer; the
   new model will never need it.
5. **Writers are listed, not converted.** Possession, the physics tier and the player's transit code *write* into
   agents (`released`, `directed`, `bodyLag`, `setBodyObstacles`). List them in the table. They are dealt with in E5
   and E8.
6. **No fields move off `Agent` in this task.** That is E9.

## Traps

- A reader that does arithmetic on `link` and `param` to find the next junction is using the old model's road
  layout, not just a car's position. Put it in the list under "needs the old roads" and leave it; don't invent a
  view question for it.
- `carGrid_` (the walkers' check for cars) is built from agent positions each tick. It should be built from the
  view, so the new model's cars appear in it later for free.
- Converting a reader can change floating-point results if the order of operations changes. Keep each expression
  as it was.

## Not in this task

Any new behaviour. The `driving` library. Removing the ghost car used by possession.

## Done when

- The table is in the ADR.
- A search finds no reads of the driving fields outside `city_sim.cpp`, `city_sim.h` and the old model's own files.
- The lab baseline (ADR-0395's measurements: junction stops, wide swings, spins) is **identical**, and a two-minute
  lab run produces the same state as before the change. If a state hash for the sim doesn't exist, add one as part
  of this task; C6 needs it too.
- All existing tests pass unchanged.

## Show Glenn

Nothing to see, which is the point. Report the table and the identical numbers.
