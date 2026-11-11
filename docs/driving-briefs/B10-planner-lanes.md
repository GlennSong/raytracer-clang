# B10 — The planner places bus lanes, bike lanes and parking

**Read first:** design 5.4 and section 18 answers 2, 8 and 9. `src/apps/citysim/city_bus.h` (`BusNetwork`).
ADR-0092 (the city planner), ADR-0111 (bus lines on the map). `src/engine/procgen/city/plan/`.

## The goal

Bus lanes, bike lanes and kerbside parking become things a city's brief can ask for. The planner decides where they
go, as routes, before blocks and lots are laid out. With nothing asked for, a city's roads come out exactly as today.

## Decisions already made (Glenn, 2026-11-11)

- **Bus lanes take an existing lane:** the kerb-side travel lane, red, for buses and taxis. Kerbs don't move.
- **Bike lanes and kerbside parking widen the road.** The street is planned wider, and blocks and lots are laid out
  round it.
- **More parking overall.** The planner counts spaces against the people expected to arrive by car.
- **Everything is tunable in Lua.**

## The steps, in order

1. **Bus lines move into the planner.** Today the city sim works them out from finished streets (`BusNetwork`). The
   working-out becomes one shared function the planner calls; the result is written into the city plan; the city sim
   reads the lines from the plan and no longer works them out. One source. With this step alone, nothing visible
   changes, and the bus tests must still pass.
2. **Bus lanes.** Along each line, on stretches with two or more travel lanes each way, the kerb-side lane's kind
   becomes `bus`. Short gaps between such stretches are not filled: a bus lane that appears for one block is noise,
   so a stretch shorter than a length in the settings is skipped.
3. **A bike network.** Chosen as a connected set of streets linking homes, the centre, schools and stations:
   shortest ways between those places over collectors and quieter arterials, preferring gentle slopes. Each chosen
   street gets a `bike` lane each way. The test of "connected" is literal: from any bike lane you can reach any
   other without leaving the network.
4. **Parking.** For each district, spaces needed = people expected to arrive by car × a share from the settings.
   Streets get kerbside parking lanes until the need is met; what is still short becomes car parks (a lot use that
   already exists).
5. **Wider cross-sections.** Each road class has a table of cross-sections in Lua: plain, with parking, with a bike
   lane, with both. The planner picks one per street *before* block layout, so the kerb lines the blocks are built
   from are already the wide ones.

## Traps

- **Step 5 changes block shapes,** so it changes lots and buildings. Turning these settings on in an existing level
  is a lots rebuild and a lanes rebuild, and the level suite after. Agree with Glenn before doing it to any shipped
  level. Build and judge it in a test city first.
- **With the settings off, output must be identical to before.** Test that first and keep the test.
- **Narrow streets:** a local street may have no room for a bike lane even widened to its class limit. The network
  then routes round it; it does not squeeze one in.
- **Bike lane beside parking:** put the bike lane between the parked cars and the travel lane (the American
  standard). The cross-section table says so; the planner doesn't choose.
- **Bus lines that turn at the kerb:** a bus lane ends before the junction where the bus turns across lanes. The
  line's turns are known, so end the lane there.

## Not in this task

How the lanes are built and painted (B9, B8). How people choose to travel (group F).

## Done when

- With the settings off: the planned city is identical to today's.
- A test city with them on: the map shows the bus lanes on the bus lines and a connected bike network; the parking
  count meets the need in every district; the lanes builder builds it with no refusals that the settings caused.
- The city sim's buses run the planner's lines, and the bus tests pass.

## Show Glenn

The test city's map (SVG) with bus lanes in red, bike lanes in green and parking marked, beside the same city with
the settings off.
