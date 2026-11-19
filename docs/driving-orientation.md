# Driving: orientation for a helper

Start here before touching `src/apps/driving/` or the city sim's driving side (`src/apps/citysim/city_driving.*`,
`city_near.*`, `city_car.*`, `city_crowd.*`, `city_brain.*`). This page says where things are, who reads what and the
rules that apply. The *why* is in `docs/driving-model-design.md`, `docs/two-drivers-meeting-design.md` and the ADRs it
names (ADR-0398 to ADR-0416 for the driving library, ADR-0440 to ADR-0452 for the brain, the near tier, things in the
way and crosswalks, ADR-0453 for one kind of car, ADR-0455 for the brain's car facts on the car and one way to take a
car, ADR-0456 for one signal clock, ADR-0457 for river_town on the new driving, ADR-0459 for deleting the old
driving's possession, passenger seat and trip lab, ADR-0460 for its buses and cabs, ADR-0461 for its cars outside the
sim, ADR-0462 for the sim's own way in to them and their tests, ADR-0463 for the sim's own car code). The roads it drives
on are described in `docs/roads-orientation.md`. Written 2026-11-17, after ADR-0452; section 3 and 4 brought up to
ADR-0453 on 2026-11-18, to ADR-0455 the same day, and to ADR-0456, ADR-0457, ADR-0459, ADR-0461, ADR-0462 and ADR-0463
after it. Anything marked
*(unsure)* was not
checked in the code.

**Which levels the driving drives (ADR-0457, ADR-0461).** Every level that names a rule set (`citysim.drivingRules`):
traffic_lab, brain_lab, brain_crowd and river_town. There is one driving since ADR-0461: the `citysim.driving` key that
chose between the old and the new is gone (ignored if a level still sets it), and a level with no rule set has no
driving and no traffic at all (the old levels, until they get the new driving's data). river_town's cars are all a
crowd's households' (`citysim.crowd = "crowd_river_town.lua"`, the same format as crowd.lua; `cars: 0`); the old sim
still walks its other people on foot. A sim-only citysim field (cars, a crowd, the rule set, the near tier) never
re-bakes a level: the lots key hashes only the fields the lot pass reads (`src/engine/lot_citysim.h`).

**One signal clock (ADR-0456).** The traffic lights are the new driving's alone: `signal_programme.h`, pure functions of
`Driving::clockSeconds()`. The drawn signal heads show them (`headAt`, read by `CityRenderSystem` for each head the loader
placed, by the arm the head is attached to), the walkers at the model's crosswalks wait on its walk signals
(`walkAtCrosswalk`), and the cars obey them. The old sim's own clock (`SignalController`, `traffic_signal.*`,
`signals.lua`) is deleted. Where the new driving does not drive, every head is dark and no walker waits for a light.

**Deleting the old driving (v1), piece C, in three parts (#306).** C1 is done (ADR-0459): possession drives no car of its
own any more (`possess walker`, `walk_to`, `board`/`alight` a household's car of the new driving; `possess car`,
`drive_to` and the bus `ride` are gone), the player's passenger seat on a bus, cab or old car
(`CityPlayerTransitSystem`, the `ride` verb) is gone, and the one-agent trip lab (`CityTripSystem`, the `trip` verbs, Y
/ I / U, agent_lab's purpose) is gone, the agent brain's trips having superseded it; its `stepsOffThePaths` is the
brain's now (`city_brain.cpp`). C2 is done (ADR-0460): no buses and no cabs. The sim's buses, cabs, depots, bus stops,
riders, hailing, bus commuters and "towns" (which were the bus network's street networks) are gone, with their drawing
(destination signs, doors, riders in seats, stop furniture, `depots?`) and the `busRoutes`/`buses`/`taxiFraction`/
`hail*`/`busService`/`busCommuteShare` level keys (ignored if a level still sets them). Left: `BusNetwork`'s route
planner alone (`city_bus.h`, for the island planner's map lines), the fleet's bus and taxi bodies, the lot pass's bus
depot yards (built, drawn, unused). C3 is split in two (ADR-0461). **C3a is done:** no level makes an old car any
more (the sim is built with no drivers; the level's `cars` are the driving's, made only where a rule set is named), and
everything outside the sim that served the old cars is gone: their drawing (agent-row cars, parked rows, their lamps,
the four-wheel fit memory, the far band's two-box cars, the old bays' paint), the ghost-backed physical tier
(`possessTier`, `spotClear`, `physicalCars`), their promotion with E, `carsNear`'s old-sim branch, the lane match
(`lane_map.*`, `LevelLanes`, `TrafficLane`, `realLanes`), `car_lamps.h`, `RideBook` and `Dispatch`, the parked fire
engines, `giveDriversRole`/`driverRole`/`lab_driver` and the `citysim.driving` flag. C3b is split in two (ADR-0462).
**C3b-1 is done:** `CitySim::build(graph, pedCount, seed)` takes no driver count, so nothing can make an old driver; every
test that built old drivers or measured old cars is deleted (or now builds walkers, where it was about days, places,
tiers or walking); and the old cars' ways in from outside the stepper are gone: `addStationedVehicle`, `ejectDriver`,
`releaseDriver`, `setGarages` (with the loader's garage specs, `CitySimConfig::garages`), `parkingBays()` and `parking?`,
`moveHome` and `citysim.homeNear`, `setFleetLaunch`, `setAgentBodyLag`, `parkedNear`, `walkersInRoad`, the census's
driver counts, and the `Mode::Driver` readers in possession, spectate (`SpectateFraming`), the poses, the near tier and
the debug panel. **C3b-2 is done (ADR-0463), and with it the old driving is gone:** an `Agent` is a person on foot
(no `Agent::Mode`, no `archetype`, no car fields), there is no `SimVehicle`, and the stepper's car branches, the bays,
the crash and hold passes, the cars' perception, `traffic_rules.*` (`approachStop` is `city_sim.h`'s) and
`engine/ai/idm.h` are deleted. The population cache is format 13. The player still takes any car with E
(`CitySim::playerTakesCar`).

**One car, one representation (ADR-0453).** Every car the new driving moves is a car entity (`city_car.h`): a
household's (the brain sends it), one that drives round for ever (`CityCarRoams`: the hand-over sends it; the traffic
lab's 300, or `cars` on any level that names a rule set), or a test bed's (`CityNearSystem::labCar`). It is the only
kind of car since ADR-0461 (C3a): no level makes a sim agent in `Mode::Driver` or a `SimVehicle` row any more (their
code is dead until C3b deletes it). Look cars up through the entities (`World::each<CityCarBody, ...>`,
`carsNear(world, ...)`), never through `CitySim::agents()`.

**One pose per car per fixed step (ADR-0454).** Where a car entity's body is this step, on its wheels on the deck, is
`CityCarPose`, set once every fixed step after `sim.step` by the sim's pose module (`city_poses.h` `CityPoses`, held by
`CitySim::poses()`: `stepCarPoses`, and `poseCarNow` for a car made between steps). The four-wheel fit, its tilt
filter, the ground it stands on, the models' wheels and the walkers' pose live there too; `CityRenderSystem` only draws. Its kinematic box (`CityPhysicsSystem`), the near tier's switch into
physics (`toPhysics`), the player's seats in it (`carEntitySeatsOf`) and the drawing all read it; the drawing's
blend of one bake to the next is drawing only. The Transform stays the sim's record (plan place at the tick's end,
PrevTransform at its start). Never read a car's place for anything but drawing from the car instance groups.

**Every fact about a car is on its entity (ADR-0455).** The brain keeps no record of a household car of its own: what
it knows (where the car stands as a place it plans with, which resident has it, its slide between kerb space and
lane, a trip wanted) is the car's `CityCarKept`, beside `CityCarLink` (household, driver, `away`: the player has it).
The brain holds only the entities, found from a car's number at once. **The player takes any car one way,
`CitySim::playerTakesCar`** (the key and the near tier both call it; it tells the brain for a household's car and the
hand-over for a roaming one), **and gives a household's car back one way, `CitySim::playerLeftCar`** (the near tier,
once the car has stopped). Never call the brain's or the hand-over's `takenByPlayer` from a new place.

Words: a **path** is one curve a car can be on, a **lane** (along a street) or a **connector** (through a junction).
The **lane level** is the table of paths the road model builds (`RoadModel::laneLevel`). A **crossing point** is a
stretch where two paths' bodies (or a path and a crosswalk) could touch. A **fact** is a named value a driver notices
("the car ahead is 12 m away"); a **rule** turns facts into **answers** (a place to stop by, a top speed, an intention,
an action). A **rule set** is one Lua file of rules and numbers (`assets/scripts/rules/lab.lua`). A **lane car** is
moved by the driving along its lane; a **physics car** (a **near** car) is a Jolt car the host carries out the driver's
wishes with.

## 1. The rules that apply here

- **One road model** (AGENTS.md, "One Road Model"). The driving never works out a road fact: junctions, arms, kerbs,
  pavements, crosswalks, stop lines, which path crosses what. It asks the road model (`Driving(const RoadModel*)`;
  `RoadModel::controlAt`, `turnOf`, `postedLimit`, `facing`, `regulationsAlong`, `crosswalk`, `crossersOf`, ...). A
  question the model cannot answer is added to the model first. The one angle it still measures is
  `right_of_way.h kAlongsideDot` (two approaches running alongside), listed there.
- **Nothing arbitrates** (design section 2). Each driver decides from what it perceives and the right-of-way table (the
  law as a driver knows it); nothing hands out turns. No timeouts, no deadlock breakers: a long wait is fixed in the
  drivers' habits or the roads. The wait-for graph and the measurements are counted in tests and lab tools only.
- **Kinds are data.** A new control, device, turn kind or rule is a row of the rule set; no C++ names a kind.
- **Runs repeat.** Every chance is a hash of the person, the tick and the question; nothing walks an unordered
  container where order matters. Pinned hashes are re-pinned only with old, new and why.
- No `auto` but lambdas; compare computed numbers with `approxEqual` and named tolerances.

## 2. `src/apps/driving/` file by file

| File | What it holds |
|---|---|
| `driving.h/.cpp` | `Driving`, the front door: given the road model, steps traffic a tick at a time (plan, settle, move), takes trips, hands back events. Holds the cars, trips, the right-of-way table, the light clock, the things on the road and the crosswalk people of the tick (ADR-0452). |
| `traffic_state.h/.cpp` | A car's record (`Car`: path, along, offset, speed, acceleration, trail) and every path's cars (`PathLists`); `carPose` reads where it is off the path's curve. |
| `act.h/.cpp` | The arithmetic of speeding up, following and stopping: the Intelligent Driver Model and the "could still stop" limit. |
| `notice.h/.cpp` | Facts: the registry, the built-in facts, and `DriverView` (what a driver can see when it decides). |
| `decide.h/.cpp` | Rules to answers, by fixed arithmetic (nearest stop, lowest top speed). |
| `rule_set.h/.cpp`, `scripting/rule_set_lua.*`, `scripting/rules_lua.*`, `scripting/lua_table_reader.h` | The rule set as plain structs, read from Lua once at load: ranks, devices, controls (settle questions, arrows, `protected_when`, `arrow_when`), numbers with units and sources, personality traits. |
| `road_rules.h/.cpp` | Binds the road model's names (controls, regulation kinds, classes) to the rule set's rows, once at load. |
| `right_of_way.h/.cpp` | Who gives way to whom at every meeting of two turns, per phase; the phases of each junction's lights (protected and arrow turns); the walk signals of the model's crosswalks under lights (`WalkSignalRules`, keyed on `ModelCrosswalk` ids, crossed by `RoadModel::crossersOf`). |
| `signal_programme.h/.cpp` | The lights' clock, the only one (ADR-0456): yellow, all-red and green timing, the green wave, `lampAt` / `lampShownAt` (a green arrow), `headAt` (what a drawn head at one arm shows: its face and its arrow), `walkAt` / `walkAtCrosswalk` (WALK, FLASHING DON'T WALK, DON'T WALK), and `checkSignalProgrammes`, the safety check. Pure functions of the clock. |
| `announce.h/.cpp` | The board of announcements: each car says when it will be on each crossing point ahead. |
| `conflict.h/.cpp` | The one conflict resolver over the board (being replaced by judgement, design sitting 6). |
| `give_way.h/.cpp` | A driver's judgement at the junctions ahead: gaps, lights, the dilemma at a yellow. |
| `junction_plan.h/.cpp` | A driver's plan for each junction ahead: its connector, where its body can wait. |
| `room.h/.cpp` | Room beyond a junction: nobody enters a box it cannot leave. |
| `things.h/.cpp` | What is in my way (ADR-0451): `RoadThing` (anything on the road the driving does not drive, with the crosswalk a person stands at, ADR-0452), `findThingAhead`, `findCrosswalkAhead`, their facts (`thing_*`, `person_at_crosswalk`, `crosswalk_stop`), the horn, swerve and another-way answers. |
| `route.h/.cpp` | The router over paths. |
| `trip_places.h/.cpp` | Where a host's aimless trips can begin and end. |
| `vehicle_class.h/.cpp`, `scripting/vehicle_class_lua.*` | The vehicle classes, derived from the vehicle data. |
| `measure.h/.cpp` | The measurements table (tests and lab tools only). |
| `trace.h/.cpp` | Each car's recent decisions and its driver card. |
| `wait_graph.h/.cpp` | Who waits for whom; rings of waits. |

## 3. The city sim's driving side

| File | What it holds |
|---|---|
| `citysim/city_driving.h/.cpp` | `DrivingHost`: runs the driving's tick; puts its own cars (those that drive round for ever) down and sends them on trip after trip; copies every car entity's pose from the driving; answers `driving?`. |
| `citysim/city_car.h/.cpp` | Every car the new driving moves is one ECS entity (ADR-0444, ADR-0453): pose, body, household, driver, `away`; a lane component while the driving has it; `CityCarRoams` on a car with nobody in it that drives round for ever (the traffic lab's); `CityCarKept`, the brain's facts, on a household's car (ADR-0455); `carsNear`; the one take and give-back (`CitySim::playerTakesCar`, `playerLeftCar`). |
| `citysim/city_near.h/.cpp` | The near tier (ADR-0447..0450): the cars nearest the player as Jolt cars carrying out their drivers' intent, switched on one entity; physics sweeps answer "what is in my way" for them (ADR-0451). |
| `citysim/city_crowd.h/.cpp` | A crowd of residents made from the road model and `crowd.lua` (ADR-0446): places beside streets, households, cars, days. |
| `citysim/city_brain.h/.cpp` | The residents with the agent brain: what they want, their trips, carried out with the sim's walking and the new driving. |
| `citysim/city_sim.h/.cpp` (parts) | The old sim, which still walks every person on foot on the street map: `thingsForDriving` writes them into the driving's things each tick; `crossingAhead`, `gapToCross`, `jaywalks` make them wait at a crosswalk's kerb end and cross (ADR-0452). |
| `citysim/city_render.h/.cpp` (parts) | The signal heads (ADR-0456): one `SignalHead` per post the loader placed (`StreetFurniture::Signal`, which carries the model arm the head is attached to), tied to the new driving's lane junction there (`tieSignalHeads`, by `RoadModel::laneJunctionOf`), lit each bake from `headAt` (the face as the post instance's number, `headFaceNumber`; the arrow as a lens beside it). |

## 4. Who reads what

- **The road model** (`LevelRoadModel` from the loader; tests build one in `tests/driving_test_roads.h`): the driving
  holds it and asks it. Crosswalks: `RoadModel::crosswalks`, `crosswalk(id)`, `crossersOf(id)`, `crosswalkOfLane(w)`,
  `crosswalksOnWalk(junction, armIn, armOut)`, `armReachedAt` (the arm a street-map link arrives by).
- **The rule set** (`rules/lab.lua`): the driving, at load. People's crossing habits are not driving rules: they are
  `assets/scripts/people.lua` `people.crossing`, read into the city sim's `PeopleCatalog`.
- **The things on the road**: the city sim writes them (`setThingsOnRoad`) before each tick; lane cars look along
  their way for them, near cars get physics' answer (`seeAhead`).
- **Tests**: `driving_tests` (hand-made roads in `tests/driving_test_roads.h`, stated on a small model:
  `stateStreet`, `stateControl`, `stateTurn`, `stateCrosswalk` via `TestRoadSpec::crosswalks`), `level_tests`
  (`level_traffic_lab_v2_*`, `level_brain_lab_*`, `level_brain_crowd_*`), `physics_tests` (`near_tier_*`).

## 5. Checking a change

Build with `cmake --build build-viewer --target driving_tests level_tests physics_tests viewer`. Filters are substrings:
- `./build-viewer/driving_tests` (about 200 cases, a minute or two); `RT_TEST_FILTER=walks_`, `arrows_`, `crosswalks_`.
- `RT_TEST_FILTER=signal_heads ./build-viewer/level_tests`: every drawn head on traffic_lab against `lampShownAt`.
- `RT_TEST_FILTER=brain_crowd ./build-viewer/level_tests`: 30 simulated minutes of 350 residents; per-minute lines
  include the crosswalk counts. Pins its hash.
- `RT_TEST_FILTER=river_town_crowd ./build-viewer/level_tests`: river_town's 630 residents (`crowd_river_town.lua`), 30
  minutes; per-minute lines, why the cars standing at the end stand, what is in the longest-standing car's way, tick
  cost and memory. Pins its hash. (The two crowd tests share `tests/crowd_test_rig.h`.)
- `RT_TEST_FILTER=near_tier ./build-viewer/physics_tests`: contacts and hits on walkers.
- `RT_TEST_FILTER=v2_drives ./build-viewer/level_tests`: traffic_lab, 300 cars, 5 minutes; two known failures.
- Viewer: `tools/rtctl.py` launches `build-viewer/viewer assets/levels/brain_crowd.json`; `tools/brain_lab_probe.py`
  drives it (`brain labels off`, `near marks on`, `brain follow rN`, `sim speed 16`, `near?`, `things?`). Stop only the
  viewers you started, by PID.

## 6. Known faults and open work

- The old sim's people walk the street map's kerb-side line, not a pavement of the model, and cross through the
  junction's corner blend; their places waiting at a crosswalk are where their route comes level with its kerb end.
- The sim's `junctionRadius` still measures the roads from the street map (`nodeBoxRadius_`, the widest street at a
  node, worked out at build: a road fact computed outside `roads/`). Its car readers went with ADR-0463; the walkers read
  it (`onStreet`, `stopLineBack`, the place a walker stands back from a junction), so taking it away moves the walkers
  (and the pinned hashes): that part is task 4's (people route on the road model), not a deletion.
- A walker's place standing at a crosswalk's kerb end (`refreshPose`'s ModelCrosswalk::kerbWait step, ADR-0452) is
  never reached: `refreshPose` returns for every walker before it (an explicit `return` since ADR-0463, commented).
  Found in ADR-0461, not fixed (it moves the walkers and every pinned hash); likely part of brain_crowd's
  kerb-end-share failure, so task 4's.
- A walker at a junction the model's crosswalks do not rule (no crosswalk with pavements there, or a junction of a city
  the new driving does not drive) waits for nothing: no light, and since ADR-0463 no gap either (it judged the gap in the
  old driving's cars, which were gone already).
- The signal heads have no lit walk signal: the pole's walk panel (`street_kit.cpp`) is painted, and which crosswalk a
  panel would serve is stated nowhere (ADR-0456).
- A walk round a junction that crosses two arms (a far turn on foot) is judged one crosswalk at a time, waiting at the
  corner between.
- Since ADR-0453 the traffic lab's cars are entities with nobody in them: `spectate` follows them after the agents,
  `CitySim::census(world)` counts them, and the key takes one (`carEntityNumbered`, `CitySim::playerTakesCar`). The
  near test bed's cars drive as one person number (`kLabPerson`) where the roaming cars drive as their own numbers, and
  have no owner for `playerTakesCar` to tell. A roaming car the player took is never handed back to the hand-over.
- `carEntityNumbered` walks every car entity (the key uses it, not per tick).
- A household car given back mid-street on brain_lab where it was taken 30 s into a drive: the household plans with it
  and the driving refuses every trip from where it was put in the kerb lane, retried every minute (ADR-0455, not
  looked into).
- traffic_lab has no near tier: switched on (brain_crowd's `citysim.near`), no two Jolt cars touch, but a few near cars
  leave their carriageway on some streets and stand off their lane for minutes. How it was measured is in ADR-0453.
- river_town (ADR-0457, measured, not filed yet):
  - a queue of about 40 residents waiting at crosswalk 123's kerb end (lane junction 29, near -142 482) reaches back
    over lane 388, and car 91 stands behind it ("something in the way") from minute 5 to the end; the people at the
    front never go, not diagnosed;
  - its road model fails its own lane-graph check (lane 1183 and crosswalk 481's meeting recorded twice), so it is never
    saved and is rebuilt at every load (about 70 s);
  - a car can reach only 89.6% of its lane paths, and 116 turns are refused;
  - 8 residents stand in `no_way` all run, planning again every minute.
