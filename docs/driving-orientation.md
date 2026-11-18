# Driving: orientation for a helper

Start here before touching `src/apps/driving/` or the city sim's driving side (`src/apps/citysim/city_driving.*`,
`city_near.*`, `city_car.*`, `city_crowd.*`, `city_brain.*`). This page says where things are, who reads what and the
rules that apply. The *why* is in `docs/driving-model-design.md`, `docs/two-drivers-meeting-design.md` and the ADRs it
names (ADR-0398 to ADR-0416 for the driving library, ADR-0440 to ADR-0452 for the brain, the near tier, things in the
way and crosswalks, ADR-0453 for one kind of car). The roads it drives on are described in `docs/roads-orientation.md`.
Written 2026-11-17, after ADR-0452; section 3 and 4 brought up to ADR-0453 on 2026-11-18. Anything marked *(unsure)* was
not checked in the code.

**One car, one representation (ADR-0453).** Every car the new driving moves is a car entity (`city_car.h`): a
household's (the brain sends it), one that drives round for ever (`CityCarRoams`: the hand-over sends it; the traffic
lab's 300, or `cars` on any v2 level), or a test bed's (`CityNearSystem::labCar`). No sim agent in `Mode::Driver` and
no `SimVehicle` row stands for a car of the new driving; those are the old driving's (v1) only. Look cars up through
the entities (`World::each<CityCarBody, ...>`, `carsNear`), never through `CitySim::agents()`.

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
| `signal_programme.h/.cpp` | The lights' clock: yellow, all-red and green timing, the green wave, `lampAt` / `lampShownAt` (a green arrow), `walkAt` / `walkAtCrosswalk` (WALK, FLASHING DON'T WALK, DON'T WALK), and `checkSignalProgrammes`, the safety check. Pure functions of the clock. |
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
| `citysim/city_car.h/.cpp` | Every car the new driving moves is one ECS entity (ADR-0444, ADR-0453): pose, body, household, driver; a lane component while the driving has it; `CityCarRoams` on a car with nobody in it that drives round for ever (the traffic lab's); `carsNear`. |
| `citysim/city_near.h/.cpp` | The near tier (ADR-0447..0450): the cars nearest the player as Jolt cars carrying out their drivers' intent, switched on one entity; physics sweeps answer "what is in my way" for them (ADR-0451). |
| `citysim/city_crowd.h/.cpp` | A crowd of residents made from the road model and `crowd.lua` (ADR-0446): places beside streets, households, cars, days. |
| `citysim/city_brain.h/.cpp` | The residents with the agent brain: what they want, their trips, carried out with the sim's walking and the new driving. |
| `citysim/city_sim.h/.cpp` (parts) | The old sim, which still walks every person on foot on the street map: `thingsForDriving` writes them into the driving's things each tick; `crossingAhead`, `gapToCross`, `jaywalks` make them wait at a crosswalk's kerb end and cross (ADR-0452). |

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
- `RT_TEST_FILTER=brain_crowd ./build-viewer/level_tests`: 12 simulated minutes of 350 residents; per-minute lines
  include the crosswalk counts. Pins its hash.
- `RT_TEST_FILTER=near_tier ./build-viewer/physics_tests`: contacts and hits on walkers.
- `RT_TEST_FILTER=v2_drives ./build-viewer/level_tests`: traffic_lab, 300 cars, 5 minutes; two known failures.
- Viewer: `tools/rtctl.py` launches `build-viewer/viewer assets/levels/brain_crowd.json`; `tools/brain_lab_probe.py`
  drives it (`brain labels off`, `near marks on`, `brain follow rN`, `sim speed 16`, `near?`, `things?`). Stop only the
  viewers you started, by PID.

## 6. Known faults and open work

- The old sim's people walk the street map's kerb-side line, not a pavement of the model, and cross through the
  junction's corner blend; their places waiting at a crosswalk are where their route comes level with its kerb end.
- The sim's `walkersInRoad`, `junctionRadius` and the old signals still measure the roads from the street map for the
  old driving (v1); they are slated to go with it.
- A walk round a junction that crosses two arms (a far turn on foot) is judged one crosswalk at a time, waiting at the
  corner between.
- Since ADR-0453 the traffic lab's cars are entities with nobody in them: `spectate` follows them after the agents,
  `CitySim::census(world)` counts them, and the key takes one (`carEntityNumbered`, `DrivingHost::takenByPlayer`). The
  near test bed's cars drive as one person number (`kLabPerson`) where the roaming cars drive as their own numbers.
- traffic_lab has no near tier: switched on (brain_crowd's `citysim.near`), no two Jolt cars touch, but a few near cars
  leave their carriageway on some streets and stand off their lane for minutes. How it was measured is in ADR-0453.
