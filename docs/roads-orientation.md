# Roads: orientation for a helper

Start here before touching anything under `src/engine/procgen/city/roads/`. This page says where things are and
who reads what. The *why* is in `docs/road-model-design.md` (the design and its step table) and in ADR-0417 to
ADR-0426 in `docs/decisions.md`. Written 2026-11-14, after step 8 (commit 5fbd797a). Anything marked *(unsure)* was
not checked in the code.

Words: a **station** is a distance along a road's centre line (its **spine**), in metres. A **junction** is where
roads meet at one level. Its **arms** are the streets' ways in and out, and a **through pair** is two arms that are
one street running on. A **street** is one road-file edge from one junction to the next. A lane's **slot** is its
place across the road, counted from the centre-line side (0) towards the kerb. **Plan coordinates** are world
(x, z) carried as (x, y). That is a mirror image, so the world drives on the left on screen. The kerb side comes from
one place, `kerbPerpSign()` in `lanes/lane_graph_build.h`. Never write "left" or "right" for a road side: say kerb
side or centre-line side, near turn (crosses no oncoming traffic) or far turn (crosses it).

## 1. The one road model

The road model (`road_model.h`) is one description of a level's roads, with ids that never change: junctions,
arms, streets, lanes, connections (one lane leaving or joining another) and stated turns. Each fact is copied from
the stage that states it and never worked out again. It has two layers:

- **Planned layer** (`plannedRoadModel`). Made from the road file alone: the planner, or the one-off layout tool
  `lanes_tool layout`, writes the road file.
- **Built layer** (`assembleRoadModel(build, vehicles, report)`). The road builder adds lanes, connections, junction
  heights, the paint's boxes and stop lines, and the stated turns. Its last step fills the model's **lane level**
  (`RoadModel::laneLevel`, ADR-0429): the table of paths the driving drives on, checked against the vehicles named.

**A level's model is made in one place** (ADR-0435): `obtainLevelRoadModel(level, cacheDir)` in
`lanes/road_model_cache.h`. It returns one model per lane-built city: the first city's built model (read from
`cache/roadmodel` or built and saved) when its road file states what the lane level needs, otherwise its planned
layer; every later city's planned layer. The loader calls it once a load (`lanesLevelRoads` in `level_loader.cpp`) and
publishes the result as `LevelRoadModel` (components.h). The lot pass grown at load and the driving host read that
object. The bake's lot producer and the offline tracer run without the loader and call the same function. Nothing else
makes a road model for a level. A tool or a test that has a road file and no level calls `plannedRoadModel` or
`assembleRoadModel` itself.

**The vehicles the turns are checked against** are the level's own fleet. `levelVehicles(level)` (same header) runs
the vehicle script the level names (`citysim.vehicles`, `citysim.fleet`) and reads each vehicle tag's most demanding
body through the engine's fleet reader (`engine/scripting/fleet_lua.h`, `roads/vehicle_size.h`). The driving library
derives its vehicle classes with the same functions. So the cache key needs no host, and `rt_bake`, the loader and a
tool make the same key. Cost: about 1.2 s a load to run the script.

**A lane's facts are declared once** (ADR-0435): `LaneFacts` in `road_model_parts.h` (id, kind, road, direction, slot,
width, stretch of spine). The builder's `Lane` (lanes/lane_expand.h) and the model's `ModelLane` both are that struct
plus their own part; `builtRoadModel` takes the builder's facts whole. A lane's road is `edge` (`road()` gives it as an
int, -1 for a connector) and its width is `width`: the old names `parent` and `w` are gone.

Readers ask the model by id instead of measuring geometry; section 3 lists them. Kinds of attachment (a name, a
sign) are data, not code.

## 2. File map

Under `src/engine/procgen/city/roads/`:

| File | What it holds |
|---|---|
| `road_model.h/.cpp` | The model: types, planned and built layers, save/load, `dump`, `streetNameNear`. The built model holds the lane level and is saved whole by `lanes/road_model_cache`. |
| `road_model_parts.h` | The one set of junction, arm, street, node and stretch types (`ModelJunction`, `ModelArm`, `ModelStreet`...). The road file is read into them (`RoadLabGraph::junctions` / `streets`), the layout and the control plan write them, and `plannedRoadModel` copies them whole and adds what only the model knows. A junction or street named by another is a place in the same list, not an id string. `graph_index.h` holds the index types it shares with the lane graph. |
| `street_map.h/.cpp` | The street map every agent routes, drives and walks on (`NavGraph`), built at load from the planned layer and the deck heights (ADR-0425). |
| `lane_turns.h/.cpp` | Which arriving lane takes which turn: reads `assets/data/lane_turns.json`; `turnsAtJunction` is the one function that applies it (ADR-0426). Also `laneLineBetween`, the one rule for whether the line between two lanes may be crossed (its lane kinds are the table's `lane_lines`). |
| `control_signs.h/.cpp` | The stop, give-way, do-not-enter and one-way signs that follow from stated facts (ADR-0424). Also ADR-0419's stop-line rule, `stopLineFromMouth`. |
| `lane_graph.h/.cpp` | The lane graph's types (paths, sections, junctions, crossing points, regulations), its save/load (LGRF), `validate`, dump, reach. The driving library reads these types. |
| `road_builder.h/.cpp`, `lanes_builder.cpp` | The roads module's one builder interface and the lanes builder behind it (ADR-0089). |
| `road_entity.h/.cpp`, `road_net_internal.h` | The engine's road entity (`RoadGraph` + look + plan). |
| `ground_grid.h/.cpp` | Absolute ground heights on a grid, as the builder hands them back. |
| `lanes/road_graph_spec.h/.cpp` | Reads the road file (`RoadLabGraph::fromJson`), including the road class table `roadClasses()` and every section in part 4. |
| `lanes/road_layout.h/.cpp` | The road file's layout: junctions and streets with ids, the text writers used by `lanes_tool layout`, `keyedRoadFileText` (what the bake key hashes) and `laneLevelKey`. |
| `lanes/control_plan.h/.cpp` | The control plan: each junction's control and each road's limit, from the placement policy. Also `findJunctions`, the layout's grouping of road ends into junctions by the `rules.storage` rule. Rework: the design wants the planner to state these, not group them (step 9 / design "deleted at the end"). |
| `lanes/lanes.h/.cpp` | The lanes pipeline: graph to lanes to profiles to pavement to surfaces to terrain. `Result` holds the paint's junction boxes (`junctionBoxes`). The ramp-end check still uses an angle test (`kJunctionParallelForGraph`). |
| `lanes/lane_expand.h/.cpp` | Roads become lane ribbons: pockets, ramps composed from their gore anchors over their whole stated length (`rampEndDistances`). The builder's `Lane` is `LaneFacts` plus its centre line, edges and working data (`sweptFrom` / `diesInto` for pockets and ramp lanes; ADR-0435). The streets a lane runs along and the junction at each end are held by the model's lane only (`ModelLane`, filled in `builtRoadModel`; ADR-0434). |
| `lanes/deck_mesh.h/.cpp` | Meshes and paint. `junctionAreas` gives each stated arm's box edge and stop line (`kStopLineBack`, 5.2 m, outside the box). The paint's lane lines, stop bars and arrows are read off `builtRoadModel` and listed in `RoadPaint` (ADR-0431). `paintFactsKey` is what the paint reads, for the bake key. **Open in step 9**: draw corners to a stated radius. |
| `lanes/deck_height.*`, `vertical_profile.*`, `road_design.*`, `terrain_conform.*`, `terrain_recipe.*` | Heights: profiles along spines, blended decks, the planner-shared vertical design, cut and fill. |
| `lanes/pavement.*`, `geom2d.*`, `polyline_ops.*` | 2-D geometry: footprints, the one triangulation, the Clipper2/CDT seam, polyline helpers. |
| `lanes/interchange.*` | Diamond interchanges (ramps at the streets a freeway crosses). The solid stretch before a stop line is handed in (`DiamondOptions::solidBeforeStopLine`): the planner's `SceneOptions::solidBeforeStopLine`, which it also writes into the road file's `lane_rules` (ADR-0435). |
| `vehicle_size.h/.cpp` | The body a road is checked against per vehicle tag (`VehicleSize`, `addVehicleSize`), and how a wheelbase and a tightest turn follow from the vehicle data. The driving library derives its classes with the same functions. |
| `lanes/lane_level.h/.cpp` | **The lane level as a table built from the model** (`buildLaneLevel`, ADR-0426): the last step of `assembleRoadModel`, which fills `RoadModel::laneLevel` (ADR-0429). The header lists where each part comes from. |
| `lanes/lane_graph_build.h/.cpp` | What is left of the old builder: report types (`LaneGraphBuild`, `CornerFault`), `cutFolds`, bend profiles, kinks, the vehicle sizes. Has the `kLaneGraphBuildVersion` cache version. |
| `lanes/lane_graph_connect.h/.cpp` | Connector shaping and `fitConnector`, the per-class corner-fit test that produces corner faults; `reachReport`. Step 9 will change what it reports, not delete it *(unsure)*. |
| `lanes/lane_graph_crossings.h/.cpp` | Crossing points: where two paths' bodies could touch (a kept measurement). |
| `lanes/swept_body.*` | The ground a long body covers on a bend. |
| `lanes/road_model_cache.h/.cpp` | **The one maker of a level's road model**, `obtainLevelRoadModel` (ADR-0435; part 1), and cache/roadmodel: the built model (lane level inside) saved under one key. Also `levelVehicles` (the level's fleet as the roads read it) and `levelRoadModelCacheDir`. The roads' one cache outside the bake (ADR-0429). |
| `lanes/city_producer.*`, `lots_producer.*` | The bundle producers: road meshes, ground, the drawn deck and pavement holes; then lots. `lots_producer.h` turns the level's road model into the roads as the lot pass reads them (`lotRoadInputs`: the model's streets by id and the deck, ADR-0430); it makes no model of its own. The lot twin (`road_twin.*`, `lotNav`, the sections roads/twin and roads/lotnav) is deleted. |
| `lanes/block_audit.*` | The lot pass run headless and audited. |
| `lanes/lanes_export.*` | glTF, plan SVG, stats JSON, lane graph plan pictures. |

Tools: `tools/lanes_tool.cpp` (all subcommands; its top comment lists them, except `layout --lane-rules`, which it
does not mention) and `city_plan` (the planner; `city_plan controls`).

Tests:
- `lanes_tests`: `test_lanes.cpp`, `test_road_model.cpp`, `test_control_plan.cpp`; unit scenes in
  `assets/lanelab/*.json`; `tests/lane_level_scene.h` gives a scene the lane-level facts.
- `run_tests`: `test_lane_graph.cpp`, `test_nav_graph.cpp`, `test_street_signs.cpp` and more.
- `driving_tests`: hand-made graphs; does not read the model.
- `level_tests`: `test_traffic_lab_v2.cpp`, `test_levels_playable.cpp`.

## 3. Who reads the model today

- **Street names, name signs, the map**: planned layer (ADR-0420). Loader: `mapStreetsOf`, `signCornersOf`,
  `attachSignBlades`.
- **Lights and furniture**: each junction's stated control (ADR-0421). The street map's nodes carry their junction,
  and `NavGraph::signalControlled` reads the stated signal.
- **Control signs**: `control_signs.h` over stated arms and regulations (ADR-0424). Placed at load and attached to
  arms.
- **Street map / routing**: `LevelRoadModel::streetMap`, the level-wide routing table held by the one road object
  beside its `cities` (built by `appendStreetMap`, street_map.h, ADR-0425). Read by the city sim, buses, rail riders,
  parking, walkers, the trip planner, furniture, the day/night system and the light probes.
- **Lane level / driving v2**: the model's own `laneLevel`, filled by `assembleRoadModel` (ADR-0426, ADR-0429). The
  host makes nothing: it hands the driving the lane level of the built model the loader published in `LevelRoadModel`
  (ADR-0435), and warns when a vehicle class carries a tag the roads were not checked for. Only traffic_lab drives v2
  (`"driving": "v2"` in its level file); river_town and agent_lab get their built model too, since the loader makes it.
- **The lane table's copies of model facts** (issue #291, open): a junction's control, a path's posted limit, a
  section's road class, the regulations, a connector's turn kind and the merge mark are still copied into the lane
  table, because the driving library reads the lane table alone (`const LaneGraph*`) and its 193 tests run on hand-made
  lane tables with no model. `road_model_12` holds the control, the turn and the limit equal to the model's.
- **The old driving's lanes** (`LevelLanes`, `TrafficLane` in components.h; `src/apps/citysim/lane_map.*`): the loader
  still lays the lanes a second time for it (`expand` in `publishCityProducts`). Slated for deletion with v1.
- **Placement policy**: the planner writes controls and signs into the road file from `control_policy.lua`; the
  model only reads what was written.

Still working roads out for itself:
- *(Gone in step 9a, ADR-0428: the paint's own junction boxes. Each arm's box edge and stop line now come from the
  stated junction through one function, `junctionAreas` in `lanes/deck_mesh.h`. The rule: back from the mouth by
  the widest crossing road's half width, then `kStopLineBack`. The paint, `ModelArm::stopLine`, the lane level and the
  control signs all read it. A file that states no junctions has its layout derived once, at the top of `build()`.
  The bake key now includes where the stated arms stand.)*
- *(Gone, ADR-0431: the paint's own lane lines, stop bars and lane arrows. `buildMeshes` builds the model's built layer
  for its build (`builtRoadModel`) and draws from a list a test can read (`RoadPaint`, `lanes/deck_mesh.h`). The line
  between two lanes is `laneLineBetween` (`lane_turns.h`), the rule the lane level fills its table from: dashed where it
  may be crossed, solid where not. A stop bar is `markingsOnArm` (`control_signs.h`) over the `marking` rows of
  `road_attachments.json`. An arrow is the turns `RoadModel::turns` states from its lane. Which arm a lane meets is
  `laneSpanEnd` (`road_model.h`), asked by the turns, the lane level and the paint. Nothing stated: nothing painted, and
  a `[paint]` warning says so once. The city's bake key includes what the paint reads: `paintFactsKey`.)*
- **The paint, still by measuring**: where a lane lies inside a crossing road's lane (`junctionCrossRank`, a point test
  that stops lines inside a junction the areas do not cover) and the crosswalk's width (the lanes found beside the spine at
  the crosswalk). Lines are painted along no connector lane (nothing states what lies beside one).
  *(Stated since ADR-0434: which arms are marked at all is the road class's `junction_markings` datum, read by
  `paintsJunctionBoxes`; the lane arrows' layout is the `lane_arrows` marking row of `road_attachments.json`.)*
- **Corners**: the builder draws them without a stated radius. The lane level only tests the fit and reports corner
  faults.
- **The layout's junction grouping** (`findJunctions`, the `rules.storage` rule). It wrote the files' junctions once.
- **The lot pass** (partly moved, ADR-0430): it reads the road model's streets and the drawn deck through one look-up
  (`LotRoads` in `city_lots.cpp`, fed by `LotRoadInputs`), and no copy of the road graph exists. Still worked out by
  measuring: which street a lot fronts and a door faces is the NEAREST street a building may front, not the street
  bounding its block as `enclosedRegions` states it; blocks are the gaps in the built pavement; alleys are the lot
  pass's own segments, not streets of the model.
- **The planner's scoring**: still builds its own nav graph (`buildNavGraph` in `plan/city_plan.cpp`).
- **Bus-stop next-stop names**: `streetNameNear` finds the nearest named street within 40 m. The model answers it,
  but by position.
- **`NavGraph::signalledByDegree`**: the old count-the-arms rule, used where a map has no stated signals.
- **The old driving (v1)**: reads the lane match (`LaneMap` in `src/apps/citysim`), tied by road, direction,
  stretch and slot since ADR-0425. Its own junction boxes and stop lines are listed in the design doc; not checked
  here *(unsure)*.
- **Kept measurements, allowed by the design**: crossing points, bend profiles, the corner-fit test, and the ramp-end
  angle check in `lanes.cpp`.

## 4. Road file sections (`assets/lanelab/planned/<level>_lanelab.json`)

| Section | What it states | Bake key sees it? |
|---|---|---|
| `edges`, `classes`, `terrain`, ramps, pockets, connectors | Roads, their class overrides, ground | yes |
| `rules` (incl. `storage`, default 6.5 m) | Builder tolerances. `storage` is the waiting room between two junction points below which they are one junction (`findJunctions`). | yes (only `streets`, `divided`, `junctions`, `arms_in_ring`, `lane_rules` are cut) |
| `carry_on_control` | The control the plan gives carry-ons (control_plan.cpp) | yes *(unsure what reads it at load)* |
| `junctions` | Id, `at`, arms in their ring (street, heading, regulations; see below), `through` (by place in that list), `inside`, `control`, `why`, `nodes` (step 7's points) | no |
| `arms_in_ring` | `true`: every junction's arms are listed in their ring (ADR-0427; see below) | no |
| `streets` | Id, road, stations, the junction at each end, `carries_on`, `name`. A street's end is at a junction or carries on, never both (ADR-0435): where a junction is stated the junction's through pair says how the street runs on, and no `carries_on` is written for that end. | no |
| `divided` | Pairs of roads that are one divided road | no |
| `lane_rules` | `{"solid_before_stop_line": 20.0}`: the stretch before a stop line where lane lines are solid. The number's one home: the planner writes it (`SceneOptions::solidBeforeStopLine`, the number its ramps are laid by) and the lane level and the paint read it (ADR-0435). | no |

The lane graph's key (`laneLevelKey`) hashes `junctions` (without `nodes`), `arms_in_ring`, `streets`, `divided` and
`lane_rules`.
It is one part of the built road model's one cache key (`roadModelCacheKey`). Since ADR-0431 the paint reads the junctions' controls, arms and through pairs, the streets (not their names), `arms_in_ring` and `lane_rules`, so writing those re-bakes the city (`paintFactsKey`); a street's name, a junction's `nodes` or `why`, and `divided` still change no bake. Only traffic_lab and river_town have `lane_rules`. Every
other road file gets no lane graph, and the error names `lanes_tool layout <file> --lane-rules 20 --write`. Write
facts with the tools (`lanes_tool layout ... --write`, `--names`, `--nodes`, `--lane-rules`;
`city_plan controls <file> rules/lab.lua --layout --write --controls-only`), never by hand.

**Writing the layout again changes nothing** (ADR-0435). `lanes_tool layout <file> --write` on a file that has not
changed gives the same bytes: it keeps each junction's `nodes` where they are (and so their order), writes `through`
back before them, and states no carry-on at a junction. It prints "the file is unchanged, byte for byte" or "CHANGED".
`road_model_19` holds this for traffic_lab, river_town and agent_lab. `--fresh` starts the layout over and drops the
nodes; `--nodes --write` finds them again.

**A junction's arms are listed in their ring** (ADR-0427): counter-clockwise round the junction in plan coordinates
(the way headings grow). The file says so with `"arms_in_ring": true` beside the junctions list.
- **Readers trust a marked file.** The turns (`turnKindsFrom`: which arm is straight on, near or far), the regions
  the streets enclose, the lane level's junction outline and the street signs read the list as it is. Nothing sorts
  by heading or angle, and nothing measures the ring at load.
- **An unmarked file** that lists junctions gets one road-model problem line naming the command. The loader logs the
  model's problems, and the lane level refuses the file. Load checks only the structure: no arm listed twice, and
  `through` pairs naming two different arms that exist.
- **One measure** (`armRing` in `road_layout.h`) is used by the writers and the check, never at load. It takes only
  `ComposedRoads`: the graph with its ramps composed, because an arm's station is a composed-spine station. A compact
  junction (every mouth within 1 m) is ordered by the arms' headings. A spread one is ordered by where each arm's road
  leaves a circle 5 m wider than the farthest mouth. Exact ties keep the list order. A road that never leaves the
  circle, or has no spine beyond its mouth, is noted.
- **Writers:** the layout (`deriveLayout`, `writeLayout`, `insertLayoutText`), the control plan (`writeControlPlan`),
  and the one-off `lanes_tool layout <file> --arm-order --write`. The tool orders an old file, renumbers `through` and
  marks it.
- **The check:** `lanes_tool layout <file> --arm-order` without `--write` lists each junction whose stated ring
  differs from the measure, with the measure's notes. It exits 3 when one differs or the file is unmarked.
  `road_model_18` runs it on the shipped files (traffic_lab, river_town, agent_lab, ring_city), each with a list of
  junctions allowed to differ (empty).
- **What moved** when the three level files were put in order is in `docs/road-model-compare/<level>_arm_order.txt`.

## 5. Data tables

- **`assets/data/lane_turns.json`**: five rows: straight on, near turn and far turn, plus how an arm with no way
  straight on shares its lanes. They say which arriving lanes take each turn and into which leaving lanes. `[~]`
  marks a guess. Its hash is part of the road model cache's key (`laneTurnTableHash`, the file's bytes).
- **`assets/data/road_attachments.json`**: what may attach to a street, junction or arm (names, signal poles, signs).
  A `sign` row is a whole sign: when, facing whom, where, its look. A `marking` row is paint at an arm: the stop bar
  (`means` names the rule set's device it is the paint of) and the lane arrows with their layout. These rows are the one
  table saying which device paints what; the rule set (`rules/lab.lua`) states no paint (ADR-0434).
- **`assets/scripts/rules/control_policy.lua`**: the placement policy. Class order, posted limits with sources, and
  first-match rows choosing each junction's control and signs. The planner reads it via the rule set
  (`rules/lab.lua`).
- **The road class table**: `roadClasses()` in `lanes/road_graph_spec.cpp`. It is C++, not a data file. Each
  class's lane width and counts, shoulder, sidewalk, median, rank, grade, kerb parking and whether its arms at a
  junction are marked (`junction_markings`). A road file's `classes` override rows.
- `assets/data/streets.json`: name words and sign look (names only).

## 6. Checking a change quickly

Build: `cmake --build build-viewer --target lanes_tool lanes_tests` (add `viewer level_tests run_tests
driving_tests rt_bake city_plan` before anything that loads a level; stale binaries mislead, see part 7).

| Command | What | Wall clock (seen in step 8) |
|---|---|---|
| `lanes_tool build <file> --out <dir>` | Road meshes; compare `.glb`/`.svg` md5 before and after (stats.json differs only in timings) | ~12 s traffic_lab, ~36 s river_town |
| `lanes_tool model <file> --out <dir>` | model.txt, report.txt, model.svg | about a build *(unsure)* |
| `lanes_tool junctions <file> [--top N]` | The grouping's junctions, biggest first | seconds *(unsure)* |
| `lanes_tool layout <file> [--write ...]` | Layout facts; prints whether the bake and lane-graph keys stay unchanged | seconds |
| `lanes_tool lanegraph <file> --out <dir> --vehicle tag:radius:width:length ...` | lane_graph.txt, problems.txt, reach.txt, corner_faults.txt, svg; `--junction J` for a close-up | ~26 s traffic_lab, ~63 s river_town |
| `./build-viewer/rt_bake --status assets/levels/<level>.json` | Is the bake current (must not change) | seconds *(not timed)* |

The fleet for `--vehicle`, as step 8 used it: `bus:7.30:2.55:11.45 car:6.15:2.05:5.57 goods:7.45:2.30:6.69
moped:1.92:0.70:1.70 motorcycle:2.73:0.70:2.09 truck:7.45:2.25:6.69 van:7.10:2.30:6.34`.

Tests (`RT_TEST_FILTER` is a substring of the case name):
- `./build-viewer/lanes_tests`: 124 cases, about 4 to 5 minutes whole. Filters: `road_model` (quick),
  `lanes_lane_graph_of` (the five scenes' lane-graph checks against what each scene is known to hold: `LaneGraphKnown`
  in test_lanes.cpp), `lanes_the_traffic_lab`, `connectors_of`.
- `./build-viewer/driving_tests`: 193 cases on hand-made graphs, about a minute *(roughly)*.
- `run_tests`, **always filtered**: `lane_graph`, `nav`, `route`, `trip_plan`, `control`, `signal`, `park`,
  `sign`, `map`. Seconds to a minute each *(roughly)*.
- `level_tests`, **always filtered**:
  - `level_traffic_lab_v2_drives`: about 8 to 9 minutes. It builds the lane graph (~45 s) and runs 300 cars for 5
    simulated minutes. Pins `kFiveMinuteStateHash`. Limit the runs per task.
  - `level_traffic_lab_v2_same_every_run`, `level_traffic_lab_v2_rebuild_hands_over_again`.
  - `level_traffic_lab_v2_one_road_model`: about 10 s. The loader publishes the built model, the driving drives on
    that object, and the fleet's sizes as the roads read them (ADR-0435).
  - `RT_LEVELS=river_town,traffic_lab` with `level_real_lanes_match_the_baked_roads` or
    `front_doors_face_their_street`.
- `python3 tools/agent_lab_probe.py --speed 4`: about 4 minutes; overwrites its shots in
  ~/Pictures/Raytracer/agent-lab.

Rules: never run `run_tests` or `level_tests` without `RT_TEST_FILTER`. No 30-minute soak inside a task. Wait for a
background job by its notification or its output file, never a `pgrep` loop. Stop only your own viewers, by PID
(`tools/rtctl.py` `launch`/`stop` does this). Keep scratch in `out/zz_<task>/` and delete it at the end. No lanes or
lots tag bump and no re-bake unless the task says so. Write C++ types out (`auto` only for lambdas). New kinds are
data rows.

## 7. Traps (from step 8)

- **Street stations are written to the centimetre.** A lane's end and a street's end, or a ramp's merge station and
  its lane end, differ by millimetres. Compare with `sameWrittenStation` (`road_model_parts.h`; `kWrittenStation`, 7.5 mm, `rt_math.h`: ADR-0433) and snap to the
  lane's own end. Without that, nothing was trimmed at stop lines: 2117 false corner faults, reach 50%.
- **Three orderings were one.** The turns once sorted arms by heading, the regions walked a circle and the outline
  sorted by angle round its middle. At a spread junction they disagreed (river_town 9 junctions, traffic_lab 3). Since
  ADR-0427 all three read the stated ring. Heading order alone is wrong at a spread junction: two arms can share a
  heading (j93's two west roads both say 180) or point across each other (j70's s147 and s149). An outline whose
  legs are joined in ring order must still lay each leg in the right direction: a leg's far point, then its near
  point. Tying that to the kerb side drew 136 self-crossing outlines.
- **Measure a ring only on composed roads.** A ramp arm's station is along the composed spine. Measured on the graph
  as read, a ramp arm is measured in the wrong place (ring_city j0–j3 and metro_planned have such arms). `armRing` takes
  `ComposedRoads` only, so it cannot be called on the graph as read.
- **A junction can sit mid-street.** An arm's street may run on through the junction instead of ending there.
  Stating turns per lane end missed those junctions: hill_junction got 0 connectors. Turns are stated per street end.
- **A ring street can name a carry-on at both ends.** Choose the end that has lanes leaving.
- **Stale binaries lie.** After changing `keyedRoadFileText`, an old `rt_bake` reported levels "out of date". Rebuild
  `viewer` and `level_tests` before loading a level too.
- **The cache can hide a change.** The viewer reads cache/roadmodel by key. If the builder's output changes, bump
  `kLaneGraphBuildVersion`; otherwise a viewer can show an old graph. Every load reads it now, the lab tests too
  (the load's log line says "read from the cache" or "built in"). A level whose file is not in the cache pays the
  road build and the lane level on its first load (river_town: about 71 s), since the loader makes the built model
  for every level whose road file states enough (ADR-0435).
- **Two id spaces.** Lab output gives ramp lanes as model lane ids (1110–1117); the driving reports path ids
  (1457, 1607). Do not mix them.
- **Fold tidying** (`cutFolds`) shortens lanes on tight inside corners, so section share tests must skip tidied lanes.
- **The Edit tool needs a fresh Read** after your memory is compressed. `docs/decisions.md` is 1.6 MB: read by
  offset.
- **The paint has no box at two-arm junctions** (traffic_lab's frontage ring j133–j151). Those stop lines come from
  ADR-0419's rule, and that is expected.

## 8. Known faults and open work

- **Step 9a's stop lines** (ADR-0428) sit 1–2 m further out than the paint's at most junctions. The old box was
  sampled every 2 m, so it usually ended a metre inside the crossing road. traffic_lab's self-crossing outlines went
  from 2 (j0, j22) to 5 (j0, j4, j19, j93, j96): two arms meeting at a sharp angle now have stop lines that cross.
  Corners are still drawn without a stated radius (step 9b).
- **traffic_lab five-minute test fails two held limits** (left failing on purpose, commented in
  `test_traffic_lab_v2.cpp`):
  1. Car 92 stands 261 s on frontage road s311 (lane 1457), where the off-ramp lane 1607 merges between junctions
     near (-562, 80). Two lanes there have no sign rank, movement or line, so the driving's fall-back (the lower path
     id gives way) decides it every time. Up to 143 s at the single seven-arm signal at junction 0 (115 s cycle).
  2. The queue row: 2.76 s a car against 1.7–2.1 s.
  Neither the driving's junction code nor the rule-set numbers were changed.
- **Corner faults = step 9's work list**: `docs/road-model-compare/<level>_corner_faults.txt`. traffic_lab 2219 over
  540 turns; river_town 3332 over 835 (its 31 turns with no lane path, on s228 and s287, are refusals of their own
  kind since the review, not corner faults). traffic_lab j5's four near turns (6.0 m) are too tight even for a car.
- **Street ends that meet another road with no junction stated** (problem lines since the review): traffic_lab 17 (the
  spokes at its ring), river_town 24: the planner's to state.
- **Two traffic_lab ramps carry no car** (d0_0_a_on, d1_0_a_off). Their hosts' kerb lanes end at freeway joints
  j153/j154, whose stated turns all leave the paved road.
- **river_town planner faults**: hairpin and gapped carry-ons (s277/s337 159°, s325/s344 4.9–10 m). x0_outR_link
  and x1_outR_link state a merge past the host lane's end. s320 carries on into s287 where no lane leaves. Dead-end
  spokes at the freeway ring. The planner no longer reproduces river_town's road file (ADR-0423). Car reach 41%
  without lane changes, 81% with: lane changes come first.
- **Outlines that still cross themselves**: traffic_lab j0, j22; river_town ten (problem lines). They are the same
  junctions before and after ADR-0427, so the arm order does not cause them. Their arms' lanes cross before the stop
  lines (j0 s33/s36).
- **The side rule** (lane_turns.cpp, `turnKindsFrom`): an arm with no through pair stated calls a turn near or far by
  which half of the ring the other arm lies in. Exactly halfway counts as far. traffic_lab: 56 junctions, 191 turns
  rely on it, 8 halfway notes. river_town: 43, 192, 25. At spread junctions it can call a turn near that a driver
  would call far: river_town j0's s2 into bv1_out is a 43-degree turn across the way, but its carriageway leaves the
  circle on the near side. It is listed per junction in the arm-order tables and is unchanged.
- **Deferred levels' road files are unmarked** (`arms_in_ring`). They load with a logged problem line and get no lane
  level until `lanes_tool layout <file> --arm-order --write` is run on them.
- **Deferred levels**: every lanes level but traffic_lab and river_town lacks `lane_rules`. Levels without
  `junctions[].nodes` get no street map (agent_lab, river_town and traffic_lab have them). The one-agent lab's car
  still drives v1. Steps 5, 6 and 9 are open (design step table).
- **Real cases the lane-graph checks found and name** (test_lanes.cpp, per scene): lanes of two roads crossing at one
  level before their stop lines at traffic_lab j0 (s33 and s36, four), j84 (s147, s149) and j22's approach (s57, s60);
  ring_city's two ramps crossing at grade; grid_city's boulevard lane crossing a local street's at a 100-degree carry-on.

## 9. Test gaps still open

- `LaneGraphKnown::insideRoads` entries that no longer apply never fail (a road listed but no longer inside a junction
  passes); the same for `tidied` amounts larger than needed.
- Pins by count alone, where two members can swap unseen: each scene's problem lines by kind (traffic_lab names its
  outlines, gap joins and unstated junctions by id; the rest are counts), the joins made per kind, traffic_lab's
  road-model problems by kind (its halfway notes are by id).
- Collinear overlap: two lanes of different roads lying along each other are not found by the crossing check
  (`segmentsCross` finds only a crossing).
- A lane path passing inside a junction (part 3 of `checkLaneGraph`) is still judged against the lane level's own
  outline, which is drawn through the lane paths' ends.
- Report branches with no test: an outline arm whose lanes' ways across cancel; a lane not laid point for point beside
  its spine; a lane with nothing left once its folds are cut; "runs along none of the streets nor inside a junction";
  "lies part way along its road"; a lone centimetre hole between two streets' stations with no junction beside it; the
  turns-placed count mismatch (cannot happen by construction). None arises in a hand scene; each needs degenerate input.
- The stop line where the paint has no box (`stopLineFromMouth`, 39 arms in traffic_lab) has no expected value written
  by hand anywhere: the test and the lane level call the same function. Pin at least one arm against a number worked
  out from the scene.
- The arm-order tables' "heading order" was the old sort rebuilt by a retired report (ties by the old file's list).
  It was not a run of the old binary. The lane graph diff before/after (connectors per junction) agreed with it on
  traffic_lab's three junctions.
- The ring measure's exact-tie note has no hand scene. Both the note that a road never leaves the circle and the
  check biting are tested.
- On a lane named as tidied, the allowed slack (the written cut) applies to each lane path, not once to the lane.
- The model's stop line itself is only checked as "stated and outside the box edge" (`test_road_model.cpp`) and on one
  hand-made grid_city arm against the paint.
- traffic_lab's known joins and the scenes' non-right-angle join gaps and turns are snapshots of builder output, not
  values worked out by hand.
