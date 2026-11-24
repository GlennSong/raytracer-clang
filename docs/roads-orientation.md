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
publishes the result as `LevelRoadModel` (components.h). The driving host reads that object. THE LOT PASS READS THE
PLANNED MODEL ONLY (ADR-0476: `levelPlannedRoadModels(in)`, each city's planned layer from its road file, no build): the
lots are grown before the built model is assembled, and their paths are stated on the built one after. The bake's lot
producer and the offline tracer do the same. Nothing else
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

**A lane's centre line is held once** (ADR-0438). Every lane's points and the distance along its lane to each live in
one object, `LaneCurves` (`road_model_parts.h`), made by `expand` as it lays the lanes. A lane names its stretch of it
(`LaneFacts::curve`). The build's lane set, each of its lanes and the road model made from that build all hold that
same object (a shared pointer), so nothing is copied and the two forms cannot disagree. Read it through
`Lane::xy()` / `Lane::s()` in the builder (each gives a `Span`: a pointer and a count, valid while the lanes are not
being laid) and `RoadModel::lanePoints()` / `laneAlong()` in the model. A model read back from `cache/roadmodel` holds
its own. `Span` (`graph_index.h`) is our stand-in for C++20's `std::span`; `project`, `pointAt`, `tangentAtStation`,
`tangentAt` and `SegmentGrid` take one, so a whole vector still passes as it is. The old driving's `TrafficLane::xy`
(`level_loader.cpp`) is still its own copy.

Readers ask the model by id instead of measuring geometry; section 3 lists them. Kinds of attachment (a name, a
sign) are data, not code.

**The query functions are THE way to read the roads** (ADR-0436). A reader asks one of these; it does not walk the
model's lists or write the bounds checks out again. If the question you have is not here, add a function here and
move the loop into it; never a private loop in the reader.

| Question | Function |
|---|---|
| The road model of a city; one of its junctions | `LevelRoadModel::city(c)`, `junctionAt(c, junction)` (components.h) |
| Is a junction lit; does the file state controls; a street's run key; junction points | `LevelRoadModel::lit`, `controlsStated`, `runKey`, `junctionPoints` |
| The street an arm is | `RoadModel::streetOf(arm)` |
| Which arm of the junction at a street's end that street is | `RoadModel::armAt(street, end)` |
| A street's centre line; where one of its ends is | `RoadModel::centreLine(line)`, `streetEnd(street, end)` |
| Where an arm meets its junction; the named arms round a junction | `armMouth`, `namedArmsAt` |
| A street's name; the named street nearest a point | `streetName`, `streetNameNear` |
| A street's lanes; what a lane leads to; which arm and stop line a lane meets | `lanesOf`, `leadsTo`, `laneSpanEnd` |
| Is a street one-way; its widths; do cars drive it | `ModelSection::oneWay`, `travelWidth`, `carriagewayHalfWidth`, `carriesCars` |
| The street a lane path or connector is on | `RoadModel::streetOf(path)` |
| The limit posted on a path; its road's class | `RoadModel::postedLimit(path)`, `roadClassOf(path)` |
| The control at a junction of the lane level | `RoadModel::controlAt(junction)`, `junctionOf(junction)` |
| The lane level's junction a model junction is (ask once: a walk) | `RoadModel::laneJunctionOf(modelJunction)` (ADR-0456) |
| The turn a connector takes | `RoadModel::turnOf(path)` |
| Which path joins by a stated merge | `RoadModel::mergingAt(crossing)` |
| The rules standing along a lane path (signs at its end, a limit part-way) | `RoadModel::regulationsAlong(path)` |
| The junction a lane path runs to | `RoadModel::arrivesAt(path)` |
| Do two lanes arriving at a junction face each other (oncoming traffic) | `RoadModel::facing(junction, a, b)` |
| The arm a street-map link arrives by | `RoadModel::armReachedAt(street, link.stationTo)` |
| The kerb parking spaces along a street; one space by its id; its name | `RoadModel::kerbSpacesOf(street)`, `kerbSpace(street, dir, index)`, `kerbSpaceName(space)` |
| Where a car standing in a space is and faces | `RoadModel::kerbSpacePose(space, out)` |
| The kerb lane path beside a space (or a station) and how far along it: where a car stops to turn in, or rejoins | `RoadModel::kerbLaneBeside(space)`, `kerbLaneAt(street, dir, station)` |
| The space nearest a point within a distance | `RoadModel::nearestKerbSpace(point, reach)` |
| How far along a lane path a station of its spine falls | `RoadModel::alongPathAt(path, station)` |
| Where a point (a door) fronts the road: street, station, kerb | `RoadModel::frontageAt(point)` (a search, for what states nothing); `frontageOn(street, station, dir)` (stated facts); the station and kerb from a search's hit on a centre line, the one statement both the model and the lot pass use: `frontageOnLine` (`line_frontage.h`, ADR-0474) |
| Where a lot's building and each door front, recorded at plan time (ADR-0474) | `LotBuilding::frontage`, `DoorSpec::frontage`, `AuthoredPlace::frontage` (`StatedFrontage`, road_network.h: street id, station, kerb; or a walkway id and metres along it, ADR-0475: a door on a lot's path, ADR-0476) |
| From a frontage: the nearest place to walk, the kerb lane to set down in, the kerb spaces in walking order | `RoadModel::sidewalkBeside(f, out)`, `kerbLaneBeside(f)`, `kerbSpacesNear(f, reach, most)` |
| Where a place (a marker, a door) joins the walks: a point on a walk along its street, its kerb, and whether held to a sidewalk's end, across the road, cut off (ADR-0468, THE ONE ANSWER for placing a place, starting and ending a walk; built layer) | `RoadModel::walkJoinOn(frontage, openWalks, parts)` (stated: `frontageOn`; or straight from what a place states, `walkJoinOn(StatedFrontage, ...)`, which also answers a door on a walkway: ADR-0475); the one fallback by position `walkJoinNear(point, ...)` (`frontageAt`, then the same); `WalkJoin`, `WalkJoinWhy`; a station's place on a walk: `walkAlongAt(walk, station)` |
| Every stretch of road with a centre line, in the order searches by position use | `RoadModel::roadLines()` |
| The crosswalks (ADR-0452): across an arm; by id; of a lane-level record; the paths over one; which a walk round a junction crosses; where a person waits at a kerb end | `RoadModel::crosswalkAcross`, `crosswalk`, `crosswalkOfLane`, `crossersOf`, `crosswalksOnWalk`, `ModelCrosswalk::kerbWait` (stated once by `stateCrosswalks` in road_model.cpp; laid in the lane level by `layLaneCrosswalks`) |
| Where people on foot can walk (ADR-0464): the corners round each junction's ring (which kerb of each arm); the ring pairs with no corner and why; the dead ends and whether they are joined across; what is at a street's end; is there a sidewalk (or any walk) along a kerb | `RoadModel::corners` (`ModelCorner`), `ringPairs(m, g)` (`CornerGap`), `RoadModel::deadEnds` (`ModelDeadEnd`, `deadEndsOf`), `streetEndKind(g, street, end)`, `ModelSection::sidewalkOnKerb`, `pavementOnKerb` (stated once by `plannedRoadModel`; the walks are laid from them: ADR-0465, ADR-0466). A street end "meets another road" only where the two can meet at grade, by the crossing policy (`meetAtGrade`, `lanes/road_design.h`; ADR-0469): a street ending under the freeway is a dead end |
| Whether two roads meet at grade or one bridges the other, by their classes (the planner's crossing policy: a track or the freeway meets nothing at grade but its ramps; a footway ending on a road meets it) | `meetAtGrade(g, a, b, aEndsOnB, bEndsOnA)` (`lanes/road_design.h`, ADR-0469): `planCrossings` writes it into a plan; `roadEndMeetsAnother(g, e, end)` asks it where a road's end lies inside another's band (ends at one node always meet), and is the one answer for both the design stage's open ends and `streetEndKind` (ADR-0470) |
| The walks the road build laid (ADR-0465): each kerb's sidewalk lane along each road, each corner's walk along the drawn pavement (a spread junction's pairs across its points too), and why a ring pair has none | `RoadModel::walks` (`ModelLane`: kind `sidewalk` with `dir` the kerb, or `corner`), `ModelCorner::walk`, `ModelCorner::spread`; in the build, `LaneSet::walks` and `Result::walkCorners` (`WalkCorner::why`, and its kind `WalkCorner::fault`: a kerb end out in the paved junction, or the drawn sidewalk in two pieces; ADR-0469), laid by `layWalks` (`lanes/walk_lanes.h`). A footway's lane is a car lane's record of kind `footway`. Read by the lane level (next row) |
| Where a person steps off the kerb at an arm (a station along its road) | `kerbEndStation(m, junction, arm)` (ADR-0466): the corner walks end there and the lane level cuts the sidewalks and footways there |
| The walks of the lane level (ADR-0466): each sidewalk's stretch per street and kerb, each corner walk, a crossing over each crosswalk people use, a turn-round across each joined dead end, each footway's stretch per street; which walk ends meet; every end that meets nothing, with why | `LaneGraph::walks` (`Walk`: its curve, and one of `modelWalk`, `modelLane`, `modelCrosswalk`, `modelDeadEnd`), `Walk::atStart` / `atEnd` into `LaneGraph::walkJoins` (`WalkEnd`), `LaneGraphBuild::walkEndsAlone` (a kerb end whose ring pair the build tried and could not lay says so with the build's fault, a spread junction's pair too, never "no corner is stated": ADR-0469); laid by `layWalkPaths` in `lanes/lane_level.cpp`, the lane level's last step |
| Paths for people on foot beside no street (ADR-0475): a park's path, a plaza's, a cut-through. Each states its kind (a path users' row: `footway`, `park_path`), width, line with heights, and where each end lands: a street's sidewalk (street id, station, kerb: a `StatedFrontage`), another walkway (id, metres along), or a stated dead end. Laid in the lane level as walks (`Walk::modelWalkway`), the sidewalk or walkway landed on cut there and joined; an end that does not land is a fault with why | `RoadModel::walkways` (`ModelWalkway`, `WalkwayEnd`), supplied ONLY by `stateWalkways(model, walkways)` on a built model (returns `WalkwayProblem`s: `WalkwayFault` and why), which calls `layWalkways` (`lanes/lane_level.h`). The types live in `roads/walkway.h`. The lot pass states its paths as them (ADR-0476: `city_lots.h stateLotPaths`, handed over by `lots_producer.h stateLotWalkways`); the street map reads them (`street_map.h appendWalkways`) |
| The kind of a lane path or walk; every lane path and walk a mover may use (the one mover filter); the parts of the walks a person can get round | `RoadModel::pathKind(path)`, `walkKind(walk)`, `openTo(mover, users)` (`OpenPaths`: per path, per walk); `reachableWalks(g, open)` (`lane_graph.h`) |
| Who may use which kind of path; how a dead end is joined; the wait a mover adds for stepping onto a kind (a crossing's) | `pathUsers()` (`PathUsers::usersOf`, `mayUse`, `mover`, `deadEndJoin`; `PathMover::waitOn`), from `assets/data/path_users.json`; read by `RoadModel::openTo` and the router's `bindWalker` |
| A route between two places on the lane level (the one route-finder, ADR-0467): a car's over the lane paths (legs, turns, which lanes lead on); a walker's over the walks (the walks taken with their direction, metres, seconds, the crosswalks crossed and from which kerb end) | `route.h`: `findRoute(model, studyRoads(model), RouteClass, PathPoint, PathPoint)`; `findWalkRoute(model, bindWalker(model, mover, pace), WalkPoint, WalkPoint)` (`WalkRoute`, `WalkStep`, `WalkCrossing`). A place: `PathPoint` / `WalkPoint` (`lane_graph.h`); a walk's point: `LaneGraph::pointAt(WalkId, along)`, and its height (the pavement it runs on, a bridge's sidewalk too: ADR-0480) `LaneGraph::heightAt(WalkId, along)` |
| The nearest of several centre lines to a point (the one search) | `nearestLine` (`city/polyline.h`) |
| May a building front a road of this class | `buildingMayFront` (`city/road_network.h`) |

**The lane table holds no fact about what the road is** (ADR-0436, ADR-0437, issue #291): no control, posted limit,
road class, turn kind, merging path or regulation. It holds where a car can be, and each path, junction and crossing
point carries the id the model is asked by. A lane path also carries the stretch of spine it was cut along
(`Path::onSpine`, `stationFrom`, `stationTo`, the way it is driven): the model places a street's regulations on it by
those. The last copy, `Path::throughArm` (the model's through pair), is gone since ADR-0438: an arriving lane carries
only its arm (`Path::arm`), and which approaches face each other is asked of the model (`RoadModel::facing`, read by
the builder's waiting-point pass and by the driving's `approachesFace`). A lane table made by hand states its
junctions' through pairs with `stateThrough` (`tests/driving_test_roads.h`).

**Regulations** (ADR-0437). A rule standing along a road is stated in the model, once: a sign at a junction's arm
(`ModelArm::regulations`, from the road file) or a regulation on a street (`ModelRegulation` in
`RoadModel::regulations`: kind, street, stations from and to, which way the traffic runs, value; added with
`stateRegulation`). Its kind is the name of a device in the rule set. Nothing writes street regulations into a road
file yet. The driving does not ask the model each tick: its binding (`RoadRules::regulationsOn(path)`,
`apps/driving/road_rules.h`) is filled from `regulationsAlong` each time rules are handed over and never saved.

**Kerb parking spaces** (ADR-0439). Where a car may be left at a kerb is stated in the model, once:
`RoadModel::kerbRuns`. A run is a street, a kerb, the station where its first space begins, a space's length and a
count. Which kerb is named by the traffic beside it: `dir` +1 is the kerb of the traffic running along the spine, -1
of the traffic running against it. A space is place `i` of its run; its id is street, kerb and index (`s5 along 3`).
Its width and its place across the road are the street's cross-section (`parking`, `parkingOffset`), not copied. The
planned layer states the runs (`stateKerbRuns` in road_model.cpp) from the streets, each arm's stop line and the two
numbers of `assets/data/kerb_parking.json`. Kept clear of: junction areas and stop lines, and what lies outside a
road's `lots_range`. Not known to the model, so not kept clear of: driveways, bus stops, hydrants, bridges. The lane
table holds no spaces (its unused `CarSpace` is deleted); the kerb lane path beside a space is asked
(`kerbLaneBeside`). The outlines painted on the road are drawn from these spaces on a level the new driving drives.
The old city sim still lays and parks in its own bays (`CitySim::ParkingBay`, a different list); they go with it.

**From a door to the road** (ADR-0439, ADR-0474). `frontageAt(point)` is a search by position. It is the same search
the lot pass faces its doors by: both call `nearestLine` (polyline.h) over `RoadModel::roadLines()`, in its order, with
`buildingMayFront` as the rule, and both turn the hit into a station and kerb by `frontageOnLine` (line_frontage.h).
`lotRoadInputs` makes the lot pass's roads from that list, with each line's stations as the model states them; never
add a road to one and not the other. Since ADR-0474 the lot pass records the answer (`StatedFrontage` on each building
and door, lot format 6), so a reader of a lot asks `frontageOn` with no search; `frontageAt` stays for what states
nothing.

**From a place to the walks** (ADR-0468). `walkJoinOn` answers from a stated frontage (the brain's markers state
street, station and kerb; since ADR-0474 so do a lot's buildings and doors, carried to the sim as
`AuthoredPlace::frontage` and, since ADR-0480, onto the sim's own place as `citysim::Place::frontage`, which the sim's
trips start and end from since piece 6's switch, W3: `CitySim::placeFrontageAt`); `walkJoinNear` is the only fallback
by position (the brain's walks and a possessed agent's, which hand the sim a position). Only walks along a
street (`Walk::street` set: sidewalk and footway stretches) are joined, never a corner, crossing or turn-round, never a
walk closed to the mover; `cutOff` says when the walk is outside the largest part on foot. The street map's look-ups it
replaced (`CitySim::walkStartNode`, `kerbLinkNear`, `kerbSideLink`) were deleted at W3, and with them the level test
that set them beside it. Measured before W3: river_town has 98 of 799 crowd places on sidewalks cut off from the largest
part, and 136 doors snap to park/quad footpaths the lane level does not hold. Since W3 people walking there stand where
no route leaves their part (river_town's crowd test fails; ADR-0480 W3; waiting on the walks' fix).

**No street map** (#306, ADR-0480's sweep). The street map (`NavGraph`, `street_map.h`, its router `pathfind.h`) is
deleted. Every reader asks the model: the city sim's people rest at walk points on its walks
(`src/apps/citysim/city_walk_points.h`) and walk by the one router; the furniture plans along its streets' centre
lines and at its junctions' arms; the probes, the map, the signs and the day/night footprint read its streets. The
arm a street arrives by at a station is asked of the model (`armReachedAt`).

**Walkways** (ADR-0475). A path for people on foot beside no street is stated by whoever draws it (S6b: the lot pass),
with where each end lands looked up once when drawn. `stateWalkways` is the one way in; the lane level trusts the ends,
cuts the walk each lands on and joins there, and returns what did not land. Nothing joins a walkway by distance.
Since ADR-0476 the lot pass states every path it draws (`LotBuilding::walkways`, ids `lot<i>.p<k>`, kinds `park_path` /
`footway`, each end looked up once by `stateLotPaths`: another path, a street by `frontageOf`, or a dead end; a door on a
path states it in its walkway slot), and the loader hands them to the built model it publishes (`lots_producer.h
stateLotWalkways`); the bake's lot producer does the same only to log the faults. The lane level lays them as walks.

## 2. File map

Under `src/engine/procgen/city/roads/`:

| File | What it holds |
|---|---|
| `road_model.h/.cpp` | The model: types, planned and built layers, save/load, `dump`, `streetNameNear`. The built model holds the lane level and is saved whole by `lanes/road_model_cache`. Also the kerb parking runs and their questions, the kerb parking rules' reader (`kerbParkingRules`), and the door-to-road questions (ADR-0439). `line_frontage.h` declares the one step from a centre-line hit to a station and kerb (`frontageOnLine`, defined in road_model.cpp), small so the lot pass can include it (ADR-0474). |
| `road_model_parts.h` | The one set of junction, arm, street, node and stretch types (`ModelJunction`, `ModelArm`, `ModelStreet`...). The road file is read into them (`RoadLabGraph::junctions` / `streets`), the layout and the control plan write them, and `plannedRoadModel` copies them whole and adds what only the model knows. A junction or street named by another is a place in the same list, not an id string. `graph_index.h` holds the index types it shares with the lane graph. |
| `lane_turns.h/.cpp` | Which arriving lane takes which turn: reads `assets/data/lane_turns.json`; `turnsAtJunction` is the one function that applies it (ADR-0426). Also `laneLineBetween`, the one rule for whether the line between two lanes may be crossed (its lane kinds are the table's `lane_lines`). |
| `control_signs.h/.cpp` | The stop, give-way, do-not-enter and one-way signs that follow from stated facts (ADR-0424). Also ADR-0419's stop-line rule, `stopLineFromMouth`. |
| `lane_graph.h/.cpp` | The lane graph's types (paths, sections, junctions, crossing points), its save/load (LGRF), `validate`, dump, reach. It holds where a car can be and the ids the model is asked by, no fact about what the road is and no regulation (ADR-0436, ADR-0437). Beside the paths, in a list of their own, the walks people take and where their ends meet (`Walk`, `walkJoins`, `reachableWalks`; ADR-0466): nothing that reads `paths` meets one. A place on it: `PathPoint`, `WalkPoint` (ADR-0467). |
| `route.h/.cpp` | THE ONE ROUTE-FINDER (ADR-0467), moved here from the driving: one search (`Search` in route.cpp: A* over one binary heap, ties to the lower node number, a straight-line lower bound) over a mover's steps. Cars: `findRoute` over the lane paths (lane changes, closed turns and chains, learned section times; legs, turns, the lane-changes table and its questions `lanesToCross`, `lastChance`, `wayAhead`, `remainingFrom`; `routeHash`). Walkers: `findWalkRoute` over the walks, either way along each, joined only where the model says, the mover's open walks from `RoadModel::openTo` and its waits from the path users' table (`bindWalker`). A vehicle class is bound by the driving (`apps/driving/route.h bindRouteClass`) and passed in. The city sim's walkers, in every tier, walk its routes (piece 6 W3, ADR-0480). |
| `road_builder.h/.cpp`, `lanes_builder.cpp` | The roads module's one builder interface and the lanes builder behind it (ADR-0089). |
| `road_entity.h/.cpp`, `road_net_internal.h` | The engine's road entity (`RoadGraph` + look + plan). |
| `ground_grid.h/.cpp` | Absolute ground heights on a grid, as the builder hands them back. |
| `lanes/road_graph_spec.h/.cpp` | Reads the road file (`RoadLabGraph::fromJson`) and every section in part 4. Reads the road class table from `assets/data/road_classes.json` once (`roadClasses()`; `readRoadClassRow` is the one reader of a class row, for the table and for a road file's `classes`). `classOf` maps a road to the engine's class from its row. |
| `road_problem.h/.cpp` | A problem the road build found, as a record: its kind (a list with a name, a meaning and a sentence each), the ids it concerns, its place, numbers and words. `problemLine` is the one function that writes a problem's sentence (ADR-0438). |
| `lanes/road_layout.h/.cpp` | The road file's layout: junctions and streets with ids, the text writers used by `lanes_tool layout`, `keyedRoadFileText` (what the bake key hashes) and `laneLevelKey`. |
| `lanes/control_plan.h/.cpp` | The control plan: each junction's control and each road's limit, from the placement policy. Also `findJunctions`, the layout's grouping of road ends into junctions by the `rules.storage` rule. Rework: the design wants the planner to state these, not group them (step 9 / design "deleted at the end"). |
| `lanes/lanes.h/.cpp` | The lanes pipeline: graph to lanes to profiles to pavement to surfaces to terrain. `Result` holds the paint's junction boxes (`junctionBoxes`). The ramp-end check still uses an angle test (`kJunctionParallelForGraph`). |
| `lanes/lane_expand.h/.cpp` | Roads become lane ribbons: pockets, ramps composed from their gore anchors over their whole stated length (`rampEndDistances`). The builder's `Lane` is `LaneFacts` plus its edges and working data (`sweptFrom` / `diesInto` for pockets and ramp lanes; ADR-0435); its centre line is its stretch of the build's one `LaneCurves`, read through `xy()` and `s()` (ADR-0438, part 1). The streets a lane runs along and the junction at each end are held by the model's lane only (`ModelLane`, filled in `builtRoadModel`; ADR-0434). `ribbonLane` is the one way a lane is laid (the walks too). A footway's lane is of kind `footway` (ADR-0465). |
| `lanes/walk_lanes.h/.cpp` | The walks for people on foot (ADR-0465): `layWalks`, the build's last step, lays each kerb's sidewalk lane and each ring pair's corner walk along the drawn sidewalk (`Pavement::sidewalk` and footways' paving, kept `kWalkFromKerb` in from its edges, seams up to `kSeamBridge` closed) into the build's `LaneCurves`, as `LaneSet::walks`; `Result::walkCorners` says which pairs got none and why. Nothing is drawn from them; the lane level lays them as walks (ADR-0466). |
| `lanes/deck_mesh.h/.cpp` | Meshes and paint. `junctionAreas` gives each stated arm's box edge and stop line (`kStopLineBack`, 5.2 m, outside the box) by the one rule, `boxEdgeBack` in `road_model_parts.h`. The rule is the square rule, or at a sharp angle the arm's kerbs clear of the crossing road, within the room to the next junction's facing arm; `findJunctions` asks the same rule with the same room, so the grouping never moves (ADR-0471). The arms the room stops short are `ArmStop::squeeze`, said as model problems. The paint's lane lines, stop bars and arrows are read off `builtRoadModel` and listed in `RoadPaint` (ADR-0431). `paintFactsKey` is what the paint reads, for the bake key. **Open in step 9**: draw corners to a stated radius. |
| `lanes/deck_height.*`, `vertical_profile.*`, `road_design.*`, `terrain_conform.*`, `terrain_recipe.*` | Heights: profiles along spines, blended decks, the planner-shared vertical design, cut and fill. |
| `lanes/pavement.*`, `geom2d.*`, `polyline_ops.*` | 2-D geometry: footprints, the one triangulation, the Clipper2/CDT seam, polyline helpers. `buildFootprints` draws each road's bands (asphalt, shoulder, sidewalk) from its spine and mitres them where the road file states two ends meet at a kink, at one level (`bandKinks`, `mitrePieces`, `kMitreLimit`; ADR-0472): the outside of a bend is paved to the bisector, not cut square. `smoothTangentAtStation` (polyline_ops.h, ADR-0473) is a line's direction turning smoothly between its points (a Catmull-Rom spline's), for setting points out sideways from a line without a jump at each of its points. |
| `lanes/interchange.*` | Diamond interchanges (ramps at the streets a freeway crosses), and system interchanges (`systemInterchange`: a freeway crossing a freeway, its loop ramps of 60 m and corner links of 80 m set out in the through freeway's frame). That frame turns smoothly since ADR-0473 (`Frame::at` on `smoothTangentAtStation`); before, a ramp 90-150 m out jumped 2-5 m sideways at each point of a curving freeway (river_town's eight interchange ramps; its file was set right with `lanes_tool reframe`). The solid stretch before a stop line is handed in (`DiamondOptions::solidBeforeStopLine`): the planner's `SceneOptions::solidBeforeStopLine`, which it also writes into the road file's `lane_rules` (ADR-0435). |
| `vehicle_size.h/.cpp` | The body a road is checked against per vehicle tag (`VehicleSize`, `addVehicleSize`), and how a wheelbase and a tightest turn follow from the vehicle data. The driving library derives its classes with the same functions. |
| `lanes/lane_level.h/.cpp` | **The lane level as a table built from the model** (`buildLaneLevel`, ADR-0426): the last step of `assembleRoadModel`, which fills `RoadModel::laneLevel` (ADR-0429). The header lists where each part comes from. Also `layWalkways` (ADR-0475): the stated walkways laid on a built model's lane level, after the build, by `stateWalkways`; needs no `Result`. |
| `lanes/lane_graph_build.h/.cpp` | What is left of the old builder: report types (`LaneGraphBuild`, `CornerFault`), `cutFolds`, bend profiles, kinks, the vehicle sizes. Has the `kLaneGraphBuildVersion` cache version. `LaneGraphBuild::problems` is a list of records (`RoadProblem`), not sentences (ADR-0438). |
| `lanes/lane_graph_connect.h/.cpp` | Connector shaping and `fitConnector`, the per-class corner-fit test that produces corner faults; `reachReport`. Step 9 will change what it reports, not delete it *(unsure)*. |
| `lanes/lane_graph_crossings.h/.cpp` | Crossing points: where two paths' bodies could touch (a kept measurement). |
| `lanes/swept_body.*` | The ground a long body covers on a bend. |
| `lanes/road_model_cache.h/.cpp` | **The one maker of a level's road model**, `obtainLevelRoadModel` (ADR-0435; part 1), and cache/roadmodel: the built model (lane level inside) saved under one key. Also `levelVehicles` (the level's fleet as the roads read it) and `levelRoadModelCacheDir`. The roads' one cache outside the bake (ADR-0429). |
| `lanes/city_producer.*`, `lots_producer.*` | The bundle producers: road meshes, ground, the drawn deck and pavement holes; then lots. `lots_producer.h` turns the level's road model into the roads as the lot pass reads them (`lotRoadInputs`: the model's streets by id and the deck, ADR-0430); it makes no model of its own. The lot twin (`road_twin.*`, `lotNav`, the sections roads/twin and roads/lotnav) is deleted. |
| `lanes/block_audit.*` | The lot pass run headless and audited. |
| `lanes/lanes_export.*` | glTF, plan SVG, stats JSON, lane graph plan pictures. |

Tools: `tools/lanes_tool.cpp` (all subcommands; its top comment lists them, with every `layout` flag since ADR-0438)
and `city_plan` (the planner; `city_plan controls`). One-off edits to a frozen road file (ADR-0473), each writing only what it names, in place, through
road_layout.h's text writers: `layout <file> --state-arm J ROAD --write` (a road's end at a junction's point stated as
one of its arms, in its ring, with the through pairs the rule gives it; then the layout), `layout <file> --split-junction J
R1,R2,... --write` (ADR-0477: J's points holding those roads become a new junction at the list's end, each part made by
the grouping's own `factsOverNodes` via `junctionOverNodes`; then the layout; a street the file names keeps its name),
`reclass <file> <class> <road>
... --write`, and `reframe <file> <carriageway a> <carriageway b> <ramp> ... --write` (a system interchange's ramps set
out again in the freeway's frame turning smoothly). `lanes_tool lanegraph` prints the lane level's problems counted
by kind, each kind with what it means.

Tests:
- `lanes_tests`: `test_lanes.cpp`, `test_road_model.cpp`, `test_control_plan.cpp`; unit scenes in
  `assets/lanelab/*.json`; `tests/lane_level_scene.h` gives a scene the lane-level facts.
- `run_tests`: `test_lane_graph.cpp`, `test_street_signs.cpp`, `test_street_furniture.cpp` and more.
- `driving_tests`: lane tables laid by hand inside a small road model the test states its facts on
  (`tests/driving_test_roads.h`).
- `level_tests`: `test_traffic_lab_v2.cpp`, `test_levels_playable.cpp`.

## 3. Who reads the model today

- **Street names, name signs, the map**: planned layer (ADR-0420). Loader: `mapStreetsOf`, `signCornersOf`,
  `attachSignBlades`.
- **Lights and furniture**: each junction's stated control (ADR-0421). `planStreetFurniture` (street_furniture.h)
  stands one pole at the mouth of each arriving arm of a lit junction (`junctionLit`; a road file stating no control:
  junctions of four arms or more), and marches lamps, street trees and kerb props along each kerb of each street's
  centre line that has a sidewalk there.
- **Control signs**: `control_signs.h` over stated arms and regulations (ADR-0424). Placed at load and attached to
  arms.
- **Routing**: cars by the one route-finder over the lane paths (`findRoute`), people by it over the walks
  (`findWalkRoute`). The street map that every agent once routed on (`LevelRoadModel::streetMap`, ADR-0425) is gone
  (#306). The light probes (`placeProbeSites`), the day/night footprint and the signs' carriageways
  (`carriagewaysOf`) read the model's streets.
- **Lane level / driving v2**: the model's own `laneLevel`, filled by `assembleRoadModel` (ADR-0426, ADR-0429). The
  host makes nothing: it hands the driving the lane level of the built model the loader published in `LevelRoadModel`
  (ADR-0435), and warns when a vehicle class carries a tag the roads were not checked for. Only the levels naming
  `citysim.drivingRules` drive (traffic_lab, brain_lab, river_town).
- **The city sim holds the model on every lanes level** (ADR-0480), whether or not it drives: `CitySim::setRoads`, given
  `LevelRoadModel::builtCity()` (the first city's built model), and `CitySim::roads()`. Its people walk the lane level's
  walks (#306 piece 6, W3): every tier's walker holds a `WalkRoute` from `findWalkRoute` (`Agent::route`, `leg`,
  `distOnLeg`), placed by `LaneGraph::pointAt`/`heightAt` plus its own file across a sidewalk, crossing where its route
  crosses (`CitySim::nextCrossing`, `crossingGate`). There is one walking step; the street map's is deleted.
  `RT_WALK_REFUSED=1` prints each walk the sim could not route, and why.
- **The driving holds the road model** (ADR-0436): `Driving(const RoadModel*)`. It reaches the lane level through it
  and asks the model for a junction's control, a path's posted limit and road class, a connector's turn and a stated
  merge (the table in section 1). The lane table's copies of those five are deleted, and since ADR-0437 its
  regulations too: the driving binds what `regulationsAlong` answers. The driving's tests build a small model through
  `tests/driving_test_roads.h` (`modelOf`, `stateStreet`, `stateControl`, `stateTurn`, `stateMerge`,
  `stateRegulation`, `stateSignAtEnd`).
- *(Gone in ADR-0461: the old driving's lanes, `LevelLanes` / `TrafficLane` and the lane match `lane_map.*`, the
  loader's second laying of the lanes for it.)*
- **Placement policy**: the planner writes controls and signs into the road file from `control_policy.lua`; the
  model only reads what was written.

Still working roads out for itself:
- *(Gone in step 9a, ADR-0428: the paint's own junction boxes. Each arm's box edge and stop line now come from the
  stated junction through one function, `junctionAreas` in `lanes/deck_mesh.h`. The rule: back from the mouth by
  the widest crossing road's half width (since ADR-0471: or until the arm's kerbs clear it at the angle they meet,
  within the room to the next junction), then `kStopLineBack`. The paint, `ModelArm::stopLine`, the lane level and the
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
- **The planner's scoring**: its commute sample takes the quickest way over the plan's own road graph (a plain
  Dijkstra in `plan/city_plan.cpp scorePlan`), not over the model.
- **Bus-stop next-stop names**: `streetNameNear` finds the nearest named street within 40 m. The model answers it,
  but by position.
- **The count-the-arms rule** (four arms or more: lit) stands where a road file states no control
  (`StreetFurnitureParams::controlsStated`).
- **The old driving (v1)**: reads the lane match (`LaneMap` in `src/apps/citysim`), tied by road, direction,
  stretch and slot since ADR-0425. Its own junction boxes and stop lines are listed in the design doc; not checked
  here *(unsure)*.
- **Kept measurements, allowed by the design**: crossing points, bend profiles, the corner-fit test, and the ramp-end
  angle check in `lanes.cpp`.

## 4. Road file sections (`assets/lanelab/planned/<level>_lanelab.json`)

| Section | What it states | Bake key sees it? |
|---|---|---|
| `edges`, `classes`, `terrain`, ramps, pockets, connectors | Roads, their class rows stated on top of the table (`assets/data/road_classes.json`, whose own bytes the key hashes too), ground. An edge may state `"sidewalks": "both" \| "along" \| "against" \| "none"` (ADR-0464; not stated: both): which kerbs have its class's sidewalk, named by the traffic beside each. Only the road model reads it; the build still paves both kerbs, and the model says so in a problem line. No shipped file states it. | yes |
| `rules` (incl. `storage`, default 6.5 m) | Builder tolerances. `storage` is the waiting room between two junction points below which they are one junction (`findJunctions`). | yes (only `streets`, `divided`, `junctions`, `arms_in_ring`, `lane_rules` are cut) |
| `carry_on_control` | The control the plan gives carry-ons (control_plan.cpp) | yes *(unsure what reads it at load)* |
| `junctions` | Id, `at`, arms in their ring (street, heading, regulations; see below), `through` (by place in that list), `inside`, `control`, `why`, `nodes` (step 7's points) | no |
| `arms_in_ring` | `true`: every junction's arms are listed in their ring (ADR-0427; see below) | no |
| `streets` | Id, road, stations, the junction at each end, `carries_on`, `name`. A street's end is at a junction or carries on, never both (ADR-0435): where a junction is stated the junction's through pair says how the street runs on, and no `carries_on` is written for that end. | no |
| `divided` | Pairs of roads that are one divided road | no |
| `lane_rules` | `{"solid_before_stop_line": 20.0}`: the stretch before a stop line where lane lines are solid. The number's one home: the planner writes it (`SceneOptions::solidBeforeStopLine`, the number its ramps are laid by) and the lane level and the paint read it (ADR-0435). | no |

The lane graph's key (`laneLevelKey`) hashes `junctions` (without `nodes`), `arms_in_ring`, `streets`, `divided` and
`lane_rules`.
It is one part of the built road model's one cache key (`roadModelCacheKey`). Since ADR-0431 the paint reads the junctions' controls, arms and through pairs, the streets (not their names), `arms_in_ring` and `lane_rules`, so writing those re-bakes the city (`paintFactsKey`); a street's name, a junction's `nodes` or `why`, and `divided` still change no bake. Only traffic_lab, river_town and agent_lab (since ADR-0436) have `lane_rules`. Every
other road file gets no lane graph, and the error names `lanes_tool layout <file> --lane-rules 20 --write`. Write
facts with the tools (`lanes_tool layout ... --write`, `--names`, `--nodes`, `--lane-rules`;
`city_plan controls <file> rules/lab.lua --layout --write --controls-only`), never by hand.

**Writing the layout again changes nothing** (ADR-0435). `lanes_tool layout <file> --write` on a file that has not
changed gives the same bytes: it keeps each junction's `nodes` where they are (and so their order), writes `through`
back before them, and states no carry-on at a junction. It prints "the file is unchanged, byte for byte" or "CHANGED".
`road_model_19` holds this for traffic_lab, river_town and agent_lab. `--fresh` starts the layout over and drops the
nodes; `--nodes --write` finds them again. `--lane-rules N --write` writes `lane_rules` where the layout writer leaves
it (before `streets`), so one write is final (ADR-0437; the same test).

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
- **`assets/data/road_classes.json`** (ADR-0438): the road class table, twelve rows. Each class's lane width and
  counts, shoulder, sidewalk, median, rank, grade, kerb parking, barriers, whether its arms at a junction are marked
  (`junction_markings`), and what the builder once asked the class's name for: `engine_class` (which of the engine's
  six driving classes it is), `limited_access` (a freeway: crossings are bridges, a median wall between two of them),
  `default_barriers` (walls and guardrails where the row states none) and `plan_colour`. The file's own comment says
  what every key means. `like` starts a row from another (the mountain road is the rural road graded differently).
  Read once by `roadClasses()` (`lanes/road_graph_spec.cpp`); there is no copy in C++, and a missing file stops the
  road build with an error naming it. A road file's `classes` go on top, row by row, through the same reader
  (`readRoadClassRow`); a class name the table lacks starts from the `default` row (`local`). The file's bytes are part
  of the city's bake key (`roadClassTableHash`), so editing a row re-bakes. No code compares a class NAME any more;
  what is left is listed in ADR-0438 (the interchange maker gives its ramps the class named `ramp`, and two places
  compare a class's `rank` to 3).
- **`assets/data/kerb_parking.json`** (ADR-0439): how a street's parking band is cut into spaces. Two numbers:
  `space_length` (7 m) and `clear_of_stop_line` (10 m). Read once by `kerbParkingRules()` (road_model.cpp); no copy
  in C++, and a missing file means no street has a space. Its bytes are part of the saved road model's key
  (`roadModelTablesHash`, with the turn table's), not of any bake key: nothing baked reads the spaces.
- **`assets/data/path_users.json`** (ADR-0464): who may use which kind of path. `movers` (each with its properties:
  `vehicles: true` stands for every vehicle tag of the level's fleet), `kinds` (each kind of path, a builder lane kind or
  a walking kind the lane level will lay, with the movers that may use it; a kind not listed is open to nobody) and
  `dead_end.joined_by` (the kind of path joining a dead end's two sidewalks; null: not joined). Read once by
  `pathUsers()` (road_model.cpp); no copy in C++, and no C++ names a mover or a kind. Its bytes are part of the saved
  road model's key (`roadModelTablesHash`), not of any bake key. Its kinds are read by `RoadModel::openTo`, the one mover filter
  (ADR-0466). A mover may state `waits` (ADR-0467): `{kind: seconds}` a walker's route adds for each walk of that kind
  it steps onto (the walker's `crossing`: 10 s); refused on the vehicles' mover. The router's `bindWalker` reads both.
  A walkway (ADR-0475) names its kind from the same rows: `footway` (between buildings, across a plaza) or `park_path`
  (through a park), both the walker's only.
- `assets/data/streets.json`: name words and sign look (names only).

## 6. Checking a change quickly

Build: `cmake --build build-viewer --target lanes_tool lanes_tests` (add `viewer level_tests run_tests
driving_tests rt_bake city_plan` before anything that loads a level; stale binaries mislead, see part 7).

| Command | What | Wall clock (seen in step 8) |
|---|---|---|
| `lanes_tool build <file> --out <dir>` | Road meshes; compare `.glb`/`.svg` md5 before and after (stats.json differs only in timings) | ~12 s traffic_lab, ~36 s river_town |
| `lanes_tool model <file> --out <dir> [--door x y]` | model.txt, report.txt, model.svg (the kerb parking spaces drawn on it), parking.txt (every space: stations, where its car stands, the kerb lane beside it); prints the parking counts; `--door` prints where that point fronts the road, its sidewalk point, set-down point and nearest spaces | ~16 s traffic_lab, ~43 s river_town |
| `lanes_tool junctions <file> [--top N]` | The grouping's junctions, biggest first | seconds *(unsure)* |
| `lanes_tool layout <file> [--write ...]` | Layout facts; prints whether the bake and lane-graph keys stay unchanged | seconds |
| `lanes_tool lanegraph <file> --out <dir> --vehicle tag:radius:width:length ...` | lane_graph.txt, problems.txt, reach.txt, corner_faults.txt, svg; `--junction J` for a close-up | ~26 s traffic_lab, ~63 s river_town |
| `./build-viewer/rt_bake --status assets/levels/<level>.json` | Is the bake current (must not change) | seconds *(not timed)* |

The fleet for `--vehicle`, as step 8 used it: `bus:7.30:2.55:11.45 car:6.15:2.05:5.57 goods:7.45:2.30:6.69
moped:1.92:0.70:1.70 motorcycle:2.73:0.70:2.09 truck:7.45:2.25:6.69 van:7.10:2.30:6.34`.

Tests (`RT_TEST_FILTER` is a substring of the case name):
- `./build-viewer/lanes_tests`: 130 cases, about 4 to 5 minutes whole. Filters: `road_model` (quick),
  `lanes_lane_graph_of` (the five scenes' lane-graph checks against what each scene is known to hold: `LaneGraphKnown`
  in test_lanes.cpp), `lanes_the_traffic_lab`, `connectors_of`.
- `./build-viewer/driving_tests`: 194 cases on hand-made graphs, about a minute *(roughly)*.
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

- **7 of the 13 lanes levels have no lane level, so no walks** (ADR-0480): light_rail_lab, metro_mountain, metro_planned (Glenn's work in progress: never touch), metro_v2_test, piedmont, rail_lab, wheel_lab.
  Their road files state no lane rules (all but wheel_lab also no ring order of their junctions' arms; rail_lab also not
  every control), so the loader publishes the planned layer only. They stay so by Glenn's decision.
  `level_every_lanes_level_hands_the_sim_its_road_model` names the tool run for each.
- **island_8_nature has its lane level** (ADR-0480 W1b and "W1b, j396"): its road file states its lane facts, and j396
  is fixed. The planner had broken one avenue into two spurs; it is now a crossroads at j396 and a T at P (new junction
  j1373, new edge p1_s144_1, both at the ends of their lists). The lane level passes its check, with 14,573 walks, 51
  parts on foot (19.8% of sidewalks in the largest), and a car gets round 50.2% of the lane paths. It is cached
  (`cache/roadmodel/241597b0593ad586.roadmodel`, 167 MB; building it takes 13-15 min). Its bake still runs out of memory in the lots producer under the 30 GB limit (22.8 GB
  resident at the peak), with the model cached and with `MALLOC_ARENA_MAX=4`. The memory goes in growing the lots, not
  in building the model again (#317). So the island has no current bundle, and a load grows its city cold. A lane
  level that fails its check is still never cached.
- **Step 9a's stop lines** (ADR-0428) sit 1–2 m further out than the paint's at most junctions. The old box was
  sampled every 2 m, so it usually ended a metre inside the crossing road. Corners are still drawn without a stated
  radius (step 9b). Since ADR-0471 the stop lines at sharp-angled junctions stand back further, which costs river_town
  turns that now cut a corner (fits no vehicle 175 -> 199, at j0, j14, j90, j119, j135). Step 9b should win them back.
- **Junctions too close for the angle their roads meet at** (ADR-0471, model problem lines, the planner's to fix):
  traffic_lab 9 arms (j0, j22, j24, j37, j70, j84), river_town and agent_lab 26 (j6, j7, j8, j14, j35, j37, j43, j56,
  j68, j72, j73, j75, j90, j91, j135). The room to the next junction caps the box edge short of clear, so most of
  these keep a kerb end out in the asphalt. Listed by arm in ADR-0471, pinned in `road_model_39`.
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
- **Street ends that meet another road with no junction stated** (problem lines since the review): none since ADR-0473
  (traffic_lab's s208 stopped 15 m short of j95's point; it now reaches it and j95 states it as its fourth arm). The 16
  (traffic_lab, the spokes at its ring) and 24 (river_town) that end 8 m under the freeway are dead ends since ADR-0469,
  each joined across by a turn-round.
- **Walks cut off by the junction drawing** (ADR-0469, ADR-0471): a corner gets no walk where a crosswalk's kerb end
  stands out in a sharp-angled junction's asphalt. `boxEdgeBack` now sets each arm back at the angle the arms meet, so
  this is down to traffic_lab 4 corners and river_town 13 (was 13 and 27): the junctions too close for their angle,
  above, and river_town j19, j116, j119, which the rule's straight strips do not reach. The wedges and notches of
  bare ground at a kink between two roads' bands (B2, traffic_lab 9, river_town 18) are gone since ADR-0472 mitres
  the bands. One river_town corner still reads "two pieces", not at a kink: j90 0-1 (arm 0's kerb end on a scrap of
  s228's sidewalk inside s227's carriageway). (j0 6-0 and j14 5-0, walks round a block face, went when ADR-0477 stated
  j0 and j14 each as two junctions.) Sidewalks cut off from the largest part on foot:
  traffic_lab 0, river_town 10 (14 until ADR-0473 made the freeway's turn-round loops tn0/tn1 ramps, with no
  sidewalks). The s337/s277 hairpin's open end is now paved over by the capped mitre.
- **Two traffic_lab ramps carry no car** (d0_0_a_on, d1_0_a_off). Their hosts' kerb lanes end at freeway joints
  j153/j154, whose stated turns all leave the paved road (since ADR-0472 the freeway's own straight on across those
  joints fits; the ramps' turns still do not).
- **river_town planner faults**: hairpin and gapped carry-ons (s277/s337 159°, s325/s344 4.9–10 m). x0_outR_link
  and x1_outR_link state a merge past the host lane's end. s320 carries on into s287 where no lane leaves. Dead-end
  spokes under the freeway ring (dead ends, ADR-0469). The planner no longer reproduces river_town's road file (ADR-0423). Car reach 41%
  without lane changes, 81% with: lane changes come first.
- **Staggered junctions stated apart** (ADR-0477): river_town/agent_lab j14 is now j14 (north) and j144 (south), and
  j0's south-east T is j145. The new ids are at the list's end; no other id moved. The street between j14 and j144 (s313)
  has only 3.9 m between its facing stop lines, the one stated exception in `road_model_39`. j90, j119 and j135 are left
  as single junctions (reasons in the ADR). A junction file list is no longer exactly what the grouping (`lanes_tool
  junctions`) gives for river_town: the grouping still chains these points.
- **river_town j103 stands 21 m in the air** (ADR-0473, not fixed): the design stage's height agreement lifts the
  junction from 6.5 m to 27.3 m over 30 rounds while the ground is 5-7 m, so every street leaving it is far too steep
  (s192 42 %, s334 41 %, s193 30 %, s335 23.7 %). The pump is at j90, 82 m away along s334: s334's end lies inside
  s195's band at j90's spread points, so the "tee" rule (crossingConsistency) holds it to s195's height while the node
  rule averages the ends, and the two never agree. Skipping tees between two arms of one stated junction (tried, not
  kept) brings s335 to 8.9 % and s192/s193/s334 to 12.3/3.3/14.0 %, but doubles river_town's deck cracks (316 -> 643)
  and step edges (102 -> 258). A design-stage sitting of its own; s335's file profile cannot fix it (at 12 % from j103
  it would meet the freeway's underside).
- **Outlines that still cross themselves**: traffic_lab none; river_town and agent_lab j75 (problem lines). Since
  ADR-0471 sharp-angled arms' stop lines stand back far enough not to overlap. It was 5 and 11.
- **The side rule** (lane_turns.cpp, `turnKindsFrom`): an arm with no through pair stated calls a turn near or far by
  which half of the ring the other arm lies in. Exactly halfway counts as far. traffic_lab: 56 junctions, 191 turns
  rely on it, 8 halfway notes. river_town: 43, 192, 25. At spread junctions it can call a turn near that a driver
  would call far: river_town j0's s2 into bv1_out is a 43-degree turn across the way, but its carriageway leaves the
  circle on the near side. It is listed per junction in the arm-order tables and is unchanged.
- **Deferred levels' road files are unmarked** (`arms_in_ring`). They load with a logged problem line and get no lane
  level until `lanes_tool layout <file> --arm-order --write` is run on them.
- **Deferred levels**: every lanes level but traffic_lab, river_town and agent_lab lacks `lane_rules`, so has no lane
  level and no walks: no city sim. Steps 5, 6 and 9 are open (design step table).
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
