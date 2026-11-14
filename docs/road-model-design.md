# One road network: design

Status: approved; steps 1 and 2 built (2026-11-13, ADR-0417) beside the old forms. Two fixes the comparison showed were
needed came before any reader moves (ADR-0418): a street has one lane count, with its kerb parking stated in its
cross-section (built); and which road ends make one junction follows one stated rule (ADR-0419: built for every road
file but traffic_lab's, whose builder still merges two of the rule's junctions into one). The comparison of the model
with the old forms is in docs/road-model-compare/.

## The goal

There is one description of a level's roads. The planner creates it, the road builder adds to it, and every other
system reads it. Nothing copies it into a second form, and nothing works out by measuring what an earlier stage
already knew. A new fact about the roads (a bus stop, a speed limit, a parking space) is added in one place and can be
found by everything.

## Words used here

- **Junction**: a place where traffic from different roads meets at one level.
- **Arm**: one road's way into or out of a junction.
- **Street**: a stretch of road from one junction to the next (or to a dead end).
- **Cross-section**: what a street is made of across its width: lanes each way, median, shoulders, parking, sidewalks.
- **Lane**: one line of travel along a street, with its real curve.
- **Connection**: how one lane leads to another: carrying on, turning through a junction, merging, or ending.
- **Control**: what governs a junction: lights, stop signs, give-way signs, all-way stop, or nothing.
- **Station**: distance along a street, in metres.
- **Kerb side / centre-line side**: the two sides of a lane. (This world drives on the left; the document never says
  left or right.)
- **Bake**: the step that builds a level's roads and buildings once and stores the result.

## What exists today, and what is wrong with it

A read-only inventory of the code (2026-11-13) found the same roads described in at least these forms. Each is worked
out from another, and none is the master.

| # | Form | Made from | Knows | Has lost |
|---|---|---|---|---|
| 1 | The city plan | the brief | junction nodes, streets, blocks and their uses, river bridges | (the source) |
| 2 | The road file | the plan | road centre lines, classes with lanes and sidewalks, ramps | the junction nodes, block faces, which crossings are bridges |
| 3 | The built roads | the road file | every lane's curve, heights, pavement | nothing, but it has no notion of a junction and finds them from overlapping lanes |
| 4 | The "road twin" | the built roads | a thinned node-and-link copy; agents route on it | lane shapes, lane counts, pockets, merges, speed limits, controls, heights of ordinary streets |
| 5 | A second twin for lots | the built roads | the same, with different widths | the same |
| 6 | The lane match | the twin and the lanes | which built lanes lie under each link | anything that did not match (pockets, merges) |
| 7 | The lane graph | the built roads | lanes, turns, conflicts, for the new driving | road identity, sidewalks, parking, names |

Besides these there are three separate models of a street's cross-section, two copies of the routing graph built from
the same data, a deck-height field, kerb outlines, and street names that are generated at load and stored nowhere.

What that costs, by the inventory's count:

- **About 56 places work a fact back out** that an earlier stage held, each with its own threshold.
- **The same idea has different numbers in different places.** "At the same level" is 1.0, 1.5, 2.0, 4.0 or 4.5 m
  depending on who asks. "Straight on" is 20, 30, 32 or 35 degrees. "One junction" is decided five different ways.
- **About 4,500 lines exist only to guess**, plus about 1,800 lines that make the twin, plus the junction and turn
  inference inside the lane graph.
- **Facts with no home.** Parking was invented by the twin (stated since ADR-0418). Signal phases are worked out
  separately by the old and new driving. Street names exist only in memory after load.
- **Faults that are really one fault**: phantom junctions from lanes that miss by 1.5 cm, three junctions 4 m apart,
  ramps with no way on, signal heads that disagree with the lights, buildings nobody can reach, 27% of river_town
  closed to the new driver.

## The design

### 1. What the one road network holds

Everything below lives in one structure, called the road model in the rest of this document.

- **Junctions.** Position and height, the arms in order round it, which pairs of arms are one street continuing
  through, its control, and its outline on the ground. WHICH ROAD ENDS MAKE ONE JUNCTION (ADR-0419), one rule: two
  junction points next to each other along a road are one junction when not even a car could wait between their stop
  lines -- the stretch between them is shorter than the road file's storage length (`rules.storage`, 6.5 m: a car and its
  gap); a junction point's stop line stands back from it by the widest other road's half width and 5.2 m. And by
  construction: where the two carriageways of a divided road (the file's `divided` list) meet one street, that is one
  junction however far apart. A place where a street only carries on or bends (two arms) is no junction at all: its two
  streets name each other (`carries_on`). A longer vehicle that cannot wait between two junctions is the driving's to keep
  out (it checks the room beyond for each vehicle class).
- **Streets.** The junction at each end, the centre line, the class, the cross-section, one-way or two-way, the speed
  limit, the name, and whether buildings may front it.
- **Lanes.** For each street: its lanes from the centre-line side to the kerb side, each with its kind (through, turn
  pocket, merge lane, ramp, bus, parking), width, real curve, heights, and the stations where it begins and ends.
- **Connections.** Lane to lane: carries on, turns through a junction (with the path it follows), merges into a
  neighbour over a stated stretch, or ends. Also which lines between lanes may be crossed.
- **Controls and signs.** Per junction and per arm: the control, signs, stop-line station, signal groups.
- **Crossings.** Where two roads cross without meeting: which goes over, the clearance, the bridge and its piers.
- **Ramps.** The road they leave, the road they join, the lane and station at each end, and the merge length.
- **Parking.** Kerb parking as part of a street's cross-section, and the spaces along it. Built (ADR-0418): the road
  class table states it (`parking`, the width of a band along each kerb, over the kerb side of the kerb lane: a planned
  local street's 6 m lane is 3.5 m to drive in and 2.5 m to park in), a road may restate it, and the twin's band, the
  sim's bays and the model's cross-section all read that one statement. It is not a lane of its own: the lanes each way
  are the travel lanes, the one lane count every reader uses. The spaces along it are still laid by the sim at load.
- **Walking.** Sidewalks as part of the cross-section, footpaths as streets of their own class, and crosswalks at each
  arm.
- **Rail.** Tracks, platforms and stations. Rail is already a class of road in the road file and stays in the model.
- **Blocks.** The land between streets, with which streets bound each block and what the block is for.

Every junction, street, lane and connection has an **id that never changes** once given. That is what lets anything
else attach to the roads: a bus stop is "street 41, station 120, kerb side"; a building's door is "faces street 41";
a sign is "arm 2 of junction 17".

### 2. Who writes what

Each fact has exactly one writer. Later stages add; they never rewrite and never copy.

| Stage | Writes |
|---|---|
| The planner | junctions, streets, classes and cross-sections, one-way, controls, speed limits, crossings, ramps, blocks and their uses, names |
| The road builder | each lane's curve and heights, stop-line stations, junction outlines, turn paths through junctions, merge stretches, crosswalk positions, bridges and piers, parking spaces |
| Load time | nothing about the roads. It may build look-up tables (see 4) |

The builder's job changes from "work out what is here" to "build what was stated, and report where the result does not
match". A lane that ends 1.5 cm from the lane it was stated to join is joined, and reported. A turn that does not fit
its corner is an error against the corner, not a turn quietly closed.

### 3. Where it lives

- In the road file, as written by the planner: the planned part of the model.
- In the level's bake, once, with the builder's additions: the whole model.
- The twin, the second twin, the lane match and the lane graph cache are not stored, because they no longer exist.

### 4. Look-up tables are allowed; second descriptions are not

Routing wants compact arrays. Finding "the street under this point" wants a grid. These may be built from the model at
load. The rule that keeps them from becoming form number eight:

- A table holds **no fact of its own**. Everything in it is an id into the model, or a number copied from it.
- It is rebuilt from the model, never saved beside it, and never edited.
- If a system needs a fact that is not in the model, the fact is added to the model by its rightful writer.

### 5. One network at two levels of detail

Routing across town does not need lanes; driving through a junction does. Both read the same model:

- **Street level**: junctions and streets. Used for trip planning, agents that are far away, buses, the map.
- **Lane level**: the lanes and connections of those same streets. Used by a driver near the player.

Because a lane belongs to a street, an agent can be handed from one level to the other without anything being
matched or guessed. This is what the trip planner and the far-agent timetables need.

### 6. Questions the model answers

Small and fixed, so readers stop walking raw data:

- the street, lane and station nearest a point; the height there
- the streets at a junction, in order; the turns allowed from a lane
- the lanes of a street; what a lane leads to
- a route between two places, at street level or lane level
- what is attached to a street or junction (stops, signs, names, doors)

## How to get there without breaking the game

One reader moves at a time. After each step every level still loads and plays, and the old form stays until its last
reader has gone.

| Step | What changes | Re-bake? | Built |
|---|---|---|---|
| 1 | The planner writes junctions and streets with ids into the road file. Hand-made road files get them from a one-off tool that writes today's inference into the file for a person to check. | No | Yes (ADR-0417). One list of junctions: the control plan's "junctions" list, extended. The bake key ignores the new parts. |
| 2 | The builder carries those ids onto every lane it lays and keeps its stop lines and junction outlines. The whole model is stored in the bake beside the old forms. A test compares the model with the twin on every level and lists the differences. | Bake gains a section; meshes unchanged | Yes, with two differences: the model is stored in its own cache file (cache/roadmodel), not in the bake, so nothing re-bakes; and the builder has no junction outline to keep, only the paint's junction boxes (a stretch along each street), which it keeps. The comparison ran on three levels, not every level. |
| 2b | Two fixes before any reader moves: a street has one lane count (the road file's), with its kerb parking stated in its cross-section; and which road ends are one junction follows one stated rule. | No | The lane count, yes (ADR-0418). The nav graph is still built from the bake's road graph, whose edges carry no road id, so each nav link gets its road's stated lanes through the lane match (the road whose centre line the link lies on): an interim until step 7. The current city sim drives differently (streets gained travel lanes); its pinned values were retaken. The junction rule: built (ADR-0419) and written into nine road files; traffic_lab keeps its old grouping, because the paint's own "boxes within 12 m are one junction" merges two of the new rule's junctions (one with lights, one with a stop) into one, which the lab's rule set cannot drive (until step 9 paints the stated junctions). |
| 3 | Readers that only label things move first: street names, name signs, the map. | No | Yes (ADR-0420). The model has a planned layer, read from the road file alone at load and published level-wide (`LevelRoadModel`); names are stated in the road file (`streets[].name`, written once into the existing files, bake keys unchanged) and are attachments of the model, as are the name signs. Differences: a name's run follows every stated through pair, carry-on and divided pair, so some runs are long and cross classes (and in metro_v2_test two junctions have one name on every arm); the attachment kinds live in their own data file (assets/data/road_attachments.json); a junction the plan spread over several points gets one sign post, not one per point; the island keeps its place names. Sign posts are walker obstacles, so river_town's pinned v1 state moves with them (not retaken here). |
| 4 | Street furniture, signal poles and lights read the model's junctions and controls. Signal heads and the lights now agree. | No | Yes (ADR-0421). Every road file states each junction's control: the one-off tool (`lanes_tool controls`) wrote what the city sim's old rule lit, once (lit when any of a junction's nav nodes was); traffic_lab keeps its planned controls. The city sim's signals and the furniture's poles both read the stated control; the poles stand one per arm of a lit junction; lamps, trees and kerb props keep clear of the model's junctions; a street's planting follows its stated run. Differences: the nav graph has no junction ids yet, so its nodes are tied to the model's junctions by position in one helper (`LevelRoadModel::junctionAt`), an interim until step 7; the bake key no longer sees the junctions list at all (controls and signs included), and the lane graph keys the control plan itself, so traffic_lab re-baked once (meshes byte-identical); stop and give-way signs are not drawn anywhere today and were not added; a road file with no control plan (the planner's fresh output, until the tool runs) still takes the old rule. |
| 4b | The placement policy, part one: every junction's control by shared rules (see "Placement policy" below). The planner states controls in every new road file; existing levels are only compared. | No | Yes (ADR-0422): the lab's rows moved to assets/scripts/rules/control_policy.lua, shared; the planner and the island scene writer state controls; `city_plan controls ... --compare` sets the policy beside a file. No existing road file changed. |
| 5 | Lots, block frontage and door facing read the model's blocks and streets. The second twin is deleted. | Lots re-bake | |
| 5b | The placement policy, part two: the signs that follow from controls (stop, give way, do not enter), then the rest of the street furniture, each as rows of the policy, reading the block uses step 5 gives. | No | |
| 6 | Buses, rail riders and parking attach to streets by id, reading what the policy placed. Parking becomes part of the cross-section. | No | |
| 7 | Routing reads the model at street level. The twin and both routing-graph copies are deleted. This changes what the current city sim does, so its pinned test values are retaken once, knowingly. | No | |
| 8 | Lane-level driving reads the model's lanes and connections. The lane match and the lane graph's inference are deleted; its measuring of conflicts and bends stays. The one-agent lab's car moves to the new driver here. | No | |
| 9 | The builder paints junctions from the stated junctions and builds corners to a stated radius, so every stated turn fits. | Every level | |

Steps 1 and 2 are the foundation and change nothing a player can see. Step 9 is the only one that changes the drawn
roads everywhere.

## Placement policy

Words: a FACT here is something about the road that changes how people behave on it: a junction's control (traffic
lights, an all-way stop, a stop or give-way sign on the minor road, or nothing), which arm stops or gives way, which way
a one-way street runs, where buses stop. An OBJECT is something that stands on the street: a signal pole and its head, a
stop sign, a tree, a lamp, a bin, a hydrant, a bench, a newspaper box. The KERB is the line where the road meets the
pavement; objects stand along it.

**Two kinds of thing, decided in order.** First the facts, by rules over what the road model states. Then the objects:
some follow from a fact (poles and heads at a lit junction, a stop sign at an arm that must stop, a give-way sign, a
"do not enter" sign at the exit end of a one-way street, a shelter at a bus stop); the rest serve the street, spaced by
the street's type (trees, lamps, hydrants, bins, newspaper boxes, benches).

**Every rule is one row of data, always with the same parts:**
- what it places: a kind from the attachments table (assets/data/road_attachments.json);
- what it attaches to: a junction, an arm, or a street on one side;
- when: conditions over STATED facts only (the classes meeting, the number of arms, one-way, the pavement's width,
  the neighbouring block's use), never over something measured;
- how many, or at what spacing;
- where exactly: the set-back from the kerb, the distance from the junction;
- what it keeps clear of;
- a priority;
- its source: a citation (a manual, a vehicle code) or, marked as such, a guess to tune.

**Conflicts.** The kerb along each street is a line whose stretches are claimed in priority order: law and safety
first (signals, stop and give-way signs, do-not-enter), then transit (bus stops and shelters), then utilities
(hydrants, lamps), then amenities (bins, benches, newspaper boxes), then trees. A rule that finds no room is skipped
and reported, never squeezed in.

**Results are attachments on the road model.** Facts that change behaviour are written into the road file by the
planner, so a person can read and edit them, and the bake never sees them. Decoration is generated from the policy
when the level is built.

**Order of building.**
1. Controls (step 4b, ADR-0422).
2. Lots and blocks (step 5), which give the policy each block's use.
3. The signs that follow from controls, then the rest of the furniture as rows (step 5b).
4. Buses, rail and parking read what the policy placed (step 6).

**Part one, built: the controls** (ADR-0422).
- The rows live in assets/scripts/rules/control_policy.lua, shared by every city.
- A rule set refers to it: the lab's says `placement = { policy = "rules/control_policy.lua" }`. Each row's control and
  signs are checked against that rule set's.
- The planner states controls in every new road file, and so does the island scene writer.
- Each junction records the row that decided it, as its `why`.

**What the control rows can see today.** These are the facts the control planner works from (control_plan.h
JunctionFacts). Some are stated in the road file; others are measured:

| Fact | Stated or measured | What should replace a measured one |
|---|---|---|
| How many arms a junction has | Stated: the junction's arms list | — |
| Each arm's road class | Stated: the road file's class of the edge | — |
| Lanes arriving and leaving by an arm | Stated: the road file's lane counts (ADR-0418) | — |
| Each road's posted limit | Stated: from the policy's class limits, written into the edge | — |
| Whether a ramp's free end is in the junction | Stated: the ramp's `from`/`to` naming the street it ends on | — |
| Which approaches are MAJOR among equals (the road running straight through a T) | Measured: two arms arriving within kStraightOnDegrees of head on | The junction's stated `through` pairs (step 1). Not done here: it would re-decide the lab's junctions. |
| Which road ends make one junction | Stated since ADR-0419 (the layout's junction list). The control planner can still find them itself from road ends, as the lab's own planning does. | The layout's list, always: planLayoutControls does this for new road files. |
| `apart` (no lights within so many metres of another) | Measured: distance between junction centres | Distance along the street between stated junctions. No row uses it today. |

## What gets deleted at the end

- The road twin and the second twin for lots, and the code that thins, splits, welds and re-joins them.
- The lane match.
- The lane graph's junction finding, carry-on junctions, lane-order measuring and turn guessing.
- The control plan's own grouping of junctions (one stated rule since ADR-0419, still applied to the road ends rather
  than written by the planner as it plans; the paint's own "boxes within 12 m are one junction" is a second rule, until
  step 9, and where they differ the stated one cannot yet be driven: traffic_lab).
- Two of the three cross-section models (since ADR-0418 the road_rules one, DesignRules, is read by no system, only its
  own tests; the twin's RoadSpec bands are made from the road class table).
- The per-system inferences listed in the inventory: junction boxes and stop lines in the old driving, signal
  junctions and phases, street chains for names and trees, block frontage votes, and the rest.

The inventory puts this at roughly 6,000 lines. That number is from a reading of the code, not from doing the work.

## What this does not fix by itself

- **The driving brain.** The model gives a driver correct lanes; how a driver behaves is separate work.
- **Tight corners** until step 9.
- **Bad plans.** If the planner puts a ramp on the wrong side, the model will state it faithfully. The gain is that
  the builder can now refuse it by name.

## Open questions for Glenn

1. **Pace.** Steps 1 and 2 first, then stop and look at the comparison test before moving any reader? I recommend this.
2. **The island.** It has several towns in one level, and today only the first city gets some of the forms. I propose
   one model per level covering all its towns and the roads between them.
3. **The new lane graph's data types.** They are well made and could become the lane level of the model, keeping the
   driving library's reading code. Or the lane level could be designed fresh. I recommend reusing them.
4. **Step 9.** It re-bakes every level. It can wait until the rest is done, or go early for river_town alone so the
   one-agent lab gets a working freeway.

## What has and has not been checked

- Read by me in the code: the plan's junction nodes, the road file's contents, the builder's lane records, the twin
  and how it is made, the lane match, and the lane graph's inference.
- From the inventory, not re-checked by me: the count of 56 inferences, the line totals, and the full list of readers.
- Not measured: how long any step takes, or how the stored size changes.
