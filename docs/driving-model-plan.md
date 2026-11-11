# The driving model, rebuilt — plan

Status: **draft for Glenn's review** (2026-11-11). Nothing in it is built yet.
Replaces the driving parts of `vehicle-agent-plan.md` (ADR-0061) and the traffic layers of #225's first plan
(#218–#221). The people side (`agent-sim-plan.md`, roles, goals, schedules) stays and feeds this.

## Why a rebuild

The current model can't be patched. A car's real state is route bookkeeping (link, distance along it, lane number). Everything else
is computed from that, and each part computes it differently:

- **Drawing:** lane offsets, two turn-curve rules, and a pull-out that only changes the drawn car.
- **The nose:** a separate heading, turned toward the path at a limited rate.
- **Following:** route order — who is further along the same lane.
- **The vision wedge:** a straight ray from the drawn position along the nose.
- **Junction rules:** distance to the node, and which way other cars point.
- **Crashes:** body shapes.

When these disagree, cars wait on each other. Timers then break the standoffs (crawl after 6 s, freeze 1–2.5 s after a touch,
the "tow-truck escape"). Routes don't know which way a car faces or how tightly it can turn, so the drawing hides moves no car
could make. About 3,400 lines of special cases, each one written for a situation someone saw.

Measured in the traffic lab on the current model (commit 1a315bf8, 289 drivers over 2 minutes):
- 166 stops of over 1 s inside a junction, 1,987 s of standing in all (88 of them "a body in the wedge");
- several cars standing for over 100 s: deadlocks;
- 78 of 304 trip starts swinging round more than 90 degrees out of their space;
- 3 cars spinning on the spot.

## Principles (from the interview, 2026-11-11)

1. **Traffic is emergent.** It is what many individual drivers make by using the same roads under the same rules at the same
   time. No code is about "traffic". Never a fluid, never spawned to fill a road.
2. **Every vehicle is someone going somewhere.** It's driven by an agent with a need: a person, a couple, a family; a cab driver
   at work, a bus driver on shift, a delivery. More traffic comes from more people with reasons (tourists, hotels, island
   commutes: #213, #223). Every vehicle can answer "who's driving, who's aboard, and why".
3. **Driving is the rules of the road, as data.**
   - Rule sets live in Lua and can be switched per level or region.
   - The default is driving on the left under Japanese rules (or a mix); others can be tried later.
   - C++ supplies the general machinery (perceive, decide by rule, act within the vehicle's limits) and never a rule for one
     situation.
   - Test of the design: if a behaviour can only happen because code was written for that exact case, the design is wrong.
4. **One truth per vehicle.** Where the car is, which way it points, and where it's going are one state. Drawing, following,
   perception, junction rules and physics all read that state.
5. **Every move is drivable.** Paths come from the road data and respect each vehicle's turning circle. No pivots, no slides,
   no drawn swings. Pulling out of a space is a path too.
6. **A level of detail drops richness, never rules.** Far from the player drivers are robotic (no personality, cheaper
   perception, a slower update), but they are the same people on the same trips under the same rules. A car changing level
   never jumps or changes its plan.
7. **No escape hatches.** No unstick timers, teleports or crawl-through-the-box. Anything that has to break a standstill is a
   bug, counted.
8. **Outcomes are emergent too.** Near-misses, horns, crashes and what follows come from drivers' attention, judgement and
   personality meeting each other. They're never scripted.
9. **Scrutable.** Any car can say what it's doing and why: the rule behind it, and its last few decisions.
10. **Judged by real numbers, not by eye alone** (see Measurements).

## Prior art we build on

The design adopts established traffic-simulation fundamentals instead of inventing them. The research report is in the session
scratchpad; its sources go in CREDITS.md when the code lands.

- **SUMO** (Eclipse; Erdmann & Krajzewicz, "SUMO's Road Intersection Model", LNCS 8594, 2014):
  - a lane graph with junction "internal lanes" as the only geometry a car stands on;
  - a per-junction conflict matrix (foes, who yields to whom);
  - right of way decided by drivers: each posts its arrival and leave times on the entry links ahead, and enters only if its
    window clears every foe's by a safety gap;
  - "link leaders" inside the junction: first in leads, the relation can never loop, and the follower keeps a virtual gap to the
    crossing point;
  - a don't-block-the-box check that gives up the car's claim when it fails;
  - impatience that grows with waiting, and first-come-first-served at all-way stops.
- **IDM** (Treiber et al.) for following, capped by a **Krauss** safe speed so following can never collide.
- **MOBIL** (Kesting, Treiber, Helbing) for discretionary lane changes, with **LC2013**-style strategic urgency for route-driven
  ones. A lane change takes 3–4 s.
- **Kinematic bicycle model + pure pursuit** for physical cars steering along the same path.
- Turning circles from AASHTO and EU type-approval sizes; turn speed against corner radius from AASHTO/NACTO.

## The parts

### 1. Road data: the lane graph (the road builder's job)

Per "roads: no guessing": the road builder states every movement, and the sim never infers one.

- **Lanes:** through lanes, turn pockets, parking lanes and bays as lanes, and the ramp lanes the builder already makes.
- **A connector for every permitted movement at every junction,** lane to lane: which lanes may go which way.
  - Its curve is drivable by the vehicle classes allowed on it (minimum radius per class).
  - The painted lane arrows come from the same data.
  - Today the builder can make connectors but no city has any. Only `hill_junction.json` does.
- **Junction control:** signal groups with their movements, stop and give-way lines, priority roads, crosswalks, stop lines.
- **Conflict matrix per junction**, baked from connector geometry plus the rule set: which movements cross or merge, who yields to
  whom, and which may wait inside the junction.
- **Speed limits per road**, posted (signs placed by the builder). The rule set gives defaults by road class.
- **Regulations as road data, their signs and paint planted from it** (Glenn, 2026-11-11: "make and plant those traffic signs so we
  can visually see something on the road and gauge the AI against it"):
  - every regulation is a statement on a lane, connector or junction approach: stop, give way, priority road, speed limit, no
    entry, one way, no U-turn, no parking, no stopping, bus lane, turn restrictions;
  - **the picture of each is planted from the same statement:** the sign (a face laid out by the existing road-sign system,
    ADR-0110, baked into its atlas, on a post or gantry) and the road paint (stop line, 止まれ on the road, arrows, stall lines,
    crosswalks). A driver reads exactly what the player sees, so a car that runs a stop has a visible sign it ran;
  - which regulations exist, what they mean, and what their faces look like belong to the **rule set**: Japanese faces by
    default (the red inverted 止まれ triangle, the blue one-way arrow, the red-ringed speed limit), from the official standard
    and credited;
  - **parking is road data too:** kerbside spaces and bays the road states and paints, with the rule set's no-parking zones (near
    junctions, crossings, bus stops, hydrants). Today the sim invents a bay every 7 m along any road with a parking band; that
    goes.
- **Published to the game as one LaneGraph.** Vehicles route and drive on it. The NavGraph stays for walking.
- A lanes-tag bump rebuilds every city level's roads (Glenn's OK at that step).

### 2. The vehicle: one state

- The truth is `{lane or connector, s along it, lateral offset d, speed, acceleration}` plus the route as a sequence of lanes and
  connectors.
- The drawn pose, heading and indicators are read straight from it:
  - **Position:** on the lane curve, plus d.
  - **Heading:** the curve's tangent, turned by the rate of change of d.

  Paths respect the turning circle, so the heading can never pivot.
- **Kinds are data** (`vehicle_classes.lua`, extended): length, width, wheelbase, minimum turning radius, acceleration and
  braking, which roads and lanes it may use. Articulated vehicles' trailers follow the hitch (tractrix).
- **Physical cars** (Jolt, near the player) steer along the same path by pure pursuit. While physical, the body's pose is the
  truth, projected back onto `{lane, s, d}` each tick. There's no ghost.

### 3. The driver: perceive, decide, act

**Perceive.** Each tick C++ builds one picture of the road ahead along the driver's own path:
- the leader on its path (across connectors);
- foes on crossing or merging connectors (link leaders, with virtual gaps), and foes approaching the junctions ahead (time
  windows);
- the signal and sign that apply to its movement;
- space beyond the junction;
- people on its path or crosswalk;
- neighbours in the lanes beside it.

These are **facts** with names. The fact vocabulary grows in C++ when a rule needs something new to look at.

**Decide.** A **rule set** (Lua data) is a driving state machine:
- **States:** cruising, following, approaching a junction, stopping at the line, waiting, giving way, crossing, turning,
  preparing a lane change, changing lane, merging, pulling out, parking.
- **Rules:** conditions over facts. Each produces a standard **output**:
  - a stop point ("stop at the line", "stop behind the crossing point");
  - a speed cap ("the posted limit", "turn speed for this radius");
  - an intent ("change to the turn lane", "indicate left");
  - an action ("sound the horn").
- Rules carry priorities, and every output names the rule that made it.
- **Rule sets are declarative data, read at load and evaluated in C++.** That keeps thousands of drivers cheap, and it keeps
  "data in Lua, C++ is the language". No Lua runs per car per tick.
- **The default rule set: Japan, driving on the left.**
  - green / yellow / red, no turn on red, green arrows;
  - a turn across oncoming traffic gives way to it;
  - stop (止まれ) and give-way signs;
  - at equal unsignalled junctions, traffic from the left has priority;
  - pedestrians have priority at crosswalks;
  - limits in km/h.

  The exact rules are to be checked against the Road Traffic Act before they're written, and credited.
- **Signal timing comes from the rule set:** cycle, greens, yellow by the ITE formula, all-red clearance.

**Act.** IDM turns the tightest stop point or speed cap into acceleration, never above the Krauss safe speed.
- **Lane changes** run in order: decide, indicator on, wait for a gap that's safe by the rules, move over 3–4 s, indicator off.
- **Indicators** are decisions, not a reading of the path's curvature. They come on before the move and match the turn lane.
- **The horn** is an action rules can take on perceived danger (cut off, someone stepping out).

**Personality** (#222) is the person's own parameters: headway, accepted gap, speed relative to the limit, patience, attention
(lapses are missed facts), assertiveness, indicator diligence, horn temper. The ranges are in Lua; the values belong to the
person. Robotic drivers use the defaults.

**Right of way without deadlock** (from SUMO):
- time-window admission;
- an antisymmetric link-leader relation;
- the box check with claim release;
- impatience that grows with waiting (part of the rules, bounded by the rule set).

No timers break standstills. A car that waits past a threshold with no rule reason is counted as a bug.

### 4. Levels of detail (same people, same rules)

- **Near** (roughly the nearest 150 m):
  - full drivers with personality and attention, every tick;
  - physical cars for the closest dozen.
- **Middle** (in view, to about 1.5 km):
  - the same lane graph, rules and trips, with robotic drivers;
  - cheaper perception and a slower update;
  - must still look right from a distance: no jumps or odd turns.
  - CPU first. A compute-shader version only if the measured cost needs it.
- **Out of sight:**
  - no driving. Each person is fast-forwarded along their trip by their schedule, using travel times learned from the
    simulated tiers by road and time of day, so congestion carries over.
  - Arriving in an area places its cars mid-trip on their lanes at realistic gaps, then runs a few seconds of simulation before
    it's shown (#224).
- **Handovers** happen only at safe points, insert only into free space, and never change the plan.

### 5. Seeing why

- **The driver card:** clicking a car in god mode (#215) shows who's driving, who's aboard, where to and why, the current
  state, and the rule in force with its facts ("giving way: oncoming car, gap 2.1 s < 4.1 s").
- **A decision trace:** the last N decisions per car, each with its rule and facts.
- **The same over the MCP** (`driver? <id>`).
- **F9 reports** made while looking at a car carry its card and trace.

## Measurements (the lab is the judge)

Held by `level_tests` in the traffic lab. The current model's baseline is above.

| What | Target |
|---|---|
| Deadlocks (waiting with no rule reason > 20 s) | 0 |
| Teleports, unsticks, despawns | 0 (there is no mechanism for them) |
| Spins, swings, pivots (heading change faster than the class's turning circle allows) | 0 |
| Stops inside a junction | only when giving way by a rule, and recorded as such |
| Body overlaps between vehicles | 0 with robotic drivers |
| Queue discharge at a green | saturation headway about 1.9 s from the 4th car; start-up lost time about 2 s |
| Lane changes | 100% indicator before the move; 3–4 s each; accepted gaps within the rule set's |
| Turn speeds | within the class's comfortable lateral acceleration for the connector's radius |
| Speed | at or under the posted limit, plus each driver's personal margin |
| Crashes | 0 with robotic drivers; with personality on, they're reported, with the rules and lapses behind each |
| Cost | ms per tick per 1,000 drivers, per tier |

Unit tests cover the rule engine, connector radii, matrix generation, IDM and Krauss, and determinism. About 85 existing
tests exercise driving. Each one is either rewritten against the new model as a requirement, or retired with the old model.
The list is drawn up in phase 2.

## Build order

Each phase ends with its measurements passing in the lab, behind a per-level switch (`citysim.driving = "v2"`). The cities keep
the old model until phase 9.

0. **This plan approved; GitHub issues reshaped** (#225 rewritten, one issue per phase, #218–#221 superseded).
1. **The lane graph in the lab.** Connectors for every movement, lane turn assignments, control, the conflict matrix, speed
   limits, parking lanes. Checks: every connector within its class's radius, every lane reachable, nothing contradicting
   the paint. Lanes-tag bump.
   **1b. Regulations, signs, paint and parking** as road data, planted in the lab: every regulation stated and shown,
   Japanese faces, parking spaces the road states. Can start first: it gives the lab something visible to judge drivers
   against from phase 2 on.
2. **The core.**
   - the vehicle state;
   - lane-level routing that knows where the car faces;
   - IDM/Krauss following;
   - pose from state;
   - the rule engine with a first Japanese rule set;
   - the card and trace over the MCP;
   - cars only, on the lab's grid, without junction control yet (an all-way-stop grid, first come first served).
3. **Junctions** (milestone 1, agreed): signals on rule-set timing, matrix right of way, time windows, link leaders, the box
   check, impatience. **Glenn watches the downtown grid.** Cars queue in lanes, indicate, wait for green, turn smoothly lane to
   lane, change lanes in order, keep the limit. The table above holds.
4. **Lane changes and turn-lane choice** in full: MOBIL plus strategic changes, cooperative yielding.
5. **The freeway:** ramps, merges and zipper merges, lane drops; unsignalled junctions with stop and give-way signs; maybe a
   roundabout.
6. **The rest of the road's users:**
   - parking and pulling out as paths;
   - vehicle kinds as data: buses at stops, cabs, vans;
   - crosswalks with walkers;
   - the player's car as a perceived vehicle.
7. **Drivers as people:** personality (#222), attention, the horn. Emergent incidents and their aftermath as rules.
8. **Levels of detail:** the middle tier, out-of-sight fast-forward and catch-up (#224), the physical tier on the new state.
9. **The cities switch** (metro, river_town, the island): their lane graphs baked, the level suite run. Then the old driving
   code (~3,400 lines) and its tests are deleted.

Running alongside, outside the driving model:
- demand: island commutes and cab sizing (#223); hotels and visitors (#213);
- god mode (#215), which hosts the card.

## Risks and open questions

- **How expressive declarative rules can be before they need code.** The fact vocabulary is the pressure valve. Watch for rules
  that want to be functions.
- **Connector generation at awkward junctions** (skewed, offset, five-way, ramps meeting streets). The builder must refuse what it
  can't make drivable, not emit a bad curve.
- **The middle tier's cost** at island scale: measured in phase 8 before choosing CPU or GPU.
- **Physical cars:** Jolt bodies tracking a path well enough at low speed (pure-pursuit lookahead tuning is documented as fiddly).
- **Rule-set accuracy:** the Japanese rules are to be checked against the Act, and the US-derived calibration numbers (HCM
  gaps) mirrored for driving on the left.
