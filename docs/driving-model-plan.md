# The driving model, rebuilt — plan

Status: **draft for Glenn's review** (2026-11-11). Nothing in it is built yet.
It replaces the driving parts of `vehicle-agent-plan.md` (ADR-0061) and the first traffic plan in #225 (#218–#221). The people
side stays and feeds this: who lives where, their jobs, their days, their roles (`agent-sim-plan.md`, `roles.lua`).

## Words used in this plan

- **Agent** — one simulated person in the city.
- **Lane graph** — the roads described lane by lane: every lane, and every way a car can get from one lane to another.
- **Connector** — the path a car follows *through* a junction, from the lane it arrives in to the lane it leaves by. One for each
  move the junction allows: left from this lane, straight from that one, and so on.
- **Movement** — one of those allowed moves through a junction.
- **Crossing table** — for each junction, which movements cross or merge with which, and who has to give way to whom. Worked out
  once when the roads are built, not while driving.
- **Rule set** — the rules of the road for a country or region, written as Lua data: what signs exist and mean, who gives way to
  whom, signal timing, speed limits, whether you can turn on a red.
- **Regulation** — one rule placed on the road: "stop here", "no entry", "40 km/h from here".
- **Fact** — something a driver has noticed, with a name the rules can refer to: "the car ahead is 12 m away and slowing", "my
  light is red", "a car is coming the other way, 3 seconds out".
- **Level of detail** — how much work the simulation spends on a car, depending on how far it is from the player.
- **The lab** — `traffic_lab`, a test level with only roads and drivers, where all of this is built and judged first.
- **Baseline** — the old model's numbers in the lab, which the new one has to beat.

## Why a rebuild

The current model can't be patched. What it really tracks about a car is its place on its route: which piece of road it's on,
how far along, which lane number. Everything else is worked out from that, and each part works it out differently:

- **Drawing** adds lane offsets, two different rules for curves at corners, and an easing out of a parking space that only changes
  the picture.
- **The car's nose** is a separate direction that slowly turns toward where the car is going.
- **Following** decides who's ahead by who is further along the same lane of the route.
- **Looking ahead** casts a straight line out of the car's nose.
- **Junction rules** look at how far the car is from the middle of the junction, and at which way other cars point.
- **Crashes** use the car body shapes.

When these disagree, two cars each wait for the other, and timers break the standoff: creep forward after 6 seconds, freeze after a
bump. Routes don't know which way a car is facing or how tightly it can turn, so the drawing hides moves no car could make. About
3,400 lines of special cases, each written for one situation someone saw.

The baseline, measured in the lab on the current model (commit 1a315bf8, 289 drivers over 2 minutes):
- 166 times a car stood still for over a second in the middle of a junction, 1,987 seconds in all;
- several cars standing for over 100 seconds: deadlocks, where cars wait on each other forever;
- 78 of 304 trips starting with the car swinging round more than 90 degrees out of its space;
- 3 cars spinning round on the spot.

## Principles (from Glenn's interview, 2026-11-11)

1. **Traffic is emergent.** It's what many individual drivers make by using the same roads, under the same rules, at the same
   time. No code is about "traffic" as a whole. Cars never flow like a fluid, and are never spawned to fill a road.
2. **Every vehicle is someone going somewhere.** An agent with a reason is driving it: a person, a couple, a family; a cab
   driver at work, a bus driver on shift, someone delivering. More traffic comes from more people with reasons to travel
   (tourists, hotels, commutes across the island: #213, #223). Any vehicle can answer "who's driving, who's aboard, and why".
3. **Driving is the rules of the road, as data.**
   - Rule sets are written in Lua and can be switched per level or region.
   - The default is driving on the left under Japanese rules (or a mix). Others can be tried later.
   - The C++ is general machinery: notice, decide by the rules, act within what the vehicle can do. It never holds a rule written
     for one situation.
   - The test of the design: if something can only happen because someone wrote code for that exact case, the design is wrong.
4. **Each vehicle has one true state.** Where it is, which way it points, and where it's going are kept in one place, and
   drawing, following, looking ahead, junction rules and physics all read that same place.
5. **Every move is drivable.** Paths come from the road data and never turn tighter than the vehicle can. No pivoting, no
   sliding sideways, no swinging round in a drawn arc. Pulling out of a space follows a real path too.
6. **Further away means less detail, never different rules.** Distant drivers are robots: no personality, simpler noticing,
   updated less often. But they're the same people on the same trips under the same rules. A car moving between levels of
   detail never jumps or changes its plans.
7. **No escape hatches.** Nothing unsticks cars by timer, teleports them, or lets them push through a blocked junction. If
   something has to break a standstill, that's a bug, and it's counted.
8. **Outcomes are emergent too.** Near-misses, horns, crashes and what happens after come from drivers' attention, judgement and
   personality meeting each other. They're never scripted.
9. **You can always see why.** Any car can say what it's doing, which rule made it do that, and what it decided recently.
10. **Judged by real numbers, not by eye alone** (see Measurements).

## What we borrow, in plain words

We're not inventing traffic simulation. These are the established ideas we build on, each with the name you'd search for. The
full research notes are in the session scratchpad, and every source goes in CREDITS.md when the code lands.

- **From SUMO** (a widely used open-source traffic simulator, from the German Aerospace Center) and its 2014 paper on junctions
  (Erdmann & Krajzewicz, "SUMO's Road Intersection Model"):
  - roads described lane by lane, with connectors through each junction as the only places a car can be;
  - the crossing table for each junction;
  - **booking a junction:** as a car approaches, it announces when it expects to enter and leave. It goes in only if its time
    is clear, by a safety margin, of every other car whose path crosses its own. The drivers sort it out among themselves; the
    junction doesn't decide.
  - **inside a junction, whoever got in first goes first,** and that order can never loop round (A waits for B waits for A).
    The paper names that no-loop rule as what prevents deadlocks. A car behind keeps its distance from the point where the two
    paths cross, as if following.
  - **don't block the box:** enter only if there's room to leave on the other side. If there isn't, wait before the junction
    and give up your turn, so cross traffic can still go.
  - **patience runs out gradually:** a car that has waited long accepts smaller gaps. At an all-way stop, whoever waited
    longest goes.
- **Following the car ahead:** each car speeds up toward its preferred speed and eases off smoothly as it closes on the car
  ahead, keeping a gap of so many seconds (the *Intelligent Driver Model*, IDM). On top of that, a hard limit: never go so fast
  that you couldn't stop if the car ahead braked as hard as it can (the *Krauss safe speed*). Together they mean following can't
  cause a crash.
- **Changing lanes:** a driver weighs whether the move helps them more than it slows the car it cuts in front of (*MOBIL*),
  and changes lanes it must make for its route more urgently the nearer the turn is (SUMO's *LC2013*). A lane change takes 3–4
  seconds.
- **Physical cars steering:** a car chooses its steering by aiming at a point a short way ahead on its path (*pure pursuit*),
  with a simple model of how a car with front and back wheels turns (the *kinematic bicycle model*).
- **How tight vehicles can turn, and how fast round a corner:** standard road-design figures (from AASHTO, the US road
  designers' association, NACTO, the US city transport officials' association, and the EU's rule on how tightly every bus and
  truck must be able to turn).

## The parts

### 1. Road data: the lane graph (the road builder's job)

Under our rule "roads: no guessing", the road builder states every move a car can make, and the sim never guesses one.

- **Lanes:** through lanes, turn lanes, parking lanes and parking bays, and the ramp lanes the builder already makes.
- **A connector for every allowed movement at every junction,** lane to lane.
  - Each one is a curve that every kind of vehicle allowed on it can actually drive (no tighter than its turning circle).
  - The arrows painted on the lanes come from the same data.
  - Today the builder *can* make connectors, but no city has any. Only one hand-made test road, `hill_junction.json`, does.
- **How each junction is controlled:** traffic lights and which movements each light controls, stop and give-way lines,
  which road has priority, crosswalks, stop lines.
- **The crossing table for each junction**, worked out from the connectors' shapes plus the rule set.
- **Speed limits for each road**, with signs placed by the builder. The rule set gives defaults by kind of road.
- **Regulations are road data, and their signs and paint are planted from the same data.** Glenn, 2026-11-11: "make and plant
  those traffic signs so we can visually see something on the road and gauge the AI against it."
  - Each regulation sits on a lane, a connector or a junction approach: stop, give way, priority road, speed limit, no entry,
    one way, no U-turn, no parking, no stopping, bus lane, turn bans.
  - Its picture is planted from that same statement:
    - the sign: a sign face laid out by the existing road-sign system (ADR-0110), which today only signs the island's freeway;
    - the road paint: stop line, 止まれ ("stop") on the road, arrows, parking-space lines, crosswalks.

    A driver obeys exactly what the player sees, so a car that runs a stop sign has a visible sign it ran.
  - **The rule set decides which signs exist, what they mean, and what they look like:** Japanese signs by default (the red
    upside-down 止まれ triangle, the blue one-way arrow, the red-ringed speed limit), from the official Japanese standard,
    credited.
  - **Parking is road data too:** kerbside spaces and bays that the road states and paints, and the rule set's no-parking
    zones (near junctions, crossings, bus stops, hydrants). Today the sim invents a parking space every 7 m along any road with
    a parking strip. That goes.
- **The game gets one lane graph.** Vehicles plan routes on it and drive on it. Walkers keep their own path network.
- Changing the roads this way means rebuilding every city level's roads (a "lanes tag bump"), which needs Glenn's OK.

### 2. The vehicle: one true state

- **What we store:**
  - which lane or connector the vehicle is on;
  - how far along it;
  - how far to the side of the lane's centre (for lane changes);
  - its speed and acceleration;
  - its route, as the lanes and connectors still ahead.
- **Everything visible is read straight from that.**
  - **Position:** the point on the lane's curve, moved sideways by the offset.
  - **Direction:** along the curve, turned a little while drifting sideways in a lane change.

  Since every path is drivable, the car can never pivot or slide.
- **Kinds of vehicle are data** (`vehicle_classes.lua`): length, width, wheelbase (the distance between front and back axles),
  tightest turning circle, how hard it can speed up and brake, which roads and lanes it may use. A truck's trailer follows its
  hitch the way a real trailer does.
- **Physical cars** near the player (real physics bodies, in Jolt, the physics engine) steer along the same path. While a car
  is physical, its physics body *is* the truth, and its place on the lane is read from where the body is. There's no second copy
  of the car to fall out of step.

### 3. The driver: notice, decide, act

**Notice.** Every tick, the C++ looks along the driver's own path and writes down facts:
- the car ahead on its path, through junctions too;
- cars on paths that cross or merge with its own, and how far each is from the crossing point;
- cars heading for the next junctions, and when they'll get there;
- the light or sign that applies to its own move;
- whether there's room on the far side of the junction;
- people on its path or on the crosswalk;
- cars in the lanes beside it.

Each fact has a name. When a rule needs to notice something new, the list of facts grows, in C++.

**Decide.** A rule set (Lua data) describes driving as a set of states and the rules for moving between them:
- **States:** cruising, following, approaching a junction, stopping at the line, waiting, giving way, going through a junction,
  turning, getting ready to change lanes, changing lanes, merging, pulling out, parking.
- **Rules:** conditions on facts. Each rule gives one standard kind of answer:
  - a place to stop by ("stop at the line", "stop before the other car's path");
  - a top speed ("the speed limit", "a safe speed for this corner");
  - an intention ("move to the turn lane", "indicate left");
  - an action ("sound the horn").
- Rules have priorities, and every answer carries the name of the rule that gave it. That name is what the driver card shows.
- **Rule sets are descriptions, not programs:** they're read once when the level loads, and the C++ applies them. That keeps
  thousands of drivers cheap, and keeps to "data in Lua, C++ is the language". No Lua runs for each car each tick.
- **The default rule set: Japan, driving on the left.**
  - lights go green, yellow, red, and you can't turn on a red; green arrows let a turn go;
  - a turn across oncoming traffic gives way to it;
  - stop (止まれ) and give-way signs;
  - where two equal roads meet without a light, traffic from the left goes first;
  - people on a crosswalk have priority;
  - speed limits in km/h.

  Each rule is checked against Japan's Road Traffic Act before it's written down, and the Act is credited.
- **Light timing comes from the rule set:** how long each phase lasts, and how long yellow lasts. Yellow uses the standard
  traffic engineers' formula (from the ITE, the US Institute of Transportation Engineers), which bases it on the road's speed.
  There's an all-red moment between phases so a junction can clear.

**Act.**
- **Speed:** the car follows the tightest of its answers (the nearest place to stop, the lowest top speed), easing smoothly
  toward it, and never above the hard "could I stop in time" limit.
- **Lane changes** happen in order: decide, indicator on, wait for a gap that's safe by the rule set, move over 3–4 seconds,
  indicator off.
- **Indicators** are decisions, not a reading of how the road bends. They come on before the move and match the lane.
- **The horn** is an action rules can take when the driver notices danger: being cut off, someone stepping out in front.

**Personality** (#222) is the person's own settings for these rules:
- how big a gap they keep;
- how small a gap they'll accept;
- how far over or under the limit they drive;
- how patient they are;
- how often their attention slips (a slip means a fact they didn't notice);
- how pushy they are;
- how reliably they indicate;
- how quick they are to use the horn.

The ranges live in Lua; the values belong to the person, so they drive the same way every day. Robot drivers use the defaults.

**Giving way without deadlock** comes from the borrowed ideas above:
- booking the junction;
- the no-loop "first in goes first" order inside it;
- don't block the box, giving up your turn if you can't;
- patience that runs out gradually, within limits the rule set sets.

No timers break standstills. A car waiting with no rule saying why is counted as a bug.

### 4. Levels of detail (same people, same rules)

- **Near** (roughly the nearest 150 m):
  - full drivers with personality and attention, updated every tick;
  - real physics cars for the closest dozen.
- **Middle** (in view, out to about 1.5 km):
  - the same roads, rules and trips, but robot drivers;
  - simpler noticing, updated less often;
  - from a hill it must still look right: queues at lights, turns at corners, no jumps, no odd turns, no stop-start.
  - On the CPU first. A graphics-card version (a compute shader) only if the measured cost needs it.
- **Out of sight:**
  - no driving. Each person is moved along their trip by their schedule, using how long each road takes at that time of day,
    learned from the simulated drivers, so rush hour still slows them.
  - When you arrive in an area, its people's cars are placed part-way through their trips, in their lanes, at realistic gaps.
    A few seconds of simulation run before the area is shown (#224).
- **Moving between levels** happens only where it's safe, only into empty road, and never changes anyone's plans.

### 5. Seeing why

- **The driver card:** click a car in god mode (#215) to see:
  - who's driving, who's aboard, and where they're going and why;
  - what it's doing now;
  - the rule it's following, with the facts behind it. For example: "giving way: car coming the other way, 2.1 seconds off;
    I need 4.1".
- **Recent decisions:** the last few for each car, each with its rule and facts.
- **The same over the MCP** (`driver? <id>`), so Claude can read it.
- **F9 reports** made while looking at a car carry its card and recent decisions.

## Measurements (the lab is the judge)

These are checked by tests in the lab. The baseline is above.

| What | Target |
|---|---|
| Deadlocks (a car waiting over 20 seconds with no rule saying why) | none |
| Teleports, unsticks, cars removed to clear a jam | none (there's no way left to do them) |
| Spins, swings, pivots (a car turning faster than its turning circle allows) | none |
| Stopping in the middle of a junction | only when giving way by a rule, and recorded as such |
| Cars overlapping each other | none with robot drivers |
| How a queue empties at a green | after the first few cars, one car every ~1.9 seconds; the first car loses about 2 seconds getting going (traffic engineers' "saturation flow" and "start-up lost time", from the FHWA, the US Federal Highway Administration) |
| Lane changes | indicator always on before the move; 3–4 seconds each; gaps accepted within the rule set's |
| Speed round corners | no faster than comfortable for the corner's tightness |
| Speed | at or under the limit, plus each driver's own habit |
| Crashes | none with robot drivers; with personalities on, each one reported with the rules and lapses behind it |
| Cost | milliseconds per tick per 1,000 drivers, at each level of detail |

Smaller tests cover:
- the rules engine;
- that connectors are drivable;
- that the crossing tables are built right;
- that following can't crash;
- that the same start always gives the same result.

About 85 existing tests exercise driving. Each one is either rewritten for the new model as a requirement, or retired with the
old model. That list is drawn up in phase 2.

## Build order

Each phase ends with its measurements passing in the lab, behind a per-level switch (`citysim.driving = "v2"`). The cities keep
the old model until phase 9.

0. **This plan approved, and the GitHub issues reshaped:** #225 rewritten, one issue per phase, #218–#221 closed as superseded.
1. **The lane graph in the lab** (#226):
   - connectors for every movement, and which lanes may turn which way;
   - how each junction is controlled;
   - the crossing tables;
   - speed limits;
   - parking lanes.

   Checks: every connector drivable, every lane reachable and leavable, nothing contradicting the paint. Needs a lanes tag bump.

   **1b. Regulations, signs, paint and parking as road data, planted in the lab** (#233): every regulation stated and shown,
   Japanese signs, parking spaces the road states. This can go first: it gives the lab something visible to judge drivers
   against from phase 2 on.
2. **The core** (#227):
   - the one true state;
   - routes that know which way the car is facing;
   - following;
   - drawing from the state;
   - the rules engine with a first Japanese rule set;
   - the driver card over the MCP.

   Cars only, on the lab's grid, with every junction run as an all-way stop (whoever arrived first goes) until phase 3.
3. **Junctions** (#228), **milestone 1, as agreed:** lights on the rule set's timing, giving way by the crossing tables, booking
   junctions, first-in-goes-first, don't block the box, patience. **Glenn watches the downtown grid:**
   - cars queue in their lanes, indicate before turning (matching the turn lane), and wait for green;
   - turns are smooth from lane to lane;
   - lane changes go in order;
   - the limit is kept;

   and the measurements table holds.
4. **Lane changes and choosing the turn lane early** (#229), with drivers letting each other in.
5. **The freeway and junctions without lights** (#230): ramps, merging into a gap, lanes that end, stop and give-way signs,
   maybe a roundabout.
6. **Everyone else on the road** (#231):
   - parking and pulling out on real paths;
   - buses at stops, cabs, vans;
   - trucks with trailers, bikes;
   - people on crosswalks;
   - the player's car as something drivers see and react to.
7. **Drivers as people** (#222): personality, attention, the horn, and whatever follows from them.
8. **Levels of detail** (#224): the middle distance, out of sight, arriving in an area, and physical cars, all on the new state.
9. **The cities switch** (#232): metro, river_town, then the island, with their roads rebuilt and their tests run. Then the old
   driving code (about 3,400 lines) and its tests are deleted.

Alongside, outside the driving model:
- more reasons to travel: island-wide commutes and enough cabs (#223); hotels and visitors (#213);
- god mode (#215), where the driver card lives.

## Risks and open questions

- **How much rules-as-descriptions can express before a rule wants to be a program.** The list of facts is where that
  pressure goes. Watch for rules that keep needing new facts.
- **Awkward junctions:** skewed, offset, five-way, ramps meeting streets. The builder must refuse a movement it can't make
  drivable, and say where, rather than produce a bad curve.
- **The cost of the middle distance at island scale:** measured in phase 8 before choosing CPU or graphics card.
- **Physical cars following a path well at low speed:** the "aim at a point ahead" steering is known to be fiddly to tune.
- **Getting the rules right:** Japan's rules checked against the Act. The US-measured numbers we borrow (how big a gap drivers
  accept when turning, from the HCM, the US *Highway Capacity Manual*) mirrored for driving on the left.
