# The driving model, rebuilt — technical design

Status: **draft for Glenn's review** (2026-11-11). Nothing in it is built yet.
This is the design behind `driving-model-plan.md`. The plan says *what* we build and in what order; this says *how the
pieces fit*, what each piece owns, where the rules live, and what will go wrong. The two together are meant to be cut
into tasks (section 17) and handed to an implementer who has not read this conversation.

Where this design sharpens or departs from the plan, section 16 lists it, so the plan can be amended or the design
overruled.

## Words used in this document

The plan's words (agent, lane graph, connector, movement, crossing table, rule set, regulation, fact, level of detail,
the lab, baseline) mean the same here. New ones:

- **Section** — a stretch of road where the same lanes run side by side from start to end. Lanes are cut wherever a
  neighbour appears or ends, so inside one section "the lane to my kerb side" is always the same lane.
- **Path** — a lane or a connector: any curve a car can be on. A car is always on exactly one path.
- **Crossing point** — where two paths through a junction cross or join. It has a position along each path and a
  length (the stretch of each path where the two cars' bodies could touch).
- **Near turn / far turn** — the turn that stays by the kerb, and the turn that crosses oncoming traffic. Driving on
  the left, a near turn is a left turn and a far turn is a right turn. Rules and code use these words, never "left"
  and "right", so a rule can't be written backwards.
- **Kerb side / offside** — the side of the car nearest the kerb, and the side nearest the middle of the road.
- **Answer** — what a rule gives back: a place to stop by, a top speed, an intention, or an action.
- **Announcement** — what a car approaching a crossing point tells everyone: which path it's on, when it expects to
  reach the crossing point and when it will be clear of it.
- **Hold** — a "stop here until released" placed on a driver by something outside the driving model (a bus stop
  waiting for passengers to board, a cab picking up).
- **Wait-for graph** — a picture of who is waiting for whom: an arrow from each waiting car to the thing it's waiting
  for. A loop in it is a deadlock.
- **Host** — the program using the driving model. Today that is the city sim (`citysim`) and the lab.
- **Tick** — one step of the simulation.

## 1. The four questions, answered briefly

**How do we make it modular and extensible?** Four layers that only know the layer below them, and three narrow
openings where new things plug in:

1. *Road data* (the lane graph) — what exists on the road. No behaviour.
2. *Rule set* — what the things on the road mean, and how drivers behave. Data only.
3. *The driving core* — general machinery: notice, decide by the rules, act. Knows nothing about any one rule, sign,
   country or kind of vehicle.
4. *The host* — who is driving, where they're going and why. The core never asks.

The three openings: a **fact** (something a driver can notice), an **answer kind** (something a rule can ask for), and
an **announcer** (anything that can occupy a crossing point: a car, a person, a train, the player). A new sign, a new
country, a new kind of vehicle, a new kind of junction control is a data edit. A new thing to *notice* is one small
C++ function registered by name. Section 4.

**How are the rules data?** Most of the rules of the road are not "if this then that". They are *tables*: what each
sign means, who gives way to whom at each kind of junction, how big a gap is safe, how long a yellow lasts. Those
tables are Lua, read once at load, and combined with the road data into flat lookups. On top sits a short list of
rules that tie facts to answers ("my light is red → stop at the line"). No Lua runs per car per tick. Section 6.

**At what level should it be built?** Two answers, because the question has two meanings:

- *Level of simulation:* one car at a time, as a point with a length moving along a lane's curve (traffic engineers
  call this "microscopic, lane-based"). Not free 2D movement (too costly, and nothing to follow the rules against),
  and not "so many cars per road" (can't show a queue or a turn). Physics sits *under* it for the dozen nearest cars
  only; a schedule sits *over* it for cars out of sight. Section 2.
- *Level in the code:* its own library, `driving`, between the engine and the city sim. It depends on the engine only.
  The city sim depends on it. It never sees a person, a building, a renderer or the physics engine. Section 3.

**Drawbacks and edge cases:** sections 14 and 15. The three that matter most:

1. With no escape hatches, a real gridlock (every road round a block full, each front car waiting for room in the
   next) freezes and stays frozen. The design prevents the causes we control and *measures* the rest, but cannot
   promise it never happens.
2. A car bound to lanes can't improvise: it can't swerve round a wreck, squeeze past a double-parked van or make a
   three-point turn unless a path for that move exists in the road data.
3. Everything rests on the quality of the lane graph. A junction the road builder gets slightly wrong is a junction
   where drivers behave wrongly, with no code to blame.

## 2. The level of simulation

A car's whole state is: which path it's on, how far along (`s`, metres), how far to the side of the centre (`d`,
metres, non-zero only while changing lanes), its speed and acceleration, and its route. Position and direction are
*read off the path's curve*. They are never stored, so they can never disagree with it. This is principle 4 of the plan.

Three ways of moving a car, one state:

| Where | How the car moves | What is the truth |
|---|---|---|
| The nearest dozen (physical) | A real physics body steers along the path | The body. `s` and `d` are read back from where the body is. |
| Near and middle distance | The driving core moves `s`, `d` and speed | The state above. |
| Out of sight | The person's schedule moves them along their trip | Trip progress: route and time. No car exists. |

Why this level and not another:

- **Free 2D movement** (each car plans its own way through open space) would let cars swerve and improvise, but costs
  10–100 times more per car, and "obey the lane markings" becomes something to enforce instead of something that is
  true by construction. We take the lane-bound model and accept its limit (drawback 2).
- **Whole-road flow** ("this road holds 30 cars moving at 20 mph") is cheap but breaks principle 1: there would be no
  individual drivers, so no queue to see, no turn, no one to click on. It is used *only* out of sight, and only as
  travel times learned from the real simulated drivers.
- **Physics for every car** costs too much and is harder to keep to a path. It stays for the nearest dozen, where a
  bump from the player has to feel real.

## 3. Where it lives in the code

The engine library (`engine_core`) is city-free by rule, and `citysim` is an 8,500-line application file that the old
driving model is tangled into. The new model goes in neither.

```
engine_core                    lane graph TYPES and the road builder that fills them
     ▲                         (src/engine/procgen/city/roads/lane_graph.h, with the road builder)
     │
driving   (new static library, src/apps/driving/)
     ▲                         the core: rule sets, state, notice/decide/act, junctions, routes, trace, measurements
     │
citysim                        people, trips, reasons; drawing; the physics tier; the old model until phase 9
```

Rules of the boundary:

- `driving` includes nothing from `citysim`, the renderer, the entity world or the physics engine. It is given a lane
  graph, a compiled rule set and vehicle classes; it is told "start this trip"; it reports events.
- It has its own headless test program and runs the lab without the city sim's people. That is what makes
  "the lab is the judge" cheap.
- Lua is read by a small reader in `src/apps/driving/scripting/` that fills plain C++ structs. The core takes the
  structs, so unit tests can build a three-line rule set in the test itself. There is **no built-in C++ copy** of
  the rule set (the rule from #192: a C++ copy of a Lua table is a duplicate). If the file is missing, loading fails
  loudly.

Modules, one header and one `.cpp` each:

| Module | Owns | Reads |
|---|---|---|
| `procgen/city/roads/lane_graph` | Sections, lanes, connectors, junctions, crossing points, crosswalks, regulations, parking spaces. Fixed after load. | — |
| `rule_set` | The compiled rule set: device catalogue, right-of-way orders, gaps and other numbers, signal timing, modes, rules, personality ranges. | Lua, via the reader |
| `vehicle_class` | Per kind of vehicle: size, turning circle, speeding-up and braking limits, tags. | `vehicles.lua` |
| `junction_plan` | Lane graph × rule set, worked out at load: for each junction, who gives way to whom for each pair of movements, and its signal programme. | lane graph, rule set |
| `traffic_state` | Every car's state; for each path, its cars in order. | — |
| `route` | Routes over sections and connectors; which lanes of each section lead to the next move. | lane graph |
| `notice` | Facts for one driver. A registry of named fact functions. | state, lane graph, junction plan |
| `decide` | Runs the mode table and the rules over the facts; gives answers. | rule set |
| `act` | Turns answers into acceleration, sideways movement and hops between paths. | vehicle class |
| `junction` | Announcements, who may enter, room on the far side, signal clocks. | junction plan |
| `detail` | Carrying out the level of detail the host asks for each trip; moving between levels safely. | — |
| `trace` | Each car's last few decisions; the wait-for graph. | — |
| `measure` | The lab's measurements table, as code. | trace, state |
| `driving` | The front door: the tick, trips in, events out. | all of the above |

### 3.1 How it meets the people

Every agent is a real person in the world, and every car on the road has one of them at the wheel. The driving
library never creates, removes or decides anything about a person. It is handed a person and a car for the length of
one trip, and hands them back.

**Today** the person and the driving are one thing: the `Agent` struct holds who they are (role, household, goal
state) *and* where their car is on the road (link, lane, distance along, speed, driving state), and the car's own
record (`SimVehicle`) just copies its driver's position. That is one of the reasons the old model can't be separated
from `city_sim.cpp`.

**In the new model there are three things, each owning one kind of truth:**

| Thing | Lives in | Owns |
|---|---|---|
| The **person** (`Agent`) | `citysim` | Who they are, their day, what they're doing and why (the goal table). Where they are when on foot. |
| The **car** (a vehicle record) | `driving`, for every car on a road, moving or parked | Where the car is: its path, distance along, speed. A parked car is a car at rest in a parking lane with nobody at the wheel. |
| The **trip** | `driving`, while it lasts | The link between the two: this person is driving this car to that place. Route, mode, remembered facts, last decisions. |

Where a person is while in a car is never stored: it is read from the car (the driver through the trip, passengers
through the existing `RideBook`, which doesn't change). Cars kept off the street (a garage, a driveway) stay with the
host; they enter the driving model when they come out onto a path.

**The hand-over, step by step:**

1. The person's goal table fires a "go to" state, as today (ADR-0064). The city sim decides they'll drive and walks
   them to their car.
2. The city sim calls `beginTrip(car, person, destination)`. `person` is their stable id (`AgentId`), which the
   driving library treats as an opaque number. It uses it for two things only: working out this person's driving
   habits (a hash of the id against the rule set's ranges, so nothing is stored), and addressing events back.
3. The agent keeps one small handle to the trip. The fields it has today for link, lane, distance and speed go.
4. The driving library drives. The city sim can ask at any time, through the handle: where is the car, what is it
   doing, why (the driver card).
5. Events come back each tick as a list, and the city sim turns them into the goal events it already has:

   | Event from `driving` | Becomes |
   |---|---|
   | Parked at this space | `Arrived`: the person gets out at the kerb and walks to the door |
   | No route | `NoRoute`, as today |
   | Can't park | new: the goal table decides (open question 2) |
   | Missed a turn, re-routed | nothing; it's on the card |
   | Crashed | new: for phase 7 |

**The host can steer a trip, never the car:** change the destination (a cab given a new fare, a bus's next stop),
place and release a hold (passengers boarding), end the trip early. Buses, cabs and deliveries stay in `city_bus`
and `city_dispatch`, speaking only these three verbs.

**Level of detail is the host's decision.** The city sim's existing tier pass already knows the things that matter
(where the player is, who is being followed, who was seen lately and must not vanish). It tells the driving library
which level each trip is at; the driving library carries it out and may answer "not yet" (a lane change finishes
first). The mapping from today's tiers:

| Today's tier | Becomes |
|---|---|
| K (simulated in full) | near |
| V (far: follows its route once a second at a guessed speed, ignoring lights) | middle: same rules as near, looked at less often. The guessed-speed model goes. |
| D (dormant: rebuilt from the schedule) | out of sight. No trip and no moving car exist in `driving`. The person's schedule is the truth, and `driving` supplies the travel times it runs on. |

Going out of sight ends the trip record and leaves the person's schedule carrying on from the same progress. Coming
back starts a trip part-way along (section 12).

**The player.** When the player takes a car, its trip (if any) ends and the car becomes a physics body the model sees
as an obstacle and an announcer (section 11). There is no second "ghost" car following a plan, which is what freezes
a possessed driver today.

**What is not yet designed:** parked cars of people who are out of sight. Today a dormant person's parked car keeps
its place. When that person's schedule says they have driven away, their car must leave its space, and when the
schedule says they've arrived somewhere, it must take one. With hundreds of thousands of people this has to be
worked out for an area only as it wakes, in a way that gives the same answer each time. It belongs to task E7 and is
listed as open question 7.

### 3.2 Driving is one way of getting around, not all of it

Most people in the city should not be driving a car of their own. Some households own a car (couples and families,
mostly); students and many single people don't. People also walk, cycle, ride mopeds, take buses, trains and cabs.
On top of that are the people whose *job* is to drive: cab and bus drivers, delivery and service crews.

Today each agent is fixed for the whole run as either a driver or a walker (`Agent::archetype`), and buses, trains
and cabs are things the city sim sends a walker to. There is no moment where a person *chooses* how to make a trip.
That choice is not part of the driving library, and must not be put there. It belongs to a layer above it, in the
city sim, here called **getting around**. It needs its own plan; this section fixes only how the two meet.

**A journey is a list of legs.** Home to work might be: walk to the car, drive, walk from the parking space. Or:
walk to the stop, wait, ride the bus, walk. Each leg is carried out by the system that owns that kind of movement:

| Leg | Carried out by | Uses the driving library? |
|---|---|---|
| Walk | the walkers (unchanged) | no, but walkers are seen by drivers at crosswalks (section 11) |
| Drive your own car | `driving`: one trip (3.1) | yes |
| Cycle, ride a moped | `driving`: a trip in a narrow, slow vehicle class | yes |
| Ride a bus or a cab | the ride book: the person is a passenger in a vehicle someone else is driving | the *vehicle* is a trip; the passenger isn't |
| Ride a train | the rail sim (unchanged) | no, except trams in the street and level crossings (section 11) |
| Wait | the city sim | no |

So the driving library sees four of these as exactly the same thing (a trip by some vehicle class with someone at
the wheel) and never sees the rest. A bus driver on shift, a courier on a round and a parent on the school run are
all "a person driving a vehicle to a place", which is principle 2 of the plan.

**Who has what, and who chooses what, is data in the getting-around layer:**

- *Vehicles belong to someone:* a household (shared by its members, so when one partner has taken the car the other
  must go another way), or a place of work (the cab firm's cabs, the depot's buses, a shop's van), driven by
  whoever is on shift in a role that has that vehicle. Roles already hire staff this way (`roles.lua`).
- *Ways of travelling are rows in a table,* not kinds in C++: each says what it needs (a car, a bike, a stop within
  walking distance, a fare), which network it runs on and how it's costed.
- *A person picks, for each journey,* the way that is best for them: time taken, plus what it costs, plus their own
  leanings (worked out from their identity, like driving habits). Ownership rates by kind of household and the
  weights are Lua tables.

**What the driving library owes that layer** (three questions it must answer cheaply, without simulating anything):

1. *How long would it take to drive from here to there at this time of day,* for a given vehicle class? From the
   same travel times, learned from real simulated drivers, that out-of-sight trips use (section 12).
2. *How hard is it to park near there right now?* A count of free spaces by area.
3. *Where are the stops, ranks and loading places?* They are road data, stated by the road like parking spaces.

**How the driving simulation changes public transport**, which is the point of doing it this way:

- **Buses and cabs are in the traffic.** A bus sits in the same queue at the same light as everyone else, and its
  stop in a lane holds up the cars behind. Bus times stop being a timetable and become what actually happened: late
  in the rush hour, two buses arriving together. A bus lane is a regulation in the road data (`open_to = { "bus" }`)
  and visibly helps.
- **Congestion pushes people off the road.** Because the choice uses real travel times, a jammed route makes the
  train or the bike the better choice for the next person. This is the real relief for gridlock (drawback 1): fewer
  people choosing to drive into it, not a rule that unsticks cars.
- **Parking pushes too.** A full centre makes driving there slower (the search for a space), so more people arrive
  by bus.
- **Trains don't care about traffic,** except trams sharing a street. That is why they are worth taking.
- **Cabs depend on traffic twice:** the wait for one and the ride in one both get longer when roads are full.

**Limits to know about:**

- *Bikes and mopeds can't share a lane with a car.* A lane-bound model has one line of vehicles per lane. A bike
  uses a bike lane where the road has one; otherwise it takes the lane as a slow vehicle and cars pass it by
  changing lanes, or wait behind it. Slipping past a queue down the side is not possible without a different model.
  How much that matters depends on how many bike lanes the planner draws.
- *Deliveries need somewhere to stop.* A van stopping in a lane with nowhere to pull in blocks it. Loading places
  must be road data, and where there are none the van blocks the lane, as in life.
- *The choice feeds back on itself.* If everyone re-chooses at once when a road jams, they all pile onto the next
  road. Each person's view of travel times should lag (what the trip took last time *they* made it), which is also
  more true to life.
- *Numbers at island scale.* With hundreds of thousands of people out of sight, the choice must be a cheap lookup
  per journey, made when the journey starts, not a search.

### 3.3 What a person remembers, where, and for how long

**Today** an agent remembers almost nothing for long:

| What | Where it's stored | How long |
|---|---|---|
| Things they've seen: other cars and people, as positions with a guessed speed | `Agent::memory` (`engine::AgentMemory`), about 1.5 KB each, up to 32 things | 4 seconds after losing sight |
| Who they know: household, coworker, neighbour, acquaintance | one shared `RelationshipTable`, keyed by the person's stable id | the whole run |
| Who they are: home, job, household, age, role | the `Agent` struct; a population cache on disk | the whole run (time is frozen) |

A person who goes out of sight is rebuilt from their schedule when they come back, so only the last two rows
survive. Nothing a person learns is saved between sessions.

**The driving model adds short memory only,** and all of it belongs to the trip, not the person:

| What | Where | How long |
|---|---|---|
| Remembered facts ("I came to a full stop at this line", "I've waited 9 seconds") (6.3) | on the trip, in `driving` | until the rule set clears it; at most the trip |
| The last few decisions, for the driver card (13) | a short ring on the trip | the trip; near cars only |
| Driving habits: gap kept, patience and the rest (6.1 d) | nowhere: worked out from the person's id each time | always the same |
| How long each road takes, by time of day (12) | one table for the whole world. It is the city's knowledge, not one person's. | the run |

The 4-second sight memory is not used by drivers in the new model. A driver reads the cars ahead straight from the
lists each path keeps, and "didn't notice" is an attention slip (principle 8), recorded in the trace. Walkers keep
their sight memory as it is.

**What is missing: memory that outlives a trip.** Section 3.2 says a person should judge a journey by how long it
took the last time *they* made it. Nothing above can hold that. The design adds one place for it:

- **A side table in the city sim, keyed by the person's stable id** (`AgentId`), holding only what must outlive a
  trip. Not on the `Agent` struct, which is already 432 bytes and is walked whole by several passes every tick.
- **First contents:** for each of a person's regular journeys (home to work and back, and the like), how long it took
  last time by each way of travelling they have tried. A few numbers each: about 4 MB for 500,000 people.
- **How long:** the whole run, and it survives the person going out of sight. An out-of-sight journey writes the
  time the schedule gave it, so people nobody is watching still learn.
- **The driving library never reads or writes it.** It reports "this trip took this long" as part of the arrival
  event, and the getting-around layer stores it.

**Not decided** (open questions 11 and 12):

- *Do experiences change people?* A crash, a near miss or being cut off leaves no trace on the person today, and in
  this design. The side table could hold "more careful for a while" or "takes the bus now", and the rule set could
  read it as a fact. That is a new feature with its own cost: it makes two runs from the same start differ depending
  on what the player caused.
- *Is anything kept between sessions?* Saving the side table and the roads' travel times would let a city keep its
  habits from one play to the next. Nothing is saved today.

## 4. The three openings (how it stays extensible)

The test from the plan: *if something can only happen because someone wrote code for that exact case, the design is
wrong.* The design answers by fixing a small number of places where new things are added, and making each one general.

**Facts.** A fact is a named value a driver has noticed: a number, a yes/no, or one of a short list of words. Each is
produced by one C++ function registered under its name:

```cpp
// What one driver can see from the wheel this tick, read-only: its own car, the road ahead along its route, the
// cars on that road, the junctions coming up and who has announced for them.
struct DriverView {
    const CarState& me;
    const RouteAhead& road;            // the paths ahead, in order, out to the look-ahead distance
    const TrafficState& traffic;       // every car, and each path's cars in order
    const JunctionPlan& junctions;
    const Announcements& announced;
};

// One thing a driver has noticed: its value, and what it was about (for the driver card).
struct NoticedFact {
    FactKind kind;                     // a number, a yes/no, or a word from a short list
    Real number; bool yes; WordId word;
    CarId aboutCar; PathId aboutPath; Real seconds, metres;   // whichever apply
};

using NoticeFn = void (*)(const DriverView&, NoticedFact& out);
void registerNotice(const char* factName, FactKind kind, NoticeFn fn, const char* plainMeaning);
```

The names say what each is: a `DriverView` is what the driver looks at, a `NoticeFn` looks, a `NoticedFact` is what
it saw. A fact function can only read the view, so it cannot change a car or reach anything a driver couldn't see.

A rule set refers to facts by name. An unknown name is an error at load, with the file and line. A fact also records
*what it was about* (which car, which light, how many seconds), so the driver card can show it without extra code.
Facts a rule set doesn't use are never worked out.

**Answers.** A rule can give exactly four kinds of answer, and that list is closed:

| Answer | Example | How answers combine |
|---|---|---|
| A place to stop by | the stop line; just before another car's path; the end of a lane that runs out | the nearest wins |
| A top speed | the limit; a comfortable speed for this corner | the lowest wins |
| An intention | move one lane toward the kerb; indicate; take the next gap | the most urgent wins, one per kind |
| An action | sound the horn | all happen |

Because answers combine by fixed arithmetic, rules never need to know about each other, and adding one can't break
another. Whichever answer ends up limiting the car is its **binding answer**; the rule that gave it is what the driver
card shows.

**Announcers.** A crossing point doesn't care what is about to cross it. Anything can announce "I'll be on this
crossing point from time A to time B": a car in the model, a person stepping onto a crosswalk, a train from the rail
sim, the player's car (guessed from its speed and heading). Drivers give way to announcements, not to kinds of thing.
This is why crosswalks, level crossings and the player need no code of their own in the driver.

*How an announcement is written down.* It is a small record on a board that is wiped and rewritten every tick:

```cpp
struct Announcement {
    CrossingId crossing;        // which crossing point (road data, section 5.2)
    PathId onPath;              // which of the crossing's two paths the announcer is on
    Real arriveAt, clearAt;     // when its front reaches the crossing point and when its tail is past, in seconds
    Real speedThere;
    bool stopsFirst;            // it must come to a stop before the crossing (a stop sign, a red light)
    Real reachedLineAt;         // when it got to its stop line, for "who arrived first"
    AnnouncerKindId kind;       // a row of the rule set's `announcers` table, by name
    uint64_t who;               // an opaque id: for the driver card and for tie-breaks
};
```

- **The crossing is road data.** A crossing point is where two paths meet. The "other path" doesn't have to be a
  lane: the road builder also records where a crosswalk crosses each lane and where a rail track does. So "a person
  on this crosswalk" and "a car on that connector" land on the board in the same form.
- **Who writes them.** Cars in the model write their own at the start of each tick, from their own state. Everything
  else is written by a function the host registers under a kind name:

  ```cpp
  using AnnounceFn = void (*)(const LaneGraph&, Real now, AnnouncementBoard& out);
  void registerAnnouncer(const char* kindName, AnnounceFn fn);
  ```

  The people sim registers one for people about to step onto a crosswalk, the rail sim one for trains nearing a
  level crossing, the city sim one for the player's car. The driving library calls them and never knows what they are.
- **What a kind means is rule-set data,** not code:

  ```lua
  announcers = {
    car    = { settle = "by_control" },                       -- right of way from the junction's control table
    person = { goes_first = true, margin = 1.5 },             -- seconds clear a driver must leave
    train  = { goes_first = true, margin = 5.0, keep_clear = true },
    player = { settle = "by_control", margin_times = 2.0 },   -- a guess about someone who obeys nothing
  }
  ```

  Adding trams, a marching band or a herd of cows is a row here and a function in the host.
- **Nothing is ever withdrawn.** Because the board is rewritten every tick, an announcer that changes its mind or
  disappears simply isn't there next tick. There is no stale booking to clean up, which is where the old model's
  standoffs came from.
- **Fixed order.** The board is sorted by crossing, then arrival time, then id before anyone reads it, so results
  don't depend on which thread or which sim wrote first.

Two smaller openings follow the same idea:

- **Path obstacles.** Anything not in the model that sits on the road (the player's car, a wreck, a person lying in
  the street) registers its outline. `notice` lays outlines onto paths and treats each as a stopped car ahead.
- **Holds.** The host can place a named hold on a driver at a point on its path and release it later. A bus dwelling
  at a stop, a cab waiting for a fare and a delivery are all holds. The core has no idea what a bus is.

The check, per kind of change:

| Tomorrow we want… | What changes |
|---|---|
| A new sign or road marking | one row in the rule set's device catalogue |
| A new country's rules | one new rule-set file |
| A new kind of vehicle | one row in the vehicle data |
| A new kind of junction control (say a roundabout) | one entry in the rule set's control table; the road builder must be able to draw it |
| Emergency vehicles that others pull over for | one new fact ("siren behind me"), then rules |
| Wet roads slow people down | one new fact ("road is wet"), then a rule |
| A move no path exists for (three-point turn, reversing out of a bay) | new geometry from the road builder, and possibly a new primitive in `act`. This is the expensive kind. |

## 5. Road data: the lane graph

### 5.1 Three layers, kept apart

"Road data" is three things with different lifetimes. Keeping them apart is what makes a rule set switchable.

| Layer | Says | Made by | Changes when |
|---|---|---|---|
| **Shape** | lanes, connectors, crossing points, crosswalks, parking spaces | the road builder, baked | the roads change (a lanes tag bump) |
| **Control plan** | which junction has lights, a stop, a give-way or nothing; speed limits; every regulation and where it stands | the road planner, written into the road graph as intent | the plan or the placement policy changes |
| **Meaning** | what each regulation means to a driver; what its sign and paint look like | the rule set, at load | the rule-set file changes |

The control plan names regulations by **abstract kind** (`stop`, `give_way`, `signal`, `speed_limit`), never by a
country's sign. The rule set turns the kind into a meaning and a picture. Under "roads: no guessing", the planner
writes the control plan using a placement policy that is also rule-set data (for example "a local street meeting an
arterial gets a stop on the local street"); the builder and the sim never infer one.

What a switch costs:

- Different behaviour numbers or sign artwork: reload the rule set. No road work.
- Different placement policy (a country with no all-way stops): re-run the control plan. No shape rebuild.
- Driving on the other side of the road: a full road rebuild. Lanes and connectors are different shapes.

*As built* (task B4, ADR-0403): the placement policy is the rule set's `placement` table (`class_order`,
`speed_limits`, ordered `junctions` rows: conditions on a junction's facts, then a control and the signs at its major
and minor approaches). The planner (`lanes/control_plan.h`) writes the plan into the road graph JSON: a `junctions`
list (each junction's control, why, its arms and the regulations where they arrive) and each road's `speed_limit`;
`city_plan controls GRAPH RULES --write` re-runs it with no shape change. The lane graph builder matches each built
junction's lanes to the planned arms and carries the control, the signs (regulations at arriving lanes' ends) and the
posted limits through; a junction the plan does not name, or names twice with different controls, is reported and left
`not_planned`, never filled in. Controls are named as the sign kit's `means` names them: `stop` (the design's
`minor_stop`), `all_way_stop`, `give_way`; and `signal`, `unmarked`.

### 5.2 Types

Sketch only; field names are for the implementer to settle.

**As built** (task A3, `src/engine/procgen/city/roads/lane_graph.h`), where it differs from the sketch below:

- a connector records the lane it comes *from* as well as the lane it joins;
- a lane's "next" may be another lane (where a section is cut with no junction), not only connectors;
- crossing points and crosswalks may belong to no junction (a crosswalk mid-block, a rail track across a street);
- rail tracks have their own table, so a crossing point's other side can be a path, a crosswalk or a track;
- a parking space is called `CarSpace`, because the word for a kind of lane is data and may not appear in the code;
- tags, regulation kinds, controls and road classes are name tables carried by the graph, at most 32 names for
  each set of tags;
- one path is 104 bytes plus its curve points; curves are stored in full precision until the island is measured;
- crossing points (task B3, ADR-0400) come in three kinds, cross, join and **split** (two paths leaving one lane);
  each path lists the ones it is part of (`Path::crossings`, by where they begin along it), so a record stored once is
  found from either path; two paths that do not run through one junction (two junctions' connectors, or a connector and
  a lane) are filed under no junction;
- each path has a **bend profile** (issue #281, ADR-0404): a short run of (distance along, radius) pieces in one shared
  array, measured once by the builder as a vehicle feels the bend (section 8); `tightestRadius` is a lane's tightest
  piece, and a connector's designed arc (what closes it to a vehicle that turns wider). A path with none bends at its
  `tightestRadius` throughout.

```cpp
struct LanePath {                 // a lane or a connector
    PathId id;
    PathKind kind;                // lane or connector. Not "through/turn/ramp": those are tags.
    uint32_t tags;                // through, turn, ramp, aux, parking, bay, bus_only ... (names from data)
    Curve2 curve;                 // centre line with distance along it; height from the road deck
    Real length, width;
    Real tightestRadius;          // so a class with a wider turning circle can be refused
    SectionId section;            // lanes only
    int indexInSection;           // 0 = offside ... last = kerb
    JunctionId junction;          // connectors only
    Movement movement;            // connectors only: straight, near turn, far turn, U-turn (from its shape)
    uint32_t allowedClassTags;    // which vehicles may use it
    Real speedLimit;              // metres per second; the rule set converts from mph at load
    Span<PathId> next;            // lanes: the connectors leaving it. connectors: the one lane it joins
    Span<RegulationRef> regulations;   // sorted by distance along
    Real waitInsideAt;            // connectors: where a far turn may wait inside the junction; <0 = nowhere
};

struct Section {                  // lanes side by side, same start and end
    Span<PathId> lanes;           // offside to kerb
    Span<LaneLine> lines;         // between neighbours: may it be crossed, and from which side
    RoadClassId roadClass;
};

struct CrossingPoint {            // two paths in one junction that cross or join
    PathId a, b;
    CrossingKind kind;            // cross or join
    Real aFrom, aTo, bFrom, bTo;  // the stretch of each path where two bodies could touch
};

struct Junction {
    Span<PathId> connectors;
    Span<CrossingPoint> crossings;
    Span<Crosswalk> crosswalks;   // each crosses some paths at some stretch; tied to the walkers' network
    ControlId control;            // from the control plan: an entry in the rule set's control table
    Polygon box;                  // the area that must not be blocked
};

struct Regulation { RegulationKindId kind; PathId path; Real at; Real value; };   // "stop here", "25 from here"
struct ParkingSpace { PathId lane; Real from, to; };                               // on a parking lane or a bay
```

Design choices worth stating:

- **Sections.** The builder makes lanes freely (pockets born out of a neighbour, extra lanes at ramps, lanes that
  end). A compile step cuts them wherever the set of neighbours changes. Inside a section, changing lanes is "the next
  index over, at the same fraction along". A turn pocket opening, a lane ending, a ramp joining the freeway and a car
  pulling out from the kerb then all become the *same* thing: a lane change with a deadline. One mechanism, not four.
- **Kerbside parking is a lane.** A parking lane is the kerb-most lane of its section, with spaces along it. Pulling
  out is a lane change from it; parking is a lane change into it and stopping at the space. That is how principle 5
  ("pulling out of a space follows a real path") is met with no special code. Bays at an angle need a short path in
  and out each (section 15, reversing).
- **Crossing points are shape; who gives way is not.** The builder finds where paths cross, in 3D (a bridge over a
  street is not a crossing). Who gives way depends on the rule set, so it is worked out at load (`junction_plan`).
- **The lane graph is whole-world and fixed.** Routes and out-of-sight trips need all of it in memory. Its size on the
  island must be measured in the first task; if it's large, curves are stored compactly, but it is not streamed.

### 5.3 What the builder must guarantee, and check

These are checks that fail the build, not warnings:

- Every connector is drivable by every class allowed on it (its tightest radius is no tighter than their turning
  circle). A movement that can't be made drivable is **refused and reported with its place**, and the lanes leading
  to it don't offer it.
- Every lane can be reached and can be left. No lane leads only to a dead end unless the dead end has a turnaround.
- Paths join smoothly: position and direction match where a lane meets a connector, so reading a car's direction off
  the curve never jumps.
- Every crossing between two connectors at the same level is in the table, and none between different levels is.
- Paint, signs and data agree: every arrow painted on a lane matches the connectors leaving it; every regulation has
  its sign or paint, and every sign has its regulation.
- Every junction has a control from the control plan. None is left to a default.

### 5.4 Bus lanes and bike lanes

The road network has neither today. A road's cross-section (`RoadSpec`) does list a "bike" kind of band, but the
lanes builder never makes one, and there is no bus lane at all. Both should be things a city *may* have, chosen by
the planner, and neither should need new code in the driving model.

**A bus lane or bike lane is an ordinary lane with three pieces of data:**

1. *who may use it* (the lane's allowed vehicle tags: `bus`, or `bike` and `moped`);
2. *its width* (a bike lane is about 1.8 m, a bus lane a full 3.3–3.6 m);
3. *its paint* (section 5.5).

The driving model already routes and changes lanes by "may this class use this lane", so a bus keeps to the bus
lane and a car stays out of it with no rule written for either. Kinds of lane become rows in a table read by the
road builder, replacing the fixed list in C++:

```lua
lane_kinds = {
  travel  = { width = 3.3, open_to = "all" },
  parking = { width = 2.5, open_to = "all", parking = true },
  bus     = { width = 3.5, open_to = { "bus", "taxi" }, paint = { fill = "red", legend = "BUS ONLY", line = "solid_white_wide" },
              turners_may_enter = 30 },     -- metres before a junction where a car turning at the kerb may use it
  bike    = { width = 1.8, open_to = { "bike", "moped" }, paint = { symbol = "bike", line = "solid_white_wide" } },
}
```

**Where they go is the planner's choice, and optional.** A city's brief says how much it wants ("no bike lanes", "a
few", "a network"), and the planner places them as *routes*, not street by street:

- **Bus lanes follow the bus lines.** On the stretches of a line that run along a road with two or more lanes each
  way, the kerb-side travel lane becomes a bus lane. This means the bus lines have to be known when the roads are
  planned. Today the city sim works them out afterwards from the finished streets, so that step moves into the
  planner. That is a real change and is listed in the tasks.
- **Bike lanes form a connected network** between homes, the centre, schools and stations, along collectors and
  quieter arterials. A bike lane that starts and stops is worse than none.

**Where the room comes from** (Glenn, 2026-11-11):

- **Bus lanes take a lane the road already has:** the kerb-side travel lane, painted red and open to buses and
  taxis, as in San Francisco. The kerbs don't move.
- **Bike lanes and kerbside parking widen the road.** The street's cross-section is planned wider from the start, so
  the kerbs, the blocks and the lots beside it are laid out around the wider road. This is a decision made when the
  city is planned, not something added to a finished city: adding either to an existing level re-plans its blocks
  (a lots tag bump as well as a lanes one, and the level suite after it).
- More parking is wanted in general (open question 2, answered). The planner checks spaces against the people who
  will arrive by car in each district, and widens streets for kerbside parking or adds car parks until there are
  enough.

**At junctions,** which is where these lanes get hard:

- A car turning at the kerb has to cross the bike lane beside it. The builder records that as a crossing point
  between the car's turn and the bike's straight-on path, and the rule set's table says the turning car gives way.
- A car turning at the kerb from beside a bus lane moves into the bus lane for the last stretch (`turners_may_enter`),
  as in San Francisco. The lane is cut into a section there, the line between becomes broken, and the lane opens to
  turning cars.
- Bus stops in a bus lane are stops like any other (holds). A bus stopped in its own lane holds up only buses.

**Drawbacks:** every bike lane adds crossing points at each junction along it, which makes turns slower and the
junction plan bigger. A bus lane takes a lane from cars, so it makes car traffic worse on that road; that is the
intended trade, but it will show in the measurements.

### 5.5 Lane markings

**What we have:** paint is thin strips of geometry laid 2 cm above the road, in white and yellow: a broken white line
between lanes, a solid yellow centre line, solid white edges, stop bars, striped crosswalks, and turn arrows built
from a strip with a triangle on the end. Which arrow goes in which lane is worked out by a rule of thumb
(`assignLaneMoves`: "the outside lane turns, the middle ones go straight"), which is a guess.

**Three changes.**

**1. What to paint comes from the lane graph, never a rule of thumb.** The same data drivers obey:

| Paint | Comes from |
|---|---|
| The line between two lanes | The lane line record in the section: same direction and may be crossed → broken white; same direction, may not be crossed (the approach to a junction, beside a turn pocket) → solid white; opposite directions → double yellow; beside a bus or bike lane → wide solid white. |
| Edge lines | The lane's outer edge: white at the kerb side, yellow at the offside edge of a one-way or divided road. |
| Turn arrows | The connectors that leave the lane: one arrow shape per set of movements (straight, turn, straight-and-turn, both turns). A lane whose only connector is a turn also gets ONLY. |
| Stop line, STOP, give-way triangles | The regulation on that approach. |
| Crosswalk | The crosswalk in the junction data. |
| Short dotted lines through a junction | Each turning connector's edges, where two turn lanes run side by side, so drivers and the player can see which lane leads where. |
| Dotted line and merge arrow where a lane ends | A lane with no way on. |
| Striped wedge where a ramp leaves or joins | The gap between two paths that split or join. |
| Parking space marks | The parking spaces. |
| Bike symbol, BUS ONLY, coloured lane | The lane kind's paint (5.4). |
| Speed painted on the road | A speed-limit regulation, where the rule set asks for it. |

So a driver in a lane with a painted turn arrow always turns, because both came from the same connector.

**2. What the paint looks like is a catalogue in the rule set,** drawn to the US manual (the MUTCD) and credited:

```lua
markings = {
  lines = {
    broken_white     = { colour = "white",  width = 0.10, dash = 3.0, gap = 9.0 },   -- 10 ft line, 30 ft gap
    solid_white      = { colour = "white",  width = 0.10 },
    solid_white_wide = { colour = "white",  width = 0.20 },
    double_yellow    = { colour = "yellow", width = 0.10, double = 0.10 },
    dotted_white     = { colour = "white",  width = 0.10, dash = 0.6, gap = 1.8 },   -- through junctions, lane ends
    stop_line        = { colour = "white",  width = 0.45 },
  },
  symbols = {                                  -- shapes drawn once, flat, in metres
    arrow_straight      = { shape = "arrow_straight",      long = 2.9 },
    arrow_turn          = { shape = "arrow_turn",          long = 2.4 },             -- mirrored for the other side
    arrow_straight_turn = { shape = "arrow_straight_turn", long = 3.9 },
    give_way_teeth      = { shape = "triangle_row" },
    bike                = { shape = "bike_rider" },
  },
  legends = { height = 2.4, stretch = 2.5 },   -- letters on the road are drawn tall so they read from a low angle
  crosswalk = { style = "ladder", bar = 0.6, gap = 0.6 },
}
```

Today's broken line is 3 m on, 6 m off; the American standard is 3 on, 9 off, one of several small things that
make a road look right. A different rule set gives different paint with no code change.

**3. How it is drawn.** Two kinds of mark, two methods:

- **Lines stay as strips** of geometry, as now (they are long, thin and follow the road's curve and crest), but
  built by one function from a catalogue row instead of numbers written in the mesher.
- **Arrows, words and symbols become flat stamps:** each is a small patch of road-hugging quads textured from a
  shared picture sheet. The road-sign system (ADR-0110) already lays out lettering and arrows and packs them into
  cached sheets; road legends use the same lettering and the same cache, with shapes stretched lengthwise. This
  replaces arrows assembled from strips, and makes STOP, ONLY, BUS, numerals and the bike symbol possible at all.
**4. Sitting flush on the road.** Today paint breaks on hilly roads (Glenn, 2026-11-11), and stamps would break worse,
because they are wider. The cause is in how paint is fitted now (`deck_mesh.cpp`, "paint on the drawn road"):

- Each *corner point* of a paint strip is moved onto the road triangle under it and lifted 2 cm. But the flat piece
  of paint *between* its corners is not touched. The road surface is made of triangles that fold along their edges;
  where a fold runs under a piece of paint, the paint bridges it. Over a crest the middle of the paint sinks into
  the road; in a dip it floats. On flat roads there are no folds, so it only shows on hills, cambered curves and ramps.
- A corner that falls just outside every road triangle (an edge line on the very edge of the road) is not moved at
  all and keeps its old height, so the strip kinks there.
- A fixed 2 cm lift is too much up close (a visible step, and a shadow under it) and too little far away (the paint
  flickers against the road).

The fix is one shared operation, used by every decoration that lies on a road, and no per-decoration fitting:

- **Cut the decoration along the road's own triangle edges** ("draping"). Given a flat outline and the road mesh
  under it, the outline is split wherever a road triangle edge crosses it. Every piece then lies inside exactly one
  road triangle and takes that triangle's plane, so it matches the surface exactly, on any hill, with nothing left
  to bridge a fold. A piece that reaches past the road's edge is trimmed there.
- **No lift in the geometry.** Paint is given the same heights as the road and drawn with a depth bias (a renderer
  setting that says "when this is level with what's behind it, draw it on top"). That removes both the step up close
  and the flicker far away. It is one flag on the paint material, the same in every renderer backend.
- **One function, in the mesh library** (ADR-0173: add a word, never a private helper): `drapeOnMesh(outline,
  mesh)`. Lines, stamps, crosswalks, parking marks, coloured bus and bike lanes, manhole covers and patches all go
  through it. The existing per-corner fitting is removed.
- **Which surface, where roads stack.** Under a bridge there are two road surfaces at one spot. The decoration
  names the lane it belongs to, and is draped only on triangles of that lane's deck. Today's "the triangle nearest
  in height" guess goes.

What it costs: more paint triangles where the road is finely folded (curves and crests), none extra on flat
straights. Paint is a small share of a road's triangles today, so this should stay small, but it is measured in the
task.

The alternative considered: draw the markings in the road's own material from the lane position each road triangle
already carries, with no paint geometry at all. That is flush by construction and fine for plain lane lines, but
arrows, words, crosswalks and anything that differs lane by lane would need a lookup per lane inside the shader.
Draping handles every kind of decoration one way, so it is the choice; the shader route stays open for lane lines
alone if triangle counts ever demand it.

Two things for looks, both in the paint material and optional:

- **Wear:** paint fades where tyres run, more on busy roads. A wear amount per mark, from the road's class at first
  and from real counted traffic later.
- **Shine at night:** road paint has glass beads in it and lights up in headlights. The sign system lists the same
  thing as not yet done; one material feature serves both.

**Checks** (added to 5.3): every lane line record has paint and all paint has a record; no line is painted through
a junction except the dotted guides; every stamp lies wholly on road surface.

**What this costs:** it is a change to the road mesher's output, so every city's roads are rebuilt (the same lanes
tag bump the lane graph already needs; they should land together). Stamps add a material and a texture sheet per
level. Lettering seen from far away at a low angle will shimmer unless the sheet has smaller copies for distance.

## 6. Rules as data

### 6.1 The five kinds of table

A rule set is one Lua file (`assets/scripts/rules/us_left.lua`) holding five kinds of thing. Only the last is a
"rule" in the if-then sense, and it is the smallest.

**(a) The device catalogue: what can stand on the road, what it means, what it looks like.**

```lua
devices = {
  stop        = { means = { stop_line = true, rank = "stop" },   sign = "R1-1", paint = { "stop_bar", "STOP" } },
  give_way    = { means = { rank = "give_way" },                 sign = "R1-2", paint = { "give_way_teeth" } },
  speed_limit = { means = { top_speed = "value" },               sign = "R2-1" },
  no_entry    = { means = { closed_to = "all" },                 sign = "R5-1" },
  bus_lane    = { means = { open_to = { "bus" } },               paint = { "BUS ONLY" } },
  no_turn_on_red = { means = { forbids = "turn_on_red" },        sign = "R10-11a" },
}
```

The `sign` codes are the US manual's own (the MUTCD, defined in the plan); the existing road-sign system (ADR-0110)
lays out the face. One row gives the driver its meaning and the player its picture, so they can't disagree.

**(b) Right of way: for each kind of junction control, the order of questions that settles two movements.**

```lua
control = {
  signal       = { settle = { "light", "movement" },
                   turn_on_red = { movement = "near_turn", after = "full_stop" } },
  all_way_stop = { settle = { "arrived_first", "kerb_side", "movement" }, everyone_stops = true },
  minor_stop   = { settle = { "sign_rank", "movement", "arrived_first" } },
  give_way     = { settle = { "sign_rank", "movement" } },
  unmarked     = { settle = { "arrived_first", "kerb_side" } },
}
movement_rank = { "straight", "near_turn", "far_turn", "u_turn" }   -- earlier goes first
```

`settle` is read left to right: the first question that tells the two movements apart decides. The questions
(`light`, `sign_rank`, `movement`, `arrived_first`, `kerb_side`) are a small fixed vocabulary in C++. Questions that
depend only on the road (`sign_rank`, `movement`, `light` per signal phase) are answered **at load** and stored per
pair of movements. Questions that depend on the moment (`arrived_first`) are stored as "ask while driving". A
roundabout is `give_way` on every entry; it needs no new question.
*As built* (task B4, ADR-0403): the two-way stop row is named `stop` (as the sign kit's STOP sign names it), and a row
with lights may name `when_dark`, the row it is treated as while no light runs a programme (a host that registers no
light clock: section 9.1). A sixth table, `placement`, is the planner's (section 5.1).

**(c) Numbers: gaps, margins, timing, defaults.** How many seconds clear a far turn needs; the safety margin at a
crossing point; how patience shrinks it and its floor; default speed limits by kind of road, in mph; how far before a
turn to indicate; yellow and all-red timing by formula. Every number carries a comment naming its source.
*As built* (#255, ADR-0399): the default limits are their own table, `default_limits = { local = { 25, unit = "mph",
source = "..." }, ... }`, keyed by the lane graph's road class and written like the numbers; a path with no posted limit
takes its class's (a connector the class of the lane it joins), bound once at load. A personal setting may carry a
`source` too.

**(d) Personality ranges.** For each driver setting (gap kept, gap accepted, speed habit, patience, attention, how
pushy, indicating, horn), a default and a range. A person's own values are worked out from their identity by the
stateless hash in `src/hash.h`, so they cost no memory and are the same every day.

**(e) Modes and rules: what a driver does with what it notices.**

```lua
modes = {
  driving     = { rules = "all" },
  lane_change = { rules = "all", also = { "hold_gap_both_lanes" }, lasts = { 3, 4 } },   -- seconds
  pulling_out = { like = "lane_change" },
  parking     = { like = "lane_change", ends_stopped = true },
}

rules = {
  { name = "red light",       when = { "light", "is", "red" },          unless = "inside_junction",
    give = { stop_at = "stop_line" } },
  { name = "yellow, can stop", when = { all = { { "light", "is", "yellow" }, "can_stop_comfortably" } },
    give = { stop_at = "stop_line" } },
  { name = "stop sign",       when = { all = { { "sign_rank", "is", "stop" }, { "not", "has_stopped_at_line" } } },
    give = { stop_at = "stop_line" } },
  { name = "giving way",      when = "must_give_way",                   give = { stop_at = "give_way_point" } },
  { name = "keep the box clear", when = { "room_beyond", "<", "room_needed" }, give = { stop_at = "stop_line" } },
  { name = "speed limit",     give = { top_speed = "my_limit" } },
  { name = "corner",          give = { top_speed = "corner_speed" } },
  { name = "lane ends",       when = "lane_ends_ahead",                 give = { stop_at = "lane_end", move = "to_open_lane" } },
  { name = "get in lane",     when = { "lanes_to_cross", ">", 0 },      give = { move = "toward_route_lane" } },
  { name = "indicate",        when = { "turn_within", "<", "indicate_distance" }, give = { indicate = "turn_side" } },
  { name = "person crossing", when = "person_on_my_crosswalk",          give = { stop_at = "crosswalk" } },
  { name = "horn: cut off",   when = { all = { "cut_off", "quick_to_horn" } }, give = { action = "horn" } },
}
```

**As built** (task C5, `src/apps/driving/decide.h`):

- Rules are an ordered list. A condition is a fact's name, `not`, `all`, `any`, or one comparison whose left side
  is a fact. `unless` means "and not". A comparison with a fact that has nothing to report is false.
- The right side of a comparison names a rule-set number, a personal setting or another fact. A number written
  straight into a rule is allowed **only against a plain ratio or count** (`lanes_to_cross > 0`). Against a speed,
  a distance or a time it is refused, because it would have no unit: the rule must name a number, which carries one.
- So a personal setting can't sit on the left: "this driver is quick to use the horn" is a yes/no fact
  (`quick_to_horn`), worked out in C++ from the setting, as in the last rule above.
- There are no rule priorities. Answers combine by fixed arithmetic, so a priority would do nothing.
- Intentions and actions are a vocabulary registered in C++ like facts, because code has to act on them.
- Deciding reads no strings: about 180 billionths of a second for a dozen rules.

### 6.2 Where the weight sits, honestly

Read (e) again: the rules are thin. "Giving way" is one line because the fact `must_give_way` did the work, using
tables (b) and (c), the junction plan and the announcements. That is deliberate:

- The rules of the road that differ between countries are mostly **tables** (who goes first, what signs mean, how
  long, how far). Tables are easy to read, check against a law book and swap.
- The **if-then rules** are kept few and dumb: conditions that compare named facts with named numbers, joined by
  "all", "any" and "not". No arithmetic, no variables, no loops, nothing that looks at another driver's state. When a
  rule wants arithmetic, the result becomes a fact (`my_limit` is the limit times this driver's speed habit, worked
  out in C++).
- The **facts** are where the engine's cleverness lives, and they are general: `must_give_way` knows nothing about
  stop signs. It asks the junction plan who outranks it and the announcements whether there's room in time.

So the plan's risk ("how much can descriptions express before a rule wants to be a program") has a planned answer:
pressure goes into new facts and new table columns, never into making the rule language richer. If a rule can't be
written, the rule language does not grow; a fact does.

### 6.3 Modes: only where memory is needed

The plan lists thirteen states (cruising, following, approaching, stopping, waiting, giving way…). Most of those are
not states a driver has to *remember*; they are descriptions of which answer is currently binding. A car is
"following" when the car ahead is what limits its speed. Making each a stored state invites the old bug: a car in
state "waiting" with nothing to wait for.

So the design keeps two things apart:

- **What the card shows** is the binding answer's rule name. All thirteen of the plan's words appear here, for free,
  and can never be stale.
- **Modes** are the few things that need memory across ticks: a lane change under way (which needs to finish or
  abort), pulling out, parking. They are data (above), run on the engine's existing `engine::StateMachine`, as the
  people's goal tables already do (ADR-0064).

A few facts also need memory ("I came to a full stop at this line", "I've been waiting 9 seconds"). Those are
**remembered facts**: the rule set declares when each is set and cleared; C++ stores one small value per driver.

### 6.4 Loading, checking, reloading

- The reader turns the Lua into flat arrays: each condition becomes a short list of "compare slot N with value V"
  steps over a per-driver array of fact values. No strings and no Lua at tick time.
- Load-time checks, each an error naming the file and the full key (`us_left.lua: rule_set.devices.stop.means.rank
  is "halt", which is not one of ranks (...)`; a value in a Lua table carries no line number, so the key is what
  can be named): unknown fact, device, answer or mode name; a number without
  a unit; a `settle` list that can leave two movements tied (the reader appends a final
  tie-break and warns); a regulation kind used by the road data that the rule set doesn't define. All faults in a
  file are reported together, not only the first.
- **Units.** As built (task C3): every number carries its own unit, `default_limit = { 25, unit = "mph", source =
  "..." }`, from a short list (s, m, ft, mph, km/h, m/s, m/s2, ft/s2, ratio). The reader converts and keeps what
  was written for the driver card. C++ only ever sees metres and seconds. Numbers with no `source` are counted.
- **Reloading while running** (the Lua hot-reload from ADR-0259): tables are swapped between ticks. Cars keep their
  state; a car in a mode that no longer exists returns to `driving`. Junction plans are worked out again.
- **Rule coverage.** The lab counts how often each rule was the binding one. A rule that never binds in a full lab
  run is either untested or dead, and the report lists it.

### 6.5 What will still need C++

A new fact; a new `settle` question; a new answer kind (should be very rare); a new way of moving (`act` primitive);
new road shapes. Everything else is data. A review that finds a sign's name, a vehicle kind's name or a country in the
`driving` library has found a bug.

## 7. The tick

### 7.1 Three steps

Each tick runs three steps over all the cars being driven. The split is what makes the result the same every run and
lets the work go on several threads.

1. **Plan** (many threads; reads last tick's state, writes nothing shared). For each driver due this tick: work out
   its facts, run the mode table and the rules, combine the answers, and write down what it *wants*: an acceleration,
   and any claim (start a lane change into that gap; enter that junction; take that parking space).
2. **Settle claims** (one thread per junction or section, in a fixed order). Two cars may have wanted the same thing
   in the same tick. Claims on each shared thing are granted in a fixed order (by right of way, then arrival, then
   position, then id), each checked again against the ones already granted. The loser simply doesn't start, and
   plans again next tick.
3. **Move** (many threads). Each car updates speed, `s`, `d`; hops to the next path when it runs off the end of this
   one; the per-path lists are put back in order; events are queued for the host.

Nothing reads a value another car wrote in the same step. Any randomness (an attention slip) comes from a hash of
the person, the tick and the question, never from a shared generator, so thread order can't change the outcome.

### 7.2 How often

Deciding and moving run at different rates:

| Level of detail | Decide | Move |
|---|---|---|
| Near | 10 times a second, cars spread evenly over the ticks | every tick |
| Middle | 2 times a second | 10 times a second |
| Out of sight | never | never (the schedule moves the person) |

These are starting numbers for the rule set's `detail` table, to be tuned against the cost target. The "could I stop
in time" speed limit (section 8) takes the decide interval as the driver's reaction time, so a car that decides less
often keeps a longer gap and stays safe. That is what lets the middle distance be cheap without different rules.

### 7.3 Lists per path

Each path keeps its cars in order of distance along. The car ahead is the next in the list, or the last car on the
next path of the route, and so on up to the look-ahead distance. Two details that cause classic bugs if skipped:

- **A car's tail.** A car whose front has moved onto a connector still has its back half on the lane behind. Each
  path also records "a tail is still here, ending at…", so a follower sees it. A bus or a truck with a trailer can
  cover three paths at once at a small junction; the record is a short list, not a single slot.
- **Changing lanes.** For the 3–4 seconds of a lane change the car is in *both* lanes' lists, so cars in both keep
  their gap from it.

Look-ahead is always measured in metres along the route, through however many paths that is. Nothing may assume "the
junction is at the end of this lane": the old model broke when roads were cut into 4 m pieces, and connectors will be
shorter than that.

*As built* (#283, ADR-0408): **at a split** (two paths leaving one lane) the car ahead may have left by the other path while
the two still run within touching distance (57 m at the lab's junction 21). A car whose body lies on the other path's
stretch of the split, its front further along than mine, is my car ahead, its rear placed where it is level on my path
(`carAhead`). Before this, a car that turned off my lane vanished from my following as soon as its tail left the lane.

## 8. Speed and following

One formula decides acceleration, and every stop answer goes through it.

- **Easing toward a speed and off a car ahead** (the *Intelligent Driver Model*, IDM, as in the plan). In words: speed
  up toward your own top speed, less so as you near it; ease off as the gap to the car ahead falls below the gap you
  like, which is a standstill distance plus so many seconds of travel plus extra if you're closing fast.

  `accel = a · [ 1 − (v / v_top)⁴ − (gap_wanted / gap)² ]`
  `gap_wanted = gap_min + v · T + v · (v − v_ahead) / (2 · √(a · b))`

  with `a` the car's comfortable speeding-up, `b` its comfortable braking, `T` the seconds of gap this driver keeps.
- **A place to stop by is a stopped car at that place.** The stop line, a give-way point, a lane's end, a hold, an
  obstacle: each is fed to the same formula as `v_ahead = 0` at that distance. The lowest acceleration over all of
  them is taken. There is one stopping behaviour, so a car stops at a red light exactly as smoothly as behind a bus.
- **The hard limit** (the *Krauss safe speed*): never faster than the speed from which you could still stop if the
  car ahead braked as hard as it can, given your reaction time `τ`:

  `v_safe = −b_max · τ + √( (b_max · τ)² + v_ahead² + 2 · b_max · gap )`

  Speed after the tick is capped at `v_safe`. This is what the test "following can't crash" proves, at every decide
  interval in the table above.
- **Corner speed** is a top-speed answer from the path's tightest radius ahead and a comfortable sideways
  acceleration from the rule set, applied early enough to brake comfortably.
  *As built* (#281, ADR-0404): each path carries a **bend profile** (pieces of its length, each with the radius a
  vehicle feels there), measured by the lane graph builder from the line's shape over a stretch whose length follows
  from how far a driver may stray from the drawn line (`kBendRoom`), so a kink between two points is not a hairpin.
  `corner_speed` reads every stretch of the profiles on the way ahead out to the look-ahead (a connector as one stretch
  at its tightest, the speed the junction step announces), and gives the speed the car may do now so that, holding it
  until its next decision and then braking comfortably, it meets each stretch at or below that stretch's speed.
- **Hills.** A class's speeding-up and braking are adjusted by the path's slope, from its own power and weight
  figures. A loaded truck is slow uphill because of its data, not a rule.

All numbers are per vehicle class or per person, from data. Speed is never negative; a car at rest stays at rest
until its binding answer lets it go.

## 9. Junctions

This is the part the old model got wrong, so it is specified most tightly. Everything here is built on crossing
points and announcements.

### 9.1 The junction plan (at load)

For each junction, from its connectors, its control and the rule set:

- For each pair of movements that share a crossing point: who gives way, for each signal phase if there are lights.
  One of three values: *A gives way*, *B gives way*, or *settle while driving* (with the questions left to ask).
- For lights: the signal programme. Movements are grouped into phases so that no two movements in a phase have to
  cross each other unless the rule set allows it (a far turn that gives way to oncoming traffic). The existing
  signal controller's method (ADR-0060, ADR-0109: grouping by what conflicts) is kept, but it groups *movements*
  instead of whole approaches, and timing comes from the rule set. Yellow lasts
  `reaction time + speed / (2 × braking)` (the ITE formula in the plan), so a 25 mph street gets about 3 seconds.
  The light's state stays a pure function of the clock, so out-of-sight junctions cost nothing and never drift.
- For each connector: the list of crossing points along it, in order, with the movements it must give way to at each.

*As built* (task D1, ADR-0401): the table is `src/apps/driving/right_of_way.h`, built by `Driving::useRules`; a
connector's crossing points are read from `Path::crossings` and the table, not copied. Each driver also keeps a plan for
the junctions on its way ahead (`junction_plan.h`: junction, connector, turn, where the connector starts, its waiting
points), made when the junction comes into view and kept until it is through; `DriverView` carries both to the facts.

*As built* (task D2, ADR-0406): **the lights' clock** is `src/apps/driving/signal_programme.h`. The phases stay D1's
(grouped by what conflicts, per movement, in `right_of_way.h`); at load each lit junction's programme is timed from the
rule set's numbers: the **yellow** (the warning that red is coming) by the ITE formula, reaction + approach speed / (2 x
deceleration + 2 x gravity x grade), the approach speed being the arriving lane's limit and the grade its climb at the
line, held to 3-6 s (MUTCD 4D.26); the **all-red** (every movement losing its green shows red while the last cars clear)
by ITE's (width to clear + a 20 ft vehicle) / approach speed, the width being how far along the turn its last crossing
ends; the **greens** share a target cycle (40 s in the lab) less the yellows and all-reds, equally, held to 7-30 s; and a
**green wave** offsets each junction so the phase serving its road most nearly along the wave's line begins as a car at
the wave's speed would reach it (east at 30 mph in the lab), which also makes two close lights serve one road together.
A movement green in two phases running one after the other stays green through the change. Which phase a junction is in,
and what each movement's lamp shows (green, yellow, red, "red, may turn"), are **pure functions of the sim time**
(`signalAt`, `currentPhase`, `lampAt`): nothing is stored per junction while cars drive, and drivers, the conflict
resolver, the driver card and anything that draws lamps read the same answer. `checkSignalProgrammes` checks every stage
of every programme: no two movements whose paths meet let traffic in together unless the rule set lets one give way.
**Drivers** read three facts (`light`: the lamp at the nearest lights ahead that are not green for them, else the nearest;
`light_line`; `can_stop_for_light`: the dilemma, by the yellow's own deceleration and the decide interval, kept once a
driver has begun to stop) and the lab's rules say what to do: stop at a red, stop at a yellow if you can, turn on red
after a full stop where `turn_on_red` allows, and the give-way table read at the current phase does the rest. A car
inside the junction when its light turns red is **committed** (it entered lawfully; the next green waits for it), a red
before the line starts the approach afresh (its decision and arrival were for a green that is over), and the lights run
only for a host that registers the light clock (`registerLightClock`); otherwise `when_dark` still holds.

### 9.2 Approaching: announce

Once a junction is within look-ahead, a driver announces for each crossing point on its connector: the time it
expects to reach it, the time its tail will be clear, the speed it will be doing, whether it must stop first (a stop
sign, a red), when it arrived at the line, and how long it has waited. An announcement is worked out from the
driver's own state alone, so all of them can be made in the plan step without order mattering.

*As built* (task D3, ADR-0402): `src/apps/driving/announce.h`. The board is rewritten at the start of every plan step
from the cars as they stood at the start of the tick, filed by crossing point (a counting sort over the dense crossing
ids: an indexed flat array, found in one lookup), each crossing point's few in order of arrival, then who. Who announces
is a time (`junction_look_ahead`, 10 s in the lab); a driver's way ahead is at least that long, and never shorter than
the junctions it has planned for. One thing is not the car's own: a queue for one line announces no earlier than the
car in front plus a following interval (gap kept, plus that car's length at the speed through), worked nearest the line
first, and the interval carries on, shrinking, after the car in front has crossed. A car standing beyond the line
limits how far the car behind can get: it announces "not going" for crossing points it cannot reach and "never clear"
of one it would stop on. Besides when it reaches the line, it announces whether it is GOING (it cannot stop at the
line, or will not be able to by its next decision) and its fixed ARRIVAL (9.3).

### 9.3 The fact `must_give_way`

For each crossing point on my connector, for each announcement by a movement I give way to (from the junction plan,
asking the "while driving" questions if needed): is my time on the crossing point clear of theirs by the safety
margin? If not, I must give way, and the fact records to whom and by how many seconds.

- The margin and the gap needed come from table (c). They shrink as the driver's wait grows, down to a floor the rule
  set sets. That is "patience runs out gradually"; it is not a timer that overrides a rule, and the floor is still safe.
- The one with right of way does not check the one giving way. The asymmetry is what stops two cars each waiting for
  the other. Where the junction plan says "settle while driving", the questions always end in a tie-break that
  can't tie (lower lane id, then lower car id), so exactly one of any two gives way.
- **Where to wait.** Normally at the stop line. A far turn on a green may pull forward to the connector's
  `waitInsideAt` and wait there for a gap, as American drivers do. It has then entered the junction (below).

*As built* (task D3, ADR-0402): `src/apps/driving/give_way.h`. The judgement is worked out once per decision and the
facts (`must_give_way`, `give_way_line`, `give_way_gap`, `give_way_gap_needed`, `committed`, `stopped_at_line`, and for
full stops `next_line`, `everyone_stops`, `sign_rank`) read it; the rule set says what to do with them
(`{ when = "must_give_way", give = { stop_at = "give_way_line" } }`). Pairs asked while driving are ordered: a car that
is going first; then by each car's ARRIVAL at its line, fixed once when its driver first judges the junction (as its own
estimate then had it) or, for a full stop, when it comes to rest there; then the table's fall-back; a ring of
fall-backs among cars arriving in one tick is broken at the lower car id. The arrival is fixed rather than read off the
line because read off the line a car that had stopped to give way to me took the right of way from me as I arrived.
A driver decides only within its decision horizon (its comfortable stopping distance and one decision's travel), and a
driver that is not going (held, or blocked beyond the line) judges but decides nothing. A full stop (all-way stop, stop
sign) is a rule in the rule set, not C++.

### 9.4 Inside the junction: first in goes first

A car is **committed** once it has passed its *last waiting point*: the stop line for most turns, or the
wait-inside point for a far turn that pulls forward to wait for a gap. (Not simply "passed the line": a far-turner
waiting inside entered before the oncoming cars it is waiting for, and must still give way to them.) From then on,
for everyone, its place on each crossing point ahead of it is treated as a car to follow: a later car, *even one
with right of way*, keeps its distance from the crossing point until the earlier car's tail is clear. Separately,
any car's body lying on a crossing point blocks that point for everyone. Commit time gives a single order over the
committed cars in a junction, and a single order has no loops. The full rules are in `driving-briefs/D4`. This is the rule the SUMO paper names as what prevents deadlock inside a junction.

*As built* (task D4, ADR-0405): **one conflict resolver** (`src/apps/driving/conflict.h`) decides who goes first for
every pair of cars announced on crossing points they share, once a tick from the board, one record per pair per
junction (crossing points filed under no junction make one more group). It asks named questions in order, each a small
pure function, until one answers: whose body is already on a shared crossing point (`on_it`), who committed first
(`committed_first`), the give-way table's settled answer (`table`), who can no longer stop at its line (`going`), who
reached its line first (`arrived_first`), the table's fall-back (`fallback`), the lower car id (`lower_id`). Who yields
to whom stays policy in the rule set (the table); the resolver is mechanism, and `give_way.cpp` keeps no ordering of
its own. A car past its line (or too fast to stop at it) still judges: it stops inside, at its last waiting point
short of the crossing point (ADR-0401's gaps a body fits in) or with its front at the crossing point's stretch, for a
car the resolver puts first; the stop goes through the rule set's "give way" rule like any other (`give_way_line` is
"where I stop to give way"). Where `on_it` or `committed_first` decided, the car going second may only come behind.
Nobody plans to go behind a car waiting inside for somebody else.

*As built* (task D2, ADR-0406): **waiting inside for a far turn on a green.** The lane graph builder places each far
turn's `waitInsideAt` by shape: its front just short of the first stretch where it meets a path from the approach facing
it (`lanes/lane_graph_crossings.h placeWaitInsidePoints`; builder version 5). At load the driving model keeps a point
only at lights that run a programme, and only if, in every phase where the turn is green, it goes first at every crossing
point its body has reached by then (the D4 brief's road-data check, made against the rule set's table: 5 of the lab's
105 points fail it). A car whose whole body fits past the line has it as one more waiting point. On its own green, a far
turn that must give way at a crossing point at or beyond the point pulls forward to it instead of stopping at the line;
never on a yellow, a red or an arrow. Once its light turns red, a car past its line is committed whatever its waiting
points say, so it clears in the change and the next green waits for its body. A car from the same lane that would pass
the waiting car (two turns leaving one lane: a split) waits behind it, since its tail may be off the lane and still in
the split's stretch, where following cannot see it; it announces "not going" beyond, as a queued car does.

### 9.5 Don't block the box: room is claimed, not just seen

A car may enter only if there is room for it beyond the junction: its own length plus a standstill gap, on the lane
its connector joins, before the next place it might have to stop.

"Seeing" room is not enough: two cars from different approaches can both see the same one car's worth of room. So
room is a **claim** settled in step 2 of the tick. Each junction grants room on each exit lane in order; a car that
is not granted room stops at the line and announces that it's stopping, so cross traffic can use its turn.

Closely spaced junctions: if the next stop line is nearer than the car is long, "room" is measured through the next
junction too. A car never enters a junction it can't clear, however the junctions are spaced.

*As built* (task D5, ADR-0407): `src/apps/driving/room.h`. A driver ASKS in the plan step when it decides to go at the
nearest junction it holds no room at (one decision before it could no longer stop comfortably), or to pull forward and wait
inside (the rule set's `room_to_wait_inside`), or when, at rest at its line, it is about to move off (released by a red);
the claim is remembered on its junction plan. The settle step grants asks once a tick in one order -- first ask earlier
first; among first asks of one tick, the conflict resolver's order (`ConflictResolver::order`), then the lower id -- each
grant counted before the next. Room is measured to where the back of the queue would come to rest (each car on the stretch
braking comfortably, held behind the one ahead), from past the far crosswalk, less every car in transit (front on a
connector into the lane, or granted and not yet there) and every car first in the queue for it (refused for want of it
earlier, still at its line). A car refused stops at its line by the rule set's rule ("no room beyond": facts
`no_room_beyond`, `room_line`) and announces that it is not going. A car whose car ahead in its lane, short of the same
line, holds no grant is refused. Grants are kept until the front reaches the lane, given back when the car holds a stop at
or before the line for something else; they live on the plan, so they cannot leak. Through junctions too close to stop
between, the claim passes on, up to `room_through_junctions` (3), only where no full stop is owed and each light passed will
let the car in (green or yellow) as it gets there and for `room_arrival_slack` after; the car is committed through them. A
turn a class could never (or, at two uncoordinated lights, too seldom: `room_cycles_through`) be granted room through is
closed to that class's routes at load and reported as a road-data problem. This replaced #261's "stop at the line before
when the body won't fit" for lights. The measurements row `box_stops` checks the brief's "done when" (nobody at rest in a box
but by a recorded give-way). *Since #265 (ADR-0408)* the load check follows the whole chain a claim could pass through, up to
`room_through_junctions`, with every light on it: the soak found a car waiting 1,129 s at a turn whose claim ran through
two close junctions to a light that never let it in, which the old one-junction check had left open.

### 9.6 Why this can't deadlock, and what can still freeze

A deadlock needs a loop of cars each waiting for the next. Check each kind of waiting:

- *Waiting at the line to give way:* the waiter holds nothing anyone needs (it is outside the box), so it can't be
  part of a loop.
- *Waiting inside the junction for an earlier car:* ordered by entry time, so no loop.
- *Waiting at the line for room beyond:* the waiter is outside the box. The cars it waits on are in the next road.

That last kind can still close a loop **between junctions**: every road round a block full, each front car waiting
for room in the next road. No car is in a box and every rule is being followed, but nothing moves. That is real
gridlock, and no rule of the road undoes it. The design's answer has three parts:

1. **Don't feed it.** A car doesn't pull out into a lane with no room. Routes are costed by how long each road
   actually takes right now, so drivers avoid jammed roads. Signal cycles are short enough that one red's queue fits
   the block (the existing note in `signals.lua`).
2. **Let drivers do what real drivers do:** a driver waiting for room, whose route has another way from where it
   stands, may take it. That is a decision by a rule, shown on the card, and it changes the route, not the car's
   position.
3. **Tell the two apart, always.** Every stop answer names what it is waiting for, so the lab builds the wait-for
   graph each second and looks for loops. A loop made only of "waiting for room" arrows is **gridlock** and is
   reported as a road-design or demand problem. A loop with any other arrow in it is a **deadlock**, a bug, and
   fails the test. A car at rest with *no* arrow at all is the worst bug and also fails.

*As built* (task D4, ADR-0405): the argument above is not enough on its own, and the lab showed where. A car stopped
to keep out of a body's way can itself lie on another car's crossing point; a queue behind a car waiting at a line can
lie across a long connector of a junction next door; and a car whose body would not fit between two junctions waits at
the first junction's line for a conflict at the second, so a loop can run through two junctions. So the conflict
resolver also guarantees, by construction, that its "goes after" answers hold no ring: all its answers, together with
the arrows no answer can turn round (a car behind another on one path; a body on a crossing point), are cut into
strongly connected parts (each car in a part can reach each other by the arrows: Tarjan's method), and inside a part
with a loop every pair is answered by one order that keeps the fixed arrows (a topological order, Kahn's method), the
cars free to come next taken committed first, then going, then by arrival, then by the lower id. One order has no loop,
and the arrows between parts already had none. A loop made of bodies and queues alone, which no answer can undo, is
counted; the lab has none through bodies. One more guard keeps a car waiting inside from being held there: nobody plans
to go "behind" it, because committing behind it would put it after them.

*As built* (review #284, ADR-0409): **the guarantees now hold for what cars can do, not only for the resolver's
records.** A car that cannot keep its front short of a crossing point even braking as hard as it can announces so
(`cannotStop`), and the resolver's second question, `cannot_stop`, puts it first: nobody is told to give way who cannot;
the hard gap a driver keeps going on is never below its own stopping time. A car that will come to rest before its tail is
past a stretch (waiting inside, a red at the next close junction, a hold, no room) is never clear of it, so nobody plans to
go behind it (this replaced the separate guard above). A loop of bodies alone is left standing and shows as gridlock (an
arrow about space, `StopFor::ForABody`), never answered by telling a car to drive through another. Cars on a lane that
meets another junction's connector judge those crossing points too, and who goes first there is the rule set's
`between_junctions`. A car is committed once it can no longer wait short of its next crossing stretch (on it, or unable to
stop short of it), not a centimetre past its line. A claim through close junctions is granted only with its way through
them seen clear of every car that could make it wait, its arrival allowing for the car ahead, and is worked out afresh
every tick until the first line. Which ordering decisions live in the resolver, which in the rule set and which outside on
purpose is ADR-0409's table.

## 10. Routes and lane changes

**Routes.** The router works on sections and connectors, starting from the lane the car is actually in and the way it
faces (so a trip can never start with a swing round). The result is the list of sections and connectors ahead, and
for each section, **which of its lanes lead to the next move** and how many lanes over the car is from one. That
gives the facts `lanes_to_cross` and `distance_to_last_chance`. Costs are travel times: free-flow at first, then
what simulated drivers actually took, by time of day. The vehicle class filters out paths it may not use or can't
turn through. A destination nothing leads to is reported to the host as "no route"; the router never invents a U-turn.

**Lane changes.** One mechanism, driven by intentions:

- *Wanting to:* a "must" intention (the route needs it, the lane ends, pulling out) grows more urgent as the last
  chance nears; a "would like to" intention (the lane beside is faster) is weighed against how much it slows the car
  it would cut in front of (the plan's MOBIL).
- *Doing it:* indicator on; wait for a gap safe by the rule set (both the car ahead and the car behind in the target
  lane, by the same "could stop in time" test); claim the gap in step 2 of the tick; move across over 3–4 seconds
  while listed in both lanes; indicator off.
- *Letting in:* a driver who sees a car indicating into its lane *urgently* eases off to open a gap, by a rule. The
  size of that courtesy is a personality setting.
- *Aborting:* if the gap closes in the first part of the move, the car eases back. `d` makes that a smooth path.
- *Failing:* a driver who cannot get across before the last chance **carries on and re-routes** (the missed turn).
  It does not stop dead in a through lane to wait for a gap; that is how other simulators make their jams. The one
  exception is a lane that physically ends, where the end is a place to stop by.

## 11. Others on the road

No kind of road user is named in the core. Each is covered by one of the openings in section 4:

| Who | How the core sees them |
|---|---|
| Buses, cabs, vans | Vehicle classes with tags (a bus may use `bus_only` lanes). Stops and pickups are holds, placed and released by the host. A bus bay is a short parking-style lane. |
| Trucks with trailers | A class with a hitch. Along the path the trailer is just more length. Its swing at corners is checked by the builder when it decides which connectors the class may use. |
| Bikes | A class: narrow, slow, its own lanes where they exist. |
| People on crosswalks | Announcers on the crosswalk's crossing points. At lights they cross in the walk phase. The walkers' model must look for cars in return; that side is the people sim's job. |
| People anywhere else on the road | Path obstacles. |
| Trains at level crossings and trams in the street | Announcers, from the rail sim, on crossing points where track meets lane, with right of way set in the control table. |
| The player's car | A path obstacle always; an announcer when it's on a path and heading for a junction (guessed from its speed). The player obeys nothing, so drivers treat the guess with a wider margin from the rule set. |
| Wrecks and stopped physical cars | Path obstacles until cleared. For now a wreck stays for a time set in the rule set, then is removed: the people in it are put back at home and take the rest of the day off, and their cars are put back near their homes. Each clearance is counted in the measurements, separately from everything else. Later a tow truck, with a driver and a reason, does it for real. This is the one place a car is removed from the road, and it is never used to clear a queue. |
| Police, ambulances, fire engines (**noted, not built**) | None are on the road yet. When they are: a vehicle with its siren on is an announcer of a kind that goes first everywhere, including through red lights; other drivers get one new fact ("siren behind me" or "siren coming to this junction") and rules to stop short of the junction or pull to the kerb side. Pulling over needs room at the kerb, which a lane-bound car only has where there is a parking lane or shoulder; elsewhere they stop where they are. Nothing in the design has to change to add this. |

## 12. Levels of detail

Same people, same trips, same rule set at every level. What changes:

| | Near | Middle | Out of sight |
|---|---|---|---|
| State | full | full | route and progress only |
| Rules | all | all | none: no car exists |
| Facts | all | the cheap set (no side lanes unless a "must" change is pending; no horn facts) | — |
| Attention slips | yes | no | — |
| Optional lane changes | yes | no | — |
| Decide / move rate | 10 / every tick | 2 / 10 per second | — |

Which facts count as cheap, and the rates, are a `detail` table in the rule set, so the levels are data too.

**Moving between levels:**

- *Near ↔ middle:* nothing about the car changes except how often it's looked at. A lane change under way finishes.
- *Middle → out of sight:* allowed anywhere. The car's progress along its route becomes the person's trip progress.
- *Out of sight → middle (arriving in an area):* the person's progress says roughly where the car should be. It is
  placed on its route at the nearest point that is on a lane (never a connector), with a safe gap ahead and behind,
  at the speed of the cars around it. If there's no such place, the person stays out of sight a little longer and
  tries again further along. People are never lost and never doubled. Queues at red lights are not placed; they
  form: people are placed in a ring outside what can be seen, so they drive for some seconds by the rules before
  anyone sees them. (The plan says "run the area for a few seconds before it's shown"; an area can't be run ahead
  on its own, because the lights follow the one clock. `driving-briefs/E7`.)
- *Near ↔ physical:* handed over at matching position and speed. Going back from physical happens only when the body
  is close to its path and pointing along it; a car knocked sideways stays physical until it has driven back, or
  becomes a wreck.

**Travel times for out of sight** are kept per section and time of day, starting from free-flow and updated from what
simulated drivers take.

## 13. Seeing why, and measuring

- **Every answer carries its reason:** the rule's name, the facts that made it true with their values, and what it is
  about (that car, that light). This is built as part of deciding. It costs a few numbers per driver.
- **The driver card** shows the binding answer in words made from the rule's name and facts, and the last few
  decisions (a short ring per driver, written only when the binding rule *changes*). Who is driving and why comes
  from the host.
- **`driver? <id>`** over the control channel returns the same as text.
- **The wait-for graph** (section 9.6) comes from the same reasons.
- **The measurements table** in the plan is one module, `measure`, run by the lab tests and available live. Each row
  is a count or a distribution with the cars that broke it listed, so a failure points at a car and a moment.
  *As built* (#283, ADR-0408): the row `bodies_touching` tests every pair of car bodies, as the host draws them, every
  tick, knowing nothing of paths, and sorts each spell afterwards by how the two paths relate. The lab's five-minute test
  and the thirty-minute soak (`level_traffic_lab_v2_soak`, `RT_LAB_SOAK=1`) require it at none and the whole table to pass.
- **Replay.** Because a run is the same every time, a failing lab run is reproduced by its seed and tick number.

## 14. Drawbacks

1. **Real gridlock stays frozen** (section 9.6). The player can cause it on purpose by parking across a road. We
   measure it and remove causes; we don't break it.
2. **No improvising.** A lane-bound car can't go round an obstacle unless a lane change does it. A wreck on a
   one-lane road blocks that road until something removes the wreck (a tow truck with a driver and a reason, later).
3. **The lane graph must be right.** Much more is asked of the road builder than today: connectors for every
   movement in every city, drivable for every class, with crossing points. Awkward junctions (skewed, five-way, ramp
   meeting street) will be refused at first, and each refusal is a movement drivers can't make, which changes
   routes. Every city's roads are rebuilt, which takes hours.
4. **Rules as data are harder to debug than code.** There is no line to step through. The trace, the load-time
   checks and rule coverage are not optional extras; without them a wrong table is a mystery.
5. **The rule language will be pushed on.** Each "just this once" condition must be turned into a fact or refused.
   This takes discipline in review for as long as the model lives.
6. **Robots drive too well.** Until personality (phase 7), every driver keeps the same gap and never errs. Traffic
   will look orderly and a little dead. That is expected, not a fault of the phase.
7. **Emergent crashes need an aftermath.** Once drivers can crash, crashed cars need to become something: obstacles,
   then cleared by someone. Until that exists, crashes with personalities on will pile up.
8. **Two models side by side until phase 9.** The old model stays in `city_sim.cpp` behind the per-level switch.
   Drawing, the physics tier, buses and cabs all reach into the old state today, so each needs a seam before the new
   model can drive them. That seam work is real and comes before any visible result.
9. **Calibration.** American figures, mirrored, in mph, in a metric engine. Units are handled at load, and near/far
   turns remove the mirroring from the rules, but each number still has to be found, sourced and credited.
10. **Seams between levels of detail.** A queue that goes out of sight and comes back is rebuilt, not remembered.
    Travel times learned near the player colour out-of-sight trips, so two play-throughs differ slightly.
11. **Cost is unproven.** Facts across several junctions for thousands of drivers is the heavy part. The staggered
    rates are the plan for it; the number is not known until the core runs in the lab.

## 15. Edge cases

Each has a handling or is marked **open** (needs a decision or later work).

**Road shape**

| Case | Handling |
|---|---|
| Paths shorter than a car | Look-ahead in metres along the route; tails recorded on every path they cover (7.3). |
| Junctions closer together than a car's length | Room beyond is measured through the next junction (9.5). |
| A bridge over a street | Crossing points are found in 3D; different levels don't cross (5.2). |
| Dead ends | Need a turnaround path from the builder, or the router never sends a car in. |
| A movement the builder can't make drivable | Refused and reported; not offered (5.3). Routes go round. |
| A long vehicle's swing at a tight corner | Checked per class when deciding which connectors it may use. A truck that needs two lanes to turn is **open**: first version bars it from that movement. |
| Steep hills | Speeding-up and braking adjusted by slope from class data (8). |

**Junctions**

| Case | Handling |
|---|---|
| Four cars reach an all-way stop together | The `settle` list ends in a tie-break that can't tie (9.3). |
| A far turn never gets a gap in heavy traffic | Waits inside the junction; clears on yellow and all-red as first in. Patience shrinks the gap to a floor. A long wait here is "giving way", not a deadlock. |
| The light changes with a car in the box | It is committed and finishes; later cars follow it over the crossing points (9.4). All-red gives it time. |
| Yellow, too close to stop | `can_stop_comfortably` is false, so no stop answer; it goes through. The yellow formula makes this safe. |
| Room beyond vanishes after entering (someone changed into it) | Room was claimed, and a lane change into a claimed stretch isn't granted (9.5). |
| A car stopped on a crosswalk when people get the walk | Prevented by treating the crosswalk like the box: don't enter without room beyond it. |
| Turn on red | A control-table entry: near turn, after a full stop, giving way to everything. A `no_turn_on_red` device forbids it. |

**Lane changes and routes**

| Case | Handling |
|---|---|
| Two cars want the same gap from either side | Claims settled in fixed order; one waits (7.1). |
| Can't get across before the turn | Carry on and re-route (10). |
| A ramp joining with no gap | The lane's end is a place to stop by; drivers on the main road let it in by rule. |
| Destination on the far side of the road | The route ends in a lane on the correct side; the router knows the side. |
| No free parking space at the destination | The driver looks for the next nearest, driving there. That is real traffic. If the whole area is full it keeps looking, within a limit from the rule set, then reports "can't park" to the host. What the person does then is **open** (go home? a car park?). The planner should guarantee enough spaces. |
| Angled bays need reversing | **Open.** First version: bays are entered forward and left forward where the shape allows, and other bays aren't used. Later: paths flagged "driven backwards", same formulas. |
| Route becomes impossible mid-trip (a blockage) | Re-route from where the car is. If nothing leads on, it waits behind the blockage, and the card says so. |

**Others on the road**

| Case | Handling |
|---|---|
| The player runs a red light | A guessed announcement with a wide margin; drivers who notice brake. Those who don't may hit the player: principle 8. |
| The player parks across a lane | Path obstacle. Cars change lanes round it or queue. Can cause gridlock (drawback 1). |
| A person steps out mid-block | Path obstacle; the hard speed limit can't promise a stop for something that appears inside stopping distance. A possible collision, by design. |
| A bus stops in a lane with no bay | A hold. Cars behind change lanes if they can, else queue. |
| A wreck | Path obstacle until cleared. Clearing is **open** (drawback 7). |
| A train at a level crossing | An announcer with right of way; the "keep the box clear" rule applies to the tracks. |

**Levels of detail and time**

| Case | Handling |
|---|---|
| Nowhere to place a car arriving from out of sight | It stays out of sight and tries further on (12). |
| A physical car knocked off its path | Stays physical; becomes kinematic again only when back on its path, else a wreck. |
| The player flies fast across the island | Many areas wake at once. Placement is spread over several ticks; the warm-up is skipped if the area is already in view, accepting less tidy queues. |
| The city clock runs faster than real time | Driving always runs in real simulated seconds. Schedules convert at the boundary. Learned travel times are stored in driving seconds. |
| Pausing, saving, loading | State is plain data and saves as is. Announcements and claims are rebuilt on the first tick. |
| A rule set reloaded mid-run | Tables swapped between ticks; unknown modes fall back to `driving` (6.4). |
| A driver's attention slips | The slipped fact is recorded as "not noticed" in the trace, so a crash can name the lapse. |

## 16. Where this sharpens or departs from the plan

For Glenn to accept or overrule; the tasks assume these.

1. **Fewer stored states.** The plan's thirteen driving states become what the card *shows* (the binding rule), not
   states a car is *in*. Only lane change, pulling out and parking are stored modes (6.3). Reason: a stored state
   that outlives its cause is the old model's kind of bug.
2. **Most rules are tables, few are if-then** (6.1, 6.2). The plan describes rules as "conditions on facts". They
   are, but the bulk of what differs between countries sits in the device, right-of-way and number tables.
3. **Near turn / far turn, kerb side / offside** in all rules and code, never left and right. The existing
   `Move::Left` (which means the driver's right) is not carried over.
4. **Road data splits in three** (shape, control plan, meaning), with regulations stored by abstract kind (5.1).
5. **Kerbside parking is a lane, pulling out is a lane change** (5.2).
6. **Missed turns re-route; nobody stops in a through lane to wait for a gap** (10).
7. **Robots keep their own gap and speed habits.** The plan says distant drivers use default settings. Since a
   person's settings are worked out from their identity at no cost, the design keeps them at every level and drops
   only attention slips and optional lane changes with distance. Otherwise a car changes how it drives the moment it
   crosses the 150 m line, which shows. *This one contradicts a sentence in the plan and needs a yes or no.*
8. **Gridlock and deadlock are measured as two different things** (9.6). The plan's "deadlocks: none" stays; a
   separate "gridlock" row is added, with a target to be set once the lab shows what's normal.
9. **A `driving` library of its own**, between the engine and the city sim (3).

## 17. Tasks

Grouped by the plan's phases and issues. Each task is meant to be one reviewable change with its own tests. "Lab"
means `traffic_lab` with `citysim.driving = "v2"`. Tasks inside a group are in order; a task names what it needs
from other groups.

Standing rules for every task (from `AGENTS.md`; the implementer should read it first): no kind names in C++; numbers
live in data or one named constant; no `auto` outside lambdas; one header and one `.cpp` per module; faces through
`MeshBuilder::emitQuad`; a `CREDITS.md` line for every outside source; an ADR for each decision that's hard to
reverse; the old driving code is not patched, only measured.

### 17.1 Who can take what, and what can run side by side

Every hard task has a brief in `docs/driving-briefs/`; its `README.md` says how tasks are handed out and reviewed.

Three grades of task. "Hard" means the task needs design judgement that this document doesn't fully settle, or works
inside the 8,500-line `city_sim.cpp`; those should get a short written brief first and a careful review after.
"Plain" means the task is fully specified here and checked by its own tests. "Simple" means mostly data, tables or
pure functions, suitable for a smaller model.

| Grade | Tasks |
|---|---|
| Hard | A2 (the seam in the city sim), B2 (connectors at awkward junctions), B3 (crossing points in 3D), B10 (planner), C6 (the tick and its sameness every run), C12 (threads), D1, D3, D4, D5 (junction logic), E1, E3a, E7, E8, E9 |
| Plain | A1, A3, B1, B4, B5, B6, B7, B7a, B8, B9, C2, C4, C5, C7, C8, C9, C11, D2, D6, E2–E6, F2–F4 |
| Simple | C1 (vehicle data block), C3 (the rule-set reader and its error messages), C10 (the measurements as code); the catalogue rows for devices, markings and lane kinds; finding and sourcing each rule-set number, with its `CREDITS.md` line; unit tests for the formulas in section 8; the list of the 85 old tests and what becomes of each |

Five lines of work that don't need each other until they meet:

| Line | Tasks, in order | Can start today |
|---|---|---|
| 1. Paint | B7a → B8 | yes: B7a works on today's roads |
| 2. Road data | A3 → B1 → B2 → B3 → B4 → B5, B6 → B7 | yes |
| 3. The core, on small hand-made roads written in tests | A1 → C2 → C4 → C6 → C7 | yes: needs only A3's types |
| 4. Rules as data | C3 → C5; the catalogue rows and sourced numbers alongside | yes |
| 5. The seam | A2 → C8 | yes: the first half of A2 is a list, changing nothing |

They meet at C11 (the first rule set driving real lab roads) and then D. Lines 3 and 4 never need a real city, so
they aren't held up by road rebuilds. All of the B tasks that change the road mesher's output should land as one
rebuild, not one each.

One limit from how this repository is worked: there is one working branch, and no new branches without Glenn's say.
Tasks on different lines touch different files, so they can be done one after another on that branch in any order,
but running several implementers at the same moment needs either his say for separate work folders or care to keep
them in separate files.

**A. Foundations** (new; do first)

| # | Task | Done when |
|---|---|---|
| A1 | Create the `driving` library and its headless test program. Empty front door (section 3.1): load, tick, begin a trip, steer it, events out. Wire the per-level switch so a v2 level runs it and draws nothing yet. | Builds; links only `engine_core`; a test ticks an empty world. |
| A2 | The seam in `citysim` (section 3.1): list every place that reads the old car state (drawing, physics tier, buses, cabs, audio, map, possession) and put each behind one "where is this car and what is it doing" interface the old model implements. No behaviour change. | The lab baseline numbers are unchanged; the list is in the ADR. |
| A3 | Lane graph types in `procgen/city/roads/lane_graph`, with save, load and a text dump. | Round-trip test; dump of a hand-made two-road junction reads correctly. |

**B. Road data** (phase 1, #226, and 1b, #233)

| # | Task | Done when |
|---|---|---|
| B1 | Compile the builder's lanes into sections (cut where neighbours change), with lane lines and next-path links. | Lab: every lane in exactly one section; neighbours agree both ways. |
| B2 | Connectors for every allowed movement at every junction in the lab, with tightest radius; refuse and report what can't be drawn. | Every connector drivable by its classes; smooth joins; report lists refusals with places. |
| B3 | Crossing points, in 3D, with the stretch on each path. Crosswalks as crossings of paths. | Hand-checked junctions match; no crossing between levels. |
| B4 | The control plan: the planner writes each junction's control and every regulation into the road graph, from a placement policy in the rule set. | No junction without a control; lab dump shows them. |
| B5 | The device catalogue and the planting of signs and paint from regulations. Sign bodies are real shapes built with the mesh library (the STOP octagon, the give-way triangle, posts and brackets), textured from the road-sign system's cached faces (ADR-0110). | Every regulation in the lab has its sign or paint and the reverse; Glenn can see them. |
| B6 | Parking lanes and spaces as road data; the old "a space every 7 m" stays only in the old model. | Spaces painted where stated; none inside the rule set's no-parking zones. |
| B7 | The builder's checks from 5.3 as a test over the lab. Measure the lane graph's size on the island. | All checks pass in the lab; the size is written in the ADR. |

| B7a | Decorations sit flush (5.5, part 4): `drapeOnMesh` in the mesh library, cutting an outline along the road's triangle edges; paint drawn with a depth bias and no lift; the per-corner fitting removed. Can be done first, on today's paint, before any other task here. | A test: the greatest gap between any paint point and the road under it is under 1 mm on a hill, a cambered curve and a ramp. Glenn looks at the hilliest street in a city level and at `hill_junction`. Paint triangle count before and after recorded. |
| B8 | Lane markings (5.5): the markings catalogue in the rule set; lines built from lane line records; arrows, words and symbols as stamps through the sign system's lettering and cache; `assignLaneMoves` retired in favour of the connectors. | Lab: every painted arrow matches its lane's connectors; a sheet of every marking for Glenn to look at, in a small paint test level. Needs B1, B2, B5. |
| B9 | Kinds of lane as a table (5.4); bus and bike lanes built by the lanes builder from it, with their crossing points and the turning stretch beside a bus lane. | A lab street with a bus lane and one with a bike lane, painted and correct in the lane graph dump. Needs B1–B3, B8. |
| B10 | The planner places bus lanes along bus lines and a connected bike network, by an optional setting in the city's brief. Bus lines move from the city sim into the planner. | A planned city with both on, shown on its map; with the setting off, the roads are unchanged. Needs B9; a decision from Glenn first (open question 9). |

**C. The core** (phase 2, #227)

| # | Task | Done when |
|---|---|---|
| C1 | Vehicle classes for driving: a `driving` block in the vehicle data, derived from what's there where possible. | Loaded into flat structs; no size written twice. |
| C2 | Car state and per-path lists, including tails and double listing (7.3). Position and direction read from the path. | Tests: a long vehicle over three short paths is seen by a follower on each. |
| C3 | The rule-set reader and its checks (6.4): devices, numbers, units, personality ranges. | A bad name fails with file and line; mph converts. |
| C4 | Facts registry and the first facts: car ahead along the route, limit, corner ahead, lane end. | Each fact unit-tested on hand-made roads. |
| C5 | Rules and answers: conditions compiled to flat steps; the four answer kinds combined; the binding answer and its reason. | Tests on three-line rule sets; rule coverage counted. |
| C6 | Speed and following (8), with the three-step tick (7.1) on one thread first. | "Following can't crash" at every decide rate; same seed, same result. |
| C7 | The router over sections and connectors, with lanes-that-lead-on. | A trip never starts facing the wrong way; no route is reported, not invented. |
| C8 | Drawing from the state through the A2 seam; indicators and brake lights from answers. | Lab: cars follow lanes and connectors with no pivots (the spin and swing rows read zero). |
| C9 | Trace, `driver?` over the control channel, and the wait-for graph with loop finding. | A staged deadlock is found and named; a car at rest always has a reason. |
| C10 | `measure`: the plan's table as code, run against the lab, with the old model's baseline beside it. | The table prints for both models. |
| C11 | First rule set file, with every junction as an all-way stop. Each rule sourced and each mirrored rule tested. | Lab runs with zero deadlocks, overlaps, spins. |
| C12 | Threads for the plan and move steps. | Same result as one thread, bit for bit; cost per 1,000 drivers recorded. |

**D. Junctions** (phase 3, #228; milestone 1)

| # | Task | Done when |
|---|---|---|
| D1 | The junction plan at load: right of way per pair of movements from the control table. | Table dumps for the lab's junction kinds match hand-worked ones. |
| D2 | Signal programmes by movement, timing from the rule set, state a function of the clock. Lamps read the same data. | Yellow lengths match the formula; no two crossing movements green together unless allowed. |
| D3 | Announcements, `must_give_way`, patience. | Unit tests per control kind; gaps accepted stay within the rule set's range. |
| D4 | First-in-goes-first inside the junction; waiting inside for a far turn. | No stops inside a junction without a recorded reason. |
| D5 | Room beyond as a claim, through close junctions; crosswalk kept clear. | No car at rest in a box except by a recorded give-way. |
| D6 | Milestone run on the downtown grid with the full table, and the gridlock/deadlock split reported. | The plan's phase 3 list holds; Glenn watches. |

**E. Later phases** (outlined; to be detailed when D lands, because their shape depends on what D teaches)

| # | Task | Plan phase |
|---|---|---|
| E1 | Lane changes: intentions, gap test, claim, both-lane listing, abort, letting in, missed turn. | 4, #229 |
| E2 | Freeway: ramps as lanes that end, merging; stop and give-way junctions; roundabout as a control entry. | 5, #230 |
| E3 | Parking and pulling out as lane changes; the search for a space. | 6, #231 |
| E3a | Reversing: paths that are driven backwards (out of an angled bay, out of a driveway), with the same speed and following formulas and reversing lamps. Three-point turns after that. | 6, #231 |
| E4 | Holds (buses, cabs, vans); trailers; bikes. | 6, #231 |
| E5 | Announcers for people on crosswalks and trains; path obstacles; the player. | 6, #231 |
| E6 | Personality from identity; attention slips; the horn; crash reports that name the lapse. | 7, #222 |
| E7 | Middle distance from the `detail` table; out of sight and learned travel times; placing arrivals; warm-up. | 8, #224 |
| E8 | Physical cars steering along the path; hand-over both ways. | 8, #224 |
| E9 | Cities switch one at a time; old model and its tests deleted. | 9, #232 |

**F. Getting around** (outside the driving library; needs its own plan before any of it is built. Related: #223, #213)

| # | Task | Needs |
|---|---|---|
| F1 | Write the getting-around plan: journeys as legs, who owns which vehicle, ways of travelling as a table, how a person chooses (section 3.2). | Glenn's answers on ownership and mix |
| F2 | The three questions `driving` answers for that layer: drive time, parking near a place, where stops and loading places are. | C7, E7 |
| F3 | Stops, cab ranks and loading places as road data, planted like parking spaces. | B6 |
| F4 | Bike and moped vehicle classes; bike lanes as lanes with a class tag. | C1, B1 |

## 18. Questions, answered (Glenn, 2026-11-11) and still open

**Answered.** The sections above still read as written; where an answer changes one, this list wins until the
section is revised.

1. *Do distant drivers keep their own habits?* Yes, if it's possible. "Habits" here means personality: the person's
   own driving settings. It is possible, since they are worked out from the person's id.
2. *Nowhere to park.* More parking gets placed in the city (5.4). The fallback when a district is still full
   stays: the nearest car park, then give up the trip.
3. *Wrecks.* For now a wreck stays for a set time and then disappears; its people are put back at home and take
   the day off; their cars are put back near their homes. A tow truck comes later. (Section 11.)
4. *Reversing.* In scope: task E3a.
6. *Where the code lives.* With the procedural city, not in the engine: the engine may be used for games with no
   driving. So `driving` is its own library beside the city sim (`src/apps/driving/`), and the lane graph types sit
   with the road builder that fills them. Nothing about driving goes into the general engine folders.
7. *Parked cars of people out of sight.* A parked car is near where its owner is. When an area wakes, each person
   there who owns a car has it placed in the nearest free space to them, in a fixed order so the answer is the same
   each time. Nothing tracks cars leaving and arriving while nobody is looking.
8. *Who owns a car, and the mix of ways to travel.* All tunable in Lua. Starting point: at least 60% of people own
   a car; the rest of the mix is to be tuned.
9. *Room for bus and bike lanes.* Roads are widened for kerbside parking and bike lanes; bus lanes take an
   existing lane and are painted red, for buses and taxis (5.4). Moving the bus lines into the planner follows
   from that and is assumed.
10. *Markings.* Wear and night shine are in the first version. Signs get real shapes from the mesh library,
    textured from the sign faces (task B5). The US standard is assumed throughout, as nothing was said against it.
11. *Do experiences change people?* Later, when agents' states are evolved and saved. For now the goal is realism.
12. *Saving what a city learns.* Not at run time. Instead, an offline process that tunes the data: a script, or an
    AI, runs the lab, reads the measurements and changes the Lua tables. The design already allows this, because
    every number is in Lua and `measure` gives its results as data; the measurements should also be written out
    as a file a script can read.

5. *The gridlock target.* None in the lab over a ten-minute run at the lab's normal number of drivers, as a test.
   In the real cities, no target yet: count it, report it, and set one once a normal day has been seen.

Nothing is open. Two places where the briefs in `docs/driving-briefs/` tighten this document are folded into
sections 9.4 and 12 above.
