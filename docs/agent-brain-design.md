# The agent brain: how a resident decides, holds what it is doing, and moves between states

Status: design, agreed in outline with Glenn on 2026-11-16. The brain's core is built as a library with tests
(ADR-0440, `src/apps/citysim/brain/`): section 14 says what is built and where the build had to differ from this
page. The first thing to be seen on it, one resident making one trip in the traffic lab, is built too
(ADR-0441, and its faults put right in ADR-0442; section 9 says what was built and where it differs). It is not
wired into the crowd.

## Words used here

- **Agent / resident**: one simulated person. The island has about 238,000.
- **Intent**: what the person wants right now, such as "be at work" or "have lunch". It says nothing about
  how they get there.
- **Plan**: how they will do it. A trip is a short list of legs.
- **Leg**: one part of a trip made one way: walk to the car, drive, walk to the door.
- **Way of travelling** (mode): walk, car, bus, train, cab, skateboard, bike. One data row each.
- **Doing state**: the one thing the person is doing this moment: walking, waiting, riding, driving, sitting.
- **Skill**: a system that moves a body: walking, driving, riding. A state asks a skill to do something and
  waits for its report.
- **Event**: a report delivered to an agent: "arrived", "parked", "the bus left", "your car is gone".
- **Detail level**: how closely an agent is simulated, set by distance from the player: near, middle, far.
- **Possession**: the player (or an outside decider) taking control of an agent.

## 1. Why

Today there is no single answer to "what is this person doing". It is spread over about ten separate
encodings: a day-table state, a walker/driver mode, a "moving" flag that means seven different things, a seat
phase, five yes/no flags for possession and physics, and four side tables for bus, cab, train and group
activities. Wanting and doing are mixed: the day table's "to work" means both "wants to be at work" and "is
travelling". There is no stored plan, so a bus journey works by the person going idle after each part and
planning again from scratch. One 400-line arrival function handles walkers, buses, cabs, parking, seats and
venues. There is no idea of an interruption: nothing tells a person their car was taken or their bus was
missed. About a dozen repair mechanisms (tethers, snap-backs, watchdogs) exist because the state can become
inconsistent. A far-away person can be in only seven situations, so lunch, errands and bus use are lost when
they leave view.

The new driving needs a real driver: a person with a reason to travel, who chose to drive, and who can be
told "parked", "no route" or "crashed". Wiring it to the brain described above would rebuild the same seam
later. So the brain is designed first, and grows state by state with a working case each time.

## 2. The four layers

Each layer has one job and talks only to the layer beside it.

| Layer | Question it answers | Holds | Exists today as |
|---|---|---|---|
| Intent | What do I want? | one intent (a place or an activity) and when it began | roles, day parts, activity menus in Lua; the goal table |
| Plan | How will I do it? | a short list of legs and which one I am on | the trip planner, used only by the one-agent lab |
| Doing | What am I doing this moment? | one state and its small data | nothing explicit |
| Skills | How does a body move? | the body's position and motion | walking, the old driving, the new driving, transit, rail |

Control flows down as requests and up as events:

- Intent changes (the clock, a finished activity, an outside decider) and asks for a plan.
- The plan gives the doing layer its next leg.
- The doing state asks a skill to carry it out.
- The skill reports an event. The state ends, the plan moves to the next leg, and when the last leg ends
  the intent is satisfied and the person starts doing the thing they came for.

## 3. Intent

The day table says only "be at X" or "do activity Y". Travel states come out of it. That removes the root
of the mixing: a person at lunch and a person on the way to lunch have the same intent and different plans.

Choosing stays as it is: roles and day parts in Lua, activity menus by hour, first matching row wins. A
smarter decider (the local model, or the player's choices) plugs in here, at the moment an intent ends: the
engine builds the menu of what this person could do next, the decider picks, and the table is the fallback.
That is the earlier decision ADR-0091, now with a defined place.

## 4. Plan

Every agent uses the trip planner. A person going somewhere prices each way of travelling as a whole trip,
door to door, and takes the cheapest in weighted minutes. A way that needs a vehicle they own starts with
getting to it and ends with getting from where they leave it to the door. So a car trip is three legs: walk
to the car, drive to a parking space, walk to the door. The teleport from the parked car to the door goes.

Ways of travelling are data rows (`assets/scripts/travel.lua`). Today: walk and car. To add: bus, train,
cab, skateboard, bike. Each row says its network (footpaths, roads, a transit line), whether it needs an
owned vehicle, its fixed minutes at each end, and how much the person minds a minute of it. Adding a way is
a row, plus a skill if it moves a body in a new manner. The hard-coded rules for who takes the bus go.

What an agent stores for a plan, sized for hundreds of thousands of people:

- a small fixed number of legs (up to six), each: the way, where it ends, and the vehicle or line it uses;
- which leg it is on;
- no route. A route is worked out when its leg begins and belongs to the skill running it.

A plan is replaced whole when it stops being possible (section 7).

## 5. Doing

One state id per agent, with a small fixed block of data that only that state reads. Each state has a start,
an end, a step, and a list of the events it understands and the states they lead to. The machine is C++;
the list of states and their transitions is a data table, so adding a state is a data edit plus its step.
Two small machines already in the code have this shape and are the pattern: the train journey's phases and
the player's own body modes (walk, climb, swim).

The states, grouped:

| Group | States |
|---|---|
| Travelling | walking, riding a board or bike, driving, pulling out, parking |
| Using transit | waiting at a stop, boarding, riding, alighting; hailing a cab |
| At a place | indoors, standing outside, using a spot (sit, lie, lean), in a group activity, working a post |
| Resting | asleep |
| Body trouble | knocked down, stuck, in a crash |
| Held | possessed (the player or a director decides); released |

Rules that remove today's ambiguity:

- "Not moving" is never a state. Waiting, riding, sitting and being indoors are different states.
- A person is not "a driver" or "a walker" for life. They own things (a car, a board) and choose per trip.
- The car is not part of the person. Its position on the road belongs to the driving. The person holds a
  handle to it while driving and a note of where it is parked otherwise.
- Bus drivers and cab drivers are people whose intent is a shift. Their legs are a route's stops or a fare.
- Possession is a state at the top that suspends intent. It replaces the five flags.
- A knocked-down or stuck body is agent state, fed by events from the physics body. That is what lets the
  tethers and snap-backs go, and it is where crashes land.

## 6. Skills and events

Every skill takes a request and reports by events, read once a tick in a fixed order so runs repeat exactly.
The new driving already works this way: "this person drives this car to that place", then "parked",
"no route", "cannot park", "missed turn", "crashed", "ended early". Walking, riding and the trains are
brought to the same shape, which replaces the single large arrival function with each state's own handling.

## 7. Interruptions

An interruption is an event delivered to the current state. Each state says what it does with it; the usual
answer is "end this leg and plan again from where I stand, ruling out what failed". The planner already
supports ruling a way out.

| Event | Example | Usual result |
|---|---|---|
| vehicle gone | the player took the car | plan again without it |
| missed ride | the last bus left | plan again without that line |
| blocked | no route, cannot park | plan again; another space or another way |
| crashed | hit while driving | crash state, then on foot or wait |
| knocked down | hit while walking | body state, then carry on |
| possessed / released | the player takes or leaves the person | held state; on release, intent resumes |

## 8. Detail by distance, and the cost

The same intent, plan and state exist at every distance. What changes is how often they advance and whether
a skill is running.

| Level | Who | What runs | How often |
|---|---|---|---|
| Near | around the player | full skills; a physics body where close enough; visible crashes | every tick |
| Middle | out of sight but close in time | the state advances along its leg at the leg's planned pace; meetings between vehicles settled by a dice roll | about once a second, in turns |
| Far | everyone else (over 90%) | no skill. The state is "leg N, due to end at time T". A clock queue wakes the agent only when a leg or an activity ends | only at those moments |

Consequences:

- **Nothing is lost by leaving view.** A far person at lunch is at lunch; a far person on the bus is "riding,
  due at their stop at 12:41". Today's limit of seven far situations goes, because the state and the time
  it was entered are kept through every level.
- **Arriving anywhere works the same way.** Walk, drive or teleport to a place: the people there are raised
  to near. Each is placed along their current leg by the clock (part-way down a street, on a bus, at a
  desk), their skill is started mid-leg, and they carry on with their intent.
- **Planning at scale.** A far agent plans when its intent changes, a few times a day. Pricing uses the
  route table's times and makes no route. With 238,000 people that is a few thousand plans a minute, spread
  by the clock queue's budget. Near agents make real routes, one leg at a time.
- **Rules for speed** (already the project's): no allocation per agent on a transition, no Lua per agent per
  tick, no per-agent virtual objects, far agents touched only through the clock queue, fixed order.

### Where the GPU stands

A compute shader (`crowd.comp`) already moves and draws the far travellers: each has a 32-byte record (route
and clock), and the GPU works out where along its route each person is and culls and draws them. So far
MOTION and DRAWING are on the GPU today. DECISIONS (what to do next) are on the CPU, driven by the clock
queue.

This design keeps that split and feeds it better: a far agent's "leg, start time, end time" is exactly the
record the shader wants, for every way of travelling and not only walking. Moving decisions themselves to
the GPU is not proposed now. They are rare per agent (a few a day), they branch heavily and they need the
planner; the clock queue makes them cheap. If measurements later show the queue is the cost, the first
candidate for compute is pricing trips in bulk.

## 9. The first build: one agent, one trip

In the traffic lab, one resident with the new brain:

1. has an intent ("be at that building");
2. plans, choosing among the ways they have (walk, car; then bus, skateboard as rows are added);
3. walks to their parked car, gets in, pulls out;
4. drives the route on the new driving, obeying lanes, turns, lights and stop lines;
5. parks in a space near the destination, gets out, walks to the door;
6. is there, and the intent is satisfied.

Watched end to end, with a test, and inspectable: selecting the person shows intent, plan, current leg,
current state and what it is waiting for.

Road-model work this stands on (agreed 2026-11-16):

- parking spaces stated in the road model (done, ADR-0439: `RoadModel::kerbSpacesOf`, `kerbSpacePose`,
  `kerbLaneBeside`);
- a question the model answers: from a building's door to a place on the road (done, ADR-0439:
  `RoadModel::frontageAt`, then `sidewalkBeside`, `kerbLaneBeside`, `kerbSpacesNear`; a search by position
  until the lot pass records each door's street);
- for the first build a car stops in the kerb lane beside its space and slides in; real pull-out and
  parking come with lane changes;
- the driving's decision rate in seconds, not ticks, so levels at different tick rates behave the same
  (done, ADR-0439: `DrivingSettings::decideInterval`);
- the physics body follows the simulated car for now; the body becomes the truth with visible crashes;
- the junction code is used as it is (one car never exercises it) and reworked when two agents first meet.

### Built (ADR-0441, then ADR-0442, 2026-11-16)

Where to see it: the level `assets/levels/brain_lab.json` (the traffic lab's roads with one resident and no other
traffic). The glue between the brain library and the city sim is `src/apps/citysim/city_brain.h`; the verbs and the
drawing are `city_brain_system.h`; the test is `tests/test_brain_lab.cpp`. `python3 tools/brain_lab_probe.py` runs
the trip in the viewer and takes the pictures, from behind and, where it matters, from above.

| Step above | Built | How |
|---|---|---|
| 1. an intent | yes | "be at that MARKER". Glenn decided markers, not buildings: a marker is a named point beside a street, stated in the level by street, station and kerb. The traffic lab has eight |
| 2. plans among the ways they have | yes | walk, car, skateboard, scooter, by the trip planner; `brain trait has_board on` changes the next plan |
| 3. walks to the car, gets in, pulls out | yes, pulling out is a slide | walks on the sim's own walking; gets in; when the kerb lane beside the space is clear the car is placed in it and drawn sliding out over 2.5 s |
| 4. drives on the new driving | yes | lanes, turns, lights and stop lines are the driving's own; the car is one more car in it |
| 5. parks, gets out, walks to the door | yes, parking is a slide | stops in the kerb lane beside the space held for it, slides in over 2.5 s, the person gets out on the kerb side and walks to the marker |
| 6. is there | yes | stands at the marker (the intent `be_outside_at`) |
| inspectable | by a verb, not by selecting | `brain?` gives one line; the screen shows what they want, the plan, and what they are doing in the data's words with the marker they stand at |

**Since ADR-0443 (2026-11-16) the lab has six residents in three households**, and sections 7 and 12 are built on
it: see "Built: several residents, a shared car, interruptions" after section 12. The table above is still how one
of them makes one trip; "inspectable" is now also by looking (the resident or household car in the middle of the
view has its line shown).

To watch it:

```
cd /home/glennsong/Projects/raytracer-clang/.claude/worktrees/buildings
cmake --build build-viewer --target viewer
./build-viewer/viewer assets/levels/brain_lab.json --play
```

Console (`<who>` is a resident's name or number; with none, the followed resident): `residents?`, `brain? ana`,
`brain follow` (the next resident), `brain follow ben`, `brain follow off`, `brain day all off`, `goto ana work`,
`goto ben ramp`, `brain trait ana has_board on`, `markers?`, `sim speed 8`. A director: `possess walker <x> <z>`,
`walk_to <x> <z>`, `board`, `alight`, `release`. The player: look at the driver's seat of a car with its driver in
it (Tab moves the focus) and press E.

Where the build differs from this section, or this page did not say:

1. **A marker, not a building's door.** Nothing here reads a lot or a door. When the lot pass records each
   door's street, station and kerb, a door becomes a place the same way a marker is.
2. **The two fixed waits are long to watch.** The car's row of `travel.lua` gives 2 minutes at the start of a
   drive and 3 at its end, and the brain's pulling-out and parking states last exactly those. On screen the
   person gets into the car and sits in it for two minutes before it moves, and sits in it for three after it
   has slid into its space. The numbers are the planner's and were not changed. To be decided: shorter numbers
   for a kerb space, or the minutes spent doing something visible.
3. **A short trip is never driven.** With five fixed minutes, the car wins only beyond about 650 m on foot. The
   traffic lab is 1.1 km across, so `home` and `work` had to be put on opposite sides for the car to be chosen.
4. **A space is held from the moment the plan chooses it** until the driver parks there or drops the plan (my
   choice, for Glenn to confirm; it answers section 14's difference 5 for now).
5. **A taken space is answered with the next nearest free one** (section 7's "another space"): the car drives on
   to it. Only when there is none does the person leave the car and walk (section 14's difference 6 stands for
   "no route").
6. **A new intent at the wheel redirects the drive** (ADR-0442; before, it waited for the whole old trip). The
   same trip of the same car is given a new end (`Driving::changeDestination`), the plan becomes "drive to the
   space nearest the new place, walk", and the space held at the old place is let go for the new one. Told
   while pulling out, they are redirected as the drive begins; told while parking, they finish parking and plan
   again as they step out. In the data this is one row entry (`driving`: `new_intent = "redirect"`), and the
   rule that a new intent a state ignores is delivered again to the trip's next state.
7. **Waiting for a clear lane has a limit** (ADR-0442; before, none). The car's row of `travel.lua` says how long
   (`set_off_wait_minutes`, 1.5, an estimate). After it the person gives the car up (`blocked`), leaves it in
   its space and plans again without it. "Clear" now looks back 30 m along every path that leads into the
   lane, not only along the lane itself.
8. **The resident is the brain's, and the sim's day table does not place them** (ADR-0442): their agent is
   marked `brainOwned`, and the one function every placing pass goes through leaves such a person alone. The
   per-tick repair that put them back is gone.
9. **A new person starts in the state their intent arrives in** (ADR-0442): at a marker that is "standing
   outside", not "indoors". The screen says what they are doing in the data's words and which marker they
   stand at.
10. **Where the car stands at a red light was measured, and is right** (ADR-0442): its front 1 cm before the road
    model's stop line, which is also the end of the lane path the driving stops at and the middle of the
    painted bar. A picture from behind the car made it look 3 to 5 m past the bar; pictures from above show
    it at the bar. A test holds it.

## 10. Order after that

1. One agent, one trip (above): the brain's core, the states for walking, driving, pulling out, parking.
2. More ways for that agent: skateboard and bike (own vehicle, footpaths or roads), then the bus (waiting,
   boarding, riding, alighting), each a row and its states.
3. Interruptions: car taken, bus missed, blocked, possession on top.
4. Two agents meeting on the road: following, giving way, joining from a ramp by zipper. Lane changes. The
   junction code reworked. Visible crashes near the player.
5. Detail levels for brain agents: middle and far, the clock queue, raising people mid-leg.
6. A bus driver on a shift, then cabs.
7. The crowd moves over, intent by intent, while the old paths still run for the rest. River town first.
   The old driving, the big arrival function and the repair mechanisms are deleted as their last user goes.

The 300-car lab stays as a flow check. Its pinned-state test is dropped once agents with schedules drive.

## 11. What this replaces

| Today | Becomes |
|---|---|
| "moving" flag with seven meanings | distinct states |
| walker or driver for life | owns things, chooses per trip |
| travel states in the day table | intent only; travel is the plan |
| hard-coded bus, train, cab rules | rows the planner prices |
| one route, re-planned after each part | a stored plan of legs |
| the large arrival function | each state's own events |
| five possession and physics flags | the held state on top |
| tethers, snap-backs, watchdogs | body-trouble states fed by events |
| seven far situations found by label | every state, kept with its time |
| the car's road position inside the person | owned by the driving; the person holds a handle |

## 12. Decided with Glenn, 2026-11-16

**How many legs.** The number of legs varies with the trip. There is a cap of eight, so a plan can never
grow without end or call itself. Walk, bus, change, bus, walk is five. A plan that would need more than
eight is refused, and the planner takes the next cheapest way.

**A household shares its car. Whoever reaches it first takes it.** Nobody is assigned the car. Each person
plans from where the car stands at that moment and whether it is free. If someone else drives off in it
before they arrive, they get the "vehicle gone" event at the kerb and plan again without it (section 7).
The car stays where its last driver parked it, and the next person plans from there, so a car left at work
is not at home for the person who stayed behind.

**Small wheels: skateboard, scooter.** They are a faster way of doing a leg on foot, not a vehicle to be
parked.

- The prop appears when the leg starts and disappears when it ends. Nothing is carried, stored or left
  anywhere, so it can be used to reach a bus stop and the person simply boards.
- They are data rows like any other way: on the footpaths, quicker than walking, no vehicle to return to.
- Not everyone has one. Whether a person has a board or a scooter is a trait dealt with the population, by
  life stage, as a data table. Without that, every trip would be priced as if everyone could skate and
  walking would nearly vanish.
- Each row says how much the person minds a minute of it and the longest leg it suits, so a board is taken
  for the ten minutes to the bus stop and not for an hour across town.
- A way that needs getting to (a car, a bus stop, a station) is reached by the cheapest foot way the person
  has. So "skate to the stop, take the bus, walk to the door" comes out of the pricing with no special rule.
- A bike is different: it is a real thing that is left where it is parked, like a car. It comes later.

### Built: several residents, a shared car, interruptions (ADR-0443, 2026-11-16)

On `brain_lab`: ana and ben (household `east`, one car at `home`); cat, dan with a skateboard and eve (`trio`, one
car at `west`); fay (`solo`, no car, a scooter, at `shop`). The level states them as two lists, `residents` and
`households`. Tests: `tests/test_brain_lab.cpp` (five new cases) and `tests/test_brain_trip.cpp`.

| From sections 7 and 12 | Built | How |
|---|---|---|
| A household shares its car; whoever reaches it first takes it | yes | a plan prices the car only if it is free then, from where it stands then; at the kerb the pulling-out state asks again and raises `vehicle_gone`; the car stays where it was parked |
| Parking spaces held from the plan, right with several cars | yes | one hold a person; dropped on planning again, a redirect, parking, and the car being taken; a check of the holds runs every tick of the tests |
| The player takes a parked car | yes, through the possession channel's `board` | it is not free and stands nowhere; a later plan does not price it; a person walking to it gets `vehicle_gone` at the kerb |
| The player takes a car while a resident drives it | yes, through the real key | the driver is stood on the kerb side, told `vehicle_gone`, the trip ended and the car off the driving's road; they plan again on foot |
| The player leaves the car somewhere | on the brain's side, and through `alight`; NOT from a car taken with the key | in a kerb space if it is in one; else at a point, put into the kerb lane beside it; in a field, nobody plans with it |
| The player possesses a resident | yes (`possess walker`, `release`) | the `possessed` state; on release they plan again from where they stand, with the intent they had |
| Blocked; no possible plan | yes | `blocked` rules that way out and plans again; with no way left, the state `no_way`: tries again every 60 s (data) and at once when a trait or the household's car changes |

Where the build differs from sections 7 and 12, or they did not say:

1. **"First to reach it" is decided by where each person happens to stand.** In the lab ben, told a second after
   ana, takes the car because his spot at the marker is 0.9 m nearer it. Ana walks four seconds to a car she
   could see being taken.
2. **Nobody is told while walking to the car** that it has gone: only at the kerb, as section 12 says. A person
   ten minutes from the car finds out on arriving. To be decided: tell them at once.
3. **The player's own car cannot be given back.** Taking a driven car with the key makes a physical car of the
   player's and puts the sim's copy away for good; the engine cannot turn it back. So "the player leaves it
   somewhere" works for a car taken through the possession channel, and for a car handed back by a call
   (`CityBrain::carLeftAt`), and not for the player stepping out of a car they took with the key.
4. **A car left at a point stands in a live lane** as far as the driving knows, and is drawn wherever it was
   left. With no lane changes the traffic behind it waits.
5. **`no_way` forgets what failed**: trying again allows every way again, so a person whose walk is blocked sets
   out on the same walk each minute. Section 7 does not say when a ruled-out way is forgiven.
6. **A long walk begun beside the car starts on the far pavement.** The sim's own walking puts a routed walker on
   its path at once, which may be across the street. Seen when ana, finding the car gone, set off on foot.
7. **Two brain cars on the road at once were not seen.** In the tests and the pictures one household car drives
   at a time. The lab's day tables will make two meet; nothing is known yet about how.

## 13. Still open

- Group activities are joined by several people at once; whether the group or each person holds the state
  is decided when that state is moved over.
- Weather and time of day changing how much a way is minded (skating in the rain, walking at night). Later.
- Who holds a parking space, and from when (section 14, difference 5).
- What a driver does when the space is taken or there is no route: today they get out and walk, leaving the
  car where it stopped (section 14, difference 6).

## 14. Built so far, and where the build differs (ADR-0440, 2026-11-16)

**Built**, as a library (`src/apps/citysim/brain/`) that includes nothing from the city sim and is tested
with stand-in skills (`brain_tests`):

| Part of this page | Built | Where |
|---|---|---|
| Section 2, the four layers | each person's record: intent, plan, doing, what they have | `brain_record.h` |
| Section 4, the plan | the existing trip planner, extended (legs cap, traits, longest leg, cheapest way on foot to a vehicle, a car in use) | `trip_plan.h`, `brain_planner.h`, `travel.lua` |
| Section 5, states as data | all 21 states and their transitions | `assets/scripts/brain.lua`, `brain_table.h` |
| Section 5, what a state does | walking, small wheels, pulling out, driving, parking, indoors, standing, possessed, released, knocked down, stuck, crashed. The transit states, using a spot, group activity, working a post and asleep are in the data and refuse entry ("not built yet") | `brain_states.cpp` |
| Section 6, skills and events | the interfaces; one event queue answered once a tick, lowest person first | `brain_skills.h`, `brain.h` |
| Section 7, interruptions | vehicle gone, no route, cannot park, knocked down and got up, possessed and released | `brain.lua` rows |
| Section 8, detail by distance | near and far, the "next time this person needs a visit" a clock queue wants, raising and lowering part-way through a leg. Middle runs as near | `brain.h` |
| Section 9, inspectable | one line per person: intent, plan, leg, state, since when, waiting for what | `Machine::describe` |
| Section 9, the real driving | the drive skill over the driving library, and the road model's kerb spaces as places. Not yet driven end to end | `brain_driving.h` |

**Not built**: the middle level; dealing boards and scooters to the population; a world over a real city's
doors. (Built since, ADR-0441 and section 9: the real walking skill over the city sim's walkers, a world over
the traffic lab's markers, its car and the clock, and the lab resident.)

**Where the build differs from this page, and why.**

1. **Legs: eight, not six.** Section 4 says six; section 12 decided eight. Eight is built.
2. **A person's record is 120 bytes, not the 96 aimed for.** Two things this page did not count are stored.
   Each leg's planned whole seconds (16 bytes), because section 8's far person is "leg N, due at time T"
   and T must be known when the leg begins without finding a route. And where the plan began (8 bytes),
   because the first leg has no leg before it to say where it starts, and a far person raised part-way
   along it must be placed. The state's own data is 4 bytes.
3. **Time is whole milliseconds in 32 bits**, so runs repeat to the bit and "due at" is exact. That holds a
   little over 49 days of simulated time. A longer-running world needs 64 bits (8 more bytes a person) or a
   clock that is re-based.
4. **A car leg is three states, and "parked" is not the end of it.** Pulling out, driving and parking are
   one leg of the plan. The driving library's "parked" report means "stopped at the destination, in the lane
   beside the space"; the brain's parking state (into the space, out of the car) follows it. The names read
   oddly side by side and were kept, because they are the driving's own.
5. **Nobody holds a parking space until the car is beside it.** The planner is told the nearest free space
   at the moment of planning, and the space is taken on entering the parking state. Two cars sent to one
   door at one time are told the same space, and the second finds it taken. In the 1,000-person test 239
   drivers met this. This page did not say who reserves a space or when. To be decided.
6. **"Plan again, ruling out what failed" leaves the car where it stopped.** That is the rule section 7
   gives, and it is what is built for "no route" and "cannot park": the person gets out, the car stands free
   in the lane, and they walk. Section 7's "another space" is not built. It needs deciding with 5.
7. **Released does not carry on; it plans again.** Section 7 says "on release, intent resumes". The player
   may have moved the person anywhere, so the old plan is dropped and a new one made from where they stand.
   Knocked down and stuck do carry on with the same leg, from where it had got to.
8. **A new intent is not acted on in every state.** Walking, indoors and standing act on it at once. A
   person pulling out, driving or parking finishes the leg; on arriving they are not where they now want to
   be, and go on from there. This page did not say. (Changed by ADR-0442: a driver is redirected at the
   wheel; a state that ignores a new intent hands it to the trip's next state; section 9, difference 6.)
9. **A far person part-way along a leg is placed on the straight line between the leg's ends.** No route is
   stored (section 4), so that is all the brain can say; whoever raises them finds the real route.
10. **Near and far agree only as closely as a leg's whole seconds.** A leg's planned time is kept in whole
    seconds; a real skill takes what it takes. In the tests the stand-in skills take exactly the planned
    time, so the times are equal there.
11. **The planner's fixed "way to the car" (`access`) is now optional**, and the shipped car row no longer
    sets it: a vehicle is reached and left by the cheapest way on foot the person has (section 12). A table
    that sets it still gets it.
12. **"No plan" is a state of its own, and it ends** (ADR-0443). Section 5's table has no state for "wants to be
    somewhere and has no way there"; the build had used `standing`, for ever. Now `no_way`, which the clock ends
    after the data's `seconds`, and which `things_changed` ends at once. Two things the machine gained for it, both
    general: a state's own time in the data (`seconds`), and the verb `replan_afresh` (plan again with every
    ruled-out way allowed).
13. **Which car a person plans with is the world's answer, not the record's** (ADR-0443): the record holds one
    car handle and stays 120 bytes; each time they plan, the world names the household car they would take. Two
    cars in one household are answered and not yet tested.
14. **A place can be "this marker, this person's spot"** (ADR-0443): the handle of a marker carries whose spot is
    meant, so two people at one marker stand 0.9 m apart. The brain still only stores and hands back the handle.
