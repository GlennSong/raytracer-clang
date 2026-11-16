# The agent brain: how a resident decides, holds what it is doing, and moves between states

Status: design, agreed in outline with Glenn on 2026-11-16. No code yet. The first thing built on it is one
resident making one trip in the traffic lab (section 9).

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

- parking spaces stated in the road model;
- a question the model answers: from a building's door to a place on the road;
- for the first build a car stops in the kerb lane beside its space and slides in; real pull-out and
  parking come with lane changes;
- the driving's decision rate in seconds, not ticks, so levels at different tick rates behave the same;
- the physics body follows the simulated car for now; the body becomes the truth with visible crashes;
- the junction code is used as it is (one car never exercises it) and reworked when two agents first meet.

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

## 12. Open questions

- How many legs is enough? Six covers walk, bus, change, bus, walk with one to spare.
- Does a household share a car, and who decides who takes it? Not needed for the first build.
- Carrying things (a skateboard on the bus, shopping): where does that live? Later.
- Group activities are joined by several people at once; whether the group or each person holds the state
  is decided when that state is moved over.
