# Drivers among others: what a driver sees and decides, where a car lives, and crashes

Status: DRAFT for Glenn, rewritten 2026-11-16 after his review in conversation. Nothing here is built. It answers
issue #285 and the junction half of #286, and is the first item of `docs/agent-driving-handoff.md`'s "what is next".
It changes `docs/driving-model-design.md` (sections 7 to 12) and adds to `docs/agent-brain-design.md`; section 11 says
where. (The first draft, commit 7a508e43, extended the central arbiter; this replaces it.)

## Words

- **Driver**: a resident while they drive. Driving is a skill the person's brain uses (agent-brain-design.md); the
  brain says "be at X", the skill drives there and reports back.
- **Perceive**: what a driver can find out the way a person could: where things are, how fast they move, which way a
  car's indicator blinks, whether its brake lights are on, what the road and its signs and lights say.
- **Obstacle**: anything that would be in the car's way if it went on. Not a list of kinds (section 5).
- **Following**: keeping a safe distance from whatever is ahead (`src/apps/driving/act.h`: easing toward a speed, plus
  a hard "could I still stop" limit).
- **Crossing point**: a stretch where two paths' cars could touch, such as two turns crossing in a junction (ADR-0400).
- **Right-of-way table**: who must give way to whom at each meeting of two turns, worked out at load from the lights,
  signs and the rule set (`right_of_way.h`). It is the law as a driver knows it, not a referee.
- **Gap**: the space on a lane between two cars, the one that would be ahead of me (the **leader**) and the one that
  would be behind me (the **follower**).
- **Merge**: where the road model says one lane runs into its neighbour and ends: a ramp's lane into the freeway's, a
  lane that drops (`ConnectionKind::Merge`).
- **Zipper**: at a merge, cars from the two lanes take turns, one from each.
- **Entity, component**: the engine's ECS (ADR-0006). An entity is a handle; a component is one kind of data it has,
  stored with all others of its kind in one packed array; a system runs over those arrays.
- **Near / middle / far**: distance from the player. Near: the cars around the player, real physics cars. Middle: cars
  that can be seen, moved along their lanes. Far: no car; the person is "on leg N of the trip, due at time T".
- **Contact**: two things' outlines overlap.

## 1. What Glenn asked for

- "The vehicles shouldn't drive through each other. They should collide and there should be a wreck. Really the car
  that's coming onto the freeway should slow down and properly zipper."
- "If I'm close to those vehicles I should see them crash. They should have physics bodies. If it's distant ... it
  could be fudged with a dice roll ... Maybe very distant cars don't crash at all because they're not present yet."
- The brain is about the agent, "not trying to massage a system of many actors and the potential outcomes, like that
  should be emergent."
- "Drivers should see obstacles and register that. I don't want to define obstacles because it could be anything that
  impedes their way." When the player stepped in front of a resident's car, it should have stopped or swerved, and a
  horn would be good.
- Cars near the player are always physics cars: "then any car could be stolen and there's no switch over between a
  non-jolt car and a jolt one ... it also helps unify the car system." About 100 of them, coming and going as the
  player moves.
- A car near the player still follows its route, but can swerve round things or take a different way.
- "We should only have one representation of a car." The ghost car (a physics car chasing the sim's idea of where it
  should be) was not good. Use the ECS rather than working around it.

## 2. The principles

1. **Each driver decides for itself, from what it perceives and what it knows.** Nothing hands out turns. The rules
   of the road are the driver's knowledge (the right-of-way table, the rule set), applied by that driver to what it
   sees. How many drivers interact, and what comes of it, is emergent.
2. **Only the world is central**: the lights (one clock), physics, and finding contact. The world says what is;
   drivers decide what to do about it.
3. **One car, one representation, one owner of its motion at any moment.** Never a ghost.
4. **Detail by distance drops richness, never rules.** The same driver decides the same way near and middle; only how
   the decision is carried out, and how much the driver can see, changes.
5. **Checks are measurements, not control.** The wait-for graph, contact counts and long stands are counted and
   reported. A bad number is fixed by improving drivers' habits or the roads, never by an arbiter.
6. **Runs repeat.** Every driver decides from the same snapshot of the last tick; any chance (hesitation, a missed
   look, a crash roll) comes from a hash of the person, the tick and the question. The lab runs without a player, so
   it has no near tier and stays exact.

## 3. What there is today (checked against the tree)

- **Following** works along one lane and the lanes the route leads into.
- **Junctions are arbitrated centrally.** Every car announces when it will reach each crossing point (`announce.h`);
  once a tick the resolver (`conflict.h`) orders every pair by seven questions and, where a loop of cars could form,
  imposes one order on a whole group (Tarjan's and Kahn's methods); room beyond a junction is granted one car at a time
  (`room.h`). The drivers then obey. This is SUMO's approach, which the 11-11 design was built on.
- **Lane changes do not exist.** The router can plan them; the tick cannot move a car across, so callers switch them off
  (`city_sim.cpp:282`). Pulling out of and into a kerb space is a sideways slide.
- **Drivers perceive no person and nothing that is not a car of the driving.** The 11-11 design's "the player is a path
  obstacle" was never built. A resident's car drove into the player.
- **Merges** are crossing points; `stated_merge` (the joining lane gives way) is switched off in `rules/lab.lua:118`
  because of two faults (#285): a comparison off by 7e-15 m (`announce.cpp:257`, `:298`), and a stretch where, once both
  fronts were on it, neither giving way nor following watched the pair, so a bus and a car overlapped. In the lane
  graph, a merging lane's last path is joined end to end into the host lane (`lane_level.cpp`, near line 570); it is
  not the host's neighbour, so "the lane beside" cannot be seen from it.
- **Cars are not entities.** The city's cars are rows in the sim's arrays, drawn instanced; near the player each drawn
  car has a kinematic box (`city_physics.h`, ADR-0060) that pushes things and is pushed by nothing. The player's car is
  an entity with a `Vehicle` component run by `VehicleSystem` (Jolt, damage ADR-0353 to 0356). Taking a car turns the
  row into an entity, one way only (ADR-0062; #293). An older "every NPC car a Jolt vehicle chasing a planner ghost"
  bridge was retired for needing endless reconciliation (`city_vehicles.h`).
- **Measured today** (temporary tests, not committed): a Jolt car driven by the existing lane controller costs about
  0.013 ms a physics step on one thread, 0.004 ms on the shared pool (100 cars: 0.48 ms; 200: 0.86 ms), physics only.
  The ECS: one pass over 100,000 entities 0.33 ms against 0.11 ms for a plain array; adding and removing a component on
  100 entities 0.05 ms. The gap is fixable (#295).

## 4. Where a car lives

**A car is an entity** whenever it is near or middle, and only then. One entity, its components changing with
distance:

| | Always (near and middle) | Near: about 100 nearest | Middle: seen, further off | Far |
|---|---|---|---|---|
| What it has | position, the model to draw, its household, a link to whoever is driving | a `Vehicle` component: Jolt body, wheels, damage, lamps | a "moves along its lane" component: lane, distance along it, sideways offset, speed | no entity |
| Who moves it | | physics, from the driver's steering and pedals | the driving, along the lane | the person's trip: "leg N, due at T" |

- **Out of sight a car is a record**, not an entity: "parked at this kerb space", or "on leg 3 of ana's trip". It is
  made an entity when it comes into view, and the entity is destroyed when it leaves.
- **Near and middle swap one component.** Same entity, same handle, nothing to reconcile: becoming near, the
  `Vehicle` component is added at the car's position and speed and the lane component removed; leaving near, the
  reverse.
- **Hand-back rules**: a car leaves the near tier only when it is on a lane, upright, pointing along it and not a
  wreck. A car pushed onto the pavement stays near (over the cap if it must) until it has driven back, or is a wreck.
- **Coming and going as the player moves**: the near tier is the nearest cars up to a cap (100, data), entered inside
  one distance and left beyond a larger one so cars do not flicker. A few cars change per frame, nearest first, so
  driving fast into a busy street does not make 40 Jolt cars in one frame.
- **Any car can be stolen**, because any car near the player is already a physics car: taking it changes who sits in
  the driver's seat, nothing else. And the player giving a car back (#293) is the same as any car leaving the near
  tier.
- **Engine work this needs**: removing a `Vehicle` and its Jolt body cleanly (`VehicleSystem` never removes one today;
  #293's engine part); and #295, so the ECS walks its arrays as fast as a plain array.
- **People stay where they are**: the brain's packed records. A car's driver link names a person; the brain library
  still includes nothing from the engine's world.

## 5. What a driver perceives

One question, asked by every driver at every decision: **what is ahead of me, along the way I mean to go, and how is
it moving?** The world answers it; the driver does not ask "is there a person" or "is there a wreck".

- **Near**: the world answers with physics: a shape the width of the car, swept along the path the car means to
  drive for its look-ahead, against every body in the physics world. A person, the player, a crate, a wreck, a fallen
  tree, a thing nobody has thought of yet: if it has a body, it is seen, with where it is and how fast it moves.
- **Middle**: there is no physics out there for most things, so the world answers from what it knows stands on the
  roads: cars (their lane positions), people on the road (walkers crossing, where the sim puts them), wrecks, and any
  thing registered as standing on a lane. This is the one place "anything" narrows; it is a property of distance, and
  the driver's question and decision are the same.
- **Other cars, as a person sees them**: position, speed, heading, the indicator, brake lights, and whether it is
  creeping forward. Not its plans. (Today's announcements estimate "when will that car reach the crossing" from its own
  state; the estimate stays, made from what can be seen, by the driver who wants it.)
- **The road**: lanes ahead and beside (the road model), signs, lights, stop lines, the right-of-way table.

## 6. What a driver decides

The four kinds of answer the rule set already gives (a place to stop by, a top speed, an intention, an action) are
enough; what is new is facts and rules.

- **Something in the way**: follow it like a car ahead (brake, stop short). Then, by rules: **swerve** if there is
  room beside it in the lane or the next lane is clear (near: steering round it; middle: a lane change); **sound the
  horn** (an action; a person standing in front of a car gets honked at); after waiting long enough with no way past,
  **find another way** from where the car stands (the router, avoiding that stretch).
- **At a junction**: judge, from what is seen, whether the gap before each car that has right of way over me (the
  table says who) is big enough to go; otherwise wait at the line, or inside at a waiting point. Where nobody has right
  of way over the other (two cars arriving at a four-way stop), go by who arrived first, as people do; a tie is broken
  by a seeded hesitation and the driver's personality (an assertive driver goes, a timid one waits). Don't enter a
  junction with no room beyond it for me, judged by me, now.
- **Lane changes**, one mechanism for five reasons (the route needs a lane, the lane ends, pulling out of a kerb
  space, pulling into one, the lane beside is faster): want it (an intention, more urgent as the last chance nears);
  **indicate**; look for a gap whose leader I could follow and whose follower could stop behind me; if none, change
  speed to come level with one (this is what makes a joining car slow down or speed up along a ramp instead of
  stopping at its end); move across over a few seconds; abort in the first half if the gap closes; miss the turn and
  re-route if the last chance passes.
- **Letting in** is the other driver's own choice: a driver who sees a car indicating into the gap ahead of it may
  ease off, by its personality. **Zipper** is one habit both lanes share: having let one car in, don't let the next
  in until that one is across. No central claim; two cars wanting one gap is resolved by each seeing the other start
  to move, and the second backing off, as people do. When they don't, they touch (section 8).
- **Numbers from sources**: gap acceptance and the follower's acceptable braking from MOBIL (Kesting, Treiber &
  Helbing 2007); urgency toward the last chance from SUMO's LC2013 (Erdmann 2015); critical gaps at give-way junctions
  from the traffic engineering literature. Read from the papers when built, each with a CREDITS line; none are guessed
  here.

**What can go wrong now that nothing arbitrates**: two drivers can both wait (a stand-off) or both go (a near miss or
a crash). Stand-offs are broken by habits (a long wait makes a driver bolder, seeded so it is not both at once);
crashes are counted. Both are measured, and a bad count is fixed in the habits.

## 7. How a decision is carried out

- **Near**: a controller turns "this speed, along this path, this far to the side" into steering, throttle and brake
  for the Jolt car. It exists: `computeDriverInput` and the pursuit controller, tested on a real Jolt car following a
  lane (`agent_driver_tracks_a_lane_on_real_physics_and_stops_at_its_end`). The car is where physics says; the driver
  reads that and decides again. A skid, a shove, a swerve are facts the driver deals with, never errors to correct.
  Holding a lane through corners and junctions on physics needs its own test bed (the old driving's roll and fishtail
  history, ADR-0087); `wheel_lab` and `movement_lab` are the starting points.
- **Middle**: the car moves along its lane at the decided speed, with a sideways offset for lane changes and swerves.

## 8. Contact, crashes, wrecks

- **Near**: physics. Cars and anything else collide for real; `VehicleSystem`'s damage dents, scuffs, breaks lamps and
  throws sparks. Nothing to build but the near tier itself. The player struck by a car ragdolls (#294).
- **Middle**: the world finds contact between cars from their positions and outlines, every tick. Each contact is
  counted (who, where, why: what each driver was doing). Then a seeded roll with a rule-set chance: a crash, or they
  go on. A crash stops both where they are as a wreck, and the damage rules (pure functions in `vehicle_damage.h`) are
  applied at the contact point and closing speed, so a wreck the player arrives at later is dented where it was hit.
- **Far**: no cars, no contact.
- **A wreck is an obstacle** (section 5): others follow it to a stop, swerve or change lanes round it, or find another
  way. It stays for a rule-set time, then is cleared: the people go home and take the day off, their cars are put back
  near home, and each clearance is counted (driving-model-design.md 11). A tow truck later.
- **The people**: the brain gets an event "crashed"; "in a crash" is a state row in `brain.lua`.
- **A crash between two drivers both following the rules is still a bug to look at.** The contact count says where.

## 9. Merges

- The road model already states each merge: the host lane, where the joining lane starts running beside it, for how
  long, and the taper. The driving asks the road model one new question, **which lane runs beside mine here, and may I
  cross into it**, answered from a section's neighbours and from stated merges. A merge is then a lane that ends: a
  lane change with a hard last chance. No lanes builder change, no re-bake.
- Merges leave the crossing-point machinery, and with them fault 2. Fault 1 is fixed wherever its comparison ends up,
  with `approxEqual` and its ordered companions from `rt_math.h` and a named tolerance.
- The two known-fault tests (`merge_known_fault_a`, `_b`) are rewritten to check the right outcome: the joining car
  takes a gap, or a counted contact; never an unnoticed overlap.

## 10. What happens to today's junction machinery

| Piece | Becomes |
|---|---|
| Right-of-way table (`right_of_way.h`) | kept: the law as drivers know it |
| Announcements (`announce.h`) | kept as each driver's estimate of when another car will reach a crossing, from what can be seen; flags only the other driver could know (committed, arrival fixed) go |
| The resolver (`conflict.h`) and its group ordering | removed: each driver judges |
| Room grants (`room.h`) | removed as grants; "is there room beyond for me" is the driver's own judgement |
| Waiting points inside the junction (`junction_plan.h`) | kept: where a car may wait without blocking a crossing is road knowledge |
| Wait-for graph (`wait_graph.h`) | kept, as a measurement |
| The lab's pinned run (`0x7aade29530b4fdd1`) | will move; re-pinned once, with old and new numbers and why |

Removing the resolver is the largest change, so it comes after drivers can perceive and change lanes, and is done
with the lab's numbers watched before and after.

## 11. What changes in the other documents

- `driving-model-design.md`: section 9 (junctions) loses central ordering and room grants; section 10 (lane changes)
  is replaced by section 6 here; section 11 (others on the road) is replaced by sections 5 and 8; section 12 (levels of
  detail) by section 4.
- `agent-brain-design.md`: one event (`crashed`), one state row; the car as an entity near and middle, a record far.
- The road model: one question, "the lane beside, and may I cross".
- `AGENTS.md` or an ADR: cars are ECS entities when near or middle; one owner of a car's motion at any moment.

## 12. Sittings, hardest first

Each is one or two items, reviewed before the next, reporting the same numbers each time (the lab's hash and limits,
contacts, long stands, stand-offs, the brain lab).

1. **Cars as entities, middle tier only**: the brain lab's cars become entities with the lane component, drawn and
   moved through the ECS, the sim's rows gone for them. With #295 first if the numbers need it. The hardest because it
   moves the city's cars between worlds; nothing else changes behaviour.
2. **The near tier**: a `Vehicle` component swapped in and out by distance, the controller on Jolt, hand-back rules,
   removing a Jolt car (#293's engine part). On brain_lab: drive up to a resident's car and take it; push one with
   your own car.
3. **Perceiving the way ahead and reacting**: the one "what is ahead" question (physics near, lane knowledge middle),
   brake, horn, swerve, find another way. The player in the road is the first test.
4. **Lane changes and merges**: the road model's "lane beside", the five reasons, zipper as a habit, real pull-out
   and parking.
5. **Junctions by judgement**: the resolver and room grants removed, gap judgement and tie habits in their place;
   #286's limits re-measured.
6. **Contact and crashes in the middle tier**: the contact count, the roll, wrecks, the brain's crash state, clearance.

Optional lane changes (the faster lane) and the far tier's clock queue and GPU crowd come after.

## 13. Decisions

Settled (Glenn, 2026-11-16):
- **The crash chance in the middle tier**: very low. Most unseen contacts go on; a rule-set number, starting at 1 in
  50. Every contact is still counted, crash or not.
- **How long a wreck stays**: 10 game minutes to start (rule-set number).
- **The order**: cars as ECS entities first (sitting 1).
- **The pinned lab run**: needs no decision; it follows the standing rule (never re-pinned silently: old, new, why).

- **No timeouts, no deadlock machinery in the game** (Glenn: cars "need to wait as long as they have to ... all
  systems have to work in tandem without having machinery sitting around to detect deadlocks"):
  - Junction 0 stays as built; a car waits through its red however long it is.
  - The lab test's "nobody stands over 100 s" limit (`kLongestStand`, `tests/test_traffic_lab_v2.cpp`) is removed. The
    test fails instead on a real deadlock: after the run, the wait-for graph finds a ring of cars each waiting on the
    next where nothing they wait for can ever change. A queue at a red light is not one.
  - The measurements (`driving::Measurements`, the wait-for graph among them) attach only in tests and lab tools, not
    in normal play (today `city_driving.cpp:281` attaches them for every level on the new driving).
  - The resolver's ring-breaking order goes with the resolver (section 10).
  - Kept, as a driver's own habit in data: a driver who has waited long enough may look for another way
    (`room_refused_another_way` today). It is a person's choice, not the system unsticking a car.
