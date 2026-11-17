# Two drivers meeting: lane changes, joining by zipper, the junction layer, crashes

Status: DRAFT for Glenn, 2026-11-16. Nothing here is built. It answers issue #285 and the junction half of #286, and
is the first item of `docs/agent-driving-handoff.md`'s "what is next". It sits on top of `docs/driving-model-design.md`
(sections 7 to 12) and `docs/agent-brain-design.md`; where it changes either, it says so.

## Words

- **Car ahead / car behind**: on one lane, the nearest car in front of my front, and the nearest car behind my tail.
- **Following**: keeping a safe distance from the car ahead (the formulas in `src/apps/driving/act.h`: easing toward a
  speed, plus a hard "could I still stop" limit).
- **Crossing point**: a stretch where two paths' cars could touch: two turns crossing in a junction, two paths joining
  into one lane (ADR-0400).
- **The resolver**: the one piece of code that says who goes first at a crossing point (`conflict.h`), asked once a
  tick for every pair of cars announced there.
- **Gap**: the space on a lane between two cars, the one that will be my car ahead (the **leader**) and the one that
  will be my car behind (the **follower**). Either may be missing.
- **Claim**: asking, in the plan step of the tick, for something two cars might both want; the settle step grants
  claims one at a time in a fixed order (design 7.1). Room beyond a junction is already a claim (`room.h`).
- **Merge**: where the road model says one lane runs into its neighbour and ends: a ramp's lane into the freeway's, a
  lane that drops (`ConnectionKind::Merge`, `RoadModel::mergingAt`).
- **Zipper**: at a merge, the cars of the two lanes take turns, one from each, each joining car having slowed or sped
  up along its lane to arrive beside its gap.
- **Contact**: two cars' outlines overlap. Whatever happens next, a contact between two drivers who follow the rules
  is a bug in the rules and is counted.
- **Near / out of sight / far**: the three distances of the brain design. Near: a full agent the player can see.
  Out of sight: still driven by the driving model, nobody looking. Far: no car exists; the person is "leg N, due at T".

## 1. What Glenn asked for

- "The vehicles shouldn't drive through each other. They should collide and there should be a wreck. Really the car
  that's coming onto the freeway should slow down and properly zipper."
- "If I'm close to those vehicles I should see them crash. They should have physics bodies. If it's distant ...
  it could be fudged with a dice roll ... Maybe very distant cars don't crash at all because they're not present yet."
- The roads now state what the junction layer used to guess, so the junction layer can be simpler (hand-over).

## 2. What there is today (checked against the tree)

- **Following** works on one path: a car sees the car ahead on its own path and the paths its way leads into.
- **Lane changes do not exist.** The router can plan them (`route.h`, 4 s a change), the lane graph knows which lines
  may be crossed (`Section`, `laneLines`), but the tick cannot move a car across, so every caller turns them off
  (`city_sim.cpp:282 settings.laneChanges = false`). Pulling out of and into a kerb space is a sideways slide.
- **Junctions**: cars announce when they will be on each crossing point (`announce.h`); the resolver orders each pair
  by seven questions; a car told to go second stops at its line or inside at a waiting point (`give_way.h`); nobody
  enters a junction it cannot leave (`room.h`); a wait-for graph tells gridlock from deadlock (`wait_graph.h`).
- **Merges are treated as crossing points**, like two turns meeting in a junction. Who gives way there
  (`stated_merge`: the lane that joined) is built and switched OFF in `rules/lab.lua:118`, because switching it on
  showed two faults (issue #285):
  1. `announce.cpp:257` and `:298` compare two numbers meant to be the same point with `<=`; 7e-15 m out, a car
     creeping to its give-way stop is judged unable to stop and put first.
  2. A merge is one stretch 43 to 52 m long. Once both fronts are on it, the give-way code skips it
     (`give_way.cpp:125`: "my front is on it: I go first") and following does not see a car on the other path. **For
     that moment nobody owns the pair**, and a bus and a car drove through each other.
  3. A joining car from rest needs about a 15 s gap, so ramps crawl.
  Both are pinned by tests that fail when fixed (`merge_known_fault_a`, `_b` in `tests/test_driving_give_way.cpp`).
- **The lab's limits (#286)**: queues at a green empty at 2.75 s a car (target 1.7 to 2.1); cars stand over 100 s at
  the off-ramp join on frontage road s311 (fault 2 above) and at junction 0, one seven-arm junction on a 115 s cycle.
- **Physics**: each drawn AI car has a kinematic box that follows it (`city_physics.h`, ADR-0060), so the player
  bumps into cars but cars never react. A car becomes a real physical vehicle only when the player takes it
  (`city_vehicles.h`, ADR-0062), and the engine cannot turn it back (#293). No car ever crashes into another.

## 3. The one rule: every pair of cars has exactly one owner, at every moment

Fault 2 is the shape of every "drove through each other" bug: two cars near each other, and each piece of code
assumed another piece was watching them. So the design starts from a rule that can be checked every tick:

> Any two cars whose bodies could touch within the look-ahead are related in exactly one of two ways:
> **one follows the other** (it is the other's car behind, on a lane or in a claimed gap), or **one gives way to the
> other** at a crossing point (the resolver said so). Passing from one to the other is a hand-over that happens in
> one tick, never a gap where neither holds.

Three consequences:

- **A merge stops being a crossing point.** A joining car picks a gap and from then on follows its leader, and its
  follower follows it (section 4). Two cars side by side on a merge are always one ahead of the other. The resolver
  keeps only meetings that really are crossings: turns through a junction, two turns joining one lane at a junction,
  a car crossing a lane between junctions.
- **The check is cheap and runs always**: per tick, for each pair of cars near each other on lanes beside each other
  or on crossing paths, is there an owner? A pair with none is counted with the place, like the wait-for graph's "a
  car at rest with no reason". The lab's target is zero, and a test fails on any.
- **Contact is a second, separate count** (section 7). A pair with an owner that still touched means a rule's numbers
  are wrong (a gap too short, braking too weak). A pair with no owner that touched means a hole in the design.

## 4. Changing lanes: one mechanism for five things

One mechanism covers everything that moves a car from one lane to the one beside it:

| Why | How urgent |
|---|---|
| the route needs another lane before a turn | grows as the last chance nears (`distance_to_last_chance`, design 10) |
| the lane ends (a merge, a dropped lane) | must; the lane's end is the last chance |
| pulling out of a kerb space | must, at the driver's own pace (the space is not a lane yet: see below) |
| pulling into a kerb space | must, before the space |
| the lane beside is faster | would like to (later, near only) |

**The steps**, each a step of the existing tick (design 7.1):

1. **Want** (plan step, a rule): the rules give an intention "change to the kerb-side lane" or "the centre-line-side
   lane" with an urgency. This is the decide layer's existing intention answer; no new kind of answer.
2. **Choose a gap** (plan step): look at the target lane beside me, find the gaps whose leader I could follow and whose
   follower could follow me, by the same "could I still stop" test following already uses, for both. Choose the one
   that needs the least change of my speed. If none is safe now, **aim for one**: set my speed to arrive beside a gap
   (this is what makes a joining car slow down or speed up along a ramp, rather than stopping at the taper).
3. **Claim it** (plan step → settle step): "I will go between leader L and follower F on lane X". Claims on one lane
   are granted in a fixed order (urgency, then position, then id), each grant counted before the next, so two cars
   from either side never take one gap. This is the same machinery as room claims.
4. **Granted**: in that tick, the follower's car ahead becomes me, and my car ahead becomes L, **while I am still in my
   own lane**. That is the hand-over of section 3: from now the pair me/F and me/L are owned by following.
5. **Move across** over about 3 to 4 seconds (design 10's number, to be checked against a source): my sideways offset
   `d` goes from my lane to the target; I am listed on both lanes until across, so cars behind me in either lane see me.
6. **Abort** in the first half if the gap closes (my leader brakes hard): `d` eases back, the claim is given up.
7. **Fail**: a car that never got across before its last chance carries on and is routed again (design 10). The one
   exception is a lane that physically ends: its end is a place to stop by, as today.

**Letting in.** The follower F does not do anything special: following its new car ahead (me) already makes it ease
off. How hard it may brake to let me in is the gap test's limit in step 2 (in the published model this is a "safe
braking" number, MOBIL, Kesting, Treiber & Helbing 2007; SUMO's LC2013, Erdmann 2015, is the reference for urgency
rising toward the last chance). The numbers go in the rule set and get CREDITS lines when they are read from the papers;
none are guessed here.

**Zipper.** With the above, one courtesy rule turns joining into taking turns: a driver who has just let one car in
ahead of it does not open a gap for another until that car is fully across (a rule over a new fact, "let in, not yet
across"). So the next joining car's best gap is behind me. That is a rule in the rule set, not C++.

**Pulling out and parking.** A kerb space is not a lane. To pull out, the car claims a gap on the kerb lane beside the
space, with itself standing still at the space's end; to park, it claims nothing (it leaves the lane) but slows in its
lane as any stop does. This replaces today's sideways slide, and is why lane changes come first.

## 5. Merges, after this

- A merge the road model states (`ConnectionKind::Merge`: a ramp's auxiliary lane beside its host for `length`, then
  a taper) becomes, for the driving: "this lane ends here; its neighbour is the lane to change into; the line between
  them may be crossed from here". The road model already states all three numbers.
- **How the lane graph holds a merge today** (checked: `lane_level.cpp`, the loop over stated connections near line
  570): the merging lane is a lane of its own road, and its last path is joined END TO END into the host lane's path
  where their lines meet. It is not the host's neighbour in a section, so a lane change "into the lane beside" cannot
  see the host from it. Two ways to fix that:
  - **(a) Ask the road model** (recommended): the model already states, for each merge, the host lane, where the
    joining lane starts running beside it (`hostStation`), for how long (`length`) and the taper. The driving asks
    "which lane runs beside mine here, and may I cross into it" of the model, and the lane change uses the answer.
    No lanes builder change, no re-bake. This is "ask the road model, never derive from geometry".
  - (b) Rebuild merges in the lane graph as neighbours in one section with a crossable line. Cleaner for the lane
    graph's own readers, but a builder change and a re-bake of every level, for no behaviour (a) cannot give.
  Not yet checked: whether any section of today's levels already holds a merging lane beside its host (a dropped
  lane on a street might). Sitting 1 counts them.
- The `stated_merge` question and the merge stretches leave the resolver. Fault 2 goes away with them. Fault 1 is a
  plain bug wherever it sits: the two comparisons use `approxEqual` and its ordered companions from `rt_math.h` with a
  named tolerance, as AGENTS.md says.
- The two known-fault tests change from pinning the wrong outcome to checking the right one: the bus holds back, or
  takes the gap behind the car; never an overlap.

## 6. The junction layer, simplified

The 2026-11-13 review called the junction layer patched and contradictory. Two things change what it has to do:

- **Merges are no longer its job** (section 5), nor are the "between junctions" meetings that are really two lanes
  joining. What is left is crossings: turns in a junction, joins at a junction, a lane crossing a connector.
- **The roads now state** through pairs, approaches facing each other, turn kinds, stop lines and merge marks. Every
  pair the resolver orders should be settled by stated data. Its last fall-backs ("the lower path id gives way", "the
  lower car id") should never answer, so they become counts the lab requires to be zero, not answers.

What I do not know yet, and will not guess: which of the resolver's seven questions and the give-way code's special
cases (the guards added by #284, the split rule, the honest clear times) are still needed once merges leave. So the
junction sitting starts with a measurement, not a rewrite: switch merges over, run the pinned lab, and count, for each
question and each special case, how many pairs it decided. A question that decides nothing in the lab is a candidate to
delete, shown by deleting it and seeing the lab and the tests unchanged. Then #286's limits:

- **Long stands at the s311 join**: expected gone with merges (section 5). Checked by the limit, not assumed.
- **Long stands at junction 0**: a seven-arm junction on a 115 s cycle makes waits of 144 s at red by design. That is
  the road and its signal plan, not the driving. Glenn's call (section 10).
- **Queues emptying at 2.75 s a car, not 1.7 to 2.1**: cause not known. Candidates: how fast a car from rest speeds up
  (the vehicle class's numbers), each car waiting for the one ahead to move before it decides (its reaction time
  stacking along the queue), or the announcement of a queued car being held back "no earlier than the car ahead plus a
  following interval" (`announce.h`). Measured one at a time, in the lab, before any change.

## 7. Contact, and crashes by distance

**Contact is found by the driving, every tick, at every distance the driving runs**: for the pairs the owner check
already looks at (section 3), do the two outlines overlap? Each contact is counted with the pair, the place, and who
owned the pair. Then what the world does with it depends on how far it is from the player:

| | Near (within the physics radius) | Out of sight (driven, unseen) | Far (no car) |
|---|---|---|---|
| What happens | Both cars become physical vehicles at their pose and speed, and the physics plays the crash | A dice roll: a crash, or they go on | Nothing: no car exists |
| Afterwards | A wreck: the cars stay where physics left them | A crash: both stop where they are, as a wreck; the people's state is "in a crash" | — |
| If the player arrives later | — | The wreck is there, placed as the driving left it | — |

- **The dice are seeded** by the two people and the tick, never a shared generator, so a run repeats (design 7.1).
  The chance is a rule-set number.
- **A wreck is a path obstacle** (design 11): cars behind follow it to a stop and, once lane changes exist, change
  lanes round it. It stays for a rule-set time, then is cleared: the people go home and take the day off, their cars
  are put back near home, and each clearance is counted (design 11, Glenn's answer of 2026-11-11). A tow truck later.
- **The people**: the brain gets an event "crashed"; "in a crash" is a state in `brain.lua` (the brain design already
  names crash as an agent state). From there it is data: wait at the wreck, then go home.
- **The physics hand-over near the player** reuses the promotion that already turns a sim car into a physical vehicle
  when the player takes it (ADR-0062), started by a contact instead of a key. Clearing a physical wreck needs the engine
  to remove a physical car, the same missing piece as #293; so #293's engine part comes forward into this work, and the
  player's "give the car back" can follow cheaply.
- **Which radius is "near"**: the one inside which cars already have kinematic boxes (`possessRadius_` 90 m in
  `city_physics.h`)? To confirm that the new driving's cars get those boxes at all; the hand-over copies them into the
  sim's vehicles, which suggests so, but nobody has checked.

## 8. What changes elsewhere

- `driving-model-design.md` section 10 (lane changes) is replaced by section 4 here; section 9 loses merges; section
  11's wreck row is kept and given the distance table above.
- `agent-brain-design.md`: one new event (`crashed`) and one state row in `brain.lua`; no new machinery.
- The road model gains one question: which lane runs beside this one here (from its sections, and from its stated
  merges), and whether its line may be crossed. One query, used by every lane change, so nothing else derives it.
- Kinds stay data: the five reasons to change lanes, the courtesy rule, the crash chance and the wreck time are rule-set
  rows and numbers; no C++ branch names a reason, a vehicle kind or a road kind.

## 9. Sittings, hardest first

Each is one or two items, ends with the same numbers reported (the pinned lab's hash and its failing checks, the
owner and contact counts, the brain lab), and is reviewed before the next starts.

1. **Lane changes for the "must" cases**, on a test scene and then traffic_lab: want, gap, claim, hand-over, move,
   abort, fail. With it the owner check (section 3), so the hand-over is measured from the first day. Largest; may need
   two sittings. Includes the road model's "lane beside" question (section 5 (a)), so merges can use it in sitting 2.
2. **Merges as lane changes, and the zipper**: section 5, the courtesy rule, fault 1 with `approxEqual`, the two
   known-fault tests turned round. Done when the s311 join has no car standing over 100 s and the lab's join overlap
   count is zero.
3. **Two brain cars meeting**: on brain_lab, households sent so two of their cars meet at a junction and at a lane
   change (it has never been seen: today only one brain car is on the road at a time). Real pull-out and parking
   replace the sideways slide.
4. **The junction layer measured and cut** (section 6), then #286's queue-emptying limit.
5. **Contact and crashes out of sight**: the contact count, seeded dice, wrecks as obstacles, the brain's crash state,
   clearance.
6. **Crashes near the player**: the physics hand-over, the engine's missing "remove a physical car" (#293's engine
   part), a wreck the player can walk up to. Pictures from above as well as the chase camera.

Optional lane changes (the faster lane) and the middle distance's cheaper rules come after, with the rest of the
"middle and far" work.

## 10. Decisions for Glenn

1. **Junction 0** (seven arms, 115 s cycle, waits up to 144 s): change the lab's road or signal plan so it can pass,
   or accept it and drop that junction from the 100 s limit? Recommended: change the signal plan, since a real
   seven-arm junction would be split or given a shorter cycle; it is a lab level, made for this.
2. **The crash chance out of sight**: a starting number (for example, one contact in two is a crash), or always a crash?
   Recommended: always a crash to start, so every contact is visible in the counts and a wreck; lower it only if wrecks
   pile up because of a rules fault we cannot fix soon.
3. **How long a wreck stays** before it is cleared. Recommended: 10 minutes of game time to start.
4. **Merges in the lane graph**: (a) the driving asks the road model which lane runs beside (recommended, no re-bake)
   or (b) rebuild merges as neighbours in the lane graph (section 5)?
5. **The order**: lane changes first (my recommendation, because merges, parking and wrecks all depend on them) or
   crashes first (the most visible)?
