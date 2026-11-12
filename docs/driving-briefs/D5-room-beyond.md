# D5 — Don't block the box, as a claim

**As built (ADR-0407).** Room is asked for in the plan step and granted in the settle step (`src/apps/driving/room.h`),
remembered on each driver's junction plan; a refused car stops at its line by the rule set's "no room beyond" rule and
announces that it is not going. Each decision below was followed; where the brief turned out short or wrong:
- *How much room*: "the last car brakes" alone would let a moving last car "rest" past a stopped car ahead of it, so each
  car on the stretch is held behind the one ahead. Room also counts, besides cars in transit and grants, every car FIRST IN
  THE QUEUE for it (refused earlier, still at its line): asking order only among one tick's asks let a car at a red lose a
  one-car link to other approaches every cycle (145 s).
- *When I ask*: also when, at rest at the line, a driver is about to move off without that counting as a decision
  (released by a red: D3's gate). It rolled a few centimetres over the line before its refusal came, and a car past its line
  is committed; so a refused car that rolled no more than at_line_within over its line stops where it is.
- *Granting*: "by right of way (D1)" is the conflict resolver's settled order (`ConflictResolver::order`, a new read of it),
  not the D1 table. A car whose car ahead in its lane holds no grant is refused, or a car behind could hold room the one in
  front can never get.
- *Once granted*: "holds until the car is on the lane beyond" -- or until it gives it back by holding a stop at or before its
  line for something else (giving way at the line, a red), so a grant is never held through a red.
- *Junctions close together*: room alone is not enough at lights. A chain is granted only where each light passed through
  will let the car in (green or yellow) as it gets there, and the car is then committed through those junctions (a yellow
  there does not stop it). This replaced #261's "stop at the line before" stop-gap. Two lights that almost never line up for
  a turn pair (the 216 s case) leave cars waiting at the first line: such turns are closed to the classes they are too
  short for at load (`room_cycles_through`) and reported, so routes go round. The driver's look-ahead is lengthened so a
  plan exists for each junction a chain passes (a car at rest saw no further than its own junction).
- *Waiting inside needs a grant too*: built (`room_to_wait_inside`), and never through a chain.
- *Done when 4* (1.9 s a car): room beyond changes queue discharge not at all, but the following model empties a queue at
  2.75 s a car on the hand-made roads; that is task C6's to meet. *Done when 6*: no loop in the lab; one car stood 123 s, a
  permitted far turn starving at a five-phase light (ADR-0406's owed arrows), not room beyond.
- Staged tests: `tests/test_driving_give_way.cpp` room_26 to room_34; the lab's `box_stops` row is the "done when".

**Read first:** design 9.5, 9.6, 7.1 (the settle step). Brief D4.

## The goal

No car enters a junction unless it will be able to leave it. Two cars from different approaches can both see the
same one car's worth of room beyond, so seeing is not enough: room is asked for and granted, one car at a time.

## Decisions already made

**How much room there is,** on the lane a connector joins:

- Find where the back of the queue *would come to rest*: take the last car on that lane and suppose it brakes
  comfortably to a stop from its present speed. Its tail's resting place is the far end of the room.
- If there is no car before the next place I might have to stop (the next stop line, the lane's end, a hold), the
  far end is that place.
- The near end is where the box ends (past the far crosswalk).
- Room = far end − near end − the lengths and gaps of every car already granted room here and not yet on the lane.

Supposing the last car brakes is the important choice. Measuring to where it is *now* would refuse entry behind any
moving car and choke every junction. Measuring as if it will keep moving would let cars in behind a queue that is
about to stop.

**How much I need:** my length (with trailer) plus a standstill gap.

**When I ask:** at the moment I would otherwise become committed (D4's last waiting point, or the point where I can
no longer stop comfortably before the line, whichever is first). Not before, and not at the line.

**Granting** happens in the settle step, per junction, in a fixed order: cars that asked earlier first; among those
asking in the same tick, by right of way (D1), then by id. Each grant reduces the room before the next is looked at.

**Once granted,** it is a remembered fact on the car and holds until the car is on the lane beyond. A lane change
into that stretch by another car is not granted (E1 reads the same count).

**Not granted:** the car stops at the line and announces "not going" (D3), so cross traffic uses the time.

**Junctions close together.** If the room up to the next stop line is shorter than I need, I must also be granted
room beyond the *next* junction, and so on, up to three junctions. If three is not enough, refuse, and report the
place as a road-data problem: the road has a stretch no vehicle of this length can ever stop in.

**Waiting inside** (D4) needs a grant too, held for as long as the car waits. This is cautious: it can hold a space
for most of a green. It is a named setting in the rule set so it can be relaxed after measuring.

## Traps

- **The crosswalk on the far side is inside the box.** A car must not come to rest on it. The near end of the room
  is past it.
- **Counting cars in transit:** cars on connectors heading for this lane count against the room whether or not they
  hold a grant (a car placed there by a level-of-detail change, say). Derive the count from where cars are, each
  tick; use the remembered grant only to stop them being refused twice.
- **A queue that starts moving** while I'm stopped at the line: room appears as its tail's resting place moves on.
  I ask again each time I decide. There is no timer.
- **Throughput.** If this is wrong in the cautious direction, greens empty slowly. Test 4 below catches it.
- **This is where gridlock shows.** Waiting for room at the line is the one kind of waiting that can loop between
  junctions (design 9.6). Every such wait must name the lane it is waiting for room on, so the wait-for graph can
  tell gridlock from deadlock.

## Done when

1. Two approaches, room for one: exactly one car enters; the other waits at the line; the same one every run.
2. A lane beyond that is full and stopped for ten minutes: no car is in the box at any tick, and cross traffic
   keeps moving.
3. Two junctions closer together than a bus is long: the bus doesn't enter the first until it has room beyond the
   second.
4. A green with a moving, uncrowded lane beyond: after the first few, cars leave the queue about one every 1.9
   seconds (plan, measurements). Not slower than 2.3.
5. No car at rest on a crosswalk.
6. The lab's downtown grid at its normal number of drivers, ten minutes: the wait-for graph has no deadlock loop;
   gridlock loops are counted and reported (target: none, Glenn 2026-11-11).

## Show Glenn

Task D6 is the showing: the downtown grid at rush, boxes clear, queues holding at their lines.
