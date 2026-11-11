# D5 — Don't block the box, as a claim

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
