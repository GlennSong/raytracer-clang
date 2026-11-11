# D4 — First in goes first; waiting inside for a far turn

**Read first:** design 9.4, 9.6. Brief D3.

## The goal

Once cars are inside a junction, they sort themselves out in an order that can't form a loop. A driver making a far
turn on a plain green may pull forward into the junction and wait there for a gap, as American drivers do.

The design says "entry time gives a single order". That is not quite enough, and this brief corrects it.

## The correction

A driver waiting inside the junction for a gap entered *before* the oncoming cars it is waiting for. By "first in
goes first" they would have to wait for it. So the order is not by when a car crossed the line. It is by when a car
passed its **last waiting point**: the last place on its path where it may stop and wait. That is the stop line for
most turns, and the wait-inside point (`waitInsideAt`, design 5.2) for a far turn that has one.

## Decisions already made

Three separate things, checked in this order:

1. **A body on a crossing point blocks it for everyone.** If any part of any car is on a stretch, no other car
   enters that stretch, whoever has right of way. Treated as a stopped car at the start of my stretch.
2. **Cars past their last waiting point are committed.** At each crossing point, committed cars go in the order they
   committed (ties: lower id). A later car, even one with right of way, treats an earlier committed car's crossing
   point as a car to follow until its tail is clear.
3. **Cars not yet committed** decide by D3.

**Waiting inside:**

- Only where the connector has a wait-inside point, and only on a green that makes the turn give way to oncoming
  traffic. Never on a turn's own green arrow (it just goes) and never on a red.
- The wait-inside point must lie before the first crossing point with anything the turn gives way to or ties with.
  That is a check on the road data (add it to design 5.3's checks); if it fails, that connector has no wait-inside
  point.
- As many cars may wait inside as fit between the line and the wait-inside point, with gaps. Usually one.
- A car needs room beyond granted (D5) before it may enter to wait.
- At the end of the green, oncoming cars stop and announce "not going"; the waiting car's check in D3 then passes
  and it completes the turn during the yellow and the all-red. Cars on the next green see its body on their crossing
  points (rule 1) and wait.

## Why no loop

- Rule 1: a car on a crossing point is moving off it unless something stops it. What can stop it is rule 2 (an
  earlier committed car, an order with no loops) or no room ahead, which D5 prevents for anything inside the box.
- Rule 2: commit times are a single order.
- A car waiting inside sits on crossing points of the *cross street's* turns, which have a red, and not on any
  crossing point of a turn that can move in this phase (the check above). So it blocks nobody who could go.

Write this argument into the ADR, with the wait-inside check, because the argument fails without it.

## Traps

- **Two opposing far turns.** If their paths cross, both wait-inside points must be before that crossing, and the
  pair is settled by D1 like any other. If their paths don't cross (each turns in front of the other), they don't
  affect each other. Which it is comes from B2's shapes; test both.
- **Long vehicles.** A bus waiting inside may leave its tail on the crosswalk or on a crossing point behind it. The
  count of how many fit uses real lengths; a vehicle that doesn't fit waits at the line.
- **The light changes as a car reaches its last waiting point.** Committed means past it. A car not past it obeys
  the new light.
- **A committed car must still not hit anything:** people (announcers) and bodies (rule 1) apply to it too.

## Done when

Staged tests:

1. A far-turning car enters, waits inside while three oncoming cars pass, and goes in the gap after.
2. With no gap all green, it completes on the yellow; the cross street's first car waits until it is clear.
3. Two cars committed to the same crossing point pass in commit order, the second following the first.
4. A car with right of way arriving while another's body is on its crossing point slows and does not hit it.
5. Opposing far turns, both shapes.
6. Ten minutes of heavy random traffic at a four-way with lights: no overlaps; no loop in the wait-for graph; every
   stop inside the junction has a recorded reason; no car inside the junction at the start of two greens in a row.

## Show Glenn

The lab's downtown grid: far turns pulling forward, waiting and clearing on yellow. The measurements row "stopping
in the middle of a junction" reads only "giving way" reasons.
